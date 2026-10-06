"""Async listening uses the wire framing and sensors."""
import asyncio
import os
import socket
from pathlib import Path
import unittest
from unittest.mock import AsyncMock, Mock, patch

from pykwb.kwb import KWBEasyfire, PROP_MODE_FILE, PROP_MODE_TCP, PROP_MODE_SERIAL
from pykwb.messages import FrameType
from test_temperatures import frame


ROOT = Path(__file__).resolve().parents[1]


class AsyncListeningTests(unittest.IsolatedAsyncioTestCase):
    def tcp_reader(self):
        reader = KWBEasyfire(-1)
        reader._mode = PROP_MODE_TCP
        reader._reader = asyncio.StreamReader()
        self.addAsyncCleanup(reader.close)
        return reader, reader._reader

    def furnace(self, reader):
        return next(s.value for s in reader.get_sensors() if s.key == 'boiler_temp')

    def temperature_frame(self):
        payload = bytearray(32)
        payload[12:14] = b'\x02\xe5'
        return frame(32, payload)

    async def test_file_replay_updates_sensors_without_threads(self):
        reader = KWBEasyfire(PROP_MODE_FILE, _file_path=ROOT / 'tests' / 'data' / 'kwb_33_32.txt')
        self.addAsyncCleanup(reader.close)
        with patch('threading.Thread.start') as start:
            with self.assertRaises(EOFError):
                await reader.listen_forever()
        start.assert_not_called()
        self.assertIsNotNone(self.furnace(reader))
        self.assertIsNone(reader._file)

    async def test_idle_tcp_deadline_keeps_event_loop_responsive(self):
        reader, sender = self.tcp_reader()
        ticks = []

        async def heartbeat():
            await asyncio.sleep(0.005)
            ticks.append(True)

        before = asyncio.get_running_loop().time()
        await asyncio.gather(reader.listen_for(seconds=0.03), heartbeat())
        elapsed = asyncio.get_running_loop().time() - before
        self.assertTrue(ticks)
        self.assertGreaterEqual(elapsed, 0.02)
        self.assertLess(elapsed, 1)
        self.assertIs(reader._reader, sender)
        self.assertIsNone(self.furnace(reader))

    async def test_partial_escape_is_discarded_at_listen_for_deadline(self):
        reader, sender = self.tcp_reader()
        wire = self.temperature_frame()
        split = wire.index(b'\x02\x00') + 1
        sender.feed_data(wire[:split])
        await reader.listen_for(0.02)
        self.assertIsNone(self.furnace(reader))
        sender.feed_data(wire[split:])
        await reader.listen_for(0.02)
        self.assertIsNone(self.furnace(reader))
        sender.feed_data(wire)
        sender.feed_eof()
        with self.assertRaises(EOFError):
            await reader.listen_forever()
        self.assertEqual(self.furnace(reader), 74.1)

    async def test_cancelled_listener_discards_partial_packet(self):
        reader, sender = self.tcp_reader()
        wire = self.temperature_frame()
        sender.feed_data(wire[:8])
        task = asyncio.create_task(reader.listen_forever())
        await asyncio.sleep(0.01)
        task.cancel()
        with self.assertRaises(asyncio.CancelledError):
            await task
        self.assertIs(reader._reader, sender)
        sender.feed_data(wire[8:])
        await reader.listen_for(0.02)
        self.assertIsNone(self.furnace(reader))
        sender.feed_data(wire)
        sender.feed_eof()
        with self.assertRaises(EOFError):
            await reader.listen_for(0.2)
        self.assertEqual(self.furnace(reader), 74.1)

    async def test_eof_after_unknown_packet_keeps_sensor_values(self):
        reader, sender = self.tcp_reader()
        sender.feed_data(self.temperature_frame() + frame(87, bytes(24), frame_type=FrameType.CONTROL))
        sender.feed_eof()
        with self.assertRaises(EOFError):
            await reader.listen_forever()
        self.assertEqual(self.furnace(reader), 74.1)
        self.assertIsNone(reader._sensors[33][0].value)

    async def test_message_64_updates_extension_temperatures(self):
        reader, sender = self.tcp_reader()
        payload = bytearray(23)
        payload[19:21] = b'\x02\x5f'
        payload[21:23] = b'\xff\xc9'
        sender.feed_data(self.temperature_frame() + frame(64, payload))
        sender.feed_eof()
        with self.assertRaises(EOFError):
            await reader.listen_forever()
        values = {s.key: s.value for s in reader.get_sensors() if s.key}
        self.assertEqual(values['loop_4_out_temp'], 60.7)
        self.assertEqual(values['loop_3_out_temp'], -5.5)
        self.assertEqual(self.furnace(reader), 74.1)

    async def test_serial_stream_decodes_and_closes(self):
        stream = asyncio.StreamReader()
        writer = Mock()
        writer.wait_closed = AsyncMock()
        reader = KWBEasyfire(PROP_MODE_SERIAL, _serial_device='/dev/test')
        with patch('pykwb.kwb.serial_asyncio_fast.open_serial_connection',
                   new_callable=AsyncMock, return_value=(stream, writer)) as connect:
            connect.assert_not_called()
            stream.feed_data(self.temperature_frame())
            await reader.listen_for(0.02)
            self.assertEqual(self.furnace(reader), 74.1)
            connect.assert_awaited_once_with(url='/dev/test', baudrate=19200)
            await reader.close()
            await reader.close()
        writer.close.assert_called_once()
        writer.wait_closed.assert_awaited_once()

    async def test_real_tcp_stream_discards_partial_frame_and_closes(self):
        receiver, sender = socket.socketpair()
        receiver.setblocking(False)
        self.addCleanup(sender.close)
        self.addCleanup(receiver.close)
        reader = KWBEasyfire(PROP_MODE_TCP)
        self.addAsyncCleanup(reader.close)
        open_connection = asyncio.open_connection

        async def connect(*args):
            return await open_connection(sock=receiver)

        wire = self.temperature_frame()
        with patch('pykwb.kwb.asyncio.open_connection', side_effect=connect) as opened:
            sender.sendall(wire[:8])
            await reader.listen_for(0.02)
            self.assertIsNone(self.furnace(reader))
            sender.sendall(wire[8:])
            await reader.listen_for(0.02)
            self.assertIsNone(self.furnace(reader))
            sender.sendall(wire)
            sender.shutdown(socket.SHUT_WR)
            with self.assertRaises(EOFError):
                await asyncio.wait_for(reader.listen_forever(), 1)
            opened.assert_awaited_once()
        self.assertEqual(self.furnace(reader), 74.1)
        self.assertEqual(receiver.fileno(), -1)

    @unittest.skipUnless(hasattr(os, 'openpty'), 'Requires a POSIX pseudo-terminal')
    async def test_real_serial_transport(self):
        master, slave = os.openpty()
        self.addCleanup(os.close, master)
        self.addCleanup(os.close, slave)
        reader = KWBEasyfire(PROP_MODE_SERIAL, _serial_device=os.ttyname(slave))
        self.addAsyncCleanup(reader.close)
        await reader._open_connection()
        serial_port = reader._writer.transport.serial
        os.write(master, self.temperature_frame())
        await reader.listen_for(0.05)
        self.assertEqual(self.furnace(reader), 74.1)
        await reader.close()
        self.assertFalse(serial_port.is_open)

    async def test_deadline_under_continuous_ready_input(self):
        reader, sender = self.tcp_reader()
        sender.feed_data(bytes(100000))
        await reader.listen_for(0.02)
        self.assertIsNone(self.furnace(reader))


if __name__ == '__main__':
    unittest.main()
