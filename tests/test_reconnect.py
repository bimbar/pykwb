"""Async connection lifecycle and transport reconnect coverage."""
import asyncio
import time
import unittest
from unittest.mock import AsyncMock, Mock, patch

from pykwb.messages import FrameType, Message, parse_message
from pykwb.kwb import KWBEasyfire, PROP_MODE_TCP
from test_temperatures import frame


def reader_with_config(**settings):
    reader = KWBEasyfire(PROP_MODE_TCP, _config={'connection': {
        'retry_initial': 0.005, 'retry_max': 0.01,
        **settings}})
    reader.load_sensors()
    return reader


def stream_pair():
    writer = Mock()
    writer.wait_closed = AsyncMock()
    return asyncio.StreamReader(), writer


class ReconnectTests(unittest.IsolatedAsyncioTestCase):
    async def test_initial_failure_is_retryable_only_when_enabled(self):
        for enabled in (False, True):
            with self.subTest(enabled=enabled), patch(
                    'pykwb.inputs.asyncio.open_connection', new_callable=AsyncMock,
                    side_effect=ConnectionRefusedError()) as connect:
                reader = reader_with_config(reconnect=enabled)
                connect.assert_not_called()
                if enabled:
                    await reader.listen_for(0.05)
                    self.assertGreaterEqual(connect.await_count, 2)
                else:
                    with self.assertRaises(ConnectionRefusedError):
                        await reader.listen_forever()
                    connect.assert_awaited_once()
                await reader.close()

    async def test_backoff_caps_and_resets_on_connection_success(self):
        reader = reader_with_config(retry_initial=1, retry_max=4)
        self.assertEqual([reader._input._next_retry_delay() for _ in range(4)], [1, 2, 4, 4])
        stream, writer = stream_pair()
        with patch('pykwb.inputs.asyncio.open_connection', new_callable=AsyncMock,
                   return_value=(stream, writer)):
            await reader._input.open()
        self.assertEqual(reader._input._next_retry_delay(), 1)
        await reader.close()

    async def test_cancellation_interrupts_retry_delay(self):
        reader = reader_with_config(retry_initial=10, retry_max=10)
        with patch.object(reader._input, 'open', new_callable=AsyncMock,
                          side_effect=ConnectionRefusedError()) as connect:
            await asyncio.wait_for(reader.listen_for(0.02), 1)
            connect.assert_awaited_once()

    async def test_disconnect_closes_stream_and_keeps_sensor_state(self):
        reader = reader_with_config()
        reader._input._reader, reader._input._writer = stream_pair()
        writer = reader._input._writer
        message = Message(32, 1, bytes(73), FrameType.SENSE)
        reader._update_sensors(parse_message(reader._sensors, message))
        before = [(s.value, s.available) for s in reader.get_sensors()]
        await reader._input._connection_lost(ConnectionResetError())
        self.assertIsNone(reader._input._reader)
        self.assertEqual([(s.value, s.available) for s in reader.get_sensors()], before)
        writer.close.assert_called_once()
        writer.wait_closed.assert_awaited_once()

    async def test_idle_and_invalid_input_never_reconnect_or_clear_sensors(self):
        # A legacy stale_timeout setting must not restore packet-health reconnects.
        for garbage in (b'', bytes(2000)):
            reader = reader_with_config(stale_timeout=0.01)
            reader._input._reader, reader._input._writer = stream_pair()
            stream = reader._input._reader
            stream.feed_data(garbage)
            message = Message(32, 1, bytes(73), FrameType.SENSE)
            reader._update_sensors(parse_message(reader._sensors, message))
            values = [(s.value, s.available) for s in reader.get_sensors()]
            before = time.monotonic()
            with patch.object(reader._input, 'open', new_callable=AsyncMock) as connect:
                await reader.listen_for(0.04)
                connect.assert_not_awaited()
            self.assertLess(time.monotonic() - before, 1)
            self.assertIs(reader._input._reader, stream)
            self.assertEqual([(s.value, s.available) for s in reader.get_sensors()], values)
            payload = bytearray(32)
            payload[12:14] = b'\x02\xe5'
            stream.feed_data(frame(32, payload))
            # End the capture after the fresh packet without triggering a retry.
            reader._config['connection']['reconnect'] = False
            stream.feed_eof()
            with self.assertRaises(EOFError):
                await asyncio.wait_for(reader.listen_forever(), 1)
            self.assertEqual(next(s.value for s in reader.get_sensors() if s.key == 'boiler_temp'), 74.1)
            await reader.close()

    async def test_connect_timeout_and_cancellation(self):
        for cancel in (False, True):
            reader = reader_with_config(reconnect=False, connect_timeout=0.02)
            connecting, cleaned = asyncio.Event(), asyncio.Event()

            async def stall(*args):
                connecting.set()
                try:
                    await asyncio.Event().wait()
                finally:
                    cleaned.set()

            with self.subTest(cancel=cancel), patch(
                    'pykwb.inputs.asyncio.open_connection', side_effect=stall):
                task = asyncio.create_task(reader.listen_forever())
                await asyncio.wait_for(connecting.wait(), 1)
                if cancel:
                    task.cancel()
                with self.assertRaises(asyncio.CancelledError if cancel else asyncio.TimeoutError):
                    await task
                self.assertTrue(cleaned.is_set())
                self.assertIsNone(reader._input._reader)
                self.assertIsNone(reader._input._writer)

    async def test_listen_for_propagates_connection_timeout(self):
        reader = reader_with_config(reconnect=False, connect_timeout=0.01)

        async def stall(*args):
            await asyncio.Event().wait()

        with patch('pykwb.inputs.asyncio.open_connection', side_effect=stall):
            with self.assertRaises(asyncio.TimeoutError):
                await reader.listen_for(1)

    async def test_disabled_reconnect_propagates_eof(self):
        reader = reader_with_config(reconnect=False)
        reader._input._reader, reader._input._writer = stream_pair()
        reader._input._reader.feed_eof()
        with patch.object(reader._input, 'open', new_callable=AsyncMock) as connect:
            with self.assertRaises(EOFError):
                await reader.listen_forever()
            connect.assert_not_awaited()
        self.assertIsNone(reader._input._reader)

    async def test_default_reconnect_receives_fresh_packet(self):
        reader = reader_with_config()
        first, first_writer = stream_pair()
        second, second_writer = stream_pair()
        interrupted = frame(32, bytes(32))
        first.feed_data(interrupted[:8])
        first.feed_eof()
        payload = bytearray(32)
        payload[12:14] = b'\x02\xe5'
        second.feed_data(interrupted[8:] + frame(32, payload))
        with patch('pykwb.inputs.asyncio.open_connection', new_callable=AsyncMock,
                   side_effect=[(first, first_writer), (second, second_writer)]) as connect:
            await reader.listen_for(0.015)
            self.assertEqual(connect.await_count, 2)
            self.assertEqual(next(s.value for s in reader.get_sensors() if s.key == 'boiler_temp'), 74.1)
        first_writer.close.assert_called_once()
        await reader.close()
        second_writer.close.assert_called_once()
