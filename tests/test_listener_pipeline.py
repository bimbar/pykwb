"""The listener updates sensor state and logs completed packets."""
import asyncio
from contextlib import redirect_stdout
from io import StringIO
import unittest
from unittest.mock import AsyncMock, patch

from pykwb.kwb import KWBEasyfire, PROP_MODE_TCP
from pykwb.messages import FrameType, Message, parse_message
from test_temperatures import frame


class ListenerPipelineTests(unittest.IsolatedAsyncioTestCase):
    def reader(self):
        reader = KWBEasyfire(PROP_MODE_TCP)
        stream = asyncio.StreamReader()
        reader._reader = stream
        self.addAsyncCleanup(reader.close)
        return reader, stream

    async def test_listener_parses_updates_then_logs_and_propagates_eof(self):
        reader, stream = self.reader()
        first_payload = bytes(18) + b'\x00\xe6'
        second_payload = bytes(18) + b'\x05\x14'
        stream.feed_data(frame(80, first_payload) + frame(80, second_payload))
        stream.feed_eof()
        logged = []
        log_message = reader._log_message

        def log(message):
            self.assertEqual(reader._sensors[80][-1].value, message.values[-1])
            log_message(message)
            logged.append(message)

        with self.assertLogs('pykwb.kwb', level='INFO') as output, \
                patch.object(reader, '_log_message', side_effect=log):
            with self.assertRaises(EOFError):
                await reader.listen_forever()
        self.assertEqual(len(logged), 2)
        self.assertEqual(logged[0].values, (first_payload, 23))
        self.assertEqual(logged[1].values, (second_payload, None))
        self.assertTrue(any('Loop 4 Room Temp' in record.getMessage() for record in output.records))
        self.assertIsNone(reader._sensors[80][-1].value)
        self.assertFalse(reader._sensors[80][-1].available)

    async def test_unknown_ids_are_logged_without_sensor_values(self):
        reader, stream = self.reader()
        payload = b'\x00\x00\x00\x02\x5f'
        stream.feed_data(frame(87, payload, FrameType.CONTROL) + frame(250, b''))
        stream.feed_eof()
        output = StringIO()
        with redirect_stdout(output), patch.object(reader, '_log_message', wraps=reader._log_message) as log:
            with self.assertRaises(EOFError):
                await reader.listen_forever()
        messages = [call.args[0] for call in log.call_args_list]
        self.assertEqual(output.getvalue(), '')
        self.assertEqual(messages, [Message(87, 1, payload, FrameType.CONTROL),
                                    Message(250, 1, b'', FrameType.SENSE)])
        self.assertTrue(all(s.value is None for s in reader.get_sensors()))

    async def test_parsing_populates_message_without_updating_sensors(self):
        reader, _ = self.reader()
        message = Message(80, 1, bytes(18) + b'\x00\xe6', FrameType.SENSE)
        self.assertIs(parse_message(reader._sensors, message), message)
        self.assertEqual(message.values, (message.payload, 23))
        self.assertTrue(all(s.value is None for s in reader.get_sensors()))
        reader._update_sensors(message)
        self.assertEqual(reader._sensors[80][-1].value, 23)

    async def test_bounded_listener_keeps_connection_for_next_call(self):
        reader, stream = self.reader()
        stream.feed_data(frame(80, bytes(20)))
        await reader.listen_for(0.02)
        self.assertIs(reader._reader, stream)
        self.assertEqual(reader._sensors[80][-1].value, 0)
        stream.feed_data(frame(80, bytes(18) + b'\x00\xe6'))
        stream.feed_eof()
        with self.assertRaises(EOFError):
            await reader.listen_forever()
        self.assertEqual(reader._sensors[80][-1].value, 23)
        self.assertIsNone(reader._reader)

    async def test_listen_for_cancels_and_awaits_listener_on_timeout(self):
        reader, _stream = self.reader()
        stopped = asyncio.Event()

        async def listen():
            try:
                await asyncio.Event().wait()
            finally:
                stopped.set()

        with patch.object(reader, 'listen_forever', AsyncMock(side_effect=listen)) as listener:
            await reader.listen_for(0.02)
        listener.assert_awaited_once_with()
        self.assertTrue(stopped.is_set())

    async def test_reading_does_not_update_or_log_sensor_state(self):
        reader, stream = self.reader()
        stream.feed_data(frame(80, bytes(20)))
        output = StringIO()
        with redirect_stdout(output):
            packet = await reader._read_message()
        self.assertEqual(packet, Message(80, 1, bytes(20), FrameType.SENSE))
        self.assertEqual(output.getvalue(), '')
        self.assertTrue(all(s.value is None for s in reader.get_sensors()))

    async def test_read_message_raises_on_eof_including_partial_frame(self):
        for data in (b'', frame(80, bytes(20))[:8]):
            with self.subTest(data=data):
                reader, stream = self.reader()
                stream.feed_data(data)
                stream.feed_eof()
                with self.assertRaises(EOFError):
                    await reader._read_message()
                self.assertIsNone(reader._reader)

    async def test_read_message_propagates_transport_failure(self):
        reader, stream = self.reader()
        error = OSError('host unreachable')
        stream.set_exception(error)
        with self.assertRaises(OSError) as raised:
            await reader._read_message()
        self.assertIs(raised.exception, error)
        self.assertIsNone(reader._reader)
