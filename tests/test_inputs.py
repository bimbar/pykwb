"""Input contracts independent of framing and sensor decoding."""

import asyncio
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import AsyncMock, Mock, patch

from pykwb.inputs import FileInput, SerialInput, TCPInput
from pykwb.kwb import KWBEasyfire, PROP_MODE_FILE, PROP_MODE_SERIAL, PROP_MODE_TCP


class InputTests(unittest.IsolatedAsyncioTestCase):
    async def test_file_is_lazy_and_reads_decimal_bytes_until_eof(self):
        with TemporaryDirectory() as directory:
            path = Path(directory) / 'capture.txt'
            source = FileInput(path)
            # Construction does not open the as-yet nonexistent file.
            path.write_text('0\n2\n255\n')
            self.addAsyncCleanup(source.close)
            self.assertEqual([await source.read_byte() for _ in range(3)], [0, 2, 255])
            handle = source._file
            with self.assertRaises(EOFError):
                await source.read_byte()
            self.assertFalse(await source.recover(EOFError()))
            self.assertTrue(handle.closed)
            await source.close()

    async def test_file_rejects_invalid_capture_values(self):
        with TemporaryDirectory() as directory:
            path = Path(directory) / 'capture.txt'
            for value in ('-1', '256', 'garbage', ''):
                with self.subTest(value=value):
                    path.write_text(value + '\n')
                    source = FileInput(path)
                    try:
                        with self.assertRaises(ValueError):
                            await source.read_byte()
                    finally:
                        await source.close()

    async def test_ready_file_read_can_be_cancelled_without_closing(self):
        with TemporaryDirectory() as directory:
            path = Path(directory) / 'capture.txt'
            path.write_text('2\n' * 10000)
            source = FileInput(path)
            self.addAsyncCleanup(source.close)
            await source.read_byte()
            handle = source._file

            async def consume():
                while True:
                    await source.read_byte()

            task = asyncio.create_task(consume())
            await asyncio.sleep(0)
            task.cancel()
            with self.assertRaises(asyncio.CancelledError):
                await task
            self.assertFalse(handle.closed)
            self.assertEqual(await source.read_byte(), 2)

    async def test_serial_failure_never_retries_even_with_reconnect_setting(self):
        for error in (EOFError(), OSError('serial disconnected')):
            with self.subTest(error=error):
                kwb = KWBEasyfire(PROP_MODE_SERIAL, _config={
                    'connection': {'reconnect': True}})
                stream = asyncio.StreamReader()
                if isinstance(error, EOFError):
                    stream.feed_eof()
                else:
                    stream.set_exception(error)
                writer = Mock(wait_closed=AsyncMock())
                with patch('pykwb.inputs.serial_asyncio_fast.open_serial_connection',
                           AsyncMock(return_value=(stream, writer))) as connect:
                    with self.assertRaises(type(error)):
                        await kwb.listen_forever()
                    connect.assert_awaited_once()
                writer.close.assert_called_once()
                writer.wait_closed.assert_awaited_once()

    async def test_tcp_read_propagates_eof_before_any_reconnect(self):
        kwb = KWBEasyfire(PROP_MODE_TCP, _ip='heater', _port=4196,
                         _config={'connection': {'reconnect': True}})
        source = kwb._input
        self.addAsyncCleanup(source.close)
        stream = asyncio.StreamReader()
        stream.feed_data(b'\x02')
        stream.feed_eof()
        writer = Mock(wait_closed=AsyncMock())
        with patch('pykwb.inputs.asyncio.open_connection',
                   AsyncMock(return_value=(stream, writer))) as connect:
            self.assertEqual(await source.read_byte(), 2)
            with self.assertRaises(EOFError):
                await source.read_byte()
            connect.assert_awaited_once_with('heater', 4196)

    async def test_stream_close_tolerates_transport_error_and_is_idempotent(self):
        source = SerialInput('/dev/test', 19200, 5)
        writer = Mock(wait_closed=AsyncMock(side_effect=OSError('disconnected')))
        source._reader = asyncio.StreamReader()
        source._writer = writer
        await source.close()
        await source.close()
        writer.close.assert_called_once()
        writer.wait_closed.assert_awaited_once()
        self.assertIsNone(source._reader)
        self.assertIsNone(source._writer)

    def test_mode_selects_input_class(self):
        for mode, input_class in ((PROP_MODE_TCP, TCPInput),
                                  (PROP_MODE_SERIAL, SerialInput),
                                  (PROP_MODE_FILE, FileInput)):
            with self.subTest(mode=mode):
                self.assertIsInstance(KWBEasyfire(mode)._input, input_class)
        with self.assertRaisesRegex(ValueError, 'Unsupported input mode'):
            KWBEasyfire(-1)
