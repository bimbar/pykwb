"""Exact output captured before the listener pipeline refactor."""
from contextlib import redirect_stdout
import asyncio
from io import StringIO
import json
import logging
from pathlib import Path
import unittest
from unittest.mock import patch

from pykwb.kwb import PROP_MODE_TCP, KWBEasyfire, _print_summary
from pykwb.messages import FrameType, load_sensor_definitions
from test_temperatures import frame


FIXTURE = Path(__file__).parent / 'data' / 'listener_output.json'


async def capture_output(level, file_level=logging.DEBUG):
    rows = [row for row in load_sensor_definitions()
            if (row['message_id'], row['key']) in
            {('32', 'boiler_temp'), ('80', 'loop_4_room_temp')}]
    with patch('pykwb.kwb.load_sensor_definitions', return_value=rows):
        reader = KWBEasyfire(PROP_MODE_TCP, _config={'connection': {'reconnect': False}})
        reader.load_sensors()
    wire = (frame(80, bytes(18) + b'\x00\xe6')
            + frame(87, b'\x00\x00\x00\x02\x5f', FrameType.CONTROL)
            + frame(250, b'', FrameType.CONTROL))
    reader._input._reader = asyncio.StreamReader()
    reader._input._reader.feed_data(wire)
    reader._input._reader.feed_eof()

    terminal, logfile = StringIO(), StringIO()
    logger = logging.getLogger('pykwb.kwb')
    handler = logging.StreamHandler(logfile)
    handler.setFormatter(logging.Formatter('%(levelname)s:%(message)s'))
    old_level, old_propagate = logger.level, logger.propagate
    logger.propagate = False
    handler.setLevel(file_level)
    handler.addFilter(lambda record: getattr(record, 'diagnostic', False))
    logger.addHandler(handler)
    terminal_level = [51, logging.ERROR, logging.WARNING, logging.INFO, logging.DEBUG][level]
    logger.setLevel(terminal_level)
    terminal_handler = logging.StreamHandler(terminal)
    terminal_handler.setLevel(terminal_level)
    terminal_handler.setFormatter(logging.Formatter('%(message)s'))
    terminal_handler.addFilter(lambda record: getattr(record, 'terminal', True))
    logger.addHandler(terminal_handler)
    try:
        with redirect_stdout(terminal), \
                patch('pykwb.kwb.time.strftime', return_value='FIXED TIME'):
            try:
                await reader.listen_forever()
            except EOFError:
                pass
            _print_summary(reader)
            await reader._input._connection_lost(ConnectionResetError('connection reset'))
            reader._input._next_retry_delay()
    finally:
        logger.removeHandler(terminal_handler)
        logger.removeHandler(handler)
        logger.setLevel(old_level)
        logger.propagate = old_propagate
        await reader.close()
    return {'terminal': terminal.getvalue(), 'log': logfile.getvalue()}


class OutputCompatibilityTests(unittest.IsolatedAsyncioTestCase):
    async def test_info_omits_raw_bytes_from_terminal_and_file(self):
        actual = await capture_output(3, file_level=logging.INFO)
        self.assertNotIn('READ:', actual['terminal'])
        self.assertEqual(actual['log'], '')

    async def test_existing_terminal_and_log_output(self):
        expected = json.loads(FIXTURE.read_text())
        for level in range(5):
            with self.subTest(level=level):
                actual = await capture_output(level)
                self.assertEqual(actual, expected[f'{level}:False'])
