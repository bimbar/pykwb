"""Exact output captured before the listener pipeline refactor."""
from contextlib import redirect_stdout
from io import StringIO
import json
import logging
from pathlib import Path
import unittest
from unittest.mock import patch

from pykwb.kwb import KWBEasyfire, _print_summary
from pykwb.messages import FrameType, load_messages
from test_temperatures import frame


FIXTURE = Path(__file__).parent / 'data' / 'listener_output.json'


async def capture_output(level):
    rows = [row for row in load_messages()
            if (row['message_id'], row['key']) in
            {('32', 'boiler_temp'), ('80', 'loop_4_room_temp')}]
    with patch('pykwb.kwb.load_messages', return_value=rows):
        reader = KWBEasyfire(-1)
    reader._debug_level = level
    wire = (frame(80, bytes(18) + b'\x00\xe6')
            + frame(87, b'\x00\x00\x00\x02\x5f', FrameType.CONTROL)
            + frame(250, b'', FrameType.CONTROL))
    source = iter(wire)

    async def read():
        try:
            value = next(source)
        except StopIteration:
            raise EOFError from None
        reader._record_byte(value)
        return value

    terminal, logfile = StringIO(), StringIO()
    logger = logging.getLogger('pykwb.kwb')
    handler = logging.StreamHandler(logfile)
    handler.setFormatter(logging.Formatter('%(levelname)s:%(message)s'))
    old_level, old_propagate = logger.level, logger.propagate
    logger.setLevel(logging.DEBUG)
    logger.propagate = False
    logger.addHandler(handler)
    try:
        with redirect_stdout(terminal), patch.object(reader, '_read_async_byte', side_effect=read), \
                patch('pykwb.kwb.time.strftime', return_value='FIXED TIME'):
            try:
                await reader.listen_forever()
            except EOFError:
                pass
            _print_summary(reader)
            await reader._connection_lost(ConnectionResetError('connection reset'))
            reader._next_retry_delay()
    finally:
        logger.removeHandler(handler)
        logger.setLevel(old_level)
        logger.propagate = old_propagate
        await reader.close()
    return {'terminal': terminal.getvalue(), 'log': logfile.getvalue()}


class OutputCompatibilityTests(unittest.IsolatedAsyncioTestCase):
    async def test_existing_terminal_and_log_output(self):
        expected = json.loads(FIXTURE.read_text())
        for level in range(6):
            with self.subTest(level=level):
                actual = await capture_output(level)
                self.assertEqual(actual, expected[f'{level}:False'])
