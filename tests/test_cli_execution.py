"""CLI uses async listening and always closes its connection."""
import unittest
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from unittest.mock import AsyncMock, patch

from pykwb.kwb import PROP_MODE_FILE, main


class CLIExecutionTests(unittest.TestCase):
    def test_log_levels_apply_before_listening(self):
        cases = [([], 3), (['--log', 'false'], 0),
                 (['--log-level', 'trace', '--log', 'false'], 0)]
        cases += [(['--log-level', name], level) for name, level in
                  [('none', 0), ('error', 1), ('warn', 2), ('warning', 2),
                   ('info', 3), ('debug', 4), ('trace', 5), ('DEBUG', 4)]]
        for options, expected in cases:
            with self.subTest(options=options), \
                    patch('sys.argv', ['kwb', '--wait', '0', '--no-summary'] + options), \
                    patch('pykwb.kwb.KWBEasyfire') as factory:
                reader = factory.return_value
                reader.close = AsyncMock()
                reader.listen_for = AsyncMock(side_effect=lambda **kwargs: self.assertEqual(reader._debug_level, expected))
                main()
                reader.listen_for.assert_awaited_once_with(seconds=0)
                reader.close.assert_awaited_once_with()

    def test_invalid_log_level_is_rejected(self):
        with patch('sys.argv', ['kwb', '--log-level', 'invalid']), \
                patch('sys.stderr'), patch('pykwb.kwb.KWBEasyfire') as factory:
            with self.assertRaises(SystemExit) as error:
                main()
            self.assertEqual(error.exception.code, 2)
            factory.assert_not_called()

    def test_summary_printed_once_after_listener_and_close(self):
        for options, method in ((['--forever', '--wait', '0'], 'listen_forever'),
                                (['--wait', '0.25'], 'listen_for')):
            with self.subTest(options=options), \
                    patch('sys.argv', ['kwb'] + options), \
                    patch('pykwb.kwb.KWBEasyfire', autospec=True) as factory, \
                    patch('pykwb.kwb._print_summary') as summary:
                reader = factory.return_value
                events = []
                getattr(reader, method).side_effect = lambda **kwargs: events.append('listen')
                reader.close.side_effect = lambda: events.append('close')
                summary.side_effect = lambda reader: events.append('summary')
                main()
                getattr(reader, method).assert_awaited_once()
                summary.assert_called_once_with(reader)
                self.assertEqual(events, ['listen', 'close', 'summary'])

    def test_script_and_module_load_bundled_sensors(self):
        root = Path(__file__).resolve().parents[1]
        environment = os.environ.copy()
        environment.pop('PYTHONPATH', None)
        with tempfile.TemporaryDirectory() as directory:
            capture = Path(directory) / 'empty.txt'
            capture.write_text((root / 'tests' / 'data' / 'kwb_33_32.txt').read_text())
            for command, cwd in (
                    ([sys.executable, str(root / 'pykwb' / 'kwb.py')], directory),
                    ([sys.executable, '-m', 'pykwb.kwb'], root)):
                with self.subTest(command=command):
                    result = subprocess.run(
                        command + ['--file', '--name', str(capture),
                                   '--wait', '5', '--log-level', 'info'],
                        cwd=cwd, env=environment, capture_output=True, text=True,
                        check=False, timeout=10,
                    )
                    self.assertNotEqual(result.returncode, 0)
                    self.assertIn('EOFError: EOF', result.stderr)
                    self.assertIn('Boiler Temp', result.stdout)

    def test_cli_uses_async_listener_and_closes(self):
        with patch('sys.argv', ['kwb', '--file', '--name', 'capture.txt',
                               '--wait', '0.25', '--no-summary']), \
                patch('pykwb.kwb.KWBEasyfire') as factory:
            reader = factory.return_value
            reader.listen_for = AsyncMock()
            reader.close = AsyncMock()
            main()
            self.assertEqual(factory.call_args.args[0], PROP_MODE_FILE)
            reader.listen_for.assert_awaited_once_with(seconds=0.25)
            reader.close.assert_awaited_once_with()

    def test_cli_closes_after_listener_failure(self):
        with patch('sys.argv', ['kwb', '--wait', '1']), \
                patch('pykwb.kwb.KWBEasyfire') as factory:
            reader = factory.return_value
            reader.listen_for = AsyncMock(side_effect=OSError('connection failed'))
            reader.close = AsyncMock()
            with self.assertRaises(OSError):
                main()
            reader.close.assert_awaited_once()
