"""Exploratory decoding and its CLI output."""
from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

from pykwb.kwb import PROP_MODE_TCP, KWBEasyfire, main
from pykwb.messages import decode_pairs
from test_temperatures import frame


class PairDecodeTests(unittest.TestCase):
    def test_both_alignments_and_trailing_byte(self):
        reader = KWBEasyfire(PROP_MODE_TCP, _config={'connection': {'reconnect': False}})
        reader.load_sensors()
        payload = bytes.fromhex('00 00 00 02 5f ff c9 05 14')
        before = [s.value for s in reader.get_sensors()]
        output = StringIO()
        with redirect_stdout(output):
            lines = list(decode_pairs(87, payload))
        self.assertEqual(output.getvalue(), '')
        self.assertEqual(lines, [
            'ID 87 two-byte decode from offset 3:',
            '  Offset 3: raw=607 temperature=60.7 mbar=0.607 rpm=364.2 ms=6070',
            '  Offset 5: raw=-55 temperature=-5.5 mbar=65.481 rpm=39288.6 ms=654810',
            '  Offset 7: raw=1300 temperature=None mbar=1.3 rpm=780.0 ms=13000',
            'ID 87 two-byte decode from offset 4:',
            '  Offset 4: raw=24575 temperature=2457.5 mbar=24.575 rpm=14745.0 ms=245750',
            '  Offset 6: raw=-14075 temperature=-1407.5 mbar=51.461 rpm=30876.6 ms=514610',
        ])
        self.assertEqual([s.value for s in reader.get_sensors()], before)

    def test_unsigned_units_at_zero_and_maximum(self):
        for pair, expected in (
                (b'\x00\x00', 'mbar=0.0 rpm=0.0 ms=0'),
                (b'\xff\xff', 'mbar=65.535 rpm=39321.0 ms=655350')):
            with self.subTest(pair=pair):
                lines = list(decode_pairs(64, bytes(3) + pair))
                self.assertTrue(lines[1].endswith(expected), lines[1])

    def test_short_payloads_have_no_pairs(self):
        for length in range(5):
            with self.subTest(length=length):
                self.assertEqual(list(decode_pairs(87, bytes(length))), [
                    'ID 87 two-byte decode from offset 3:',
                    'ID 87 two-byte decode from offset 4:',
                ])

    def test_cli_list_and_default(self):
        for args, expected in (([], []), (['--decode'], []),
                               (['--decode=65'], [65]),
                               (['--decode', '64', '87'], [64, 87]),
                               (['--decode', '-1', '256'], [-1, 256])):
            with self.subTest(args=args), \
                    patch('sys.argv', ['kwb', '--no-summary'] + args), \
                    patch('pykwb.kwb.KWBEasyfire', autospec=True) as factory:
                main()
            self.assertEqual(factory.call_args.kwargs['_config']['decode'], expected)

    def test_cli_decodes_only_selected_messages_and_respects_log_level(self):
        payload = bytes.fromhex('00 00 00 02 5f 01')
        expected = (
            'ID 65 two-byte decode from offset 3:\n'
            '  Offset 3: raw=607 temperature=60.7 mbar=0.607 rpm=364.2 ms=6070\n'
            'ID 65 two-byte decode from offset 4:\n'
            '  Offset 4: raw=24321 temperature=2432.1 mbar=24.321 rpm=14592.6 ms=243210\n'
        )
        with TemporaryDirectory() as directory:
            capture = Path(directory) / 'capture.txt'
            capture.write_text(''.join(f'{byte}\n' for byte in
                                      frame(65, payload) + frame(66, payload)))
            for args, visible in (([], False), (['--decode=65'], True),
                                  (['--decode', '65', '67'], True),
                                  (['--decode=65', '--log-level=debug'], True),
                                  (['--decode=65', '--log-level=warn'], False),
                                  (['--decode=65', '--log=false'], False)):
                with self.subTest(args=args):
                    output = StringIO()
                    with patch('sys.argv', ['kwb', '--file', '--name', str(capture),
                                            '--forever', '--no-summary'] + args), \
                            redirect_stdout(output), self.assertRaises(EOFError):
                        main()
                    text = output.getvalue()
                    if visible:
                        self.assertIn(expected, text)
                    else:
                        self.assertNotIn('two-byte decode', text)
                    self.assertNotIn('ID 66 two-byte decode', text)

    def test_cli_rejects_noninteger_ids(self):
        for value in ('abc', '32.5'):
            with self.subTest(value=value), patch('sys.argv', ['kwb', '--decode', value]), \
                    patch('sys.stderr'), patch('pykwb.kwb.KWBEasyfire') as factory:
                with self.assertRaises(SystemExit) as error:
                    main()
                self.assertEqual(error.exception.code, 2)
                factory.assert_not_called()
