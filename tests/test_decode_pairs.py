"""Standalone exploratory decoding and the retained CLI argument."""
from contextlib import redirect_stdout
from io import StringIO
import unittest
from unittest.mock import patch

from pykwb.kwb import KWBEasyfire, main
from pykwb.decode import decode_pairs


class PairDecodeTests(unittest.TestCase):
    def test_both_alignments_and_trailing_byte(self):
        reader = KWBEasyfire(-1)
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
        for args in ([], ['--decode'], ['--decode', '64', '87'],
                     ['--decode', '-1', '256']):
            with self.subTest(args=args), \
                    patch('sys.argv', ['kwb', '--no-summary'] + args), \
                    patch('pykwb.kwb.KWBEasyfire', autospec=True) as factory:
                main()
            self.assertNotIn('_config', factory.call_args.kwargs)

    def test_cli_rejects_noninteger_ids(self):
        for value in ('abc', '32.5'):
            with self.subTest(value=value), patch('sys.argv', ['kwb', '--decode', value]), \
                    patch('sys.stderr'), patch('pykwb.kwb.KWBEasyfire') as factory:
                with self.assertRaises(SystemExit) as error:
                    main()
                self.assertEqual(error.exception.code, 2)
                factory.assert_not_called()
