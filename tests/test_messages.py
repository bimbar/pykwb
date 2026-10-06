"""CSV sensor construction and decoding beyond the original sensor lists."""
import unittest
from contextlib import redirect_stdout
from io import StringIO
from unittest.mock import patch

from pykwb.messages import FrameType, Message, load_messages, parse_message

from pykwb.kwb import (
    KWBEasyfire, KWBEasyfireSensor, _print_summary, PROP_SENSOR_RAW, PROP_SENSOR_TEMPERATURE,
    PROP_SENSOR_PRESSURE, PROP_SENSOR_DURATION, PROP_SENSOR_SPEED, PROP_SENSOR_NUMBER,
)


class MessageSensorTests(unittest.TestCase):
    def setUp(self):
        self.reader = KWBEasyfire(-1)

    def sensor(self, name):
        return next(s for s in self.reader.get_sensors() if s.name == name)

    def test_csv_definitions_and_raw_diagnostics(self):
        sensors = self.reader.get_sensors()
        rows = load_messages()
        self.assertEqual(set(self.reader._sensors), {int(r['message_id']) for r in rows})
        self.assertEqual(sum(s.sensor_type != PROP_SENSOR_RAW for s in sensors),
                         len({r['key'] or (r['name_en'] or r['name_de']).lower().replace(' ', '_')
                              for r in rows if r['type'] != '???'}))
        self.assertEqual(sum(s.sensor_type == PROP_SENSOR_RAW for s in sensors),
                         len(self.reader._sensors))
        self.assertEqual(self.sensor('Boiler Temp').key, 'boiler_temp')
        self.assertEqual(self.sensor('Boiler Temp').unit_of_measurement, '°C')
        self.assertEqual(self.sensor('Pressure').unit_of_measurement, 'mbar')

    def test_missing_keys_use_english_then_german_name(self):
        row = dict(load_messages()[0], key='', name_en='Ash Clearing On',
                   name_de='Asche Austragung')
        self.assertEqual(KWBEasyfireSensor.from_message(row).key, 'ash_clearing_on')
        row['name_en'] = ''
        self.assertEqual(KWBEasyfireSensor.from_message(row).key, 'asche_austragung')
        row['key'] = 'Explicit_KEY'
        self.assertEqual(KWBEasyfireSensor.from_message(row).key, 'Explicit_KEY')

    def test_shared_key_has_one_final_state_across_message_layouts(self):
        row = dict(load_messages()[0], message_id='120', key='shared_temp',
                   name_en='Shared Temp', type='int', offset='0', length='2',
                   bit='', signed='1', scale='0.1', units='C')
        rows = [row, dict(row, message_id='121', offset='2'),
                dict(row, message_id='122', key='other_temp')]
        with patch('pykwb.kwb.load_messages', return_value=rows):
            reader = KWBEasyfire(-1)
        sensor = next(s for s in reader.get_sensors() if s.key == 'shared_temp')
        self.assertEqual(len([s for s in reader.get_sensors()
                              if s.sensor_type != PROP_SENSOR_RAW]), 2)
        for message_id, payload, expected in (
                (120, b'\x00\xe6', 23),
                (121, b'\xff\xff\x01\x2c', 30),
                (120, b'\x05\x14', None),
                (121, b'\xff\xff\x00\xfa', 25)):
            with self.subTest(message_id=message_id, expected=expected):
                message = Message(message_id, 1, payload, FrameType.SENSE)
                reader._update_sensors(parse_message(reader._sensors, message))
                self.assertEqual(sensor.value, expected)
                self.assertEqual(sensor.available, expected is not None)
                self.assertIs(next(s for s in reader.get_sensors()
                                   if s.key == 'shared_temp'), sensor)
        self.assertIsNone(next(s for s in reader.get_sensors() if s.key == 'other_temp').value)
        output = StringIO()
        with redirect_stdout(output):
            _print_summary(reader)
        # Identical display names with different keys remain separate sensors.
        self.assertEqual(output.getvalue().count('Shared Temp:'), 2)
        self.assertIn('V: 25.0', output.getvalue())

    def test_generated_key_merges_with_explicit_key_in_summary(self):
        reader = self.reader
        message = Message(33, 1, bytes(24), FrameType.SENSE)
        reader._update_sensors(parse_message(reader._sensors, message))
        sensors = [s for s in reader.get_sensors() if s.key == 'ash_clearing_on']
        self.assertEqual(len(sensors), 1)
        self.assertEqual(sensors[0].value, 0)
        output = StringIO()
        with redirect_stdout(output):
            _print_summary(reader)
        self.assertEqual(output.getvalue().count('Ash Clearing On:'), 1)

    def test_csv_alone_selects_message_ids(self):
        row = dict(load_messages()[-1], message_id='123', key='custom_temp',
                   name_en='Custom Temp', offset='0', type='int', signed='1',
                   length='2', scale='0.1', units='C')
        with patch('pykwb.kwb.load_messages', return_value=[row]):
            reader = KWBEasyfire(-1)
        self.assertEqual(set(reader._sensors), {123})
        message = Message(32, 1, bytes(20), FrameType.SENSE)
        reader._update_sensors(parse_message(reader._sensors, message))
        self.assertTrue(all(s.value is None for s in reader.get_sensors()))
        message = Message(123, 1, b'\x00\xe6', FrameType.SENSE)
        reader._update_sensors(parse_message(reader._sensors, message))
        self.assertEqual([s.value for s in reader.get_sensors()], [b'\x00\xe6', 23])

    def test_undocumented_type_retains_raw_message(self):
        row = dict(load_messages()[0], message_id='123', type='???')
        with patch('pykwb.kwb.load_messages', return_value=[row]):
            reader = KWBEasyfire(-1)
        message = Message(123, 1, b'\xff', FrameType.SENSE)
        reader._update_sensors(parse_message(reader._sensors, message))
        self.assertEqual(len(reader.get_sensors()), 1)
        self.assertEqual(reader.get_sensors()[0].value, b'\xff')

    def test_message_80_temperature_and_short_payload(self):
        payload = bytes(18) + b'\x00\xe6'
        message = Message(80, 1, bytes(payload), FrameType.SENSE)
        self.reader._update_sensors(parse_message(self.reader._sensors, message))
        sensor = self.sensor('Loop 4 Room Temp')
        self.assertEqual(sensor.value, 23)
        self.assertTrue(sensor.available)
        message = Message(80, 1, bytes(payload[:-1]), FrameType.SENSE)
        self.reader._update_sensors(parse_message(self.reader._sensors, message))
        self.assertIsNone(sensor.value)
        self.assertFalse(sensor.available)

    def test_message_32_flags_signed_integers_and_scaled_numbers(self):
        payload = bytearray(73)
        payload[3] = 1 << 6
        payload[32:34] = (-123).to_bytes(2, 'big', signed=True)
        payload[34:36] = (1234).to_bytes(2, 'big')
        payload[69:71] = (100).to_bytes(2, 'big')
        payload[71:73] = (65535).to_bytes(2, 'big')
        message = Message(32, 1, bytes(payload), FrameType.SENSE)
        self.reader._update_sensors(parse_message(self.reader._sensors, message))
        self.assertEqual(self.sensor('Ash Can OK').value, 1)
        self.assertEqual(self.sensor('Heater Running').value, 0)
        self.assertEqual(self.sensor('Photodiode').value, -123)
        self.assertAlmostEqual(self.sensor('Pressure').value, 1.234)
        self.assertEqual(self.sensor('Suction Speed').value, 60)
        self.assertEqual(self.sensor('Fan Speed').value, 39321)
        message = Message(32, 1, bytes(34), FrameType.SENSE)
        self.reader._update_sensors(parse_message(self.reader._sensors, message))
        self.assertIsNone(self.sensor('Pressure').value)
        self.assertFalse(self.sensor('Fan Speed').available)

    def test_measurement_types_from_csv_units(self):
        expected = {
            'Boiler Temp': (PROP_SENSOR_TEMPERATURE, '°C'),
            'Pressure': (PROP_SENSOR_PRESSURE, 'mbar'),
            'Suction Speed': (PROP_SENSOR_SPEED, 'rpm'),
            'Fan Speed': (PROP_SENSOR_SPEED, 'rpm'),
            'Feed Screw Cycle Time': (PROP_SENSOR_DURATION, 'ms'),
            'Feed Screw On Time': (PROP_SENSOR_DURATION, 'ms'),
            'Boiler Pumping': (PROP_SENSOR_NUMBER, '%'),
            'Photodiode': (PROP_SENSOR_NUMBER, ''),
        }
        for name, (sensor_type, units) in expected.items():
            with self.subTest(name=name):
                sensor = self.sensor(name)
                self.assertEqual(sensor.sensor_type, sensor_type)
                self.assertEqual(sensor.unit_of_measurement, units)

    def test_duration_unit_variants_preserve_scale(self):
        for units in ('ms', 'sec'):
            with self.subTest(units=units):
                sensor = KWBEasyfireSensor.from_message({
                    'message_id': '33', 'offset': '0', 'name_en': 'Duration',
                    'type': 'int', 'bit': '', 'length': '2', 'signed': '0',
                    'scale': '10', 'units': units, 'key': '',
                })
                message = parse_message({33: [sensor]},
                                        Message(33, 1, b'\x05\x14', FrameType.SENSE))
                self.assertEqual(sensor.sensor_type, PROP_SENSOR_DURATION)
                self.assertEqual(sensor.unit_of_measurement, units)
                self.assertEqual(message.values, (13000,))
                self.assertIsNone(sensor.value)

    def test_message_33_numbers_and_csv_ash_discharge_position(self):
        payload = bytearray(17)
        payload[2] = 1 << 6
        payload[8] = 255
        payload[10:12] = (123).to_bytes(2, 'big')
        payload[12:14] = (50).to_bytes(2, 'big')
        message = Message(33, 1, bytes(payload), FrameType.SENSE)
        self.reader._update_sensors(parse_message(self.reader._sensors, message))
        self.assertEqual(self.sensor('Ash Discharge').value, 1)
        self.assertAlmostEqual(self.sensor('Boiler Pumping').value, 100)
        self.assertEqual(self.sensor('Feed Screw Cycle Time').value, 1230)
        self.assertEqual(self.sensor('Feed Screw On Time').value, 500)
        self.assertEqual(self.sensor('Heater Output').value, 50)
        message = Message(33, 1, bytes(11), FrameType.SENSE)
        self.reader._update_sensors(parse_message(self.reader._sensors, message))
        self.assertIsNone(self.sensor('Feed Screw Cycle Time').value)
        self.assertFalse(self.sensor('Feed Screw On Time').available)
