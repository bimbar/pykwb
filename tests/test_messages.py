"""CSV sensor construction and decoding beyond the original sensor lists."""
import unittest

from pykwb.kwb import (
    KWBEasyfire, KWBEasyfireSensor, PROP_SENSOR_RAW, PROP_SENSOR_TEMPERATURE,
    PROP_SENSOR_PRESSURE, PROP_SENSOR_DURATION, PROP_SENSOR_SPEED, PROP_SENSOR_NUMBER,
)


class MessageSensorTests(unittest.TestCase):
    def setUp(self):
        self.reader = KWBEasyfire(-1)
        self.reader._debug_level = 0

    def sensor(self, name):
        return next(s for s in self.reader.get_sensors() if s.name == name)

    def test_csv_definitions_and_raw_diagnostics(self):
        sensors = self.reader.get_sensors()
        self.assertEqual(sum(s.sensor_type != PROP_SENSOR_RAW for s in sensors), 57)
        self.assertEqual(sum(s.sensor_type == PROP_SENSOR_RAW for s in sensors), 3)
        self.assertEqual(self.sensor('Boiler Temp').key, 'boiler_temp')
        self.assertEqual(self.sensor('Boiler Temp').unit_of_measurement, '°C')
        self.assertEqual(self.sensor('Pressure').unit_of_measurement, 'mbar')

    def test_sense_flags_signed_integers_and_scaled_numbers(self):
        payload = bytearray(73)
        payload[3] = 1 << 6
        payload[32:34] = (-123).to_bytes(2, 'big', signed=True)
        payload[34:36] = (1234).to_bytes(2, 'big')
        payload[69:71] = (100).to_bytes(2, 'big')
        payload[71:73] = (65535).to_bytes(2, 'big')
        self.reader._decode_sense_packet(32, payload)
        self.assertEqual(self.sensor('Ash Can OK').value, 1)
        self.assertEqual(self.sensor('Heater Running').value, 0)
        self.assertEqual(self.sensor('Photodiode').value, -123)
        self.assertAlmostEqual(self.sensor('Pressure').value, 1.234)
        self.assertEqual(self.sensor('Suction Speed').value, 60)
        self.assertEqual(self.sensor('Fan Speed').value, 39321)
        self.reader._decode_sense_packet(32, bytes(34))
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
                sensor.decode(b'\x05\x14')
                self.assertEqual(sensor.sensor_type, PROP_SENSOR_DURATION)
                self.assertEqual(sensor.unit_of_measurement, units)
                self.assertEqual(sensor.value, 13000)

    def test_control_numbers_and_csv_ash_discharge_position(self):
        payload = bytearray(17)
        payload[2] = 1 << 6
        payload[8] = 255
        payload[10:12] = (123).to_bytes(2, 'big')
        payload[12:14] = (50).to_bytes(2, 'big')
        self.reader._decode_ctrl_packet(33, payload)
        self.assertEqual(self.sensor('Ash Discharge').value, 1)
        self.assertAlmostEqual(self.sensor('Boiler Pumping').value, 100)
        self.assertEqual(self.sensor('Feed Screw Cycle Time').value, 1230)
        self.assertEqual(self.sensor('Feed Screw On Time').value, 500)
        self.assertEqual(self.sensor('Heater Output').value, 50)
        self.reader._decode_ctrl_packet(33, bytes(11))
        self.assertIsNone(self.sensor('Feed Screw Cycle Time').value)
        self.assertFalse(self.sensor('Feed Screw On Time').available)
