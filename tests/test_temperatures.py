"""Regression coverage for boiler temperature layouts and wire framing."""
import asyncio
import unittest
from pathlib import Path
from unittest.mock import patch
from pykwb.messages import FrameType, Message, parse_message, _byte_rot_left
from pykwb.messages import decode_temperature

from pykwb.kwb import KWBEasyfire, PROP_MODE_FILE, PROP_MODE_TCP, PROP_SENSOR_TEMPERATURE, PROP_SENSOR_FLAG


ROOT = Path(__file__).resolve().parents[1]


def frame(message_id, payload, frame_type=FrameType.SENSE):
    """Encode a frame with the KWB rotating checksum and payload escaping."""
    header = bytes((2, len(payload) + 5, message_id, 1))
    checksum = 0
    for value in header + payload:
        checksum = ((checksum << 1) | (checksum >> 7)) & 255
        checksum += value
        if checksum > 255:
            checksum -= 255
    return ((b'\x02' if frame_type is FrameType.SENSE else b'') + header
            + payload.replace(b'\x02', b'\x02\x00') + bytes((checksum,)))


class TemperatureTests(unittest.IsolatedAsyncioTestCase):
    def make_reader(self):
        reader = KWBEasyfire(PROP_MODE_TCP, _config={
            'connection': {'reconnect': False}, 'include_unkeyed': True})
        reader.load_sensors()
        return reader

    async def test_signed_temperatures_and_disconnected_sensor(self):
        for encoded, expected in ((b'\x02\x5f', 60.7), (b'\xff\xc9', -5.5),
                                  (b'\x80\x00', -3276.8), (b'\x00\x00', 0),
                                  (b'\x01\xf4', 50), (b'\x05\x14', None)):
            with self.subTest(encoded=encoded):
                self.assertEqual(decode_temperature(*encoded), expected)

    async def test_framing_escapes_lengths_and_checksum(self):
        reader = self.make_reader()
        # Include an escaped 2 followed by a real zero, plus another escaped 2.
        payload = b'\x02\x00\x02\x07'
        good = frame(32, payload)
        corrupt = good[:-1] + bytes((good[-1] ^ 1,))
        stream = iter(corrupt + good + frame(33, b'\x01' * 24, frame_type=FrameType.CONTROL))
        with patch.object(reader._input, 'read_byte', side_effect=lambda: next(stream)):
            self.assertEqual(await reader._read_message(), Message(32, 1, payload, FrameType.SENSE))
            self.assertEqual(await reader._read_message(), Message(33, 1, b'\x01' * 24, FrameType.CONTROL))

    def test_byte_rotation_wraps_within_one_byte(self):
        for value, distance, expected in ((0x80, 1, 1), (0x81, 1, 3),
                                           (0x03, 7, 0x81), (0xff, 4, 0xff),
                                           (0x42, 0, 0x42), (0x42, 8, 0x42)):
            with self.subTest(value=value, distance=distance):
                self.assertEqual(_byte_rot_left(value, distance), expected)

    async def test_sensors_update_only_after_full_frame_validation(self):
        for frame_type in FrameType:
            for payload in (b'', b'\x02\x00\x02\x07'):
                with self.subTest(frame_type=frame_type, payload=payload):
                    reader = self.make_reader()
                    wire = frame(80, payload, frame_type=frame_type)
                    corrupt = wire[:-1] + bytes((wire[-1] ^ 1,))
                    truncated = bytes((2, 25, 17, 1, 255))
                    source = iter(corrupt + truncated + wire)

                    def read():
                        try:
                            value = next(source)
                        except StopIteration:
                            raise EOFError from None
                        update.assert_not_called()
                        return value

                    with patch.object(reader, '_update_sensors', wraps=reader._update_sensors) as update, \
                            patch.object(reader._input, 'read_byte', side_effect=read):
                        with self.assertRaises(EOFError):
                            await reader.listen_forever()
                        update.assert_called_once()
                        message = update.call_args.args[0]
                        self.assertEqual((message.message_id, message.payload), (80, payload))
                    self.assertEqual(reader._sensors[80][0].value, payload)

    async def test_recorded_boiler_layouts(self):
        cases = (
            ('kwb_17_16.txt', 16, 59,
             [None] * 13),
            ('kwb_33_32.txt', 32, 62,
             [30.4, 77.5, 45.3, 74.1, None, None, 14.6, 73.1,
              32.4, 19.8, 50.0, None, 3276.7]),
        )
        for filename, temperature_id, count, expected in cases:
            with self.subTest(filename=filename):
                reader = KWBEasyfire(PROP_MODE_FILE, _file_path=ROOT / 'tests' / 'data' / filename,
                                     _config={'include_unkeyed': True})
                reader.load_sensors()
                self.addAsyncCleanup(reader.close)
                counts = {}
                while True:
                    try:
                        packet = await reader._read_message()
                    except EOFError:
                        break
                    message_id, payload = packet.message_id, packet.payload
                    counts[message_id] = counts.get(message_id, 0) + 1
                    if message_id == temperature_id and counts[message_id] == 1:
                        message = Message(message_id, 1, bytes(payload), FrameType.SENSE)
                        reader._update_sensors(parse_message(reader._sensors, message))
                        sensors = [s for s in reader._sensors[32]
                                   if s.sensor_type == PROP_SENSOR_TEMPERATURE]
                        self.assertEqual([s.value for s in sensors], expected)
                        self.assertEqual([s.available for s in sensors],
                                         [v is not None for v in expected])
                self.assertEqual(counts, {temperature_id: count, temperature_id + 1: count})

    async def test_truncated_payload_recovers_at_next_header(self):
        for frame_type in FrameType:
            with self.subTest(frame_type=frame_type):
                reader = self.make_reader()
                # A frame declaring 20 payload bytes stops after just one.
                truncated = bytes((2, 25, 17, 1, 255))
                payload = b'\x02\x00\x02\x07'
                stream = iter(truncated + frame(32, payload, frame_type=frame_type))
                with patch.object(reader._input, 'read_byte', side_effect=lambda: next(stream)):
                    self.assertEqual(await reader._read_message(), Message(32, 1, payload, frame_type))

    async def test_empty_and_short_payloads_do_not_crash_temperature_decoder(self):
        reader = self.make_reader()
        for message_id in (16, 32, 64, 255):
            for length in range(33):
                with self.subTest(message_id=message_id, length=length):
                    message = Message(message_id, 1, bytes(length), FrameType.SENSE)
                    reader._update_sensors(parse_message(reader._sensors, message))

    async def test_closed_tcp_connection_raises_eof(self):
        reader = self.make_reader()
        reader._input._reader = asyncio.StreamReader()
        reader._input._reader.feed_eof()
        with self.assertRaises(EOFError):
            await reader.listen_forever()
        self.assertIsNone(reader._input._reader)

    async def test_eof_in_partial_message_raises(self):
        reader = self.make_reader()
        with patch.object(reader._input, 'read_byte', side_effect=[2, 25, 17, 1, EOFError()]):
            with self.assertRaises(EOFError):
                await reader.listen_forever()
        self.assertIsNone(reader._input._reader)

    async def test_captured_short_unknown_frame_keeps_reader_running(self):
        reader = self.make_reader()
        message = Message(33, 1, bytes((255, 255, 255)), FrameType.SENSE)
        reader._update_sensors(parse_message(reader._sensors, message))
        flags_before = [sensor.value for sensor in reader._sensors[33][1:]]
        payload = bytearray(32)
        payload[12:14] = b'\x02\xe5'
        wire = bytes((2, 7, 0, 65, 27, 82, 62)) + frame(32, payload)
        position = 0

        def read_byte():
            nonlocal position
            if position == len(wire):
                raise EOFError
            value = wire[position]
            position += 1
            return value

        with patch.object(reader._input, 'read_byte', side_effect=read_byte):
            with self.assertRaises(EOFError):
                await reader.listen_forever()
        self.assertEqual(reader._sensors[33][0].value, bytes((255, 255, 255)))
        self.assertEqual([sensor.value for sensor in reader._sensors[33][1:]], flags_before)
        self.assertEqual(next(s for s in reader.get_sensors() if s.key == 'boiler_temp').value, 74.1)

    async def test_message_33_flags_use_message_33_positions(self):
        reader = self.make_reader()
        positions = [
            (1, 2), (1, 5), (1, 6), (1, 7), (2, 0), (2, 1), (2, 2),
            (2, 3), (2, 4), (2, 5), (2, 6), (2, 7), (3, 0),
            (3, 2), (3, 6), (3, 7), (4, 1), (4, 5), (5, 0),
            (9, 1), (9, 2), (16, 2),
        ]
        flags = [s for s in reader._sensors[33]
                 if s.sensor_type == PROP_SENSOR_FLAG]
        self.assertEqual(len(flags), len(positions))
        # Walk every payload bit to detect wrong offsets and cross-talk.
        for offset in range(24):
            for bit in range(8):
                payload = bytearray(24)
                payload[offset] = 1 << bit
                message = Message(33, 1, bytes(payload), FrameType.SENSE)
                reader._update_sensors(parse_message(reader._sensors, message))
                for sensor, position in zip(flags, positions):
                    expected = None if position is None else int(position == (offset, bit))
                    with self.subTest(sensor=sensor.name, offset=offset, bit=bit):
                        self.assertEqual(sensor.value, expected)
                        self.assertEqual(sensor.available, position is not None)

    async def test_short_message_33_payloads_mark_missing_flags_unavailable(self):
        reader = self.make_reader()
        for length in range(25):
            message = Message(33, 1, bytes(bytes((255,)) * 24), FrameType.SENSE)
            reader._update_sensors(parse_message(reader._sensors, message))
            message = Message(33, 1, bytes(length), FrameType.SENSE)
            reader._update_sensors(parse_message(reader._sensors, message))
            for sensor in reader._sensors[33][1:]:
                if sensor.sensor_type != PROP_SENSOR_FLAG:
                    continue
                present = sensor.index < length
                with self.subTest(sensor=sensor.name, length=length):
                    self.assertEqual(sensor.value, 0 if present else None)
                    self.assertEqual(sensor.available, present)

    async def test_unrelated_messages_do_not_overwrite_boiler_temperatures(self):
        reader = self.make_reader()
        payload = bytearray(32)
        payload[12:14] = b'\x02\xe5'
        message = Message(32, 1, bytes(payload), FrameType.SENSE)
        reader._update_sensors(parse_message(reader._sensors, message))
        message = Message(64, 1, bytes(24), FrameType.SENSE)
        reader._update_sensors(parse_message(reader._sensors, message))
        self.assertEqual(next(s for s in reader.get_sensors() if s.key == 'boiler_temp').value, 74.1)

    async def test_message_64_wire_decoding_and_missing_temperatures(self):
        reader = self.make_reader()
        payload = bytearray(23)
        payload[19:21] = b'\x02\x5f'
        payload[21:23] = b'\xff\xc9'
        wire = iter(frame(64, payload))
        with patch.object(reader._input, 'read_byte', side_effect=lambda: next(wire)):
            packet = await reader._read_message()
            message = Message(packet.message_id, 1, bytes(packet.payload), FrameType.SENSE)
            reader._update_sensors(parse_message(reader._sensors, message))
        sensors = {s.key: s for s in reader.get_sensors() if s.key}
        loop_4 = sensors['zone_4_out_temp']
        loop_3 = sensors['zone_3_out_temp']
        self.assertEqual((loop_4.value, loop_3.value), (60.7, -5.5))
        self.assertEqual((loop_4.unit_of_measurement, loop_3.unit_of_measurement),
                         ('°C', '°C'))
        self.assertEqual(reader._sensors[64][0].value, payload)
        self.assertIsNone(sensors['boiler_temp'].value)

        # Other message IDs cannot change these values.
        message = Message(32, 1, bytes(32), FrameType.SENSE)
        reader._update_sensors(parse_message(reader._sensors, message))
        message = Message(65, 1, bytes(23), FrameType.SENSE)
        reader._update_sensors(parse_message(reader._sensors, message))
        self.assertEqual((loop_4.value, loop_3.value), (60.7, -5.5))

        payload[19:21] = b'\x05\x14'
        message = Message(64, 1, bytes(payload[:22]), FrameType.SENSE)
        reader._update_sensors(parse_message(reader._sensors, message))
        self.assertEqual((loop_4.value, loop_3.value), (None, None))
        self.assertFalse(loop_4.available)
        self.assertFalse(loop_3.available)
        message = Message(64, 1, bytes(23), FrameType.SENSE)
        reader._update_sensors(parse_message(reader._sensors, message))
        self.assertEqual((loop_4.value, loop_3.value), (0, 0))
        self.assertTrue(loop_4.available)
        self.assertTrue(loop_3.available)

    async def test_unconfigured_packets_get_only_a_summary(self):
        reader = KWBEasyfire(PROP_MODE_TCP, _config={'connection': {'reconnect': False}})
        reader.load_sensors()
        wire = iter(frame(87, bytes(24), frame_type=FrameType.CONTROL)
                    + frame(250, bytes(34)))

        def read_byte():
            try:
                return next(wire)
            except StopIteration:
                raise EOFError from None

        with self.assertLogs('pykwb.kwb', level='INFO') as output, \
                patch.object(reader._input, 'read_byte', side_effect=read_byte):
            with self.assertRaises(EOFError):
                await reader.listen_forever()
        self.assertEqual(
            [record.getMessage().strip() for record in output.records],
            ['Message ID 87 frame_type=CONTROL counter=1 length=24',
             'Message ID 250 frame_type=SENSE counter=1 length=34'])
        self.assertTrue(all(s.value is None for s in reader.get_sensors()))

    async def test_message_ids_decode_with_either_header_form(self):
        for message_id in self.make_reader()._sensors:
            results = []
            for frame_type in FrameType:
                reader = self.make_reader()
                payload = bytes(range(74))
                source = iter(frame(message_id, payload, frame_type=frame_type))
                with patch.object(reader._input, 'read_byte', side_effect=lambda: next(source)):
                    packet = await reader._read_message()
                self.assertEqual(packet, Message(message_id, 1, payload, frame_type))
                message = Message(packet.message_id, 1, bytes(packet.payload), FrameType.SENSE)
                reader._update_sensors(parse_message(reader._sensors, message))
                self.assertEqual(reader._sensors[message_id][0].value, payload)
                self.assertEqual(reader._sensors[message_id][0].name, 'RAW %d' % message_id)
                self.assertTrue(all(s.value is None
                                    for mid, sensors in reader._sensors.items()
                                    if mid != message_id for s in sensors))
                results.append([(s.value, s.available) for s in reader._sensors[message_id]])
            self.assertEqual(*results)

    async def test_disconnected_sensor_clears_previous_reading(self):
        reader = self.make_reader()
        payload = bytearray(32)
        payload[12:14] = b'\x02\xe5'
        message = Message(32, 1, bytes(payload), FrameType.SENSE)
        reader._update_sensors(parse_message(reader._sensors, message))
        sensor = next(s for s in reader.get_sensors() if s.key == 'boiler_temp')
        self.assertTrue(sensor.available)
        payload[12:14] = b'\x05\x14'
        message = Message(32, 1, bytes(payload), FrameType.SENSE)
        reader._update_sensors(parse_message(reader._sensors, message))
        self.assertIsNone(sensor.value)
        self.assertFalse(sensor.available)


if __name__ == '__main__':
    unittest.main()
