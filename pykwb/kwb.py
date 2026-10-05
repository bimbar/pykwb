# -*- coding: utf-8 -*-
"""
The MIT License (MIT)

Copyright (c) 2017 Markus Peter mpeter at emdev dot de

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.


Support for KWB Easyfire central heating units.
"""

import asyncio
import logging
import time
import argparse
import serial_asyncio_fast

# Make testing easier for HomeAssistant HACS integration
if __name__ == "__main__" and not __package__:
    # Direct script execution puts pykwb/, not its parent, on sys.path.
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from pykwb.decode import decode_pairs, decode_temperature
from pykwb.messages import load_messages

PROP_LOGLEVEL_TRACE = 5
PROP_LOGLEVEL_DEBUG = 4
PROP_LOGLEVEL_INFO = 3
PROP_LOGLEVEL_WARN = 2
PROP_LOGLEVEL_ERROR = 1
PROP_LOGLEVEL_NONE = 0

PROP_MODE_SERIAL = 0
PROP_MODE_TCP = 1
PROP_MODE_FILE = 2

PROP_PACKET_SENSE = 32
PROP_PACKET_CTRL = 33
PROP_PACKET_SENSE_64 = 64

PROP_SENSOR_TEMPERATURE = 0
PROP_SENSOR_FLAG = 1
PROP_SENSOR_RAW = 2
PROP_SENSOR_NUMBER = 3
PROP_SENSOR_PRESSURE = 4
PROP_SENSOR_DURATION = 5
PROP_SENSOR_SPEED = 6

SERIAL_SPEED = 19200

_LOGGER = logging.getLogger(__name__)


class KWBEasyfireSensor:
    """This Class represents as single sensor."""

    def __init__(self, _packet, _index, _name, _sensor_type, _bit=None,
                 _length=2, _signed=True, _scale=0.1, _units="", _key=""):

        self._packet = _packet
        self._index = _index
        self._bit = _bit
        self._name = _name
        self._sensor_type = _sensor_type
        self._value = None
        self._available = False
        self._length = _length
        self._signed = _signed
        self._scale = _scale
        self._units = _units
        self._key = _key

    @classmethod
    def from_message(cls, message):
        """Create a sensor from one packet definition in messages.csv."""
        if message['type'] == 'bit':
            sensor_type = PROP_SENSOR_FLAG
        elif message['type'] == 'int':
            sensor_type = {
                'C': PROP_SENSOR_TEMPERATURE,
                'mbar': PROP_SENSOR_PRESSURE,
                'ms': PROP_SENSOR_DURATION,
                'msec': PROP_SENSOR_DURATION,
                'sec': PROP_SENSOR_DURATION,
                'rpm': PROP_SENSOR_SPEED,
            }.get(message['units'], PROP_SENSOR_NUMBER)
        else:
            raise ValueError("Unsupported sensor type: " + message['type'])
        return cls(
            int(message['message_id']), int(message['offset']),
            message['name_en'] or message['name_de'] or message['key'],
            sensor_type,
            _bit=int(message['bit']) if message['bit'] else None,
            _length=int(message['length'] or 1),
            _signed=message['signed'] == '1',
            _scale=float(message['scale'] or 1),
            _units=message['units'], _key=message['key'],
        )

    @property
    def key(self):
        """Return the optional CSV key (not necessarily unique)."""
        return self._key

    def decode(self, packet):
        """Update from an unescaped, big-endian payload."""
        if self.sensor_type == PROP_SENSOR_RAW:
            self.value = packet
            return
        offset = self.index
        length = 1 if self.sensor_type == PROP_SENSOR_FLAG else self._length
        if offset is None or offset < 0 or offset + length > len(packet):
            self.value = None
        elif self.sensor_type == PROP_SENSOR_FLAG:
            self.value = ((packet[offset] >> self.bit) & 1
                          if self.bit is not None and 0 <= self.bit < 8 else None)
        else:
            value = int.from_bytes(packet[offset:offset + length], 'big',
                                   signed=self._signed)
            if self.sensor_type == PROP_SENSOR_TEMPERATURE and value == 1300:
                self.value = None
            else:
                self.value = round(value * self._scale, 10)

    @property
    def index(self):
        """Return the unescaped payload byte offset, or None if unmapped."""
        return self._index

    @property
    def bit(self):
        """Return the bit position within the payload byte for flags."""
        return self._bit

    @property
    def name(self):
        """Returns the name of the sensor."""
        return self._name

    @property
    def sensor_type(self):
        """Return the sensor's measurement or data type."""
        return self._sensor_type

    @property
    def unit_of_measurement(self):
        """Return the CSV unit, displaying Celsius as °C."""
        if (self._sensor_type == PROP_SENSOR_TEMPERATURE):
            return "°C"
        else:
            return self._units

    @property
    def value(self):
        """Returns the value of the sensor. Unit is unit_of_measurement."""
        return self._value

    @value.setter
    def value(self, _value):
        """Sets the value of the sensor. Unit is unit_of_measurement."""
        self._available = _value is not None
        self._value = _value

    @property
    def available(self):
        """Return if sensor is available."""
        return self._available

    def __str__(self):
        """Returns an informational text representation of the sensor."""
        return self.name + ": I: " + str(self.index) + " T: " + str(self.sensor_type) + "(" + str(self.unit_of_measurement) + ") V: " + str(self.value)


# pylint: disable=too-many-instance-attributes
class KWBEasyfire:
    """Communicate asynchronously with the KWB Easyfire unit."""

    def __init__(self, _mode, _ip="", _port=0, _serial_device="", _serial_speed=19200,
                 _file_path="", _config=None):
        """Initialize the Object."""

        self._config = dict(_config or {})
        self._config['connection'] = {
            'reconnect': False,
            'connect_timeout': 5,
            'stale_timeout': 30,
            'retry_initial': 1,
            'retry_max': 30,
            **self._config.get('connection', {}),
        }
        self._debug_level = PROP_LOGLEVEL_INFO
        self._packet_parser = None
        self._reader = None
        self._writer = None
        self._file = None
        self._retry_delay = self._config['connection']['retry_initial']
        self._last_valid_packet = time.monotonic()
        for key in ('connect_timeout', 'stale_timeout', 'retry_initial', 'retry_max'):
            if not 0 < self._config['connection'][key] < float('inf'):
                raise ValueError("connection.%s must be finite and positive" % key)

        self._mode = _mode
        self._ip = _ip
        self._port = _port
        self._serial_device = _serial_device
        self._serial_speed = _serial_speed
        self._file_path = _file_path
        self._logdatalen = 1024
        self._logdata = []

        self._sensors = {
            PROP_PACKET_SENSE: [
                KWBEasyfireSensor(PROP_PACKET_SENSE, 0, "RAW SENSE", PROP_SENSOR_RAW),
            ],
            PROP_PACKET_CTRL: [
                KWBEasyfireSensor(PROP_PACKET_CTRL, 0, "RAW CTRL", PROP_SENSOR_RAW),
            ],
            PROP_PACKET_SENSE_64: [
                KWBEasyfireSensor(PROP_PACKET_SENSE_64, 0, "RAW SENSE 64", PROP_SENSOR_RAW),
            ],
        }
        for message in load_messages():
            message_id = int(message['message_id'])
            if message_id in self._sensors:
                self._sensors[message_id].append(KWBEasyfireSensor.from_message(message))

    def _debug(self, level, text):
        """Output a debug log text."""
        if (level <= self._debug_level):
            print(text)

    async def _open_connection(self):
        """Open streams lazily on the listener's event loop."""
        if self._mode == PROP_MODE_FILE:
            self._file = open(self._file_path, "r")
            return
        if self._mode == PROP_MODE_TCP:
            connect = asyncio.open_connection(self._ip, self._port)
        elif self._mode == PROP_MODE_SERIAL:
            connect = serial_asyncio_fast.open_serial_connection(
                url=self._serial_device, baudrate=self._serial_speed)
        else:
            raise ValueError("Unsupported input mode")
        self._reader, self._writer = await asyncio.wait_for(
            connect, self._config['connection']['connect_timeout'])
        self._last_valid_packet = time.monotonic()

    async def close(self):
        """Release input resources after stopping/awaiting the listener task.

        Cancelling listening alone preserves the connection and partial frame
        for a subsequent listen on the same event loop. Explicit close resets it.
        """
        writer, self._writer = self._writer, None
        self._reader = None
        self._packet_parser = None
        if self._file is not None:
            self._file.close()
            self._file = None
        if writer is not None:
            writer.close()
            try:
                await writer.wait_closed()
            except OSError:
                pass

    def _reconnect_enabled(self):
        return self._mode == PROP_MODE_TCP and self._config['connection']['reconnect']

    async def _connection_lost(self, error):
        self._debug(PROP_LOGLEVEL_WARN, "TCP disconnected: %s" % error)
        await self.close()
        for sensor in self.get_sensors():
            sensor.value = None

    def _next_retry_delay(self):
        settings = self._config['connection']
        delay = min(self._retry_delay, settings['retry_max'])
        self._retry_delay = min(delay * 2, settings['retry_max'])
        self._debug(PROP_LOGLEVEL_INFO, "TCP reconnect in %g seconds" % delay)
        return delay

    def _stale_remaining(self):
        remaining = (self._config['connection']['stale_timeout']
                     - (time.monotonic() - self._last_valid_packet))
        if remaining <= 0:
            raise TimeoutError("No valid TCP packet within stale_timeout")
        return remaining

    @staticmethod
    def _byte_rot_left(byte, distance):
        """Rotate a byte left by distance bits."""
        return ((byte << distance) | (byte >> (8 - distance))) % 256

    def _add_to_checksum(self, checksum, value):
        """Add a byte to the checksum."""
        checksum = self._byte_rot_left(checksum, 1)
        checksum = checksum + value
        if (checksum > 255):
            checksum = checksum - 255
        self._debug(PROP_LOGLEVEL_TRACE, "C: " + str(checksum) + " V: " + str(value))
        return checksum

    def _record_byte(self, value):
        """Record diagnostics for every input transport."""
        _LOGGER.debug("READ: %s", value)
        self._logdata.append(value)
        if len(self._logdata) > self._logdatalen:
            self._logdata = self._logdata[-self._logdatalen:]
        self._debug(PROP_LOGLEVEL_TRACE, "READ: " + str(value))

    @staticmethod
    def _decode_temp(byte_1, byte_2):
        """Decode a signed short temperature as two bytes to a single number."""
        return decode_temperature(byte_1, byte_2)

    async def _read_packet(self):
        """Read a checksum-valid frame and return its unescaped payload."""
        while True:
            # Buffered input must still yield to deadlines and cancellation.
            await asyncio.sleep(0)
            packet = self._consume_byte(await self._read_async_byte())
            if packet is not None:
                return packet

    def _consume_byte(self, value):
        """Retain partial framing state across reads and listening sessions."""
        if self._packet_parser is None:
            self._packet_parser = self._parse_packet()
            next(self._packet_parser)
        try:
            self._packet_parser.send(value)
        except StopIteration as complete:
            self._packet_parser = None
            self._last_valid_packet = time.monotonic()
            self._retry_delay = self._config['connection']['retry_initial']
            return complete.value
        return None

    def _parse_packet(self):
        """Accept bytes via send(), returning one valid, unescaped frame."""
        pending_length = None
        while True:
            if pending_length is None:
                if (yield) != 2:
                    continue
                length = (yield)
            else:
                length = pending_length
                pending_length = None

            if length == 0:
                continue
            mode = PROP_PACKET_CTRL
            while length == 2:
                mode = PROP_PACKET_SENSE
                length = (yield)
            if length < 5:
                continue

            version = (yield)
            counter = (yield)
            checksum = 2
            for value in (length, version, counter):
                checksum = self._add_to_checksum(checksum, value)

            # Length includes the four header bytes and the checksum, but
            # excludes the extra sense header and payload escape padding.
            packet = bytearray()
            valid = True
            for _ in range(length - 5):
                value = (yield)
                packet.append(value)
                checksum = self._add_to_checksum(checksum, value)
                if value == 2:
                    padding = (yield)
                    if padding != 0:
                        # An unescaped 2 starts a new frame. Reuse its next
                        # byte as the length (or extra sense header), rather
                        # than discarding the beginning of that frame.
                        pending_length = padding
                        valid = False
                        break
            if not valid:
                continue
            if (yield) != checksum:
                continue

            packet_type = "SENSE" if mode == PROP_PACKET_SENSE else "CTRL"
            summary = "\n\nPacket ID %d %s counter=%d length=%d" % (
                version, packet_type, counter, len(packet))
            if self._debug_level >= PROP_LOGLEVEL_DEBUG:
                summary += " payload=" + packet.hex(" ")
            self._debug(PROP_LOGLEVEL_INFO, summary)
            return (mode, version, packet)

    def _decode_sense_packet(self, version, packet):
        """Decode boiler temperatures using the message ID's payload layout."""
        if version not in (PROP_PACKET_SENSE, PROP_PACKET_SENSE_64):
            return
        for sensor in self._sensors[version]:
            sensor.decode(packet)

        for sensor in self._sensors[version]:
            level = (PROP_LOGLEVEL_DEBUG if sensor.sensor_type == PROP_SENSOR_RAW
                     else PROP_LOGLEVEL_INFO)
            self._debug(level, str(sensor))

    def _decode_ctrl_packet(self, version, packet):
        """Decode a control packet into the list of sensors."""
        if version != PROP_PACKET_CTRL:
            return

        for i in range(min(5, len(packet))):
            input_bit = packet[i]
            self._debug(PROP_LOGLEVEL_DEBUG, "Byte " + str(i) + ": " + str((input_bit >> 7) & 1) + str((input_bit >> 6) & 1) + str((input_bit >> 5) & 1) + str((input_bit >> 4) & 1) + str((input_bit >> 3) & 1) + str((input_bit >> 2) & 1) + str((input_bit >> 1) & 1) + str(input_bit & 1))

        for sensor in self._sensors[PROP_PACKET_CTRL]:
            sensor.decode(packet)

        if version == 33:
            self._debug(PROP_LOGLEVEL_INFO, "ID 33 control values:\n" +
                        "\n".join(str(sensor) for sensor in self._sensors[PROP_PACKET_CTRL]))

    def get_sensors(self):
        """Return the list of sensors."""
        return [sensor for sensors in self._sensors.values() for sensor in sensors]

    def __str__(self):
        """Returns an informational text representation of the object."""
        ret = ""

        for sensor in self.get_sensors():
            ret = ret + str(sensor) + "\n"

        return ret

    def _decode_packet(self, mode, version, packet):
        """Decode only configured message IDs with matching frame types."""
        if mode == PROP_PACKET_SENSE and version in (PROP_PACKET_SENSE, PROP_PACKET_SENSE_64):
            self._decode_sense_packet(version, packet)
        elif mode == PROP_PACKET_CTRL and version == PROP_PACKET_CTRL:
            self._decode_ctrl_packet(version, packet)
        if version in self._config.get('decode', []):
            for line in decode_pairs(version, packet):
                self._debug(PROP_LOGLEVEL_INFO, line)

    async def _read_async_byte(self):
        if self._reader is None and self._file is None:
            await self._open_connection()
        if self._mode == PROP_MODE_FILE:
            line = self._file.readline()
            if not line:
                raise EOFError("EOF")
            value = int(line)
            if not 0 <= value <= 255:
                raise ValueError("Capture byte must be between 0 and 255")
        else:
            if self._reconnect_enabled():
                remaining = self._stale_remaining()
                data = await asyncio.wait_for(self._reader.read(1), remaining)
            else:
                data = await self._reader.read(1)
            if not data:
                raise EOFError("Input connection closed")
            value = data[0]
        self._record_byte(value)
        return value

    async def listen_forever(self):
        """Update sensors until EOF or cancellation; allow only one listener.

        Cancellation preserves input and partial framing for resumption on the
        same event loop. Call close() when finished with the connection.
        """
        while True:
            try:
                packet = await self._read_packet()
            except (EOFError, OSError, asyncio.TimeoutError) as error:
                if self._reconnect_enabled():
                    await self._connection_lost(error)
                    await asyncio.sleep(self._next_retry_delay())
                    continue
                await self.close()
                if isinstance(error, EOFError):
                    return
                raise
            self._decode_packet(*packet)

    async def listen_for(self, seconds=1):
        """Update sensors for at most seconds, or until EOF; preserve partial input."""
        if not 0 <= seconds < float('inf'):
            raise ValueError("seconds must be finite and non-negative")
        if seconds == 0:
            return
        listener = asyncio.create_task(self.listen_forever())
        try:
            done, _ = await asyncio.wait({listener}, timeout=seconds)
            if done:
                listener.result()
        finally:
            listener.cancel()
            try:
                await listener
            except asyncio.CancelledError:
                pass


def _print_summary(kwb):
    """Print sensor values in alphabetical order."""
    print("\n\n---\nSUMMARY: " + time.strftime("%Y-%m-%d %H:%M:%S %Z"))
    for sensor in sorted(kwb.get_sensors(), key=lambda sensor: sensor.name.casefold()):
        if sensor.sensor_type != PROP_SENSOR_RAW:
            print(sensor)


async def _listen_with_summaries(kwb, seconds, summary):
    """Keep listening while reporting periodically, until EOF or cancellation."""
    listener = asyncio.create_task(kwb.listen_forever())
    try:
        while True:
            done, _ = await asyncio.wait({listener}, timeout=seconds)
            if done:
                listener.result()
            if summary:
                _print_summary(kwb)
            if done:
                break
    finally:
        listener.cancel()
        try:
            await listener
        except asyncio.CancelledError:
            pass


def main():
    """Main method for debug purposes."""
    parser = argparse.ArgumentParser()
    group_execution = parser.add_argument_group('Execution')
    group_execution.add_argument('--wait', type=float, default=5,
                                 help="Seconds to listen, or summary interval with --forever (default: 5)")
    group_execution.add_argument('--forever', action='store_true', default=False,
                                 help="Listen continuously, printing summaries every --wait seconds")
    group_execution.add_argument('--decode', nargs='*', type=int, default=[], metavar='ID',
                                 help="Also decode two-byte values from offsets 3 and 4 for these message IDs (0-255)")
    group_tcp = parser.add_argument_group('TCP')
    group_tcp.add_argument('--tcp', dest='mode', action='store_const', const=PROP_MODE_TCP, help="Set tcp mode")
    group_tcp.add_argument('--host', dest='hostname', help="Specify hostname", default='')
    group_tcp.add_argument('--port', dest='port', help="Specify port", default=23, type=int)
    group_serial = parser.add_argument_group('Serial')
    group_serial.add_argument('--serial', dest='mode', action='store_const', const=PROP_MODE_SERIAL, help="Set serial mode")
    group_serial.add_argument('--interface', dest='interface', help="Specify interface", default='')
    group_file = parser.add_argument_group('File')
    group_file.add_argument('--file', dest='mode', action='store_const', const=PROP_MODE_FILE, help="Set file mode")
    group_file.add_argument('--name', dest='file', help="Specify file name", default='')
    group_terminal = parser.add_argument_group('Terminal')
    log_levels = {
        'none': PROP_LOGLEVEL_NONE,
        'error': PROP_LOGLEVEL_ERROR,
        'warn': PROP_LOGLEVEL_WARN,
        'warning': PROP_LOGLEVEL_WARN,
        'info': PROP_LOGLEVEL_INFO,
        'debug': PROP_LOGLEVEL_DEBUG,
        'trace': PROP_LOGLEVEL_TRACE,
    }
    group_terminal.add_argument('--log-level', type=str.lower, choices=log_levels,
                                default='info', help="Log verbosity (default: info)")
    group_terminal.add_argument('--log', choices=('true', 'false'), default='true',
                                help="Print individual messages; false overrides --log-level (default: true)")
    group_terminal.add_argument('--summary', action='store_true', default=True,
                                help="Print sensor summaries (default: true)")
    group_terminal.add_argument('--no-summary', dest='summary', action='store_false',
                                help="Disable sensor summaries")
    args = parser.parse_args()
    if not 0 <= args.wait < float('inf'):
        parser.error('--wait must be a finite, non-negative number')
    if args.forever and args.wait == 0:
        parser.error('--wait must be positive with --forever')
    if any(message_id < 0 or message_id > 255 for message_id in args.decode):
        parser.error('--decode IDs must be between 0 and 255')

    kwb = KWBEasyfire(args.mode, args.hostname, args.port, args.interface, SERIAL_SPEED, args.file,
                     _config={'decode': args.decode})
    kwb._debug_level = (PROP_LOGLEVEL_NONE if args.log == 'false'
                        else log_levels[args.log_level])
    async def listen():
        try:
            if args.forever:
                await _listen_with_summaries(kwb, args.wait, args.summary)
            else:
                await kwb.listen_for(seconds=args.wait)
        finally:
            await kwb.close()

    try:
        asyncio.run(listen())
    except KeyboardInterrupt:
        return
    # Print summary
    if not args.forever and args.summary:
        _print_summary(kwb)


if __name__ == "__main__":
    main()
