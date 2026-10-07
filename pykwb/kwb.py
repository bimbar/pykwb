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
import sys
from copy import copy

# Make testing easier for HomeAssistant HACS integration
if __name__ == "__main__" and not __package__:
    # Direct script execution puts pykwb/, not its parent, on sys.path.
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from pykwb.inputs import FileInput, SerialInput, TCPInput
from pykwb.messages import (
    FrameType, Message, SensorDefinition, add_to_checksum, decode_pairs, load_sensor_definitions, parse_message,
    _byte_rot_left as _byte_rot_left,
    PROP_SENSOR_TEMPERATURE, PROP_SENSOR_FLAG, PROP_SENSOR_RAW,
    PROP_SENSOR_NUMBER, PROP_SENSOR_PRESSURE, PROP_SENSOR_DURATION, PROP_SENSOR_SPEED,
)

PROP_MODE_SERIAL = 0
PROP_MODE_TCP = 1
PROP_MODE_FILE = 2

SERIAL_SPEED = 19200

_LOGGER = logging.getLogger(__name__)

class KWBEasyfireSensor:
    """This Class represents as single sensor."""

    def __init__(self, _message_id, _index, _name, _sensor_type, _bit=None,
                 _length=2, _signed=True, _scale=0.1, _units="", _key=""):

        self._message_id = _message_id
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
        self._key = _key or _name.lower().replace(' ', '_')

    @classmethod
    def from_message(cls, sensor_def: SensorDefinition):
        """Create a sensor from one message definition row in messages.csv."""
        if sensor_def['type'] == 'bit':
            sensor_type = PROP_SENSOR_FLAG
        elif sensor_def['type'] == 'int':
            sensor_type = {
                'C': PROP_SENSOR_TEMPERATURE,
                'mbar': PROP_SENSOR_PRESSURE,
                'ms': PROP_SENSOR_DURATION,
                'sec': PROP_SENSOR_DURATION,
                'rpm': PROP_SENSOR_SPEED,
            }.get(sensor_def['units'], PROP_SENSOR_NUMBER)
        else:
            raise ValueError("Unsupported sensor type: " + sensor_def['type'])
        return cls(
            int(sensor_def['message_id']), int(sensor_def['offset']),
            sensor_def['name_en'] or sensor_def['name_de'] or sensor_def['key'],
            sensor_type,
            _bit=int(sensor_def['bit']) if sensor_def['bit'] else None,
            _length=int(sensor_def['length'] or 1),
            _signed=sensor_def['signed'] == '1',
            _scale=float(sensor_def['scale'] or 1),
            _units=sensor_def['units'], _key=sensor_def['key'],
        )

    @property
    def key(self):
        """Return the sensor identity, using its name when no CSV key is set."""
        return self._key

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


class KWBEasyfire:
    """Communicate asynchronously with the KWB Easyfire unit."""

    def __init__(self, _mode, _ip="", _port=0, _serial_device="", _serial_speed=19200,
                 _file_path="", _config=None):
        """Initialize the Object."""

        self._config = dict(_config or {})
        self._config['connection'] = {
            'reconnect': True,
            'connect_timeout': 5,
            'retry_initial': 1,
            'retry_max': 30,
            **self._config.get('connection', {}),
        }

        # Create data input
        settings = self._config['connection']
        if _mode == PROP_MODE_TCP:
            self._input = TCPInput(_ip, _port, settings, _LOGGER)
        elif _mode == PROP_MODE_SERIAL:
            self._input = SerialInput(_serial_device, _serial_speed,
                                      settings['connect_timeout'], _LOGGER)
        elif _mode == PROP_MODE_FILE:
            self._input = FileInput(_file_path, _LOGGER)
        else:
            raise ValueError("Unsupported input mode")

        self._sensors: dict[int, list[KWBEasyfireSensor]] = {}
        self._sensors_by_key: dict[str, KWBEasyfireSensor] = {}

    def load_sensors(self) -> None:
        """Load sensors from the packaged CSV before starting listening."""
        self._sensors = {}
        self._sensors_by_key = {}
        for sensor_def in load_sensor_definitions():
            message_id = int(sensor_def['message_id'])
            if message_id not in self._sensors:
                self._sensors[message_id] = [KWBEasyfireSensor(
                    message_id, 0, "RAW %d" % message_id, PROP_SENSOR_RAW)]
            # Historical reference rows with unknown types have no decoder yet.
            if sensor_def['type'] == '???':
                _LOGGER.debug("Skipping undocumented field for message %d: %s",
                              message_id, sensor_def['name_de'], extra={'terminal': False, 'diagnostic': True})
                continue
            self._sensors[message_id].append(KWBEasyfireSensor.from_message(sensor_def))

        # A sensor key can appear in multiple messages with different field
        # layouts. Keep those definitions in _sensors and create one independent
        # public sensor per key
        for sensors in self._sensors.values():
            for sensor in sensors:
                if sensor.key not in self._sensors_by_key:
                    self._sensors_by_key[sensor.key] = copy(sensor)

    async def close(self):
        """Release input resources after stopping/awaiting the listener task.

        Cancelling listening discards the in-progress frame but keeps the
        connection open. Explicit close releases the connection as well.
        """
        await self._input.close()

    async def _read_message(self) -> Message:
        """Return a validated Message or raise; discard frames before recovery."""
        while True:
            try:
                return await self._collect_frame()
            except (EOFError, OSError, asyncio.TimeoutError) as error:
                # Attempt to recover connection before reraising error
                if not await self._input.recover(error):
                    raise

    async def _collect_frame(self) -> Message:
        """Collect and validate a frame before parsing it.

        Partial input is local to this call and discarded when it is cancelled
        or the connection fails. Invalid frames are skipped until a valid one
        arrives. Return unescaped payload and header metadata.
        """
        pending_length = None
        while True:
            if pending_length is None:
                if (await self._input.read_byte()) != 2:
                    continue
                length = (await self._input.read_byte())
            else:
                length = pending_length
                pending_length = None

            if length == 0:
                continue
            # The first 0x02 was already consumed, including on resynchronization.
            # A single marker is CONTROL; an additional marker identifies SENSE.
            # This metadata does not select a decoder. Tolerate repeated markers.
            frame_type = FrameType.CONTROL
            while length == 2:
                frame_type = FrameType.SENSE
                length = (await self._input.read_byte())
            if length < 5:
                continue

            version = (await self._input.read_byte())
            counter = (await self._input.read_byte())
            checksum = 2
            for value in (length, version, counter):
                checksum = add_to_checksum(checksum, value)
                _LOGGER.debug("C: %s V: %s", checksum, value)

            # Length includes the four header bytes and the checksum, but
            # excludes the additional header marker and payload escape padding.
            payload = bytearray()
            valid = True
            for _ in range(length - 5):
                value = (await self._input.read_byte())
                payload.append(value)
                checksum = add_to_checksum(checksum, value)
                _LOGGER.debug("C: %s V: %s", checksum, value)
                if value == 2:
                    padding = (await self._input.read_byte())
                    if padding != 0:
                        # An unescaped 2 starts a new frame. Reuse its next
                        # byte as the length (or additional header marker), rather
                        # than discarding the beginning of that frame.
                        pending_length = padding
                        valid = False
                        break
            if not valid:
                continue
            if (await self._input.read_byte()) != checksum:
                continue

            return Message(version, counter, bytes(payload), frame_type)

    def _log_message(self, message: Message) -> None:
        """Log the completed message and its results using the existing format."""
        summary = "\n\nMessage ID %d frame_type=%s counter=%d length=%d" % (
            message.message_id, message.frame_type.name, message.counter, len(message.payload))
        if _LOGGER.isEnabledFor(logging.DEBUG):
            summary += " payload=" + message.payload.hex(" ")
        _LOGGER.info(summary)
        for sensor in self._sensors.get(message.message_id, []):
            level = (logging.DEBUG if sensor.sensor_type == PROP_SENSOR_RAW
                     else logging.INFO)
            _LOGGER.log(level, "%s", sensor)
        if message.message_id in self._config.get('decode', []):
            for line in decode_pairs(message.message_id, message.payload):
                _LOGGER.info("%s", line)

    def __str__(self):
        """Returns an informational text representation of the object."""
        ret = ""

        for sensor in self.get_sensors():
            ret = ret + str(sensor) + "\n"

        return ret

    def _update_sensors(self, message: Message) -> None:
        """Apply the already parsed values to this message's sensors."""
        for sensor, value in zip(self._sensors.get(message.message_id, []), message.values):
            sensor.value = value
            self._sensors_by_key[sensor.key].value = value

    ## Public API

    def get_sensors(self):
        """Return one sensor per key, holding its latest received value."""
        return list(self._sensors_by_key.values())

    async def listen_forever(self) -> None:
        """Update sensors and log messages until EOF; allow only one listener.

        Await directly or run as a task. Cancellation discards partial frames
        but retains sensor readings and the connection; call close() to release it.
        """

        # Ensure sensors have been loaded
        if not self._sensors: self.load_sensors()

        while True:
            message = await self._read_message()
            parse_message(self._sensors, message)
            self._update_sensors(message)
            self._log_message(message)

    async def listen_for(self, seconds=1):
        """Update sensors for at most seconds, or until EOF; discard partial frames."""
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


def main():
    """Main method for debug purposes."""
    parser = argparse.ArgumentParser()
    group_execution = parser.add_argument_group('Execution')
    group_execution.add_argument('--wait', type=float, default=5,
                                 help="Seconds to listen; ignored with --forever (default: 5)")
    group_execution.add_argument('--forever', action='store_true', default=False,
                                 help="Listen continuously until input closes or interrupted")
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
        'none': logging.CRITICAL + 1,
        'error': logging.ERROR,
        'warn': logging.WARNING,
        'warning': logging.WARNING,
        'info': logging.INFO,
        'debug': logging.DEBUG,
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

    # Validate inputs
    if not 0 <= args.wait < float('inf'):
        parser.error('--wait must be a finite, non-negative number')
    
    # Construct KWBEasyfire connector
    kwb = KWBEasyfire(args.mode, args.hostname, args.port, args.interface, SERIAL_SPEED,
                     args.file, _config={'decode': args.decode})
    kwb.load_sensors()

    # Configure logging
    handler = logging.StreamHandler(sys.stdout)
    level = logging.CRITICAL + 1 if args.log == 'false' else log_levels[args.log_level]
    handler.setLevel(level)
    handler.setFormatter(logging.Formatter('%(message)s'))
    handler.addFilter(lambda record: getattr(record, 'terminal', True))
    previous_level, previous_propagate = _LOGGER.level, _LOGGER.propagate
    _LOGGER.setLevel(level)
    _LOGGER.propagate = False
    _LOGGER.addHandler(handler)

    async def listen():
        try:
            if args.forever:
                await kwb.listen_forever()
            else:
                await kwb.listen_for(seconds=args.wait)
        finally:
            await kwb.close()

    try:
        asyncio.run(listen())
    except KeyboardInterrupt:
        pass
    finally:
        _LOGGER.removeHandler(handler)
        handler.close()
        _LOGGER.setLevel(previous_level)
        _LOGGER.propagate = previous_propagate
        
    # Summarize if requested
    if args.summary:
        _print_summary(kwb)


if __name__ == "__main__":
    main()
