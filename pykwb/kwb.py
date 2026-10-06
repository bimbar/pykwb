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
from copy import copy
import serial_asyncio_fast

# Make testing easier for HomeAssistant HACS integration
if __name__ == "__main__" and not __package__:
    # Direct script execution puts pykwb/, not its parent, on sys.path.
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from pykwb.messages import (
    FrameType, Message, add_to_checksum, load_messages, parse_message,
    _byte_rot_left as _byte_rot_left,
    PROP_SENSOR_TEMPERATURE, PROP_SENSOR_FLAG, PROP_SENSOR_RAW,
    PROP_SENSOR_NUMBER, PROP_SENSOR_PRESSURE, PROP_SENSOR_DURATION, PROP_SENSOR_SPEED,
)

PROP_LOGLEVEL_TRACE = 5
PROP_LOGLEVEL_DEBUG = 4
PROP_LOGLEVEL_INFO = 3
PROP_LOGLEVEL_WARN = 2
PROP_LOGLEVEL_ERROR = 1
PROP_LOGLEVEL_NONE = 0

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
    def from_message(cls, message):
        """Create a sensor from one message definition in messages.csv."""
        if message['type'] == 'bit':
            sensor_type = PROP_SENSOR_FLAG
        elif message['type'] == 'int':
            sensor_type = {
                'C': PROP_SENSOR_TEMPERATURE,
                'mbar': PROP_SENSOR_PRESSURE,
                'ms': PROP_SENSOR_DURATION,
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
            'retry_initial': 1,
            'retry_max': 30,
            **self._config.get('connection', {}),
        }
        self._debug_level = PROP_LOGLEVEL_INFO
        self._reader = None
        self._writer = None
        self._file = None
        self._retry_delay = self._config['connection']['retry_initial']
        for key in ('connect_timeout', 'retry_initial', 'retry_max'):
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

        self._sensors: dict[int, list[KWBEasyfireSensor]] = {}
        self._sensors_by_key: dict[str, KWBEasyfireSensor] = {}
        for message in load_messages():
            message_id = int(message['message_id'])
            if message_id not in self._sensors:
                self._sensors[message_id] = [KWBEasyfireSensor(
                    message_id, 0, "RAW %d" % message_id, PROP_SENSOR_RAW)]
            # Historical reference rows with unknown types have no decoder yet.
            if message['type'] == '???':
                _LOGGER.debug("Skipping undocumented field for message %d: %s",
                              message_id, message['name_de'])
                continue
            self._sensors[message_id].append(KWBEasyfireSensor.from_message(message))

        for sensors in self._sensors.values():
            for sensor in sensors:
                # Keep final state separate from each message's field layout.
                if sensor.key not in self._sensors_by_key:
                    self._sensors_by_key[sensor.key] = copy(sensor)

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
        self._retry_delay = self._config['connection']['retry_initial']

    async def close(self):
        """Release input resources after stopping/awaiting the listener task.

        Cancelling listening discards the in-progress frame but keeps the
        connection open. Explicit close releases the connection as well.
        """
        writer, self._writer = self._writer, None
        self._reader = None
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

    def _next_retry_delay(self):
        settings = self._config['connection']
        delay = min(self._retry_delay, settings['retry_max'])
        self._retry_delay = min(delay * 2, settings['retry_max'])
        self._debug(PROP_LOGLEVEL_INFO, "TCP reconnect in %g seconds" % delay)
        return delay

    def _record_byte(self, value):
        """Record diagnostics for every input transport."""
        _LOGGER.debug("READ: %s", value)
        self._logdata.append(value)
        if len(self._logdata) > self._logdatalen:
            self._logdata = self._logdata[-self._logdatalen:]
        self._debug(PROP_LOGLEVEL_TRACE, "READ: " + str(value))

    async def _read_message(self) -> Message:
        """Return a validated Message or raise; retry TCP failures when enabled."""
        while True:
            try:
                return await self._collect_frame()
            except (EOFError, OSError, asyncio.TimeoutError) as error:
                if self._reconnect_enabled():
                    await self._connection_lost(error)
                    await asyncio.sleep(self._next_retry_delay())
                    continue
                await self.close()
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
                if (await self._read_async_byte()) != 2:
                    continue
                length = (await self._read_async_byte())
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
                length = (await self._read_async_byte())
            if length < 5:
                continue

            version = (await self._read_async_byte())
            counter = (await self._read_async_byte())
            checksum = 2
            for value in (length, version, counter):
                checksum = add_to_checksum(checksum, value)
                self._debug(PROP_LOGLEVEL_TRACE, "C: " + str(checksum) + " V: " + str(value))

            # Length includes the four header bytes and the checksum, but
            # excludes the additional header marker and payload escape padding.
            payload = bytearray()
            valid = True
            for _ in range(length - 5):
                value = (await self._read_async_byte())
                payload.append(value)
                checksum = add_to_checksum(checksum, value)
                self._debug(PROP_LOGLEVEL_TRACE, "C: " + str(checksum) + " V: " + str(value))
                if value == 2:
                    padding = (await self._read_async_byte())
                    if padding != 0:
                        # An unescaped 2 starts a new frame. Reuse its next
                        # byte as the length (or additional header marker), rather
                        # than discarding the beginning of that frame.
                        pending_length = padding
                        valid = False
                        break
            if not valid:
                continue
            if (await self._read_async_byte()) != checksum:
                continue

            return Message(version, counter, bytes(payload), frame_type)

    def _log_message(self, message: Message) -> None:
        """Log the completed message and its results using the existing format."""
        summary = "\n\nPacket ID %d frame_type=%s counter=%d length=%d" % (
            message.message_id, message.frame_type.name, message.counter, len(message.payload))
        if self._debug_level >= PROP_LOGLEVEL_DEBUG:
            summary += " payload=" + message.payload.hex(" ")
        self._debug(PROP_LOGLEVEL_INFO, summary)
        for sensor in self._sensors.get(message.message_id, []):
            level = (PROP_LOGLEVEL_DEBUG if sensor.sensor_type == PROP_SENSOR_RAW
                     else PROP_LOGLEVEL_INFO)
            self._debug(level, str(sensor))

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

    async def _read_async_byte(self):
        # Ready streams and capture files must still allow cancellation.
        await asyncio.sleep(0)
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
            data = await self._reader.read(1)
            if not data:
                raise EOFError("Input connection closed")
            value = data[0]
        self._record_byte(value)
        return value

    ## Public API

    def get_sensors(self):
        """Return one sensor per key, holding its latest received value."""
        return list(self._sensors_by_key.values())

    async def listen_forever(self) -> None:
        """Update sensors and log messages until EOF; allow only one listener.

        Await directly or run as a task. Cancellation discards partial frames
        but retains sensor readings and the connection; call close() to release it.
        """
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
    kwb = KWBEasyfire(args.mode, args.hostname, args.port, args.interface, SERIAL_SPEED, args.file)
    kwb._debug_level = (PROP_LOGLEVEL_NONE if args.log == 'false'
                        else log_levels[args.log_level])
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
    # Summarize if requested
    if args.summary:
        _print_summary(kwb)


if __name__ == "__main__":
    main()
