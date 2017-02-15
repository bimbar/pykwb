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

import logging
import socket
import time
import threading
import serial


PROP_LOGLEVEL_TRACE = 5
PROP_LOGLEVEL_DEBUG = 4
PROP_LOGLEVEL_INFO = 3
PROP_LOGLEVEL_WARN = 2
PROP_LOGLEVEL_ERROR = 1
PROP_LOGLEVEL_NONE = 0

PROP_MODE_SERIAL = 0
PROP_MODE_TCP = 1

STATUS_WAITING = 0
STATUS_PRE_1 = 1
STATUS_SENSE_PRE_2 = 2
STATUS_SENSE_PRE_3 = 3
STATUS_SENSE_PRE_LENGTH = 6
STATUS_SENSE_DATA = 8
STATUS_SENSE_CHECKSUM = 9
STATUS_CTRL_PRE_2 = 10
STATUS_CTRL_PRE_3 = 11
STATUS_CTRL_DATA = 12
STATUS_CTRL_CHECKSUM = 19
STATUS_PACKET_DONE = 255

PROP_PACKET_SENSE = 0
PROP_PACKET_CTRL = 1

PROP_SENSOR_TEMPERATURE = 0
PROP_SENSOR_FLAG = 1
PROP_SENSOR_RAW = 2

TCP_IP = "127.0.0.1"
TCP_PORT = 23

SERIAL_INTERFACE = "/dev/ttyUSB0"
SERIAL_SPEED = 19200

_LOGGER = logging.getLogger(__name__)


class KWBEasyfireSensor:
    """This Class represents as single sensor."""

    def __init__(self, _packet, _index, _name, _sensor_type):

        self._packet = _packet
        self._index = _index
        self._name = _name
        self._sensor_type = _sensor_type
        self._value = None

    @property
    def index(self):
        """Returns the offset from the start of the packet."""
        return self._index

    @property
    def name(self):
        """Returns the name of the sensor."""
        return self._name

    @property
    def sensor_type(self):
        """Returns the type of the sensor. It can be CTRL or SENSE."""
        return self._sensor_type

    @property
    def unit_of_measurement(self):
        """Returns the unit of measurement of the sensor. It can be °C or empty."""
        if (self._sensor_type == PROP_SENSOR_TEMPERATURE):
            return "°C"
        else:
            return ""

    @property
    def value(self):
        """Returns the value of the sensor. Unit is unit_of_measurement."""
        return self._value

    @value.setter
    def value(self, _value):
        """Sets the value of the sensor. Unit is unit_of_measurement."""
        self._value = _value

    def __str__(self):
        """Returns an informational text representation of the sensor."""
        return self.name + ": I: " + str(self.index) + " T: " + str(self.sensor_type) + "(" + str(self.unit_of_measurement) + ") V: " + str(self.value)


# pylint: disable=too-many-instance-attributes
class KWBEasyfire:
    """Communicats with the KWB Easyfire unit."""

    def __init__(self, _mode, _ip="", _port=0, _serial_device="", _serial_speed=19200):
        """Initialize the Object."""

        self._debug_level = PROP_LOGLEVEL_NONE
        self._run_thread = True

        self._mode = _mode
        self._ip = _ip
        self._port = _port
        self._serial_device = _serial_device
        self._serial_speed = _serial_speed
        self._logdatalen = 1024
        self._logdata = []

        self._sense_sensor = []

        self._sense_sensor.append(KWBEasyfireSensor(PROP_PACKET_SENSE, 0, "RAW SENSE", PROP_SENSOR_RAW))
        self._sense_sensor.append(KWBEasyfireSensor(PROP_PACKET_SENSE, 0, "Supply", PROP_SENSOR_TEMPERATURE))
        self._sense_sensor.append(KWBEasyfireSensor(PROP_PACKET_SENSE, 1, "Return", PROP_SENSOR_TEMPERATURE))
        self._sense_sensor.append(KWBEasyfireSensor(PROP_PACKET_SENSE, 2, "Boiler 0", PROP_SENSOR_TEMPERATURE))
        self._sense_sensor.append(KWBEasyfireSensor(PROP_PACKET_SENSE, 3, "Furnace", PROP_SENSOR_TEMPERATURE))
        self._sense_sensor.append(KWBEasyfireSensor(PROP_PACKET_SENSE, 4, "Buffer Tank 2", PROP_SENSOR_TEMPERATURE))
        self._sense_sensor.append(KWBEasyfireSensor(PROP_PACKET_SENSE, 5, "Buffer Tank 1", PROP_SENSOR_TEMPERATURE))
        self._sense_sensor.append(KWBEasyfireSensor(PROP_PACKET_SENSE, 6, "Outside", PROP_SENSOR_TEMPERATURE))
        self._sense_sensor.append(KWBEasyfireSensor(PROP_PACKET_SENSE, 7, "Exhaust", PROP_SENSOR_TEMPERATURE))
        self._sense_sensor.append(KWBEasyfireSensor(PROP_PACKET_SENSE, 8, "Unknown", PROP_SENSOR_TEMPERATURE))
        self._sense_sensor.append(KWBEasyfireSensor(PROP_PACKET_SENSE, 12, "Stoker Channel", PROP_SENSOR_TEMPERATURE))

        self._ctrl_sensor = []

        self._ctrl_sensor.append(KWBEasyfireSensor(PROP_PACKET_CTRL, 0, "RAW CTRL", PROP_SENSOR_RAW))
        self._ctrl_sensor.append(KWBEasyfireSensor(PROP_PACKET_CTRL, 17, "Return Mixer", PROP_SENSOR_FLAG))
        self._ctrl_sensor.append(KWBEasyfireSensor(PROP_PACKET_CTRL, 25, "Unknown Resupply", PROP_SENSOR_FLAG))

        self._thread = threading.Thread(target=self.run)

        self._open_connection()

    def _debug(self, level, text):
        """Output a debug log text."""
        if (level <= self._debug_level):
            print(text)

    def __del__(self):
        """Destruct the object."""
        self._debug(PROP_LOGLEVEL_DEBUG, self._logdata)
        self._close_connection()

    def _open_connection(self):
        """Open a connection to the easyfire unit."""
        if (self._mode == PROP_MODE_SERIAL):
            self._serial = serial.Serial(self._serial_device, self._serial_speed)
        elif (self._mode == PROP_MODE_TCP):
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._socket.connect((self._ip, self._port))

    def _close_connection(self):
        """Close the connection to the easyfire unit."""
        if (self._mode == PROP_MODE_SERIAL):
            self._serial.close()
        elif (self._mode == PROP_MODE_TCP):
            self._socket.close()

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

    def _read_byte(self):
        """Read a byte from input."""

        to_return = ""
        if (self._mode == PROP_MODE_SERIAL):
            to_return = self._serial.read(1)
        elif (self._mode == PROP_MODE_TCP):
            to_return = self._socket.recv(1)

        _LOGGER.debug("READ: " + str(ord(to_return)))
        self._logdata.append(ord(to_return))
        if (len(self._logdata) > self._logdatalen):
            self._logdata = self._logdata[len(self._logdata) - self._logdatalen:]

        self._debug(PROP_LOGLEVEL_TRACE, "READ: " + str(ord(to_return)))

        return to_return

    def _read_ord_byte(self):
        """Read a byte as number from the input."""
        return ord(self._read_byte())

    @staticmethod
    def _sense_packet_to_data(packet):
        """Remove the escape pad bytes from a sense packet (\2\0 -> \2)."""
        data = bytearray(0)
        last = 0
        i = 1
        while (i < len(packet)):
            if not (last == 2 and packet[i] == 0):
                data.append(packet[i])
            last = packet[i]
            i += 1

        return data

    @staticmethod
    def _decode_temp(byte_1, byte_2):
        """Decode a signed short temperature as two bytes to a single number."""
        temp = (byte_1 << 8) + byte_2
        if (temp > 32767):
            temp = temp - 65536
        temp = temp / 10
        return temp

    # pylint: disable=too-many-branches, too-many-statements
    def _read_packet(self):
        """Read a packet from the input."""

        status = STATUS_WAITING
        mode = 0
        checksum = 0
        checksum_calculated = 0
        length = 0
        i = 0
        cnt = 0
        packet = bytearray(0)

        while (status != STATUS_PACKET_DONE):

            read = self._read_ord_byte()
            if (status != STATUS_CTRL_CHECKSUM and status != STATUS_SENSE_CHECKSUM):
                checksum_calculated = self._add_to_checksum(checksum_calculated, read)
            self._debug(PROP_LOGLEVEL_TRACE, "R: " + str(read))
            self._debug(PROP_LOGLEVEL_TRACE, "S: " + str(status))

            if (status == STATUS_WAITING):
                if (read == 2):
                    status = STATUS_PRE_1
                    checksum_calculated = read
                else:
                    status = STATUS_WAITING
            elif (status == STATUS_PRE_1):
                checksum = 0
                if (read == 2):
                    status = STATUS_SENSE_PRE_2
                    checksum_calculated = read
                elif (read == 21):
                    status = STATUS_CTRL_PRE_2
                else:
                    status = STATUS_WAITING
            elif (status == STATUS_SENSE_PRE_2):
                length = read
                status = STATUS_SENSE_PRE_LENGTH
            elif (status == STATUS_SENSE_PRE_LENGTH):
                if (read == 16):
                    status = STATUS_SENSE_PRE_3
                else:
                    status = STATUS_WAITING
            elif (status == STATUS_SENSE_PRE_3):
                cnt = read
                i = 0
                status = STATUS_SENSE_DATA
            elif (status == STATUS_SENSE_DATA):
                packet.append(read)
                i = i + 1
                if (i == length):
                    status = STATUS_SENSE_CHECKSUM
            elif (status == STATUS_SENSE_CHECKSUM):
                checksum = read
                mode = PROP_PACKET_SENSE
                status = STATUS_PACKET_DONE
            elif (status == STATUS_CTRL_PRE_2):
                if (read == 17):
                    status = STATUS_CTRL_PRE_3
                else:
                    status = STATUS_WAITING
            elif (status == STATUS_CTRL_PRE_3):
                cnt = read
                i = 0
                length = 16
                status = STATUS_CTRL_DATA
            elif (status == STATUS_CTRL_DATA):
                packet.append(read)
                i = i + 1
                if (i == length):
                    status = STATUS_CTRL_CHECKSUM
            elif (status == STATUS_CTRL_CHECKSUM):
                checksum = read
                mode = PROP_PACKET_CTRL
                status = STATUS_PACKET_DONE
            else:
                status = STATUS_WAITING

        self._debug(PROP_LOGLEVEL_DEBUG, "MODE: " + str(mode) + " Checksum: " + str(checksum) + " / " + str(checksum_calculated) + " Count: " + str(cnt) + " Length: " + str(len(packet)))
        self._debug(PROP_LOGLEVEL_TRACE, "Packet: " + str(packet))

        return (mode, packet)

    def _decode_sense_packet(self, packet):
        """Decode a sense packet into the list of sensors."""

        data = self._sense_packet_to_data(packet)

        offset = 4
        i = 0

        datalen = len(data) - offset - 6
        temp_count = int(datalen / 2)
        temp = []

        for i in range(temp_count):
            temp_index = i * 2 + offset
            temp.append(self._decode_temp(data[temp_index], data[temp_index + 1]))

        self._debug(PROP_LOGLEVEL_DEBUG, "T: " + str(temp))

        for sensor in self._sense_sensor:
            if (sensor.sensor_type == PROP_SENSOR_TEMPERATURE):
                sensor.value = temp[sensor.index]
            elif (sensor.sensor_type == PROP_SENSOR_RAW):
                sensor.value = packet

        self._debug(PROP_LOGLEVEL_DEBUG, str(self))

    def _decode_ctrl_packet(self, packet):
        """Decode a control packet into the list of sensors."""

        for i in range(5):
            input_bit = packet[i]
            self._debug(PROP_LOGLEVEL_DEBUG, "Byte " + str(i) + ": " + str((input_bit >> 7) & 1) + str((input_bit >> 6) & 1) + str((input_bit >> 5) & 1) + str((input_bit >> 4) & 1) + str((input_bit >> 3) & 1) + str((input_bit >> 2) & 1) + str((input_bit >> 1) & 1) + str(input_bit & 1))

        for sensor in self._ctrl_sensor:
            if (sensor.sensor_type == PROP_SENSOR_FLAG):
                sensor.value = (packet[sensor.index // 8] >> (sensor.index % 8)) & 1
            elif (sensor.sensor_type == PROP_SENSOR_RAW):
                sensor.value = packet

    def get_sensors(self):
        """Return the list of sensors."""
        return self._sense_sensor + self._ctrl_sensor

    def __str__(self):
        """Returns an informational text representation of the object."""
        ret = ""

        for sensor in self._sense_sensor:
            ret = ret + str(sensor) + "\n"

        for sensor in self._ctrl_sensor:
            ret = ret + str(sensor) + "\n"

        return ret

    def run(self):
        """Main thread that reads from input and populates the sensors."""
        while (self._run_thread):
            (mode, packet) = self._read_packet()
            if (mode == PROP_PACKET_SENSE):
                self._decode_sense_packet(packet)
            elif (mode == PROP_PACKET_CTRL):
                self._decode_ctrl_packet(packet)

    def run_thread(self):
        """Run the main thread."""
        self._run_thread = True
        self._thread.setDaemon(True)
        self._thread.start()

    def stop_thread(self):
        """Stop the main thread."""
        self._run_thread = False


def main():
    """Main method for debug purposes."""
    kwb = KWBEasyfire(PROP_MODE_TCP, "10.0.2.30", 23)
    kwb.run_thread()
    time.sleep(5)
    kwb.stop_thread()


if __name__ == "__main__":
    main()
