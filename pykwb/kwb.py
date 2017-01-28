"""
Support for KWB Easyfire central heating units.
"""

import logging
import os
import sys
import serial
import socket
import struct


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

TCP_IP = "10.0.2.30"
TCP_PORT = 23

SERIAL_INTERFACE = "/dev/ttyUSB0"
SERIAL_SPEED = 19200

_LOGGER = logging.getLogger(__name__)


class KWBSensor:
    
    def __init__(self, _packet, _index, _name, _sensor_type):
        
        self._packet = _packet
        self._index = _index
        self._name = _name
        self._sensor_type = _sensor_type
        self._value = 0

    @property
    def index(self):
        return self._index
    
    @property
    def name(self):
        return self._name
    
    @property
    def sensor_type(self):
        return self._sensor_type
    
    @property
    def unit_of_measurement(self):
        if (self._sensor_type == PROP_SENSOR_TEMPERATURE):
            return "°C"
        else:
            return ""
    
    @property
    def value(self):
        return self._value
    
    @value.setter
    def value(self, _value):
        self._value=_value

    def __str__(self):
        return self.name + ": I: " + str(self.index) + " T: " + str(self.sensor_type) + "(" + str(self.unit_of_measurement) + ") V: " + str(self.value)



class KWBEasyfire:

    def __init__(self, _mode, _ip="", _port=0, _serial_device="", _serial_speed=19200):
        """Initialize the Object."""
        
        self._mode = _mode
        self._ip = _ip
        self._port = _port
        self._serial_device = _serial_device
        self._serial_speed = _serial_speed
        self._logdatalen = 1024
        self._logdata = []
        
        self._sense_sensor = []

        self._sense_sensor.append(KWBSensor(PROP_PACKET_SENSE, 0, "Flow", PROP_SENSOR_TEMPERATURE))
        self._sense_sensor.append(KWBSensor(PROP_PACKET_SENSE, 1, "Return", PROP_SENSOR_TEMPERATURE))
        self._sense_sensor.append(KWBSensor(PROP_PACKET_SENSE, 2, "Boiler 0", PROP_SENSOR_TEMPERATURE))
        self._sense_sensor.append(KWBSensor(PROP_PACKET_SENSE, 3, "Furnace", PROP_SENSOR_TEMPERATURE))
        self._sense_sensor.append(KWBSensor(PROP_PACKET_SENSE, 4, "Buffer Tank 2", PROP_SENSOR_TEMPERATURE))
        self._sense_sensor.append(KWBSensor(PROP_PACKET_SENSE, 5, "Buffer Tank 1", PROP_SENSOR_TEMPERATURE))
        self._sense_sensor.append(KWBSensor(PROP_PACKET_SENSE, 6, "Outside", PROP_SENSOR_TEMPERATURE))
        self._sense_sensor.append(KWBSensor(PROP_PACKET_SENSE, 7, "Exhaust", PROP_SENSOR_TEMPERATURE))
        self._sense_sensor.append(KWBSensor(PROP_PACKET_SENSE, 8, "?", PROP_SENSOR_TEMPERATURE))
        self._sense_sensor.append(KWBSensor(PROP_PACKET_SENSE, 12, "Stoker Channel", PROP_SENSOR_TEMPERATURE))

        self._ctrl_sensor = []

        self._ctrl_sensor.append(KWBSensor(PROP_PACKET_CTRL, 17, "Return Mixer", PROP_SENSOR_FLAG))
        self._ctrl_sensor.append(KWBSensor(PROP_PACKET_CTRL, 25, "?Resupply", PROP_SENSOR_FLAG))
        
        self._open_connection()
    
    def __del__(self):
        print(self._logdata)
        self._close_connection()
    
    def _open_connection(self):
        if (self._mode == PROP_MODE_SERIAL):
            self._serial = serial.Serial(self._serial_device, self._serial_speed)
        elif (self._mode == PROP_MODE_TCP):
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._socket.connect((self._ip, self._port))
    
    def _close_connection(self):
        if (self._mode == PROP_MODE_SERIAL):
            self._serial.close()
        elif (self._mode == PROP_MODE_TCP):
            self._socket.close()

    def _byte_rot_left(self, byte, distance):
        return ((byte << distance) | (byte >> (8 - distance))) % 256
    
    def _add_to_checksum(self, checksum, value):
        
        checksum = self._byte_rot_left(checksum, 1)
        checksum = checksum + value
        if (checksum > 255):
            checksum = checksum - 255
#        print("C: " + str(checksum) + " V: " + str(value))
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
        if (len(self._logdata)>self._logdatalen):
            self._logdata = self._logdata[len(self._logdata) - self._logdatalen:]

 #       print("READ: " + str(ord(to_return)))
        
        return to_return
    
    def _read_ord_byte(self):
        
        return ord(self._read_byte())

    def _sense_packet_to_data(self, packet):
        data = bytearray(0)
        last = 0
        i = 1
        while (i < len(packet)):
            if not (last == 2 and packet[i] == 0):
                data.append(packet[i])
            last = packet[i]
            i += 1


        return data

    def _decode_temp(self, b1, b2):
        temp = (b1 << 8) + b2
        if (temp > 32767):
            temp = temp - 65536
        temp = temp / 10
        return temp

    def _read_packet(self):

        status = STATUS_WAITING
        mode = 0
        checksum = 0
        checksum_calculated = 0
        length = 0
        i = 0
        cnt = 0
        packet = bytearray(0)
        do_read = 1
        
        while (status != STATUS_PACKET_DONE):
            
            read = self._read_ord_byte()
            if (status != STATUS_CTRL_CHECKSUM and status != STATUS_SENSE_CHECKSUM):
                checksum_calculated = self._add_to_checksum(checksum_calculated, read)
#            print("R: " + str(read))
#            print("S: " + str(status))
            
            
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
                length = read + 1
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
                if ( i == length):
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
                if ( i == length):
                    status = STATUS_CTRL_CHECKSUM
            elif (status == STATUS_CTRL_CHECKSUM):
                checksum = read
                mode = PROP_PACKET_CTRL
                status = STATUS_PACKET_DONE
            else:
                status = STATUS_WAITING
        
        print("MODE: " + str(mode) + " Checksum: " + str(checksum) + " / " + str(checksum_calculated) + " Count: " + str(cnt) + " Length: " + str(len(packet)))
        print("Packet: " + str(packet))
        
        return (mode, packet)
    
    def _decode_sense_packet(self, packet):
        data = self._sense_packet_to_data(packet)
        
        offset = 4
        i = 0

        datalen = len(data) - offset - 6
        tempCount = int(datalen / 2)
        temp=[]
        
        for i in range(tempCount):
            n = i * 2 + offset
            temp.append(self._decode_temp(data[n], data[n+1]))
        
        print("T: " + str(temp))
        
        for sensor in self._sense_sensor:
            sensor.value = temp[sensor.index]
        
        print(str(self))
    
    def _decode_ctrl_packet(self, packet):
        
        for i in range(5):
            b = packet[i]
            print("Byte " + str(i) + ": " + str((b>>7)&1) + str((b>>6)&1) + str((b>>5)&1) + str((b>>4)&1) + str((b>>3)&1) + str((b>>2)&1) + str((b>>1)&1) + str(b&1))

        for sensor in self._ctrl_sensor:
            sensor.value = (packet[sensor.index // 8] >> (sensor.index % 8)) & 1

        
    def __str__(self):
        ret = ""
        
        for sensor in self._sense_sensor:
            ret = ret + str(sensor) + "\n"
        
        for sensor in self._ctrl_sensor:
            ret = ret + str(sensor) + "\n"
        
        return ret
        
    
    def do_it(self):
        while (1):
            (mode, packet) = self._read_packet()
            if (mode == PROP_PACKET_SENSE):
                print("SENSE")
                self._decode_sense_packet(packet)
            elif (mode == PROP_PACKET_CTRL):
                print("CTRL")
                self._decode_ctrl_packet(packet)
            
        
kwb = KWBEasyfire(PROP_MODE_TCP, "10.0.2.30", 23)
kwb.do_it()


