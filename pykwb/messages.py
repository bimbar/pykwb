# -*- coding: utf-8 -*-
from __future__ import annotations

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

"""Load KWB Comfort 3 definitions and parse message payloads."""

import csv
from dataclasses import dataclass
from enum import Enum
from importlib.resources import files
from pathlib import Path
from typing import TYPE_CHECKING, Optional, Union

if TYPE_CHECKING:
    from pykwb.kwb import KWBEasyfireSensor


PROP_SENSOR_TEMPERATURE = 0
PROP_SENSOR_FLAG = 1
PROP_SENSOR_RAW = 2
PROP_SENSOR_NUMBER = 3
PROP_SENSOR_PRESSURE = 4
PROP_SENSOR_DURATION = 5
PROP_SENSOR_SPEED = 6


SensorValue = Union[bytes, int, float, None]
SensorDefinition = dict[str, str]
SensorDefinitions = list[SensorDefinition]


class FrameType(Enum):
    """Wire header forms; payload decoding is determined by message ID."""

    CONTROL = "control"
    SENSE = "sense"


@dataclass
class Message:
    """A validated frame and its parsed values in CSV sensor order."""

    message_id: int
    counter: int
    payload: bytes
    frame_type: FrameType
    values: tuple[SensorValue, ...] = ()


def _byte_rot_left(byte, distance):
    """Rotate a byte left by distance bits."""
    return ((byte << distance) | (byte >> (8 - distance))) % 256


def add_to_checksum(checksum: int, value: int) -> int:
    """Add a byte to the checksum."""
    checksum = _byte_rot_left(checksum, 1)
    checksum = checksum + value
    if checksum > 255:
        checksum = checksum - 255
    return checksum


def load_sensor_definitions(file_path: Optional[str] = None) -> SensorDefinitions:
    """Load string-valued definitions from a CSV path or the packaged CSV."""
    resource = (Path(file_path) if file_path is not None
                else files("pykwb").joinpath("messages.csv"))
    with resource.open("r", encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def parse_message(sensors: dict[int, list[KWBEasyfireSensor]], message: Message) -> Message:
    """Extract and interpret CSV-defined fields without updating sensors."""
    values: list[SensorValue] = []
    for sensor in sensors.get(message.message_id, []):
        offset = sensor.index
        length = 1 if sensor.sensor_type == PROP_SENSOR_FLAG else sensor._length
        if sensor.sensor_type == PROP_SENSOR_RAW:
            # Keep the full unescaped payload for raw diagnostics.
            values.append(message.payload)
        elif offset is None or offset < 0 or offset + length > len(message.payload):
            # Unmapped or missing fields have no value.
            values.append(None)
        elif sensor.sensor_type == PROP_SENSOR_FLAG:
            # Extract the configured bit, or return None for an invalid bit position.
            bit = sensor.bit
            values.append((message.payload[offset] >> bit) & 1
                          if bit is not None and 0 <= bit < 8 else None)
        else:
            # Read a big-endian integer using the field's configured signedness.
            value = int.from_bytes(message.payload[offset:offset + length], 'big',
                                   signed=sensor._signed)
            if sensor.sensor_type == PROP_SENSOR_TEMPERATURE and value == 1300:
                # The temperature sentinel 0x0514 means no reading is available.
                values.append(None)
            else:
                # Apply the CSV scale to convert the integer into sensor units.
                values.append(round(value * sensor._scale, 10))
    message.values = tuple(values)
    return message


def decode_temperature(byte_1, byte_2):
    """Decode a signed big-endian temperature in tenths of a degree."""
    value = (byte_1 << 8) + byte_2
    if value == 1300:
        return None
    if value > 32767:
        value -= 65536
    return value / 10


def decode_pairs(message_id, packet):
    """Try temperature, pressure, speed, and duration at both alignments.

    messages.csv specifies unsigned pressure (0.001 mbar/count), speed
    (0.6 rpm/count), and duration (10 ms/count). Only temperature uses
    signed values and the unavailable sentinel.
    """
    for start in (3, 4):
        yield "ID %d two-byte decode from offset %d:" % (message_id, start)
        for offset in range(start, len(packet) - 1, 2):
            raw = int.from_bytes(packet[offset:offset + 2], 'big', signed=True)
            unsigned = int.from_bytes(packet[offset:offset + 2], 'big')
            value = decode_temperature(packet[offset], packet[offset + 1])
            yield "  Offset %d: raw=%d temperature=%s mbar=%s rpm=%s ms=%d" % (
                offset, raw, value, round(unsigned * 0.001, 10),
                round(unsigned * 0.6, 10), unsigned * 10)
