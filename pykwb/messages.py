"""Load KWB Comfort 3 definitions and parse message payloads."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from enum import Enum
from importlib.resources import files
from typing import TYPE_CHECKING, Union

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


def load_messages():
    """Return message definitions as dictionaries of strings (Python 3.9+)."""
    resource = files("pykwb").joinpath("messages.csv")
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
