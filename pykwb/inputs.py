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

"""Byte inputs with lazy resource ownership and explicit failure recovery.

Reads never reconnect internally: the caller must discard its partial frame
before asking the input to recover from a transport failure.
"""

import asyncio
import logging
from abc import ABC, abstractmethod

import serial_asyncio_fast


class ByteInput(ABC):
    """Read one byte or raise; cancellation leaves the input open."""

    @abstractmethod
    async def read_byte(self) -> int:
        """Read a byte, raising EOFError at end of input."""

    @abstractmethod
    async def close(self) -> None:
        """Release resources; safe to call more than once."""

    async def recover(self, error: Exception) -> bool:
        """Close failed input and report whether the caller should retry."""
        await self.close()
        return False


class StreamInput(ByteInput):
    """Common lifecycle for asyncio TCP and serial streams."""

    def __init__(self, connect_timeout, logger=None):
        self._connect_timeout = connect_timeout
        self._logger = logger if logger is not None else logging.getLogger(__name__)
        self._reader = None
        self._writer = None

    @abstractmethod
    async def _connect(self):
        """Create a reader/writer pair."""

    async def open(self):
        self._reader, self._writer = await asyncio.wait_for(
            self._connect(), self._connect_timeout)

    async def read_byte(self) -> int:
        # Ready streams must still allow listening deadlines and cancellation.
        await asyncio.sleep(0)
        if self._reader is None:
            await self.open()
        data = await self._reader.read(1)
        if not data:
            raise EOFError("Input connection closed")
        value = data[0]
        self._logger.debug("READ: %s", value, extra={'diagnostic': True})
        return value

    async def close(self) -> None:
        writer, self._writer = self._writer, None
        self._reader = None
        if writer is not None:
            writer.close()
            try:
                await writer.wait_closed()
            except OSError:
                pass


class TCPInput(StreamInput):
    """TCP stream with optional backoff after explicit transport failures."""

    def __init__(self, host, port, settings, logger):
        super().__init__(settings['connect_timeout'], logger)
        self._host = host
        self._port = port
        self._settings = settings
        self._retry_delay = settings['retry_initial']

    async def _connect(self):
        return await asyncio.open_connection(self._host, self._port)

    async def open(self):
        await super().open()
        self._retry_delay = self._settings['retry_initial']

    async def _connection_lost(self, error):
        self._logger.warning("TCP disconnected: %s", error)
        await self.close()

    def _next_retry_delay(self):
        delay = min(self._retry_delay, self._settings['retry_max'])
        self._retry_delay = min(delay * 2, self._settings['retry_max'])
        self._logger.info("TCP reconnect in %g seconds", delay)
        return delay

    async def recover(self, error: Exception) -> bool:
        if not self._settings['reconnect']:
            return await super().recover(error)
        await self._connection_lost(error)
        await asyncio.sleep(self._next_retry_delay())
        return True


class SerialInput(StreamInput):
    """Serial stream; failures propagate without reconnecting."""

    def __init__(self, device, baudrate, connect_timeout, logger=None):
        super().__init__(connect_timeout, logger)
        self._device = device
        self._baudrate = baudrate

    async def _connect(self):
        return await serial_asyncio_fast.open_serial_connection(
            url=self._device, baudrate=self._baudrate)


class FileInput(ByteInput):
    """Replay a capture containing one decimal byte per line."""

    def __init__(self, path, logger=None):
        self._path = path
        self._logger = logger if logger is not None else logging.getLogger(__name__)
        self._file = None

    async def read_byte(self) -> int:
        await asyncio.sleep(0)
        if self._file is None:
            self._file = open(self._path, 'r')
        line = self._file.readline()
        if not line:
            raise EOFError("EOF")
        value = int(line)
        if not 0 <= value <= 255:
            raise ValueError("Capture byte must be between 0 and 255")
        self._logger.debug("READ: %s", value, extra={'diagnostic': True})
        return value

    async def close(self) -> None:
        if self._file is not None:
            self._file.close()
            self._file = None
