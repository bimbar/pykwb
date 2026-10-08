# pykwb
Library to interpret the serial output of KWB Comfort 3 controllers.

Supports Easyfire 1 and Easyfire 2 heaters.

## Quick Start

Install:

```sh
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install .
```

Run with your RS485 terminal server's host and port:

```sh
python3 -m pykwb.kwb --tcp --host 127.0.0.1 --port 23 --summary
```

(Note: You will need a RS485 to network converter like this : https://www.amazon.de/dp/B0BGHVRMPJ)

Or use your serial device:

```sh
python3 -m pykwb.kwb --serial --interface /dev/ttyUSB0 --summary
```

## Running

### Install

#### From Source

```sh
python3 setup.py build
python3 setup.py install
```

#### From Pypi Repo

```sh
pip3 install pykwb
```

## Integrating

Both examples connect to an RS485 terminal server. Replace the host and port with
your own settings.

### Blocking

Await `listen_for()` to collect readings before continuing. This waits in the
calling coroutine while allowing other asyncio tasks to run.

```python
import asyncio

from pykwb.kwb import KWBEasyfire, PROP_MODE_TCP


async def main():
    kwb = KWBEasyfire(PROP_MODE_TCP, _ip="127.0.0.1", _port=23)
    try:
        await kwb.listen_for(seconds=60)
        for sensor in kwb.get_sensors():
            print(sensor)
    finally:
        await kwb.close()


asyncio.run(main())
```

### Nonblocking

Run `listen_forever()` as a background task while your application does other
asynchronous work. 

```python
import asyncio

from pykwb.kwb import KWBEasyfire, PROP_MODE_TCP

async def main():
    kwb = KWBEasyfire(PROP_MODE_TCP, _ip="127.0.0.1", _port=23)
    listener = asyncio.create_task(kwb.listen_forever())
    try:
        # Replace this sleep with your application's asynchronous work.
        await asyncio.sleep(60)
        for sensor in kwb.get_sensors():
            print(sensor)
    finally:
        listener.cancel()
        try:
            await listener
        except asyncio.CancelledError:
            pass
        finally:
            await kwb.close()


asyncio.run(main())
```

## Development

### Linting

Run all lint checks:

```sh
python3 -m pip install tox
tox -e lint
```

To run the tools individually, install development dependencies. Ruff checks unused imports,
local variables, and unpacked variables; Vulture also checks unused module-level
and class-level variables and other dead code across the source and tests:

```sh
python3 -m pip install -r requirements_test.txt
python3 -m ruff check pykwb tests setup.py
python3 -m vulture
```

Prefix intentionally unused variables with an underscore (for example, `_unused`).
Ruff does not check unused function arguments; Vulture can report them.
Lint failures return a nonzero exit code; both checks run on pushes and pull
requests in GitHub Actions. Vulture uses a 60% confidence threshold to include
unused globals. It cannot prove whether external consumers or dynamic lookups
use a symbol: review findings before removing public API symbols. For a confirmed
false positive, add an exact name to `tool.vulture.ignore_names` in
`pyproject.toml` with a comment explaining the external use. Avoid broad patterns;
exceptions apply to matching names throughout the project.

### Testing

```sh
python3 -m unittest discover -s tests -v
```

## Bug Reports

To file a bug report, append `--log-level debug > trace.log` to the command you're using to run pykwb. For example

```sh
python3 pykwb/kwb.py --tcp --host 127.0.0.1 --port 23 --log-level debug > trace.log
```

Then open an issue on Github and attach trace.log.

## References

- KWB Kessel RS485 Protokoll
https://www.mikrocontroller.net/topic/274137

- C implementation from thomas_t33
https://www.mikrocontroller.net/attachment/190264/rs485kwb.c
https://www.mikrocontroller.net/attachment/190265/rs485kwb.h

- Python implementation from markus_h62
https://www.mikrocontroller.net/attachment/200110/grabserial.py

- Python implementation from haros
https://www.mikrocontroller.net/attachment/345168/grab32.py
or
https://www.mikrocontroller.net/attachment/345375/logkwb.py

- PHP implementation from ksau
https://www.mikrocontroller.net/attachment/200878/kwb_log.php

- Perl implementation from markus_h62
https://www.mikrocontroller.net/attachment/203419/00_KWB.pm

- https://github.com/windundsterne/esp-kwb-mqttlogger/blob/main/esp-kwb-mqttlogger.ino
