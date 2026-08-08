# OpenArm 2.0 — Data Collection Pipeline

A data collection platform for the [OpenArm 2.0](https://docs.openarm.dev) bimanual
robot arm: reads joint state over CAN FD, synchronises it with four camera streams,
stores episodes for robot learning, and serves them over a REST API with a live
monitoring dashboard.

**Status: work in progress.** This README is a placeholder and will be replaced by the
full write-up (architecture, design decisions, trade-offs, and what I could not verify).

## Built and tested without hardware

I did not have access to an OpenArm, a CAN adapter, or any of the four cameras. Every
data source in this project therefore has two implementations behind a common interface:
a simulated one that runs anywhere, and a hardware one written against the SocketCAN and
python-can documentation but **never executed**.

Simulated data is labelled as such in episode metadata, in filenames, and in the
dashboard. Nothing here should be mistaken for a real demonstration.

## Quick start

```bash
python3 -m venv .venv
./.venv/bin/pip install -r requirements.txt
./.venv/bin/python -m pytest tests/ -q
```

## Layout

```
openarm_pipeline/
  config.py        rig description: 14 joints, 4 cameras, bus rates
  clock.py         the single timestamp policy every subsystem uses
  can/
    protocol.py    Damiao MIT-mode frame encode/decode
    source.py      the mock-vs-hardware seam
    mock.py        simulated arm
    socketcan.py   real hardware path (untested)
    assembler.py   individual motor frames -> whole-arm snapshots
tests/
docs/
```

## Licence

MIT
