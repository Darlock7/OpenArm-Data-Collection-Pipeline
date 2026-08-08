"""One clock, used by everything.

If each subsystem timestamps with whatever it finds convenient, the streams
cannot be compared afterwards and the dataset is quietly worthless. So every
timestamp in this project comes from here.

THE RULE: measure elapsed time with a monotonic clock; record absolute time
exactly once, as an anchor.

Why not just use wall-clock time everywhere? Because wall-clock time is not
monotonic. NTP steps it, users change it, DST exists. A backwards jump
mid-episode produces negative durations and frames that appear to arrive
before the frame preceding them. Rare, silent, and unrecoverable after the
fact -- the exact failure profile you cannot afford in recorded data.

Conversely, a monotonic clock alone is useless across reboots: it counts
from an arbitrary origin, so "t = 12.4" means nothing tomorrow.

The fix is to record both, once, at episode start:

    t0_monotonic  ->  the origin all samples are measured against
    t0_utc        ->  what that origin corresponded to in real time

Every sample then stores seconds-since-t0_monotonic. Absolute time for any
sample is recoverable as t0_utc + offset, but no arithmetic inside the
episode ever touches the wall clock.
"""

from __future__ import annotations

import time
from dataclasses import dataclass


def monotonic_ns() -> int:
    """Nanoseconds from an arbitrary origin, guaranteed never to go backwards.

    Integer nanoseconds rather than float seconds: a float64 holding a large
    monotonic value loses resolution, and at 1 kHz we care about
    sub-millisecond differences.
    """
    return time.monotonic_ns()


@dataclass(frozen=True)
class TimeBase:
    """The anchor pair, captured once when an episode starts."""

    t0_monotonic_ns: int
    t0_utc_s: float

    @classmethod
    def now(cls) -> "TimeBase":
        # Read both back to back. The gap between the two calls is the
        # anchor's accuracy -- microseconds, far below our sync tolerance.
        return cls(t0_monotonic_ns=time.monotonic_ns(), t0_utc_s=time.time())

    def elapsed_s(self, sample_monotonic_ns: int) -> float:
        """Convert a raw monotonic reading into seconds since episode start."""
        return (sample_monotonic_ns - self.t0_monotonic_ns) / 1e9

    def to_utc(self, elapsed_s: float) -> float:
        """Recover absolute UTC for a sample. Metadata only, never for math."""
        return self.t0_utc_s + elapsed_s


class RateLimiter:
    """Paces a loop at a target rate without drifting.

    The naive version, `sleep(1/rate)` each pass, accumulates error: the work
    in the loop body is not free, so every iteration runs slightly late and
    the lateness compounds. Over a 60 s episode at 1 kHz that is thousands of
    missed cycles.

    This version tracks the absolute time each tick was *due* and sleeps
    until then, so a slow iteration is absorbed rather than propagated.
    """

    def __init__(self, rate_hz: float):
        self.period_ns = int(1e9 / rate_hz)
        self._next_ns = time.monotonic_ns()
        self.missed = 0  # cycles we could not keep up with

    def sleep(self) -> None:
        self._next_ns += self.period_ns
        now = time.monotonic_ns()
        delay_ns = self._next_ns - now
        if delay_ns > 0:
            time.sleep(delay_ns / 1e9)
        else:
            # We are already past the deadline. Count it and resynchronise
            # rather than trying to "catch up" with a burst, which would
            # distort the timestamps we are trying to record accurately.
            self.missed += 1
            self._next_ns = now
