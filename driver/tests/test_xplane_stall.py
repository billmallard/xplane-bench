# SPDX-License-Identifier: Apache-2.0
"""A web API call that times out inside a wait loop is "not yet" (SIM3c, 2026-11-07).

On 2026-11-07, three of four launches died in setup() at xp.pause(False, wait_s=120):
the first paused() check timed out after the client's 10 s, and the exception
escaped the 120 s wait. These tests replay that with a fake clock."""
import urllib.error

import pytest

from xpbench import xplane
from xpbench.xplane import XPlane, XPlaneError


class Clock:
    def __init__(self):
        self.t = 0.0

    def time(self):
        return self.t

    def sleep(self, s):
        self.t += s


@pytest.fixture
def clock(monkeypatch):
    c = Clock()
    monkeypatch.setattr(xplane, "time", c)
    return c


class StallingXPlane(XPlane):
    """Paused at launch. The first `stall` reads time out, each one costing the
    client's 10 s timeout, then the sim answers. pause_off lands on the next read."""

    def __init__(self, clock, stall, error=TimeoutError("timed out")):
        super().__init__()
        self.clock, self.left, self.error = clock, stall, error
        self.is_paused, self.cmds = True, []

    def get(self, name):
        if self.left:
            self.left -= 1
            self.clock.t += self.timeout
            raise self.error
        if name == "sim/time/paused":
            return int(self.is_paused)
        return 200.0  # indicated airspeed

    def cmd(self, name, duration=0):
        self.cmds.append(name)
        self.is_paused = name == "sim/operation/pause_on"


def test_unpause_survives_a_read_that_times_out(clock):
    xp = StallingXPlane(clock, stall=1)  # the 2026-11-07 failure
    xp.pause(False, wait_s=120)
    assert not xp.is_paused
    assert xp.stalls == 1
    assert xp.cmds == ["sim/operation/pause_off"]


def test_unpause_survives_several_stalls_inside_the_budget(clock):
    xp = StallingXPlane(clock, stall=6)  # 60 s of silence against a 120 s budget
    xp.pause(False, wait_s=120)
    assert xp.stalls == 6


def test_a_connect_timeout_counts_too(clock):
    xp = StallingXPlane(clock, stall=2, error=urllib.error.URLError(TimeoutError("timed out")))
    xp.pause(False, wait_s=120)
    assert xp.stalls == 2


def test_silence_past_the_budget_still_fails_and_says_why(clock):
    xp = StallingXPlane(clock, stall=10_000)
    with pytest.raises(XPlaneError) as e:
        xp.pause(False, wait_s=120)
    assert "did not unpause within 120 s" in str(e.value)
    assert "web API call(s) timed out" in str(e.value)


def test_an_error_that_is_not_a_timeout_is_not_swallowed(clock):
    xp = StallingXPlane(clock, stall=1, error=ConnectionRefusedError("X-Plane is gone"))
    with pytest.raises(ConnectionRefusedError):
        xp.pause(False, wait_s=120)


def test_wait_flying_survives_a_stall(clock):
    xp = StallingXPlane(clock, stall=1)
    xp.wait_flying(100, wait_s=60)
    assert xp.stalls == 1


class StuckXPlane(StallingXPlane):
    """Unpaused and answering, but the flight model is not running: IAS and
    sim time stand still (the 2026-11-18 cold-start failure's open question)."""

    def get(self, name):
        if name == "sim/time/paused":
            return 0
        if name == "sim/time/total_running_time_sec":
            return 42.0
        if name == "sim/flightmodel/position/elevation":
            return 2286.0  # 7500 ft
        return 3.0  # indicated airspeed


def test_wait_flying_returns_the_time_it_took(clock):
    xp = StallingXPlane(clock, stall=1)
    assert xp.wait_flying(100, wait_s=60) == pytest.approx(10.5)  # one 10 s stall + one 0.5 s poll


def test_wait_flying_failure_says_what_the_sim_was_doing(clock):
    xp = StuckXPlane(clock, stall=0)
    with pytest.raises(XPlaneError) as e:
        xp.wait_flying(88.0, wait_s=180)
    msg = str(e.value)
    assert "within 180 s" in msg
    assert "last IAS 3.0 KIAS" in msg
    assert "paused false" in msg
    assert "sim time advanced 0.0 s in 180 s" in msg
    assert "altitude 7500 ft" in msg
