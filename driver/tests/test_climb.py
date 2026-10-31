# SPDX-License-Identifier: Apache-2.0
from xpbench.session import CLIMB_MIN_FPM, climb_rate


def test_holds_inside_the_dead_band():
    assert climb_rate(700, 700, 95, 95) == 700
    assert climb_rate(500, 700, 97, 95) == 500


def test_backs_off_while_slow():
    # the 33758e63080f abort: 700 fpm commanded at 84-88 kt against 95
    assert climb_rate(700, 700, 87, 95) == 600


def test_never_below_the_floor():
    assert climb_rate(CLIMB_MIN_FPM, 700, 80, 95) == CLIMB_MIN_FPM
    assert climb_rate(250, 700, 80, 95) == CLIMB_MIN_FPM


def test_recovers_but_never_past_the_requested_rate():
    assert climb_rate(400, 700, 100, 95) == 500
    assert climb_rate(700, 700, 110, 95) == 700


def test_converges_on_a_rate_the_aircraft_can_hold():
    """Toy plant: full power gives 95 kt at 450 fpm; each 100 fpm costs 2 kt."""
    cmd = 700
    for _ in range(20):
        ias = 95 - (cmd - 450) / 100 * 2
        cmd = climb_rate(cmd, 700, ias, 95)
    assert 300 <= cmd <= 600
    assert abs((95 - (cmd - 450) / 100 * 2) - 95) <= 3
