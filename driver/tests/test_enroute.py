# SPDX-License-Identifier: Apache-2.0
"""En-route legs (SIM3): profile shape, airway expansion, the .fms writer."""
import os
import textwrap

import pytest

from xpbench import fmsplan
from xpbench.navdata import NavData
from xpbench.profile import load

XP = r"C:\X-Plane 12"


def write(tmp_path, name, text):
    p = tmp_path / name
    p.write_text(textwrap.dedent(text), encoding="utf-8")
    return str(p)


# ---- profile ---------------------------------------------------------------------
def test_airway_into_an_approach(tmp_path):
    p = load(write(tmp_path, "a.yaml", """
        name: a
        start: {fix: GOLET, before_nm: 3, alt_ft: 6000}
        route:
          - waypoint: GVO
          - airway: {id: V27, exit: VTU}
          - procedure: {airport: KSMX, approach: R30, missed: true}
        """))
    assert p.enroute == [{"kind": "waypoint", "fix": "GVO"},
                         {"kind": "airway", "id": "V27", "exit": "VTU"}]
    assert (p.airport, p.approach, p.missed) == ("KSMX", "R30", True)


def test_en_route_only_needs_a_destination(tmp_path):
    p = load(write(tmp_path, "b.yaml", """
        name: b
        destination: KSMX
        start: {fix: KWANG, before_nm: 2, alt_ft: 6000}
        route: [{waypoint: GVO}, {airway: {id: V27, exit: VTU}}]
        end: {after_fix: VTU, max_s: 1800}
        """))
    assert p.approach is None and p.airport == "KSMX" and p.end_after_fix == "VTU"
    with pytest.raises(ValueError, match="destination"):
        load(write(tmp_path, "c.yaml", """
            name: c
            start: {fix: GVO, alt_ft: 6000}
            route: [{waypoint: GVO}]
            """))


@pytest.mark.parametrize("route, msg", [
    ("[{airway: {id: V27, exit: VTU}}]", "entry fix"),
    ("[{procedure: {airport: KSMX, approach: R30}}, {waypoint: GVO}]", "last"),
    ("[{procedure: {airport: KSMX, approach: R30}}, {procedure: {airport: KSMX, approach: R12}}]", "last"),
    ("[{airway: V27}]", "airway item"),
    ("[{vor: GVO}]", "not a waypoint"),
])
def test_route_shape_errors(tmp_path, route, msg):
    with pytest.raises(ValueError, match=msg):
        load(write(tmp_path, "d.yaml", f"""
            name: d
            destination: KSMX
            start: {{fix: GVO, alt_ft: 6000}}
            route: {route}
            """))


def test_approach_only_profile_is_unchanged():
    here = os.path.dirname(__file__)
    prof = os.path.join(here, "..", "..", "profiles.local", "fp8-ksmx-rnav30-missed.yaml")
    if not os.path.exists(prof):
        pytest.skip("local FP8 profile not present")
    p = load(prof)
    assert p.enroute == [] and p.approach and p.start_fix


# ---- .fms writer --------------------------------------------------------------------
def test_route_plan_writes_every_airway_fix():
    plan = fmsplan.route_plan("2611", "KSBA", (34.42, -119.84), "KSMX", (34.90, -120.46),
                              [("GVO", "VOR", "DRCT", 34.53, -120.09),
                               ("GOLET", "FIX", "V27", 34.30, -119.95),
                               ("VTU", "VOR", "V27", 34.11, -119.05)],
                              "R30", None, "RW30")
    lines = plan.splitlines()
    assert lines[:7] == ["I", "1100 Version", "CYCLE 2611", "ADEP KSBA", "ADES KSMX", "DESRWY RW30", "APP R30"]
    assert "NUMENR 5" in lines
    assert lines[-4:] == ["3 GVO DRCT 0.000000 34.530000 -120.090000",
                          "11 GOLET V27 0.000000 34.300000 -119.950000",
                          "3 VTU V27 0.000000 34.110000 -119.050000",
                          "1 KSMX ADES 0.000000 34.900000 -120.460000"]


def test_approach_plan_is_byte_identical_to_before():
    plan = fmsplan.approach_plan("2611", "KSBA", (1, 2), "KSMX", (3, 4), "R30", "GVO", "RW30")
    assert plan == ("I\n1100 Version\nCYCLE 2611\nADEP KSBA\nADES KSMX\nDESRWY RW30\nAPP R30\n"
                    "APPTRANS GVO\nNUMENR 2\n1 KSBA ADEP 0.000000 1.000000 2.000000\n"
                    "1 KSMX ADES 0.000000 3.000000 4.000000\n")


# ---- nav data -----------------------------------------------------------------------
@pytest.fixture
def fake_xp(tmp_path):
    d = tmp_path / "xp" / "Resources" / "default data"
    (d / "CIFP").mkdir(parents=True)
    (d / "earth_fix.dat").write_text(
        "I\n1200 Version - data cycle 2611, build 1\n\n"
        " 34.300000 -119.950000 GOLET ENRT K2 4530263\n"
        " 34.200000 -119.800000 KWANG ENRT K2 4530263\n"
        "-40.000000  175.000000 KWANG ENRT NZ 4530263\n", encoding="latin-1")
    (d / "earth_nav.dat").write_text(
        "I\n1200 Version - data cycle 2611\n\n"
        "3  34.530000 -120.090000 0 11280 130 15.0 GVO ENRT K2 GAVIOTA VORTAC\n"
        "3  34.110000 -119.050000 0 10830 130 15.0 VTU ENRT K2 VENTURA VOR-DME\n", encoding="latin-1")
    (d / "earth_awy.dat").write_text(
        "I\n1100 Version - data cycle 2611\n\n"
        "  GVO K2  3 GOLET K2 11 N 1  64 179 V27\n"
        "GOLET K2 11 KWANG K2 11 N 1  64 179 V27-V485\n"
        "KWANG K2 11   VTU K2  3 N 1  64 179 V27\n"
        "  GVO K2  3   VTU K2  3 N 1  64 179 V186\n", encoding="latin-1")
    return str(tmp_path / "xp")


def test_airway_path_and_point_kinds(fake_xp):
    nav = NavData(fake_xp)
    assert nav.airway("V27", "GVO", "VTU") == [("GVO", "K2"), ("GOLET", "K2"), ("KWANG", "K2"), ("VTU", "K2")]
    assert nav.airway("V186", "GVO", "VTU") == [("GVO", "K2"), ("VTU", "K2")]
    assert nav.point("GVO")[2] == "VOR" and nav.point("GOLET")[2] == "FIX"
    # same-named fixes: the airway's region picks the Californian KWANG
    assert nav.point("KWANG", region="K2")[0] == pytest.approx(34.2)
    with pytest.raises(KeyError):
        nav.airway("V27", "GVO", "NOSUCH")
    with pytest.raises(KeyError):
        nav.airway("J999", "GVO", "VTU")


@pytest.mark.skipif(not os.path.exists(os.path.join(XP, "Custom Data", "earth_awy.dat"))
                    and not os.path.exists(os.path.join(XP, "Resources", "default data", "earth_awy.dat")),
                    reason="no X-Plane nav data on this machine")
def test_golden_route_v27_on_this_machines_data():
    """The RS3 golden route's airway, as X-Plane's own data has it (SIM2b)."""
    path = [f for f, _ in NavData(XP).airway("V27", "GVO", "AVOLS")]
    assert path[0] == "GVO" and path[-1] == "AVOLS" and "VTU" in path and len(path) >= 5
