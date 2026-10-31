# SPDX-License-Identifier: Apache-2.0
"""Read a scenario profile (the shared format both tiers fly).

The format is owned by fix-gateway (``src/fixgw/sim/PROFILE_FORMAT.md``).
This is an independent reader of the fields the X-Plane tier needs; it does
not copy fix-gateway's loader (that is GPL, this repo is Apache-2.0). The
profile carries nothing X-Plane-specific by design -- aircraft type, guard
speeds and the like are driver configuration (``AIRCRAFT`` below).

En-route items (SIM3) use fix-gateway's route-segment kinds (RS1,
``doc/flightplan_keys.md``): a ``waypoint`` row, and an ``airway`` whose entry
is the fix before it::

    route:
      - waypoint: GVO
      - airway: {id: V27, exit: AVOLS}
      - procedure: {airport: KSMX, approach: R30, missed: true}   # optional, last

A route with no procedure names its ``destination`` airport. fix-gateway's
own loader does not read en-route items yet, so such a profile is X-Plane-tier
only until it does.
"""
from dataclasses import dataclass, field

import yaml


@dataclass
class Profile:
    name: str
    raw: dict
    tas_kt: float
    start_fix: str
    start_before_nm: float
    start_alt_ft: float
    airport: str               # the procedure's airport, else the destination
    approach: str = None       # None for an en-route-only profile
    transition: str = None
    missed: bool = False
    missed_after_map_s: float = 5
    enroute: list = field(default_factory=list)   # [{"kind": "waypoint"|"airway", ...}]
    departure: str = None
    events: list = field(default_factory=list)
    end_after_event: str = None
    end_after_fix: str = None
    max_s: float = 3600
    cifp_cycle: str = None


def _enroute(route):
    """The en-route items before the procedure, checked for shape."""
    items, procs = [], 0
    for n, r in enumerate(route):
        if "procedure" in r:
            procs += 1
            if n != len(route) - 1:
                raise ValueError("the procedure must be the last route item")
        elif "waypoint" in r:
            items.append({"kind": "waypoint", "fix": str(r["waypoint"])})
        elif "airway" in r:
            a = r["airway"]
            if not isinstance(a, dict) or "id" not in a or "exit" not in a:
                raise ValueError("an airway item is {airway: {id: V27, exit: AVOLS}}")
            if not items:
                raise ValueError("an airway needs an entry fix: put a waypoint before it")
            items.append({"kind": "airway", "id": str(a["id"]), "exit": str(a["exit"])})
        else:
            raise ValueError(f"route item {n} is not a waypoint, airway or procedure: {r}")
    if procs > 1:
        raise ValueError("the X-Plane tier flies at most one procedure per profile")
    if not procs and not items:
        raise ValueError("the route needs a procedure, en-route items, or both")
    return items


def load(path):
    with open(path, encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    route = raw.get("route") or []
    enroute = _enroute(route)
    procs = [r["procedure"] for r in route if "procedure" in r]
    proc = procs[0] if procs else {}
    airport = proc.get("airport") or raw.get("destination")
    if not airport:
        raise ValueError("a route with no procedure must name its destination airport")
    start = raw["start"]
    end = raw.get("end") or {}
    return Profile(
        name=raw["name"],
        raw=raw,
        tas_kt=float(raw.get("aircraft", {}).get("tas_kt", 110)),
        start_fix=start["fix"],
        start_before_nm=float(start.get("before_nm", 0)),
        start_alt_ft=float(start["alt_ft"]),
        airport=airport,
        approach=proc.get("approach"),
        transition=proc.get("transition"),
        missed=bool(proc.get("missed", False)),
        missed_after_map_s=float((raw.get("pilot") or {}).get("missed_after_map_s", 5)),
        enroute=enroute,
        departure=raw.get("departure"),
        events=raw.get("events") or [],
        end_after_event=end.get("after_event"),
        end_after_fix=end.get("after_fix"),
        max_s=float(end.get("max_s", 3600)),
        cifp_cycle=str((raw.get("cifp") or {}).get("cycle", "")) or None,
    )


# Driver-side aircraft configuration. vs0 is the stall speed in landing
# configuration (KIAS); the guard trips at 1.3 x vs0 by default.
AIRCRAFT = {
    "sr22": {"path": "Aircraft/Laminar Research/Cirrus SR22/Cirrus SR22.acf", "vs0": 60,
             "cruise_kias": 110, "approach_kias": 100, "climb_kias": 95},
    "c172": {"path": "Aircraft/Laminar Research/Cessna 172 SP/Cessna_172SP_G1000.acf", "vs0": 48,
             "cruise_kias": 100, "approach_kias": 85, "climb_kias": 75},
}
