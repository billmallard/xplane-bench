# SPDX-License-Identifier: Apache-2.0
"""Read a scenario profile (the shared format both tiers fly).

The format is owned by fix-gateway (``src/fixgw/sim/PROFILE_FORMAT.md``).
This is an independent reader of the fields the X-Plane tier needs; it does
not copy fix-gateway's loader (that is GPL, this repo is Apache-2.0). The
profile carries nothing X-Plane-specific by design -- aircraft type, guard
speeds and the like are driver configuration (``AIRCRAFT`` below).
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
    airport: str
    approach: str
    transition: str
    missed: bool
    missed_after_map_s: float
    events: list = field(default_factory=list)
    end_after_event: str = None
    max_s: float = 3600
    cifp_cycle: str = None


def load(path):
    with open(path, encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    route = raw.get("route") or []
    procs = [r["procedure"] for r in route if "procedure" in r]
    if len(procs) != 1:
        raise ValueError("the X-Plane tier flies exactly one procedure per profile (v0)")
    proc = procs[0]
    start = raw["start"]
    end = raw.get("end") or {}
    return Profile(
        name=raw["name"],
        raw=raw,
        tas_kt=float(raw.get("aircraft", {}).get("tas_kt", 110)),
        start_fix=start["fix"],
        start_before_nm=float(start.get("before_nm", 0)),
        start_alt_ft=float(start["alt_ft"]),
        airport=proc["airport"],
        approach=proc["approach"],
        transition=proc.get("transition"),
        missed=bool(proc.get("missed", False)),
        missed_after_map_s=float((raw.get("pilot") or {}).get("missed_after_map_s", 5)),
        events=raw.get("events") or [],
        end_after_event=end.get("after_event"),
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
