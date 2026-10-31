# SPDX-License-Identifier: Apache-2.0
"""Read X-Plane's own navigation data (earth_fix.dat, CIFP/<ICAO>.dat).

The X-Plane tier deliberately uses X-Plane's data, not the profile's pinned
CIFP: the point of this tier is an independent navigator. The driver records
X-Plane's cycle beside the profile's in every run so a mismatch is visible.
"""
import math
import os
import re
from dataclasses import dataclass, field


@dataclass
class Leg:
    fix: str            # "" for a leg with no fix (e.g. CA)
    path: str           # ARINC path terminator: IF, TF, CA, DF, HM, ...
    desc: str           # waypoint description code (4 chars)
    lat: float = None
    lon: float = None
    missed: bool = False

    @property
    def role(self):
        d = self.desc.ljust(4)
        return {"A": "IAF", "B": "IAF", "I": "IF", "F": "FAF", "M": "MAP", "H": "MAHP"}.get(d[3], "")


@dataclass
class Approach:
    airport: str
    ident: str
    transition: str
    legs: list = field(default_factory=list)

    def index_of(self, fix, missed=None):
        for i, leg in enumerate(self.legs):
            if leg.fix == fix and (missed is None or leg.missed == missed):
                return i
        raise KeyError(fix)


class NavData:
    def __init__(self, xplane_root=r"C:\X-Plane 12"):
        base = os.path.join(xplane_root, "Resources", "default data")
        custom = os.path.join(xplane_root, "Custom Data")
        # Custom Data (a navdata subscription) wins over the default data, as in X-Plane
        self.fix_file = self._pick(custom, base, "earth_fix.dat")
        self.cifp_dir = self._pick(custom, base, "CIFP")
        self.awy_file = self._pick(custom, base, "earth_awy.dat")
        self.cycle = self._cycle(self.fix_file)
        self._fixes = None
        self._awys = None

    @staticmethod
    def _pick(custom, base, name):
        c = os.path.join(custom, name)
        return c if os.path.exists(c) else os.path.join(base, name)

    @staticmethod
    def _cycle(path):
        with open(path, encoding="latin-1") as f:
            for _ in range(3):
                m = re.search(r"data cycle (\d{4})", f.readline())
                if m:
                    return m.group(1)
        return "unknown"

    def fix(self, ident, airport=None, near=None):
        """(lat, lon) of a fix. A terminal fix of ``airport`` wins; otherwise,
        among same-named fixes (WINCH exists in California and New Zealand),
        the one nearest ``near``."""
        lat, lon, _ = self.point(ident, airport, near)
        return lat, lon

    def point(self, ident, airport=None, near=None, region=None):
        """(lat, lon, kind) of a fix, VOR or NDB; ``kind`` is "FIX", "VOR" or
        "NDB" (an .fms entry needs it). ``region`` (the ICAO region an airway
        record names) narrows same-named candidates first."""
        if self._fixes is None:
            self._fixes = {}
            with open(self.fix_file, encoding="latin-1") as f:
                for line in f:
                    p = line.split()
                    if len(p) >= 5:
                        try:
                            self._fixes.setdefault(p[2], []).append(
                                (float(p[0]), float(p[1]), p[3], "FIX", p[4]))
                        except ValueError:
                            pass
            # VORs and NDBs live in earth_nav.dat:
            # row code, lat, lon, elev, freq, class, var, ident, terminal, region, name
            with open(self.fix_file.replace("earth_fix.dat", "earth_nav.dat"), encoding="latin-1") as f:
                for line in f:
                    p = line.split()
                    if len(p) >= 8 and p[0] in ("2", "3"):
                        try:
                            self._fixes.setdefault(p[7], []).append(
                                (float(p[1]), float(p[2]), "ENRT", "VOR" if p[0] == "3" else "NDB",
                                 p[9] if len(p) > 9 else ""))
                        except ValueError:
                            pass
        cands = self._fixes.get(ident, [])
        if region:
            cands = [c for c in cands if c[4] == region] or cands
        if not cands:
            raise KeyError(ident)
        terminal = [c for c in cands if airport and c[2] == airport]
        if terminal:
            c = terminal[0]
        elif near is not None:
            c = min(cands, key=lambda c: (c[0] - near[0]) ** 2 + ((c[1] - near[1]) * math.cos(math.radians(near[0]))) ** 2)
        else:
            c = cands[0]
        return c[0], c[1], c[3]

    def airway(self, ident, entry, exit_):
        """The fixes along airway ``ident`` from ``entry`` to ``exit_``, both
        included, as [(fix, region), ...] -- the shortest hop path through
        earth_awy.dat. KeyError if the airway does not join them."""
        if self._awys is None:
            # earth_awy.dat (1100): fix1 region1 type1 fix2 region2 type2 dir level base top NAMES
            # where NAMES joins every airway sharing the segment with "-" (V208-V27-V458)
            self._awys = {}
            with open(self.awy_file, encoding="latin-1") as f:
                for line in f:
                    p = line.split()
                    if len(p) < 11:
                        continue
                    a, b = (p[0], p[1]), (p[3], p[4])
                    for name in p[10].split("-"):
                        g = self._awys.setdefault(name, {})
                        g.setdefault(a, set()).add(b)
                        g.setdefault(b, set()).add(a)
        g = self._awys.get(ident)
        if not g:
            raise KeyError(f"no airway {ident}")
        starts = [n for n in g if n[0] == entry]
        prev = {s: None for s in starts}
        queue = list(starts)
        while queue:
            n = queue.pop(0)
            if n[0] == exit_:
                path = []
                while n:
                    path.append(n)
                    n = prev[n]
                return path[::-1]
            for m in sorted(g[n]):
                if m not in prev:
                    prev[m] = n
                    queue.append(m)
        raise KeyError(f"airway {ident} does not join {entry} to {exit_}")

    def runway(self, airport, rwy):
        with open(os.path.join(self.cifp_dir, f"{airport}.dat"), encoding="latin-1") as f:
            for line in f:
                if line.startswith(f"RWY:{rwy}"):
                    m = re.search(r";([NS])(\d{2})(\d{2})(\d{4}),([EW])(\d{3})(\d{2})(\d{4})", line)
                    if m:
                        lat = int(m[2]) + int(m[3]) / 60 + int(m[4]) / 360000
                        lon = int(m[6]) + int(m[7]) / 60 + int(m[8]) / 360000
                        return (lat if m[1] == "N" else -lat, lon if m[5] == "E" else -lon)
        raise KeyError(f"{airport} {rwy}")

    def approach(self, airport, ident, transition=None):
        trans, final = [], []
        with open(os.path.join(self.cifp_dir, f"{airport}.dat"), encoding="latin-1") as f:
            for line in f:
                if not line.startswith("APPCH:"):
                    continue
                p = line[6:].split(",")
                rtype, proc, tr, fix, desc, path = p[1], p[2], p[3].strip(), p[4].strip(), p[8], p[11].strip()
                if proc != ident:
                    continue
                if rtype == "A":
                    if transition and tr == transition:
                        trans.append(Leg(fix, path, desc))
                else:
                    final.append(Leg(fix, path, desc))
        if not final:
            raise KeyError(f"{airport} {ident}")
        # legs after the MAP are the missed approach
        missed = False
        for leg in final:
            if missed:
                leg.missed = True
            if leg.role == "MAP":
                missed = True
        # the transition ends where the final begins (same fix): drop the duplicate
        if trans and final and trans[-1].fix == final[0].fix:
            trans = trans[:-1]
        app = Approach(airport, ident, transition or "", trans + final)
        near = None
        for leg in app.legs:
            if not leg.fix:
                continue
            if leg.fix.startswith("RW"):
                leg.lat, leg.lon = self.runway(airport, leg.fix)
            else:
                leg.lat, leg.lon = self.fix(leg.fix, airport, near)
            near = (leg.lat, leg.lon)
        return app


def bearing(a, b):
    """Initial great-circle bearing a -> b, degrees true."""
    la1, lo1, la2, lo2 = map(math.radians, (a[0], a[1], b[0], b[1]))
    y = math.sin(lo2 - lo1) * math.cos(la2)
    x = math.cos(la1) * math.sin(la2) - math.sin(la1) * math.cos(la2) * math.cos(lo2 - lo1)
    return math.degrees(math.atan2(y, x)) % 360


def offset(p, brg_deg, dist_nm):
    """Point ``dist_nm`` from ``p`` along ``brg_deg`` (true), spherical."""
    R = 3440.065
    la1, lo1, b = math.radians(p[0]), math.radians(p[1]), math.radians(brg_deg)
    d = dist_nm / R
    la2 = math.asin(math.sin(la1) * math.cos(d) + math.cos(la1) * math.sin(d) * math.cos(b))
    lo2 = lo1 + math.atan2(math.sin(b) * math.sin(d) * math.cos(la1),
                           math.cos(d) - math.sin(la1) * math.sin(la2))
    return math.degrees(la2), (math.degrees(lo2) + 540) % 360 - 180
