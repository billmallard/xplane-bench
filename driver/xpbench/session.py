# SPDX-License-Identifier: Apache-2.0
"""Fly one scenario profile in X-Plane, unattended, and leave a run directory.

    python -m xpbench.session PROFILE.yaml [--aircraft sr22] [--runs runs]

The run directory holds: the profile, flight.json, plan.fms, timeline.jsonl
(what the driver did and saw, with reasons), truth.jsonl (X-Plane's own
position and navigator at 5 Hz, written by the plugin), and result.json.

Flying rules learned the hard way (SIM0, 2026-10-28):
  * power is the driver's job: the plugin holds a target IAS on the throttle
    before altitude hold is engaged -- altitude hold without it stalls the
    aircraft;
  * the autopilot is engaged only after the sim is unpaused, with joystick
    input overridden so a connected yoke cannot interfere;
  * the plugin's guard pauses the sim on low airspeed, servo drop, or ground
    contact; the driver then ends the session and reports why, rather than
    letting X-Plane crash and silently reset the flight.
"""
import argparse
import datetime as dt
import json
import math
import os
import shutil
import sys
import time

from . import fmsplan
from .navdata import NavData, bearing, offset
from .profile import AIRCRAFT, load
from .xplane import TRIPPED, Launcher, XPlaneError

FT_PER_M = 3.28084


def xtk_nm(p, a, b):
    """Signed cross-track of p from the great circle a->b, nm, + = right of course."""
    R = 3440.065
    d13 = _dist(a, p) / R
    t13 = math.radians(bearing(a, p))
    t12 = math.radians(bearing(a, b))
    return math.asin(math.sin(d13) * math.sin(t13 - t12)) * R


def _dist(a, b):
    la1, lo1, la2, lo2 = map(math.radians, (a[0], a[1], b[0], b[1]))
    h = math.sin((la2 - la1) / 2) ** 2 + math.cos(la1) * math.cos(la2) * math.sin((lo2 - lo1) / 2) ** 2
    return 2 * 3440.065 * math.asin(math.sqrt(h))


class Session:
    def __init__(self, profile_path, aircraft="sr22", runs="runs", xplane_root=r"C:\X-Plane 12",
                 requester="cli", issue=None):
        self.p = load(profile_path)
        self.ac = AIRCRAFT[aircraft]
        self.aircraft = aircraft
        stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self.dir = os.path.abspath(os.path.join(runs, f"{stamp}-{self.p.name}"))
        os.makedirs(self.dir, exist_ok=True)
        shutil.copy2(profile_path, os.path.join(self.dir, "profile.yaml"))
        self.nav = NavData(xplane_root)
        self.launcher = Launcher(xplane_root, log=self.note)
        self.requester, self.issue = requester, issue
        self.xp = None
        self.should_stop = lambda: False
        self.t0 = time.time()
        self.result = {"profile": self.p.name, "aircraft": aircraft, "requester": requester,
                       "issue": issue, "run_dir": self.dir,
                       "xplane_nav_cycle": self.nav.cycle, "profile_cifp_cycle": self.p.cifp_cycle,
                       "events": {}, "sequence": [], "outcome": None, "reason": None}
        self._tl = open(os.path.join(self.dir, "timeline.jsonl"), "a", encoding="utf-8")

    # ---- logging -------------------------------------------------------------
    def note(self, msg, **kw):
        row = {"t": round(time.time() - self.t0, 1), "msg": msg, **kw}
        self._tl.write(json.dumps(row) + "\n")
        self._tl.flush()
        print(f"[{row['t']:7.1f}] {msg}" + (f" {kw}" if kw else ""), flush=True)

    # ---- autopilot helpers ---------------------------------------------------
    def set_bug(self, hdg_mag):
        """The plugin walks the bug there with heading_up/down (a direct write
        can make HDG mode turn the long way round)."""
        self.xp.set("xpbench/ap/bug_target", float(hdg_mag % 360))

    def steer_track(self, trk_mag, st):
        """Hold a magnetic *track*: correct the bug by the current wind
        correction angle (X-Plane's live weather is not calm)."""
        psi_true = self.xp.get("sim/flightmodel/position/psi")
        wca = (st["trk"] - psi_true + 540) % 360 - 180
        self.set_bug(trk_mag - wca)

    def hdg_mode(self):
        self.xp.cmd("sim/autopilot/heading_sync")
        self.set_bug(self.xp.get("sim/cockpit2/autopilot/heading_dial_deg_mag_pilot"))
        if self.xp.get("sim/cockpit2/autopilot/heading_mode") != 1:
            self.xp.cmd("sim/autopilot/heading")

    def nav_mode(self):
        self.xp.set("xpbench/ap/bug_target", -1.0)
        self.xp.set("sim/cockpit2/radios/actuators/HSI_source_select_pilot", 2)  # GPS
        if self.xp.get("sim/cockpit2/autopilot/heading_mode") != 2:
            self.xp.cmd("sim/autopilot/NAV")

    def goto_alt(self, alt_ft, fpm):
        """Climb or descend to ``alt_ft`` at ``fpm`` with altitude capture armed.

        Order matters (learned 2026-10-28): ``alt_vs`` engages VS at the
        *current* vertical speed, so the rate is written after the command,
        never before -- otherwise the aircraft keeps its old rate and flies
        straight through (or away from) the armed altitude."""
        cur = self.xp.get("sim/flightmodel/position/elevation") * FT_PER_M
        self.xp.set("sim/cockpit2/autopilot/altitude_dial_ft", float(alt_ft))
        self.vert = {"alt": float(alt_ft), "fpm": abs(fpm), "t": time.time(), "wrong_since": None}
        if abs(cur - alt_ft) < 60:
            if self.xp.get("sim/cockpit2/autopilot/altitude_hold_status") != 2:
                self.xp.cmd("sim/autopilot/altitude_hold")
            return
        self.xp.cmd("sim/autopilot/alt_vs")
        time.sleep(0.5)
        self.xp.set("sim/cockpit2/autopilot/vvi_dial_fpm", float(abs(fpm) if alt_ft > cur else -abs(fpm)))
        self.note("vertical", to_ft=alt_ft, fpm=fpm, from_ft=round(cur))

    def supervise_vertical(self, st):
        """Re-command if the aircraft moves away from the armed altitude for 5 s."""
        v = getattr(self, "vert", None)
        if not v or st["alt_status"] == 2:
            return
        err = v["alt"] - st["alt_ft"]
        wrong = abs(err) > 100 and (err * st["vs_fpm"] < 0 or abs(st["vs_fpm"]) < 0.3 * v["fpm"])
        if not wrong:
            v["wrong_since"] = None
            return
        v["wrong_since"] = v["wrong_since"] or time.time()
        if time.time() - v["wrong_since"] >= 5:
            self.note("vertical not converging -- re-commanding", alt_ft=round(st["alt_ft"]),
                      vs=round(st["vs_fpm"]), target=v["alt"])
            self.goto_alt(v["alt"], v["fpm"])

    def speed(self, kias):
        self.xp.set("xpbench/speed/target_kias", float(kias))

    # ---- the flight ------------------------------------------------------------
    def setup(self):
        p, nav = self.p, self.nav
        if p.cifp_cycle and p.cifp_cycle != nav.cycle:
            self.note("nav cycle mismatch: X-Plane flies its own data", xplane=nav.cycle, profile=p.cifp_cycle)
        self.app = nav.approach(p.airport, p.approach, p.transition)
        self.legs = [l.fix for l in self.app.legs]
        i = self.app.index_of(p.start_fix)
        prev, fix = self.app.legs[i - 1], self.app.legs[i]
        crs = bearing((prev.lat, prev.lon), (fix.lat, fix.lon))
        start = offset((fix.lat, fix.lon), (crs + 180) % 360, p.start_before_nm)
        flight = {"aircraft": {"path": self.ac["path"]},
                  "lle_air_start": {"latitude": start[0], "longitude": start[1],
                                    "elevation_in_meters": p.start_alt_ft / FT_PER_M,
                                    "heading_true": crs,
                                    "speed_in_meters_per_second": self.ac["cruise_kias"] * 0.514444}}
        self.start_idx = i
        self.xp = self.launcher.start(flight, self.dir)
        xp = self.xp
        # power first; the air start only initialises on the first unpaused
        # frame, so let it fly a moment, then pause to set everything up
        xp.set("sim/operation/override/override_joystick", 1)
        xp.set("xpbench/guard/min_kias", 0.0)
        xp.cmd("xpbench/guard/reset")
        self.speed(self.ac["cruise_kias"])
        xp.pause(False)
        xp.wait_flying(0.8 * self.ac["cruise_kias"])
        xp.pause(True)
        # flight plan
        dep = "KSBA" if p.airport != "KSBA" else "KSMX"
        rwy = next((l.fix for l in self.app.legs if l.fix.startswith("RW")), None)
        plan = fmsplan.approach_plan(nav.cycle, dep, nav.runway(dep, self._any_rwy(dep)), p.airport,
                                     nav.runway(p.airport, rwy), p.approach, p.transition, rwy)
        with open(os.path.join(self.dir, "plan.fms"), "w") as f:
            f.write(plan)
        xp.set_str("xpbench/fms/plan_in", plan)
        xp.cmd("xpbench/fms/load")
        time.sleep(4)
        loaded = self.entries("A")
        self.note("fms loaded", approach=loaded)
        want = [l for l in self.legs if l]
        if [x for x in loaded if x and not x.startswith("(")] != want:
            raise XPlaneError(f"X-Plane's approach {loaded} differs from the CIFP legs {want}")
        xp.set("xpbench/fms/plan_sel", 2)
        xp.set("xpbench/fms/dest", i)
        xp.set_str("xpbench/log/path", os.path.join(self.dir, "truth.jsonl"))
        xp.cmd("xpbench/log/start")

    def _any_rwy(self, apt):
        with open(os.path.join(self.nav.cifp_dir, f"{apt}.dat"), encoding="latin-1") as f:
            for line in f:
                if line.startswith("RWY:"):
                    return line[4:].split(",")[0].strip()
        raise KeyError(apt)

    def entries(self, tag):
        for plan in self.xp.get_str("xpbench/fms/entries").split(";"):
            t, _, body = plan.partition("|")
            if t == tag:
                return [e.split(":")[1] for e in body.split(",")] if body else []
        return []

    def engage(self):
        xp = self.xp
        xp.set("sim/cockpit2/autopilot/altitude_dial_ft", self.p.start_alt_ft)
        xp.pause(False)
        xp.wait_flying(0.8 * self.ac["cruise_kias"])
        xp.cmd("sim/autopilot/servos_on")
        self.hdg_mode()
        time.sleep(0.5)
        if xp.get("sim/cockpit2/autopilot/altitude_hold_status") != 2:
            xp.cmd("sim/autopilot/altitude_hold")
        self.nav_mode()
        for _ in range(30):
            time.sleep(1)
            st = xp.state()
            if st["servos"] == 1 and st["hmode"] == 2:
                break
        st = xp.state()
        if not (st["servos"] == 1 and st["hmode"] == 2):
            raise XPlaneError(f"autopilot did not couple: servos={st['servos']} hmode={st['hmode']}")
        xp.set("xpbench/guard/min_kias", float(round(1.3 * self.ac["vs0"])))
        xp.set("xpbench/guard/watch_servos", 1)
        xp.set("xpbench/guard/watch_ground", 1)
        self.note("autopilot coupled to GPS", ias=round(st["ias"]), alt_ft=round(st["alt_ft"]))

    def fly(self):
        p, xp, legs = self.p, self.xp, self.app.legs
        faf = next(i for i, l in enumerate(legs) if l.role == "FAF")
        map_i = next(i for i, l in enumerate(legs) if l.role == "MAP")
        first_missed_fix = next(i for i, l in enumerate(legs) if l.missed and l.fix)
        events = {e["id"]: {"spec": e, "state": "wait", "t": None} for e in p.events}
        order = [e["id"] for e in p.events]
        last_dest, leg_t = None, time.time()
        t_map, missed_flown = None, False
        offset_ctl = None
        vert_done = set()
        dme_hist, from_since, abeam_done = [], None, set()
        t_start = time.time()
        while True:
            time.sleep(1.0)
            st = xp.state()
            now = time.time()
            if st["tripped"]:
                return "aborted", TRIPPED.get(int(st["tripped"]), "guard")
            if now - t_start > p.max_s:
                return "timeout", f"max_s {p.max_s} reached"
            if self.should_stop():
                return "cancelled", "cancelled by the requester"
            self.supervise_vertical(st)
            dest = int(st["app_dest"])
            coupled = st["hmode"] == 2
            # passed abeam the active fix without X-Plane sequencing it (it was
            # off course in HDG mode): go direct to it, so the next leg starts
            in_hold = 0 <= dest < len(legs) and legs[dest].path == "HM"
            if coupled and int(st["fromto"]) == 2 and not in_hold and 0 <= dest < len(legs) and legs[dest].fix:
                from_since = from_since or now
                if now - from_since >= 10 and dest not in abeam_done:
                    abeam_done.add(dest)
                    xp.set("xpbench/fms/direct_to", dest)
                    self.note("passed abeam without sequencing -- direct to", fix=legs[dest].fix)
            else:
                from_since = None
            # flying away from the active waypoint while coupled: not navigating
            dme_hist.append((now, st["dme_nm"], dest, coupled))
            dme_hist[:] = [h for h in dme_hist if now - h[0] <= 60]
            if (coupled and not in_hold and len(dme_hist) > 50 and all(h[2] == dest and h[3] for h in dme_hist)
                    and st["dme_nm"] - dme_hist[0][1] > 1.0):
                return "aborted", f"flying away from {legs[dest].fix or dest} for 60 s while coupled"
            if dest != last_dest:
                name = legs[dest].fix if 0 <= dest < len(legs) else "?"
                self.note("sequence", dest=dest, fix=name or f"(leg {legs[dest].path})")
                self.result["sequence"].append({"t": round(now - self.t0, 1), "dest": dest, "fix": name})
                last_dest, leg_t = dest, now

            # vertical profile: hold the start altitude to the FAF, then step down
            if dest == faf and "faf" not in vert_done:
                vert_done.add("faf")
                self.goto_alt(2200, 500)
            if faf < dest <= map_i and "final" not in vert_done:
                vert_done.add("final")
                self.speed(self.ac["approach_kias"])
                self.goto_alt(1000, 600)

            # MAP: X-Plane may suspend there; we go missed missed_after_map_s later
            if t_map is None and (dest > map_i or (dest == map_i and st["dme_nm"] < 0.2)):
                t_map = now
                self.note("MAP", dme_nm=round(st["dme_nm"], 2))
            if t_map and not missed_flown and now - t_map >= p.missed_after_map_s:
                missed_flown = True
                self.speed(self.ac["climb_kias"])
                self.goto_alt(4000, 700)
                if dest <= first_missed_fix - 1 or dest == map_i:
                    xp.set("xpbench/fms/direct_to", first_missed_fix)
                self.note("missed approach", direct_to=legs[first_missed_fix].fix)
                self.nav_mode()

            # events, in profile order
            for eid in order:
                ev = events[eid]
                if ev["state"] != "wait":
                    continue
                at = ev["spec"].get("at", {})
                ready = False
                if "leg" in at:
                    a, b = at["leg"].split("-")
                    ready = (0 < dest < len(legs) and legs[dest].fix == b and legs[dest - 1].fix in (a, "")
                             and now - leg_t >= at.get("after_s", 0)) or (
                        legs[dest].fix == b and a == legs[first_missed_fix].fix and missed_flown
                        and now - leg_t >= at.get("after_s", 0))
                elif "after" in at:
                    ready = events[at["after"]]["state"] == "done"
                elif "hold_at" in at:
                    ready = legs[dest].path == "HM" or (dest < len(legs) and legs[dest].fix == at["hold_at"]
                                                        and st["dme_nm"] < 0.3)
                if ready:
                    ev["state"], ev["t"] = "run", now
                    self.note("event start", id=eid, do=ev["spec"]["do"])
                break  # strictly one event at a time

            for eid in order:
                ev = events[eid]
                if ev["state"] != "run":
                    continue
                do = ev["spec"]["do"]
                if "offset_nm" in do:
                    done = self.step_offset(ev, do, st, dest)
                elif "hold_min" in do:
                    done = now - ev["t"] >= do["hold_min"] * 60
                else:
                    self.note("event kind not supported in v0 -- skipped", id=eid)
                    done = True
                if done:
                    ev["state"] = "done"
                    self.result["events"][eid] = {"start": round(ev["t"] - self.t0, 1),
                                                  "end": round(now - self.t0, 1)}
                    self.note("event done", id=eid)
            if p.end_after_event and events.get(p.end_after_event, {}).get("state") == "done":
                return "complete", None
            if st["alt_ft"] < 500:
                xp.pause(True)
                return "aborted", f"below 500 ft MSL ({st['alt_ft']:.0f})"

    def step_offset(self, ev, do, st, dest):
        """Fly to ``offset_nm`` right (+) / left (-) of the active leg in HDG
        mode, hold ``hold_s``; offset 0 re-couples GPSS and is done on capture."""
        legs = self.app.legs
        a, b = legs[dest - 1], legs[dest]
        if not a.fix:  # leg from a fixless leg (e.g. after CA): use the fix before that
            a = legs[dest - 2]
        x = xtk_nm((st["lat"], st["lon"]), (a.lat, a.lon), (b.lat, b.lon))
        dtk = st["gps_crs_mag"]
        target = float(do["offset_nm"])
        c = ev.setdefault("ctl", {"phase": "intercept"})
        if c["phase"] == "intercept":
            if "began" not in c:
                c["began"] = True
                self.hdg_mode()
            err = target - x
            if abs(err) <= 0.15:
                if target == 0.0:
                    self.nav_mode()
                    c["phase"] = "recouple"
                else:
                    self.steer_track(dtk, st)
                    c["phase"], c["t_hold"] = "hold", time.time()
                self.note("offset reached", target=target, xtk=round(x, 3))
            else:
                # 45 deg intercept, easing to 20 deg inside 0.4 nm
                self.steer_track(dtk + math.copysign(45 if abs(err) > 0.4 else 20, err), st)
            return False
        if c["phase"] == "hold":
            # track-hold on the offset line: 20 deg per nm of error, at most 20 deg
            corr = max(-20.0, min(20.0, -20.0 * (x - target)))
            self.steer_track(dtk + corr, st)
            return time.time() - c["t_hold"] >= do.get("hold_s", 0)
        if c["phase"] == "recouple":
            return st["hmode"] == 2

    def run(self):
        outcome, reason = "error", None
        try:
            self.setup()
            self.engage()
            outcome, reason = self.fly()
        except Exception as e:
            outcome, reason = "error", f"{type(e).__name__}: {e}"
        finally:
            try:
                if self.xp:
                    self.xp.pause(True)
                    self.xp.cmd("xpbench/log/stop")
            except Exception:
                pass
            self.result.update(outcome=outcome, reason=reason,
                               duration_s=round(time.time() - self.t0, 1))
            self.note("session end", outcome=outcome, reason=reason)
            self.launcher.stop(self.xp)
            with open(os.path.join(self.dir, "result.json"), "w") as f:
                json.dump(self.result, f, indent=2)
            self._tl.close()
        return self.result


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("profile")
    ap.add_argument("--aircraft", default="sr22", choices=sorted(AIRCRAFT))
    ap.add_argument("--runs", default="runs")
    ap.add_argument("--xplane", default=r"C:\X-Plane 12")
    a = ap.parse_args(argv)
    r = Session(a.profile, a.aircraft, a.runs, a.xplane).run()
    print(json.dumps({k: r[k] for k in ("outcome", "reason", "run_dir", "events")}, indent=2))
    return 0 if r["outcome"] == "complete" else 1


if __name__ == "__main__":
    sys.exit(main())
