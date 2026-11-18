# SPDX-License-Identifier: Apache-2.0
"""X-Plane 12 control: the web API client, and the launcher.

Two lessons from SIM0 (2026-10-28) shape this module:

* Starting a flight through the web API (``POST /api/v3/flight``) deadlocked
  X-Plane 12.4.3 on the reference workstation, 4 of 4 times. The same flight
  JSON passed on the command line (``--new_flight_json``) works in ~24 s. So
  every session is one X-Plane launch, and the API is used only once X-Plane
  is flying.
* A command-line start rewrites preferences (it turned *Use system time* off),
  and X-Plane saves ``Output/preferences`` on every exit. The launcher snapshots
  that directory before a session and restores it after, so the owner's own
  flying is unaffected.
"""
import base64
import json
import os
import shutil
import subprocess
import time
import urllib.error
import urllib.parse
import urllib.request

# keep in step with plugin/xpbench.c r_state()
STATE = ["lat", "lon", "alt_ft", "ias", "gs_kt", "trk", "hdg_mag", "xtk_nm", "gps_crs_mag",
         "dest", "servos", "hmode", "alt_status", "paused", "tripped", "roll", "vs_fpm",
         "dme_nm", "hdef_dot", "fromto", "app_dest", "app_count"]

TRIPPED = {0: None, 1: "low airspeed", 2: "autopilot servos dropped", 3: "on ground"}


class XPlaneError(Exception):
    pass


def _stalled(e):
    """True for a web API call that timed out with X-Plane still running.

    X-Plane stops answering its web API for tens of seconds while it settles
    after a launch (SIM3c: three of four launches on 2026-11-07, all inside
    ``pause(False)``). A read timeout arrives as a bare ``TimeoutError``, and a
    connect timeout as a ``URLError`` wrapping one."""
    if isinstance(e, TimeoutError):
        return True
    return isinstance(e, urllib.error.URLError) and isinstance(e.reason, TimeoutError)


class XPlane:
    """Thin client for the X-Plane 12 web API (v3). Use 127.0.0.1: on Windows
    ``localhost`` tries IPv6 first and costs ~1 s per call."""

    def __init__(self, base="http://127.0.0.1:8086/api/v3", timeout=10):
        self.base = base
        self.timeout = timeout
        self._ids = {}
        self.stalls = 0  # web API calls that timed out inside a wait loop

    def call(self, method, path, body=None, timeout=None):
        data = json.dumps(body).encode() if body is not None else None
        req = urllib.request.Request(self.base + path, data=data, method=method,
                                     headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=timeout or self.timeout) as r:
            raw = r.read()
            return json.loads(raw) if raw else None

    def alive(self):
        try:
            self.call("GET", "/datarefs/count", timeout=3)
            return True
        except Exception:
            return False

    def _id(self, name, kind="datarefs"):
        key = (kind, name)
        if key not in self._ids:
            q = urllib.parse.urlencode({"filter[name]": name})
            d = self.call("GET", f"/{kind}?{q}")["data"]
            if not d:
                raise XPlaneError(f"no {kind[:-1]} named {name}")
            self._ids[key] = d[0]["id"]
        return self._ids[key]

    def get(self, name):
        return self.call("GET", f"/datarefs/{self._id(name)}/value")["data"]

    def get_str(self, name):
        v = self.get(name)
        return base64.b64decode(v).decode("latin-1").rstrip("\x00") if v else ""

    def set(self, name, value):
        return self.call("PATCH", f"/datarefs/{self._id(name)}/value", {"data": value})

    def set_str(self, name, text):
        return self.set(name, base64.b64encode(text.encode("latin-1")).decode("ascii"))

    def cmd(self, name, duration=0):
        return self.call("POST", f"/command/{self._id(name, 'commands')}/activate",
                         {"duration": duration})

    def state(self):
        return dict(zip(STATE, self.get("xpbench/state")))

    def paused(self):
        return bool(self.get("sim/time/paused"))

    def pause(self, on=True, wait_s=30):
        """Pause or unpause and wait until X-Plane reports it. The command can
        take several seconds to land after a fresh start, and an air start is
        only initialised on the first unpaused frame (IAS and altitude read 0
        until then) -- nothing may assume the state before this returns.

        Uses the explicit pause_on / pause_off commands, never pause_toggle: on
        a cold start (the 2026-10-29 04:00Z session, X-Plane flying only after
        51 s) a toggle that lands late is cancelled by the retry toggle sent
        10 s later, and the sim never unpauses. An explicit command is safe to
        resend.

        A call that times out counts as "not yet", not as a failure. The wait
        budget, not one call's timeout, decides when to give up (SIM3c)."""
        t0 = time.time()
        sent = 0
        name = "sim/operation/pause_on" if on else "sim/operation/pause_off"
        n0, last = self.stalls, None
        while time.time() - t0 < wait_s:
            try:
                if self.paused() == on:
                    return
                if sent == 0 or time.time() - sent > 5:
                    self.cmd(name)
                    sent = time.time()
            except Exception as e:
                if not _stalled(e):
                    raise
                self.stalls += 1
                last = e
            time.sleep(0.5)
        raise XPlaneError(f"sim did not {'pause' if on else 'unpause'} within {wait_s} s"
                          + self._stall_note(n0, last))

    SIM_TIME = "sim/time/total_running_time_sec"

    def _try_get(self, name):
        """One read for a diagnostic: None instead of an exception."""
        try:
            return self.get(name)
        except Exception:
            return None

    def wait_flying(self, min_kias, wait_s=30):
        """Wait until indicated airspeed reaches ``min_kias``; return the seconds
        it took. On failure the message says what the sim was doing, so "the
        flight model never ran" (sim time stood still, or still paused) reads
        differently from "it ran but did not accelerate" (2026-11-18: a slow
        cold start after the 12.4.4 update failed here with no other clue)."""
        t0 = time.time()
        n0, last, ias, sim0 = self.stalls, None, None, None
        while time.time() - t0 < wait_s:
            try:
                ias = self.get("sim/flightmodel/position/indicated_airspeed")
                if ias >= min_kias:
                    return time.time() - t0
                if sim0 is None:
                    # first answer, not before it: a stalled read here would
                    # be swallowed by _try_get and drop out of the stall count
                    sim0 = self._try_get(self.SIM_TIME)
            except Exception as e:
                if not _stalled(e):
                    raise
                self.stalls += 1
                last = e
            time.sleep(0.5)
        raise XPlaneError(f"airspeed never reached {min_kias} KIAS after unpause"
                          f" within {wait_s} s ({self._flying_note(ias, sim0, wait_s)})"
                          + self._stall_note(n0, last))

    def _flying_note(self, ias, sim0, wait_s):
        parts = ["last IAS " + ("unread" if ias is None else f"{ias:.1f} KIAS")]
        paused = self._try_get("sim/time/paused")
        parts.append("paused " + ("unread" if paused is None else str(bool(paused)).lower()))
        sim1 = self._try_get(self.SIM_TIME)
        if sim0 is None or sim1 is None:
            parts.append("sim time unread")
        else:
            parts.append(f"sim time advanced {sim1 - sim0:.1f} s in {wait_s} s")
        alt = self._try_get("sim/flightmodel/position/elevation")
        if alt is not None:
            parts.append(f"altitude {alt / 0.3048:.0f} ft")
        return ", ".join(parts)

    def _stall_note(self, n0, last):
        n = self.stalls - n0
        return f" ({n} web API call(s) timed out; last: {last})" if n else ""


class Launcher:
    """Start X-Plane straight into a flight, and put everything back afterwards."""

    def __init__(self, root=r"C:\X-Plane 12", log=print):
        self.root = root
        self.exe = os.path.join(root, "X-Plane.exe")
        self.prefs = os.path.join(root, "Output", "preferences")
        self.log = log
        self._snapshot = None
        self.proc = None

    @staticmethod
    def running():
        out = subprocess.run(["tasklist", "/FI", "IMAGENAME eq X-Plane.exe", "/NH"],
                             capture_output=True, text=True).stdout
        return "X-Plane.exe" in out

    def snapshot_prefs(self, run_dir):
        self._snapshot = os.path.join(run_dir, "prefs_snapshot")
        shutil.copytree(self.prefs, self._snapshot, dirs_exist_ok=True)

    def restore_prefs(self):
        if not self._snapshot:
            return
        for name in os.listdir(self._snapshot):
            src = os.path.join(self._snapshot, name)
            dst = os.path.join(self.prefs, name)
            if os.path.isdir(src):
                shutil.copytree(src, dst, dirs_exist_ok=True)
            else:
                shutil.copy2(src, dst)
        self.log("preferences restored from snapshot")

    def start(self, flight, run_dir, wait_s=300):
        """``flight`` is the X-Plane flight-initialisation dict (aircraft,
        lle_air_start / runway_start, ...)."""
        if self.running():
            raise XPlaneError("X-Plane is already running -- not ours to drive (Bill first)")
        path = os.path.abspath(os.path.join(run_dir, "flight.json"))
        with open(path, "w") as f:
            json.dump(flight, f)
        self.snapshot_prefs(run_dir)
        self.proc = subprocess.Popen([self.exe, f"--new_flight_json={path}"], cwd=self.root)
        xp = XPlane()
        t0 = time.time()
        while time.time() - t0 < wait_s:
            time.sleep(3)
            if self.proc.poll() is not None:
                raise XPlaneError(f"X-Plane exited during start (code {self.proc.returncode})")
            if xp.alive():
                try:
                    if xp.get("xpbench/version") >= 1:
                        lat = xp.get("sim/flightmodel/position/latitude")
                        want = flight.get("lle_air_start", {}).get("latitude")
                        if want is None or abs(lat - want) < 0.05:
                            self.log(f"X-Plane flying after {time.time() - t0:.0f} s")
                            return xp
                except XPlaneError:
                    raise XPlaneError("xpbench plugin not loaded -- run plugin/build.bat <X-Plane dir>")
                except Exception:
                    pass
        raise XPlaneError(f"X-Plane did not reach the flight within {wait_s} s")

    def stop(self, xp=None, wait_s=60):
        if xp is not None:
            try:
                xp.cmd("sim/operation/quit")
            except Exception:
                pass
        t0 = time.time()
        while self.running() and time.time() - t0 < wait_s:
            time.sleep(2)
        if self.running():
            subprocess.run(["taskkill", "/IM", "X-Plane.exe", "/F"], capture_output=True)
            self.log("X-Plane force-stopped")
        self.restore_prefs()
