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
import urllib.parse
import urllib.request

# keep in step with plugin/xpbench.c r_state()
STATE = ["lat", "lon", "alt_ft", "ias", "gs_kt", "trk", "hdg_mag", "xtk_nm", "gps_crs_mag",
         "dest", "servos", "hmode", "alt_status", "paused", "tripped", "roll", "vs_fpm",
         "dme_nm", "hdef_dot", "fromto", "app_dest", "app_count"]

TRIPPED = {0: None, 1: "low airspeed", 2: "autopilot servos dropped", 3: "on ground"}


class XPlaneError(Exception):
    pass


class XPlane:
    """Thin client for the X-Plane 12 web API (v3). Use 127.0.0.1: on Windows
    ``localhost`` tries IPv6 first and costs ~1 s per call."""

    def __init__(self, base="http://127.0.0.1:8086/api/v3", timeout=10):
        self.base = base
        self.timeout = timeout
        self._ids = {}

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
        """Pause or unpause and wait until X-Plane reports it. The toggle can
        take several seconds to land after a fresh start, and an air start is
        only initialised on the first unpaused frame (IAS and altitude read 0
        until then) -- nothing may assume the state before this returns."""
        t0 = time.time()
        toggled = 0
        while time.time() - t0 < wait_s:
            if self.paused() == on:
                return
            if toggled == 0 or time.time() - toggled > 10:
                self.cmd("sim/operation/pause_toggle")
                toggled = time.time()
            time.sleep(0.5)
        raise XPlaneError(f"sim did not {'pause' if on else 'unpause'} within {wait_s} s")

    def wait_flying(self, min_kias, wait_s=30):
        t0 = time.time()
        while time.time() - t0 < wait_s:
            if self.get("sim/flightmodel/position/indicated_airspeed") >= min_kias:
                return
            time.sleep(0.5)
        raise XPlaneError(f"airspeed never reached {min_kias} KIAS after unpause")


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
