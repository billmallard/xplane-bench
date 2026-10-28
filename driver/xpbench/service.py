# SPDX-License-Identifier: Apache-2.0
"""xplane-bench service: agents queue scenario sessions; one flies at a time.

    python -m xpbench.service --config config.local.yaml

Collision control lives here, not in any agent (plan section 8, ruling 5):
one session at a time, a first-in-first-out queue visible at /health, a token
per requester, a maximum session length, and a flying window outside which
requests wait. Bill first: if X-Plane is running and we did not start it, or
the hands-off flag file exists, nothing starts.

HTTP API (bearer token on every call except /health):
    GET  /health                    who is flying, the queue, the window, hands-off
    POST /sessions                  {"profile": "<name>" | "profile_yaml": "<text>",
                                     "issue": "AER-123", "aircraft": "sr22"}
                                    -> {"id", "position", "blocked_now"}
    GET  /sessions/{id}             state + result.json once finished
    GET  /sessions/{id}/timeline    timeline.jsonl
    GET  /sessions/{id}/truth       truth.jsonl (X-Plane's own navigator, 5 Hz)
    POST /sessions/{id}/cancel      queued: removed; flying: stopped at the next poll
"""
import argparse
import datetime as dt
import json
import os
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from zoneinfo import ZoneInfo

import yaml

from .profile import AIRCRAFT
from .xplane import Launcher

DEFAULTS = {
    "listen": "0.0.0.0",
    "port": 8770,
    "tokens": {},                       # token -> requester name
    "allowed_ips": [],                  # empty = any; normally the QNAP and the benches
    "window": {"tz": "America/Chicago", "start": "23:00", "end": "06:00"},
    "profile_dirs": [],                 # where bare profile names are looked up
    "runs": "runs",
    "xplane": r"C:\X-Plane 12",
    "hands_off_file": "HANDS_OFF",      # if this file exists, nothing starts
    "max_session_s": 2700,
    "default_aircraft": "sr22",
}


class Bench:
    def __init__(self, cfg):
        self.cfg = cfg
        self.lock = threading.Lock()
        self.queue = []          # list of session dicts, FIFO
        self.sessions = {}       # id -> session dict
        self.current = None
        self.stop = threading.Event()
        os.makedirs(cfg["runs"], exist_ok=True)

    # ---- policy ----------------------------------------------------------------
    def in_window(self, now=None, duration_s=0):
        w = self.cfg["window"]
        tz = ZoneInfo(w["tz"])
        now = (now or dt.datetime.now(tz)).astimezone(tz)
        sh, sm = map(int, w["start"].split(":"))
        eh, em = map(int, w["end"].split(":"))
        start = now.replace(hour=sh, minute=sm, second=0, microsecond=0)
        end = now.replace(hour=eh, minute=em, second=0, microsecond=0)
        if end <= start:  # overnight window
            if now >= start:
                end += dt.timedelta(days=1)
            else:
                start -= dt.timedelta(days=1)
        ends = now + dt.timedelta(seconds=duration_s)
        return start <= now and ends <= end

    def blocked_reason(self, duration_s):
        if os.path.exists(self.cfg["hands_off_file"]):
            return "hands-off switch is on"
        if Launcher.running() and self.current is None:
            return "X-Plane is running and not ours (Bill first)"
        if not self.in_window(duration_s=duration_s):
            w = self.cfg["window"]
            return f"outside the flying window {w['start']}-{w['end']} {w['tz']}"
        return None

    def resolve_profile(self, name):
        if os.path.isabs(name) and os.path.exists(name):
            return name
        for d in self.cfg["profile_dirs"]:
            for cand in (name, name + ".yaml"):
                p = os.path.join(d, cand)
                if os.path.exists(p):
                    return p
        raise FileNotFoundError(f"profile {name!r} not found in {self.cfg['profile_dirs']}")

    # ---- queue -----------------------------------------------------------------
    def submit(self, requester, body):
        if body.get("profile_yaml"):
            # inline profile: agents cannot write to this workstation, so a new
            # scenario travels in the request and is kept with the queue entry
            raw = yaml.safe_load(body["profile_yaml"])
            if not isinstance(raw, dict) or "name" not in raw:
                raise ValueError("profile_yaml must be a profile with a name")
            inbox = os.path.join(self.cfg["runs"], "inline_profiles")
            os.makedirs(inbox, exist_ok=True)
            path = os.path.join(inbox, f"{uuid.uuid4().hex[:8]}-{raw['name']}.yaml")
            with open(path, "w", encoding="utf-8") as f:
                f.write(body["profile_yaml"])
        else:
            path = self.resolve_profile(body["profile"])
            with open(path, encoding="utf-8") as f:
                raw = yaml.safe_load(f)
        aircraft = body.get("aircraft", self.cfg["default_aircraft"])
        if aircraft not in AIRCRAFT:
            raise ValueError(f"aircraft must be one of {sorted(AIRCRAFT)}")
        est = min(float((raw.get("end") or {}).get("max_s", 3600)), self.cfg["max_session_s"]) + 180
        s = {"id": uuid.uuid4().hex[:12], "requester": requester, "issue": body.get("issue"),
             "profile": path, "aircraft": aircraft, "estimate_s": est, "state": "queued",
             "submitted": dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds"),
             "run_dir": None, "result": None, "blocked": None}
        with self.lock:
            self.queue.append(s)
            self.sessions[s["id"]] = s
            return s, len(self.queue)

    def cancel(self, sid):
        with self.lock:
            s = self.sessions.get(sid)
            if not s:
                return None
            if s in self.queue:
                self.queue.remove(s)
                s["state"] = "cancelled"
            elif s is self.current:
                s["cancel"] = True
            return s

    def health(self):
        with self.lock:
            cur = self.current
            return {
                "flying": None if not cur else {k: cur[k] for k in ("id", "requester", "issue", "profile", "started")},
                "queue": [{k: s[k] for k in ("id", "requester", "issue", "profile", "estimate_s", "blocked")}
                          for s in self.queue],
                "window": self.cfg["window"],
                "in_window_now": self.in_window(),
                "hands_off": os.path.exists(self.cfg["hands_off_file"]),
                "xplane_running": Launcher.running(),
            }

    def worker(self):
        from .session import Session
        while not self.stop.is_set():
            time.sleep(5)
            with self.lock:
                nxt = self.queue[0] if self.queue else None
            if not nxt:
                continue
            reason = self.blocked_reason(nxt["estimate_s"])
            nxt["blocked"] = reason
            if reason:
                continue
            with self.lock:
                self.queue.pop(0)
                self.current = nxt
                nxt["state"] = "flying"
                nxt["started"] = dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds")
            try:
                sess = Session(nxt["profile"], nxt["aircraft"], self.cfg["runs"], self.cfg["xplane"],
                               requester=nxt["requester"], issue=nxt["issue"])
                sess.p.max_s = min(sess.p.max_s, self.cfg["max_session_s"])
                sess.should_stop = lambda: nxt.get("cancel", False)
                nxt["run_dir"] = sess.dir
                nxt["result"] = sess.run()
                nxt["state"] = "finished"
            except Exception as e:
                nxt["state"] = "error"
                nxt["result"] = {"outcome": "error", "reason": f"{type(e).__name__}: {e}"}
            finally:
                with self.lock:
                    self.current = None


def make_handler(bench):
    cfg = bench.cfg

    class H(BaseHTTPRequestHandler):
        server_version = "xplane-bench/0.1"

        def log_message(self, fmt, *a):
            print(f"{self.client_address[0]} {fmt % a}", flush=True)

        def _send(self, code, obj=None, raw=None, ctype="application/json"):
            body = raw if raw is not None else json.dumps(obj, indent=1).encode()
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _auth(self):
            if cfg["allowed_ips"] and self.client_address[0] not in cfg["allowed_ips"]:
                self._send(403, {"error": "address not allowed"})
                return None
            tok = self.headers.get("Authorization", "").removeprefix("Bearer ").strip()
            who = cfg["tokens"].get(tok)
            if not who:
                self._send(401, {"error": "bad or missing bearer token"})
            return who

        def _file(self, sid, name, ctype):
            s = bench.sessions.get(sid)
            if not s or not s.get("run_dir"):
                return self._send(404, {"error": "no such session or not started"})
            p = os.path.join(s["run_dir"], name)
            if not os.path.exists(p):
                return self._send(404, {"error": f"{name} not written yet"})
            with open(p, "rb") as f:
                self._send(200, raw=f.read(), ctype=ctype)

        def do_GET(self):
            parts = [p for p in self.path.split("?")[0].split("/") if p]
            if parts == ["health"]:
                return self._send(200, bench.health())
            if not self._auth():
                return
            if len(parts) == 2 and parts[0] == "sessions":
                s = bench.sessions.get(parts[1])
                return self._send(200, {k: v for k, v in s.items() if k != "cancel"}) if s else \
                    self._send(404, {"error": "no such session"})
            if len(parts) == 3 and parts[0] == "sessions" and parts[2] in ("timeline", "truth"):
                return self._file(parts[1], f"{parts[2]}.jsonl", "application/x-ndjson")
            self._send(404, {"error": "not found"})

        def do_POST(self):
            who = self._auth()
            if not who:
                return
            parts = [p for p in self.path.split("?")[0].split("/") if p]
            n = int(self.headers.get("Content-Length") or 0)
            body = json.loads(self.rfile.read(n) or b"{}")
            if parts == ["sessions"]:
                try:
                    s, pos = bench.submit(who, body)
                except (FileNotFoundError, ValueError, KeyError) as e:
                    return self._send(400, {"error": str(e)})
                return self._send(202, {"id": s["id"], "position": pos,
                                        "blocked_now": bench.blocked_reason(s["estimate_s"])})
            if len(parts) == 3 and parts[0] == "sessions" and parts[2] == "cancel":
                s = bench.cancel(parts[1])
                return self._send(200, {"id": parts[1], "state": s["state"]}) if s else \
                    self._send(404, {"error": "no such session"})
            self._send(404, {"error": "not found"})

    return H


def main(argv=None):
    ap = argparse.ArgumentParser(description="xplane-bench service")
    ap.add_argument("--config", required=True)
    a = ap.parse_args(argv)
    with open(a.config, encoding="utf-8") as f:
        cfg = {**DEFAULTS, **(yaml.safe_load(f) or {})}
    bench = Bench(cfg)
    threading.Thread(target=bench.worker, daemon=True).start()
    srv = ThreadingHTTPServer((cfg["listen"], cfg["port"]), make_handler(bench))
    print(f"xplane-bench listening on {cfg['listen']}:{cfg['port']}", flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
