# SPDX-License-Identifier: Apache-2.0
"""A session that errors keeps the raising line (SIM3c, 2026-11-03)."""
import json
import os

from xpbench.session import Session

PROFILE = """name: t1
start: {fix: PAMJI, before_nm: 2, alt_ft: 2500}
route:
  - procedure: {airport: KSMX, approach: R30}
end: {max_s: 600}
"""


def fake_xplane(tmp_path):
    d = tmp_path / "xp" / "Resources" / "default data"
    (d / "CIFP").mkdir(parents=True)
    (d / "earth_fix.dat").write_text("I\n1200 Version - data cycle 2611, build 1\n\n", encoding="latin-1")
    (d / "earth_awy.dat").write_text("I\n1100 Version - data cycle 2611\n\n", encoding="latin-1")
    return str(tmp_path / "xp")


def test_an_error_leaves_its_traceback(tmp_path, monkeypatch):
    profile = tmp_path / "p.yaml"
    profile.write_text(PROFILE, encoding="utf-8")
    s = Session(str(profile), runs=str(tmp_path / "runs"), xplane_root=fake_xplane(tmp_path))

    def a_call_that_times_out():
        raise TimeoutError("timed out")

    monkeypatch.setattr(s, "setup", a_call_that_times_out)
    monkeypatch.setattr(s.launcher, "stop", lambda xp=None: None)
    r = s.run()

    assert r["outcome"] == "error"
    assert r["reason"] == "TimeoutError: timed out"  # unchanged: agents read this
    assert "a_call_that_times_out" in r["traceback"]
    with open(os.path.join(s.dir, "result.json"), encoding="utf-8") as f:
        assert "a_call_that_times_out" in json.load(f)["traceback"]
    with open(os.path.join(s.dir, "timeline.jsonl"), encoding="utf-8") as f:
        rows = [json.loads(line) for line in f]
    assert any(row["msg"] == "exception" and "a_call_that_times_out" in row["traceback"] for row in rows)


def test_a_clean_session_has_no_traceback(tmp_path, monkeypatch):
    profile = tmp_path / "p.yaml"
    profile.write_text(PROFILE, encoding="utf-8")
    s = Session(str(profile), runs=str(tmp_path / "runs"), xplane_root=fake_xplane(tmp_path))
    monkeypatch.setattr(s, "setup", lambda: setattr(s, "start_route_idx", None))
    monkeypatch.setattr(s, "engage", lambda: None)
    monkeypatch.setattr(s, "fly", lambda: ("complete", None))
    monkeypatch.setattr(s.launcher, "stop", lambda xp=None: None)
    r = s.run()
    assert r["outcome"] == "complete"
    assert "traceback" not in r
