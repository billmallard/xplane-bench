# SPDX-License-Identifier: Apache-2.0
import datetime as dt
import json
import os

import pytest

from xpbench.service import DEFAULTS, Bench

PROFILE = "name: t1\nend: {max_s: 600}\n"


@pytest.fixture
def cfg(tmp_path):
    return {**DEFAULTS, "runs": str(tmp_path / "runs"), "hands_off_file": str(tmp_path / "HANDS_OFF")}


def test_queue_survives_a_restart(cfg):
    a = Bench(cfg)
    s1, _ = a.submit("QA", {"profile_yaml": PROFILE, "issue": "AER-1"})
    s2, _ = a.submit("AVIONICS", {"profile_yaml": PROFILE, "issue": "AER-2"})
    b = Bench(cfg)
    assert [s["id"] for s in b.queue] == [s1["id"], s2["id"]]
    assert b.sessions[s1["id"]]["requester"] == "QA"
    assert b.queue[0] is b.sessions[s1["id"]]  # one object, so cancel and worker see the same entry


def test_cancel_is_saved(cfg):
    a = Bench(cfg)
    s1, _ = a.submit("QA", {"profile_yaml": PROFILE})
    a.cancel(s1["id"])
    b = Bench(cfg)
    assert b.queue == []
    assert b.sessions[s1["id"]]["state"] == "cancelled"


def test_a_flight_cut_by_a_restart_is_an_error_and_is_not_re_flown(cfg):
    a = Bench(cfg)
    s1, _ = a.submit("QA", {"profile_yaml": PROFILE})
    with a.lock:
        a.queue.pop(0)
        s1["state"] = "flying"
        a._save()
    b = Bench(cfg)
    assert b.queue == []
    assert b.sessions[s1["id"]]["state"] == "error"
    assert "restarted" in b.sessions[s1["id"]]["result"]["reason"]


def test_finished_sessions_stay_fetchable_but_old_ones_are_dropped(cfg):
    a = Bench(cfg)
    s1, _ = a.submit("QA", {"profile_yaml": PROFILE})
    s2, _ = a.submit("QA", {"profile_yaml": PROFILE})
    with a.lock:
        a.queue.clear()
        s1["state"] = s2["state"] = "finished"
        s2["submitted"] = (dt.datetime.now(dt.timezone.utc) - dt.timedelta(days=40)).isoformat()
        a._save()
    b = Bench(cfg)
    assert s1["id"] in b.sessions
    assert s2["id"] not in b.sessions


def test_state_file_is_written_atomically(cfg):
    a = Bench(cfg)
    a.submit("QA", {"profile_yaml": PROFILE})
    assert not os.path.exists(a.state_file + ".tmp")
    with open(a.state_file, encoding="utf-8") as f:
        assert len(json.load(f)["queue"]) == 1
