"""Smoke test (run from driver/: python ../tools/smoke_fms.py): launch at PAMJI, load the KSMX R30 approach, list both flight plans, quit."""
"""Smoke test: launch at the profile start, load the approach into the FMS, read it back, quit."""
import os
import sys
import time

from xpbench import fmsplan
from xpbench.navdata import NavData, bearing, offset
from xpbench.xplane import Launcher

run = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else "runs/smoke")
os.makedirs(run, exist_ok=True)
nd = NavData()
app = nd.approach("KSMX", "R30", "GVO")
i = app.index_of("PAMJI")
prev, fix = app.legs[i - 1], app.legs[i]
crs = bearing((prev.lat, prev.lon), (fix.lat, fix.lon))
start = offset((fix.lat, fix.lon), (crs + 180) % 360, 2.0)
flight = {"aircraft": {"path": "Aircraft/Laminar Research/Cirrus SR22/Cirrus SR22.acf"},
          "lle_air_start": {"latitude": start[0], "longitude": start[1],
                            "elevation_in_meters": 2500 * 0.3048, "heading_true": crs,
                            "speed_in_meters_per_second": 110 * 0.514444}}
L = Launcher()
xp = L.start(flight, run)
try:
    xp.pause(True)
    print("plugin version", xp.get("xpbench/version"))
    plan = fmsplan.approach_plan(nd.cycle, "KSBA", nd.runway("KSBA", "RW25"), "KSMX",
                                 nd.runway("KSMX", "RW30"), "R30", "GVO", "RW30")
    xp.set_str("xpbench/fms/plan_in", plan)
    print("plan_in readback ok:", xp.get_str("xpbench/fms/plan_in") == plan)
    xp.cmd("xpbench/fms/load")
    time.sleep(3)
    print("count", xp.get("xpbench/fms/count"), "dest", xp.get("xpbench/fms/dest"))
    time.sleep(3)
    for plan in xp.get_str("xpbench/fms/entries").split(";"):
        tag, _, body = plan.partition("|")
        for n, e in enumerate(body.split(",") if body else []):
            print(tag, n, e)
    st = xp.state(); print("app_dest", st["app_dest"], "app_count", st["app_count"], "gps_id", xp.get_str("sim/cockpit2/radios/indicators/gps_nav_id"))
    print("state", xp.state())
finally:
    L.stop(xp)
