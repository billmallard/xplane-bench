# xplane-bench

Fly scripted test flights in X-Plane 12 with nobody at the controls, and keep
X-Plane's own view of the flight as the independent "truth" to test an avionics
stack against.

It was built for the MakerPlane / pyEfis open avionics stack: an agent queues a
scenario ("start 2 nm before PAMJI at 2,500 ft, fly the RNAV 30 at KSMX, go
missed, fly 1 nm either side of the HANNU-WINCH leg, hold at WINCH"), and the
bench flies it overnight, logs what X-Plane's navigator saw, and hands back a
run directory. Scenario profiles use the same format as fix-gateway's synthetic
flight generator (`src/fixgw/sim/PROFILE_FORMAT.md`), so one scenario runs
cheaply in CI and, when it matters, for real in X-Plane.

## Parts

| Part | What it does |
|---|---|
| `plugin/` | A small X-Plane plugin (C, XPLM SDK 4.2). Loads `.fms` flight plans with procedures, holds airspeed on the throttle, steps the heading bug, guards against stalls / autopilot drops / ground contact by pausing the sim, and writes a 5 Hz truth log. Everything is exposed as `xpbench/...` datarefs and commands, so the driver needs only X-Plane's web API. |
| `driver/xpbench/session.py` | Flies one profile: launches X-Plane straight into the air start, loads and verifies the approach, couples the autopilot to GPS, flies the vertical profile, the missed approach, offsets and holds, then quits X-Plane and restores its preferences. |
| `driver/xpbench/service.py` | The shared bench: an HTTP API with per-requester tokens, a first-in-first-out queue, one session at a time, a flying window, a hands-off switch, and "owner first" (never touches an X-Plane it did not start). |

## Why it works the way it does

Findings from bringing it up on X-Plane 12.4.3 (2026-10-28), each now a rule in the code:

- **Start flights from the command line** (`X-Plane.exe --new_flight_json=<file>`).
  Starting a flight through the web API (`POST /api/v3/flight`) deadlocked X-Plane
  4 times out of 4 on the reference machine.
- **The driver owns engine power.** Altitude hold with no speed loop trades
  airspeed for altitude until the wing stalls and the autopilot lets go. The
  aircraft we fly (SR22, C172) have no autothrottle, so the plugin holds IAS on
  the throttle.
- **Never write the heading bug directly.** A direct write can make HDG mode turn
  the long way round; the plugin walks the bug with the cockpit's own
  heading-up/down commands.
- **Steer by track, not heading.** X-Plane's weather is live; the driver applies
  the wind-correction angle.
- **Command the mode, then the rate.** `alt_vs` engages vertical speed at the
  *current* rate, so the target rate is set after the command.
- **Wait for unpause to land.** An air start is only initialised on the first
  unpaused frame, and a pause toggle can take seconds to apply after launch.
- **X-Plane 12 keeps procedure legs in a separate approach flight plan**
  (`xplm_Fpl_Pilot_Approach`); the `.fms` keywords are `APP` / `APPTRANS`
  (not `APPTP`). A plan X-Plane cannot parse raises a modal alert in the sim.
- **Restore preferences.** A command-line start changes preferences, and X-Plane
  rewrites them on exit; the launcher snapshots and restores them every session.

## Build and run (Windows)

```bat
plugin\build.bat "C:\X-Plane 12"         :: builds and installs Resources\plugins\xpbench
cd driver
python -m xpbench.session ..\path\to\profile.yaml --aircraft sr22
python -m xpbench.service --config ..\config.local.yaml
```

Requirements: X-Plane 12.1+ (XPLM 4.1), Visual Studio Build Tools (MSVC),
Python 3.11+ with PyYAML (and `tzdata` on Windows if `zoneinfo` cannot find
America/Chicago).

## Licence

Apache-2.0. The X-Plane SDK is fetched at build time under Laminar Research's
own licence and is not part of this repository.
