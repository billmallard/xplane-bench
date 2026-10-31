# SPDX-License-Identifier: Apache-2.0
"""Build an X-Plane 11+ ``.fms`` flight plan (format version 1100)."""


def approach_plan(cycle, dep, dep_pos, dest, dest_pos, approach, transition=None, dest_rwy=None):
    """A plan that is just departure, destination, and the approach -- the
    procedure's own legs come from X-Plane's CIFP when it loads the plan."""
    # Keyword set and order as X-Plane 12's own parser expects them (its error
    # strings name ADEP, DEPRWY, SID, SIDTRANS, ADES, DESRWY, STAR, STARTRANS,
    # APP, APPTRANS, NUMENR). A plan X-Plane cannot parse raises a modal alert
    # in the sim -- fatal to an unattended session -- so build only this shape.
    lines = ["I", "1100 Version", f"CYCLE {cycle}", f"ADEP {dep}", f"ADES {dest}"]
    if dest_rwy:
        lines.append(f"DESRWY {dest_rwy}")
    lines.append(f"APP {approach}")
    if transition:
        lines.append(f"APPTRANS {transition}")
    lines += ["NUMENR 2",
              f"1 {dep} ADEP 0.000000 {dep_pos[0]:.6f} {dep_pos[1]:.6f}",
              f"1 {dest} ADES 0.000000 {dest_pos[0]:.6f} {dest_pos[1]:.6f}"]
    return "\n".join(lines) + "\n"


# .fms entry type codes (X-Plane flight plan format 1100)
ENTRY_TYPE = {"NDB": 2, "VOR": 3, "FIX": 11}


def route_plan(cycle, dep, dep_pos, dest, dest_pos, enroute, approach=None, transition=None, dest_rwy=None):
    """Departure, en-route entries, destination, and optionally an approach.

    ``enroute`` is a list of (ident, kind, via, lat, lon): ``kind`` FIX/VOR/NDB,
    ``via`` ``DRCT`` or the airway the leg *into* this entry follows. Every
    fix along an airway is written as its own entry with the airway in the via
    column, so the plan does not depend on X-Plane expanding the airway."""
    lines = ["I", "1100 Version", f"CYCLE {cycle}", f"ADEP {dep}", f"ADES {dest}"]
    if dest_rwy:
        lines.append(f"DESRWY {dest_rwy}")
    if approach:
        lines.append(f"APP {approach}")
        if transition:
            lines.append(f"APPTRANS {transition}")
    lines.append(f"NUMENR {len(enroute) + 2}")
    lines.append(f"1 {dep} ADEP 0.000000 {dep_pos[0]:.6f} {dep_pos[1]:.6f}")
    for ident, kind, via, lat, lon in enroute:
        lines.append(f"{ENTRY_TYPE[kind]} {ident} {via} 0.000000 {lat:.6f} {lon:.6f}")
    lines.append(f"1 {dest} ADES 0.000000 {dest_pos[0]:.6f} {dest_pos[1]:.6f}")
    return "\n".join(lines) + "\n"
