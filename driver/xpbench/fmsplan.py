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
