#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Fill Tables 3 and 4 of manuscript_p2_v0.md from results_numbers.csv.

Project rule: no number enters the manuscript unless it comes out of a script.
This file is the one that puts the table numbers there, so the tables are regenerated
whole -- every cell, not only the ones still marked [csv] -- and any cell that was
already in the file but disagrees with the CSV is reported on stderr before it is
overwritten.  Run results_numbers.py first.

Inputs   results_numbers.csv   (written by results_numbers.py)
Outputs  ../manuscript_p2_v0.md  rewritten in place (Tables 3 and 4 only)
         the two filled tables on stdout

Run:
    python3 fill_tables.py            # rewrite the manuscript
    python3 fill_tables.py --check    # print only, do not touch the manuscript
"""
import csv
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
CSV = os.path.join(HERE, "results_numbers.csv")
MS = os.path.join(HERE, "..", "manuscript_p2_v0.md")

# Site label in the manuscript -> site label in results_numbers.csv
SITES = [("Concrete yard", "concrete yard"),
         ("Orchard block", "orchard"),
         ("Greenhouse", "greenhouse")]
# Published-step classes of Table 4, in column order
STEPS = (0.05, 0.10, 0.20, 0.30, 0.50)
GATE_G = 17          # deg, the recommended gate (results_numbers.csv: gate/recommended g)

R = {}


def load():
    with open(CSV, encoding="utf-8") as f:
        for row in csv.DictReader(f):
            R[(row["group"], row["quantity"])] = row["value"]


def v(group, quantity):
    try:
        return R[(group, quantity)]
    except KeyError:
        sys.exit("missing key in results_numbers.csv: %s / %s\n"
                 "rerun results_numbers.py" % (group, quantity))


def f2(group, quantity):
    return "%.2f" % float(v(group, quantity))


def i(group, quantity):
    return "%d" % int(float(v(group, quantity)))


def table3():
    """Recordings used.  Hours are wall-clock, fixed-solution hours are status-4 frames x tau."""
    head = ["| Site | Recordings | Hours | Fixed-solution hours | With IMU | With raw GGA topic |",
            "|---|---|---|---|---|---|"]
    rows = []
    for label, s in SITES:
        rows.append("| %s | %s | %s | %s | %s | %s |" % (
            label,
            i("corpus_by_site", "%s bags" % s),
            f2("corpus_by_site", "%s hours" % s),
            f2("corpus_by_site", "%s fixed hours" % s),
            i("corpus_by_site", "%s bags with /ch110/data_raw" % s),
            i("corpus_by_site", "%s bags with /rtk/fix" % s)))
    rows.append("| All | %s | %s | %s | %s | %s |" % (
        i("corpus", "bags"), f2("corpus", "hours"), f2("corpus", "fixed_hours"),
        i("corpus", "bags_with_ch110"), i("corpus", "bags_with_rtk_fix")))
    return head + rows


def table4():
    """Pseudo-step frames by published-step size, and what each method sees.

    Row 2 is |c| of eq. (3) in the pre-jump body frame against the deployed 0.30 m
    cross-track threshold; row 3 is the published step against the deployed 0.50 m
    chord threshold; row 4 is the gate v2 replay at the recommended g, where "caught"
    means the gate held or withheld the frame.  The 0.05 m column of the two position
    rows is left as "--": the pseudo-step definition of Section 4.3 starts at 0.05 m,
    but neither deployed test is claimed for a class below its own threshold.
    """
    cols = "".join(" %.2f m |" % t for t in STEPS)
    head = ["| Published step ≥ |" + cols,
            "|---|" + "---|" * len(STEPS)]
    rows = ["| Frames |" + "".join(
        " %s |" % i("pseudo_step", "count >=%.2f m" % t) for t in STEPS)]
    rows.append("| Seen by cross-track test, T = 0.30 m |" + "".join(
        " %s |" % ("–" if t < 0.10 else
                   i("pseudo_step", "caught by cross-track 0.30 m >=%.2f m" % t))
        for t in STEPS))
    rows.append("| Seen by chord test, T = 0.50 m |" + "".join(
        " %s |" % ("–" if t < 0.10 else
                   i("pseudo_step", "caught by norm 0.50 m >=%.2f m" % t))
        for t in STEPS))
    rows.append("| Caught by heading gate, g = %d° |" % GATE_G + "".join(
        " %s |" % i("gate_v2", "at g=%d deg: pseudo-step frames caught >=%.2f m" % (GATE_G, t))
        for t in STEPS))
    return head + rows


def splice(lines, first_line_prefix, new_block):
    """Replace the markdown table whose header line starts with first_line_prefix."""
    for k, ln in enumerate(lines):
        if ln.startswith(first_line_prefix):
            j = k
            while j < len(lines) and lines[j].startswith("|"):
                j += 1
            old = lines[k:j]
            if len(old) == len(new_block):
                for a, b in zip(old, new_block):
                    if a != b and "[csv]" not in a:
                        print("  changed: %s\n        -> %s" % (a, b), file=sys.stderr)
            return lines[:k] + new_block + lines[j:], old
    sys.exit("table header not found in the manuscript: %s" % first_line_prefix)


def main():
    load()
    t3, t4 = table3(), table4()
    with open(MS, encoding="utf-8") as f:
        lines = f.read().split("\n")
    lines, _ = splice(lines, "| Site | Recordings |", t3)
    lines, _ = splice(lines, "| Published step ≥ |", t4)
    print("**Table 3.**")
    print("\n".join(t3))
    print("\n**Table 4.**")
    print("\n".join(t4))
    if "--check" in sys.argv:
        print("\n--check: manuscript not modified", file=sys.stderr)
        return
    with open(MS, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    print("\nwrote %s" % os.path.normpath(MS), file=sys.stderr)


if __name__ == "__main__":
    main()
