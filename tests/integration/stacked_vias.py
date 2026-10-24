#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Stacked laser vias end to end (doc 05 §21.1): route, write, and KiCad's DRC on what was written.

  stacked_vias.py <tracemaker> <work dir>

Writes a six-layer board (GND planes on In1 and In4, the sensor_ts JLC HDI rules) on which net S1 must go F.Cu -> In2 -> F.Cu
and net S2 B.Cu -> In3 -> B.Cu round keep-outs, routes it with --stacked-vias and the plane-void pitches, and checks:
  * every connection is routed with four stacks: eight micro vias, in pairs at one point (F.Cu-In1 + In1-In2,
    B.Cu-In4 + In4-In3), written with (capping yes) (filling yes), no track on In1 or In4;
  * TraceMaker's DRC after a refill finds no violation type more often than on the input, and nothing unconnected;
  * the same holds for KiCad's DRC with --refill-zones (scripts/kicad-cli: KiCad 10.0.6); without kicad-cli the test is
    skipped (77), never passed.
"""
import json
import pathlib
import shutil
import subprocess
import sys
from collections import Counter

HDI_RULES = """(version 1)
(rule "jlc-hdi-microvia-hole-to-hole" (condition "A.Via_Type == 'Micro' || B.Via_Type == 'Micro'") (constraint hole_to_hole (min 0.24mm)))
(rule "jlc-hdi-microvia-edge" (condition "A.Via_Type == 'Micro'") (constraint edge_clearance (min 0.35mm)))
(rule "s1-top-side" (layer "In3.Cu") (condition "A.NetName == 'S1'") (constraint disallow track))
(rule "s2-bottom-side" (layer "In2.Cu") (condition "A.NetName == 'S2'") (constraint disallow track))
"""

PROJECT = {
    "board": {"design_settings": {"rules": {
        "min_clearance": 0.15, "min_track_width": 0.09, "min_hole_clearance": 0.25, "min_hole_to_hole": 0.2,
        "min_copper_edge_clearance": 0.2, "min_microvia_diameter": 0.3, "min_microvia_drill": 0.15, "min_via_diameter": 0.45,
        "min_through_hole_diameter": 0.3, "min_via_annular_width": 0.075}}},
    "net_settings": {"classes": [{"name": "Default", "clearance": 0.15, "track_width": 0.15, "via_diameter": 0.45, "via_drill": 0.3,
                                  "microvia_diameter": 0.3, "microvia_drill": 0.15, "diff_pair_width": 0.15, "diff_pair_gap": 0.2}]},
}


def pad(ref, x, y, layer, net, name):
    return (f'  (footprint "P" (layer "{layer}") (at {x} {y}) (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS"))\n'
            f'    (pad "1" smd rect (at 0 0) (size 0.6 0.6) (layers "{layer}") (net {net} "{name}")))\n')


def keepout(layer, x0, y0, x1, y1):
    return (f'  (zone (net 0) (net_name "") (layer "{layer}") (hatch edge 0.5) (connect_pads (clearance 0)) (min_thickness 0.25)\n'
            '    (keepout (tracks not_allowed) (vias not_allowed) (pads allowed) (copperpour allowed) (footprints allowed))\n'
            f'    (fill (thermal_gap 0.5) (thermal_bridge_width 0.5)) (polygon (pts (xy {x0} {y0}) (xy {x1} {y0}) (xy {x1} {y1}) (xy {x0} {y1}))))\n')


def plane(layer):
    return (f'  (zone (net 3) (net_name "GND") (layer "{layer}") (hatch edge 0.5) (connect_pads (clearance 0.2)) (min_thickness 0.2)\n'
            '    (fill yes (thermal_gap 0.5) (thermal_bridge_width 0.5) (island_removal_mode 1))\n'
            '    (polygon (pts (xy 0.5 0.5) (xy 19.5 0.5) (xy 19.5 11.5) (xy 0.5 11.5))))\n')


def board():
    return ('(kicad_pcb (version 20240108) (generator "pcbnew")\n'
            '  (general (thickness 1.6))\n'
            '  (layers (0 "F.Cu" signal) (1 "In1.Cu" signal) (2 "In2.Cu" signal) (3 "In3.Cu" signal) (4 "In4.Cu" signal)\n'
            '    (31 "B.Cu" signal) (37 "F.SilkS" user) (44 "Edge.Cuts" user))\n'
            '  (setup (pad_to_mask_clearance 0))\n'
            '  (net 0 "") (net 1 "S1") (net 2 "S2") (net 3 "GND")\n'
            '  (gr_rect (start 0 0) (end 20 12) (layer "Edge.Cuts") (stroke (width 0.1) (type solid)))\n'
            + pad("P1", 3, 4, "F.Cu", 1, "S1") + pad("P2", 17, 4, "F.Cu", 1, "S1")
            + pad("P3", 3, 8, "B.Cu", 2, "S2") + pad("P4", 17, 8, "B.Cu", 2, "S2")
            + keepout("F.Cu", 8, 0, 12, 12) + keepout("B.Cu", 8, 0, 12, 12) + plane("In1.Cu") + plane("In4.Cu") + ")\n")


def write(path, text):
    path.write_text(text)
    path.with_suffix(".kicad_pro").write_text(json.dumps(PROJECT))
    path.with_suffix(".kicad_dru").write_text(HDI_RULES)


def tm_drc(tm, pcb, out):
    subprocess.run([str(tm), "drc", str(pcb), "--refill-zones", "--json", str(out)], capture_output=True)
    return json.loads(out.read_text())


def kicad_drc(pcb, out):
    subprocess.run(["kicad-cli", "pcb", "drc", "--format", "json", "--severity-error", "--refill-zones", "-o", str(out), str(pcb)],
                   capture_output=True)
    return json.loads(out.read_text())


def worse(before, after):
    b, a = Counter(v["type"] for v in before["violations"]), Counter(v["type"] for v in after["violations"])
    return {t: (b[t], n) for t, n in a.items() if n > b[t]}


def main() -> int:
    tm, work = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2])
    work.mkdir(parents=True, exist_ok=True)
    src, out = work / "in.kicad_pcb", work / "out.kicad_pcb"
    write(src, board())
    shutil.copy(src.with_suffix(".kicad_pro"), out.with_suffix(".kicad_pro"))
    shutil.copy(src.with_suffix(".kicad_dru"), out.with_suffix(".kicad_dru"))
    r = subprocess.run([str(tm), "route", str(src), "-o", str(out), "--work", "2000000", "--time", "600", "--variants", "1", "--threads", "1",
                        "--no-kb", "--no-gpu", "--soft-zones", "--no-tracks-on", "In1.Cu,In4.Cu", "--stacked-vias", "--stack-pitch", "0.95",
                        "--stack-through-pitch", "1.0", "--json", str(work / "route.json")], capture_output=True, text=True)
    if r.returncode not in (0, 3):
        print(f"tracemaker route failed\n{r.stdout[-2000:]}\n{r.stderr[-2000:]}")
        return 1
    errors = []
    res = json.loads((work / "route.json").read_text())
    if res["routed"] != res["connections"] or res.get("stacked_vias") != 4:
        errors.append(f"routed {res['routed']}/{res['connections']}, {res.get('stacked_vias')} stacks (expected all and 4)")
    text = out.read_text()
    if text.count("(capping yes)") != 8 or text.count("(filling yes)") != 8:
        errors.append(f"{text.count('(capping yes)')} capped and {text.count('(filling yes)')} filled vias written (expected 8)")
    truth = work / "truth.json"
    subprocess.run([str(tm), "inspect", str(out), "--json", str(truth)], check=True, capture_output=True)
    b = json.loads(truth.read_text())
    spans = Counter((v["x"], v["y"]) for v in b["vias"])
    if len(b["vias"]) != 8 or any(n != 2 for n in spans.values()):
        errors.append(f"vias {len(b['vias'])} at {len(spans)} points (expected 8 in pairs)")
    if any(t["layer"] in ("In1.Cu", "In4.Cu") for t in b["tracks"]):
        errors.append("a track on a plane layer")

    own = worse(tm_drc(tm, src, work / "in.tm.json"), after := tm_drc(tm, out, work / "out.tm.json"))
    if own or after["unconnected_items"]:
        errors.append(f"TraceMaker DRC: more of {own}, {len(after['unconnected_items'])} unconnected")
    if shutil.which("kicad-cli"):
        before_k, after_k = kicad_drc(src, work / "in.kicad.json"), kicad_drc(out, work / "out.kicad.json")
        kw = worse(before_k, after_k)
        if kw or after_k["unconnected_items"]:
            errors.append(f"KiCad DRC: more of {kw}, {len(after_k['unconnected_items'])} unconnected: "
                          + "; ".join(v["description"] for v in after_k["violations"][:5]))
        print(f"KiCad DRC: {len(before_k['violations'])} errors before, {len(after_k['violations'])} after, "
              f"{len(after_k['unconnected_items'])} unconnected")
    else:
        print("kicad-cli not found: the KiCad half of this test cannot run (put scripts/ first on PATH)")
        for e in errors:
            print("FAIL:", e)
        return 1 if errors else 77
    print(f"routed {res['routed']}/{res['connections']}, {res.get('stacked_vias')} stacks, {len(b['vias'])} vias")
    for e in errors:
        print("FAIL:", e)
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
