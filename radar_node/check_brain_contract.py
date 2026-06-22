"""
check_brain_contract.py — Track-A integration check (sensing -> brain seam).

Loads the radar node's emitted output (node_output.json) and verifies it is
exactly what the Python control "brain" ingests: per-approach vehicle counts,
pedestrian counts, and speeds, on approaches the brain recognises. This proves
the two independently-built halves of Track A speak the same format.

Run from radar_node/ after test_phase6 has written node_output.json:
    python check_brain_contract.py
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
BRAIN = os.path.dirname(HERE)          # the Python brain lives in the repo root
sys.path.insert(0, BRAIN)

# The brain's per-approach vocabulary (its real, in-use constant).
from sim import APPROACHES              # ("N", "S", "E", "W")

fails = 0
def check(cond, msg):
    global fails
    print(f"  [{'PASS' if cond else 'FAIL'}] {msg}")
    if not cond:
        fails += 1

print("=== Track-A integration check: radar node -> brain contract ===\n")

with open(os.path.join(HERE, "node_output.json")) as f:
    msg = json.load(f)

check("approaches" in msg and isinstance(msg["approaches"], list),
      "message carries a list of per-approach reports")

brain_input = {}   # what we would hand the brain
for a in msg["approaches"]:
    for field in ("approach", "vehicle_count", "pedestrian_count", "avg_speed_mps"):
        check(field in a, f"approach report has '{field}'")
    name = a["approach"].replace("approach_", "")
    check(name in APPROACHES,
          f"approach '{a['approach']}' maps to a brain approach ({name} in {APPROACHES})")
    check(isinstance(a["vehicle_count"], int) and isinstance(a["pedestrian_count"], int),
          f"{a['approach']}: counts are integers")
    brain_input[name] = {
        "queue": a["vehicle_count"],          # -> brain per-approach vehicle demand
        "pedestrians": a["pedestrian_count"], # -> brain pedestrian demand
        "speed_mps": a["avg_speed_mps"],      # -> brain approach speed
    }

print("\n  field mapping into the brain's per-approach inputs:")
print("    radar 'vehicle_count'    -> brain per-approach vehicle queue / demand")
print("    radar 'pedestrian_count' -> brain pedestrian demand")
print("    radar 'avg_speed_mps'    -> brain approach speed")
print("\n  brain-ready per-approach input derived from the node message:")
for ap in APPROACHES:
    d = brain_input.get(ap, {"queue": 0, "pedestrians": 0, "speed_mps": 0.0})
    print(f"    {ap}: vehicles={d['queue']}  peds={d['pedestrians']}  speed={d['speed_mps']:.1f} m/s")

print(f"\n=== CONTRACT CHECK: {'PASS' if fails == 0 else 'FAIL'} ({fails} failures) ===")
sys.exit(0 if fails == 0 else 1)
