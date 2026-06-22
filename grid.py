"""
grid.py — Phase 3: a 2x2 grid of independent intersections.

The grid is JUST four Intersection instances wired together by roads. There is
NO central coordinator and NO grid-wide joint optimization (spec §0/§4): each
intersection runs its own controller on its own LOCAL state. Coordination across
the grid is EMERGENT — cars discharged by one intersection become arrivals at its
neighbour (after a travel delay), so each neighbour's controller reacts to the
platoons the others produce. Nothing here solves a global objective.

Layout (rows x cols), neighbours share a road:

        (ext N)    (ext N)
           |          |
   (ext W)-A----------B-(ext E)
           |          |
   (ext W)-C----------D-(ext E)
           |          |
        (ext S)    (ext S)

Each corner intersection has 2 EXTERNAL approaches (grid boundary — fed by the
seeded arrival schedule) and 2 INTERNAL approaches (fed only by a neighbour's
discharged traffic). Cars leaving an outer edge simply exit the grid.
"""

import argparse
import random
from collections import defaultdict
from dataclasses import dataclass
from typing import Dict, List

from config import DEFAULT
from sim import Car, Ped, Intersection, APPROACHES, MOVEMENTS
from controllers import (FixedTimerController, HeuristicController,
                         OptimizerController)


# ---------------------------------------------------------------------------
# Geometry (LOCKED for a 2x2 grid)
# ---------------------------------------------------------------------------
POS = {"A": (0, 0), "B": (0, 1), "C": (1, 0), "D": (1, 1)}
AT = {pos: name for name, pos in POS.items()}

# The compass heading a car travels given the approach it arrived on (approach
# 'N' = arrived from the north = travelling South, etc.).
HEADING_OF_APPROACH = {"N": "S", "S": "N", "E": "W", "W": "E"}
# A turn rotates the travel heading.
LEFT_TURN = {"S": "E", "N": "W", "W": "S", "E": "N"}
RIGHT_TURN = {"S": "W", "N": "E", "W": "N", "E": "S"}
# Which approach a car ENTERS the neighbour on, given its travel heading
# (travelling South -> it reaches the neighbour from the north -> enters 'N').
ENTER_APPROACH = {"S": "N", "N": "S", "E": "W", "W": "E"}
# Row/col delta for each travel heading.
DELTA = {"N": (-1, 0), "S": (1, 0), "E": (0, 1), "W": (0, -1)}


def final_heading(approach: str, movement: str) -> str:
    """The compass direction a car travels after executing its movement."""
    h = HEADING_OF_APPROACH[approach]
    if movement == "left":
        return LEFT_TURN[h]
    if movement == "right":
        return RIGHT_TURN[h]
    return h  # through


def neighbour_of(name: str, heading: str):
    """Name of the intersection a car reaches travelling `heading` from `name`,
    or None if it leaves the grid."""
    r, c = POS[name]
    dr, dc = DELTA[heading]
    return AT.get((r + dr, c + dc))


def is_external_approach(name: str, approach: str) -> bool:
    """An approach is external (boundary, fed by the schedule) if there is no
    neighbour on the side the cars arrive FROM."""
    return neighbour_of(name, approach) is None


# ---------------------------------------------------------------------------
# Grid scheduler — seeded EXTERNAL arrivals (identical for every controller)
# ---------------------------------------------------------------------------
def generate_grid_schedule(scenario: str, cfg, seed=None):
    """External arrivals per intersection, by step. Only EXTERNAL approaches get
    scheduled cars (internal approaches fill from neighbour outflow). Pedestrians
    are local to each intersection. Identical traffic for every controller, so the
    grid comparison stays honest; internal flow differs by controller — that IS
    the emergent-coordination effect we want to measure."""
    rng = random.Random((cfg.RNG_SEED if seed is None else seed) ^ 0x9E37)
    rates = cfg.SCENARIOS[scenario]
    n_steps = int(cfg.SIM_DURATION / cfg.TIME_STEP)
    sched = {name: [[] for _ in range(n_steps)] for name in POS}

    for i in range(n_steps):
        now = i * cfg.TIME_STEP
        for name in POS:
            for approach in APPROACHES:
                if not is_external_approach(name, approach):
                    continue
                for movement in MOVEMENTS:
                    if rng.random() < rates[approach][movement] * cfg.TIME_STEP:
                        sched[name][i].append(Car(now, approach, movement))
            for group in ("EW_peds", "NS_peds"):
                if rng.random() < rates["ped"] * cfg.TIME_STEP:
                    sched[name][i].append(Ped(now, group))
    return sched


# ---------------------------------------------------------------------------
# The grid
# ---------------------------------------------------------------------------
@dataclass
class GridResult:
    avg_car_wait: float          # grid-wide delay over ALL arrived car-passages
    avg_car_wait_done: float     # over discharged car-passages only
    avg_ped_wait: float
    max_ped_wait: float
    passages_done: int           # car discharge events across all intersections
    cars_left: int               # cars still queued grid-wide at sim end
    per_intersection: Dict[str, float]   # avg car wait (all) per intersection
    peak_segment: Dict[str, int]         # peak occupancy per internal road segment
    segment_capacity: int                # the finite holding capacity
    spillback_events: int                # times a full segment blocked an upstream green

    def __str__(self) -> str:
        per = "  ".join(f"{n}={w:.1f}" for n, w in sorted(self.per_intersection.items()))
        worst_seg = max(self.peak_segment.values()) if self.peak_segment else 0
        return (f"avg_car_wait={self.avg_car_wait:7.2f}s  "
                f"avg_ped_wait={self.avg_ped_wait:6.2f}s  "
                f"max_ped_wait={self.max_ped_wait:6.1f}s  "
                f"passages={self.passages_done:5d}  cars_left={self.cars_left:4d}\n"
                f"                per-intersection avg wait: {per}\n"
                f"                peak road-segment occupancy: {worst_seg}/"
                f"{self.segment_capacity}  spillback_events={self.spillback_events}")


class Grid:
    """Four independent intersections wired by roads. Emergent coordination only."""

    def __init__(self, controller_factory, cfg):
        self.cfg = cfg
        # Each intersection gets its OWN controller instance — no shared state.
        self.inters: Dict[str, Intersection] = {
            name: Intersection(controller_factory(cfg), cfg, name) for name in POS
        }
        # Movement reassignment for routed cars is seeded for reproducibility.
        self.rng = random.Random(cfg.RNG_SEED ^ 0x5151)
        self._movements = list(cfg.ROUTING_SPLIT.keys())
        self._weights = list(cfg.ROUTING_SPLIT.values())
        # All INTERNAL directed road segments (name, heading) -> a neighbour.
        self.segments = [(name, h) for name in POS for h in ("N", "S", "E", "W")
                         if neighbour_of(name, h) is not None]
        self.in_transit = {seg: 0 for seg in self.segments}   # cars on the road now
        self.peak_seg = {seg: 0 for seg in self.segments}      # peak occupancy seen
        self.spillback_events = 0

    def _downstream_q(self, seg) -> int:
        """Cars queued at the downstream approach this segment feeds — i.e. the
        queue physically backed up onto (the far end of) this road segment."""
        name, heading = seg
        nbr = neighbour_of(name, heading)
        enter = ENTER_APPROACH[heading]
        inter = self.inters[nbr]
        return sum(len(inter.car_q[(enter, m)]) for m in MOVEMENTS)

    def _occupancy(self, seg) -> int:
        """Total cars occupying a segment: those in transit PLUS those queued at
        its downstream end (the queue extends back along the road)."""
        return self.in_transit[seg] + self._downstream_q(seg)

    def _route(self, from_name, car, current_step, travel_steps, n_steps, future):
        """Send a just-discharged car onto its outbound segment toward the
        neighbour (or off-grid). The car's wait was recorded at discharge; the
        free-flow travel time is NOT counted as wait."""
        heading = final_heading(car.approach, car.movement)
        nbr = neighbour_of(from_name, heading)
        if nbr is None:
            return  # exits the grid (off-map road, unbounded)
        arrival_step = current_step + travel_steps
        if arrival_step >= n_steps:
            return  # would arrive after the sim ends
        seg = (from_name, heading)
        new_movement = self.rng.choices(self._movements, weights=self._weights)[0]
        new_car = Car(arrival_time=arrival_step * self.cfg.TIME_STEP,
                      approach=ENTER_APPROACH[heading], movement=new_movement)
        future[arrival_step].append((nbr, new_car, seg))
        self.in_transit[seg] += 1

    def _make_reserve(self, from_name, room):
        """Backpressure for one intersection: a car may discharge onto a segment
        only if that segment has room (capacity not yet reached). Off-grid exits
        are always allowed. Reserving consumes a slot for this step so several
        cars don't oversubscribe the same segment."""
        def reserve(approach, movement):
            heading = final_heading(approach, movement)
            if neighbour_of(from_name, heading) is None:
                return True                      # leaving the grid — no limit
            seg = (from_name, heading)
            if room[seg] > 0:
                room[seg] -= 1
                return True
            self.spillback_events += 1           # full segment blocks this green
            return False
        return reserve

    # -- incremental driving (so visualize.py can step + render, owning no logic) --
    def begin(self, external_schedule) -> None:
        """Arm the grid to be advanced one step at a time via step_once()."""
        self._sched = external_schedule
        self._n_steps = len(next(iter(external_schedule.values())))
        self._travel_steps = max(1, int(round(self.cfg.GRID_TRAVEL_TIME / self.cfg.TIME_STEP)))
        self._future = defaultdict(list)   # step -> [(inter_name, Car, seg)]
        self._i = 0

    def step_once(self) -> bool:
        """Advance the whole grid by ONE TIME_STEP. ALL traffic logic lives here.
        Returns False once the schedule is exhausted."""
        cfg = self.cfg
        if self._i >= self._n_steps:
            return False
        i = self._i
        now = i * cfg.TIME_STEP
        # external arrivals (boundary inflow + each intersection's peds)
        for name, sched in self._sched.items():
            for unit in sched[i]:
                if isinstance(unit, Car):
                    self.inters[name].add_car(unit)
                else:
                    self.inters[name].add_ped(unit)
        # internal arrivals routed from neighbours, due this step (they leave the
        # road and join the downstream queue)
        for (name, car, seg) in self._future.pop(i, ()):
            self.inters[name].add_car(car)
            self.in_transit[seg] -= 1
        # snapshot remaining room on each segment for this step's backpressure
        room = {seg: cfg.SEGMENT_CAPACITY - self._occupancy(seg) for seg in self.segments}
        # step every intersection independently (with backpressure), then route
        for name, inter in self.inters.items():
            discharged = inter.step(now, self._make_reserve(name, room))
            for car in discharged:
                self._route(name, car, i, self._travel_steps, self._n_steps, self._future)
        # record peak occupancy per segment (spillback fills show up here)
        for seg in self.segments:
            occ = self._occupancy(seg)
            if occ > self.peak_seg[seg]:
                self.peak_seg[seg] = occ
        self._i += 1
        return True

    def run(self, external_schedule) -> GridResult:
        self.begin(external_schedule)
        while self.step_once():
            pass
        return self._collect(self._n_steps * self.cfg.TIME_STEP)

    # -- live accessors for the visualizer (read-only views of module state) --
    def now(self) -> float:
        return self._i * self.cfg.TIME_STEP

    def live_avg_car_wait(self) -> float:
        """Avg delay over ALL arrived cars so far (done + currently-queued partial)
        — the same honest metric the results table uses, evaluated live."""
        end = self.now()
        waits = []
        for inter in self.inters.values():
            waits.extend(inter.all_car_waits(end))
        return sum(waits) / len(waits) if waits else 0.0

    def cars_stranded(self) -> int:
        return sum(inter.cars_remaining() for inter in self.inters.values())

    def segment_occupancy(self, seg) -> int:
        return self._occupancy(seg)

    def _collect(self, end_time) -> GridResult:
        car_waits_all, car_waits_done, ped_waits = [], [], []
        per_inter = {}
        cars_left = 0
        max_ped = 0.0
        for name, inter in self.inters.items():
            all_w = inter.all_car_waits(end_time)
            car_waits_all.extend(all_w)
            car_waits_done.extend(inter.car_waits)
            ped_waits.extend(inter.ped_waits)
            cars_left += inter.cars_remaining()
            max_ped = max(max_ped, inter.max_ped_wait())
            per_inter[name] = (sum(all_w) / len(all_w)) if all_w else 0.0

        def avg(xs):
            return sum(xs) / len(xs) if xs else 0.0

        # peak occupancy keyed by a readable segment label, e.g. "A->B"
        peak_named = {}
        for (name, heading), occ in self.peak_seg.items():
            peak_named[f"{name}->{neighbour_of(name, heading)}"] = occ

        return GridResult(
            avg_car_wait=avg(car_waits_all),
            avg_car_wait_done=avg(car_waits_done),
            avg_ped_wait=avg(ped_waits),
            max_ped_wait=max_ped,
            passages_done=len(car_waits_done),
            cars_left=cars_left,
            per_intersection=per_inter,
            peak_segment=peak_named,
            segment_capacity=self.cfg.SEGMENT_CAPACITY,
            spillback_events=self.spillback_events,
        )


# ---------------------------------------------------------------------------
# Phase 3 comparison
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="Phase 3: 2x2 grid comparison")
    parser.add_argument("--scenario", default="rush",
                        choices=list(DEFAULT.SCENARIOS.keys()))
    args = parser.parse_args()

    cfg = DEFAULT
    scenario = args.scenario
    schedule = generate_grid_schedule(scenario, cfg)

    print(f"\n=== PHASE 3 - 2x2 grid - scenario: {scenario} ===")
    print(f"seed={cfg.RNG_SEED}  duration={cfg.SIM_DURATION:.0f}s  "
          f"travel_between={cfg.GRID_TRAVEL_TIME:.0f}s  (no central solver)\n")

    factories = {
        "fixed-timer": FixedTimerController,
        "heuristic": HeuristicController,
        "optimized": OptimizerController,
    }
    results = {}
    for label, factory in factories.items():
        results[label] = Grid(factory, cfg).run(schedule)
        print(f"  {label:11s}: {results[label]}")

    base = results["fixed-timer"].avg_car_wait
    if base > 0:
        h = 100.0 * (base - results["heuristic"].avg_car_wait) / base
        o = 100.0 * (base - results["optimized"].avg_car_wait) / base
        print(f"\n  grid-wide vs fixed:  heuristic {h:+.1f}%   optimized {o:+.1f}%")

    opt = results["optimized"]
    heur = results["heuristic"]
    fixed = results["fixed-timer"]
    # Spec Phase 3 gate: optimized beats the FIXED timer grid-wide, and the grid
    # is stable (not backing up unboundedly).
    beats_fixed = opt.avg_car_wait < fixed.avg_car_wait
    stable = opt.cars_left < 0.05 * opt.passages_done
    # Rule #7 (reported, not gated under light load): optimized must never
    # meaningfully lose to the heuristic. Deferral guarantees ~<= heuristic.
    not_worse_than_heur = opt.avg_car_wait <= heur.avg_car_wait * 1.01

    # Spillback: did the OPTIMIZED grid ever fill an inter-intersection road and
    # block an upstream green? Genuine stability means no (or negligible) spillback.
    opt_peak = max(opt.peak_segment.values()) if opt.peak_segment else 0
    no_spillback = opt.spillback_events == 0

    print(f"\n  per-segment peak occupancy (optimized): " +
          "  ".join(f"{s}={o}" for s, o in sorted(opt.peak_segment.items())))
    print(f"  per-segment peak occupancy (fixed)    : " +
          "  ".join(f"{s}={o}" for s, o in sorted(fixed.peak_segment.items())))
    print(f"\n  GATE (optimized beats fixed, grid-wide): "
          f"{'PASS' if beats_fixed else 'FAIL'}")
    print(f"  GATE (optimized grid stable): {'PASS' if stable else 'FAIL'} "
          f"(cars_left={opt.cars_left}, passages={opt.passages_done})")
    print(f"  GATE (no spillback in optimized grid): "
          f"{'PASS' if no_spillback else 'FAIL'} "
          f"(peak {opt_peak}/{opt.segment_capacity}, "
          f"spillback_events={opt.spillback_events}; "
          f"fixed had {fixed.spillback_events})")
    print(f"  CHECK (optimized not worse than heuristic, rule #7): "
          f"{'PASS' if not_worse_than_heur else 'FAIL'}")
    return beats_fixed and stable and not_worse_than_heur and no_spillback


if __name__ == "__main__":
    main()
