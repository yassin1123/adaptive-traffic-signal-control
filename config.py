"""
config.py — Adaptive Traffic Signal Control
============================================

SINGLE SOURCE OF TRUTH for every tunable number in the system (spec §6).
No magic numbers live anywhere else in the codebase. To tune behaviour, edit
HERE and nowhere else.

Units: time is in SECONDS. Rates are events-per-second. One simulation step is
TIME_STEP seconds (default 1s), so a "step" and a "second" coincide by default.

Phase model (LOCKED, spec §2) — a standard 4-way (+) intersection:

    Phase 1 : N–S through traffic   + E–W pedestrians cross (concurrent)
    Phase 2 : N–S protected turns   (pedestrians STOPPED)
    Phase 3 : E–W through traffic   + N–S pedestrians cross (concurrent)
    Phase 4 : E–W protected turns   (pedestrians STOPPED)

SAFETY: a protected-turn phase (2 or 4) may NEVER start green immediately after
pedestrians were walking. A CLEARANCE_INTERVAL is inserted by the simulation
between any pedestrian-walking phase and a following turn phase. See sim.py.
"""

from dataclasses import dataclass, field
from typing import Dict


# Phase identifiers (LOCKED). Phases 1 & 3 carry concurrent pedestrians;
# phases 2 & 4 are the protected-turn phases (pedestrians stopped).
PHASES = (1, 2, 3, 4)
PED_PHASES = (1, 3)        # phases during which pedestrians walk
TURN_PHASES = (2, 4)       # protected-turn phases — require clearance after peds


@dataclass
class Config:
    # ---- Green-time bounds ------------------------------------------------
    # MIN_GREEN is per-phase. Phases 1 & 3 carry pedestrians, so their minimum
    # MUST cover the time to physically cross the road (hard pedestrian-safety
    # floor). Phases 2 & 4 (turns, no peds) get a shorter vehicle minimum.
    MIN_GREEN: Dict[int, float] = field(default_factory=lambda: {
        1: 7.0,   # N–S through  — >= pedestrian crossing time (PED_CROSS_TIME)
        2: 4.0,   # N–S turns    — vehicle minimum
        3: 7.0,   # E–W through  — >= pedestrian crossing time (PED_CROSS_TIME)
        4: 4.0,   # E–W turns    — vehicle minimum
    })
    MAX_GREEN: float = 30.0          # ceiling on any single green

    # Pedestrian crossing time — the physical floor for ped phases. MIN_GREEN
    # for phases 1 & 3 must be >= this. Asserted at import (see bottom of file).
    PED_CROSS_TIME: float = 7.0

    # ---- Anti-starvation ---------------------------------------------------
    # If a phase's CAR demand has waited longer than MAX_WAIT, it jumps to the
    # front regardless of score. Guarantees no car waits forever.
    MAX_WAIT: float = 60.0
    # Dedicated, TIGHTER ceiling for pedestrians. This is a HARD STRUCTURAL bound
    # on pedestrian wait that holds no matter how the controllers weight cars in
    # their objective — pedestrians are numerically few, so an objective term
    # alone gives them weak pull; this ceiling guarantees they are never traded
    # away for car-wait gains. Enforced identically by every adaptive controller
    # via the shared anti-starvation function in controllers.py.
    PED_MAX_WAIT: float = 30.0

    # ---- Pedestrian clearance interval (NON-NEGOTIABLE SAFETY) -------------
    # Time inserted between a pedestrian-walking phase ending and a conflicting
    # protected-turn phase getting green. No new peds start; those in the
    # crosswalk finish. Inserted structurally by the sim, never by a controller.
    CLEARANCE_INTERVAL: float = 3.0

    # ---- Discharge / flow --------------------------------------------------
    # Cars discharged per second of green for an active phase (saturation flow).
    SATURATION_RATE: float = 1.5

    # ---- Heuristic scoring weights (controller 3b) ------------------------
    #   score = demand * DEMAND_WEIGHT + time_waited * URGENCY_WEIGHT
    DEMAND_WEIGHT: float = 1.0
    URGENCY_WEIGHT: float = 0.5

    # ---- Optimizer (controller 3c, Phase 2) --------------------------------
    # Receding-horizon look-ahead (seconds). Tuned: must comfortably exceed
    # MAX_GREEN so the rollout can see a long green's full payoff (a shorter
    # horizon truncates it and biases the optimizer toward over-switching). At
    # HORIZON=45 the optimizer beats the heuristic on light/rush/asymmetric.
    HORIZON: float = 45.0

    # The optimizer deviates from the heuristic's decision ONLY when its forward
    # rollout predicts a gain over the heuristic of at least OPT_MIN_PREDICTED_GAIN
    # car-seconds AND at least OPT_DEVIATION_MARGIN (fraction); otherwise it defers
    # to the proven heuristic. The ABSOLUTE gain threshold is what matters under
    # light load: there, queue differences are tiny so no plan clears the bar and
    # the optimizer simply mirrors the heuristic (never losing to it, rule #7);
    # under heavy load the predicted gains are large and it commits to the better
    # plan. (Predicted-wait units are car-seconds integrated over HORIZON.)
    OPT_MIN_PREDICTED_GAIN: float = 8.0
    OPT_DEVIATION_MARGIN: float = 0.02

    # ---- Fixed-timer baseline (controller 3a) ------------------------------
    # Pre-set green per phase, served regardless of demand. The "dumb" benchmark.
    FIXED_GREEN: Dict[int, float] = field(default_factory=lambda: {
        1: 15.0,
        2: 8.0,
        3: 15.0,
        4: 8.0,
    })

    # ---- Simulation --------------------------------------------------------
    TIME_STEP: float = 1.0           # seconds per simulation step
    RNG_SEED: int = 42               # seeded for reproducibility
    SIM_DURATION: float = 3600.0     # total simulated seconds per run

    # ---- Grid (Phase 3) ----------------------------------------------------
    GRID_TRAVEL_TIME: float = 5.0    # travel delay on a road segment between
                                     # two neighbouring intersections (seconds).
                                     # Free-flow travel — NOT counted as wait.
    # When a car is discharged toward a neighbouring intersection it is assigned
    # a fresh movement for that next intersection from this split (a car mostly
    # continues straight, sometimes turns). Routed cars only; external arrivals
    # use the scenario's own per-movement rates.
    ROUTING_SPLIT: Dict[str, float] = field(default_factory=lambda: {
        "through": 0.70, "left": 0.15, "right": 0.15,
    })
    # Finite holding capacity of one inter-intersection road segment (cars). A
    # real road has finite length: in-transit cars PLUS the queue spilling back
    # from the downstream intersection share this storage. When a segment is full,
    # the UPSTREAM intersection cannot discharge onto it even on green — that is
    # spillback / gridlock propagation. Set so a healthy (optimized) grid never
    # fills a segment, while a gridlocked (fixed) grid does. NB: the segment is a
    # delay line of GRID_TRAVEL_TIME, so at saturation ~SATURATION_RATE*TRAVEL
    # cars are in transit even with no queue.
    SEGMENT_CAPACITY: int = 20

    # ---- Traffic scenarios -------------------------------------------------
    # Arrival rates in cars/second per (approach, movement) and pedestrians/sec
    # per pedestrian group. Low rates => modelled as a Bernoulli trial per step
    # (see arrival generation). Movements: 'through', 'left', 'right'.
    #
    # 'ped' is the per-step arrival rate for EACH pedestrian group
    # (EW_peds in phase 1, NS_peds in phase 3).
    SCENARIOS: Dict[str, dict] = field(default_factory=lambda: {
        # Light traffic — everything clears easily.
        "light": {
            "N": {"through": 0.04, "left": 0.01, "right": 0.01},
            "S": {"through": 0.04, "left": 0.01, "right": 0.01},
            "E": {"through": 0.04, "left": 0.01, "right": 0.01},
            "W": {"through": 0.04, "left": 0.01, "right": 0.01},
            "ped": 0.01,
        },
        # Rush hour — heavy on all approaches. Each through-phase's demand
        # (~0.48 cars/s for N+S through+right) is pushed ABOVE the fixed timer's
        # per-axis capacity (15s green of a 52s cycle * SATURATION_RATE ~= 0.43
        # cars/s, i.e. ~111% utilised). The fixed timer can no longer clear its
        # through queue within a cycle, so it backs up across cycles (genuine
        # gridlock). The adaptive controllers reallocate green to demand and skip
        # the lightly-used turn phases, keeping their utilisation below 100%.
        "rush": {
            "N": {"through": 0.185, "left": 0.04, "right": 0.04},
            "S": {"through": 0.185, "left": 0.04, "right": 0.04},
            "E": {"through": 0.185, "left": 0.04, "right": 0.04},
            "W": {"through": 0.185, "left": 0.04, "right": 0.04},
            "ped": 0.02,
        },
        # Asymmetric — N–S axis oversaturated (~0.52 cars/s, ~120% of fixed's
        # per-axis capacity), E–W nearly empty. The fixed timer still spends ~44%
        # of every cycle on the deserted E–W axis while N–S gridlocks; the
        # adaptive controllers pour green into N–S and skip the empty E–W phases,
        # so for them the single busy axis is far below saturation. This is where
        # rigid timing is most obviously, measurably wasteful.
        "asymmetric": {
            "N": {"through": 0.185, "left": 0.04, "right": 0.04},
            "S": {"through": 0.185, "left": 0.04, "right": 0.04},
            "E": {"through": 0.02, "left": 0.005, "right": 0.005},
            "W": {"through": 0.02, "left": 0.005, "right": 0.005},
            "ped": 0.015,
        },
    })


# The single shared config instance used across the build.
DEFAULT = Config()


# ---- Safety invariants checked at import (fail fast) ----------------------
for _p in PED_PHASES:
    assert DEFAULT.MIN_GREEN[_p] >= DEFAULT.PED_CROSS_TIME, (
        f"MIN_GREEN[{_p}] must cover PED_CROSS_TIME for pedestrian safety"
    )
assert DEFAULT.CLEARANCE_INTERVAL > 0, "CLEARANCE_INTERVAL must be positive (safety)"
assert all(DEFAULT.MIN_GREEN[p] <= DEFAULT.MAX_GREEN for p in PHASES), \
    "MIN_GREEN must not exceed MAX_GREEN"
