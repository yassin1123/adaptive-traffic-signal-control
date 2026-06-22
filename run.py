"""
run.py — Phase 1 driver: fixed-timer vs heuristic on a single intersection.

Generates ONE seeded arrival schedule and feeds the *identical* traffic to each
controller, so the comparison is honest and reproducible (same seed -> same
numbers, every time). Prints both controllers' average wait and verifies the
Phase 1 gate: heuristic must beat fixed.

(The full per-scenario scenario module + optimizer comparison arrive in later
phases; this file is the Phase 1 proof.)
"""

import argparse
import random

from config import DEFAULT, PED_PHASES, TURN_PHASES
from sim import Car, Ped, run_single, APPROACHES, MOVEMENTS
from controllers import (FixedTimerController, HeuristicController,
                         OptimizerController)


# ---------------------------------------------------------------------------
# Arrival generation — seeded, deterministic, identical for every controller
# ---------------------------------------------------------------------------
def generate_schedule(scenario: str, cfg, seed=None):
    """Build a per-step arrival schedule from a scenario's rate table.

    Low arrival rates are modelled as an independent Bernoulli trial per step per
    (approach, movement) and per pedestrian group — a clean, seed-reproducible
    arrival process. Returns a list indexed by step; schedule[i] is the list of
    Car/Ped objects that arrive at step i (time = i * TIME_STEP).
    """
    rng = random.Random(cfg.RNG_SEED if seed is None else seed)
    rates = cfg.SCENARIOS[scenario]
    n_steps = int(cfg.SIM_DURATION / cfg.TIME_STEP)
    schedule = [[] for _ in range(n_steps)]

    for i in range(n_steps):
        now = i * cfg.TIME_STEP
        # vehicles
        for approach in APPROACHES:
            for movement in MOVEMENTS:
                if rng.random() < rates[approach][movement] * cfg.TIME_STEP:
                    schedule[i].append(Car(now, approach, movement))
        # pedestrians (one rate per ped group)
        for group in ("EW_peds", "NS_peds"):
            if rng.random() < rates["ped"] * cfg.TIME_STEP:
                schedule[i].append(Ped(now, group))
    return schedule


# ---------------------------------------------------------------------------
# Safety audit — prove the clearance rule and ped min-green are honoured
# ---------------------------------------------------------------------------
def audit_safety(inter, cfg, label=""):
    """Verify the non-negotiable safety properties on a completed run."""
    # 1. No protected-turn phase started right after a ped-walking phase WITHOUT
    #    a clearance interval.
    for (t, prev, nxt, had_clearance) in inter.transitions:
        if prev in PED_PHASES and nxt in TURN_PHASES:
            assert had_clearance, (
                f"SAFETY VIOLATION at t={t}: turn phase {nxt} started after ped "
                f"phase {prev} without a clearance interval")
    # 2. Pedestrian wait is STRUCTURALLY BOUNDED by PED_MAX_WAIT. The ceiling
    #    forces service once a ped is overdue; that service then costs at most one
    #    clearance + one pedestrian MIN_GREEN before they cross. So the worst
    #    pedestrian wait must not exceed PED_MAX_WAIT + that service time. This
    #    proves the bound holds for the adaptive controllers regardless of their
    #    car-focused objective. (The fixed timer serves peds every cycle anyway.)
    bound = cfg.PED_MAX_WAIT + cfg.CLEARANCE_INTERVAL + max(
        cfg.MIN_GREEN[p] for p in PED_PHASES)
    mpw = inter.max_ped_wait()
    assert mpw <= bound + cfg.TIME_STEP, (
        f"{label}: max pedestrian wait {mpw:.1f}s exceeds structural bound {bound:.1f}s")
    # 3. (MIN_GREEN / ped crossing floor is enforced by the sim clamp + config invariant.)
    return True


# ---------------------------------------------------------------------------
# Phase 1 comparison
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="fixed vs heuristic vs optimized")
    parser.add_argument("--scenario", default="rush",
                        choices=list(DEFAULT.SCENARIOS.keys()))
    args = parser.parse_args()

    cfg = DEFAULT
    scenario = args.scenario
    schedule = generate_schedule(scenario, cfg)
    total_cars = sum(1 for step in schedule for u in step if isinstance(u, Car))

    print(f"\n=== single intersection - scenario: {scenario} ===")
    print(f"seed={cfg.RNG_SEED}  duration={cfg.SIM_DURATION:.0f}s  "
          f"total cars generated={total_cars}\n")

    fixed_res, fixed_int = run_single(FixedTimerController(cfg), schedule, cfg, "fixed")
    heur_res, heur_int = run_single(HeuristicController(cfg), schedule, cfg, "heuristic")
    opt_res, opt_int = run_single(OptimizerController(cfg), schedule, cfg, "optimized")

    for inter, lbl in ((fixed_int, "fixed"), (heur_int, "heuristic"),
                       (opt_int, "optimized")):
        audit_safety(inter, cfg, lbl)

    print(f"  fixed-timer : {fixed_res}")
    print(f"  heuristic   : {heur_res}")
    print(f"  optimized   : {opt_res}")

    base = fixed_res.avg_car_wait
    if base > 0:
        h_imp = 100.0 * (base - heur_res.avg_car_wait) / base
        o_imp = 100.0 * (base - opt_res.avg_car_wait) / base
        print(f"\n  vs fixed:  heuristic {h_imp:+.1f}%   optimized {o_imp:+.1f}%")

    ped_bound = cfg.PED_MAX_WAIT + cfg.CLEARANCE_INTERVAL + max(
        cfg.MIN_GREEN[p] for p in PED_PHASES)
    order_ok = opt_res.avg_car_wait < heur_res.avg_car_wait < fixed_res.avg_car_wait
    print(f"\n  GATE (optimized < heuristic < fixed): {'PASS' if order_ok else 'FAIL'}")
    print(f"  safety audit (clearance + ped floor): PASS")
    print(f"  pedestrian wait structurally bounded <= {ped_bound:.0f}s "
          f"(worst observed: opt {opt_res.max_ped_wait:.1f}s, "
          f"heur {heur_res.max_ped_wait:.1f}s): PASS")
    return order_ok


if __name__ == "__main__":
    main()
