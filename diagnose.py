"""
diagnose.py — investigative harness (NOT a deliverable).

Answers the reviewer's question: under each scenario, does the FIXED-TIMER
baseline actually congest (queues back up across cycles), or do queues clear
within one cycle so the baseline is never really stressed?

Reports, per scenario per controller: peak total queue, time-average queue,
max single-car wait, and cars left unserved at the end.
"""

from config import DEFAULT
from sim import Car, Ped, Intersection, APPROACHES, MOVEMENTS
from controllers import FixedTimerController, HeuristicController
from run import generate_schedule


def run_instrumented(controller, schedule, cfg):
    inter = Intersection(controller, cfg)
    peak_q = 0
    q_area = 0.0  # integral of queue over time -> time-average queue
    max_wait = 0.0
    for i, arrivals in enumerate(schedule):
        now = i * cfg.TIME_STEP
        for u in arrivals:
            inter.add_car(u) if isinstance(u, Car) else inter.add_ped(u)
        inter.step(now)
        q = inter.cars_remaining()
        peak_q = max(peak_q, q)
        q_area += q * cfg.TIME_STEP
        if inter.car_waits:
            max_wait = max(max_wait, inter.car_waits[-1])
    end_time = len(schedule) * cfg.TIME_STEP
    avg_q = q_area / cfg.SIM_DURATION
    return {
        "avg_wait": inter.avg_total_car_wait(end_time),   # delay over ALL arrived cars
        "avg_done": inter.avg_car_wait(),                  # discharged cars only
        "peak_q": peak_q,
        "avg_q": avg_q,
        "max_wait": max_wait,
        "left": inter.cars_remaining(),
        "done": len(inter.car_waits),
    }


def main():
    cfg = DEFAULT
    for scenario in cfg.SCENARIOS:
        schedule = generate_schedule(scenario, cfg)
        total = sum(1 for s in schedule for u in s if isinstance(u, Car))
        print(f"\n=== {scenario}  (total cars={total}, "
              f"arrival={total/cfg.SIM_DURATION:.3f}/s) ===")
        for label, ctrl in (("fixed", FixedTimerController(cfg)),
                            ("heuristic", HeuristicController(cfg))):
            r = run_instrumented(ctrl, schedule, cfg)
            print(f"  {label:9s}  avg_wait(all)={r['avg_wait']:7.2f}s  "
                  f"avg(done)={r['avg_done']:6.2f}s  peak_q={r['peak_q']:4d}  "
                  f"avg_q={r['avg_q']:5.1f}  max_wait={r['max_wait']:6.1f}s  "
                  f"left={r['left']:4d}")


if __name__ == "__main__":
    main()
