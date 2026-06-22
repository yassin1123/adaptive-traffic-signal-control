"""
controllers.py — the three swappable signal controllers.

All controllers implement the SAME interface so they are interchangeable and
comparable on identical traffic:

    class Controller:
        def __init__(self, cfg): ...
        def decide(self, observation) -> Decision

A controller owns ONE intersection's timing decision. It is handed a LOCAL
`Observation` (per-phase demand + wait time only — what a radar node provides)
and returns which phase to serve next and for how long. The simulation, NOT the
controller, enforces the pedestrian clearance interval and "one phase at a time".

Built in this file:
  * FixedTimerController   — 3a, the dumb benchmark to beat.
  * HeuristicController     — 3b, the SAFETY NET (must beat fixed; kept forever).
  * (OptimizerController arrives in Phase 2.)
"""

from dataclasses import dataclass

from config import PHASES, PED_PHASES, TURN_PHASES


@dataclass
class Decision:
    """A controller's output: serve `phase` for `duration` seconds of green.
    The sim clamps duration to [MIN_GREEN[phase], MAX_GREEN] and inserts any
    required clearance interval — controllers never touch those safety bits."""
    phase: int
    duration: float


def anti_starvation_phase(obs, cfg):
    """SHARED structural anti-starvation, used identically by EVERY adaptive
    controller (heuristic and optimizer) so the guarantee can't drift between
    them or get re-patched per scenario.

    Two independent ceilings:
      * cars must not wait past MAX_WAIT,
      * pedestrians must not wait past the tighter PED_MAX_WAIT.

    Any phase whose car OR pedestrian demand is overdue is forced to the front;
    the most-overdue phase wins. This is the HARD guarantee that pedestrians are
    never traded away for car-wait gains — it holds regardless of how a
    controller's objective weights cars vs pedestrians, because it sits ABOVE the
    objective entirely. Returns the forced phase, or None if nothing is overdue.
    """
    overdue = {}
    for p in PHASES:
        car_over = (obs.car_time_waited[p] - cfg.MAX_WAIT
                    if obs.car_demand[p] > 0 else float("-inf"))
        ped_over = (obs.ped_time_waited[p] - cfg.PED_MAX_WAIT
                    if obs.ped_demand[p] > 0 else float("-inf"))
        overdue[p] = max(car_over, ped_over)
    forced = [p for p in PHASES if overdue[p] > 0]
    if not forced:
        return None
    return max(forced, key=lambda p: overdue[p])


# ---------------------------------------------------------------------------
# 3a. Fixed-timer baseline — the benchmark every improvement is measured against
# ---------------------------------------------------------------------------
class FixedTimerController:
    """Cycles 1 -> 2 -> 3 -> 4 with pre-set greens, ignoring demand entirely.
    Runs empty phases, starves busy ones — the classic waste adaptive control
    fixes. The clearance intervals still apply (inserted by the sim)."""

    def __init__(self, cfg):
        self.cfg = cfg
        self._order = (1, 2, 3, 4)
        self._idx = -1

    def decide(self, obs) -> Decision:
        self._idx = (self._idx + 1) % len(self._order)
        phase = self._order[self._idx]
        return Decision(phase, self.cfg.FIXED_GREEN[phase])


# ---------------------------------------------------------------------------
# 3b. Heuristic adaptive controller — the SAFETY NET (must always beat fixed)
# ---------------------------------------------------------------------------
class HeuristicController:
    """Demand-responsive scoring controller. Reliably beats the fixed timer and
    stays in the codebase permanently as a fallback even once the optimizer
    exists (spec §3b).

    At each decision point:
      score(phase) = demand * DEMAND_WEIGHT + time_waited * URGENCY_WEIGHT
      * pick the highest-scoring phase,
      * green proportional to that phase's CAR demand, clamped MIN/MAX,
      * SKIP any phase with zero demand (don't waste green on an empty turn phase),
      * ANTI-STARVATION: any phase (car OR ped) waiting past MAX_WAIT jumps to the
        front regardless of score — nobody, including pedestrians, waits forever,
      * pedestrian phases inherit a MIN_GREEN large enough to cross (hard floor,
        enforced by the sim's clamp + config invariant).
    """

    def __init__(self, cfg):
        self.cfg = cfg

    def _green_for(self, phase: int, obs) -> float:
        """Green proportional to the time needed to clear this phase's CAR queue
        at saturation flow. The sim clamps to [MIN_GREEN, MAX_GREEN]; for a
        ped-only phase (car_demand 0) the MIN_GREEN floor covers crossing time."""
        clear_time = obs.car_demand[phase] / self.cfg.SATURATION_RATE
        return max(self.cfg.MIN_GREEN[phase], min(clear_time, self.cfg.MAX_GREEN))

    def decide(self, obs) -> Decision:
        cfg = self.cfg

        # --- ANTI-STARVATION ceiling (highest priority, beats score) ----------
        # Shared structural rule: cars bounded by MAX_WAIT, pedestrians by the
        # tighter PED_MAX_WAIT. Sits above scoring so peds can't be starved.
        forced = anti_starvation_phase(obs, cfg)
        if forced is not None:
            return Decision(forced, self._green_for(forced, obs))

        # --- SKIP zero-demand phases -----------------------------------------
        candidates = [p for p in PHASES if obs.demand[p] > 0]
        if not candidates:
            # Nothing waiting anywhere — hold a short default through phase so the
            # signal stays live and ready (minimal green, no wasted long green).
            return Decision(1, cfg.MIN_GREEN[1])

        # --- SCORE and pick ---------------------------------------------------
        def score(p: int) -> float:
            return obs.demand[p] * cfg.DEMAND_WEIGHT + obs.time_waited[p] * cfg.URGENCY_WEIGHT

        phase = max(candidates, key=score)
        return Decision(phase, self._green_for(phase, obs))


# ---------------------------------------------------------------------------
# 3c. Per-intersection optimization controller — the IMPRESSIVE version
# ---------------------------------------------------------------------------
def _needs_clearance(prev_phase, next_phase) -> bool:
    """Mirror of the sim's safety rule, used inside the optimizer's forward model
    so its predictions pay the same clearance cost the real sim will impose."""
    return prev_phase in PED_PHASES and next_phase in TURN_PHASES


# Which through-phase each pedestrian group rides concurrently (mirror of the
# sim's PHASE_PEDS). EW pedestrians cross during phase 1, NS during phase 3.
PED_GROUP_PHASE = {"EW_peds": 1, "NS_peds": 3}


class OptimizerController:
    """Bounded receding-horizon search, run INDEPENDENTLY per intersection.

    At each decision point it:
      1. estimates the local arrival rate per phase from recently OBSERVED
         arrival counts (a radar node's vehicle count differenced over time) —
         never the scenario's true configured rate (refinement #1, honesty),
      2. enumerates a small set of candidate first decisions (phase, green),
      3. evaluates each by a CHEAP ANALYTIC forward rollout over HORIZON seconds
         (queue += arrivals, queue -= saturation during green, accumulate wait) —
         NOT a re-run of the real sim, so four of these stay fast in the grid
         (refinement #2),
      4. picks the candidate with the lowest predicted total car wait.

    Constraints honoured: MIN_GREEN/MAX_GREEN, the mandatory clearance interval
    (priced into every rollout), exactly one phase active at a time, and the
    anti-starvation ceiling (a phase past MAX_WAIT is forced to the front,
    overriding the search — same guarantee the heuristic gives, pedestrians
    included). The HeuristicController remains available as the fallback.

    No central coordinator: this optimizer only ever sees its own intersection's
    local Observation. Grid-wide coordination emerges from neighbours reacting to
    each other's discharged traffic (Phase 3).
    """

    PRED_DT = 1.0           # coarse predictive time-step for the analytic rollout
    N_DURATION_SAMPLES = 6  # candidate greens sampled per phase

    def __init__(self, cfg):
        self.cfg = cfg
        self._fallback = HeuristicController(cfg)
        # rolling local arrival-rate estimates (units/sec), from observed counts
        self._rate = {p: 0.0 for p in PHASES}                  # cars per phase
        self._ped_rate = {g: 0.0 for g in PED_GROUP_PHASE}     # peds per group
        self._prev_cum = None
        self._prev_ped_cum = None
        self._prev_time = None
        self._ema = 0.4     # smoothing for the rate estimate

    # ---- local arrival-rate estimation (sensor-realistic) ----------------
    def _update_rate(self, obs) -> None:
        """Estimate arrivals/sec (cars AND peds) by differencing the observed
        cumulative arrival counts over elapsed time. Purely local history — the
        controller learns demand the way a radar/ped sensor would, by watching
        arrivals come in; it never reads the scenario's configured rate."""
        if self._prev_cum is not None and obs.now > self._prev_time:
            dt = obs.now - self._prev_time
            for p in PHASES:
                inst = (obs.cum_arrivals[p] - self._prev_cum[p]) / dt
                self._rate[p] = self._ema * inst + (1 - self._ema) * self._rate[p]
            for g in PED_GROUP_PHASE:
                inst = (obs.cum_ped_arrivals[g] - self._prev_ped_cum[g]) / dt
                self._ped_rate[g] = self._ema * inst + (1 - self._ema) * self._ped_rate[g]
        self._prev_cum = dict(obs.cum_arrivals)
        self._prev_ped_cum = dict(obs.cum_ped_arrivals)
        self._prev_time = obs.now

    # ---- candidate green durations for a phase ---------------------------
    def _duration_candidates(self, phase, obs):
        cfg = self.cfg
        lo, hi = cfg.MIN_GREEN[phase], cfg.MAX_GREEN
        n = self.N_DURATION_SAMPLES
        cands = {lo + (hi - lo) * k / (n - 1) for k in range(n)}
        # Always include the heuristic's own duration so the optimizer is, by
        # construction, never forced to do worse than the heuristic's sizing.
        cands.add(self._fallback._green_for(phase, obs))
        return sorted(cands)

    # ---- cheap analytic forward rollout ----------------------------------
    def _rollout(self, first_phase, first_dur, q0, qp0, prev_phase) -> float:
        """Predict total wait over HORIZON if we serve (first_phase, first_dur)
        now and then continue greedily. Returns the integral over the horizon of
        (queued cars + queued pedestrians) = total unit-seconds of wait. Cars and
        pedestrians are treated the same way (spec §2), so the optimizer won't
        starve a low-car-but-pedestrian-bearing phase. Lower is better.

        This is a CHEAP ANALYTIC model (float arithmetic over ~HORIZON steps),
        NOT a re-run of the real sim, so four optimizers stay fast in the grid.
        """
        cfg = self.cfg
        dt = self.PRED_DT
        sat = cfg.SATURATION_RATE
        q = dict(q0)                 # cars per phase
        qp = dict(qp0)               # pedestrians per group
        rate = self._rate
        prate = self._ped_rate

        # Light-weight phase-engine state, mirroring the real sim's transitions.
        prev = prev_phase
        clear_left = cfg.CLEARANCE_INTERVAL if _needs_clearance(prev, first_phase) else 0.0
        if clear_left > 0:
            active, green_left, pending = None, 0.0, (first_phase, first_dur)
        else:
            active, green_left, pending = first_phase, first_dur, None

        total_wait = 0.0
        t = 0.0
        while t < cfg.HORIZON:
            # arrivals over this predictive step (cars and peds)
            for p in PHASES:
                q[p] += rate[p] * dt
            for g in qp:
                qp[g] += prate[g] * dt
            # service
            if clear_left > 1e-9:
                clear_left -= dt
                if clear_left <= 1e-9 and pending is not None:
                    active, green_left = pending
                    pending = None
            elif active is not None and green_left > 1e-9:
                q[active] = max(0.0, q[active] - sat * dt)
                # pedestrians ride their phase's green concurrently — they cross
                for g, ph in PED_GROUP_PHASE.items():
                    if ph == active:
                        qp[g] = 0.0
                green_left -= dt
                if green_left <= 1e-9:
                    prev, active = active, None
            # decision point inside the rollout: greedily serve the largest queue
            # (cars + any peds riding that phase), so ped demand is considered.
            if clear_left <= 1e-9 and active is None and pending is None:
                def pressure(p):
                    peds = sum(qp[g] for g, ph in PED_GROUP_PHASE.items() if ph == p)
                    return q[p] + peds
                nxt = max(PHASES, key=pressure)
                ndur = max(cfg.MIN_GREEN[nxt], min(q[nxt] / sat, cfg.MAX_GREEN))
                if _needs_clearance(prev, nxt):
                    clear_left, pending = cfg.CLEARANCE_INTERVAL, (nxt, ndur)
                else:
                    active, green_left = nxt, ndur
            total_wait += (sum(q.values()) + sum(qp.values())) * dt
            t += dt
        return total_wait

    # ---- the decision ----------------------------------------------------
    def decide(self, obs) -> Decision:
        cfg = self.cfg
        self._update_rate(obs)

        # ANTI-STARVATION overrides the search entirely — the SAME shared rule the
        # heuristic uses (cars: MAX_WAIT, pedestrians: tighter PED_MAX_WAIT). This
        # is what makes pedestrian protection structural: no matter what the
        # rollout objective prefers, a pedestrian can never wait past PED_MAX_WAIT.
        forced = anti_starvation_phase(obs, cfg)
        if forced is not None:
            return Decision(forced, self._fallback._green_for(forced, obs))

        # SKIP zero-demand phases (don't waste green on an empty approach).
        candidates = [p for p in PHASES if obs.demand[p] > 0]
        if not candidates:
            return Decision(1, cfg.MIN_GREEN[1])

        q0 = dict(obs.car_demand)
        # initial pedestrian queues per group, from the local observation
        qp0 = {g: obs.ped_demand[ph] for g, ph in PED_GROUP_PHASE.items()}

        # The heuristic's own decision is the safe baseline. Score it in the same
        # rollout model so we compare like-for-like.
        h = self._fallback.decide(obs)
        cost_h = self._rollout(h.phase, h.duration, q0, qp0, obs.current_phase)

        # Search the candidate (phase, green) grid for the lowest predicted wait.
        best, best_cost = (h.phase, h.duration), cost_h
        for phase in candidates:
            for dur in self._duration_candidates(phase, obs):
                cost = self._rollout(phase, dur, q0, qp0, obs.current_phase)
                if cost < best_cost - 1e-9:
                    best_cost, best = cost, (phase, dur)

        # Only commit to the optimized plan if it is CONFIDENTLY better than the
        # heuristic — both an absolute predicted gain (dominant under light load,
        # where tiny differences are just noise) and a relative margin. Otherwise
        # defer to the proven heuristic, so the optimizer never meaningfully loses.
        gain = cost_h - best_cost
        if gain >= cfg.OPT_MIN_PREDICTED_GAIN and best_cost < cost_h * (1.0 - cfg.OPT_DEVIATION_MARGIN):
            return Decision(best[0], best[1])
        return h
