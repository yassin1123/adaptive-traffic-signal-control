"""
sim.py — the lightweight traffic world (the test harness).

Just enough world to make cars arrive, queue, and clear. NO vehicle physics:
cars are simple queue items (spec §0). The simulation is also the *sensor*: it
holds the true per-approach queue counts that, in the real system, a roadside
mmWave radar node would report. Controllers only ever receive an `Observation`
built from LOCAL per-approach queue length + wait time — never global or future
state. This keeps the "sensor-realistic input" honesty story airtight.

What lives here (and NOT in controllers):
  * the 4-phase engine and the rule that exactly ONE phase is active at a time,
  * the mandatory pedestrian CLEARANCE_INTERVAL between a ped-walking phase and
    a following protected-turn phase (centralised so a controller can't forget),
  * arrival queues, discharge at saturation flow, and wait-time logging.
"""

from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from config import PHASES, PED_PHASES, TURN_PHASES


# ---------------------------------------------------------------------------
# Static phase wiring (LOCKED, spec §2)
# ---------------------------------------------------------------------------
# Which (approach, movement) car-queues each phase discharges. Right turns ride
# along with the through phase of their approach; protected lefts get their own
# turn phase. Through+right on the N–S axis move in phase 1, their lefts in
# phase 2; the E–W axis mirrors this in phases 3 and 4.
PHASE_SERVES: Dict[int, Tuple[Tuple[str, str], ...]] = {
    1: (("N", "through"), ("N", "right"), ("S", "through"), ("S", "right")),
    2: (("N", "left"), ("S", "left")),
    3: (("E", "through"), ("E", "right"), ("W", "through"), ("W", "right")),
    4: (("E", "left"), ("W", "left")),
}

# Pedestrian group that walks concurrently during each ped phase. Phases 2 & 4
# carry no pedestrians (they are STOPPED during protected turns).
PHASE_PEDS: Dict[int, str] = {1: "EW_peds", 3: "NS_peds"}

APPROACHES = ("N", "S", "E", "W")
MOVEMENTS = ("through", "left", "right")
PED_GROUPS = ("EW_peds", "NS_peds")


def movement_phase(approach: str, movement: str) -> int:
    """Which phase serves a given (approach, movement). Through & right ride the
    axis's through phase; protected lefts get the axis's turn phase."""
    north_south = approach in ("N", "S")
    if movement == "left":
        return 2 if north_south else 4
    return 1 if north_south else 3   # through or right


@dataclass
class Car:
    """A vehicle as a simple queue item — no physics, just timestamps."""
    arrival_time: float
    approach: str            # 'N' | 'S' | 'E' | 'W'  (inbound direction)
    movement: str            # 'through' | 'left' | 'right'
    depart_time: Optional[float] = None


@dataclass
class Ped:
    """A pedestrian, treated the same way as a car: a demand with a wait time."""
    arrival_time: float
    group: str               # 'EW_peds' | 'NS_peds'
    depart_time: Optional[float] = None


# ---------------------------------------------------------------------------
# Observation — the ONLY thing a controller sees (local, sensor-realistic)
# ---------------------------------------------------------------------------
@dataclass
class Observation:
    """Per-intersection local state handed to a controller at a decision point.

    Everything here is what a radar node at this one intersection could measure:
    how many cars/peds are queued on each phase's approaches, and how long the
    oldest of them has waited. No grid state, no future arrivals, no peeking.
    """
    now: float
    current_phase: Optional[int]
    # Per phase:
    car_demand: Dict[int, int]      # cars queued that this phase would serve
    ped_demand: Dict[int, int]      # peds queued that this phase would serve
    demand: Dict[int, int]          # car_demand + ped_demand (unified, spec §2)
    time_waited: Dict[int, float]   # oldest waiting unit (car OR ped) — unified urgency
    car_time_waited: Dict[int, float]  # oldest waiting CAR on this phase
    ped_time_waited: Dict[int, float]  # oldest waiting PEDESTRIAN on this phase
                                    # (split out so each gets its own anti-starvation
                                    #  ceiling: MAX_WAIT for cars, PED_MAX_WAIT for peds)
    cum_arrivals: Dict[int, int]    # cumulative cars that have ARRIVED into each
                                    # phase's approaches so far. A radar node counts
                                    # vehicles entering an approach; a controller
                                    # can difference this over time to estimate the
                                    # local arrival RATE — without ever seeing the
                                    # scenario's true configured rate (refinement #1).
    cum_ped_arrivals: Dict[str, int]  # same idea for pedestrians, per ped group.


class Intersection:
    """A single 4-way intersection: queues + phase engine + its controller.

    Designed to be grid-ready from the start (Phase 3): `step()` returns the
    cars discharged this step so a grid can route them to neighbours. For a
    standalone run those discharged cars simply exit and their wait is recorded.
    """

    def __init__(self, controller, cfg, name: str = "I"):
        self.cfg = cfg
        self.controller = controller
        self.name = name

        # Car queues keyed by (approach, movement); pedestrian queues by group.
        self.car_q: Dict[Tuple[str, str], deque] = {
            (a, m): deque() for a in APPROACHES for m in MOVEMENTS
        }
        self.ped_q: Dict[str, deque] = {g: deque() for g in PED_GROUPS}

        # Cumulative arrivals into each phase's approaches (sensor count).
        self.cum_arrivals: Dict[int, int] = {p: 0 for p in PHASES}
        # Cumulative pedestrian arrivals per group (sensor count, for rate est.).
        self.cum_ped_arrivals: Dict[str, int] = {g: 0 for g in PED_GROUPS}

        # --- phase-engine state ---
        self.phase: Optional[int] = None     # currently active phase (None = idle/start)
        self.green_remaining: float = 0.0
        self.clearance_remaining: float = 0.0
        self.pending_phase: Optional[int] = None   # phase waiting behind a clearance
        self.pending_green: float = 0.0
        self._discharge_credit: float = 0.0        # fractional-car discharge accumulator

        # --- metrics ---
        self.car_waits: List[float] = []     # completed car waits (depart - arrival)
        self.ped_waits: List[float] = []     # completed pedestrian waits
        self.cars_seen: int = 0

        # Safety audit trail: records (now, prev_phase, next_phase, had_clearance)
        # for every phase transition, so tests can prove no turn phase ever starts
        # green right after pedestrians walked without a clearance interval.
        self.transitions: List[Tuple[float, Optional[int], int, bool]] = []

    # ------------------------------------------------------------------ arrivals
    def add_car(self, car: Car) -> None:
        self.car_q[(car.approach, car.movement)].append(car)
        self.cum_arrivals[movement_phase(car.approach, car.movement)] += 1
        self.cars_seen += 1

    def add_ped(self, ped: Ped) -> None:
        self.ped_q[ped.group].append(ped)
        self.cum_ped_arrivals[ped.group] += 1

    # ------------------------------------------------------------------ helpers
    @staticmethod
    def needs_clearance(prev_phase: Optional[int], next_phase: int) -> bool:
        """SAFETY RULE (non-negotiable, spec §2): a protected-turn phase must
        never start green immediately after pedestrians were walking. So a
        clearance interval is required whenever we move from a pedestrian-walking
        phase (1 or 3) into a protected-turn phase (2 or 4)."""
        return prev_phase in PED_PHASES and next_phase in TURN_PHASES

    def observe(self, now: float) -> Observation:
        """Build the LOCAL observation handed to the controller."""
        car_demand, ped_demand, demand = {}, {}, {}
        time_waited, car_time_waited, ped_time_waited = {}, {}, {}
        for p in PHASES:
            cars = sum(len(self.car_q[k]) for k in PHASE_SERVES[p])
            grp = PHASE_PEDS.get(p)
            peds = len(self.ped_q[grp]) if grp is not None else 0
            car_demand[p] = cars
            ped_demand[p] = peds
            demand[p] = cars + peds
            # oldest waiting CAR and oldest waiting PED, tracked separately
            oldest_car = None
            for key in PHASE_SERVES[p]:
                q = self.car_q[key]
                if q and (oldest_car is None or q[0].arrival_time < oldest_car):
                    oldest_car = q[0].arrival_time   # FIFO -> front is oldest
            oldest_ped = None
            if grp is not None and self.ped_q[grp]:
                oldest_ped = self.ped_q[grp][0].arrival_time
            cw = (now - oldest_car) if oldest_car is not None else 0.0
            pw = (now - oldest_ped) if oldest_ped is not None else 0.0
            car_time_waited[p] = cw
            ped_time_waited[p] = pw
            time_waited[p] = max(cw, pw)
        return Observation(now, self.phase, car_demand, ped_demand, demand,
                           time_waited, car_time_waited, ped_time_waited,
                           dict(self.cum_arrivals), dict(self.cum_ped_arrivals))

    # ------------------------------------------------------------------ discharge
    def _serve_peds(self, now: float) -> None:
        """Pedestrians ride along with their through phase (concurrent). When the
        ped phase is green the whole waiting group crosses; record their waits."""
        grp = PHASE_PEDS.get(self.phase)
        if grp is None:
            return
        q = self.ped_q[grp]
        while q:
            ped = q.popleft()
            ped.depart_time = now
            self.ped_waits.append(now - ped.arrival_time)

    def _discharge_cars(self, now: float, dt: float, reserve=None) -> List[Car]:
        """Discharge up to SATURATION_RATE*dt cars from the active phase's queues,
        round-robin across its served (approach, movement) lanes. Returns the
        cars that left (used by the grid to route them onward).

        `reserve(approach, movement) -> bool` is optional backpressure (grid mode):
        it returns False when the road segment this car would enter is FULL, in
        which case the lane is blocked — the car stays queued even though the
        light is green. That is spillback. When True it also reserves a slot on
        that segment. With reserve=None (single-intersection mode) cars always
        leave, exactly as before."""
        discharged: List[Car] = []
        if self.phase is None:
            return discharged
        self._discharge_credit += self.cfg.SATURATION_RATE * dt
        lanes = [(k, self.car_q[k]) for k in PHASE_SERVES[self.phase]]
        # round-robin so no single lane monopolises the green
        while self._discharge_credit >= 1.0:
            progressed = False
            for (approach, movement), q in lanes:
                if self._discharge_credit < 1.0:
                    break
                if not q:
                    continue
                # backpressure: if the downstream segment is full, this lane is
                # blocked (spillback) — skip it; do NOT consume green capacity.
                if reserve is not None and not reserve(approach, movement):
                    continue
                car = q.popleft()
                car.depart_time = now
                self.car_waits.append(now - car.arrival_time)
                discharged.append(car)
                self._discharge_credit -= 1.0
                progressed = True
            if not progressed:
                break
        return discharged

    # ------------------------------------------------------------------ step
    def step(self, now: float, reserve=None) -> List[Car]:
        """Advance this intersection by one TIME_STEP. Arrivals must already be
        enqueued (via add_car/add_ped) for this `now`. Returns cars discharged.
        `reserve` (grid mode) applies downstream-segment backpressure — see
        _discharge_cars. None => single-intersection mode (no backpressure)."""
        dt = self.cfg.TIME_STEP
        discharged: List[Car] = []

        if self.clearance_remaining > 0:
            # CLEARANCE: no vehicle discharge, no new pedestrians released; those
            # already in the crosswalk finish. When it ends, activate the pending
            # phase (and release its peds if it is a ped phase — it won't be,
            # since clearance only ever precedes a turn phase, but kept general).
            self.clearance_remaining -= dt
            if self.clearance_remaining <= 1e-9:
                self.clearance_remaining = 0.0
                self.phase = self.pending_phase
                self.green_remaining = self.pending_green
                self.pending_phase = None
                self._discharge_credit = 0.0   # saturation flow restarts with this green
                self._serve_peds(now)
        elif self.green_remaining > 0:
            discharged = self._discharge_cars(now, dt, reserve)
            self._serve_peds(now)   # peds arriving mid-green also cross
            self.green_remaining -= dt

        # Decision point: nothing active and no clearance pending -> ask controller.
        if (self.clearance_remaining <= 0 and self.green_remaining <= 1e-9
                and self.pending_phase is None):
            obs = self.observe(now)
            decision = self.controller.decide(obs)
            self._begin(decision.phase, decision.duration, now)

        return discharged

    def _begin(self, next_phase: int, green: float, now: float) -> None:
        """Transition toward `next_phase`, inserting a clearance interval first if
        the pedestrian-safety rule requires it. Centralised here so EVERY
        controller is protected by it identically."""
        green = max(self.cfg.MIN_GREEN[next_phase],
                    min(green, self.cfg.MAX_GREEN))
        had_clearance = self.needs_clearance(self.phase, next_phase)
        self.transitions.append((now, self.phase, next_phase, had_clearance))
        if had_clearance:
            self.clearance_remaining = self.cfg.CLEARANCE_INTERVAL
            self.pending_phase = next_phase
            self.pending_green = green
        else:
            self.phase = next_phase
            self.green_remaining = green
            self._discharge_credit = 0.0   # saturation flow restarts with this green
            self._serve_peds(now)

    # ------------------------------------------------------------------ metrics
    def avg_car_wait(self) -> float:
        """Average wait over cars that have DISCHARGED (arrival -> departure)."""
        return sum(self.car_waits) / len(self.car_waits) if self.car_waits else 0.0

    def avg_ped_wait(self) -> float:
        return sum(self.ped_waits) / len(self.ped_waits) if self.ped_waits else 0.0

    def max_ped_wait(self) -> float:
        """Worst single pedestrian wait — used to prove the PED_MAX_WAIT ceiling
        actually bounds pedestrian wait (it should never exceed PED_MAX_WAIT plus
        the time to then clear them: one clearance + one ped green)."""
        return max(self.ped_waits) if self.ped_waits else 0.0

    def cars_remaining(self) -> int:
        return sum(len(q) for q in self.car_q.values())

    def all_car_waits(self, end_time: float) -> List[float]:
        """Wait of EVERY car that arrived: discharged cars use their full wait;
        cars still queued at sim end contribute their partial wait so far
        (end_time - arrival). This is the honest delay metric under congestion —
        an oversaturated controller can't hide its pain in cars that never
        discharged. Both controllers are scored identically this way."""
        waits = list(self.car_waits)
        for q in self.car_q.values():
            waits.extend(end_time - car.arrival_time for car in q)
        return waits

    def avg_total_car_wait(self, end_time: float) -> float:
        waits = self.all_car_waits(end_time)
        return sum(waits) / len(waits) if waits else 0.0


# ---------------------------------------------------------------------------
# Single-intersection driver
# ---------------------------------------------------------------------------
@dataclass
class RunResult:
    avg_car_wait: float          # headline: delay over ALL arrived cars (incl. still-queued)
    avg_car_wait_done: float     # delay over discharged cars only (for reference)
    avg_ped_wait: float
    max_ped_wait: float          # worst single pedestrian wait (ceiling proof)
    cars_completed: int
    cars_remaining: int
    peds_completed: int

    def __str__(self) -> str:
        return (f"avg_car_wait={self.avg_car_wait:7.2f}s  "
                f"avg_ped_wait={self.avg_ped_wait:6.2f}s  "
                f"max_ped_wait={self.max_ped_wait:6.1f}s  "
                f"cars_done={self.cars_completed:5d}  "
                f"cars_left={self.cars_remaining:4d}  "
                f"peds_done={self.peds_completed:5d}")


def run_single(controller, schedule, cfg, name: str = "I") -> Tuple[RunResult, Intersection]:
    """Run ONE intersection through a pre-generated arrival `schedule` (identical
    traffic for every controller — the honest comparison). `schedule` is a list
    indexed by step: schedule[i] = list of Car/Ped objects arriving at that step.
    """
    inter = Intersection(controller, cfg, name=name)
    dt = cfg.TIME_STEP
    n_steps = len(schedule)
    for i in range(n_steps):
        now = i * dt
        for unit in schedule[i]:
            if isinstance(unit, Car):
                inter.add_car(unit)
            else:
                inter.add_ped(unit)
        inter.step(now)
    end_time = n_steps * dt
    return (
        RunResult(
            avg_car_wait=inter.avg_total_car_wait(end_time),
            avg_car_wait_done=inter.avg_car_wait(),
            avg_ped_wait=inter.avg_ped_wait(),
            max_ped_wait=inter.max_ped_wait(),
            cars_completed=len(inter.car_waits),
            cars_remaining=inter.cars_remaining(),
            peds_completed=len(inter.ped_waits),
        ),
        inter,
    )
