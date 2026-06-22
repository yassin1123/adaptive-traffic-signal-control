"""
visualize.py — Phase 5: live side-by-side of FIXED vs OPTIMIZED (pygame).

Two 2x2 grids run in LOCKSTEP on IDENTICAL seeded traffic — fixed timer (left)
vs adaptive optimizer (right). The fixed side visibly gridlocks (queues pile
into red masses, road segments saturate and flash red, blocked intersections
flash, the stranded/spillback counters climb) while the optimized side keeps
flowing and self-balanced.

IMPORTANT: this file owns NO traffic logic. It only DRIVES the grids via the
grid module's step_once() and RENDERS their existing state via the read-only
accessors. Every moving dot / particle here is a *visual* derivation of observed
module state (queue lengths, segment occupancy) — it changes nothing in the sim.

Controls:  SPACE pause/resume   UP/DOWN speed (0.25x-8x)   R restart   Q/ESC quit
Headless self-test:  python visualize.py --smoke
"""

import argparse
import os

from config import DEFAULT
from controllers import FixedTimerController, OptimizerController
from grid import Grid, generate_grid_schedule, POS, neighbour_of
from sim import MOVEMENTS

# ---- window / layout -------------------------------------------------------
WIN_W, WIN_H = 1600, 920
TOP_H = 180                 # headline counter band
IH = 30                     # intersection half-size (px)
APPROACH_LEN = 120          # approach stub length (px)
BLOCK = 13                  # one queued car (px)
BLOCK_GAP = 15
MAX_BLOCKS = 13             # cap drawn cars per approach, then show the number
JAM_AT = 22                 # queue length that reads as fully "jammed" (red)

# ---- palette ---------------------------------------------------------------
BG = (16, 18, 24)
PANEL_BG = (26, 29, 38)
INK = (236, 240, 248)
DIM = (150, 156, 172)
ROAD = (54, 58, 70)
INTER = (96, 103, 122)
CAR = (90, 170, 255)
GREEN = (55, 205, 95)
RED = (235, 45, 45)
HOT = (255, 30, 30)
AMBER = (240, 180, 45)

OPP = {"N": "S", "S": "N", "E": "W", "W": "E"}
DIRV = {"N": (0, -1), "S": (0, 1), "E": (1, 0), "W": (-1, 0)}
SPEEDS = [0.25, 0.5, 1.0, 2.0, 4.0, 8.0]
BASE_SPS = 2.0              # sim-steps per real-second at 1.0x (followable start)


def lerp(a, b, t):
    t = 0.0 if t < 0 else 1.0 if t > 1 else t
    return (int(a[0] + (b[0] - a[0]) * t),
            int(a[1] + (b[1] - a[1]) * t),
            int(a[2] + (b[2] - a[2]) * t))


# ---- read-only state helpers ----------------------------------------------
def approach_queue(inter, app):
    return sum(len(inter.car_q[(app, m)]) for m in MOVEMENTS)


def light_color(inter, app):
    if inter.clearance_remaining > 0:
        return AMBER
    ph = inter.phase
    if ph is None:
        return RED
    ns = app in ("N", "S")
    served = (ph in (1, 2)) if ns else (ph in (3, 4))
    return GREEN if served else RED


def feeding_segment(name, app):
    nbr = neighbour_of(name, app)
    return None if nbr is None else (nbr, OPP[app])


def seg_ratio(grid, seg):
    return grid.segment_occupancy(seg) / grid.cfg.SEGMENT_CAPACITY


def intersection_blocked(grid, name):
    """An intersection is 'blocked' (spillback) when one of its OUTBOUND road
    segments is at capacity — its green can't discharge that way. Derived purely
    from read-only segment occupancy."""
    cap = grid.cfg.SEGMENT_CAPACITY
    for h in ("N", "S", "E", "W"):
        if neighbour_of(name, h) is not None:
            if grid.segment_occupancy((name, h)) >= cap:
                return True
    return False


def inter_center(px, name):
    row, col = POS[name]
    return px + 260 + 320 * col, TOP_H + 170 + 320 * row


# ---------------------------------------------------------------------------
# Visual-only flow particles (derived from observed discharges; no sim effect)
# ---------------------------------------------------------------------------
class Flow:
    """Spawns a short-lived dot when an approach's queue drops on a green light,
    animating a car crossing the intersection. Purely cosmetic."""

    def __init__(self):
        self.parts = []                       # [x, y, vx, vy, life]
        self.prev = {}                        # (name, app) -> last queue len

    def observe(self, px, grid):
        for name in POS:
            cx, cy = inter_center(px, name)
            inter = grid.inters[name]
            for app, (dx, dy) in DIRV.items():
                q = approach_queue(inter, app)
                key = (name, app)
                served = light_color(inter, app) == GREEN
                drop = self.prev.get(key, q) - q
                if served and drop > 0:
                    ex, ey = cx + dx * IH, cy + dy * IH
                    for _ in range(min(drop, 3)):
                        # move INTO and through the intersection (-DIRV)
                        self.parts.append([ex, ey, -dx * 150.0, -dy * 150.0, 0.55])
                self.prev[key] = q

    def update(self, dt):
        for p in self.parts:
            p[0] += p[2] * dt
            p[1] += p[3] * dt
            p[4] -= dt
        self.parts = [p for p in self.parts if p[4] > 0]

    def draw(self, screen, pygame):
        for x, y, _, _, life in self.parts:
            r = 5 if life > 0.3 else 4
            pygame.draw.circle(screen, (200, 230, 255), (int(x), int(y)), r)


# ---------------------------------------------------------------------------
# Rendering
# ---------------------------------------------------------------------------
def draw_stub(screen, pygame, grid, name, app, cx, cy, anim_t, fonts):
    _, _, small, count_font = fonts
    dx, dy = DIRV[app]
    ex, ey = cx + dx * IH, cy + dy * IH
    tx, ty = ex + dx * APPROACH_LEN, ey + dy * APPROACH_LEN
    seg = feeding_segment(name, app)

    # road tinted by how jammed the feeding segment is (stark green -> red)
    if seg is not None:
        ratio = seg_ratio(grid, seg)
        if ratio >= 0.999:
            flash = (int(anim_t * 6) % 2 == 0)
            col = HOT if flash else (150, 20, 20)
            width = 11
        else:
            col = lerp(GREEN, RED, ratio)
            width = 6
    else:
        col, width, ratio = ROAD, 5, 0.0
    pygame.draw.line(screen, col, (ex, ey), (tx, ty), width)

    # in-transit cars sliding along the road toward the intersection (flow)
    if seg is not None:
        n_transit = grid.in_transit.get(seg, 0)
        phase = (anim_t * 0.6) % 1.0
        for k in range(min(n_transit, 8)):
            f = ((k + phase) / 8.0)
            sx = tx - dx * APPROACH_LEN * f
            sy = ty - dy * APPROACH_LEN * f
            pygame.draw.circle(screen, (150, 200, 245), (int(sx), int(sy)), 3)

    # the QUEUE as a chunky mass of car-blocks, tinted blue -> red by length
    q = approach_queue(grid.inters[name], app)
    shown = min(q, MAX_BLOCKS)
    block_col = lerp(CAR, HOT, q / JAM_AT)
    # perpendicular so blocks read as a fat lane, not a thin string
    perp = (1, 0) if dx == 0 else (0, 1)
    for k in range(shown):
        bx = ex + dx * (BLOCK_GAP * (k + 1))
        by = ey + dy * (BLOCK_GAP * (k + 1))
        w = BLOCK if perp[0] else BLOCK + 4
        h = BLOCK + 4 if perp[0] else BLOCK
        pygame.draw.rect(screen, block_col,
                         (int(bx - w / 2), int(by - h / 2), w, h), border_radius=3)

    # big, obvious queue count when it matters
    if q >= 6:
        c = HOT if q >= JAM_AT else (INK if q < 12 else AMBER)
        lbl = count_font.render(str(q), True, c)
        lx = tx - dx * 6 - (lbl.get_width() if dx > 0 else 0) - (lbl.get_width() // 2 if dx == 0 else 0)
        ly = ty - dy * 6 - (lbl.get_height() if dy > 0 else 0) - (lbl.get_height() // 2 if dy == 0 else 0)
        screen.blit(lbl, (int(lx), int(ly)))

    # signal light at the mouth
    pygame.draw.circle(screen, light_color(grid.inters[name], app),
                       (int(ex + dx * 7), int(ey + dy * 7)), 5)


def draw_grid_panel(screen, pygame, fonts, grid, px, pw, flow, anim_t):
    pygame.draw.rect(screen, PANEL_BG, (px + 8, TOP_H + 8, pw - 16, WIN_H - TOP_H - 16),
                     border_radius=12)
    for name in POS:
        cx, cy = inter_center(px, name)
        for app in ("N", "S", "E", "W"):
            draw_stub(screen, pygame, grid, name, app, cx, cy, anim_t, fonts)
        # intersection box; flashes red when blocked by downstream spillback
        if intersection_blocked(grid, name) and int(anim_t * 6) % 2 == 0:
            box = HOT
        else:
            box = INTER
        pygame.draw.rect(screen, box, (cx - IH, cy - IH, 2 * IH, 2 * IH), border_radius=6)
        screen.blit(fonts[2].render(name, True, INK), (cx - 6, cy - 9))
    flow.draw(screen, pygame)


def draw_header(screen, pygame, fonts, left, right):
    huge, big, small, _ = fonts
    cx = WIN_W // 2
    pygame.draw.rect(screen, (12, 13, 18), (0, 0, WIN_W, TOP_H))

    # labels
    screen.blit(big.render("FIXED TIMER", True, AMBER), (40, 18))
    rlab = big.render("ADAPTIVE OPTIMIZER", True, GREEN)
    screen.blit(rlab, (WIN_W - 40 - rlab.get_width(), 18))

    # huge avg-wait numbers flanking the centre (the divergence)
    la = left.live_avg_car_wait()
    ra = right.live_avg_car_wait()
    lcol = lerp(AMBER, HOT, la / 80.0)          # fixed reddens as it climbs
    ln = huge.render(f"{la:.0f}s", True, lcol)
    rn = huge.render(f"{ra:.0f}s", True, GREEN)
    screen.blit(ln, (cx - 60 - ln.get_width(), 56))
    screen.blit(rn, (cx + 60, 56))
    screen.blit(small.render("avg wait", True, DIM), (cx - 60 - ln.get_width(), 132))
    screen.blit(small.render("avg wait", True, DIM), (cx + 60, 132))

    # centre gap badge: how many times lower the optimized wait is
    if ra > 0.1:
        factor = la / ra
        badge = big.render(f"{factor:.1f}x", True, INK)
        screen.blit(badge, (cx - badge.get_width() // 2, 60))
        sub = small.render("lower", True, DIM)
        screen.blit(sub, (cx - sub.get_width() // 2, 100))
    pygame.draw.line(screen, (0, 0, 0), (cx, 8), (cx, TOP_H - 8), 2)

    # stranded + spillback, fixed-red vs optimizer-green
    def stats(g, x, col_ok):
        st, sp = g.cars_stranded(), g.spillback_events
        cs = HOT if st > 200 else col_ok
        cp = HOT if sp > 0 else col_ok
        screen.blit(small.render(f"stranded {st}", True, cs), (x, 150))
        screen.blit(small.render(f"spillback {sp}", True, cp), (x + 170, 150))
    stats(left, 40, INK)
    stats(right, cx + 60, INK)


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------
def build(cfg, scenario):
    sched = generate_grid_schedule(scenario, cfg)
    left = Grid(FixedTimerController, cfg)
    right = Grid(OptimizerController, cfg)
    left.begin(sched)
    right.begin(sched)
    return left, right


def _make_fonts(pygame):
    return (pygame.font.SysFont("consolas", 60, bold=True),   # huge
            pygame.font.SysFont("consolas", 26, bold=True),   # big
            pygame.font.SysFont("consolas", 16),              # small
            pygame.font.SysFont("consolas", 22, bold=True))   # queue count


def run_visual(scenario, speed_idx):
    import pygame
    pygame.init()
    screen = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption("Adaptive Traffic Signal Control - fixed vs optimized")
    clock = pygame.time.Clock()
    fonts = _make_fonts(pygame)
    foot = pygame.font.SysFont("consolas", 16)

    cfg = DEFAULT
    left, right = build(cfg, scenario)
    lflow, rflow = Flow(), Flow()
    acc = 0.0
    anim_t = 0.0
    paused = False
    done = False

    while True:
        dt = clock.tick(60) / 1000.0
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                pygame.quit(); return
            if e.type == pygame.KEYDOWN:
                if e.key in (pygame.K_ESCAPE, pygame.K_q):
                    pygame.quit(); return
                if e.key == pygame.K_SPACE:
                    paused = not paused
                if e.key == pygame.K_UP:
                    speed_idx = min(len(SPEEDS) - 1, speed_idx + 1)
                if e.key == pygame.K_DOWN:
                    speed_idx = max(0, speed_idx - 1)
                if e.key == pygame.K_r:
                    left, right = build(cfg, scenario)
                    lflow, rflow = Flow(), Flow()
                    done = False

        if not paused and not done:
            anim_t += dt
            acc += SPEEDS[speed_idx] * BASE_SPS * dt
            steps = int(acc)
            acc -= steps
            for _ in range(steps):
                a = left.step_once()
                b = right.step_once()
                lflow.observe(0, left)
                rflow.observe(WIN_W // 2, right)
                if not (a or b):
                    done = True
                    break
        lflow.update(dt)
        rflow.update(dt)

        screen.fill(BG)
        draw_header(screen, pygame, fonts, left, right)
        draw_grid_panel(screen, pygame, fonts, left, 0, WIN_W // 2, lflow, anim_t)
        draw_grid_panel(screen, pygame, fonts, right, WIN_W // 2, WIN_W // 2, rflow, anim_t)
        pygame.draw.line(screen, (0, 0, 0), (WIN_W // 2, TOP_H), (WIN_W // 2, WIN_H), 3)

        status = (f"scenario: {scenario}   identical seeded traffic   "
                  f"speed: {SPEEDS[speed_idx]:g}x   t={left.now():.0f}s   "
                  f"[SPACE] pause  [UP/DOWN] speed  [R] restart  [Q] quit"
                  + ("   - SIM COMPLETE" if done else "   - PAUSED" if paused else ""))
        screen.blit(foot.render(status, True, DIM), (20, WIN_H - 24))
        pygame.display.flip()


def run_smoke(scenario, frames):
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
    import pygame
    pygame.init()
    screen = pygame.Surface((WIN_W, WIN_H))
    fonts = _make_fonts(pygame)
    cfg = DEFAULT
    left, right = build(cfg, scenario)
    lflow, rflow = Flow(), Flow()
    for _ in range(frames):
        for _ in range(20):
            left.step_once(); right.step_once()
            lflow.observe(0, left); rflow.observe(WIN_W // 2, right)
        lflow.update(0.1); rflow.update(0.1)
        screen.fill(BG)
        draw_header(screen, pygame, fonts, left, right)
        draw_grid_panel(screen, pygame, fonts, left, 0, WIN_W // 2, lflow, 1.0)
        draw_grid_panel(screen, pygame, fonts, right, WIN_W // 2, WIN_W // 2, rflow, 1.0)
    pygame.quit()
    print(f"smoke OK ({scenario}): rendered {frames} frames headless, no errors")
    print(f"  fixed     : avg={left.live_avg_car_wait():.1f}s  "
          f"stranded={left.cars_stranded()}  spillback={left.spillback_events}")
    print(f"  optimized : avg={right.live_avg_car_wait():.1f}s  "
          f"stranded={right.cars_stranded()}  spillback={right.spillback_events}")
    assert right.spillback_events == 0, "optimized should never spill back"
    if scenario in ("rush", "asymmetric"):
        assert left.cars_stranded() > right.cars_stranded(), "fixed should gridlock more"
    print("  contrast assertions PASS (optimized flows; fixed gridlocks under load)")


def main():
    p = argparse.ArgumentParser(description="Phase 5: live fixed-vs-optimized visual")
    p.add_argument("--scenario", default="rush", choices=list(DEFAULT.SCENARIOS.keys()))
    p.add_argument("--speed", type=float, default=1.0,
                   help="initial playback speed (0.25..8); default 1x (slow/followable)")
    p.add_argument("--smoke", action="store_true", help="headless self-test, no window")
    p.add_argument("--frames", type=int, default=40)
    args = p.parse_args()
    if args.smoke:
        run_smoke(args.scenario, args.frames)
    else:
        idx = min(range(len(SPEEDS)), key=lambda i: abs(SPEEDS[i] - args.speed))
        run_visual(args.scenario, idx)


if __name__ == "__main__":
    main()
