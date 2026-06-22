# Adaptive Traffic Signal Control

An end-to-end adaptive traffic-signal system in two halves that meet at a single,
**verified** contract:

1. **The sensing node** (`/radar_node`) — embedded **C** firmware for a TI
   **IWR6843** 60 GHz FMCW mmWave radar. It runs the full signal chain (chirp →
   ADC → range-FFT → Doppler-FFT → CFAR → angle-of-arrival → clustering →
   tracking → classification) and emits **per-approach vehicle/pedestrian counts
   and speeds**.
2. **The control brain** (`/`) — a **Python** receding-horizon optimizer that
   consumes those per-approach counts and allocates green time across a 2×2 grid
   of intersections to minimize total wait, while guaranteeing pedestrian safety.

The radar's output JSON is **exactly** what the brain ingests — proven by an
integration check that imports the brain's own approach vocabulary.

---

## Headline results

Grid-wide average car-wait reduction of the optimizer vs a fixed-timer baseline,
on identical seeded traffic (from `results.py`, reproducible):

| Scenario   | Fixed timer | Optimized | Improvement |
|------------|------------:|----------:|------------:|
| light      | 11.5 s      | 4.6 s     | **60%**     |
| rush       | 159.6 s     | 9.6 s     | **94%**     |
| asymmetric | 17.3 s      | 5.9 s     | **66%**     |

Under rush the fixed-timer grid **gridlocks** — road segments saturate and spill
back (7,144 spillback events, 6 of 8 segments at capacity) — while the optimized
grid stays stable (zero spillback) and self-balances across intersections with
**no central coordinator** (coordination emerges from neighbours reacting to each
other's outflow). Pedestrian wait is structurally bounded and never traded away
for the car metric.

---

## Repository layout

```
/                     the Python control brain
  config.py           all tunable parameters (single source of truth)
  sim.py              time-step simulation: 4-phase engine, pedestrian clearance, metrics
  controllers.py      fixed-timer, heuristic, and receding-horizon optimizer
  grid.py             2x2 grid, emergent coordination, finite-segment spillback model
  run.py              seeded scenarios + single-intersection comparison driver
  results.py          reproducible results table (-> results.md)
  visualize.py        live pygame side-by-side (fixed vs optimized)
  ped_protection_proof.py, diagnose.py   safety proof + congestion diagnostics

/radar_node           the C radar firmware (verified on host vs synthetic IQ)
  radar_config.h      all radar parameters + derived quantities + bin<->physical maps
  cplx.h, fft.c       complex type + portable FFT (HWA stand-in)
  synth_target.c      synthetic-IQ ground-truth generator
  adc_source.c        host(synth)/target(real-ADC) hardware seam
  range_proc.c, doppler_proc.c   range/Doppler FFT -> range-Doppler map
  cfar.c              CA-CFAR + OS-CFAR detection
  aoa.c               angle-of-arrival across the virtual array
  cluster.c, tracker.c   DBSCAN clustering + frame-to-frame tracking
  zones.c, node_output.c assignment, classification, JSON output (the contract)
  main_host.c         end-to-end host pipeline (the capstone)
  test/               per-stage verification harnesses
  check_brain_contract.py   integration check: radar output == brain input
  README.md, PORTABILITY.md
```

---

## Build & run

### The brain (Python 3.12)

```bash
python run.py --scenario rush      # fixed vs heuristic vs optimized, single intersection
python grid.py --scenario rush     # the 2x2 grid (with spillback)
python results.py --check          # full reproducible results table (asserts determinism)
python visualize.py                # live pygame visual (pip install pygame)
```

### The radar firmware (C11, on the host)

```bash
make -C radar_node run              # end-to-end: full chain on a synthetic scene
make -C radar_node run-phase1       # ... per-stage verification (phases 1..6)
python radar_node/check_brain_contract.py   # radar output == brain input
```

> **Build note:** on Windows with MinGW + Git-Bash, compile the C from a
> **PowerShell** shell — Git-Bash's `/usr/bin` shadows the compiler's support
> DLLs. The exact gcc invocation is in `radar_node/Makefile`.

---

## Tech stack

- **Brain:** Python 3.12, standard library only (pygame for the optional visual).
  No ML/RL — a transparent receding-horizon search with a heuristic fallback.
- **Radar:** C11, standard library + a self-contained radix-2 FFT. No heavy DSP
  framework; structured as real IWR6843 firmware (DSS/HWA vs MSS/R4F split).

---

## Honest limitations

- **The radar firmware is verified on the host against synthetic IQ ground
  truth — it has not been run on real RF hardware.** The chain is identical to
  the on-target build; only the ADC source and FFT backend swap (see
  `radar_node/PORTABILITY.md`). This is the firmware logic + full chain, proven
  in simulation and portable to the board, not a hardware-flashed demo.
- **The brain is validated in a lightweight traffic simulation**, not on field
  data. Cars are queue items (no car-following physics); coordination is emergent
  by design (no global solver).
- **Parameters are tuned for the tested regimes** (light/rush/asymmetric), not
  claimed universally optimal.
- Transport between the radar and the brain (networking) is out of scope; the
  contract is defined and verified, the wire transport is not implemented.

All quantitative claims are seeded and reproducible from the scripts above.
