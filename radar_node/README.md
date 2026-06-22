# mmWave Radar Sensing Node (TI IWR6843) — Firmware

The **sensing layer** of the adaptive traffic system: C firmware that drives a TI
IWR6843 FMCW radar through the full signal chain and outputs per-approach
vehicle/pedestrian counts + speeds — the input the Python control **brain** (in
the repo root) consumes.

**Status:** firmware logic + full FMCW chain, **verified on the host against
synthetic IQ ground truth**, portable to the real IWR6843. Not run on RF
hardware. See [PORTABILITY.md](PORTABILITY.md).

## The chain

```
ADC capture → range-FFT → Doppler-FFT → CFAR → angle-of-arrival
            → clustering → tracking → zone mapping / classification → output
```

Every stage is proven by injecting known targets and asserting recovery **in
physical units** (metres, m/s, degrees, counts) — not bin indices — so the
verification closes against physics, not against shared conventions.

## Build & run (host)

Build from **PowerShell** (MinGW gcc; Git-Bash's `/usr/bin` shadows cc1's DLLs):

```
make run            # end-to-end: full chain on a synthetic scene, all asserts
make run-phase1     # ... per-stage verification (phases 1..6)
make run-phase6
python check_brain_contract.py   # integration: node output == brain input
```

(or invoke gcc directly — see the Makefile header for the exact line).

## Files

| File | Role |
|------|------|
| `radar_config.h` | all radar parameters + derived quantities + bin↔physical maps |
| `cplx.h`, `fft.c` | complex type + portable FFT (HWA stand-in) |
| `adc_source.c` | host(synth)/target(real-ADC) seam |
| `synth_target.c` | synthetic IQ ground-truth generator |
| `range_proc.c`, `doppler_proc.c` | range/Doppler FFT → range-Doppler map |
| `cfar.c` | CA-CFAR + OS-CFAR detection |
| `aoa.c` | angle-of-arrival across the virtual array |
| `cluster.c`, `tracker.c` | DBSCAN clustering + frame-to-frame tracking |
| `zones.c`, `node_output.c` | approach mapping, classification, JSON output |
| `main_host.c` | end-to-end host verification (the capstone) |
| `test/` | per-stage verification harnesses |
| `check_brain_contract.py` | Track-A integration check vs the Python brain |

## Key derived quantities (this config)

range res 0.43 m · max range 109 m · velocity res 0.39 m/s · max unambiguous
velocity 25 m/s (90 km/h) — sane for a road intersection, and proven consistent
with the synthetic generator end to end.
