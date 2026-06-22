# On-Target Portability — TI IWR6843

## Honest status

This deliverable is the **firmware logic + the full FMCW signal chain**, written in
portable C and **verified on the host against synthetic IQ ground truth**. It has
**not** been run on real RF hardware. Every stage is proven by injecting known
targets (range, velocity, angle, RCS) and asserting the chain recovers them in
physical units — see the per-phase tests and `main_host.c`.

The chain is structured so it ports to a real IWR6843 with **no algorithm
changes**: only two things swap, both already isolated behind abstractions.

## Architecture mapping (where each stage runs on the device)

The IWR6843 has two subsystems: the **DSS** (data path — HWA hardware accelerator
+ C674x DSP) and the **MSS** (control — Cortex-R4F).

| Stage (file)                 | On-target placement            | Host stand-in        |
|------------------------------|--------------------------------|----------------------|
| Chirp/frame config           | MSS (RF/sensor config)         | `radar_config.h`     |
| ADC capture                  | RF front-end + EDMA -> L3 buffer | `synth_target.c`   |
| Range-FFT (`range_proc.c`)   | **DSS / HWA**                  | portable `fft.c`     |
| Doppler-FFT (`doppler_proc.c`) | **DSS / HWA**                | portable `fft.c`     |
| CFAR (`cfar.c`)              | **DSS / HWA** (CA) + MSS post  | scalar C             |
| AoA (`aoa.c`)               | DSS angle-FFT / MSS            | portable `fft.c`     |
| Clustering (`cluster.c`)     | **MSS / Cortex-R4F**           | same C               |
| Tracking (`tracker.c`)       | **MSS / Cortex-R4F**           | same C               |
| Zones/classify (`zones.c`)   | **MSS / Cortex-R4F**           | same C               |
| Output (`node_output.c`)     | MSS -> host interface          | same C (JSON)        |

The heavy, regular FFT/CFAR math maps onto the HWA/DSP; the irregular
object-level logic (cluster/track/zones/output) runs on the R4F — exactly the
split this code is organised around.

## The two swap points (the only things that change)

1. **`adc_source`** — the single hardware seam.
   - Host: `adc_source_synth()` returns synthetic IQ.
   - Target: `adc_source_target()` (stub in `adc_source.c`) `memcpy`s / points at
     the EDMA-delivered ADC ping-pong buffer after the frame-done interrupt, with
     TDM-MIMO demux + Doppler compensation producing the same
     `[virtual element][chirp][ADC sample]` layout the chain already consumes.
2. **FFT backend** — `fft.c` (portable radix-2) is replaced by HWA FFT
     configuration. Same call sites in `range_proc.c` / `doppler_proc.c` /
     `aoa.c`; the surrounding logic is untouched.

Everything else — the data structures, the stage sequence, the detection /
angle / clustering / tracking / classification logic, the `radar_config.h`
parameters and the bin↔physical conversions — is **identical** on host and
target.

## To flash on a real IWR6843

1. Drop these sources into a TI **mmWave SDK / Code Composer Studio** project
   (DSS + MSS split as in the table above).
2. Implement `adc_source_target()` against the SDK's ADC-buffer / DPIF data path.
3. Configure the HWA for the range/Doppler FFT sizes in `radar_config.h` and use
   it in place of `fft.c`.
4. Set the real chirp profile from `radar_config.h` via the sensor front-end API.
5. Stream `node_output` over the chosen transport (UART/SPI/Ethernet — out of
   scope here) to the control brain.

What you do **not** change: the FMCW math, the detection/AoA/cluster/track logic,
the config, or the verification harness — that synthetic-truth harness keeps
working as a regression test for the ported build.

## Build / run (host)

On this machine MinGW gcc must run from PowerShell (Git-Bash's `/usr/bin` shadows
cc1's DLLs — see the Makefile note). End-to-end:

```
gcc -std=c11 -O2 -Wall -Wextra -I. main_host.c fft.c synth_target.c adc_source.c \
    range_proc.c doppler_proc.c cfar.c aoa.c cluster.c tracker.c zones.c \
    node_output.c -o build/radar_host.exe -lm
./build/radar_host.exe
```

Or `make run` (and `make run-phaseN` for each stage's verification) in an
environment where MinGW precedes Git on PATH.
