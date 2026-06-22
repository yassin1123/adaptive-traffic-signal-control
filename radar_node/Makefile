# Makefile — host verification build for the mmWave radar node.
# Dependency-light: gcc + standard library + the in-tree FFT. No DSP framework.
#
#   make phase1     build + (use 'make run-phase1' to execute)
#   make run-phase1 build and run the Phase 1 host verification
#   make clean
#
# On-target this is replaced by the TI mmWave SDK / CCS project; the chain
# sources are the same, only adc_source + fft backend swap (see adc_source.c).
#
# BUILD ENV NOTE (this machine): MinGW gcc must run with C:\MinGW\bin AHEAD of
# Git-Bash's /usr/bin on PATH, otherwise cc1.exe loads Git's incompatible
# libgmp/mpfr/iconv DLLs and dies with no diagnostic. Simplest: build from a
# PowerShell/cmd shell (where Git's /usr/bin is not on PATH), e.g.
#   gcc -std=c11 -O2 -Wall -Wextra -I. test/test_phase1.c fft.c synth_target.c \
#       adc_source.c -o build/test_phase1.exe -lm

CC      := gcc
CFLAGS  := -std=c11 -O2 -Wall -Wextra -I.
LDLIBS  := -lm
BUILD   := build

# chain sources accumulate as phases are added
SRC_P1  := fft.c synth_target.c adc_source.c
SRC_P2  := $(SRC_P1) range_proc.c doppler_proc.c
SRC_P3  := $(SRC_P2) cfar.c
SRC_P4  := $(SRC_P3) aoa.c
SRC_P5  := $(SRC_P4) cluster.c tracker.c
SRC_P6  := $(SRC_P5) zones.c node_output.c

$(BUILD):
	mkdir -p $(BUILD)

phase1: | $(BUILD)
	$(CC) $(CFLAGS) test/test_phase1.c $(SRC_P1) -o $(BUILD)/test_phase1 $(LDLIBS)

phase2: | $(BUILD)
	$(CC) $(CFLAGS) test/test_phase2.c $(SRC_P2) -o $(BUILD)/test_phase2 $(LDLIBS)

phase3: | $(BUILD)
	$(CC) $(CFLAGS) test/test_phase3.c $(SRC_P3) -o $(BUILD)/test_phase3 $(LDLIBS)

phase4: | $(BUILD)
	$(CC) $(CFLAGS) test/test_phase4.c $(SRC_P4) -o $(BUILD)/test_phase4 $(LDLIBS)

phase5: | $(BUILD)
	$(CC) $(CFLAGS) test/test_phase5.c $(SRC_P5) -o $(BUILD)/test_phase5 $(LDLIBS)

phase6: | $(BUILD)
	$(CC) $(CFLAGS) test/test_phase6.c $(SRC_P6) -o $(BUILD)/test_phase6 $(LDLIBS)

# end-to-end host build (the capstone): one command runs the whole chain
host: | $(BUILD)
	$(CC) $(CFLAGS) main_host.c $(SRC_P6) -o $(BUILD)/radar_host $(LDLIBS)
run: host
	./$(BUILD)/radar_host

run-phase1: phase1
	./$(BUILD)/test_phase1
run-phase2: phase2
	./$(BUILD)/test_phase2
run-phase3: phase3
	./$(BUILD)/test_phase3
run-phase4: phase4
	./$(BUILD)/test_phase4
run-phase5: phase5
	./$(BUILD)/test_phase5
run-phase6: phase6
	./$(BUILD)/test_phase6

clean:
	rm -rf $(BUILD)

.PHONY: phase1 phase2 phase3 phase4 phase5 phase6 host run \
        run-phase1 run-phase2 run-phase3 run-phase4 run-phase5 run-phase6 clean
