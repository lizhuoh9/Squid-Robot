# Water test plan: C1 against B

What to do before, during and after the water test that compares the two images built in this folder. How to build and flash them is in `README.md` here. Paths start at the repository root; each command says which folder it runs from.

## 1. What the test answers

Does the retuned controller with delay prediction (B) hold depth clearly better than the controller it replaces (C1), under the same conditions on the same day?

- **C1 (control):** the sources of commit `deef5845`, the ones the image deployed on 2026-10-27 was built from, with the learned correction switched off.
- **B (new):** C1 + `option_b.patch`: retuned constants and a delay predictor.
- **The image on the robot now** (`7_ProposalA_Learning/artifacts/hybrid_mpc_v1/rollback-enabled-limit0p5-5d2fad78ecfa.bin`, SHA-256 `5d2fad78…`) is not part of the comparison. It is the rollback.

What the simulation expects. The model has a 0.85 s delay and random disturbances (σ 0.29 cm/s²), and each controller ran with 20 random seeds:

| | C1 | B |
|---|---|---|
| Mean hold RMS per run, median (10–90 %) | 1.76 cm (1.44–2.04) | 0.92 cm (0.82–1.05) |
| Hold peak-to-peak, median | 7.0 cm | 3.8 cm |
| Valve switches per minute | 61 | 73 |
| Pump starts per minute | 34 | 42 |

These are simulation numbers. The delay and the disturbance level were chosen to reproduce earlier water runs, so they are an expectation, not a result.

## 2. Pass criteria (fixed now, not changed after the test)

A run counts when the F7 automatic run completes all three holds (30, 40 and 50 cm) without manual intervention. A hold is the 20–60 s after each target takes effect; error = measured depth − target depth. Both are as in the earlier analyses of the water logs. The metric of a run is the mean of its three hold RMS errors.

B passes when all of these hold:

1. The median of B's run metrics ≤ 0.75 × C1's median (simulation: about 0.52 ×).
2. The median peak-to-peak of all B holds ≤ 0.75 × C1's (simulation: about 0.54 ×).
3. B's worst run ≤ C1's median, so that no B run is worse than a typical C1 run.
4. B's valve switches per minute ≤ 1.3 × C1's (simulation: about 1.19 ×).
5. No abort condition of section 5 occurred.

**Fail:** B's median is worse than C1's (ratio above 1.0), or an abort condition occurred. **Inconclusive:** every other outcome, for example B better by less than 25 %, or better by 25 % or more but missing criterion 2, 3 or 4.

## 3. Before the test day: build the images

On the PC with ESP-IDF v5.4.4:

1. **Back up the four `OtaConfig.local.h` files.** `main` stopped tracking them in commit `98142ba9`, while this branch still tracks them, so switching between `main` and this branch can delete them or replace them with the committed versions. Copy them outside the repository before you switch, and copy them back after you have switched back to `main`. From the repository root, in Git Bash:
   ```bash
   for d in 1_ESP32_MainController 6_DepthHold_ProposalA 7_ProposalA_Learning/firmware 7_ProposalA_Learning/artifacts/pre_learning_recorder; do mkdir -p "../ota_local_backup/$d/main" && cp "$d/main/OtaConfig.local.h" "../ota_local_backup/$d/main/"; done
   ```
2. Get the branch and build both images (`README.md`, section Build). From the repository root, in the ESP-IDF v5.4.4 PowerShell:
   ```powershell
   git fetch origin
   git switch water-test-firmware
   .\7_ProposalA_Learning\water_test\build_test_firmware.ps1
   ```
   The script has to finish without an error. It refuses any other ESP-IDF version or compiler build. The images go to `7_ProposalA_Learning/water_test/images/`, together with `SHA256SUMS.txt`.
3. **Do not change any Wi-Fi, hotspot or OTA password before the test.** C1, B and the rollback image all carry the current settings. If the passwords change first, an image with the old settings cannot reach the lab network and falls back to the open setup hotspot. The password change comes after the test (section 7).

## 4. On the test day

### 4.1 Before the first run

- Charge the battery fully and write down the voltage. Keep the ballast and the air lines as last time. Use the same water level and the same launch point all day.
- Use the console `4_PC_Console/target/release/squid-console2.exe`. The one in `target-pi-build` has no F7.
  - F7 starts only when F4 is not recording, the robot is within 15 cm of the surface, there is no depth fault, and the console has received panel data in the last 3 s.
  - The console writes the F7 logs to `7_ProposalA_Learning/data/auto-control-logs/` of the repository it was built in. The committed console was built in `D:\Squid-Robot-Delivery-20260922`. If the repository is somewhere else on the test PC, rebuild the console there first with `4_PC_Console\build.cmd`. It needs the Rust GNU toolchain, and internet access or a filled Cargo cache for the first build. It overwrites the tracked `target/release/squid-console2.exe`; do not commit that change.
- One person only watches the robot, with a hand on Esc, which always sends stop (`s`). Space sends stop only while the command input box is closed.
- Keep the rollback image and the C1 image at hand.
- Optional: run one F7 with the image already on the robot before flashing C1. That run links today's results to earlier data.

### 4.2 Flashing

The main test needs three uploads: C1 before run 1, B before run 3, C1 before run 7.

1. Compare the image's SHA-256 with `SHA256SUMS.txt` (`Get-FileHash <image>`) and write it in the run log.
2. Upload from the repository root, with the robot on Wi-Fi. `192.168.0.183` comes from DHCP and can change; on the robot's own hotspot the address is `http://192.168.4.1/ota`.
   ```powershell
   curl.exe --noproxy "*" --header "Content-Type: application/octet-stream" --header "Expect:" --data-binary "@7_ProposalA_Learning\water_test\images\B_<…>.bin" http://192.168.0.183/ota
   ```
3. After the reboot, from `7_ProposalA_Learning`:
   ```powershell
   python -B pc/collector.py --url http://192.168.0.183 status
   ```
   C1 and B report `"student_enabled": false`. The status has no image hash. A downloaded RAM capture tells the two images apart exactly: its `.json` lists the controller constants under `parameters`, and `KP` is 0.081 in C1 and 0.4215 in B. Before the first capture, `free_heap_bytes` about 6 KB lower on B than on C1 is a rough check.

**There is no automatic OTA rollback.** An image that boots but then crashes, or never brings up Wi-Fi, can only be replaced over USB with the case open. Do the first upload of C1 and the first upload of B where the case can be opened, and check `status` before the robot goes into the water.

### 4.3 Main test: 8 F7 runs in ABBA order

| Run | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |
|---|---|---|---|---|---|---|---|---|
| Image | C1 | C1 | B | B | B | B | C1 | C1 |

This order gives both images the same average position in the day. That cancels slow, steady drifts such as battery charge and water temperature.

For each run (the `collector.py` commands run from `7_ProposalA_Learning`; `--output-prefix` is relative to it):

1. Robot at the surface (depth reading ≤ 15 cm). Wait at least 1 minute so the 5 s pressure balancing after the last `s` has finished.
2. Optional: start a 60 s RAM capture. It covers the descent and most of the 30 cm hold, and it is used to recalibrate the model; the hold metrics come from the F7 log.
   ```powershell
   python -B pc/collector.py --url http://192.168.0.183 start --duration 60
   ```
3. Press F7. The console sends `l30`, `l40` and `l50`, 60 s each, then `l0`. When the robot is back within 15 cm it sends stop and closes the log.
4. Back at the surface, download the RAM capture, then clear it:
   ```powershell
   python -B pc/collector.py --url http://192.168.0.183 download --output-prefix data/test-<date>-<run>-<image>
   python -B pc/collector.py --url http://192.168.0.183 clear
   ```
5. Fill in the run log at once. The F7 log (`control-auto-<timestamp>.csv`) does not record which image ran.

| Run | Time | Image | `.bin` SHA-256 | CSV file | RAM capture | Battery voltage | Notes (aborts, anything unusual) |
|---|---|---|---|---|---|---|---|
| 1 | | C1 | | | | | |

### 4.4 Optional: step test with an ascent (after the main test, only if B passed)

One run each for C1 and B, about 6 minutes each. C1 is on the robot after run 8, so do C1's run first, then upload B for its run, and decide afterwards which image stays on the robot. Turn on F4 logging, then send by hand: `l40` (wait 90 s), `l30` (90 s), `l50` (90 s), `l35` (90 s), `l0`.

In simulation, the mean hold RMS is about 1.82 cm for C1 and 0.94 cm for B. Those scenarios differ from the tuning ones: delays of 0.6–1.0 s, other random seeds, this step schedule, the calibrated plant plus fits from three other days, and disturbances σ of 0, 0.29 and 0.575 cm/s². The calibrated plant and σ 0 and 0.29 were also used in tuning. They average over the three disturbance levels, so they cannot be compared with the table in section 1.

### 4.5 Optional: measure the loop delay (no firmware change)

The model fits disagree on the delay: 0.55–0.65 s from replaying logs, but 0.85 s to reproduce the oscillation in closed loop. This measurement settles it. The `collector.py` commands run from `7_ProposalA_Learning`.

1. Send `l40`. When the robot has settled at 40 cm (about 30 s), start a 60 s RAM capture.
2. Send `k` (full-power ascent on), and about 1 s later `k` again (off). This cancels the depth hold. After about 5 s, send `l40` to resume. Repeat 3 times.
3. Download the capture and clear it. Then start a new 60 s capture and do the same 3 times with `j` (full-power descent), and download and clear again. A capture that is still held blocks `start`, so use one capture per direction.

The capture records the commanded direction and the raw depth at 20 Hz, so the timing of the key presses need not be exact. This measures the delay at full power (PWM 255). Short low-power pulses may behave differently.

## 5. Abort conditions

Neither the console nor the firmware checks these. The observer stops the run by hand when any of them occurs.

The stop conditions of the earlier trial runs on the PC:

- depth ≥ 80 cm;
- depth invalid or not updated for more than 2 s;
- a command error;
- an ascent that has not reached the surface after 60 s.

Because B is a new control law, also stop when any of these occurs:

- A hold oscillates widely and keeps doing so (peak-to-peak above 15 cm), or the robot touches the surface or the bottom while holding.
- The pump or the valves switch abnormally often (more than 3 times per second for a sustained time, which can be heard). The simulation expects about 1.2 per second, close to today's controller.
- If two B runs in a row are aborted, stop testing B for the day.

To abort, press Esc (`s`). For the next 5 s the robot balances the pressure and refuses every motion command (`l…`, `k`, `j` and the like); `s` is still accepted. Wait until the console log shows `Global balance complete. System ready.` (the red lock banner also disappears; if that line never arrives, wait more than 5 s after the `EMERGENCY STOP` line). Then bring the robot up with `l0` or `k`, and flash C1. If C1 also misbehaves, flash the rollback image.

## 6. After the test

1. Put the `control-auto-*.csv` files, the RAM captures and the run log into `7_ProposalA_Learning/data/`. Do not commit the firmware images; `images/` is ignored on purpose.
2. Compute each run's metric as defined in section 2 and check the criteria one by one.
3. Re-run the plant calibration with the new RAM captures, including the delay measurement, to see whether the delay or the pump gains have changed. The calibration code is outside this repository.
4. Next step by outcome:
   - **Pass:** B becomes the new control. The learned correction stays off; it was trained for the old controller.
   - **Fail:** go back to C1 and study the RAM captures. Measure the delay (4.5), recalibrate the model and retune in simulation before the next water test.
   - **Inconclusive:** measure the delay and recalibrate first, then decide how to retune.

## 7. After the test: change the passwords

The Wi-Fi, hotspot and OTA passwords are exposed in the public history of this repository, so they are changed after the test. That needs a final image built from the variant chosen in section 6 and carrying the new passwords. Two things have to happen first:

- `build_test_firmware.ps1` takes `OtaConfig.local.h` from commit `deef5845`, so everything it builds today carries the old passwords. It has to be changed to use a local `OtaConfig.local.h` before it can build that final image.
- The rollback image is a prebuilt binary and cannot carry new passwords. If the test ends with keeping it, a variant that can be rebuilt from source has to replace it first.

The switch-over has its own procedure, kept outside the repository. Its order keeps the robot reachable throughout. After the change, do not flash old images and do not commit new firmware builds while the repository is public.
