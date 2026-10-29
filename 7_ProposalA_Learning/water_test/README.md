# Water-test firmware: C1 and B

This folder builds the two firmware images for the next water test of `7_ProposalA_Learning`, and keeps the record of what was checked before any image was built. The images are built from copies; nothing here changes the sources in this repository, and nothing is flashed automatically.

Paths start at the repository root. Files of this folder are named relative to this folder (`evidence/…`, `images/…`).

## The images

| Image | What it is | Role in the test |
|---|---|---|
| **C1** | The sources of commit `deef5845` with the learned correction switched off (`kEnabled = false` in `7_ProposalA_Learning/artifacts/hybrid_mpc_v1/ProposalStudentConfig.h`) | Control |
| **B** | C1 + `option_b.patch` | New depth controller |
| Robot image | `7_ProposalA_Learning/artifacts/hybrid_mpc_v1/rollback-enabled-limit0p5-5d2fad78ecfa.bin`, 1,115,552 bytes, SHA-256 `5d2fad78…`. The 2026-10-29 entry of `7_ProposalA_Learning/artifacts/hybrid_mpc_v1/deployment_report.json` says it was put back on the robot after a candidate image failed its acceptance test. It is the second deployment of 2026-10-22 (residual limit 0.5), with an older student model, not built from `deef5845`. In its file name, `enabled` refers to the learned correction (a sibling `rollback-disabled-…` exists), not to OTA rollback | Rollback |
| C0 | The image deployed on 2026-10-27, built from the sources of `deef5845`: `7_ProposalA_Learning/firmware/build-learning/squid_robot_a_recorder.bin` at that commit, 1,118,784 bytes, SHA-256 `b825bc7c…` | Shows that these sources ran on the robot; a second fallback |

**Why the sources are pinned to `deef5845`** ("Sync all project files as of 2026-10-28"). B was tuned and checked in simulation against the `DepthController` of that commit, and C0, built from the same sources, ran on the robot. Later commits change `DepthController` (residual-RL hooks, a braking zone near the target, limits on the sink duty) and the image in `build-learning`. The script therefore exports the pinned commit with `git archive`, and it does not matter what is checked out.

C1 and B differ only in the controller, so the test compares the controllers alone. The learned correction stays off in both, because it was trained for the old controller.

To get C0 as a file (in PowerShell, `cmd /c` keeps the bytes intact):

```powershell
cmd /c "git show deef5845:7_ProposalA_Learning/firmware/build-learning/squid_robot_a_recorder.bin > C0_b825bc7c.bin"
```

## What `option_b.patch` changes

Only `7_ProposalA_Learning/firmware/main/DepthController.h` and `DepthController.cpp` next to it.

| Constant | `deef5845` (C1) | B |
|---|---|---|
| `KP` | 0.081 | 0.421474822 |
| `KD` | 0.54 | 0.930500945 |
| `KI` | 0.009 | 0.0222352216 |
| `G_SINK` | 0.75 | 2.80359102 |
| `G_RISE` | 4.5 | 4.28295333 |
| `A_REST_REF` | 1.1 | 0.299659525 |
| `ALPHA` | 0.03 | 0.0179 |
| `MOD_WINDOW_MS` | 1000 | 1069 |
| `FLIP_MIN_DUTY` | 0.05 | 0.139697858 |
| `PRED_HORIZON_S` | – | 1.0 |
| `PRED_LOCK_MS` | – | 200 |

`I_LIMIT`, `DEPTH_REF` and `DUTY_MIN` are unchanged. The constants keep all the digits the simulation used.

**Delay predictor.** The PD term acts on the depth error and speed expected `PRED_HORIZON_S` ahead instead of the measured ones. The prediction adds the pump accelerations commanded during the last `PRED_HORIZON_S`, which have not acted yet, and the bias, which acts throughout. The commands are kept in a 512-entry ring buffer, one entry per 5 ms motion tick while a target depth is held and the depth reading is valid; entries older than the horizon are not used. A command counts as zero during the pump lock of `PRED_LOCK_MS` that follows a valve change on the Minima. The buffer is cleared in `resetAfterCalibration()`.

**Why.** A simulator calibrated on the Proposal A water logs put the hold oscillation down to a loop delay of about 0.55–0.85 s. Its stability criterion was a peak-to-peak of at most 3 cm while holding without disturbances. None of the constants-only settings tried met it; the tuned one still reached 6.5 cm. With the predictor the worst case in the tuning scenarios was 2.92 cm, a small margin.

B was tuned with a modified copy of `6_DepthHold_ProposalA/simulation/harness/sim_main.cpp`, which runs the calibrated plant model with the controller of `7_ProposalA_Learning` and adds a batch mode for tuning. That copy and the calibration and tuning code are not in this repository. The comments in the patch cite their report, `proposal_a_tuning/final_report.json`, which is not in the repository either.

## Requirements

- Windows PowerShell 5.1 or later.
- **ESP-IDF v5.4.4 with the toolchain `esp-14.2.0_20260121`**, active in that PowerShell. These are the versions C0 was built with: the version is in C0's app descriptor, the toolchain in the `.comment` section of its ELF. The script refuses other versions. ESP-IDF 5.3.2 cannot build this project, because the bundled esp-dl uses `MALLOC_CAP_SIMD`.
- `git` on the PATH, and Windows' own `tar.exe` (Windows 10 1803 and later).
- This repository with commit `deef5845` in it (`git fetch origin`).

## Build

From the repository root, in the ESP-IDF v5.4.4 PowerShell:

```powershell
.\7_ProposalA_Learning\water_test\build_test_firmware.ps1
```

Activating ESP-IDF already needs a PowerShell that runs scripts. If it refuses this one, allow scripts for that window only with `Set-ExecutionPolicy -Scope Process Bypass`, and run the script again in the same window. A new `powershell` process would not inherit the `idf.py` alias of the EIM activation.

The script writes to `images/`:

- `C1_<first 8 hex digits of the SHA-256>.bin` and `B_<…>.bin`, the images to upload;
- the matching `.elf` files, for decoding a crash;
- `.build.txt` build records, including the `idf.py size` report;
- `SHA256SUMS.txt`.

`images/` is ignored by git (`.gitignore` here). The build copies go to `\sqfw\c1` and `\sqfw\b` on the drive of the repository, because ESP-IDF cannot build under a path with spaces. They can be deleted afterwards.

| Option | Effect |
|---|---|
| `-PrepareOnly` | Export, modify and check the sources; no build |
| `-CompileCheck` | For a machine without v5.4.4: any ESP-IDF version; compiles only the sources of `7_ProposalA_Learning/firmware/main` with the commands CMake generated (through `compile_main_sources.py`); no image |
| `-Variant C1` or `-Variant B` | Only one image |
| `-WorkRoot`, `-OutDir`, `-RepoRoot` | Other folders |

The script stops with a message as soon as one of these checks fails:

1. Commit `deef5845` is in the repository.
2. Five exported files match pinned SHA-256 values: `ProposalStudentConfig.h`, `proposal_student.espdl`, `sdkconfig`, `DepthController.h` and `DepthController.cpp`. The rest come from the same commit. Text files are hashed with LF line ends, so git's line-end setting does not matter.
3. After the changes, only the intended files differ from the export, and they match the pinned results: for C1 the student configuration, for B also the two controller files.
4. Before building: ESP-IDF v5.4.4 and the compiler `esp-14.2.0_20260121` are active.
5. After building: the image's app descriptor names the project `squid_robot_a_recorder` and ESP-IDF v5.4.4, and `sdkconfig` was not rewritten.

## Flash and check

1. Compare the image's SHA-256 with `SHA256SUMS.txt` (`Get-FileHash <image>`), and write it in the run log.
2. Upload it over Wi-Fi. The command is the one in `1_ESP32_MainController/README.md`. In Windows PowerShell, `curl` is an alias of `Invoke-WebRequest`, so write `curl.exe`:
   ```powershell
   curl.exe --noproxy "*" --header "Content-Type: application/octet-stream" --header "Expect:" --data-binary "@7_ProposalA_Learning\water_test\images\B_<…>.bin" http://192.168.0.183/ota
   ```
   `192.168.0.183` is the address the robot has had on the lab network. It comes from DHCP and can change, so check it first. On the robot's own hotspot the address is `http://192.168.4.1/ota`.
3. After the reboot, from `7_ProposalA_Learning`:
   ```powershell
   python -B pc/collector.py --url http://192.168.0.183 status
   ```
   C1 and B report `"student_enabled": false`. The status has no image hash, and its `firmware` field is `hybrid_mpc_v1` in every image. B's `free_heap_bytes` should be about 6 KB lower than C1's (the predictor's static arrays), which tells the two apart roughly.
4. Roll back by uploading the robot image (`7_ProposalA_Learning/artifacts/hybrid_mpc_v1/rollback-enabled-limit0p5-5d2fad78ecfa.bin`) or C0 the same way.

OTA rollback is not enabled (`CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE` is not set). `esp_ota_end` rejects a corrupt upload, but an image that boots and then crashes or never brings up Wi-Fi can only be replaced over USB, with the case open. Flash a new image for the first time where the case can be opened.

## Checked on 2026-10-29 and 2026-10-30

1. **The pinned sources are the ones C0 was built from.**
   - `ProposalStudentConfig.h` (with CRLF line ends) and `proposal_student.espdl` hash to `config_sha256` and `embedded_espdl.sha256` of the 2026-10-27 deployment record (`7_ProposalA_Learning/artifacts/hybrid_mpc_v1/deployment_report.json` at `deef5845`).
   - `7_ProposalA_Learning/firmware/sdkconfig` holds the same 1421 options as `7_ProposalA_Learning/firmware/build-learning/config/sdkconfig.json` of the C0 build; the deprecated aliases at its end are left out of the comparison.
   - `check_c0_sources.py` reads C0's image and ELF at `deef5845`. The 12 `depth_cfg` constants and `kEnabled` in the debug information have the values of the sources (`kEnabled = 1`). Of 4603 declaration records in 22 compilation units, all but one name appear on the source line that the debug information gives; the exception is a lambda's implicit capture of `this`, on a line without the word. The script exits with 1 if any of this fails. It sees line shifts only where they move a later declaration, and constants only among those 13 values. Output: `evidence/check_c0_sources.txt`.
2. **The prepared copies.** C1 differs from the export only in line 6 of `ProposalStudentConfig.h`. B also differs in the two controller files, which match the sources the simulation verified. `test_build_script.ps1` prepares both once with git's `core.autocrlf` set to true and once to false; output in `evidence/script_checks.txt`.
3. **Compile check with ESP-IDF 5.3.2** (GCC 13.2, `esp-13.2.0_20240530`). The flags are the ones 5.3.2 generates for this project, which include `-Wall -Wextra -Werror=all`; compared with C0's they lack `-Wno-error=extra` and `-fuse-cxa-atexit`.
   - All 22 sources of `7_ProposalA_Learning/firmware/main` compile for C1 and for B.
   - Both give the same 3 warnings, in files the change does not touch: `uart_config_t::flags` is not initialised at `CommandHandler.cpp:170`, `Console.cpp:32` and `MotionLink.cpp:26`. `DepthController.cpp` gives none.
   - B's `DepthController.cpp` object has 6,157 bytes more `.bss` (three 2,048-byte arrays and 13 bytes of variables), 504 bytes more `.text` and 120 bytes more `.literal`.
   - Logs: `evidence/compile_check_C1_idf5.3.2.txt`, `evidence/compile_check_B_idf5.3.2.txt`, `evidence/depthcontroller_obj_size_idf5.3.2.txt`.
4. **The build path of the script.** `test_build_script.ps1` runs it with stand-ins for `idf.py` and the compiler that return C0's image as the build output. The script wrote the images, ELF files, build records and `SHA256SUMS.txt`, and it stopped when the stand-in reported ESP-IDF v5.3.2 or a different compiler build. Output in `evidence/script_checks.txt`.

## Not checked yet

- No image has been built with v5.4.4, so nothing has been linked and there is no image size.
- GCC 14.2 may warn where 13.2 did not.
- Neither image has run on the robot.

## Files

| File | Purpose |
|---|---|
| `build_test_firmware.ps1` | Exports, modifies, checks and builds C1 and B |
| `compile_main_sources.py` | Runs the compile commands of `7_ProposalA_Learning/firmware/main` one by one, for `-CompileCheck` |
| `check_c0_sources.py` | Compares C0's debug information with the sources at `deef5845` (needs pyelftools; the ESP-IDF Python has it) |
| `test_build_script.ps1` | Runs the build script with both line-end settings and with stand-ins for ESP-IDF; run it in a PowerShell without ESP-IDF. It rebuilds `\sqfw\c1` and `\sqfw\b`, recreates `\sqfw\test-images` and `\sqfw\test-c0`, and rewrites `evidence/script_checks.txt` |
| `option_b.patch` | The change from C1 to B |
| `evidence/` | Outputs of the checks above |
