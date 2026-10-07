# Depth residual learning

## Current deployment

The firmware uses the preserved five-feature PD checkpoint from
`artifacts/depth_model_1790936043/`, re-exported to
`artifacts/depth_model_pd_restore_1790936043/` with the full +/-20 output range.
The added error-memory workflow has been removed. The original PID integral
remains unchanged. History/inference and CSV telemetry use a 500 ms schedule.

```text
u_total = clamp(u_base + depth_residual, -100, 100)
```

PID base output is limited to +/-80 and the neural correction to +/-20.
Firmware embeds the ESP-DL model and its matching normalization header.

## Data and teacher

Use the PC console's CSV files; no SD card is required. Required columns:

```text
elapsed_s,depth_cm,depth_speed_cm_s,depth_accel_cm_s2,target_depth_cm,pid_base,buoyancy_pwm
```

Prepared CSVs may include session_id to prevent windows crossing experiments.
Without it, each CSV is treated as one session: split independent experiments
and target-change segments before training. Invalid numeric rows are skipped.
forward_active may remain in logs but is not a model input.

```text
label = clamp(teacher_kp * (target_depth - depth)
              - teacher_kd * depth_speed, -20, 20)
```

The deployed checkpoint used Kp=0.5 and Kd=2.0. These labels imitate a PD rule;
they are not measured optimal corrections or a guarantee of depth accuracy.
The five inputs per frame are error, speed, acceleration, PID base and PWM.
A five-frame window therefore contains 25 inputs.

## Training

Run from this directory using Python with PyTorch:

```powershell
python -m learning.train `
  --csv artifacts\depth_model_1790936043\training_data.csv `
  --output-dir artifacts\depth_model_new `
  --sample-interval-ms 500 --window 5 `
  --teacher-kp 0.5 --teacher-kd 2.0 --epochs 300 --seed 42
```

Validation uses a chronological holdout with overlapping boundary windows
purged. Use --split-by-session to hold out whole independent sessions.
Normalization uses only training windows; the best validation checkpoint is saved.

## ESP32-S3 export

Use Python with PyTorch, NumPy, ONNX and ESP-PPQ. To export the preserved model
without overwriting the deployed artifacts:

```powershell
python -m learning.export_espdl `
  --model artifacts\depth_model_1790936043\depth_model.pt `
  --csv artifacts\depth_model_1790936043\training_data.csv `
  --onnx-output artifacts\depth_model_export_new\depth_model.onnx `
  --espdl-output artifacts\depth_model_export_new\depth_model_espdl.espdl `
  --metadata-output artifacts\depth_model_export_new\DepthResidualModelConfig.h
```

Normalize inputs before INT8 quantization using the matching header. Calibration
uses representative training windows, not validation windows. Output quantization
is checked to represent both -20 and +20. The validation report compares quantized
predictions against the teacher and floating-point model, not closed-loop accuracy.

Keep the .espdl model and DepthResidualModelConfig.h paired. Firmware's
main/CMakeLists.txt selects the deployed directory. A new build and upload are
required to deploy another model. Original CSVs, graphs, basic model artifacts
and PC console directories are retained separately from removed experiments.
