"""Run fixed proposal plant variants; no hardware I/O or automatic uploads."""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import statistics
import subprocess

ROOT = Path(__file__).resolve().parents[1]
# The eight calibrated plants and three stress cases are from Proposal A,
# NOT refitted to the newly reduced PID or held-out physical measurements.
PLANTS = [
    [1.0, 2.0, 2.0, 0.1, 1.1, 0.0], [1.0, 2.0, 2.0, 0.1, 1.1, 0.03],
    [1.0, 3.0, 1.5, 0.1, 1.1, 0.0], [0.5, 3.0, 1.5, 0.1, 1.1, 0.03],
    [1.0, 2.0, 1.5, 0.1, 1.1, 0.03], [1.0, 3.0, 1.5, 0.1, 0.5, 0.03],
    [0.5, 2.0, 1.5, 0.1, 1.1, 0.03], [1.0, 3.0, 1.5, 0.1, 1.1, 0.03],
    [0.5, 4.0, 2.0, 0.4, 1.1, 0.03], [0.3, 2.0, 3.0, 0.6, 1.5, 0.05],
    [1.0, 2.0, 1.5, 0.4, 0.5, 0.0],
]
METRICS = ["settled_rms_cm", "within_1cm_pct", "within_0_5cm_pct",
           "largest_overshoot_cm", "valve_actions_per_min", "average_pump_duty_pct"]


def main():
    env = os.environ.copy()
    env["PATH"] = "C:\\MinGW\\bin;" + env.get("PATH", "")
    manifest = json.loads((ROOT / "artifacts/baseline_manifest.json").read_text(encoding="utf-8"))
    changed = [entry["Path"] for entry in manifest["baseline_files"]
               if hashlib.sha256(Path(entry["Path"]).read_bytes()).hexdigest().upper() != entry["SHA256"]]
    if changed:
        raise RuntimeError(f"Original files changed since snapshot: {changed}")
    contract = subprocess.run([str(ROOT / "simulation/bin/controller_contract.exe")],
                              capture_output=True, text=True, check=True, env=env, timeout=10)
    report = {"physical_test_performed": False, "uploaded": False,
              "baseline_pid": manifest["baseline_pid"], "baseline_neural_residual": "disabled host stub",
              "compiler": "MinGW GCC 6.3, C++14 compatibility mode",
              "schedule_cm": [40, 50, 40, 30, 40], "seconds_per_target": 90,
              "settled_exclusion_seconds": 40, "plant_parameter_order": ["Ks", "Kr", "tauW", "L", "aRest", "alpha"],
              "contract_tests": contract.stdout.strip(), "runs": {},
              "limitations": ["Simulation is not a measured performance guarantee",
                              "Baseline is frozen current PID 10/0.8/15 without its deployed ESP-DL residual",
                              "Plant variants were calibrated in the proposal, not independently identified here",
                              "Surface/floor boundary and linear pump assumptions do not capture all robot behavior"]}
    for controller, exe in [("proposal_a", "sim_proposal_a.exe"), ("current_pid_only", "sim_current_pid_only.exe")]:
        cases = []
        for index, plant in enumerate(PLANTS):
            seeds = []
            for seed in (1, 2, 3):
                output = subprocess.run([str(ROOT / "simulation/bin" / exe), "eval", *map(str, plant), str(seed)],
                                        env=env, capture_output=True, text=True, check=True, timeout=20)
                values = list(map(float, output.stdout.split()))
                if len(values) != len(METRICS):
                    raise RuntimeError(f"Unexpected simulator output: {output.stdout}")
                seeds.append(dict(zip(METRICS, values)))
            average = {key: statistics.mean(row[key] for row in seeds) for key in METRICS}
            cases.append({"plant_id": index + 1, "parameters": plant, "seed_results": seeds, "mean": average})
        calibrated, all_cases = cases[:8], cases
        report["runs"][controller] = {"cases": cases,
            "calibrated_8_mean": {key: statistics.mean(c["mean"][key] for c in calibrated) for key in METRICS},
            "all_11_mean": {key: statistics.mean(c["mean"][key] for c in all_cases) for key in METRICS},
            "all_11_worst_rms_cm": max(c["mean"]["settled_rms_cm"] for c in all_cases)}
        print(controller, json.dumps(report["runs"][controller]["calibrated_8_mean"]), flush=True)
    report["baseline_preserved_files"] = len(manifest["baseline_files"])
    output_path = ROOT / "artifacts/simulation_report.json"
    output_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"saved {output_path}")


if __name__ == "__main__":
    main()
