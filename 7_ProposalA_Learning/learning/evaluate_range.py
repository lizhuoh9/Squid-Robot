"""Frozen A-only run evaluation; never train, select a model, or send commands."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import torch

from pipeline import ROOT, Hybrid, Student, read_data, paired_evaluation, range_acceptance


def load_student(directory):
    checkpoint=torch.load(directory/"student.pt",map_location="cpu",weights_only=False)
    model=Student().eval();model.load_state_dict(checkpoint["state"])
    return model,(torch.tensor(checkpoint["input_mean"]),torch.tensor(checkpoint["input_std"]))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--previous",type=Path,required=True)
    parser.add_argument("--csv",type=Path,required=True)
    args=parser.parse_args()
    torch.set_num_threads(1)
    out=ROOT/"artifacts/hybrid_mpc_v1"
    training=json.loads((out/"training_report.json").read_text(encoding="utf-8"))
    if str(args.csv.resolve()) in [str(Path(s["path"]).resolve()) for s in training["sources"]]:
        raise ValueError("The external evaluation run must not be a training source")
    # This frozen run was logged every 500ms, unlike the 200ms training runs.
    # Keep its actual cadence and do not bridge gaps exceeding 800ms.
    data=read_data([args.csv],pc_max_gap=.800001)
    import csv
    with args.csv.open(encoding="utf-8-sig") as handle: rows=list(csv.DictReader(handle))
    segments=[]
    for row in rows:
        target=float(row["target_depth_cm"]) if row["target_depth_cm"].strip() else None
        t=float(row["elapsed_s"])
        if not segments or segments[-1][0]!=target:segments.append([target,t,t])
        else:segments[-1][2]=t
    keep=np.zeros(len(data["times"]),bool)
    for target,start,end in segments:
        if target in (30,40,50):
            keep|=(data["times"]>=start+20)&(data["times"]<=min(start+60,end)-4)
    ids=np.flatnonzero(keep)[::4]
    if not len(ids):raise ValueError("No eligible fixed steady-hold evaluation windows")
    dynamics=torch.load(out/"dynamics_ensemble.pt",map_location="cpu",weights_only=False)
    members=[]
    for state in dynamics["states"]:
        model=Hybrid().eval();model.load_state_dict(state);members.append(model)
    old=paired_evaluation(data,ids,members,*load_student(args.previous),correction_limit=.5,objective="range")
    new=paired_evaluation(data,ids,members,*load_student(out),correction_limit=.5,objective="range")
    report={"protocol":"Frozen A-only run, 20-60s hold starts with full 4s horizon; same new dynamics ensemble, phase, windows, +/-0.5 limit for previous/new students. Not measured new-controller performance.",
            "sources":data["provenance"],"previous_student_sha256":hashlib.sha256((args.previous/"student.pt").read_bytes()).hexdigest(),
            "student_sha256":hashlib.sha256((out/"student.pt").read_bytes()).hexdigest(),
            "previous":old,"candidate":new,"acceptance":range_acceptance(new,old),
            "by_target_acceptance":{k:range_acceptance({**v,"evaluation_window_sha256":new["evaluation_window_sha256"]},
                                                      {**old["by_target_cm"][k],"evaluation_window_sha256":old["evaluation_window_sha256"]})
                                    for k,v in new["by_target_cm"].items()},
            "limitations":["500ms command/sensor sampling cannot recover faster events", "simulated 4s rollouts, not a measured sustained depth range", "do not use this final evaluation to tune or select another model"]}
    (out/"range_external_evaluation.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
    print(json.dumps(report,indent=2))


if __name__=="__main__":main()
