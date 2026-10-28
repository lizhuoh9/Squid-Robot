"""Offline split, causality, plant, teacher and deployment-gate checks."""
import json
import re
import sys
import unittest
from unittest import mock
from pathlib import Path
import numpy as np
import torch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"learning"))
import pipeline as p


class LearningTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        torch.set_num_threads(1)
        cls.data=p.read_data()
        cls.pc_data=p.read_data([ROOT.parent/"4_PC_Console/target-pi-build/release/squid-logs/control-1791877820.csv"])
    def test_dimension_and_finiteness(self):
        self.assertEqual(self.data["x"].shape[1],p.DYNAMICS_DIM)
        self.assertEqual(self.data["policy"].shape[1],len(p.FEATURES))
        self.assertTrue(torch.isfinite(self.data["y"]).all())
    def test_existing_pc_control_csv_is_trainable(self):
        self.assertEqual(self.pc_data["provenance"][0]["format"],"pc_control_200ms")
        self.assertEqual(self.pc_data["provenance"][0]["record_count"],1343)
        self.assertGreaterEqual(int(self.pc_data["split"].max()),2)
        self.assertTrue(np.all(np.bincount(self.pc_data["split"],minlength=3)>=20))
        self.assertTrue(torch.isfinite(self.pc_data["x"]).all())
        self.assertTrue(torch.isfinite(self.pc_data["u"]).all())
    def test_temporal_embargo(self):
        for session,(first,second) in enumerate(self.data["split_bounds"]):
            for split,start,end in ((0,None,first),(1,first,second),(2,second,None)):
                times=self.data["times"][(self.data["session"]==session)&(self.data["split"]==split)]
                self.assertGreater(len(times),0)
                if start is not None:
                    self.assertGreaterEqual(times.min()-3,start-1e-6)
                if end is not None:
                    self.assertLessEqual(times.max()+3,end+1e-6)
    def test_forward_active_rows_remain_trainable(self):
        path=Path(self.pc_data["provenance"][0]["path"])
        with path.open(encoding="utf-8-sig") as handle:
            rows=list(p.csv.DictReader(handle))
        forward_rows=[dict(row,forward_active="1") for row in rows]
        with mock.patch.object(p.csv,"DictReader",return_value=forward_rows):
            forward_data=p.read_data([path])
        self.assertTrue(np.array_equal(forward_data["times"],self.pc_data["times"]))
        self.assertTrue(torch.equal(forward_data["x"][:,:33],self.pc_data["x"][:,:33]))
        self.assertTrue(torch.all(forward_data["x"][:,p.DYNAMICS_FORWARD]==1))
        self.assertTrue(torch.allclose(forward_data["x"][:,p.DYNAMICS_FORWARD_HISTORY],torch.ones_like(forward_data["x"][:,p.DYNAMICS_FORWARD_HISTORY])))
        self.assertTrue(torch.equal(forward_data["u"],self.pc_data["u"]))
        self.assertEqual(forward_data["provenance"][0]["forward_active_row_count"],len(rows))
        self.assertEqual(forward_data["provenance"][0]["forward_active_window_counts"],
                         np.bincount(forward_data["split"],minlength=3).tolist())
    def test_causal_history_contract(self):
        self.assertTrue(np.array_equal(self.data["policy"][:,p.POLICY_COMMANDS],self.data["x"][:,p.DYNAMICS_COMMANDS].numpy()))
        self.assertTrue(np.all(np.abs(self.data["policy"][:,p.POLICY_COMMANDS])<=1))
        self.assertTrue(np.array_equal(self.data["policy"][:,p.POLICY_FORWARD:],self.data["x"][:,p.DYNAMICS_FORWARD:].numpy()))
    def test_future_forward_does_not_leak_into_inputs(self):
        path=Path(self.pc_data["provenance"][0]["path"])
        with path.open(encoding="utf-8-sig") as handle:rows=list(p.csv.DictReader(handle))
        switch=float(rows[len(rows)//2]["elapsed_s"])
        zero=[dict(row,forward_active="0") for row in rows]
        changed=[dict(row,forward_active=str(int(float(row["elapsed_s"])>=switch))) for row in rows]
        with mock.patch.object(p.csv,"DictReader",return_value=zero):baseline=p.read_data([path])
        with mock.patch.object(p.csv,"DictReader",return_value=changed):switched=p.read_data([path])
        before=baseline["times"]<switch
        self.assertTrue(torch.equal(baseline["x"][before],switched["x"][before]))
        self.assertTrue(np.array_equal(baseline["policy"][before],switched["policy"][before]))
        self.assertTrue(torch.any(baseline["forward_u"][before]!=switched["forward_u"][before]))
    def test_hybrid_shapes_and_gradient(self):
        model=p.Hybrid();pred=model(self.data["x"][:2],self.data["u"][:2],self.data["forward_u"][:2])
        self.assertEqual(tuple(pred.shape),(2,30,2))
        pred.square().mean().backward()
        self.assertTrue(all(param.grad is not None for param in model.parameters()))
        self.assertTrue(torch.isfinite(pred).all())
    def test_identical_plans_have_identical_initial_phase(self):
        model=p.Hybrid().eval();ids=np.array([100,200])
        features=np.c_[self.data["policy"][ids],self.data["x"][ids,0].numpy()+40]
        costs,uncertainty=p.teacher_rollout([model,model,model],features,np.zeros((3,4),np.float32))
        self.assertTrue(torch.allclose(costs[:,0],costs[:,1],atol=1e-6))
        self.assertTrue(torch.allclose(costs[:,0],costs[:,2],atol=1e-6))
        self.assertTrue(torch.isfinite(costs).all())
        self.assertLess(uncertainty.max().item(),1e-4)
    def test_zero_student_equals_base_closed_loop(self):
        model=p.Hybrid().eval();student=p.Student().eval()
        for param in student.parameters():param.data.zero_()
        ids=np.array([100,200]);features=np.c_[self.data["policy"][ids],self.data["x"][ids,0].numpy()+40]
        plans=np.zeros((1,4),np.float32)
        base,_=p.teacher_rollout([model]*3,features,plans)
        corrected,_=p.teacher_rollout([model]*3,features,plans,student=student,stats=(torch.zeros(len(p.FEATURES)),torch.ones(len(p.FEATURES))))
        self.assertTrue(torch.allclose(base,corrected,atol=1e-6))
    def test_student_and_same_plan_pay_identical_actual_correction_cost(self):
        model=p.Hybrid().eval();student=p.Student().eval()
        for param in student.parameters():param.data.zero_()
        student.fc3.bias.data.fill_(.15)
        features=np.zeros((2,len(p.FEATURES)+1),np.float32);features[:,3]=1.1;features[:,-1]=40
        planned,_,plan_detail=p.teacher_rollout([model]*3,features,np.full((1,4),.15,np.float32),return_details=True)
        learned,_,detail=p.teacher_rollout([model]*3,features,np.zeros((1,4),np.float32),student=student,
                                         stats=(torch.zeros(len(p.FEATURES)),torch.ones(len(p.FEATURES))*100),return_details=True)
        self.assertTrue(torch.allclose(planned,learned,atol=1e-5))
        self.assertTrue(torch.allclose(detail["cost_components"]["correction"],torch.full((2,1),.2*.15**2*4),atol=1e-6))
        self.assertTrue(torch.allclose(sum(detail["cost_components"].values()),learned,atol=1e-5))
    def test_dagger_uses_only_train_seeds_and_preserves_controller_context(self):
        model=p.Hybrid().eval();student=p.Student().eval()
        for param in student.parameters():param.data.zero_()
        policy=np.zeros((4,len(p.FEATURES)),np.float32);policy[:,3]=1.1
        data={"policy":policy,"x":torch.zeros((4,p.DYNAMICS_DIM)),"split":np.array([0,0,1,2]),
              "session":np.array([7,7,99,100]),"times":np.array([10.,20.,30.,40.])}
        stats=(torch.zeros(len(p.FEATURES)),torch.ones(len(p.FEATURES))*100)
        extra,labels,sessions,times=p.collect_dagger(data,[model]*3,student,stats,1)
        self.assertGreater(len(extra),0)
        self.assertTrue(np.all(sessions==7))
        self.assertTrue(np.isfinite(extra).all())
        self.assertTrue(np.all(np.abs(labels)<=.3))
        _,_,detail=p.teacher_rollout([model],np.c_[policy[:1],np.array([40.])],np.zeros((1,4),np.float32),collect_ticks=(20,))
        snapshot=detail["snapshots"][0];context=snapshot["context"]
        self.assertTrue(torch.equal(context["history"],snapshot["features"][:,p.POLICY_COMMANDS]))
        expected_bias=1.1+.03*(context["state"][:,0]-40)+p.CONTROL_KI*context["integral"]
        self.assertTrue(torch.allclose(expected_bias,snapshot["features"][:,3]))
        self.assertTrue(torch.all(context["on_until"]>=0))
        self.assertTrue(torch.all(context["lock_until"]>=0))
    def test_offline_gate_and_explicit_user_activation(self):
        report=json.loads((ROOT/"artifacts/hybrid_mpc_v1/training_report.json").read_text())
        self.assertFalse(report["deployment_gate"]["enabled"])
        config=(ROOT/"artifacts/hybrid_mpc_v1/ProposalStudentConfig.h").read_text()
        if "kEnabled = true" in config:
            local=json.loads((ROOT/"artifacts/hybrid_mpc_v1/integration_report.json").read_text(encoding="utf-8"))
            self.assertTrue(local["experimental_user_override"])
            self.assertTrue(local["correction_enabled"])
        else:
            self.assertIn("kEnabled = false",config)
        exported=json.loads((ROOT/"artifacts/hybrid_mpc_v1/espdl_validation.json").read_text(encoding="utf-8"))
        limit=float(re.search(r"kLimit\s*=\s*([0-9.eE+-]+)f",config).group(1))
        self.assertAlmostEqual(limit,exported["limit_cm_s2"])
        self.assertGreaterEqual(len(report["sources"]),1)
    def test_runtime_limit_can_expand_without_changing_teacher_limit(self):
        model=p.Hybrid().eval();student=p.Student().eval()
        for param in student.parameters():param.data.zero_()
        student.fc3.bias.data.fill_(2)
        features=np.zeros((1,len(p.FEATURES)+1),np.float32);features[:,3]=1.1;features[:,-1]=40
        stats=(torch.zeros(len(p.FEATURES)),torch.ones(len(p.FEATURES))*100)
        _,_,default=p.teacher_rollout([model],features,np.zeros((1,4),np.float32),student=student,
                                     stats=stats,return_details=True)
        _,_,expanded=p.teacher_rollout([model],features,np.zeros((1,4),np.float32),student=student,
                                      stats=stats,return_details=True,correction_limit=1.0)
        self.assertAlmostEqual(default["cost_components"]["correction"].item(),.2*.3**2*4,places=5)
        self.assertAlmostEqual(expanded["cost_components"]["correction"].item(),.2*1.0**2*4,places=5)
        self.assertAlmostEqual(float(np.abs(p.PLANS).max()),.3,places=6)
        for limit in (0,-1,float("nan")):
            with self.assertRaises(ValueError):
                p.teacher_rollout([model],features,np.zeros((1,4),np.float32),correction_limit=limit)

    def test_range_objective_penalizes_excursion_speed_and_correction_changes(self):
        model=p.Hybrid().eval()
        features=np.zeros((2,len(p.FEATURES)+1),np.float32)
        features[:,0]=4;features[:,1]=2;features[:,3]=1.1;features[:,-1]=40
        plans=np.array([[.5,-.5,.5,-.5]],np.float32)
        legacy,_,old=p.teacher_rollout([model],features,plans,correction_limit=.5,return_details=True)
        cost,_,detail=p.teacher_rollout([model],features,plans,correction_limit=.5,objective="range",return_details=True)
        self.assertTrue(torch.all(cost>legacy))
        for key in ("outside_band","braking","correction_change","peak"):
            self.assertTrue(torch.all(detail["cost_components"][key]>0))
        self.assertTrue(torch.allclose(sum(detail["cost_components"].values()),cost,atol=1e-4))
        self.assertTrue(torch.all(detail["p95_abs_error_cm"]<=detail["max_abs_error_cm"]))
        self.assertTrue(torch.allclose(detail["depth_mae_cm"],old["depth_mae_cm"]))

    def test_range_acceptance_requires_both_range_metrics_and_mae(self):
        previous={"evaluation_window_sha256":"same","student_depth_mae_cm":2.,
                  "student_p95_abs_error_cm":4.,"student_max_abs_error_cm":6.}
        candidate={"evaluation_window_sha256":"same","student_depth_mae_cm":1.9,
                   "student_p95_abs_error_cm":3.5,"student_max_abs_error_cm":5.5,
                   "base_depth_mae_cm":2.,"base_p95_abs_error_cm":4.,"base_max_abs_error_cm":6.}
        self.assertTrue(p.range_acceptance(candidate,previous)["passed"])
        for key in ("depth_mae_cm","p95_abs_error_cm","max_abs_error_cm"):
            worse={**candidate,"student_"+key:previous["student_"+key]+.1}
            self.assertFalse(p.range_acceptance(worse,previous)["passed"])
        with self.assertRaises(ValueError):
            p.range_acceptance({**candidate,"evaluation_window_sha256":"different"},previous)

    def test_missing_sensor_row_is_excluded_not_interpolated_into_training(self):
        path=Path(self.pc_data["provenance"][0]["path"])
        with path.open(encoding="utf-8-sig") as handle:rows=list(p.csv.DictReader(handle))
        row=rows[len(rows)//3];missing_time=float(row["elapsed_s"])
        row["depth_cm"]=""
        with mock.patch.object(p.csv,"DictReader",return_value=rows):data=p.read_data([path])
        self.assertGreater(len(data["times"]),0)
        self.assertTrue(torch.isfinite(data["y"]).all())
        self.assertTrue(np.all(np.abs(data["times"]-missing_time)>2.9))


if __name__=="__main__":unittest.main()
