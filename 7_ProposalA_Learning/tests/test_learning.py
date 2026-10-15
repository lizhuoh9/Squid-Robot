"""Offline split, causality, plant, teacher and deployment-gate checks."""
import json
import sys
import unittest
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
        self.assertEqual(self.data["x"].shape[1],33)
        self.assertEqual(self.data["policy"].shape[1],34)
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
    def test_causal_history_contract(self):
        self.assertTrue(np.array_equal(self.data["policy"][:,4:],self.data["x"][:,3:].numpy()))
        self.assertTrue(np.all(np.abs(self.data["policy"][:,4:])<=1))
    def test_hybrid_shapes_and_gradient(self):
        model=p.Hybrid();pred=model(self.data["x"][:2],self.data["u"][:2])
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
        corrected,_=p.teacher_rollout([model]*3,features,plans,student=student,stats=(torch.zeros(34),torch.ones(34)))
        self.assertTrue(torch.allclose(base,corrected,atol=1e-6))
    def test_failed_gate_and_disabled_correction(self):
        report=json.loads((ROOT/"artifacts/hybrid_mpc_v1/training_report.json").read_text())
        self.assertFalse(report["deployment_gate"]["enabled"])
        config=(ROOT/"artifacts/hybrid_mpc_v1/ProposalStudentConfig.h").read_text()
        self.assertIn("kEnabled = false",config)
        self.assertIn("correction disabled by user",config)
        self.assertIn("kLimit = 0.3f",config)
        self.assertGreaterEqual(len(report["sources"]),1)


if __name__=="__main__":unittest.main()
