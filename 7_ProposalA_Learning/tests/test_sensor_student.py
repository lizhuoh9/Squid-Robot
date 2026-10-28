import sys
import unittest
from unittest import mock
from pathlib import Path
import numpy as np
import torch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'learning'))
import pipeline as p
from export_espdl import require_activation_approval


class SensorStudentTests(unittest.TestCase):
    def setUp(self):torch.set_num_threads(1)

    def test_bias_attenuation_when_shallow_at_any_speed(self):
        error=torch.tensor([-2.,-1.,0.,1.,-2.,-2.,-2.,-2.])
        speed=torch.tensor([-.5,-.25,-1.,-1.,0.,1.,-1.,-1.])
        bias=torch.tensor([.9,.9,.9,.9,.9,.9,.1,-.2])
        expected=torch.tensor([.7,.8,.9,.9,.7,.7,0.,-.2])
        self.assertTrue(torch.allclose(p.effective_control_bias(error,speed,bias),expected))
        self.assertTrue(torch.equal(bias,torch.tensor([.9,.9,.9,.9,.9,.9,.1,-.2])))

    def test_ascent_guard_boundary_and_normal_return(self):
        error=torch.tensor([-1.,0.,1.,-1.,-1.,-1.])
        speed=torch.tensor([-.31,-1.,-1.,-.3,.5,-1.])
        correction=torch.tensor([-.2,-.5,-.2,-.2,-.2,.2])
        expected=torch.tensor([0.,0.,-.2,0.,0.,.2])
        self.assertTrue(torch.allclose(p.guard_ascent_correction(error,speed,correction),expected))

    def test_ascent_guard_does_not_add_braking_or_predict_crossing(self):
        error=torch.tensor([3.,1.,0.,0.])
        speed=torch.tensor([-1.,-2.,-3.,.5])
        correction=torch.tensor([-.2,-.2,.45,-.2])
        expected=torch.tensor([-.2,-.2,.45,0.])
        self.assertTrue(torch.allclose(p.guard_ascent_correction(error,speed,correction),expected))

    def test_failed_acceptance_requires_explicit_research_override(self):
        with self.assertRaises(RuntimeError):require_activation_approval(True,False)
        with self.assertRaises(ValueError):require_activation_approval(False,False,True)
        require_activation_approval(True,False,True)
        require_activation_approval(True,True)
        require_activation_approval(False,False)

    def test_extended_zero_student_equals_A_and_snapshot_is_causal(self):
        plant=p.SensorHistoryHybrid().eval();student=p.Student(125).eval()
        for param in student.parameters():param.data.zero_()
        features=np.zeros((2,126),np.float32);features[:,3]=1.1;features[:,-1]=40
        plans=np.zeros((1,4),np.float32);stats=(torch.zeros(125),torch.ones(125)*100)
        a,_=p.teacher_rollout([plant],features,plans)
        # Isolate network/history equivalence from the deterministic brake,
        # which is exercised separately by the ascent boundary tests.
        with mock.patch.object(p,'guard_ascent_correction',side_effect=lambda e,v,c:c):
            b,_,detail=p.teacher_rollout([plant],features,plans,student=student,stats=stats,collect_ticks=(20,))
        self.assertTrue(torch.allclose(a,b,atol=1e-6))
        snapshot=detail['snapshots'][0]
        self.assertEqual(tuple(snapshot['features'].shape),(2,126))
        past=snapshot['features'][:,65:125].reshape(2,30,2)
        current=snapshot['features'][:,-1,None]
        self.assertTrue(torch.allclose(past[:,:,0]+current,snapshot['context']['sensor_depths']))
        self.assertTrue(torch.equal(past[:,:,1],snapshot['context']['sensor_speeds']))

    def test_teacher_refines_to_half_limit_labels(self):
        features=np.zeros((2,126),np.float32);features[:,-1]=40
        def rollout(members,features,plans,**kwargs):
            plans=torch.as_tensor(plans)
            if plans.ndim==2:plans=plans[None].expand(len(features),-1,-1)
            cost=(plans-torch.tensor([.25,0,0,0])).square().sum(2)
            return cost,torch.zeros_like(cost)
        with mock.patch.object(p,'teacher_rollout',side_effect=rollout):
            labels,_=p.label_batch([],features,None,correction_limit=.5,objective='range')
        np.testing.assert_allclose(labels,[.25,.25])

    def test_first_control_uses_measurement_not_latent_velocity(self):
        plant=p.SensorHistoryHybrid().eval()
        plant.velocity[-1].bias.data.fill_(1)
        class Recorder(torch.nn.Module):
            def __init__(self):super().__init__();self.first=None
            def forward(self,x):
                if self.first is None:self.first=x.clone()
                return x.new_zeros((len(x),1))
        student=Recorder();features=np.zeros((1,126),np.float32)
        features[:,1]=.2;features[:,2]=.1;features[:,3]=1.1;features[:,-1]=40
        p.teacher_rollout([plant],features,np.zeros((1,4),np.float32),student=student,
                          stats=(torch.zeros(125),torch.ones(125)))
        self.assertAlmostEqual(student.first[0,1].item(),.2,places=6)
        self.assertAlmostEqual(student.first[0,2].item(),.1,places=6)


if __name__=='__main__':unittest.main()
