import sys
import unittest
from pathlib import Path
import torch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'learning'))
from residual_rl import Actor, Env, combine, LIMIT, shallow_stage_cost, acceptance, validation_rank
import pipeline as p


class ResidualRLTests(unittest.TestCase):
    def test_shallow_error_and_approaching_ascent_are_penalized(self):
        e=torch.tensor([-1.,1.,1.]);v=torch.tensor([0.,0.,-2.])
        cost=shallow_stage_cost(e,v)
        self.assertGreater(cost[0],cost[1])
        self.assertGreater(cost[2],cost[1])

    def test_lower_reward_cost_cannot_hide_worse_shallow_peak(self):
        keys=('mae_cm','p95_abs_cm','shallow_mean_cm','shallow_max_cm','deep_max_cm','direction_switches_per_4s')
        ref={'all':dict.fromkeys(keys,1.),'hold':dict.fromkeys(keys,1.)}
        ref['all']['reward']=-10.
        import copy
        passing=copy.deepcopy(ref);passing['all']['shallow_mean_cm']=.9;passing['hold']['shallow_max_cm']=.9
        bad=copy.deepcopy(passing);bad['hold']['shallow_max_cm']=1.1;bad['all']['reward']=100.
        self.assertTrue(all(acceptance(passing,ref).values()))
        self.assertFalse(all(acceptance(bad,ref).values()))
        self.assertGreater(validation_rank(passing,ref),validation_rank(bad,ref))

    def test_separate_bounds_without_sum_clipping(self):
        x=combine(torch.tensor([.5,-.5,.2]),torch.tensor([.8,-.8,.9]))
        self.assertTrue(torch.allclose(x,torch.tensor([1.3,-1.3,1.0])))

    def test_actor_zero_initialization_and_bound(self):
        actor=Actor();obs=torch.randn(7,128)
        self.assertTrue(torch.equal(actor(obs),torch.zeros(7)))
        actor.net[-1].bias.data.fill_(100)
        self.assertLessEqual(actor(obs).abs().max().item(),LIMIT+1e-7)

    def test_zero_rl_matches_existing_closed_loop_final_error(self):
        torch.set_num_threads(1)
        plant=p.SensorHistoryHybrid().eval();student=p.Student(125).eval()
        for parameter in student.parameters():parameter.data.zero_()
        policy=np.zeros((1,125),np.float32);policy[:,3]=1.1
        data={'policy':policy,'x':torch.zeros(1,p.DYNAMICS_DIM),'session':np.array([0]),'times':np.array([0.])}
        stats=(torch.zeros(125),torch.ones(125)*100)
        with torch.no_grad():
            env=Env(data,plant,student,stats,np.array([0]))
            for _ in range(40):env.observation();env.step(torch.zeros(1))
            _,_,detail=p.teacher_rollout([plant],np.c_[policy,np.array([40.])],np.zeros((1,4),np.float32),student=student,stats=stats,phase_ids=p.phase_for_rows(data,np.array([0])),correction_limit=.5,return_details=True)
        self.assertAlmostEqual((env.state[:,0]-env.target).abs().item(),detail['final_abs_error_cm'].item(),places=4)


if __name__=='__main__':unittest.main()
