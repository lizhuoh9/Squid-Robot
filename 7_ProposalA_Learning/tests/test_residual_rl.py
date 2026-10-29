import sys
import unittest
from pathlib import Path
import torch
import numpy as np
import shutil
import subprocess
import tempfile
import hashlib
import json
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'learning'))
from residual_rl import Actor, Env, combine, LIMIT, shallow_stage_cost, acceptance, validation_rank, relative_peak_score
import pipeline as p
from export_residual_rl import require_current_actor


class ResidualRLTests(unittest.TestCase):
    def test_failed_candidate_cannot_reexport_old_actor(self):
        with self.assertRaises(RuntimeError):require_current_actor({'export_allowed':False,'policy_artifact':'candidate.pt','selected_update':40},{'selected_update':20})
        with self.assertRaises(ValueError):require_current_actor({'export_allowed':True,'policy_artifact':'actor.pt','selected_update':40},{'selected_update':20})
        require_current_actor({'export_allowed':True,'policy_artifact':'actor.pt','selected_update':40},{'selected_update':40})

    def test_peak_tradeoff_is_not_hidden_by_mean_error(self):
        import copy
        keys=('mae_cm','p95_abs_cm','shallow_mean_cm','deep_mean_cm','shallow_max_cm','deep_max_cm','direction_switches_per_4s')
        ref={'all':dict.fromkeys(keys,1.),'hold':dict.fromkeys(keys,1.)};ref['all']['reward']=-10.
        smaller=copy.deepcopy(ref);larger=copy.deepcopy(ref)
        smaller['hold']['shallow_max_cm']=.9;smaller['hold']['deep_max_cm']=.9
        smaller['hold']['mae_cm']=2.
        larger['hold']['mae_cm']=.01
        self.assertGreater(validation_rank(smaller,ref),validation_rank(larger,ref))
        good=relative_peak_score(torch.tensor([2.7]),torch.tensor([4.5]),torch.tensor([3.]),torch.tensor([5.]))
        bad=relative_peak_score(torch.tensor([3.3]),torch.tensor([4.]),torch.tensor([3.]),torch.tensor([5.]))
        self.assertLess(good.item(),0);self.assertGreater(bad.item(),0)

    def test_exported_actor_and_baseline_are_paired(self):
        root=Path(__file__).resolve().parents[1];out=root/'artifacts/residual_rl_v1'
        if not (out/'espdl_validation.json').exists():self.skipTest('RL export not generated')
        report=json.loads((out/'espdl_validation.json').read_text())
        self.assertEqual(report['input_dim'],128)
        self.assertEqual(report['rl_limit_cm_s2'],LIMIT)
        self.assertFalse(report['combined_correction_clipped'])
        self.assertEqual(report['actor_sha256'],hashlib.sha256((out/'actor.pt').read_bytes()).hexdigest())
        self.assertEqual(report['espdl_sha256'],hashlib.sha256((out/'residual_rl.espdl').read_bytes()).hexdigest())
        self.assertGreaterEqual(report['input_scale']*127,6)
        for name in ('validation_quantization','test_quantization'):
            self.assertLessEqual(report[name]['mae_cm_s2'],.04)
        ck=torch.load(out/'actor.pt',weights_only=False,map_location='cpu')
        for path,digest in ck['frozen_sha256'].items():self.assertEqual(hashlib.sha256(Path(path).read_bytes()).hexdigest(),digest)

    def test_cpp_observation_order_and_unclipped_sum(self):
        compiler=shutil.which('g++')
        if compiler is None:self.skipTest('Host g++ unavailable; firmware build still required')
        source='''#include "ResidualRLFeatures.h"
#include <iostream>
#include <limits>
int main() {
 float x[125]={}, o[128]; x[0]=-2; x[124]=3;
 if(!residual_rl::fillObservation(x,.25f,-.4f,50,o)) return 1;
 std::cout<<o[0]<<" "<<o[124]<<" "<<o[125]<<" "<<o[126]<<" "<<o[127]<<" "<<residual_rl::combine(.5f,.8f)<<" "<<residual_rl::combine(-.5f,-.8f);
 x[1]=std::numeric_limits<float>::quiet_NaN();
 if(residual_rl::fillObservation(x,0,0,40,o)) return 2;
}'''
        with tempfile.TemporaryDirectory() as tmp:
            src=Path(tmp)/'rl.cpp';binary=Path(tmp)/'rl.exe';src.write_text(source)
            subprocess.run([compiler,'-std=c++11','-Wall','-Wextra','-Werror','-I',str(Path(__file__).resolve().parents[1]/'firmware/main'),str(src),'-o',str(binary)],check=True,capture_output=True)
            result=subprocess.run([str(binary)],check=True,capture_output=True,text=True)
        np.testing.assert_allclose(np.fromstring(result.stdout,sep=' '),[-2,3,.5,-.5,1.25,1.3,-1.3],atol=1e-6)

    def test_two_sided_cost_and_settled_reward(self):
        e=torch.tensor([-1.,1.,0.,0.,3.]);v=torch.tensor([0.,0.,0.,2.,0.])
        cost=shallow_stage_cost(e,v)
        self.assertAlmostEqual(cost[0].item(),cost[1].item())
        self.assertLess(cost[2],cost[0]);self.assertLess(cost[2],cost[3])
        self.assertLess(cost[0],cost[4])

    def test_lower_reward_cost_cannot_hide_worse_shallow_peak(self):
        keys=('mae_cm','p95_abs_cm','shallow_mean_cm','shallow_max_cm','deep_max_cm','direction_switches_per_4s')
        ref={'all':dict.fromkeys(keys,1.),'hold':dict.fromkeys(keys,1.)}
        ref['all']['reward']=-10.
        import copy
        ref['all']['deep_mean_cm']=1.;ref['hold']['deep_mean_cm']=1.
        passing=copy.deepcopy(ref);passing['all']['shallow_mean_cm']=.9;passing['hold']['shallow_max_cm']=.9;passing['hold']['deep_max_cm']=.9
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

    def test_feedback_initialization_has_correct_directions(self):
        actor=Actor();mean=torch.zeros(125);std=torch.ones(125)
        mean[0]=-1.5;std[0]=5.;mean[1]=.2;std[1]=2.
        actor.initialize_feedback(mean,std)
        raw=torch.tensor([[-2.,-1.],[2.,1.],[0.,0.]])
        obs=torch.zeros(3,128);obs[:,:2]=(raw-mean[:2])/std[:2]
        expected=LIMIT*torch.tanh((-.05*raw[:,0]-.4*raw[:,1])/LIMIT)
        self.assertTrue(torch.allclose(actor(obs),expected,atol=1e-6))

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
