"""Prediction-only model: causal history, gradients and horizon objective."""
import sys
import unittest
from pathlib import Path

import torch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'learning'))
import pipeline as p
from fit_dynamics_only import prediction_loss,hold_acceptance,hold_masks,run_separated_data
import numpy as np
from unittest import mock


class DynamicsPredictionTests(unittest.TestCase):
    def test_prediction_shape_and_gradients(self):
        model=p.HistoryHybrid()
        x=torch.zeros(2,p.DYNAMICS_DIM);u=torch.zeros(2,30)
        pred=model(x,u,u)
        self.assertEqual(tuple(pred.shape),(2,30,2))
        loss=prediction_loss(pred,torch.zeros_like(pred))
        loss.backward()
        self.assertTrue(torch.isfinite(pred).all())
        self.assertTrue(all(param.grad is not None for param in model.parameters()))

    def test_initial_state_cannot_see_future_commands(self):
        model=p.HistoryHybrid().eval();x=torch.zeros(1,p.DYNAMICS_DIM)
        baseline=model(x,torch.zeros(1,30));changed=torch.zeros(1,30);changed[:,15:]=1
        switched=model(x,changed)
        self.assertTrue(torch.equal(baseline[:,:15],switched[:,:15]))

    def test_endpoint_depth_error_costs_more_than_nonendpoint(self):
        truth=torch.zeros(1,30,2)
        end=truth.clone();other=truth.clone()
        end[:,29,0]=1;other[:,28,0]=1
        self.assertGreater(prediction_loss(end,truth).item(),prediction_loss(other,truth).item())

    def test_legacy_checkpoint_still_loads(self):
        old=p.Hybrid();loaded=p.Hybrid();loaded.load_state_dict(old.state_dict())
        x=torch.zeros(1,p.DYNAMICS_DIM);u=torch.zeros(1,30)
        self.assertTrue(torch.equal(old(x,u),loaded(x,u)))

    def test_hold_gate_rejects_low_speed_regression_despite_global_gain(self):
        data={'x':torch.zeros(40,p.DYNAMICS_DIM),'policy':np.zeros((40,len(p.FEATURES))),
              'y':torch.zeros(40,30,2)}
        data['x'][20:,1]=2
        ids=np.arange(40);old=torch.ones(40,30,2)
        bad=old.clone();bad[:20,:,0]=1.1;bad[20:,:,0]=0
        self.assertFalse(hold_acceptance(bad,old,data,ids)['passed'])
        good=old.clone();good[:,:,0]=.9
        self.assertTrue(hold_acceptance(good,old,data,ids)['passed'])
        slow,close=hold_masks(data,ids)
        self.assertEqual(int(slow.sum()),20);self.assertEqual(int(close.sum()),20)

    def test_run_split_keeps_each_source_in_one_role(self):
        records={'x':torch.zeros(2,p.DYNAMICS_DIM),'u':torch.zeros(2,30),'y':torch.zeros(2,30,2),
                 'forward_u':torch.zeros(2,30),'policy':np.zeros((2,len(p.FEATURES))),
                 'times':np.array([10.,11.]),'provenance':[{'path':'mock'}]}
        with mock.patch('fit_dynamics_only.digest',side_effect=lambda path:str(path)),mock.patch('fit_dynamics_only.read_data',return_value=records):
            data=run_separated_data(['train.csv'],'validation.csv','test.csv')
        np.testing.assert_array_equal(data['split'],[0,0,1,1,2,2])
        np.testing.assert_array_equal(data['session'],[0,0,1,1,2,2])
        self.assertEqual([r['role'] for r in data['provenance']],['train','validation','test'])

    def test_run_split_rejects_reusing_validation_for_training(self):
        with self.assertRaises(ValueError):run_separated_data(['same.csv'],'same.csv','test.csv')

    def test_ambiguous_duplicate_timestamp_windows_are_excluded(self):
        path=p.ROOT/'data/auto-control-logs/control-auto-1792647658545.csv'
        with self.assertRaises(ValueError):p.read_data([path])
        data=p.read_data([path],exclude_duplicate_timestamps=True)
        self.assertEqual(data['provenance'][0]['excluded_duplicate_timestamp_count'],1)
        self.assertTrue(np.all(np.abs(data['times']-296.288)>2.9))
        self.assertTrue(torch.isfinite(data['y']).all())

    def test_sensor_encoder_preserves_previous_model_before_training(self):
        previous=p.HistoryHybrid().eval();candidate=p.SensorHistoryHybrid().eval()
        candidate.load_state_dict(previous.state_dict(),strict=False)
        x=torch.randn(2,p.DYNAMICS_DIM)*.1;u=torch.zeros(2,30);history=torch.randn(2,30,2)
        self.assertTrue(torch.equal(previous(x,u),candidate(x,u,sensor_history=history)))
        prediction=candidate(x,u,sensor_history=history)
        prediction.square().mean().backward()
        self.assertTrue(all(param.grad is not None for param in candidate.parameters()))
        with self.assertRaises(ValueError):candidate(x,u)

    def test_future_measurements_do_not_enter_sensor_history(self):
        path=p.ROOT/'data/auto-control-logs/control-1792041251.csv'
        with path.open(encoding='utf-8-sig') as handle:rows=list(p.csv.DictReader(handle))
        switch=float(rows[len(rows)//2]['elapsed_s'])
        changed=[dict(row) for row in rows]
        for row in changed:
            if float(row['elapsed_s'])>=switch and row['depth_cm'].strip():
                row['depth_cm']=str(float(row['depth_cm'])+10)
        with mock.patch.object(p.csv,'DictReader',return_value=rows):before=p.read_data([path],include_state_history=True)
        with mock.patch.object(p.csv,'DictReader',return_value=changed):after=p.read_data([path],include_state_history=True)
        np.testing.assert_array_equal(before['times'],after['times'])
        mask=before['times']<switch
        self.assertTrue(torch.equal(before['state_history'][mask],after['state_history'][mask]))
        self.assertTrue(torch.isfinite(before['state_history']).all())


if __name__=='__main__':unittest.main()
