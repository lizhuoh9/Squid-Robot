"""Run the actual portable C++ feature builder and check the Python schema."""
import hashlib
import json
import re
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"learning"))
import pipeline as p


class StudentFeatureTests(unittest.TestCase):
    def run_cpp_features(self,config_dir):
        compiler=shutil.which("g++")
        if compiler is None:self.skipTest("Host g++ unavailable; embedded build is still required")
        with tempfile.TemporaryDirectory(prefix="proposal_features_") as scratch:
            binary=Path(scratch)/"features_test.exe"
            arguments=["-std=c++11","-Wall","-Wextra","-Werror",
                       "-I",str(ROOT/"firmware/main"),"-I",str(config_dir),
                       str(ROOT/"tests/student_features_test.cpp"),"-o",str(binary)]
            subprocess.run([compiler,*arguments],check=True,capture_output=True,text=True)
            result=subprocess.run([str(binary)],check=True,capture_output=True,text=True)
        return np.fromstring(result.stdout,sep=" ")

    def test_cpp_history_and_training_feature_order(self):
        actual=self.run_cpp_features(ROOT/'artifacts/hybrid_mpc_v1')
        expected={"depth_error_cm":-2,"speed_cm_s":.5,"accel_cm_s2":-.1,"proposal_bias_cm_s2":1.1,
                  "forward_active":1}
        expected.update({f"applied_command_{i:02d}":1 for i in range(30)})
        expected.update({f"forward_history_{i:02d}":int(i==29) for i in range(30)})
        expected.update({name:0 for name in p.SENSOR_FEATURES[len(p.FEATURES):]})
        names=p.SENSOR_FEATURES if len(actual)==125 else p.FEATURES
        np.testing.assert_allclose(actual,[expected[name] for name in names],atol=1e-6)

    def test_cpp_sensor125_history_even_before_export(self):
        actual=self.run_cpp_features(ROOT/'tests/fixtures/sensor125')
        self.assertEqual(len(actual),125)
        np.testing.assert_allclose(actual[65:],np.zeros(60),atol=1e-6)

    def test_export_and_runtime_metadata_are_paired(self):
        out=ROOT/"artifacts/hybrid_mpc_v1"
        report=json.loads((out/"espdl_validation.json").read_text())
        header=(out/"ProposalStudentConfig.h").read_text()
        names=p.SENSOR_FEATURES if report['schema_version']==p.SENSOR_STUDENT_SCHEMA else p.FEATURES
        self.assertEqual(report["input_dim"],len(names))
        self.assertEqual(report["feature_names"],names)
        self.assertIn(report["schema_version"],(p.SCHEMA_VERSION,p.SENSOR_STUDENT_SCHEMA))
        self.assertGreaterEqual(report["input_scale"]*127,6)
        self.assertGreaterEqual(report["output_scale"]*127,report["limit_cm_s2"])
        self.assertGreaterEqual(report["output_scale"]*128,report["limit_cm_s2"])
        runtime_limit=float(re.search(r"kLimit\s*=\s*([0-9.eE+-]+)f",header).group(1))
        self.assertAlmostEqual(runtime_limit,report["limit_cm_s2"])
        self.assertAlmostEqual(float(re.search(r"kOutputScale\s*=\s*([0-9.eE+-]+)f",header).group(1)),report["output_scale"])
        digest=hashlib.sha256((out/"student.pt").read_bytes()).hexdigest()
        self.assertEqual(report["student_checkpoint_sha256"],digest)
        self.assertIn(digest,header)
        if "kEnabled = true" in header:
            local=json.loads((out/"integration_report.json").read_text(encoding="utf-8"))
            self.assertTrue(local["experimental_user_override"])
            self.assertTrue(local["correction_enabled"])
            self.assertEqual(local["student_checkpoint_sha256"],digest)
            self.assertEqual(local["espdl_sha256"],report["espdl_sha256"])
            self.assertAlmostEqual(local["limit_cm_s2"],runtime_limit)
        else:
            self.assertIn("kEnabled = false",header)
        self.assertEqual(report["espdl_sha256"],hashlib.sha256((out/"proposal_student.espdl").read_bytes()).hexdigest())
        for split in ("validation","test"):
            self.assertLessEqual(report[split]["quantized_float_mae_cm_s2"],.03)
            if "quantized_output_min_cm_s2" in report[split]:
                self.assertGreaterEqual(report[split]["quantized_output_min_cm_s2"],-runtime_limit)
                self.assertLessEqual(report[split]["quantized_output_max_cm_s2"],runtime_limit)
        training=json.loads((out/"training_report.json").read_text())
        self.assertEqual(report["heldout_quantized_model_rollout"]["evaluation_window_sha256"],
                         training["student"]["heldout_model_rollout"]["evaluation_window_sha256"])


if __name__=="__main__":unittest.main()
