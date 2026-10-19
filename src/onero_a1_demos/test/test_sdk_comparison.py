"""Offline checks only: no ROS graph, CAN transport, or real Arm construction."""

from copy import deepcopy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from onero_a1_bringup.can_ownership import ChannelOwnership
from onero_a1_demos.sdk_comparison import FlangeFK, check_start, metrics, validate_bundle
from onero_a1_demos.sdk_compare import ObservedBus
from onero_a1_demos.trial_records import fingerprint


class ComparisonTest(unittest.TestCase):
    def setUp(self):
        path = Path(__file__).resolve().parents[3] / 'src/onero_a1_description/urdf/onero_a1_body.urdf'
        urdf = path.read_text()
        q = [.5,.04,-.08,1.15,.03,.06,.07]
        fk = FlangeFK(urdf)
        mat = fk.matrix(q)
        pose = {'position':dict(zip('xyz',mat[:3,3])),
                'orientation':dict(zip('xyzw',Rotation.from_matrix(mat[:3,:3]).as_quat()))}
        self.bundle = {'schema_version':1,'kind':'onero_sdk_comparison','phase':'hold',
                       'settings':{'simulate':False,'model':'A1R','kp':120.,'kd':1.,'control_rate':100.},
                       'collision_checked':True,'initial_feedback':q,'urdf':urdf,
                       'frame_from_base':np.eye(4).tolist(),'pose_target':pose,
                       'joint_bounds':[[-3.14,3.14]]*7,
                       'samples':[{'time_s':i*.01,'positions':q[:],'velocities':[0.0]*7,
                                   'feedback_position':q[:]} for i in range(10)]}
        self.sign(self.bundle)

    @staticmethod
    def sign(bundle):
        bundle.pop('bundle_sha256',None)
        bundle['bundle_sha256']=fingerprint(bundle)

    def test_integrity(self):
        validate_bundle(self.bundle)
        changed=deepcopy(self.bundle)
        changed['settings']['kp']=50
        with self.assertRaises(ValueError):validate_bundle(changed)

    def test_invalid_sequences(self):
        for change in ['nan','time','bounds','velocity','scene']:
            b=deepcopy(self.bundle)
            if change=='nan':b['samples'][0]['positions'][0]=float('nan')
            elif change=='time':b['samples'][1]['time_s']=0
            elif change=='bounds':b['samples'][0]['positions'][0]=10
            elif change=='velocity':b['samples'][-1]['velocities'][0]=.1
            else:b['collision_checked']=False
            if change=='nan':
                with self.assertRaises(ValueError):self.sign(b)
            else:
                self.sign(b)
                with self.assertRaises(ValueError):validate_bundle(b)

    def test_start_guard_and_fk(self):
        q=self.bundle['initial_feedback']
        self.assertLess(check_start(self.bundle,q)['flange_difference_m'],1e-12)
        moved=q[:];moved[0]+=.02
        with self.assertRaises(ValueError):check_start(self.bundle,moved)
        error=FlangeFK(self.bundle['urdf']).errors(q,self.bundle['pose_target'])
        self.assertLess(error['position_m'],1e-12)
        self.assertLess(error['orientation_rad'],1e-12)

    def test_metrics(self):
        result=metrics(self.bundle,self.bundle['samples'])
        self.assertLess(result['last_3s_fk_mean_m'],1e-12)
        self.assertEqual(result['final_joint_error_rad'],[0.0]*7)

    def test_channel_ownership(self):
        with tempfile.TemporaryDirectory() as directory:
            a=ChannelOwnership('offline-test',directory=directory)
            b=ChannelOwnership('offline-test',directory=directory)
            try:
                a.acquire()
                with self.assertRaises(RuntimeError):b.acquire()
                a.close();b.acquire()
            finally:a.close();b.close()

    def test_bus_observer_does_not_rewrite_frames(self):
        from onero_sdk.model_registry import get_model
        from onero_sdk.protocol import MotorLimits,encode_mit
        class FakeBus:
            channel='offline-test'
            def __init__(self):self.frames=[]
            def send(self,can_id,data):self.frames.append((can_id,data))
            def close(self):pass
        raw=FakeBus()
        observed=ObservedBus(raw,get_model('A1R'))
        payload=encode_mit(.5,.03,120,1,3,MotorLimits())
        observed.send(1,payload)
        self.assertEqual(raw.frames,[(1,payload)])
        self.assertEqual(observed.sent[0]['data_hex'],payload.hex())
        with self.assertRaises(RuntimeError):observed.require_feedback()
        observed.feedback={i:SimpleNamespace(error=1) for i in range(7)}
        observed.require_feedback()
        observed.feedback[3].error=0
        with self.assertRaises(RuntimeError):observed.require_feedback()

    def test_post_record_hold_tolerates_missing_but_not_faults(self):
        # No transport needed: only check the feedback policy boundary.
        observed=object.__new__(ObservedBus)
        observed.feedback={i:SimpleNamespace(error=1) for i in range(7) if i!=3}
        with self.assertRaises(RuntimeError):observed.require_feedback()
        self.assertEqual(observed.require_feedback(allow_missing=True),[4])
        observed.feedback[3]=SimpleNamespace(error=1)
        self.assertEqual(observed.require_feedback(allow_missing=True),[])
        observed.feedback[3].error=0xC
        with self.assertRaises(RuntimeError):observed.require_feedback(allow_missing=True)


if __name__=='__main__':unittest.main()
