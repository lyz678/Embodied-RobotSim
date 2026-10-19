"""Numerical contract tests independent of ROS/Isaac/GPU."""
import json
import sys
import tempfile
import unittest
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import yaml

DEMO=Path(__file__).resolve().parents[1]/'src/manipulation/handeye_demo'
sys.path.insert(0,str(DEMO))
from core import METHOD, board_pose, detect, errors, fit_depth_grasp, layout, make_assets, solve, transform
from offline import calibrate, evaluate, grasp_history, write_json


class HandeyeGeometry(unittest.TestCase):
    def setUp(self):
        self.K=np.array([[500.,0,320],[0,500.,240],[0,0,1.]])

    def samples(self):
        rng=np.random.default_rng(31)
        X=transform(Rotation.from_rotvec([.2,-.15,.3]).as_matrix(),[.03,-.01,.06])
        board=transform(Rotation.from_rotvec([.1,.2,-.1]).as_matrix(),[.65,.1,.8])
        data=[]
        for i in range(40):
            hand=transform(Rotation.from_rotvec(rng.uniform(-.7,.7,3)).as_matrix(),rng.uniform(-.3,.3,3))
            camera_target=np.linalg.inv(hand@X)@board
            data.append({'sample_id':str(i),'split':'train' if i<30 else 'test',
                         'T_base_hand':hand.tolist(),'T_camera_target':camera_target.tolist(),'reprojection_px':0.})
        return X,board,data

    def test_daniilidis_recovers_camera_to_hand(self):
        X,_,samples=self.samples()
        e=errors(solve(samples[:30]),X)
        self.assertLess(e['translation_mm'],1e-6)
        self.assertLess(e['rotation_deg'],1e-5)

    def test_degenerate_motion_rejected(self):
        _,_,samples=self.samples()
        for sample in samples:
            sample['T_base_hand']=np.eye(4).tolist()
        with self.assertRaisesRegex(ValueError,'degenerate'):
            solve(samples)

    def test_board_joint_pnp_corner_order(self):
        T=transform(Rotation.from_rotvec([.3,-.2,.15]).as_matrix(),[.01,.02,.6])
        detected={}
        for k,xy in layout().items():
            p,_=cv2.projectPoints(np.c_[xy,np.zeros(4)],cv2.Rodrigues(T[:3,:3])[0],T[:3,3],self.K,np.zeros(5))
            detected[k]=p.reshape(4,2)
        estimated,_=board_pose(detected,self.K,np.zeros(5))
        self.assertLess(errors(estimated,T)['translation_mm'],.001)
        with self.assertRaisesRegex(ValueError,'no_target'):
            board_pose({},self.K,np.zeros(5))

    def test_partial_board_and_foreign_ids(self):
        T=transform(np.eye(3),[0,0,.6]); detections={}
        for tag,xy in list(layout().items())[:3]:
            points,_=cv2.projectPoints(np.c_[xy,np.zeros(4)],np.zeros(3),T[:3,3],self.K,np.zeros(5))
            detections[tag]=points.reshape(4,2)
        detections[100]=np.zeros((4,2))
        result,_=board_pose(detections,self.K,np.zeros(5))
        self.assertLess(errors(result,T)['translation_mm'],.001)

    def test_depth_plane_position_uses_depth(self):
        depth=np.full((480,640),.5,np.float32)
        corners=np.array([[280,200],[360,200],[360,280],[280,280]],float)
        goal,m=fit_depth_grasp(depth,corners,self.K)
        self.assertTrue(np.allclose(goal[:3,3],[0,0,.465],atol=1e-5))
        depth[:]=.6
        farther,_=fit_depth_grasp(depth,corners,self.K)
        self.assertAlmostEqual(farther[2,3]-goal[2,3],.1,places=5)
        self.assertGreater(m['valid_fraction'],.99)
        self.assertAlmostEqual(np.linalg.det(goal[:3,:3]),1.)

    def test_holes_and_outliers(self):
        depth=np.full((480,640),.5,np.float32)
        corners=np.array([[280,200],[360,200],[360,280],[280,280]],float)
        depth[215:220,300:330]=np.nan
        depth[230:235,310:320]=1.
        goal,metrics=fit_depth_grasp(depth,corners,self.K)
        self.assertLess(metrics['valid_fraction'],1.)
        self.assertAlmostEqual(goal[2,3],.465,places=4)
        depth[:]=np.nan
        with self.assertRaisesRegex(ValueError,'insufficient_valid_depth'):
            fit_depth_grasp(depth,corners,self.K)
        depth[:]=0
        with self.assertRaises(ValueError):
            fit_depth_grasp(depth,corners,self.K)

    def test_assets_detect_expected_ids(self):
        with tempfile.TemporaryDirectory() as d:
            make_assets(d)
            for kind,number in [('grid',16),('cube',1)]:
                ids=detect(cv2.imread(str(Path(d)/f'{kind}.png')))
                self.assertEqual(len(ids),number)
                if kind=='cube':
                    self.assertIn(100,ids)

    def test_offline_solve_without_truth_then_heldout_scoring(self):
        X,board,samples=self.samples()
        config=yaml.safe_load((DEMO/'config.yaml').read_text())
        with tempfile.TemporaryDirectory() as tmp:
            directory=Path(tmp)
            (directory/'config.yaml').write_text(yaml.safe_dump(config))
            p=directory/'grid'; p.mkdir()
            (p/'samples.json').write_text(json.dumps(samples))
            self.assertTrue(calibrate(directory,config))  # solver must not read truth
            truth=[{'sample_id':s['sample_id'],'T_hand_camera':X.tolist(),'T_base_board':board.tolist()} for s in samples]
            (p/'truth.json').write_text(json.dumps(truth))
            evaluate(directory)
            result=json.loads((directory/'evaluation.json').read_text())
            self.assertEqual(result['method'],METHOD)
            self.assertEqual(result['test']['target_translation_mm']['count'],10)
            self.assertLess(result['test']['target_translation_mm']['rmse'],1e-6)
            self.assertIn('完整物理实验完成：False',(directory/'report.md').read_text())

class DepthRegistration(unittest.TestCase):
    def setUp(self):
        sys.path.insert(0,str(DEMO.parents[1]/'simulation/x_bot_isaac/runtime'))
        from depth_registration import register_depth
        self.register=register_depth
        self.K=np.array([[100.,0,5],[0,100.,5],[0,0,1.]])

    def test_reproject_measured_z_and_preserve_noise(self):
        points=np.array([[0,0,1.],[.01,0,1.],[-.01,0,1.]])
        result=self.register(points,self.K,(10,10),measured_depth=np.array([1.002,1.003,1.004]))
        self.assertAlmostEqual(result[5,5],1.002,places=6)
        self.assertAlmostEqual(result[5,6],1.003,places=6)
        self.assertAlmostEqual(result[5,4],1.004,places=6)
        self.assertEqual(np.isfinite(result).sum(),3)

    def test_z_buffer_and_invalid_values(self):
        points=np.array([[0,0,1.],[0,0,.5],[np.nan,0,1.],[0,0,-1.],[10,0,1.]])
        result=self.register(points,self.K,(10,10))
        self.assertEqual(result[5,5],.5)
        self.assertEqual(np.isfinite(result).sum(),1)
        with self.assertRaisesRegex(ValueError,'dimensions'):
            self.register(points,self.K,(10,10),measured_depth=np.array([1.]))


class ResumeExperiment(unittest.TestCase):
    def test_preserve_history_and_refuse_changed_inputs(self):
        cfg=yaml.safe_load((DEMO/'config.yaml').read_text())
        calibration={'complete':True,'method':METHOD}
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)
            self.assertEqual(grasp_history(p,cfg,calibration),[])
            prior=[{'trial':0,'outcome':'success'}]
            write_json(p/'grasps.json',prior)
            self.assertEqual(grasp_history(p,cfg,calibration,resume=True),prior)
            with self.assertRaisesRegex(RuntimeError,'already exist'):
                grasp_history(p,cfg,calibration)
            changed=json.loads(json.dumps(cfg));changed['grasp']['lift_m']=.2
            with self.assertRaisesRegex(RuntimeError,'identical frozen'):
                grasp_history(p,changed,calibration,resume=True)
            self.assertEqual(json.loads((p/'grasps.json').read_text()),prior)

    def test_duplicate_ids_do_not_count_as_completed_trials(self):
        cfg=yaml.safe_load((DEMO/'config.yaml').read_text());calibration={'complete':True}
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);grasp_history(p,cfg,calibration)
            write_json(p/'grasps.json',[{'trial':0}]*2)
            with self.assertRaisesRegex(RuntimeError,'duplicate'):
                grasp_history(p,cfg,calibration,resume=True)


if __name__=='__main__':
    unittest.main()
