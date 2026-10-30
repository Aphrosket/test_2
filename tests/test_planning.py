import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np

from omni_project.assets import make_world
from omni_project.planning import body_velocity, clearance, load_world, path_safe, plan, segment_safe


ROOT=Path(__file__).resolve().parents[1]


class PlanningTests(unittest.TestCase):
    def setUp(self): self.world=load_world(ROOT/'config/world.json')

    def test_plan_endpoints_full_body_clearance_and_curve(self):
        raw,smooth=plan(self.world)
        np.testing.assert_allclose(smooth[0],self.world['start'])
        np.testing.assert_allclose(smooth[-1],self.world['goal'])
        self.assertTrue(path_safe(self.world,smooth,self.world['robot_radius']+self.world['planning_margin']))
        # 直接穿过障碍的路径不得被规划为直线。
        self.assertFalse(segment_safe(self.world,self.world['start'],self.world['goal'],0.41))
        vectors=np.diff(smooth,axis=0)
        cross=vectors[:-1,0]*vectors[1:,1]-vectors[:-1,1]*vectors[1:,0]
        self.assertTrue(np.any(np.abs(cross)>1e-8))

    def test_invalid_goal_and_thin_obstacle(self):
        with self.assertRaises(ValueError): plan(self.world,goal=[-0.7,-0.5])
        thin={'bounds':[-3,-3,3,3],'obstacles':[{'kind':'box','x':0,'y':0,'sx':0.005,'sy':2.}]}
        self.assertFalse(segment_safe(thin,[-1,0],[1,0],0.01))

    def test_velocity_frames(self):
        x,y=body_velocity([1,0],math.pi/2)
        self.assertAlmostEqual(x,0,places=7); self.assertAlmostEqual(y,-1,places=7)

    def test_sdf_two_obstacle_types_and_no_external_models(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'tests') as folder:
            path=Path(folder)/'arena.sdf'; make_world(self.world,path)
            root=ET.parse(path).getroot()
            self.assertIsNotNone(root.find('.//geometry/cylinder'))
            self.assertIsNotNone(root.find('.//geometry/box'))
            self.assertIsNone(root.find('.//include'))
            self.assertEqual(root.find('.//sensor').get('type'),'gpu_lidar')
            self.assertEqual(root.find(".//plugin[@name='ignition::gazebo::systems::OdometryPublisher']/dimensions").text,'3')
            self.assertIsNotNone(root.find(".//plugin[@name='ignition::gazebo::systems::VelocityControl']"))


if __name__=='__main__': unittest.main()
