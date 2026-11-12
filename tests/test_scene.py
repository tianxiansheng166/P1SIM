"""Visibility geometry must agree with camera axes and actual scene profile."""
import math
from pathlib import Path
import unittest
from p1sim_dev.scene import Scene


class ProjectionTest(unittest.TestCase):
    def test_primary_centers_when_camera_faces_it(self):
        scene = Scene(Path(__file__).resolve().parents[1] / 'worlds/autoaim.sdf')
        self.assertEqual(len(scene.targets), 1)
        _, pose, _ = scene.targets[0]
        x, y, z = pose[:3]
        yaw = math.atan2(y, x)
        pitch = -math.atan2(z-scene.camera_position[2], math.hypot(x, y))
        target = scene.project(640, 480, yaw, pitch)[0]
        self.assertAlmostEqual(target['center_px'][0], 320)
        self.assertAlmostEqual(target['center_px'][1], 240)
        self.assertTrue(target['fully_in_fov'])
        self.assertFalse(scene.project(640, 480, yaw+math.pi, pitch)[0]['fully_in_fov'])

    def test_left_yaw_moves_fixed_target_right(self):
        scene = Scene(Path(__file__).resolve().parents[1] / 'worlds/developer.sdf')
        before = scene.project(640, 480, 0, 0)
        after = scene.project(640, 480, .085, -.02)
        self.assertTrue(all(t['fully_in_fov'] for t in after))
        self.assertTrue(all(b['center_px'][0] > a['center_px'][0] for a, b in zip(before, after)))
        self.assertTrue(all(b['center_px'][1] > a['center_px'][1] for a, b in zip(before, after)))


if __name__ == '__main__':
    unittest.main()
