import os
import sys
import unittest

SRC_DIR = os.path.join(os.path.dirname(__file__), "..", "src")
sys.path.insert(0, os.path.abspath(SRC_DIR))

from mine_slam_calibration.extrinsics import audit_extrinsics, load_extrinsics


class ExtrinsicsAuditTest(unittest.TestCase):
    def test_default_config_passes_audit(self):
        path = os.path.join(os.path.dirname(__file__), "..", "config", "extrinsics.yaml")
        self.assertEqual([], audit_extrinsics(load_extrinsics(path)))

    def test_duplicate_child_is_reported(self):
        data = {
            "calibration": {"reference_frame": "base_link"},
            "frames": {"base_link": {}, "lidar_center": {}, "lidar_left": {}},
            "transforms": [
                {
                    "parent": "base_link",
                    "child": "lidar_center",
                    "translation": [0, 0, 0],
                    "rotation_rpy": [0, 0, 0],
                },
                {
                    "parent": "lidar_left",
                    "child": "lidar_center",
                    "translation": [0, 0, 0],
                    "rotation_rpy": [0, 0, 0],
                },
            ],
        }
        self.assertTrue(any("multiple parents" in issue for issue in audit_extrinsics(data)))

    def test_missing_required_frame_is_reported(self):
        data = {
            "calibration": {"reference_frame": "base_link"},
            "frames": {"base_link": {}, "lidar_center": {}},
            "transforms": [
                {
                    "parent": "base_link",
                    "child": "lidar_center",
                    "translation": [0, 0, 0],
                    "rotation_rpy": [0, 0, 0],
                }
            ],
            "required_frames": ["base_link", "lidar_center", "imu_link"],
        }
        self.assertIn("required frame 'imu_link' is not declared", audit_extrinsics(data))


if __name__ == "__main__":
    unittest.main()
