import sys
import types
import unittest

from model.llm import JudgeLLM
import model.llm as llm_module


class JudgeRuleTest(unittest.TestCase):
    def setUp(self):
        self.judge = JudgeLLM.__new__(JudgeLLM)
        self.original_get_obj_xy = llm_module.get_obj_xy
        self.original_get_obj_size = llm_module.get_obj_size
        self.original_ros_bridge = sys.modules.get("ros_bridge")

    def tearDown(self):
        llm_module.get_obj_xy = self.original_get_obj_xy
        llm_module.get_obj_size = self.original_get_obj_size
        if self.original_ros_bridge is None:
            sys.modules.pop("ros_bridge", None)
        else:
            sys.modules["ros_bridge"] = self.original_ros_bridge

    def test_move_to_xy_uses_meter_threshold(self):
        result = self.judge._rule_judge(
            {"action_id": 1, "target": (1.0, 2.0)},
            {"robot_x": 6.0, "robot_y": 2.0, "holding": False},
        )
        self.assertFalse(result["pass"])
        self.assertEqual(result["failure_code"], "localization_error")

    def test_pick_up_xy_no_undefined_passed_variable(self):
        result = self.judge._rule_judge(
            {"action_id": 3, "target": (1.0, 2.0)},
            {"robot_x": 1.02, "robot_y": 2.01, "holding": False},
        )
        self.assertFalse(result["pass"])
        self.assertEqual(result["failure_code"], "grasp_failure")

    def test_pick_up_xy_passes_when_close_and_holding(self):
        result = self.judge._rule_judge(
            {"action_id": 3, "target": (1.0, 2.0)},
            {"robot_x": 1.02, "robot_y": 2.01, "holding": True},
        )
        self.assertTrue(result["pass"])

    def test_move_to_obj_by_offset_uses_approach_position(self):
        fake_ros_bridge = types.SimpleNamespace(
            get_obj_approach_pos=lambda obj: (1.6, -0.3, 0.0)
        )
        sys.modules["ros_bridge"] = fake_ros_bridge
        llm_module.get_obj_xy = lambda obj: (2.0, -0.3)
        llm_module.get_obj_size = lambda obj: (0.05, 0.05, 0.05)

        result = self.judge._rule_judge(
            {
                "action_id": 2,
                "target": "red_block",
                "offset": (0.0, 0.0),
            },
            {"robot_x": 1.62, "robot_y": -0.31, "holding": False},
        )
        self.assertTrue(result["pass"])
        self.assertEqual(result["expected_base_kind"], "approach")


if __name__ == "__main__":
    unittest.main()
