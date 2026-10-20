"""Key offline checks for controller tolerance overrides and timeout margins."""
from copy import deepcopy
from pathlib import Path
import unittest

import yaml

from onero_a1_bringup.controller_tolerances import (
    TOLERANCE_ARGUMENTS, execution_goal_margin, resolve_controller_config,
)


class ControllerToleranceChecks(unittest.TestCase):
    def setUp(self):
        path = Path(__file__).resolve().parents[1]/'config/ros2_controllers.yaml'
        self.config = yaml.safe_load(path.read_text())

    def test_omitted_overrides_preserve_per_joint_yaml(self):
        self.config['arm_controller']['ros__parameters']['constraints']['joint4-a1_r']['goal'] = .07
        result = resolve_controller_config(self.config, {name: '' for name in TOLERANCE_ARGUMENTS})
        self.assertEqual(result, self.config)

    def test_uniform_overrides_apply_to_all_seven_joints(self):
        original = deepcopy(self.config)
        result = resolve_controller_config(self.config, {
            'arm_trajectory_tolerance': '.3', 'arm_goal_tolerance': '.08',
            'arm_goal_time_tolerance': '2.5', 'arm_stopped_velocity_tolerance': '.04',
        })
        parameters = result['arm_controller']['ros__parameters']
        constraints = parameters['constraints']
        self.assertEqual(len(parameters['joints']), 7)
        for name in parameters['joints']:
            self.assertEqual(constraints[name], {'trajectory': .3, 'goal': .08})
        self.assertEqual(constraints['goal_time'], 2.5)
        self.assertEqual(constraints['stopped_velocity_tolerance'], .04)
        self.assertEqual(self.config, original)

    def test_invalid_overrides_fail_before_startup(self):
        for name in TOLERANCE_ARGUMENTS:
            for value in ('0', '-1', 'nan', 'inf', 'bad'):
                with self.subTest(name=name, value=value), self.assertRaises(ValueError):
                    resolve_controller_config(self.config, {name: value})

    def test_moveit_margin_allows_controller_goal_wait(self):
        self.assertEqual(execution_goal_margin(self.config), 2.)
        result = resolve_controller_config(self.config, {'arm_goal_time_tolerance': '45'})
        self.assertEqual(execution_goal_margin(result), 46.)
        self.assertEqual(execution_goal_margin(result, minimum=50.), 50.)


if __name__ == '__main__':
    unittest.main()
