# SPDX-License-Identifier: Apache-2.0

"""Test the model predictive control balancer."""

import unittest

import numpy as np

from upkie.controllers import MPCBalancer


class MPCBalancerTestCase(unittest.TestCase):
    def setUp(self):
        self.balancer = MPCBalancer(leg_length=0.58)
        self.spine_observation = {
            "base_orientation": {
                "pitch": 0.05,
                "angular_velocity": [0.0, 0.1, 0.0],
            },
            "floor_contact": {"contact": True},
            "wheel_odometry": {"position": 0.0, "velocity": 0.0},
        }

    def test_references_defined_at_construction(self):
        """Check the references the QP cost is built from at construction.

        The balancer builds its QP in the constructor, so the goal state and
        reference trajectory the cost vector reads from must be defined by
        then. Leaving them undefined raises a ``ProblemDefinitionError`` from
        qpmpc 3.2.0 onwards.
        """
        mpc_problem = self.balancer.mpc_problem
        self.assertIsNotNone(mpc_problem.goal_state)
        self.assertIsNotNone(mpc_problem.target_states)
        self.assertTrue(mpc_problem.has_terminal_cost)
        self.assertTrue(mpc_problem.has_stage_state_cost)

    def test_step_commands_finite_velocity(self):
        commanded_velocity = self.balancer.step(
            target_ground_velocity=0.2,
            spine_observation=self.spine_observation,
            dt=0.01,
        )
        self.assertTrue(np.isfinite(commanded_velocity))
        self.assertLess(
            abs(commanded_velocity), self.balancer.max_ground_velocity + 1e-10
        )


if __name__ == "__main__":
    unittest.main()
