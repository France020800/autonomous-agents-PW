import unittest

import numpy as np

from mas_formation.controllers import FormationController, FormationThenGoalController, GoalNavigationController
from mas_formation.models import TriangularFormation


class TestNavigation(unittest.TestCase):
    def setUp(self) -> None:
        self.formation = TriangularFormation.equilateral((0, 1, 2), side_length=2.0)
        self.target_positions = {
            0: (0.0, 0.0),
            1: (2.0, 0.0),
            2: (1.0, np.sqrt(3)),
        }

    def test_centroid_goal_assigns_the_same_velocity_to_all_agents(self) -> None:
        controller = GoalNavigationController((0, 1, 2), goal=(4.0, 3.0), gain=0.5)
        controls = controller.controls(self.target_positions)

        np.testing.assert_allclose(controls[0], controls[1])
        np.testing.assert_allclose(controls[1], controls[2])
        self.assertGreater(np.linalg.norm(controls[0]), 0.0)
        self.assertGreater(controller.potential(self.target_positions), 0.0)

    def test_navigation_is_inactive_until_formation_is_reached(self) -> None:
        controller = FormationThenGoalController(
            formation_controller=FormationController(self.formation),
            goal_distance=8.0,
            formation_tolerance=1e-6,
            seed=7,
        )
        distorted = dict(self.target_positions)
        distorted[1] = (2.5, 0.0)

        controls = controller.controls(distorted)

        self.assertFalse(controller.navigation_active)
        np.testing.assert_allclose(controls[0], FormationController(self.formation).controls(distorted)[0])

    def test_goal_is_sampled_at_fixed_distance_when_formation_is_reached(self) -> None:
        controller = FormationThenGoalController(
            formation_controller=FormationController(self.formation),
            goal_distance=8.0,
            formation_tolerance=1e-6,
            seed=7,
        )
        centroid = np.mean(list(self.target_positions.values()), axis=0)

        controller.controls(self.target_positions)

        self.assertTrue(controller.navigation_active)
        self.assertIsNotNone(controller.goal)
        self.assertAlmostEqual(np.linalg.norm(controller.goal - centroid), 8.0)


if __name__ == "__main__":
    unittest.main()
