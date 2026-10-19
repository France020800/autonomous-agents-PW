import unittest

import numpy as np

from mas_formation.controllers import PositionBasedObstacleAvoidanceController, StaticObstacleAvoidanceController
from mas_formation.models import DynamicObstacle


class TestStaticObstacleAvoidance(unittest.TestCase):
    def setUp(self) -> None:
        self.obstacle = DynamicObstacle(identifier=0, position=(0.0, 0.0), radius=0.5)
        self.controller = StaticObstacleAvoidanceController(
            agent_radii={0: 0.15}, obstacles=[self.obstacle], gain=0.08, influence_distance=1.5
        )

    def test_control_points_away_from_nearby_obstacle(self) -> None:
        control = self.controller.control_for(0, (1.0, 0.0))

        self.assertGreater(control[0], 0.0)
        self.assertAlmostEqual(control[1], 0.0)

    def test_control_is_zero_outside_influence_distance(self) -> None:
        np.testing.assert_allclose(self.controller.control_for(0, (3.0, 0.0)), (0.0, 0.0))
        self.assertEqual(self.controller.potential({0: (3.0, 0.0)}), 0.0)

    def test_repulsive_potential_is_positive_inside_influence_distance(self) -> None:
        self.assertGreater(self.controller.potential({0: (1.0, 0.0)}), 0.0)

    def test_moving_obstacle_is_rejected_by_static_controller_but_accepted_by_reactive_one(self) -> None:
        moving = DynamicObstacle(identifier=1, position=(0.0, 0.0), velocity=(1.0, 0.0))
        with self.assertRaises(ValueError):
            StaticObstacleAvoidanceController(agent_radii={0: 0.15}, obstacles=[moving])

        controller = PositionBasedObstacleAvoidanceController(agent_radii={0: 0.15}, obstacles=[moving])
        self.assertGreater(controller.potential({0: (1.0, 0.0)}), 0.0)


if __name__ == "__main__":
    unittest.main()
