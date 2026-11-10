import unittest

import numpy as np

from mas_formation.controllers import (
    PositionBasedObstacleAvoidanceController,
    PredictiveObstacleAvoidanceController,
    StaticObstacleAvoidanceController,
)
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


class TestPredictiveObstacleAvoidance(unittest.TestCase):
    def test_approaching_obstacle_activates_repulsion_earlier_than_reactive_controller(self) -> None:
        # Attualmente fuori dal raggio d'influenza (clearance = 2.5 - 0.15 - 0.5 = 1.85 > 1.5),
        # ma in avvicinamento verso l'agente in (0, 0).
        approaching = DynamicObstacle(identifier=0, position=(-2.5, 0.0), radius=0.5, velocity=(1.0, 0.0))
        reactive = PositionBasedObstacleAvoidanceController(
            agent_radii={0: 0.15}, obstacles=[approaching], gain=0.08, influence_distance=1.5
        )
        predictive = PredictiveObstacleAvoidanceController(
            agent_radii={0: 0.15},
            obstacles=[approaching],
            gain=0.08,
            influence_distance=1.5,
            prediction_horizon=1.5,
            prediction_steps=15,
        )

        self.assertEqual(reactive.potential({0: (0.0, 0.0)}), 0.0)
        np.testing.assert_allclose(reactive.control_for(0, (0.0, 0.0)), (0.0, 0.0))

        self.assertGreater(predictive.potential({0: (0.0, 0.0)}), 0.0)
        control = predictive.control_for(0, (0.0, 0.0))
        self.assertGreater(control[0], 0.0)
        self.assertAlmostEqual(control[1], 0.0)
        np.testing.assert_allclose(predictive.closest_predicted_position((0.0, 0.0), approaching), (-1.0, 0.0))

    def test_receding_obstacle_uses_current_distance_as_minimum(self) -> None:
        receding = DynamicObstacle(identifier=0, position=(-1.2, 0.0), radius=0.5, velocity=(-1.0, 0.0))
        reactive = PositionBasedObstacleAvoidanceController(
            agent_radii={0: 0.15}, obstacles=[receding], gain=0.08, influence_distance=1.5
        )
        predictive = PredictiveObstacleAvoidanceController(
            agent_radii={0: 0.15},
            obstacles=[receding],
            gain=0.08,
            influence_distance=1.5,
            prediction_horizon=1.5,
            prediction_steps=10,
        )

        np.testing.assert_allclose(predictive.closest_predicted_position((0.0, 0.0), receding), receding.position)
        self.assertAlmostEqual(predictive.potential({0: (0.0, 0.0)}), reactive.potential({0: (0.0, 0.0)}))
        np.testing.assert_allclose(predictive.control_for(0, (0.0, 0.0)), reactive.control_for(0, (0.0, 0.0)))

    def test_zero_horizon_or_static_obstacle_matches_position_based_controller(self) -> None:
        moving = DynamicObstacle(identifier=0, position=(-1.4, 0.2), radius=0.45, velocity=(0.6, -0.2))
        static = DynamicObstacle(identifier=1, position=(1.2, -0.3), radius=0.45, velocity=(0.0, 0.0))
        reactive = PositionBasedObstacleAvoidanceController(
            agent_radii={0: 0.15}, obstacles=[moving, static], gain=0.2, influence_distance=2.0
        )
        zero_horizon = PredictiveObstacleAvoidanceController(
            agent_radii={0: 0.15},
            obstacles=[moving, static],
            gain=0.2,
            influence_distance=2.0,
            prediction_horizon=0.0,
            prediction_steps=10,
        )

        self.assertAlmostEqual(zero_horizon.potential({0: (0.0, 0.0)}), reactive.potential({0: (0.0, 0.0)}))
        np.testing.assert_allclose(zero_horizon.control_for(0, (0.0, 0.0)), reactive.control_for(0, (0.0, 0.0)))

    def test_invalid_prediction_parameters_raise_value_error(self) -> None:
        obstacle = DynamicObstacle(identifier=0, position=(0.0, 0.0), radius=0.5)
        with self.assertRaises(ValueError):
            PredictiveObstacleAvoidanceController(
                agent_radii={0: 0.15}, obstacles=[obstacle], prediction_horizon=-0.5
            )
        with self.assertRaises(ValueError):
            PredictiveObstacleAvoidanceController(
                agent_radii={0: 0.15}, obstacles=[obstacle], prediction_steps=0
            )


if __name__ == "__main__":
    unittest.main()
