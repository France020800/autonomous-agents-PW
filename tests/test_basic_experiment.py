import unittest

import numpy as np

from mas_formation.experiments import (
    run_basic_formation_experiment,
    run_dynamic_navigation_experiment,
    run_formation_goal_experiment,
    run_moving_obstacle_experiment,
    run_static_obstacle_experiment,
)


class TestBasicFormationExperiment(unittest.TestCase):
    def test_seed_makes_initial_positions_reproducible(self) -> None:
        first = run_basic_formation_experiment(duration=0.1, seed=123)
        second = run_basic_formation_experiment(duration=0.1, seed=123)
        different = run_basic_formation_experiment(duration=0.1, seed=456)

        np.testing.assert_allclose(first.positions[0], second.positions[0])
        self.assertFalse(np.allclose(first.positions[0], different.positions[0]))

    def test_formation_potential_decreases(self) -> None:
        result = run_basic_formation_experiment(seed=42)

        self.assertLess(result.potentials[-1], result.potentials[0])
        self.assertTrue(result.formation.is_satisfied(
            {identifier: result.final_positions[index] for index, identifier in enumerate(result.agent_ids)},
            tolerance=1e-5,
        ))

    def test_goal_is_generated_at_activation_and_then_reached(self) -> None:
        dt = 0.01
        goal_distance = 8.0
        result = run_formation_goal_experiment(dt=dt, seed=42, goal_distance=goal_distance)
        activation_index = round(result.navigation_start_time / dt)
        activation_centroid = result.positions[activation_index].mean(axis=0)

        self.assertAlmostEqual(np.linalg.norm(result.goal - activation_centroid), goal_distance)
        self.assertLessEqual(np.linalg.norm(result.goal - result.final_positions.mean(axis=0)), 0.1)
        self.assertFalse(np.isnan(result.goal_potentials[activation_index]))
        self.assertLess(result.goal_potentials[-1], result.goal_potentials[activation_index])

    def test_static_obstacle_experiment_keeps_agents_outside_obstacle(self) -> None:
        result = run_static_obstacle_experiment(seed=42)
        obstacle = result.obstacles[0]
        clearances = np.linalg.norm(result.positions - obstacle.position, axis=2) - obstacle.radius - 0.15

        self.assertEqual(len(result.obstacles), 3)
        self.assertGreater(clearances.min(), 0.0)
        self.assertGreater(result.obstacle_potentials.max(), 0.0)

    def test_static_experiment_accepts_a_configurable_obstacle_count(self) -> None:
        result = run_static_obstacle_experiment(seed=42, obstacle_count=2)

        self.assertEqual(len(result.obstacles), 2)

    def test_moving_obstacle_perturbs_initially_static_formation(self) -> None:
        result = run_moving_obstacle_experiment(duration=20.0, seed=42)

        np.testing.assert_allclose(result.positions[0], result.positions[200], atol=1e-12)
        self.assertGreater(np.linalg.norm(result.positions[1600] - result.positions[0]), 0.1)
        self.assertGreater(result.obstacle_potentials.max(), 0.0)

    def test_moving_obstacle_trajectories_are_reproducible_with_seed(self) -> None:
        first = run_moving_obstacle_experiment(seed=7)
        second = run_moving_obstacle_experiment(seed=7)
        different = run_moving_obstacle_experiment(seed=8)

        np.testing.assert_allclose(first.obstacle_positions, second.obstacle_positions)
        self.assertFalse(np.allclose(first.obstacle_positions, different.obstacle_positions))

    def test_dynamic_navigation_reaches_goal_after_moving_obstacles_pass(self) -> None:
        result = run_dynamic_navigation_experiment(seed=42, moving_obstacle_count=2)
        final_centroid = result.final_positions.mean(axis=0)

        self.assertLessEqual(np.linalg.norm(final_centroid - result.goal), 0.1)
        self.assertGreater(result.obstacle_potentials.max(), 0.0)
        self.assertGreater(np.linalg.norm(result.obstacle_positions[-1] - result.obstacle_positions[0]), 0.0)


if __name__ == "__main__":
    unittest.main()
