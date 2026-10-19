import unittest

import numpy as np

from mas_formation.models import Agent, DynamicObstacle, Environment


class TestModels(unittest.TestCase):
    def test_agent_single_integrator_and_saturation(self) -> None:
        agent = Agent(identifier=0, position=(1.0, -1.0), max_speed=2.0)
        next_position, applied_control = agent.propose_step((3.0, 4.0), dt=0.5)

        np.testing.assert_allclose(applied_control, (1.2, 1.6))
        np.testing.assert_allclose(next_position, (1.6, -0.2))

    def test_static_and_dynamic_obstacle(self) -> None:
        static = DynamicObstacle(identifier=0, position=(0.0, 0.0))
        moving = DynamicObstacle(identifier=1, position=(1.0, 2.0), velocity=(-2.0, 0.5))

        np.testing.assert_allclose(static.propose_step(0.4), (0.0, 0.0))
        np.testing.assert_allclose(moving.propose_step(0.4), (0.2, 2.2))

    def test_environment_updates_all_entities_synchronously(self) -> None:
        first = Agent(identifier=10, position=(0.0, 0.0), max_speed=5.0)
        second = Agent(identifier=20, position=(5.0, 0.0), max_speed=5.0)
        obstacle = DynamicObstacle(identifier=7, position=(2.0, 1.0), velocity=(0.0, -0.5))
        environment = Environment([first, second], [obstacle])

        environment.step({10: (1.0, 0.0), 20: (-2.0, 1.0)}, dt=0.2)

        np.testing.assert_allclose(first.position, (0.2, 0.0))
        np.testing.assert_allclose(second.position, (4.6, 0.2))
        np.testing.assert_allclose(obstacle.position, (2.0, 0.9))
        self.assertEqual(environment.time, 0.2)

    def test_failed_step_does_not_modify_the_environment(self) -> None:
        agent = Agent(identifier=0, position=(0.9, 0.5), max_speed=1.0)
        environment = Environment([agent], lower_bound=(0.0, 0.0), upper_bound=(1.0, 1.0))

        with self.assertRaises(ValueError):
            environment.step({0: (1.0, 0.0)}, dt=0.2)

        np.testing.assert_allclose(agent.position, (0.9, 0.5))
        np.testing.assert_allclose(agent.velocity, (0.0, 0.0))
        self.assertEqual(environment.time, 0.0)


if __name__ == "__main__":
    unittest.main()
