import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from mas_formation.experiments import (
    export_interactive_html,
    run_basic_formation_experiment,
    run_dynamic_navigation_experiment,
    run_static_obstacle_experiment,
)


class TestHtmlViewer(unittest.TestCase):
    def read_export(self, result):
        with tempfile.TemporaryDirectory() as directory:
            path = export_interactive_html(result, Path(directory) / "nested" / "simulation.html")
            html = path.read_text(encoding="utf-8")
        payload = html.split('<script id="simulation-data" type="application/json">')[1].split('</script>')[0]
        self.assertNotIn("__SIMULATION_DATA__", html)
        self.assertNotIn('<script src=', html)
        self.assertIn('type="range"', html)
        return json.loads(payload)

    def test_export_preserves_all_formation_timesteps(self):
        result = run_basic_formation_experiment(duration=0.1, seed=42)
        data = self.read_export(result)
        np.testing.assert_allclose(data["positions"], result.positions)
        self.assertEqual(len(data["times"]), len(result.times))
        self.assertIsNone(data["goal"])
        self.assertEqual(data["obstacleRadii"], [])

    def test_dynamic_export_preserves_obstacles_and_inactive_goal_values(self):
        result = run_dynamic_navigation_experiment(duration=3.0, seed=42, moving_obstacle_count=2)
        data = self.read_export(result)
        self.assertTrue(data["dynamic"])
        self.assertIsNone(data["goalPotential"][0])
        self.assertEqual(data["navigationStart"], result.navigation_start_time)
        np.testing.assert_allclose(data["obstaclePositions"], result.obstacle_positions)
        np.testing.assert_allclose(data["goal"], result.goal)

    def test_static_obstacle_export_stores_fixed_positions(self):
        result = run_static_obstacle_experiment(duration=3.0, seed=42)
        data = self.read_export(result)
        self.assertFalse(data["dynamic"])
        self.assertEqual(len(data["obstaclePositions"]), 1)
        self.assertEqual(len(data["obstacleRadii"]), len(result.obstacles))


if __name__ == "__main__":
    unittest.main()
