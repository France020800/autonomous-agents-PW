import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from mas_formation.experiments import MovingObstacleExperimentResult
from mas_formation.experiments.compare_avoidance import measure_result, run_comparison
from mas_formation.models import DynamicObstacle, TriangularFormation


class TestAvoidanceComparison(unittest.TestCase):
    def test_metrics_use_real_clearance_and_count_frames_not_events(self) -> None:
        formation = TriangularFormation.equilateral((0, 1, 2), side_length=2.0)
        positions = np.tile(np.array([[0.0, 0.0], [2.0, 0.0], [1.0, np.sqrt(3.0)]]), (3, 1, 1))
        result = MovingObstacleExperimentResult(
            agent_ids=(0, 1, 2), times=np.array([0.0, 0.1, 0.2]), positions=positions,
            potentials=np.zeros(3), formation=formation,
            obstacles=(DynamicObstacle(identifier=0, position=(0.0, 4.0), radius=0.45),),
            obstacle_positions=np.array([[[0.0, 1.0]], [[0.0, 0.5]], [[0.0, 4.0]]]),
            obstacle_potentials=np.array([0.0, 1.0, 0.0]),
        )
        metrics, curves = measure_result(result)
        self.assertAlmostEqual(metrics["minimum_real_clearance_m"], -0.1)
        self.assertEqual(metrics["contact_frames"], 1)
        self.assertEqual(metrics["first_repulsion_time_s"], 0.1)
        self.assertAlmostEqual(metrics["rms_formation_error_m"], 0.0)
        self.assertIsNone(metrics["final_goal_error_m"])
        self.assertTrue(np.isnan(curves["goal_error_m"]).all())

    def test_report_and_raw_data_are_saved_with_explicit_configuration(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output = run_comparison(temporary, duration=3.0, dt=0.02)
            data = json.loads((output / "metrics.json").read_text(encoding="utf-8"))
            self.assertEqual(data["configuration"]["seed"], 42)
            self.assertEqual(len(data["results"]), 4)
            self.assertNotIn("NaN", (output / "metrics.json").read_text(encoding="utf-8"))
            for scenario in ("moving-obstacles", "dynamic-navigation"):
                with np.load(output / f"{scenario}_reattivo.npz", allow_pickle=False) as reactive, \
                     np.load(output / f"{scenario}_predittivo.npz", allow_pickle=False) as predictive:
                    np.testing.assert_array_equal(reactive["positions"][0], predictive["positions"][0])
                    np.testing.assert_array_equal(reactive["obstacle_positions"], predictive["obstacle_positions"])
                self.assertTrue((output / f"{scenario}_metrics.png").is_file())
                self.assertTrue((output / f"{scenario}_trajectories.png").is_file())
            self.assertTrue((output / "report.md").is_file())
            self.assertTrue((output / "metrics.csv").is_file())

    def test_comparison_requires_nonzero_predictive_horizon(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            with self.assertRaises(ValueError):
                run_comparison(Path(temporary) / "invalid", prediction_horizon=0.0)
            self.assertFalse((Path(temporary) / "invalid").exists())


if __name__ == "__main__":
    unittest.main()
