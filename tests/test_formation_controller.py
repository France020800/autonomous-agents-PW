import unittest

import numpy as np

from mas_formation.controllers import FormationController
from mas_formation.models import TriangularFormation


class TestFormationController(unittest.TestCase):
    def setUp(self) -> None:
        self.formation = TriangularFormation.equilateral((0, 1, 2), side_length=1.0)
        self.controller = FormationController(self.formation, gain=2.0)
        self.target_positions = {
            0: (0.0, 0.0),
            1: (1.0, 0.0),
            2: (0.5, np.sqrt(3) / 2),
        }

    def test_potential_and_controls_vanish_at_desired_formation(self) -> None:
        controls = self.controller.controls(self.target_positions)

        self.assertAlmostEqual(self.controller.potential(self.target_positions), 0.0)
        for control in controls.values():
            np.testing.assert_allclose(control, (0.0, 0.0), atol=1e-12)

    def test_local_control_uses_only_neighbors_and_has_correct_direction(self) -> None:
        stretched = dict(self.target_positions)
        stretched[1] = (2.0, 0.0)

        control = self.controller.control_for(0, stretched[0], {1: stretched[1], 2: stretched[2]})

        # L'arco (0, 1) è troppo lungo: l'agente 0 deve muoversi verso destra.
        self.assertGreater(control[0], 0.0)

    def test_missing_neighbor_position_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            self.controller.control_for(0, (0.0, 0.0), {1: (1.0, 0.0)})

    def test_small_gradient_step_decreases_potential(self) -> None:
        distorted = dict(self.target_positions)
        distorted[1] = (1.3, 0.0)
        initial_potential = self.controller.potential(distorted)
        controls = self.controller.controls(distorted)
        updated = {identifier: np.asarray(position) + 1e-3 * controls[identifier]
                   for identifier, position in distorted.items()}

        self.assertLess(self.controller.potential(updated), initial_potential)


if __name__ == "__main__":
    unittest.main()
