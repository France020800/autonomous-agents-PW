import unittest

from mas_formation.models import FormationEdge, TriangularFormation


class TestTriangularFormation(unittest.TestCase):
    def test_equilateral_graph_has_all_k3_edges(self) -> None:
        formation = TriangularFormation.equilateral((3, 5, 8), side_length=2.0)

        self.assertEqual(len(formation.edges), 3)
        self.assertEqual(formation.desired_distance(3, 5), 2.0)
        self.assertEqual(formation.desired_distance(5, 3), 2.0)

    def test_desired_distances_define_a_triangle_without_reference_coordinates(self) -> None:
        formation = TriangularFormation.from_desired_distances(
            agent_ids=(0, 1, 2),
            desired_distances={(0, 1): 3.0, (2, 0): 4.0, (1, 2): 5.0},
        )

        self.assertEqual(formation.desired_distance(0, 1), 3.0)
        self.assertEqual(formation.desired_distance(0, 2), 4.0)
        self.assertEqual(formation.desired_distance(1, 2), 5.0)

    def test_reference_triangle_satisfies_its_graph(self) -> None:
        positions = {0: (0.0, 0.0), 1: (3.0, 0.0), 2: (0.0, 4.0)}
        formation = TriangularFormation.from_reference_positions(positions)

        self.assertTrue(formation.is_satisfied(positions))
        self.assertEqual(formation.desired_distance(1, 2), 5.0)

    def test_distance_errors_detect_deformation(self) -> None:
        formation = TriangularFormation.equilateral((0, 1, 2), side_length=1.0)
        errors = formation.distance_errors({0: (0.0, 0.0), 1: (2.0, 0.0), 2: (0.0, 1.0)})

        self.assertEqual(errors[(0, 1)], 1.0)
        self.assertFalse(formation.is_satisfied({0: (0.0, 0.0), 1: (2.0, 0.0), 2: (0.0, 1.0)}))

    def test_degenerate_triangle_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            TriangularFormation(
                agent_ids=(0, 1, 2),
                edges=(
                    FormationEdge(0, 1, 1.0),
                    FormationEdge(0, 2, 1.0),
                    FormationEdge(1, 2, 2.0),
                ),
            )

    def test_incomplete_graph_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            TriangularFormation(
                agent_ids=(0, 1, 2),
                edges=(FormationEdge(0, 1, 1.0), FormationEdge(0, 2, 1.0)),
            )

    def test_equilateral_creates_regular_polygon_with_equal_sides_for_more_than_three_agents(self) -> None:
        square = TriangularFormation.equilateral((0, 1, 2, 3), side_length=2.0)

        self.assertEqual(len(square.edges), 6)
        for first, second in ((0, 1), (1, 2), (2, 3), (0, 3)):
            self.assertAlmostEqual(square.desired_distance(first, second), 2.0)
        for first, second in ((0, 2), (1, 3)):
            self.assertAlmostEqual(square.desired_distance(first, second), 2.0 * (2.0**0.5))
        self.assertTrue(square.is_satisfied(square.regular_polygon_positions(side_length=2.0)))

        pentagon = TriangularFormation.equilateral((0, 1, 2, 3, 4), side_length=1.5)
        self.assertEqual(len(pentagon.edges), 10)
        for idx in range(5):
            self.assertAlmostEqual(pentagon.desired_distance(idx, (idx + 1) % 5), 1.5)
        self.assertTrue(pentagon.is_satisfied(pentagon.regular_polygon_positions(side_length=1.5)))

    def test_fewer_than_three_agents_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            TriangularFormation.equilateral((0, 1), side_length=2.0)


if __name__ == "__main__":
    unittest.main()
