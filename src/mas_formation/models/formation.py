"""Grafo dei vincoli metrici per una formazione poligonale regolare rigida."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from itertools import combinations

import numpy as np
from numpy.typing import ArrayLike

from ._vectors import Vector2, as_vector2


@dataclass(frozen=True)
class FormationEdge:
    """Vincolo non orientato di distanza desiderata fra due agenti."""

    first_agent: int
    second_agent: int
    desired_distance: float

    def __post_init__(self) -> None:
        if self.first_agent == self.second_agent:
            raise ValueError("Un arco di formazione deve collegare due agenti distinti.")
        if not np.isfinite(self.desired_distance) or self.desired_distance <= 0:
            raise ValueError("desired_distance deve essere finita e strettamente positiva.")

    @property
    def key(self) -> frozenset[int]:
        """Chiave non orientata dell'arco."""
        return frozenset((self.first_agent, self.second_agent))


@dataclass(frozen=True)
class TriangularFormation:
    """Grafo completo ``K_N`` (con ``N >= 3``) che descrive una formazione rigida nel piano.

    Gli archi impongono ``||p_i - p_j|| = d_ij``. Le disuguaglianze strette
    del triangolo su ogni terna garantiscono che i vincoli siano metricamente
    consistenti e non degeneri, definendo una figura rigida a meno di
    traslazioni, rotazioni e riflessioni.
    """

    agent_ids: Sequence[int]
    edges: Sequence[FormationEdge]

    def __post_init__(self) -> None:
        agent_ids = tuple(self.agent_ids)
        edges = tuple(self.edges)
        if len(agent_ids) < 3 or len(set(agent_ids)) != len(agent_ids):
            raise ValueError("Una formazione richiede almeno tre agenti distinti.")

        expected_keys = {frozenset(pair) for pair in combinations(agent_ids, 2)}
        actual_keys = [edge.key for edge in edges]
        if (
            len(edges) != len(expected_keys)
            or len(set(actual_keys)) != len(expected_keys)
            or set(actual_keys) != expected_keys
        ):
            raise ValueError("Il grafo deve contenere esattamente tutti gli archi del grafo completo.")

        distance_by_key = {edge.key: edge.desired_distance for edge in edges}
        for first, second, third in combinations(agent_ids, 3):
            lengths = sorted((
                distance_by_key[frozenset((first, second))],
                distance_by_key[frozenset((first, third))],
                distance_by_key[frozenset((second, third))],
            ))
            if lengths[0] + lengths[1] <= lengths[2]:
                raise ValueError("Le distanze desiderate devono soddisfare le disuguaglianze strette del triangolo.")

        object.__setattr__(self, "agent_ids", agent_ids)
        object.__setattr__(self, "edges", edges)

    @classmethod
    def equilateral(cls, agent_ids: Sequence[int], side_length: float) -> "TriangularFormation":
        """Costruisce un poligono regolare rigido con tutti i lati di lunghezza ``side_length``.

        Per 3 agenti genera il triangolo equilatero ``K3`` con tutti i lati uguali a
        ``side_length``. Per ``N > 3`` agenti genera il grafo completo ``K_N`` di un
        poligono regolare a ``N`` lati di lunghezza ``side_length``, assegnando alle
        diagonali le corrispondenti corde del poligono regolare per garantirne la
        rigidità metrica nel piano.
        """
        ids = tuple(agent_ids)
        count = len(ids)
        if count < 3 or len(set(ids)) != count:
            raise ValueError("Una formazione regolare richiede almeno tre agenti distinti.")
        if not np.isfinite(side_length) or side_length <= 0:
            raise ValueError("side_length deve essere finita e strettamente positiva.")

        sin_base = float(np.sin(np.pi / count))
        edges = []
        for first_index, second_index in combinations(range(count), 2):
            step = min(second_index - first_index, count - (second_index - first_index))
            distance = (
                float(side_length)
                if step == 1
                else float(side_length * np.sin(np.pi * step / count) / sin_base)
            )
            edges.append(FormationEdge(ids[first_index], ids[second_index], distance))
        return cls(ids, tuple(edges))

    def regular_polygon_positions(self, side_length: float) -> dict[int, Vector2]:
        """Restituisce i vertici del poligono regolare centrato nell'origine."""
        if not np.isfinite(side_length) or side_length <= 0:
            raise ValueError("side_length deve essere finita e strettamente positiva.")
        count = len(self.agent_ids)
        if count == 3:
            sqrt3 = float(np.sqrt(3.0))
            half = 0.5 * float(side_length)
            return {
                self.agent_ids[0]: np.array((-half, -half * sqrt3 / 3.0), dtype=float),
                self.agent_ids[1]: np.array((half, -half * sqrt3 / 3.0), dtype=float),
                self.agent_ids[2]: np.array((0.0, 2.0 * half * sqrt3 / 3.0), dtype=float),
            }
        circumradius = float(side_length / (2.0 * np.sin(np.pi / count)))
        return {
            identifier: circumradius * np.array((
                np.cos(-0.5 * np.pi - (np.pi / count) + (2.0 * np.pi * index / count)),
                np.sin(-0.5 * np.pi - (np.pi / count) + (2.0 * np.pi * index / count)),
            ), dtype=float)
            for index, identifier in enumerate(self.agent_ids)
        }

    @classmethod
    def from_desired_distances(
        cls, agent_ids: Sequence[int], desired_distances: Mapping[tuple[int, int], float]
    ) -> "TriangularFormation":
        """Costruisce il grafo direttamente dalle distanze desiderate sugli archi.

        Non richiede coordinate assolute: il controllore di formazione porterà
        gli agenti a una configurazione che rispetta tali distanze.
        """
        ids = tuple(agent_ids)
        if len(ids) < 3 or len(set(ids)) != len(ids):
            raise ValueError("Una formazione richiede almeno tre agenti distinti.")

        normalized_distances: dict[frozenset[int], float] = {}
        for pair, distance in desired_distances.items():
            if len(pair) != 2 or pair[0] == pair[1]:
                raise ValueError("Ogni distanza deve avere una coppia di agenti distinti come chiave.")
            key = frozenset(pair)
            if key in normalized_distances:
                raise ValueError(f"Distanza duplicata per l'arco {tuple(pair)}.")
            normalized_distances[key] = distance

        expected_keys = {frozenset(pair) for pair in combinations(ids, 2)}
        if set(normalized_distances) != expected_keys:
            raise ValueError("Sono richieste tutte le distanze degli archi del grafo completo.")
        return cls(
            ids,
            tuple(
                FormationEdge(first, second, normalized_distances[frozenset((first, second))])
                for first, second in combinations(ids, 2)
            ),
        )

    @classmethod
    def from_reference_positions(
        cls, reference_positions: Mapping[int, ArrayLike]
    ) -> "TriangularFormation":
        """Deriva gli archi metrici dalle coordinate dei vertici di riferimento."""
        if len(reference_positions) < 3:
            raise ValueError("Sono necessarie almeno tre posizioni di riferimento.")
        ids = tuple(reference_positions)
        positions = {identifier: as_vector2(position, name=f"reference_positions[{identifier}]")
                     for identifier, position in reference_positions.items()}
        edges = tuple(
            FormationEdge(first, second, float(np.linalg.norm(positions[first] - positions[second])))
            for first, second in combinations(ids, 2)
        )
        return cls(ids, edges)

    def desired_distance(self, first_agent: int, second_agent: int) -> float:
        """Restituisce il vincolo di distanza tra due nodi del grafo."""
        requested_key = frozenset((first_agent, second_agent))
        for edge in self.edges:
            if edge.key == requested_key:
                return edge.desired_distance
        raise KeyError(f"Nessun arco tra gli agenti {first_agent} e {second_agent}.")

    def distance_errors(self, positions: Mapping[int, ArrayLike]) -> dict[tuple[int, int], float]:
        """Calcola ``||p_i-p_j|| - d_ij`` per ciascun arco del grafo."""
        missing = set(self.agent_ids) - set(positions)
        if missing:
            raise ValueError(f"Mancano le posizioni degli agenti: {missing}.")

        vectors = {identifier: as_vector2(positions[identifier], name=f"positions[{identifier}]")
                   for identifier in self.agent_ids}
        return {
            (edge.first_agent, edge.second_agent): float(
                np.linalg.norm(vectors[edge.first_agent] - vectors[edge.second_agent]) - edge.desired_distance
            )
            for edge in self.edges
        }

    def is_satisfied(self, positions: Mapping[int, ArrayLike], tolerance: float = 1e-6) -> bool:
        """Verifica che tutti i vincoli metrici siano rispettati entro la tolleranza."""
        if tolerance < 0 or not np.isfinite(tolerance):
            raise ValueError("tolerance deve essere finita e non negativa.")
        return all(abs(error) <= tolerance for error in self.distance_errors(positions).values())
