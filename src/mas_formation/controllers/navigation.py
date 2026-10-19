"""Navigazione della formazione tramite controllo del suo baricentro."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike

from mas_formation.models._vectors import Vector2, as_vector2


def random_goal_from_centroid(
    centroid: ArrayLike, distance: float, generator: np.random.Generator
) -> Vector2:
    """Estrae un goal su una circonferenza di raggio ``distance`` dal baricentro."""
    center = as_vector2(centroid, name="centroid")
    if not np.isfinite(distance) or distance <= 0:
        raise ValueError("distance deve essere finita e strettamente positiva.")
    angle = generator.uniform(0.0, 2.0 * np.pi)
    return center + distance * np.array((np.cos(angle), np.sin(angle)))


@dataclass(frozen=True)
class GoalNavigationController:
    """Porta il baricentro di una formazione verso un goal assegnato.

    Tutti gli agenti ricevono lo stesso comando di traslazione
    ``u_g = gain * (goal - centroid)``. Perciò questo termine, da solo, non
    altera le distanze reciproche della formazione. Esso è il negativo del
    gradiente, rispetto agli stati congiunti, di
    ``V_g = gain * N / 2 * ||centroid - goal||^2``.
    """

    agent_ids: Sequence[int]
    goal: ArrayLike
    gain: float = 0.6
    arrival_tolerance: float = 0.1

    def __post_init__(self) -> None:
        ids = tuple(self.agent_ids)
        if not ids or len(ids) != len(set(ids)):
            raise ValueError("agent_ids deve contenere agenti distinti.")
        if not np.isfinite(self.gain) or self.gain <= 0:
            raise ValueError("gain deve essere finito e strettamente positivo.")
        if not np.isfinite(self.arrival_tolerance) or self.arrival_tolerance <= 0:
            raise ValueError("arrival_tolerance deve essere finita e strettamente positiva.")
        object.__setattr__(self, "agent_ids", ids)
        object.__setattr__(self, "goal", as_vector2(self.goal, name="goal"))

    def centroid(self, positions: Mapping[int, ArrayLike]) -> Vector2:
        """Calcola il baricentro degli agenti appartenenti alla formazione."""
        missing = set(self.agent_ids) - set(positions)
        if missing:
            raise ValueError(f"Mancano le posizioni degli agenti: {missing}.")
        return np.mean(
            [as_vector2(positions[identifier], name=f"positions[{identifier}]") for identifier in self.agent_ids],
            axis=0,
        )

    def potential(self, positions: Mapping[int, ArrayLike]) -> float:
        """Valore del potenziale attrattivo del goal per l'intero MAS."""
        error = self.centroid(positions) - self.goal
        return 0.5 * self.gain * len(self.agent_ids) * float(error @ error)

    def velocity(self, positions: Mapping[int, ArrayLike]) -> Vector2:
        """Restituisce la velocità comune di traslazione della formazione."""
        error = self.goal - self.centroid(positions)
        if np.linalg.norm(error) <= self.arrival_tolerance:
            return np.zeros(2, dtype=float)
        return self.gain * error

    def has_reached_goal(self, positions: Mapping[int, ArrayLike]) -> bool:
        """Indica se il baricentro è nella regione di arrivo del goal."""
        return np.linalg.norm(self.goal - self.centroid(positions)) <= self.arrival_tolerance

    def controls(self, positions: Mapping[int, ArrayLike]) -> dict[int, Vector2]:
        """Assegna lo stesso comando di navigazione a ciascun agente."""
        common_velocity = self.velocity(positions)
        return {identifier: common_velocity.copy() for identifier in self.agent_ids}
