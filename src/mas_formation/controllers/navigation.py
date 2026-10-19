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

    Il potenziale attrattivo usa una funzione di Huber scalata sulla distanza
    del baricentro dal goal. Oltre la soglia ``huber_delta`` il modulo del
    comando è costante e pari a ``gain``; ciò evita il rallentamento
    esponenziale caratteristico di un potenziale puramente quadratico.

    Tutti gli agenti ricevono lo stesso comando di traslazione, perciò questo
    termine, da solo, non altera le distanze reciproche della formazione.
    """

    agent_ids: Sequence[int]
    goal: ArrayLike
    gain: float = 0.6
    arrival_tolerance: float = 0.1
    huber_delta: float = 0.05

    def __post_init__(self) -> None:
        ids = tuple(self.agent_ids)
        if not ids or len(ids) != len(set(ids)):
            raise ValueError("agent_ids deve contenere agenti distinti.")
        if not np.isfinite(self.gain) or self.gain <= 0:
            raise ValueError("gain deve essere finito e strettamente positivo.")
        if not np.isfinite(self.arrival_tolerance) or self.arrival_tolerance <= 0:
            raise ValueError("arrival_tolerance deve essere finita e strettamente positiva.")
        if not np.isfinite(self.huber_delta) or self.huber_delta <= 0:
            raise ValueError("huber_delta deve essere finita e strettamente positiva.")
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
        """Valore del potenziale attrattivo di Huber per l'intero MAS."""
        distance = float(np.linalg.norm(self.centroid(positions) - self.goal))
        if distance <= self.huber_delta:
            huber_value = 0.5 * distance**2
        else:
            huber_value = self.huber_delta * (distance - 0.5 * self.huber_delta)
        return self.gain * len(self.agent_ids) * huber_value / self.huber_delta

    def velocity(self, positions: Mapping[int, ArrayLike]) -> Vector2:
        """Restituisce la velocità comune derivata dal potenziale di Huber."""
        error = self.goal - self.centroid(positions)
        distance = float(np.linalg.norm(error))
        if distance <= self.arrival_tolerance:
            return np.zeros(2, dtype=float)
        if distance <= self.huber_delta:
            return self.gain * error / self.huber_delta
        return self.gain * error / distance

    def has_reached_goal(self, positions: Mapping[int, ArrayLike]) -> bool:
        """Indica se il baricentro è nella regione di arrivo del goal."""
        return np.linalg.norm(self.goal - self.centroid(positions)) <= self.arrival_tolerance

    def controls(self, positions: Mapping[int, ArrayLike]) -> dict[int, Vector2]:
        """Assegna lo stesso comando di navigazione a ciascun agente."""
        common_velocity = self.velocity(positions)
        return {identifier: common_velocity.copy() for identifier in self.agent_ids}
