"""Evitamento di ostacoli circolari fissi tramite potenziale repulsivo."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike

from mas_formation.models._vectors import Vector2, as_vector2
from mas_formation.models.obstacle import DynamicObstacle


@dataclass(frozen=True)
class PositionBasedObstacleAvoidanceController:
    """Genera repulsione APF dalla posizione istantanea di ostacoli circolari.

    Per la clearance ``rho = ||p_i-q_j|| - r_i-r_j`` usa il potenziale
    ``V_o = gain/2 * (1/rho - 1/influence_distance)^2`` entro il raggio
    d'influenza e zero al di fuori. Può essere usato anche con ostacoli mobili,
    ma non sfrutta ancora la loro velocità: la variante predittiva sarà
    introdotta in un esperimento successivo.
    """

    agent_radii: Mapping[int, float]
    obstacles: Sequence[DynamicObstacle]
    gain: float = 0.08
    influence_distance: float = 1.5
    minimum_clearance: float = 0.05

    def __post_init__(self) -> None:
        radii = dict(self.agent_radii)
        if not radii or any(radius <= 0 or not np.isfinite(radius) for radius in radii.values()):
            raise ValueError("agent_radii deve contenere raggi finiti e strettamente positivi.")
        if not np.isfinite(self.gain) or self.gain <= 0:
            raise ValueError("gain deve essere finito e strettamente positivo.")
        if not np.isfinite(self.influence_distance) or self.influence_distance <= 0:
            raise ValueError("influence_distance deve essere finita e strettamente positiva.")
        if not np.isfinite(self.minimum_clearance) or self.minimum_clearance <= 0:
            raise ValueError("minimum_clearance deve essere finita e strettamente positiva.")
        obstacles = tuple(self.obstacles)
        object.__setattr__(self, "agent_radii", radii)
        object.__setattr__(self, "obstacles", obstacles)

    def potential_for(self, agent_id: int, position: ArrayLike) -> float:
        """Valore del potenziale repulsivo per un singolo agente."""
        if agent_id not in self.agent_radii:
            raise KeyError(f"Raggio sconosciuto per l'agente {agent_id}.")
        agent_position = as_vector2(position, name="position")
        potential = 0.0
        for obstacle in self.obstacles:
            center_distance = float(np.linalg.norm(agent_position - obstacle.position))
            clearance = center_distance - self.agent_radii[agent_id] - obstacle.radius
            if clearance < self.influence_distance:
                safe_clearance = max(clearance, self.minimum_clearance)
                potential += 0.5 * self.gain * (1.0 / safe_clearance - 1.0 / self.influence_distance) ** 2
        return potential

    def potential(self, positions: Mapping[int, ArrayLike]) -> float:
        """Somma dei potenziali repulsivi di tutti gli agenti e ostacoli."""
        missing = set(self.agent_radii) - set(positions)
        if missing:
            raise ValueError(f"Mancano le posizioni degli agenti: {missing}.")
        return sum(self.potential_for(agent_id, positions[agent_id]) for agent_id in self.agent_radii)

    def control_for(self, agent_id: int, position: ArrayLike) -> Vector2:
        """Calcola il contributo repulsivo totale per un singolo agente."""
        if agent_id not in self.agent_radii:
            raise KeyError(f"Raggio sconosciuto per l'agente {agent_id}.")
        agent_position = as_vector2(position, name="position")
        control = np.zeros(2, dtype=float)

        for obstacle in self.obstacles:
            displacement = agent_position - obstacle.position
            center_distance = float(np.linalg.norm(displacement))
            clearance = center_distance - self.agent_radii[agent_id] - obstacle.radius
            if clearance >= self.influence_distance:
                continue

            # Il clipping mantiene il campo numericamente finito anche in caso
            # di compenetrazione iniziale accidentale.
            safe_clearance = max(clearance, self.minimum_clearance)
            if center_distance <= 1e-12:
                # Direzione deterministica nel caso singolare agente-centro ostacolo.
                direction = np.array((1.0, 0.0))
            else:
                direction = displacement / center_distance
            magnitude = self.gain * (1.0 / safe_clearance - 1.0 / self.influence_distance) / safe_clearance**2
            control += magnitude * direction

        return control

    def controls(self, positions: Mapping[int, ArrayLike]) -> dict[int, Vector2]:
        """Calcola i controlli repulsivi per tutti gli agenti noti."""
        missing = set(self.agent_radii) - set(positions)
        if missing:
            raise ValueError(f"Mancano le posizioni degli agenti: {missing}.")
        return {agent_id: self.control_for(agent_id, positions[agent_id]) for agent_id in self.agent_radii}


class StaticObstacleAvoidanceController(PositionBasedObstacleAvoidanceController):
    """Versione position-based limitata esplicitamente a ostacoli immobili."""

    def __post_init__(self) -> None:
        super().__post_init__()
        if any(not np.allclose(obstacle.velocity, 0.0) for obstacle in self.obstacles):
            raise ValueError("StaticObstacleAvoidanceController accetta soltanto ostacoli fissi.")
