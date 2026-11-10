"""Evitamento di ostacoli circolari statici e mobili tramite potenziale repulsivo."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import ArrayLike, NDArray

from mas_formation.models._vectors import Vector2, as_vector2
from mas_formation.models.obstacle import DynamicObstacle


@dataclass(frozen=True)
class PositionBasedObstacleAvoidanceController:
    """Genera repulsione APF dalla posizione istantanea di ostacoli circolari.

    Per la clearance ``rho = ||p_i-q_j|| - r_i-r_j`` usa il potenziale
    ``V_o = gain/2 * (1/rho - 1/influence_distance)^2`` entro il raggio
    d'influenza e zero al di fuori.
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

    def _effective_obstacle_position(
        self, agent_position: Vector2, obstacle: DynamicObstacle
    ) -> tuple[Vector2, float]:
        """Restituisce posizione efficace dell'ostacolo e distanza centro-centro."""
        center_distance = float(np.linalg.norm(agent_position - obstacle.position))
        return obstacle.position, center_distance

    def potential_for(self, agent_id: int, position: ArrayLike) -> float:
        """Valore del potenziale repulsivo per un singolo agente."""
        if agent_id not in self.agent_radii:
            raise KeyError(f"Raggio sconosciuto per l'agente {agent_id}.")
        agent_position = as_vector2(position, name="position")
        potential = 0.0
        for obstacle in self.obstacles:
            _, center_distance = self._effective_obstacle_position(agent_position, obstacle)
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
            effective_position, center_distance = self._effective_obstacle_position(agent_position, obstacle)
            displacement = agent_position - effective_position
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


@dataclass(frozen=True)
class PredictiveObstacleAvoidanceController(PositionBasedObstacleAvoidanceController):
    """Genera repulsione APF considerando la posizione attuale e quelle future.

    Conoscendo la velocità costante ``v_j`` di ciascun ostacolo, predice le
    posizioni ``q_j + tau_m * v_j`` su una griglia uniforme nell'orizzonte
    ``[0, prediction_horizon]`` (inclusa la posizione attuale ``tau_0 = 0``).
    Per ogni coppia agente-ostacolo seleziona la distanza minore tra quella
    attuale e quelle future e valuta potenziale e controllo repulsivo rispetto
    al punto predetto più vicino.
    """

    prediction_horizon: float = 1.5
    prediction_steps: int = 10
    _prediction_times: NDArray[np.float64] = field(init=False, repr=False)

    def __post_init__(self) -> None:
        super().__post_init__()
        if not np.isfinite(self.prediction_horizon) or self.prediction_horizon < 0.0:
            raise ValueError("prediction_horizon deve essere finito e non negativo.")
        if isinstance(self.prediction_steps, bool) or not isinstance(self.prediction_steps, int) or self.prediction_steps < 1:
            raise ValueError("prediction_steps deve essere un intero maggiore o uguale a 1.")
        if self.prediction_horizon == 0.0:
            prediction_times = np.array((0.0,), dtype=float)
        else:
            prediction_times = np.linspace(0.0, self.prediction_horizon, self.prediction_steps + 1, dtype=float)
        object.__setattr__(self, "_prediction_times", prediction_times)

    def _effective_obstacle_position(
        self, agent_position: Vector2, obstacle: DynamicObstacle
    ) -> tuple[Vector2, float]:
        if self.prediction_horizon == 0.0 or np.allclose(obstacle.velocity, 0.0):
            return super()._effective_obstacle_position(agent_position, obstacle)

        predicted_positions = (
            obstacle.position[np.newaxis, :]
            + self._prediction_times[:, np.newaxis] * obstacle.velocity[np.newaxis, :]
        )
        distances = np.linalg.norm(agent_position[np.newaxis, :] - predicted_positions, axis=1)
        closest_index = int(np.argmin(distances))
        return predicted_positions[closest_index], float(distances[closest_index])

    def closest_predicted_position(self, position: ArrayLike, obstacle: DynamicObstacle) -> Vector2:
        """Restituisce la posizione (attuale o futura) dell'ostacolo a distanza minima."""
        agent_position = as_vector2(position, name="position")
        effective_position, _ = self._effective_obstacle_position(agent_position, obstacle)
        return effective_position.copy()


class StaticObstacleAvoidanceController(PositionBasedObstacleAvoidanceController):
    """Versione position-based limitata esplicitamente a ostacoli immobili."""

    def __post_init__(self) -> None:
        super().__post_init__()
        if any(not np.allclose(obstacle.velocity, 0.0) for obstacle in self.obstacles):
            raise ValueError("StaticObstacleAvoidanceController accetta soltanto ostacoli fissi.")
