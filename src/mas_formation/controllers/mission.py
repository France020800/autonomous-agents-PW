"""Supervisore per la sequenza: raggiungi la formazione, poi naviga al goal."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import ArrayLike

from .formation import FormationController
from .navigation import GoalNavigationController, random_goal_from_centroid
from .obstacle_avoidance import PositionBasedObstacleAvoidanceController
from mas_formation.models._vectors import Vector2


@dataclass
class FormationThenGoalController:
    """Attiva la navigazione soltanto dopo il raggiungimento della formazione.

    Il goal viene estratto una sola volta nell'istante di attivazione su una
    circonferenza centrata nel baricentro corrente. La sua distanza dal
    baricentro al momento dell'estrazione è quindi esattamente ``goal_distance``.
    """

    formation_controller: FormationController
    goal_distance: float
    formation_tolerance: float = 1e-3
    navigation_gain: float = 0.6
    goal_tolerance: float = 0.1
    goal_huber_delta: float = 0.05
    seed: int | None = None
    goal_angle: float | None = None
    obstacle_controller: PositionBasedObstacleAvoidanceController | None = None
    _goal_controller: GoalNavigationController | None = field(default=None, init=False, repr=False)
    _generator: np.random.Generator = field(init=False, repr=False)

    def __post_init__(self) -> None:
        if not np.isfinite(self.goal_distance) or self.goal_distance <= 0:
            raise ValueError("goal_distance deve essere finita e strettamente positiva.")
        if not np.isfinite(self.formation_tolerance) or self.formation_tolerance <= 0:
            raise ValueError("formation_tolerance deve essere finita e strettamente positiva.")
        if not np.isfinite(self.navigation_gain) or self.navigation_gain <= 0:
            raise ValueError("navigation_gain deve essere finito e strettamente positivo.")
        if not np.isfinite(self.goal_tolerance) or self.goal_tolerance <= 0:
            raise ValueError("goal_tolerance deve essere finita e strettamente positiva.")
        if not np.isfinite(self.goal_huber_delta) or self.goal_huber_delta <= 0:
            raise ValueError("goal_huber_delta deve essere finita e strettamente positiva.")
        if self.goal_angle is not None and not np.isfinite(self.goal_angle):
            raise ValueError("goal_angle deve essere finito oppure None.")
        if self.obstacle_controller is not None:
            formation_ids = set(self.formation_controller.formation.agent_ids)
            if set(self.obstacle_controller.agent_radii) != formation_ids:
                raise ValueError("Gli agenti del controllore ostacoli devono coincidere con quelli della formazione.")
        self._generator = np.random.default_rng(self.seed)

    @property
    def navigation_active(self) -> bool:
        """È vero dopo che la formazione è stata raggiunta almeno una volta."""
        return self._goal_controller is not None

    @property
    def goal(self) -> Vector2 | None:
        """Goal estratto, oppure ``None`` finché la navigazione non è attiva."""
        return None if self._goal_controller is None else self._goal_controller.goal.copy()

    def has_reached_goal(self, positions: Mapping[int, ArrayLike]) -> bool:
        """Indica se il baricentro è nella regione di arrivo, dopo l'attivazione."""
        return self._goal_controller is not None and self._goal_controller.has_reached_goal(positions)

    def goal_potential(self, positions: Mapping[int, ArrayLike]) -> float | None:
        """Restituisce ``V_g`` dopo l'attivazione, altrimenti ``None``."""
        return None if self._goal_controller is None else self._goal_controller.potential(positions)

    def controls(self, positions: Mapping[int, ArrayLike]) -> dict[int, Vector2]:
        """Combina formazione e, quando consentito, navigazione verso il goal."""
        formation_controls = self.formation_controller.controls(positions)
        obstacle_controls = (
            {identifier: np.zeros(2, dtype=float) for identifier in formation_controls}
            if self.obstacle_controller is None
            else self.obstacle_controller.controls(positions)
        )
        base_controls = {
            identifier: formation_controls[identifier] + obstacle_controls[identifier]
            for identifier in formation_controls
        }
        if self._goal_controller is None:
            formation = self.formation_controller.formation
            if not formation.is_satisfied(positions, tolerance=self.formation_tolerance):
                return base_controls

            agent_ids = formation.agent_ids
            centroid = np.mean([positions[identifier] for identifier in agent_ids], axis=0)
            goal = (
                random_goal_from_centroid(centroid, self.goal_distance, self._generator)
                if self.goal_angle is None
                else centroid + self.goal_distance * np.array((np.cos(self.goal_angle), np.sin(self.goal_angle)))
            )
            self._goal_controller = GoalNavigationController(
                agent_ids=agent_ids,
                goal=goal,
                gain=self.navigation_gain,
                arrival_tolerance=self.goal_tolerance,
                huber_delta=self.goal_huber_delta,
            )

        navigation_controls = self._goal_controller.controls(positions)
        return {
            identifier: base_controls[identifier] + navigation_controls[identifier]
            for identifier in formation_controls
        }


@dataclass(frozen=True)
class FormationWithObstacleAvoidanceController:
    """Mantiene una formazione aggiungendo la repulsione dagli ostacoli.

    Non contiene un termine di navigazione: in assenza di ostacoli, una
    formazione già corretta rimane immobile nel proprio equilibrio.
    """

    formation_controller: FormationController
    obstacle_controller: PositionBasedObstacleAvoidanceController

    def __post_init__(self) -> None:
        formation_ids = set(self.formation_controller.formation.agent_ids)
        if set(self.obstacle_controller.agent_radii) != formation_ids:
            raise ValueError("Gli agenti del controllore ostacoli devono coincidere con quelli della formazione.")

    def controls(self, positions: Mapping[int, ArrayLike]) -> dict[int, Vector2]:
        """Somma il controllo di formazione e quello reattivo agli ostacoli."""
        formation_controls = self.formation_controller.controls(positions)
        obstacle_controls = self.obstacle_controller.controls(positions)
        return {
            identifier: formation_controls[identifier] + obstacle_controls[identifier]
            for identifier in formation_controls
        }
