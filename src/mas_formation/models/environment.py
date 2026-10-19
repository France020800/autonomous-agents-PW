"""Ambiente di simulazione e aggiornamento sincrono dei modelli."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import ArrayLike

from ._vectors import Vector2, as_vector2
from .agent import Agent
from .obstacle import DynamicObstacle


@dataclass
class Environment:
    """Contiene entità e applica ad esse un passo temporale atomico.

    I controlli sono calcolati sullo stesso stato iniziale e solo dopo vengono
    aggiornati agenti e ostacoli. Questa scelta evita dipendenze dall'ordine
    degli agenti nella lista.
    """

    agents: Sequence[Agent]
    obstacles: Sequence[DynamicObstacle] = field(default_factory=tuple)
    lower_bound: ArrayLike | None = None
    upper_bound: ArrayLike | None = None
    time: float = field(default=0.0, init=False)

    def __post_init__(self) -> None:
        self.agents = list(self.agents)
        self.obstacles = list(self.obstacles)
        self._validate_identifiers()
        self._validate_bounds()

    def _validate_identifiers(self) -> None:
        agent_ids = [agent.identifier for agent in self.agents]
        obstacle_ids = [obstacle.identifier for obstacle in self.obstacles]
        if len(agent_ids) != len(set(agent_ids)):
            raise ValueError("Gli identificativi degli agenti devono essere univoci.")
        if len(obstacle_ids) != len(set(obstacle_ids)):
            raise ValueError("Gli identificativi degli ostacoli devono essere univoci.")

    def _validate_bounds(self) -> None:
        if (self.lower_bound is None) != (self.upper_bound is None):
            raise ValueError("Specificare entrambi i limiti oppure nessuno dei due.")
        if self.lower_bound is None:
            return
        self.lower_bound = as_vector2(self.lower_bound, name="lower_bound")
        self.upper_bound = as_vector2(self.upper_bound, name="upper_bound")
        if np.any(self.lower_bound >= self.upper_bound):
            raise ValueError("lower_bound deve essere minore di upper_bound componente per componente.")

    @property
    def has_bounds(self) -> bool:
        return self.lower_bound is not None

    def step(self, controls: Mapping[int, ArrayLike], dt: float) -> None:
        """Avanza il sistema di ``dt`` secondi mediante Eulero esplicito.

        È richiesto esattamente un controllo per ogni agente. Gli ostacoli non
        sono controllati dal chiamante: evolvono con la loro velocità definita.
        """
        if dt <= 0 or not np.isfinite(dt):
            raise ValueError("dt deve essere finito e strettamente positivo.")

        expected_ids = {agent.identifier for agent in self.agents}
        received_ids = set(controls)
        if received_ids != expected_ids:
            missing = expected_ids - received_ids
            unknown = received_ids - expected_ids
            raise ValueError(f"Controlli non validi: mancanti={missing}, sconosciuti={unknown}.")

        agent_updates = [
            (agent, *agent.propose_step(controls[agent.identifier], dt))
            for agent in self.agents
        ]
        obstacle_updates = [(obstacle, obstacle.propose_step(dt)) for obstacle in self.obstacles]

        self._validate_proposed_bounds(agent_updates, obstacle_updates)
        for agent, next_position, applied_control in agent_updates:
            agent.commit_step(next_position, applied_control)
        for obstacle, next_position in obstacle_updates:
            obstacle.commit_step(next_position)
        self.time += dt

    def _validate_proposed_bounds(
        self,
        agent_updates: list[tuple[Agent, Vector2, Vector2]],
        obstacle_updates: list[tuple[DynamicObstacle, Vector2]],
    ) -> None:
        if not self.has_bounds:
            return
        assert self.lower_bound is not None and self.upper_bound is not None
        for entity, position, *rest in [*agent_updates, *obstacle_updates]:
            if np.any(position < self.lower_bound) or np.any(position > self.upper_bound):
                raise ValueError(f"{type(entity).__name__} {entity.identifier} uscirebbe dai limiti dell'ambiente.")
