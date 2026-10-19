"""Controllo distribuito di formazione basato sul gradiente di un potenziale."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike

from mas_formation.models._vectors import Vector2, as_vector2
from mas_formation.models.formation import TriangularFormation


@dataclass(frozen=True)
class FormationController:
    """Controllore a discesa del gradiente per un grafo di formazione.

    Per ogni arco ``(i, j)`` del grafo è definito il potenziale

    ``V_ij = gain / 4 * (||p_i - p_j||^2 - d_ij^2)^2``.

    Il comando locale è il negativo del gradiente della somma dei soli archi
    incidenti al nodo ``i``. Di conseguenza, per calcolarlo ciascun agente
    richiede esclusivamente posizione propria e posizioni dei vicini.
    """

    formation: TriangularFormation
    gain: float = 1.0

    def __post_init__(self) -> None:
        if not np.isfinite(self.gain) or self.gain <= 0:
            raise ValueError("gain deve essere finito e strettamente positivo.")

    def neighbors(self, agent_id: int) -> tuple[int, ...]:
        """Restituisce gli agenti connessi ad ``agent_id`` nel grafo."""
        if agent_id not in self.formation.agent_ids:
            raise KeyError(f"L'agente {agent_id} non appartiene alla formazione.")
        return tuple(
            edge.second_agent if edge.first_agent == agent_id else edge.first_agent
            for edge in self.formation.edges
            if agent_id in edge.key
        )

    def potential(self, positions: Mapping[int, ArrayLike]) -> float:
        """Valore totale del potenziale di formazione ``V_f``."""
        missing = set(self.formation.agent_ids) - set(positions)
        if missing:
            raise ValueError(f"Mancano le posizioni degli agenti: {missing}.")
        vectors = {
            identifier: as_vector2(positions[identifier], name=f"positions[{identifier}]")
            for identifier in self.formation.agent_ids
        }
        return 0.25 * self.gain * sum(
            (float(np.sum((vectors[edge.first_agent] - vectors[edge.second_agent]) ** 2))
             - edge.desired_distance**2) ** 2
            for edge in self.formation.edges
        )

    def control_for(
        self,
        agent_id: int,
        own_position: ArrayLike,
        neighbor_positions: Mapping[int, ArrayLike],
    ) -> Vector2:
        """Calcola il controllo locale ``u_i = -∇_{p_i} V_f``.

        ``neighbor_positions`` può contenere altri agenti, ma deve includere
        tutti i vicini di ``agent_id`` nel grafo di formazione.
        """
        own = as_vector2(own_position, name="own_position")
        result = np.zeros(2, dtype=float)

        for edge in self.formation.edges:
            if edge.first_agent == agent_id:
                neighbor_id = edge.second_agent
            elif edge.second_agent == agent_id:
                neighbor_id = edge.first_agent
            else:
                continue

            if neighbor_id not in neighbor_positions:
                raise ValueError(f"Manca la posizione del vicino {neighbor_id} dell'agente {agent_id}.")
            neighbor = as_vector2(neighbor_positions[neighbor_id], name=f"neighbor_positions[{neighbor_id}]")
            displacement = own - neighbor
            squared_error = float(displacement @ displacement - edge.desired_distance**2)
            result -= self.gain * squared_error * displacement

        return result

    def controls(self, positions: Mapping[int, ArrayLike]) -> dict[int, Vector2]:
        """Calcola in parallelo i controlli locali per tutti gli agenti."""
        missing = set(self.formation.agent_ids) - set(positions)
        if missing:
            raise ValueError(f"Mancano le posizioni degli agenti: {missing}.")
        return {
            agent_id: self.control_for(agent_id, positions[agent_id], positions)
            for agent_id in self.formation.agent_ids
        }
