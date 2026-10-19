"""Modello di un agente puntiforme con dinamica a singolo integratore."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from numpy.typing import ArrayLike

from ._vectors import Vector2, as_vector2


@dataclass
class Agent:
    """Agente planare soggetto alla dinamica ``p_dot = u``.

    ``velocity`` non è uno stato dinamico indipendente: memorizza il comando
    effettivamente applicato all'ultimo passo di simulazione.
    """

    identifier: int
    position: ArrayLike
    radius: float = 0.15
    max_speed: float = 1.0
    velocity: Vector2 = field(default_factory=lambda: np.zeros(2, dtype=float), init=False)

    def __post_init__(self) -> None:
        self.position = as_vector2(self.position, name="position")
        if self.radius <= 0 or not np.isfinite(self.radius):
            raise ValueError("radius deve essere finito e strettamente positivo.")
        if self.max_speed <= 0 or not np.isfinite(self.max_speed):
            raise ValueError("max_speed deve essere finito e strettamente positivo.")

    def saturate_control(self, control: ArrayLike) -> Vector2:
        """Limita un comando alla velocità massima senza cambiarne la direzione."""
        control_vector = as_vector2(control, name="control")
        norm = float(np.linalg.norm(control_vector))
        if norm > self.max_speed:
            control_vector *= self.max_speed / norm
        return control_vector

    def propose_step(self, control: ArrayLike, dt: float) -> tuple[Vector2, Vector2]:
        """Calcola lo stato al passo successivo senza modificare l'agente."""
        if dt <= 0 or not np.isfinite(dt):
            raise ValueError("dt deve essere finito e strettamente positivo.")
        applied_control = self.saturate_control(control)
        return self.position + dt * applied_control, applied_control

    def commit_step(self, position: ArrayLike, applied_control: ArrayLike) -> None:
        """Applica uno stato già validato dall'ambiente."""
        self.position = as_vector2(position, name="position")
        self.velocity = as_vector2(applied_control, name="applied_control")
