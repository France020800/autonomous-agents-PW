"""Modello di ostacolo circolare, statico o a velocità costante."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike

from ._vectors import Vector2, as_vector2


@dataclass
class DynamicObstacle:
    """Ostacolo circolare con dinamica ``q_dot = velocity``.

    Per ottenere un ostacolo statico è sufficiente usare la velocità nulla,
    valore predefinito del modello.
    """

    identifier: int
    position: ArrayLike
    radius: float = 0.3
    velocity: ArrayLike = (0.0, 0.0)

    def __post_init__(self) -> None:
        self.position = as_vector2(self.position, name="position")
        self.velocity = as_vector2(self.velocity, name="velocity")
        if self.radius <= 0 or not np.isfinite(self.radius):
            raise ValueError("radius deve essere finito e strettamente positivo.")

    def propose_step(self, dt: float) -> Vector2:
        """Calcola la posizione futura con integrazione di Eulero."""
        if dt <= 0 or not np.isfinite(dt):
            raise ValueError("dt deve essere finito e strettamente positivo.")
        return self.position + dt * self.velocity

    def commit_step(self, position: ArrayLike) -> None:
        """Applica una posizione futura già validata."""
        self.position = as_vector2(position, name="position")
