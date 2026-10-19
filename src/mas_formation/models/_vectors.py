"""Utilità interne per validare e copiare vettori planari."""

from __future__ import annotations

from typing import Sequence

import numpy as np
from numpy.typing import ArrayLike, NDArray

Vector2 = NDArray[np.float64]


def as_vector2(value: ArrayLike | Sequence[float], *, name: str) -> Vector2:
    """Restituisce una copia di ``value`` come vettore finito di dimensione 2."""
    vector = np.asarray(value, dtype=float)
    if vector.shape != (2,):
        raise ValueError(f"{name} deve avere forma (2,), ricevuta {vector.shape}.")
    if not np.all(np.isfinite(vector)):
        raise ValueError(f"{name} deve contenere soltanto valori finiti.")
    return vector.copy()
