from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np


Array = np.ndarray
ForceLaw = Callable[[float, Array, Array], Array]


@dataclass(frozen=True)
class Particle:
    """Point mass model in three-dimensional Cartesian coordinates."""

    mass: float

    def __post_init__(self) -> None:
        if self.mass <= 0.0:
            raise ValueError("mass must be positive")

    def rhs(self, force: ForceLaw) -> Callable[[float, Array], Array]:
        """Return y_dot = [r_dot, v_dot] for y = [r, v]."""

        def evaluate(t: float, y: Array) -> Array:
            position = y[:3]
            velocity = y[3:]
            acceleration = force(t, position, velocity) / self.mass
            return np.concatenate((velocity, acceleration))

        return evaluate


def constant_gravity(mass: float, g: float = 9.81) -> ForceLaw:
    """Create a constant gravity force law acting in negative z direction."""

    gravity_force = np.array([0.0, 0.0, -mass * g], dtype=float)

    def force(_t: float, _position: Array, _velocity: Array) -> Array:
        return gravity_force

    return force
