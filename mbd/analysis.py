from __future__ import annotations

import numpy as np


Array = np.ndarray


def particle_mechanical_energy(
    mass: float,
    position: Array,
    velocity: Array,
    g: float = 9.81,
) -> Array:
    """Mechanical energy for a point mass in a uniform gravity field."""

    kinetic = 0.5 * mass * np.sum(velocity * velocity, axis=1)
    potential = mass * g * position[:, 2]
    return kinetic + potential


def relative_drift(values: Array) -> Array:
    """Relative drift from the first sample, robust to zero initial values."""

    scale = max(abs(float(values[0])), 1.0)
    return (values - values[0]) / scale
