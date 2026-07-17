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

    reference = abs(float(values[0]))
    scale = reference if reference > 1e-12 else 1.0
    return (values - values[0]) / scale


def oscillator_mechanical_energy(
    mass: float,
    stiffness: float,
    displacement: Array,
    velocity: Array,
) -> Array:
    """Mechanical energy of a linear undamped oscillator."""

    kinetic = 0.5 * mass * velocity * velocity
    elastic = 0.5 * stiffness * displacement * displacement
    return kinetic + elastic
