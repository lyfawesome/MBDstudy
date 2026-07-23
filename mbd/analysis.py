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
    """Mechanical energy stored by a linear oscillator."""

    kinetic = 0.5 * mass * velocity * velocity
    elastic = 0.5 * stiffness * displacement * displacement
    return kinetic + elastic


def cumulative_trapezoid(values: Array, time: Array) -> Array:
    """Integrate sampled values over time with the trapezoidal rule."""

    sampled_values = np.asarray(values, dtype=float)
    sampled_time = np.asarray(time, dtype=float)
    if sampled_values.ndim != 1 or sampled_time.ndim != 1:
        raise ValueError("values and time must be one-dimensional")
    if sampled_values.shape != sampled_time.shape:
        raise ValueError("values and time must have the same shape")
    if sampled_time.size == 0:
        raise ValueError("values and time must not be empty")

    time_step = np.diff(sampled_time)
    if np.any(time_step <= 0.0):
        raise ValueError("time must be strictly increasing")

    increments = 0.5 * (sampled_values[1:] + sampled_values[:-1]) * time_step
    return np.concatenate((np.array([0.0]), np.cumsum(increments)))
