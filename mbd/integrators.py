from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np


Array = np.ndarray
Derivative = Callable[[float, Array], Array]


@dataclass(frozen=True)
class TimeHistory:
    """Time samples and corresponding state samples."""

    t: Array
    y: Array


def rk4_step(rhs: Derivative, t: float, y: Array, dt: float) -> Array:
    """Advance one step with the classical fourth-order Runge-Kutta method."""

    k1 = rhs(t, y)
    k2 = rhs(t + 0.5 * dt, y + 0.5 * dt * k1)
    k3 = rhs(t + 0.5 * dt, y + 0.5 * dt * k2)
    k4 = rhs(t + dt, y + dt * k3)
    return y + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


def integrate_fixed_step(
    rhs: Derivative,
    y0: Array,
    t0: float,
    tf: float,
    dt: float,
) -> TimeHistory:
    """Integrate an initial value problem with a fixed time step."""

    if dt <= 0.0:
        raise ValueError("dt must be positive")
    if tf <= t0:
        raise ValueError("tf must be greater than t0")

    n_steps = int(np.ceil((tf - t0) / dt))
    t = np.empty(n_steps + 1, dtype=float)
    y = np.empty((n_steps + 1, y0.size), dtype=float)

    t[0] = t0
    y[0] = y0

    current_t = t0
    current_y = y0.astype(float, copy=True)
    for i in range(1, n_steps + 1):
        step = min(dt, tf - current_t)
        current_y = rk4_step(rhs, current_t, current_y, step)
        current_t += step
        t[i] = current_t
        y[i] = current_y

    return TimeHistory(t=t, y=y)
