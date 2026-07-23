from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np


Array = np.ndarray
Excitation = Callable[[float], float]


def harmonic_force(
    amplitude: float,
    angular_frequency: float,
    phase: float = 0.0,
) -> Excitation:
    """Create f(t) = amplitude * sin(angular_frequency * t + phase)."""

    if angular_frequency < 0.0:
        raise ValueError("angular_frequency must be non-negative")

    def evaluate(t: float) -> float:
        return float(amplitude * np.sin(angular_frequency * t + phase))

    return evaluate


@dataclass(frozen=True)
class LinearOscillator:
    """Single-degree-of-freedom linear mass-spring-damper model."""

    mass: float
    stiffness: float
    damping: float = 0.0
    excitation: Excitation | None = None

    def __post_init__(self) -> None:
        if self.mass <= 0.0:
            raise ValueError("mass must be positive")
        if self.stiffness <= 0.0:
            raise ValueError("stiffness must be positive")
        if self.damping < 0.0:
            raise ValueError("damping must be non-negative")

    @property
    def natural_frequency(self) -> float:
        """Return the undamped natural angular frequency in rad/s."""

        return float(np.sqrt(self.stiffness / self.mass))

    @property
    def critical_damping(self) -> float:
        """Return the critical viscous damping coefficient."""

        return float(2.0 * np.sqrt(self.mass * self.stiffness))

    @property
    def damping_ratio(self) -> float:
        """Return damping divided by critical damping."""

        return self.damping / self.critical_damping

    def rhs(self) -> Callable[[float, Array], Array]:
        """Return y_dot for y = [q, q_dot]."""

        def evaluate(t: float, y: Array) -> Array:
            if y.shape != (2,):
                raise ValueError("y must have shape (2,)")

            q, q_dot = y
            external_force = 0.0 if self.excitation is None else self.excitation(t)
            q_ddot = (
                external_force - self.damping * q_dot - self.stiffness * q
            ) / self.mass
            return np.array([q_dot, q_ddot], dtype=float)

        return evaluate

    def exact_undamped_state(self, t: Array, y0: Array) -> Array:
        """Evaluate the exact state for an undamped oscillator starting at t=0."""

        if self.damping != 0.0 or self.excitation is not None:
            raise ValueError("exact_undamped_state requires free undamped motion")
        if y0.shape != (2,):
            raise ValueError("y0 must have shape (2,)")

        time = np.asarray(t, dtype=float)
        q0, q_dot0 = y0
        omega_n = self.natural_frequency
        phase = omega_n * time
        q = q0 * np.cos(phase) + (q_dot0 / omega_n) * np.sin(phase)
        q_dot = -q0 * omega_n * np.sin(phase) + q_dot0 * np.cos(phase)
        return np.column_stack((q, q_dot))

    def harmonic_steady_state_amplitude(
        self,
        force_amplitude: float,
        angular_frequency: Array,
    ) -> Array:
        """Return displacement amplitude under harmonic force excitation."""

        frequency = np.asarray(angular_frequency, dtype=float)
        if np.any(frequency < 0.0):
            raise ValueError("angular_frequency must be non-negative")

        dynamic_stiffness = np.sqrt(
            (self.stiffness - self.mass * frequency * frequency) ** 2
            + (self.damping * frequency) ** 2
        )
        amplitude = np.full_like(dynamic_stiffness, np.inf, dtype=float)
        return np.divide(
            abs(force_amplitude),
            dynamic_stiffness,
            out=amplitude,
            where=dynamic_stiffness > np.finfo(float).eps,
        )
