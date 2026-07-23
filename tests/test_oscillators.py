from __future__ import annotations

import unittest

import numpy as np

from mbd.analysis import cumulative_trapezoid
from mbd.oscillators import LinearOscillator, harmonic_force


class LinearOscillatorTests(unittest.TestCase):
    def test_default_model_preserves_undamped_equation(self) -> None:
        oscillator = LinearOscillator(mass=2.0, stiffness=8.0)

        derivative = oscillator.rhs()(0.0, np.array([3.0, 4.0]))

        np.testing.assert_allclose(derivative, np.array([4.0, -12.0]))

    def test_rhs_includes_damping_and_excitation(self) -> None:
        oscillator = LinearOscillator(
            mass=2.0,
            stiffness=4.0,
            damping=3.0,
            excitation=lambda t: 4.0 * t,
        )

        derivative = oscillator.rhs()(0.5, np.array([1.0, 2.0]))

        np.testing.assert_allclose(derivative, np.array([2.0, -4.0]))

    def test_harmonic_response_matches_static_and_resonant_limits(self) -> None:
        oscillator = LinearOscillator(mass=2.0, stiffness=8.0, damping=1.0)
        frequency = np.array([0.0, oscillator.natural_frequency])

        amplitude = oscillator.harmonic_steady_state_amplitude(4.0, frequency)

        expected = np.array([0.5, 4.0 / oscillator.natural_frequency])
        np.testing.assert_allclose(amplitude, expected)

    def test_exact_solution_rejects_damped_model(self) -> None:
        oscillator = LinearOscillator(mass=2.0, stiffness=8.0, damping=0.2)

        with self.assertRaises(ValueError):
            oscillator.exact_undamped_state(
                np.array([0.0, 1.0]),
                np.array([1.0, 0.0]),
            )

    def test_invalid_parameters_are_rejected(self) -> None:
        with self.assertRaises(ValueError):
            LinearOscillator(mass=1.0, stiffness=1.0, damping=-0.1)
        with self.assertRaises(ValueError):
            harmonic_force(amplitude=1.0, angular_frequency=-1.0)


class CumulativeTrapezoidTests(unittest.TestCase):
    def test_integrates_linear_samples(self) -> None:
        time = np.array([0.0, 1.0, 2.0])
        values = np.array([0.0, 1.0, 2.0])

        integral = cumulative_trapezoid(values, time)

        np.testing.assert_allclose(integral, np.array([0.0, 0.5, 2.0]))


if __name__ == "__main__":
    unittest.main()
