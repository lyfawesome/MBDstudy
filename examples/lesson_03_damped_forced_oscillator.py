from __future__ import annotations

from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import numpy as np

from mbd.analysis import cumulative_trapezoid, oscillator_mechanical_energy
from mbd.integrators import integrate_fixed_step
from mbd.oscillators import LinearOscillator, harmonic_force
from mbd.plotting import Curve, save_stacked_svg


def main() -> None:
    mass = 2.0
    stiffness = 50.0
    damping_ratio = 0.05
    force_amplitude = 1.5

    undamped = LinearOscillator(mass=mass, stiffness=stiffness)
    natural_frequency = undamped.natural_frequency
    forcing_frequency = 0.9 * natural_frequency
    damping = damping_ratio * undamped.critical_damping
    excitation = harmonic_force(force_amplitude, forcing_frequency)
    oscillator = LinearOscillator(
        mass=mass,
        stiffness=stiffness,
        damping=damping,
        excitation=excitation,
    )

    forcing_period = 2.0 * np.pi / forcing_frequency
    y0 = np.array([0.0, 0.0], dtype=float)
    history = integrate_fixed_step(
        oscillator.rhs(),
        y0,
        t0=0.0,
        tf=30.0 * forcing_period,
        dt=forcing_period / 200.0,
    )

    q = history.y[:, 0]
    q_dot = history.y[:, 1]
    force = np.array([excitation(float(t)) for t in history.t])
    energy = oscillator_mechanical_energy(mass, stiffness, q, q_dot)
    input_work = cumulative_trapezoid(force * q_dot, history.t)
    dissipated_energy = cumulative_trapezoid(damping * q_dot * q_dot, history.t)
    energy_change = energy - energy[0]
    energy_balance = input_work - dissipated_energy
    balance_residual = energy_change - energy_balance

    steady_amplitude = float(
        oscillator.harmonic_steady_state_amplitude(
            force_amplitude,
            np.array([forcing_frequency]),
        )[0]
    )
    phase_lag = np.arctan2(
        damping * forcing_frequency,
        stiffness - mass * forcing_frequency**2,
    )
    steady_state = steady_amplitude * np.sin(
        forcing_frequency * history.t - phase_lag
    )

    steady_mask = history.t >= history.t[-1] - 5.0 * forcing_period
    numerical_amplitude = 0.5 * (
        np.max(q[steady_mask]) - np.min(q[steady_mask])
    )

    frequency_ratio = np.linspace(0.1, 2.0, 400)
    response_amplitude = oscillator.harmonic_steady_state_amplitude(
        force_amplitude,
        frequency_ratio * natural_frequency,
    )
    static_displacement = force_amplitude / stiffness
    magnification = response_amplitude / static_displacement

    output_dir = Path("outputs")
    output_dir.mkdir(exist_ok=True)
    figure_path = output_dir / "lesson_03_damped_forced_oscillator.svg"
    save_stacked_svg(
        figure_path,
        panels=[
            [
                Curve(history.t, q, "RK4", "#1f77b4"),
                Curve(history.t, steady_state, "steady state", "#d62728"),
            ],
            [Curve(q, q_dot, "phase trajectory", "#2ca02c")],
            [
                Curve(history.t, energy_change, "energy change", "#9467bd"),
                Curve(history.t, energy_balance, "work - dissipation", "#ff7f0e"),
            ],
            [Curve(frequency_ratio, magnification, "frequency response", "#c0392b")],
        ],
        titles=[
            "Transient and Steady Response",
            "Phase Portrait",
            "Energy Balance",
            "Harmonic Frequency Response",
        ],
        x_labels=[
            "time [s]",
            "q [m]",
            "time [s]",
            "forcing frequency / natural frequency",
        ],
        y_labels=[
            "q [m]",
            "q_dot [m/s]",
            "energy [J]",
            "amplitude / static displacement",
        ],
    )

    amplitude_error = abs(numerical_amplitude - steady_amplitude) / steady_amplitude
    print(f"natural angular frequency: {natural_frequency:.6f} rad/s")
    print(f"damping ratio: {oscillator.damping_ratio:.6f}")
    print(f"forcing frequency ratio: {forcing_frequency / natural_frequency:.6f}")
    print(f"theoretical steady amplitude: {steady_amplitude:.6e} m")
    print(f"numerical steady amplitude: {numerical_amplitude:.6e} m")
    print(f"relative amplitude error: {amplitude_error:.3e}")
    print(f"max energy-balance residual: {np.max(np.abs(balance_residual)):.3e} J")
    print(f"saved {figure_path}")


if __name__ == "__main__":
    main()
