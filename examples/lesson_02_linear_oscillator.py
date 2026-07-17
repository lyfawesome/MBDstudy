from __future__ import annotations

from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import numpy as np

from mbd.analysis import oscillator_mechanical_energy, relative_drift
from mbd.integrators import integrate_fixed_step
from mbd.oscillators import LinearOscillator
from mbd.plotting import Curve, save_stacked_svg


def convergence_study(
    oscillator: LinearOscillator,
    y0: np.ndarray,
    period: float,
) -> tuple[np.ndarray, np.ndarray, float]:
    steps_per_period = np.array([10, 20, 40, 80], dtype=int)
    step_sizes = period / steps_per_period
    errors = np.empty_like(step_sizes)
    exact_final = oscillator.exact_undamped_state(np.array([period]), y0)[0]

    for i, dt in enumerate(step_sizes):
        history = integrate_fixed_step(
            oscillator.rhs(),
            y0,
            t0=0.0,
            tf=period,
            dt=float(dt),
        )
        errors[i] = np.linalg.norm(history.y[-1] - exact_final)

    observed_order = float(np.polyfit(np.log(step_sizes), np.log(errors), 1)[0])
    return step_sizes, errors, observed_order


def main() -> None:
    mass = 2.0
    stiffness = 50.0
    oscillator = LinearOscillator(mass=mass, stiffness=stiffness)

    q0 = 0.08
    q_dot0 = 0.0
    y0 = np.array([q0, q_dot0], dtype=float)

    omega_n = oscillator.natural_frequency
    period = 2.0 * np.pi / omega_n
    history = integrate_fixed_step(
        oscillator.rhs(),
        y0,
        t0=0.0,
        tf=6.0 * period,
        dt=period / 80.0,
    )

    exact = oscillator.exact_undamped_state(history.t, y0)
    q = history.y[:, 0]
    q_dot = history.y[:, 1]
    energy = oscillator_mechanical_energy(mass, stiffness, q, q_dot)
    state_error = np.linalg.norm(history.y - exact, axis=1)

    step_sizes, final_errors, observed_order = convergence_study(
        oscillator,
        y0,
        period,
    )
    order = np.argsort(step_sizes)

    output_dir = Path("outputs")
    output_dir.mkdir(exist_ok=True)
    figure_path = output_dir / "lesson_02_linear_oscillator.svg"
    save_stacked_svg(
        figure_path,
        panels=[
            [
                Curve(history.t, q, "RK4", "#1f77b4"),
                Curve(history.t, exact[:, 0], "exact", "#d62728"),
            ],
            [
                Curve(q, q_dot, "RK4", "#2ca02c"),
                Curve(exact[:, 0], exact[:, 1], "exact", "#9467bd"),
            ],
            [Curve(history.t, relative_drift(energy), "energy drift", "#d62728")],
            [
                Curve(
                    np.log10(step_sizes[order]),
                    np.log10(final_errors[order]),
                    "RK4 error",
                    "#ff7f0e",
                )
            ],
        ],
        titles=[
            "Displacement",
            "Phase Portrait",
            "Energy Trend",
            "Step-size Convergence",
        ],
        x_labels=["time [s]", "q [m]", "time [s]", "log10(dt [s])"],
        y_labels=["q [m]", "q_dot [m/s]", "relative drift", "log10(final error)"],
    )

    print(f"natural angular frequency: {omega_n:.6f} rad/s")
    print(f"natural period: {period:.6f} s")
    print(f"max state error: {np.max(state_error):.3e}")
    print(f"max relative energy drift: {np.max(np.abs(relative_drift(energy))):.3e}")
    print(f"observed RK4 order: {observed_order:.3f}")
    print(f"saved {figure_path}")


if __name__ == "__main__":
    main()
