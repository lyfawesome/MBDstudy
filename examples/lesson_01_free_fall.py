from __future__ import annotations

from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import numpy as np

from mbd.analysis import particle_mechanical_energy, relative_drift
from mbd.integrators import integrate_fixed_step
from mbd.particles import Particle, constant_gravity
from mbd.plotting import Curve, save_stacked_svg


def main() -> None:
    mass = 2.0
    g = 9.81
    particle = Particle(mass=mass)

    initial_position = np.array([0.0, 0.0, 10.0])
    initial_velocity = np.array([2.0, 0.0, 0.0])
    y0 = np.concatenate((initial_position, initial_velocity))

    rhs = particle.rhs(constant_gravity(mass=mass, g=g))
    history = integrate_fixed_step(rhs, y0, t0=0.0, tf=1.4, dt=0.01)

    position = history.y[:, :3]
    velocity = history.y[:, 3:]
    energy = particle_mechanical_energy(mass, position, velocity, g=g)

    output_dir = Path("outputs")
    output_dir.mkdir(exist_ok=True)

    figure_path = output_dir / "lesson_01_free_fall.svg"
    save_stacked_svg(
        figure_path,
        panels=[
            [Curve(history.t, position[:, 2], "z(t)", "#1f77b4")],
            [Curve(position[:, 0], position[:, 2], "trajectory", "#2ca02c")],
            [Curve(history.t, relative_drift(energy), "energy drift", "#d62728")],
        ],
        titles=["Height", "Trajectory", "Energy Trend"],
        x_labels=["time [s]", "x [m]", "time [s]"],
        y_labels=["z [m]", "z [m]", "relative drift"],
    )
    print(f"saved {figure_path}")
    print(f"max relative energy drift: {np.max(np.abs(relative_drift(energy))):.3e}")


if __name__ == "__main__":
    main()
