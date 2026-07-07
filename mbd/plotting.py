from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np


Array = np.ndarray


@dataclass(frozen=True)
class Curve:
    x: Array
    y: Array
    label: str
    color: str


def save_stacked_svg(
    path: Path,
    panels: list[list[Curve]],
    titles: list[str],
    x_labels: list[str],
    y_labels: list[str],
    width: int = 900,
    panel_height: int = 240,
) -> None:
    """Save simple stacked line plots as an SVG file."""

    margin_left = 90
    margin_right = 30
    margin_top = 45
    margin_bottom = 55
    gap = 22
    height = len(panels) * panel_height + (len(panels) - 1) * gap

    elements = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        '<style>text{font-family:Arial,sans-serif;font-size:14px;fill:#17202a}.title{font-size:18px;font-weight:700}.tick{font-size:12px;fill:#566573}.axis{stroke:#2c3e50;stroke-width:1}.grid{stroke:#d5dbdb;stroke-width:1}.curve{fill:none;stroke-width:2.4}</style>',
    ]

    for panel_index, curves in enumerate(panels):
        top = panel_index * (panel_height + gap)
        plot_x0 = margin_left
        plot_y0 = top + margin_top
        plot_w = width - margin_left - margin_right
        plot_h = panel_height - margin_top - margin_bottom

        x_min = min(float(np.min(curve.x)) for curve in curves)
        x_max = max(float(np.max(curve.x)) for curve in curves)
        y_min = min(float(np.min(curve.y)) for curve in curves)
        y_max = max(float(np.max(curve.y)) for curve in curves)
        if abs(x_max - x_min) < 1e-12:
            x_max = x_min + 1.0
        if abs(y_max - y_min) < 1e-12:
            y_max = y_min + 1.0

        y_pad = 0.06 * (y_max - y_min)
        y_min -= y_pad
        y_max += y_pad

        elements.append(f'<text class="title" x="{plot_x0}" y="{top + 24}">{titles[panel_index]}</text>')

        for i in range(5):
            alpha = i / 4
            x_tick = plot_x0 + alpha * plot_w
            y_tick = plot_y0 + alpha * plot_h
            x_value = x_min + alpha * (x_max - x_min)
            y_value = y_max - alpha * (y_max - y_min)
            elements.append(f'<line class="grid" x1="{x_tick:.2f}" y1="{plot_y0}" x2="{x_tick:.2f}" y2="{plot_y0 + plot_h}"/>')
            elements.append(f'<line class="grid" x1="{plot_x0}" y1="{y_tick:.2f}" x2="{plot_x0 + plot_w}" y2="{y_tick:.2f}"/>')
            elements.append(f'<text class="tick" text-anchor="middle" x="{x_tick:.2f}" y="{plot_y0 + plot_h + 22}">{x_value:.3g}</text>')
            elements.append(f'<text class="tick" text-anchor="end" x="{plot_x0 - 10}" y="{y_tick + 4:.2f}">{y_value:.3g}</text>')

        elements.append(f'<line class="axis" x1="{plot_x0}" y1="{plot_y0 + plot_h}" x2="{plot_x0 + plot_w}" y2="{plot_y0 + plot_h}"/>')
        elements.append(f'<line class="axis" x1="{plot_x0}" y1="{plot_y0}" x2="{plot_x0}" y2="{plot_y0 + plot_h}"/>')
        elements.append(f'<text text-anchor="middle" x="{plot_x0 + plot_w / 2:.2f}" y="{top + panel_height - 14}">{x_labels[panel_index]}</text>')
        elements.append(f'<text text-anchor="middle" transform="translate(24 {plot_y0 + plot_h / 2:.2f}) rotate(-90)">{y_labels[panel_index]}</text>')

        legend_x = plot_x0 + plot_w - 120
        legend_y = top + 24
        for curve_index, curve in enumerate(curves):
            y_legend = legend_y + curve_index * 18
            elements.append(f'<line x1="{legend_x}" y1="{y_legend}" x2="{legend_x + 24}" y2="{y_legend}" stroke="{curve.color}" stroke-width="2.4"/>')
            elements.append(f'<text class="tick" x="{legend_x + 32}" y="{y_legend + 4}">{curve.label}</text>')

        for curve in curves:
            x_norm = (curve.x - x_min) / (x_max - x_min)
            y_norm = (curve.y - y_min) / (y_max - y_min)
            points = [
                f"{plot_x0 + float(xn) * plot_w:.2f},{plot_y0 + (1.0 - float(yn)) * plot_h:.2f}"
                for xn, yn in zip(x_norm, y_norm)
            ]
            elements.append(f'<polyline class="curve" stroke="{curve.color}" points="{" ".join(points)}"/>')

    elements.append("</svg>")
    path.write_text("\n".join(elements), encoding="utf-8")
