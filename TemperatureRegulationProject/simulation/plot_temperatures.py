#!/usr/bin/env python3
"""
Plot closed-loop building safety temperature trajectories.
Standard library (csv, argparse, pathlib) + matplotlib only.
"""

import csv
import argparse
from pathlib import Path
import matplotlib.pyplot as plt

def parse_args():
    parser = argparse.ArgumentParser(description="Plot room temperatures over time.")
    parser.add_argument("--csv", default="sim_trace3_mixed.csv", help="Simulation CSV path.")
    parser.add_argument("--output-prefix", default="plot_mixed_run", help="Output prefix for PNG/PDF.")
    parser.add_argument("--all-scenarios", action="store_true", help="Generate a combined 3-panel figure of all traces.")
    return parser.parse_args()

def load_simulation_csv(filepath):
    times = []
    rooms_data = {}
    metadata = {}

    with open(filepath, "r", encoding="utf-8") as f:
        reader = csv.reader(f)
        header = None
        for row in reader:
            if not row:
                continue
            if row[0].startswith("#"):
                content = row[0][1:].strip()
                for item in content.split(","):
                    if "=" in item:
                        k, v = item.strip().split("=", 1)
                        metadata[k.strip()] = v.strip()
                continue
            if header is None:
                header = [col.strip() for col in row]
                temp_cols = [c for c in header if c.startswith("T")]
                for tc in temp_cols:
                    rooms_data[tc] = []
                continue

            t = int(row[0])
            times.append(t)
            for tc in rooms_data.keys():
                col_idx = header.index(tc)
                rooms_data[tc].append(float(row[col_idx]))

    return times, rooms_data, metadata

def plot_trajectory(times, rooms_data, metadata, output_prefix, custom_title=None):
    fig, ax = plt.subplots(figsize=(10, 5.5), dpi=300)

    # Shaded safe region [17.0, 22.0]
    ax.axhspan(17.0, 22.0, color="#e8f5e9", alpha=0.7, label="Safe Band $[17^\\circ\\mathrm{C}, 22^\\circ\\mathrm{C}]$")

    # Boundary threshold lines
    ax.axhline(17.0, color="#d32f2f", linestyle="--", linewidth=1.6, label="Lower Bound ($17^\\circ\\mathrm{C}$)")
    ax.axhline(22.0, color="#c2185b", linestyle="--", linewidth=1.6, label="Upper Bound ($22^\\circ\\mathrm{C}$)")

    # Color palette and markers for rooms
    colors = ["#1976d2", "#388e3c", "#f57c00", "#7b1fa2", "#0097a7"]
    markers = ["o", "s", "^", "v", "D"]

    for idx, (room_name, temps) in enumerate(rooms_data.items()):
        c = colors[idx % len(colors)]
        m = markers[idx % len(markers)]
        room_idx = room_name[1:]
        init_val = temps[0] if temps else 0.0
        label_text = f"Room {room_idx} ($T_{{{room_idx}}}(0)={init_val}^\\circ\\mathrm{{C}}$)"
        ax.plot(times, temps, marker=m, markersize=3.5, linewidth=1.8, color=c, label=label_text)

    ax.set_xlabel("Discrete Time Step $k$", fontsize=12)
    ax.set_ylabel("Temperature ($^\\circ\\mathrm{C}$)", fontsize=12)
    ax.set_ylim(16.0, 23.0)
    ax.grid(True, linestyle=":", alpha=0.6)

    safe_status = metadata.get("safe", "true")
    steps = metadata.get("steps", str(len(times) - 1))
    title = custom_title or f"Building Safety Control — Closed-Loop Temperature Trajectory\n($N={len(rooms_data)}$ Rooms, Steps={steps}, Safe={safe_status})"
    ax.set_title(title, fontsize=13, fontweight="bold", pad=12)

    ax.legend(loc="upper right", framealpha=0.92, fontsize=10)
    plt.tight_layout()

    png_path = f"{output_prefix}.png"
    pdf_path = f"{output_prefix}.pdf"
    plt.savefig(png_path, dpi=300)
    plt.savefig(pdf_path)
    plt.close()

    print(f"[Plot] Generated: {png_path} and {pdf_path}")

def plot_combined_scenarios(scenarios, output_prefix="plot_all_scenarios"):
    """Plot 3 scenarios side-by-side or stacked in one publication figure."""
    fig, axes = plt.subplots(1, 3, figsize=(18, 5.5), sharey=True, dpi=300)

    colors = ["#1976d2", "#388e3c", "#f57c00"]
    markers = ["o", "s", "^"]

    for ax, (csv_path, title) in zip(axes, scenarios):
        if not Path(csv_path).exists():
            continue
        times, rooms_data, _ = load_simulation_csv(csv_path)

        ax.axhspan(17.0, 22.0, color="#e8f5e9", alpha=0.7, label="Safe Band")
        ax.axhline(17.0, color="#d32f2f", linestyle="--", linewidth=1.4, label="17°C Bound")
        ax.axhline(22.0, color="#c2185b", linestyle="--", linewidth=1.4, label="22°C Bound")

        for idx, (room_name, temps) in enumerate(rooms_data.items()):
            c = colors[idx % len(colors)]
            m = markers[idx % len(markers)]
            room_idx = room_name[1:]
            init_val = temps[0] if temps else 0.0
            ax.plot(times, temps, marker=m, markersize=3, linewidth=1.6, color=c,
                    label=f"Room {room_idx} ({init_val}°C)")

        ax.set_title(title, fontsize=12, fontweight="bold", pad=10)
        ax.set_xlabel("Discrete Time Step $k$", fontsize=11)
        ax.set_ylim(16.0, 23.0)
        ax.grid(True, linestyle=":", alpha=0.6)
        ax.legend(loc="upper right", fontsize=8.5, framealpha=0.9)

    axes[0].set_ylabel("Temperature ($^\\circ\\mathrm{C}$)", fontsize=12)
    plt.suptitle("Building Safety Control — Closed-Loop Benchmark Scenarios ($N=3$ Rooms)",
                 fontsize=14, fontweight="bold", y=1.02)
    plt.tight_layout()

    png_path = f"{output_prefix}.png"
    pdf_path = f"{output_prefix}.pdf"
    plt.savefig(png_path, dpi=300, bbox_inches="tight")
    plt.savefig(pdf_path, bbox_inches="tight")
    plt.close()
    print(f"[Combined Plot] Generated: {png_path} and {pdf_path}")

def main():
    args = parse_args()

    if args.all_scenarios:
        scenarios = [
            ("sim_trace1_cold.csv", "Scenario 1: All-Cold (17, 17, 17)"),
            ("sim_trace2_hot.csv", "Scenario 2: All-Hot (22, 22, 22)"),
            ("sim_trace3_mixed.csv", "Scenario 3: Mixed (17, 19.5, 22)")
        ]
        plot_combined_scenarios(scenarios, "plot_all_three_scenarios")
        return

    csv_file = Path(args.csv)
    if not csv_file.exists():
        print(f"Error: CSV file not found: {csv_file}")
        return

    times, rooms_data, metadata = load_simulation_csv(csv_file)
    plot_trajectory(times, rooms_data, metadata, args.output_prefix)

if __name__ == "__main__":
    main()
