#!/usr/bin/env python3
"""
plot_simulation.py
Trajectory and mission plotting for unicycle robot symbolic control simulation.

Dependencies:
    - matplotlib
    - csv (standard library)
    (No pandas required)
"""

import sys
import os
import csv
import argparse
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

# ============================================================================
# 1. Workspace and Region bounds (defined once from reference article)
# ============================================================================
# Workspace: X = [0, 10] x [0, 10] x [-pi, pi]
WORKSPACE_BOUNDS = (0.0, 10.0, 0.0, 10.0)  # (x1_min, x1_max, x2_min, x2_max)

# Regions from paper:
# R1 = [4.0, 5.0] x [8.5, 9.5] (Target 1)
# R2 = [8.5, 9.5] x [2.0, 3.0] (Target 2)
# R3 = [2.0, 3.0] x [0.5, 1.5] (Final target)
# R4 = [3.0, 7.0] x [3.0, 7.0] (Obstacle)
REGIONS = {
    'R1': {
        'bounds': (4.0, 5.0, 8.5, 9.5),
        'facecolor': '#a1d99b',
        'edgecolor': '#238b45',
        'label': 'R1 (Target 1)',
        'text_pos': (4.5, 9.0)
    },
    'R2': {
        'bounds': (8.5, 9.5, 2.0, 3.0),
        'facecolor': '#9ecae1',
        'edgecolor': '#2171b5',
        'label': 'R2 (Target 2)',
        'text_pos': (9.0, 2.5)
    },
    'R3': {
        'bounds': (2.0, 3.0, 0.5, 1.5),
        'facecolor': '#fdd0a2',
        'edgecolor': '#d94801',
        'label': 'R3 (Goal)',
        'text_pos': (2.5, 1.0)
    },
    'R4': {
        'bounds': (3.0, 7.0, 3.0, 7.0),
        'facecolor': '#fc9272',
        'edgecolor': '#cb181d',
        'label': 'R4 (Obstacle)',
        'text_pos': (5.0, 5.0)
    }
}

# Discrete ordering of Mission states for the mission-vs-time plot
MISSION_ORDER = ['START', 'VISITED_R1', 'VISITED_R2', 'SUCCESS', 'FAILURE']
MISSION_TO_Y = {name: i for i, name in enumerate(MISSION_ORDER)}


# ============================================================================
# 2. CSV Reader
# ============================================================================
def parse_simulation_csv(filepath):
    """
    Reads a simulation CSV file exported by simulation.cpp.
    Extracts metadata from commented header lines and observation rows.
    """
    metadata = {
        'seed': 'unknown',
        'reason': 'unknown',
        'initial_state': None,
        'movements': 0
    }
    rows = []

    with open(filepath, 'r', encoding='utf-8') as f:
        reader = csv.reader(f)
        for raw_row in reader:
            if not raw_row:
                continue
            first = raw_row[0].strip()
            # Parse commented metadata
            if first.startswith('#'):
                line = ','.join(raw_row).lstrip('#').strip()
                if ':' in line:
                    key, val = line.split(':', 1)
                    key = key.strip()
                    val = val.strip()
                    if key == 'seed':
                        metadata['seed'] = val
                    elif key == 'reason':
                        metadata['reason'] = val
                    elif key == 'initial_state':
                        metadata['initial_state'] = val
                    elif key == 'movements':
                        try:
                            metadata['movements'] = int(val)
                        except ValueError:
                            pass
                continue

            # Header row
            if first == 't':
                continue

            # Data row: t, x1, x2, x3, mission, u1, u2, w1, w2, w3
            t = int(raw_row[0])
            x1 = float(raw_row[1])
            x2 = float(raw_row[2])
            x3 = float(raw_row[3])
            mission = raw_row[4].strip()

            u1 = float(raw_row[5]) if len(raw_row) > 5 and raw_row[5].strip() != '' else None
            u2 = float(raw_row[6]) if len(raw_row) > 6 and raw_row[6].strip() != '' else None
            w1 = float(raw_row[7]) if len(raw_row) > 7 and raw_row[7].strip() != '' else None
            w2 = float(raw_row[8]) if len(raw_row) > 8 and raw_row[8].strip() != '' else None
            w3 = float(raw_row[9]) if len(raw_row) > 9 and raw_row[9].strip() != '' else None

            rows.append({
                't': t, 'x1': x1, 'x2': x2, 'x3': x3,
                'mission': mission,
                'u1': u1, 'u2': u2,
                'w1': w1, 'w2': w2, 'w3': w3
            })

    return metadata, rows


# ============================================================================
# 3. Plotting logic
# ============================================================================
def plot_simulations(csv_paths, output_prefix=None, show_heading=False, dpi=300):
    """
    Plots 2D trajectories and mission timelines from one or multiple simulation CSVs.
    Saves results as both PNG and PDF.
    """
    datasets = []
    for path in csv_paths:
        if not os.path.exists(path):
            print(f"Warning: File {path} not found. Skipping.")
            continue
        meta, rows = parse_simulation_csv(path)
        if not rows:
            print(f"Warning: No data in {path}. Skipping.")
            continue
        datasets.append({
            'path': path,
            'meta': meta,
            'rows': rows
        })

    if not datasets:
        print("Error: No valid simulation data to plot.")
        return False

    fig, (ax_traj, ax_mission) = plt.subplots(
        2, 1,
        figsize=(9, 11),
        gridspec_kw={'height_ratios': [2.2, 1.0]}
    )

    # ------------------------------------------------------------------------
    # Subplot 1: 2D Robot Trajectory in Workspace
    # ------------------------------------------------------------------------
    # Draw workspace boundary
    w_x1_min, w_x1_max, w_x2_min, w_x2_max = WORKSPACE_BOUNDS
    ax_traj.plot(
        [w_x1_min, w_x1_max, w_x1_max, w_x1_min, w_x1_min],
        [w_x2_min, w_x2_min, w_x2_max, w_x2_max, w_x2_min],
        'k--', lw=1.5, label='Workspace Boundary'
    )

    # Draw regions R1, R2, R3, R4
    for r_name, r_info in REGIONS.items():
        x1_min, x1_max, x2_min, x2_max = r_info['bounds']
        width = x1_max - x1_min
        height = x2_max - x2_min
        rect = patches.Rectangle(
            (x1_min, x2_min), width, height,
            linewidth=2,
            edgecolor=r_info['edgecolor'],
            facecolor=r_info['facecolor'],
            alpha=0.6,
            zorder=2,
            label=r_info['label']
        )
        ax_traj.add_patch(rect)
        # Add centered text label
        tx, ty = r_info['text_pos']
        ax_traj.text(
            tx, ty, r_name,
            fontsize=12, fontweight='bold',
            color=r_info['edgecolor'],
            ha='center', va='center', zorder=3
        )

    # Distinct palette for multiple trajectories
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#9467bd', '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']

    for idx, ds in enumerate(datasets):
        rows = ds['rows']
        meta = ds['meta']
        color = colors[idx % len(colors)]

        x1_vals = [r['x1'] for r in rows]
        x2_vals = [r['x2'] for r in rows]
        x3_vals = [r['x3'] for r in rows]

        traj_label = f"Seed {meta['seed']} ({meta['reason']})" if len(datasets) > 1 else "Robot Trajectory"
        ax_traj.plot(x1_vals, x2_vals, color=color, lw=2.2, zorder=4, label=traj_label)

        # Start point marker
        ax_traj.scatter(
            [x1_vals[0]], [x2_vals[0]],
            color=color, marker='o', s=90, edgecolors='black', lw=1.5,
            zorder=5, label='Start' if idx == 0 else ""
        )

        # End point marker
        end_marker = '*' if meta['reason'] == 'SUCCESS' else ('X' if meta['reason'] == 'FAILURE' else 's')
        ax_traj.scatter(
            [x1_vals[-1]], [x2_vals[-1]],
            color=color, marker=end_marker, s=140, edgecolors='black', lw=1.5,
            zorder=5, label=f"End ({meta['reason']})" if idx == 0 else ""
        )

        # Optional heading arrows along the trajectory
        if show_heading:
            step_skip = max(1, len(rows) // 15)
            arrow_x = [x1_vals[i] for i in range(0, len(rows), step_skip)]
            arrow_y = [x2_vals[i] for i in range(0, len(rows), step_skip)]
            arrow_u = [0.35 * np.cos(x3_vals[i]) for i in range(0, len(rows), step_skip)]
            arrow_v = [0.35 * np.sin(x3_vals[i]) for i in range(0, len(rows), step_skip)]
            ax_traj.quiver(
                arrow_x, arrow_y, arrow_u, arrow_v,
                angles='xy', scale_units='xy', scale=1,
                color=color, alpha=0.7, width=0.005, zorder=4
            )

    ax_traj.set_xlim(-0.5, 10.5)
    ax_traj.set_ylim(-0.5, 10.5)
    ax_traj.set_aspect('equal')
    ax_traj.set_xlabel('Position $x_1$ (m)', fontsize=11)
    ax_traj.set_ylabel('Position $x_2$ (m)', fontsize=11)
    ax_traj.grid(True, linestyle=':', alpha=0.6)

    # Title
    if len(datasets) == 1:
        d = datasets[0]
        ax_traj.set_title(
            f"Robot Trajectory | StopReason: {d['meta']['reason']} | Seed: {d['meta']['seed']} | Steps: {d['meta']['movements']}",
            fontsize=12, fontweight='bold', pad=10
        )
    else:
        ax_traj.set_title(
            f"Robot Trajectories Overlay ({len(datasets)} runs)",
            fontsize=12, fontweight='bold', pad=10
        )
    ax_traj.legend(loc='upper left', bbox_to_anchor=(1.02, 1.0), borderaxespad=0.0, fontsize=9)

    # ------------------------------------------------------------------------
    # Subplot 2: Mission Memory vs Time
    # ------------------------------------------------------------------------
    for idx, ds in enumerate(datasets):
        rows = ds['rows']
        meta = ds['meta']
        color = colors[idx % len(colors)]

        t_vals = [r['t'] for r in rows]
        m_vals = [MISSION_TO_Y.get(r['mission'], -1) for r in rows]

        run_lbl = f"Seed {meta['seed']}" if len(datasets) > 1 else "Mission"
        ax_mission.step(t_vals, m_vals, where='post', color=color, lw=2.0, label=run_lbl)
        ax_mission.scatter(t_vals, m_vals, color=color, s=25, zorder=4)

    ax_mission.set_yticks(range(len(MISSION_ORDER)))
    ax_mission.set_yticklabels(MISSION_ORDER, fontsize=9)
    ax_mission.set_ylim(-0.5, len(MISSION_ORDER) - 0.5)
    ax_mission.set_xlabel('Time step $t$', fontsize=11)
    ax_mission.set_ylabel('Mission State', fontsize=11)
    ax_mission.set_title('Mission Automaton Evolution over Time', fontsize=11, fontweight='bold')
    ax_mission.grid(True, linestyle=':', alpha=0.6)
    if len(datasets) > 1:
        ax_mission.legend(loc='upper left', bbox_to_anchor=(1.02, 1.0), borderaxespad=0.0, fontsize=9)

    plt.tight_layout()

    # Determine output filenames
    if output_prefix is None:
        base_name = os.path.splitext(datasets[0]['path'])[0]
        if len(datasets) > 1:
            base_name = "overlay_simulation"
    else:
        base_name = output_prefix

    png_path = f"{base_name}.png"
    pdf_path = f"{base_name}.pdf"

    plt.savefig(png_path, dpi=dpi, bbox_inches='tight')
    plt.savefig(pdf_path, bbox_inches='tight')
    plt.close()

    print(f"[Plot Generated] PNG saved to: {png_path}")
    print(f"[Plot Generated] PDF saved to: {pdf_path}")
    return True


# ============================================================================
# 4. Main Entry Point
# ============================================================================
def main():
    parser = argparse.ArgumentParser(
        description="Plot 2D robot trajectories and mission timeline from simulation CSVs."
    )
    parser.add_argument(
        'csv_files', nargs='+',
        help="One or more CSV filepaths generated by the C++ simulation."
    )
    parser.add_argument(
        '--output', '-o', default=None,
        help="Output filename prefix (without extension). Generates .png and .pdf."
    )
    parser.add_argument(
        '--show-heading', action='store_true',
        help="Overlay small orientation arrows along the trajectory."
    )
    parser.add_argument(
        '--dpi', type=int, default=300,
        help="Resolution for PNG output (default: 300 dpi)."
    )

    args = parser.parse_args()
    success = plot_simulations(
        args.csv_files,
        output_prefix=args.output,
        show_heading=args.show_heading,
        dpi=args.dpi
    )
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
