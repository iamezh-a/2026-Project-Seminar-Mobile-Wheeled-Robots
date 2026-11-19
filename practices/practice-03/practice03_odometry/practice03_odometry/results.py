import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def save_results(samples, output_dir, experiment_name, metadata):
    # Столбцы: время, GT (x, y, yaw), энкодеры (x, y, yaw),
    # команды (x, y, yaw).
    data = np.asarray(samples, dtype=float)
    time = data[:, 0]

    encoder_position_error = np.hypot(
        data[:, 4] - data[:, 1],
        data[:, 5] - data[:, 2],
    )
    command_position_error = np.hypot(
        data[:, 7] - data[:, 1],
        data[:, 8] - data[:, 2],
    )

    encoder_angle_difference = data[:, 6] - data[:, 3]
    command_angle_difference = data[:, 9] - data[:, 3]

    encoder_yaw_error = np.abs(np.arctan2(
        np.sin(encoder_angle_difference),
        np.cos(encoder_angle_difference),
    ))
    command_yaw_error = np.abs(np.arctan2(
        np.sin(command_angle_difference),
        np.cos(command_angle_difference),
    ))

    output_dir = Path(output_dir).expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)

    csv_path = output_dir / f"{experiment_name}.csv"
    png_path = output_dir / f"{experiment_name}.png"
    json_path = output_dir / f"{experiment_name}.json"

    output_data = np.column_stack((
        data,
        encoder_position_error,
        command_position_error,
        encoder_yaw_error,
        command_yaw_error,
    ))
    header = (
        "time_s,gt_x_m,gt_y_m,gt_yaw_rad,"
        "encoder_x_m,encoder_y_m,encoder_yaw_rad,"
        "command_x_m,command_y_m,command_yaw_rad,"
        "encoder_position_error_m,command_position_error_m,"
        "encoder_yaw_error_rad,command_yaw_error_rad"
    )
    np.savetxt(
        csv_path, output_data,
        delimiter=",", header=header, comments="", fmt="%.9f",
    )

    fig, axes = plt.subplots(1, 3, figsize=(16, 5))

    axes[0].plot(data[:, 1], data[:, 2], color="black", label="GT")
    axes[0].plot(data[:, 4], data[:, 5], label="Encoders")
    axes[0].plot(data[:, 7], data[:, 8], label="Commands")
    axes[0].set_title("XY trajectories")
    axes[0].set_xlabel("X (m)")
    axes[0].set_ylabel("Y (m)")
    axes[0].set_aspect("equal", adjustable="box")

    axes[1].plot(time, encoder_position_error, label="Encoders")
    axes[1].plot(time, command_position_error, label="Commands")
    axes[1].set_title("Absolute position error")
    axes[1].set_xlabel("Time from experiment start (s)")
    axes[1].set_ylabel("Position error (m)")

    axes[2].plot(time, encoder_yaw_error, label="Encoders")
    axes[2].plot(time, command_yaw_error, label="Commands")
    axes[2].set_title("Absolute yaw error")
    axes[2].set_xlabel("Time from experiment start (s)")
    axes[2].set_ylabel("Yaw error (rad)")

    for ax in axes:
        ax.grid(True, alpha=0.3)
        ax.legend()

    fig.suptitle(experiment_name)
    fig.tight_layout()
    fig.savefig(png_path, dpi=180)
    plt.close(fig)

    summary = dict(metadata)
    summary["sample_count"] = len(data)
    summary["duration_s"] = float(time[-1])

    for name, position_error, yaw_error in (
        ("encoders", encoder_position_error, encoder_yaw_error),
        ("commands", command_position_error, command_yaw_error),
    ):
        summary[name] = {
            "position_rmse_m": float(np.sqrt(np.mean(position_error ** 2))),
            "position_max_m": float(np.max(position_error)),
            "position_final_m": float(position_error[-1]),
            "yaw_rmse_rad": float(np.sqrt(np.mean(yaw_error ** 2))),
            "yaw_max_rad": float(np.max(yaw_error)),
            "yaw_final_rad": float(yaw_error[-1]),
        }

    json_path.write_text(
        json.dumps(summary, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )

    return csv_path, png_path, json_path
