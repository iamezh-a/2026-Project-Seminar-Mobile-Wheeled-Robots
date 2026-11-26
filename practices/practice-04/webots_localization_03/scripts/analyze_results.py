import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parents[2] / "results"
TARGET = ROOT / "final60"
DURATION = 60.0

SOURCES = [
    ("small_1000", "initial_small_1000_realtime01", "initial", 1000, 0.1),
    ("small_15000", "initial_small_15000_realtime60_01", "initial", 15000, 0.1),
    ("large_1000", "initial_large_1000_realtime60_01", "initial", 1000, 0.5),
    ("large_15000", "initial_large_15000_realtime60_01", "initial", 15000, 0.5),
    ("global_1000", "global_1000_realtime60_01", "global", 1000, 0.0),
    ("global_15000", "global_15000_realtime01", "global", 15000, 0.0),
]

if TARGET.exists():
    raise SystemExit(f"Папка уже существует, перезаписи не будет: {TARGET}")

# Проверяем все исходники до создания итоговой папки.
prepared = []
for name, source, mode, particles, offset in SOURCES:
    folder = ROOT / source
    settings = json.loads((folder / "summary.json").read_text())
    expected = {
        "localization_mode": mode,
        "min_particles": 500,
        "max_particles": particles,
        "error_x": offset,
        "error_y": offset,
        "error_yaw": 0.0 if mode == "global" else (
            0.15 if offset == 0.1 else 1.0
        ),
        "position_threshold": 0.1,
        "yaw_threshold": 0.05,
        "confirmation_duration": 10.0,
        "maximum_allowed_sample_gap_s": 0.3,
    }
    for key, value in expected.items():
        if settings.get(key) != value:
            raise ValueError(f"{source}: неожиданное значение {key}")

    with (folder / "data.csv").open() as file:
        reader = csv.DictReader(file)
        fields = reader.fieldnames
        full_rows = [
            {key: float(value) for key, value in row.items()}
            for row in reader
        ]

    if not full_rows:
        raise ValueError(f"{source}: пустой CSV")
    if not all(
        math.isfinite(value)
        for row in full_rows for value in row.values()
    ):
        raise ValueError(f"{source}: нечисловые или бесконечные данные")
    if any(
        b["time_s"] <= a["time_s"]
        for a, b in zip(full_rows, full_rows[1:])
    ):
        raise ValueError(f"{source}: нарушен порядок времени")
    if full_rows[0]["time_s"] != 0 or full_rows[-1]["time_s"] < 59.5:
        raise ValueError(f"{source}: неполный интервал наблюдения")

    rows = [row for row in full_rows if 0 <= row["time_s"] <= DURATION]
    interval_start = None
    convergence = None
    confirmed_at = None
    previous_time = None

    for row in rows:
        t = row["time_s"]
        if previous_time is not None and t - previous_time > 0.3:
            interval_start = None
        previous_time = t

        inside = (
            row["position_error_m"] <= 0.1
            and row["yaw_error_rad"] <= 0.05
        )
        if not inside:
            interval_start = None
        elif interval_start is None:
            interval_start = t
        elif convergence is None and t - interval_start >= 10.0:
            convergence = interval_start
            confirmed_at = t

    gaps = [
        b["time_s"] - a["time_s"]
        for a, b in zip(rows, rows[1:])
    ]
    summary = dict(settings)
    summary.update({
        "run_id": name,
        "source_run_id": source,
        "source_duration_s": settings["duration"],
        "duration": DURATION,
        "simulation_mode": "realtime",
        "sample_count": len(rows),
        "convergence_time_s": convergence,
        "confirmation_time_s": confirmed_at,
        "last_sample_time_s": rows[-1]["time_s"],
        "maximum_observed_gap_s": max(gaps, default=0.0),
        "gaps_over_0_3_s": sum(g > 0.3 for g in gaps),
        "output_dir": str(TARGET),
        "analysis": "First 60 simulation seconds; convergence recalculated",
    })
    prepared.append((name, fields, rows, summary))

TARGET.mkdir()

for name, fields, rows, summary in prepared:
    folder = TARGET / name
    folder.mkdir()

    with (folder / "data.csv").open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    (folder / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n"
    )

    # Разрывы данных показываем разрывами линий, а не соединяем.
    def series(key):
        values = []
        previous = None
        for row in rows:
            if previous is not None and row["time_s"] - previous > 0.3:
                values.append(math.nan)
            values.append(row[key])
            previous = row["time_s"]
        return values

    fig, axes = plt.subplots(3, 1, figsize=(10, 13))
    axes[0].plot(series("gt_x_m"), series("gt_y_m"), label="Ground truth")
    axes[0].plot(series("amcl_x_m"), series("amcl_y_m"), label="AMCL")
    axes[0].set(xlabel="X (m)", ylabel="Y (m)", title="Robot trajectories")
    axes[0].set_aspect("equal", adjustable="box")

    convergence = summary["convergence_time_s"]
    for axis, column, threshold, label in [
        (axes[1], "position_error_m", 0.1, "Position error (m)"),
        (axes[2], "yaw_error_rad", 0.05, "Yaw error (rad)"),
    ]:
        axis.plot(series("time_s"), series(column), label="Error")
        axis.axhline(threshold, color="red", linestyle="--", label="Threshold")
        if convergence is not None:
            axis.axvline(
                convergence, color="green", linestyle=":",
                label=f"Convergence: {convergence:.3f} s",
            )
        axis.set(
            xlabel="Time since t0 (s)", ylabel=label, xlim=(0, DURATION)
        )

    for axis in axes:
        axis.grid(True, alpha=0.3)
        axis.legend()

    status = (
        "not confirmed within 60 s" if convergence is None
        else f"{convergence:.3f} s"
    )
    fig.suptitle(f"{name} | Convergence: {status}")
    fig.tight_layout()
    fig.savefig(folder / "plots.png", dpi=160)
    plt.close(fig)

    print(
        f"{name}: {status}; samples={len(rows)}; "
        f"max_gap={summary['maximum_observed_gap_s']:.3f} s"
    )

with (TARGET / "comparison.csv").open("w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow([
        "experiment", "min_particles", "max_particles",
        "convergence_time_s", "confirmed_within_60_s", "source_run_id",
    ])
    for name, fields, rows, summary in prepared:
        value = summary["convergence_time_s"]
        writer.writerow([
            name, summary["min_particles"], summary["max_particles"],
            "" if value is None else value,
            value is not None, summary["source_run_id"],
        ])

print(f"\nИтоговые результаты: {TARGET}")
print("Исходные результаты сохранены без изменений.")
