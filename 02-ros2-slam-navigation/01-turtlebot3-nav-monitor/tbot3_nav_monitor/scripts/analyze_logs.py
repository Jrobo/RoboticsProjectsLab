#!/usr/bin/env python3

import os
import glob
import pandas as pd
import matplotlib.pyplot as plt


def find_latest_log(log_dir):
    csv_files = glob.glob(os.path.join(log_dir, "nav_monitor_log_*.csv"))
    if not csv_files:
        raise FileNotFoundError(f"No log files found in {log_dir}")
    return max(csv_files, key=os.path.getmtime)


def clean_numeric(series):
    return series.replace([float("inf"), float("-inf")], pd.NA).dropna()


def main():
    package_dir = os.path.expanduser("~/tbot3_nav_monitor_ws/src/tbot3_nav_monitor")
    log_dir = os.path.join(package_dir, "logs")
    results_dir = os.path.join(package_dir, "results")

    os.makedirs(results_dir, exist_ok=True)

    latest_log = find_latest_log(log_dir)
    print(f"Using latest log file: {latest_log}")

    df = pd.read_csv(latest_log)

    if df.empty:
        raise RuntimeError("CSV file is empty.")

    df["time_sec"] = df["timestamp"] - df["timestamp"].iloc[0]

    total_time = float(df["time_sec"].iloc[-1])
    total_distance = float(df["total_distance_m"].iloc[-1])
    final_battery = float(df["battery_level_percent"].iloc[-1])
    avg_velocity = float(df["linear_velocity_mps"].mean())
    max_velocity = float(df["linear_velocity_mps"].max())

    front_obstacle = clean_numeric(df["front_obstacle_distance_m"])
    if front_obstacle.empty:
        avg_front_obstacle = 0.0
        min_front_obstacle = 0.0
    else:
        avg_front_obstacle = float(front_obstacle.mean())
        min_front_obstacle = float(front_obstacle.min())

    avg_complexity = float(df["environment_complexity"].mean())
    max_complexity = float(df["environment_complexity"].max())

    if "recovery_event_count" in df.columns:
        recovery_events = int(df["recovery_event_count"].max())
    else:
        recovery_events = 0

    if "adaptation_active" in df.columns:
        adaptation_count = int(df["adaptation_active"].sum())
        adaptation_percentage = 100.0 * adaptation_count / len(df)
    else:
        adaptation_count = 0
        adaptation_percentage = 0.0

    summary = pd.DataFrame([{
        "log_file": os.path.basename(latest_log),
        "total_time_sec": round(total_time, 3),
        "total_distance_m": round(total_distance, 3),
        "final_battery_percent": round(final_battery, 3),
        "average_velocity_mps": round(avg_velocity, 3),
        "maximum_velocity_mps": round(max_velocity, 3),
        "average_front_obstacle_m": round(avg_front_obstacle, 3),
        "minimum_front_obstacle_m": round(min_front_obstacle, 3),
        "average_environment_complexity": round(avg_complexity, 3),
        "maximum_environment_complexity": round(max_complexity, 3),
        "recovery_event_count": recovery_events,
        "adaptation_active_samples": adaptation_count,
        "adaptation_active_percentage": round(adaptation_percentage, 3),
    }])

    summary_path = os.path.join(results_dir, "summary_metrics.csv")
    summary.to_csv(summary_path, index=False)
    print(f"Saved summary: {summary_path}")
    print(summary.to_string(index=False))

    plt.figure()
    plt.plot(df["time_sec"], df["total_distance_m"], label="Travelled distance (m)")
    plt.plot(df["time_sec"], df["battery_level_percent"], label="Battery level (%)")
    plt.xlabel("Time (s)")
    plt.ylabel("Value")
    plt.title("Travelled Distance and Simulated Battery")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(results_dir, "distance_battery_plot.png"), dpi=300)
    print("Saved distance_battery_plot.png")

    plt.figure()
    front_obstacle_plot = pd.to_numeric(
        df["front_obstacle_distance_m"],
        errors="coerce"
    ).replace([float("inf"), float("-inf")], float("nan"))

    plt.plot(df["time_sec"], front_obstacle_plot, label="Front obstacle distance (m)")
    plt.plot(df["time_sec"], df["environment_complexity"], label="Environment complexity")
    plt.xlabel("Time (s)")
    plt.ylabel("Value")
    plt.title("Obstacle Distance and Environment Complexity")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(results_dir, "obstacle_complexity_plot.png"), dpi=300)
    print("Saved obstacle_complexity_plot.png")

    if "cmd_linear_velocity_mps" in df.columns and "safe_linear_velocity_mps" in df.columns:
        plt.figure()
        plt.plot(df["time_sec"], df["cmd_linear_velocity_mps"], label="Commanded linear velocity")
        plt.plot(df["time_sec"], df["safe_linear_velocity_mps"], label="Safe linear velocity")
        plt.xlabel("Time (s)")
        plt.ylabel("Velocity (m/s)")
        plt.title("Commanded Velocity and Adaptive Safe Velocity")
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(os.path.join(results_dir, "velocity_adaptation_plot.png"), dpi=300)
        print("Saved velocity_adaptation_plot.png")

    print("Analysis completed successfully.")


if __name__ == "__main__":
    main()
