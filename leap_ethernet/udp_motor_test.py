import csv
import random
import statistics
import time
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt

from control_table_client import ControlTableClient, DataNames

UDP_IP = "10.42.42.50"
UDP_PORT = 8888
TIMEOUT = 2.0
WARMUP_COUNT = 100
TEST_COUNT = 500
MOTOR_COUNT = 16


def write_position_log_header() -> None:
    with open("position_log.csv", "w", newline="") as log_file:
        writer = csv.writer(log_file)
        writer.writerow(
            [
                "id",
                *[f"target_motor_{motor_id}" for motor_id in range(MOTOR_COUNT)],
                *[f"motor_{motor_id}" for motor_id in range(MOTOR_COUNT)],
            ]
        )


def append_position_log(
    test_id: int, target_positions: list[int], read_positions: dict[int, int]
) -> None:
    with open("position_log.csv", "a", newline="") as log_file:
        writer = csv.writer(log_file)
        writer.writerow(
            [
                test_id,
                *target_positions,
                *[read_positions[motor_id] for motor_id in range(MOTOR_COUNT)],
            ]
        )


def check_previous_targets() -> bool:
    mismatch_found = False
    with open("position_log.csv", newline="") as log_file:
        rows = list(csv.reader(log_file))[1:]

    for row_index in range(1, len(rows)):
        previous_targets = [
            int(value) for value in rows[row_index - 1][1 : MOTOR_COUNT + 1]
        ]
        current_positions = [
            int(value) for value in rows[row_index][MOTOR_COUNT + 1 :]
        ]
        mismatches = [
            motor_id
            for motor_id in range(MOTOR_COUNT)
            if current_positions[motor_id] != previous_targets[motor_id]
        ]

        if mismatches:
            print(
                f"Mismatch at line {row_index + 2}: "
                f"expected={previous_targets}, motors={mismatches}"
            )
            mismatch_found = True

    return mismatch_found


def plot_iteration_times(iteration_times: list[float]) -> None:
    plot_directory = Path("plots") / datetime.now().strftime(
        "%Y%m%d_%H%M%S_%f"
    )
    plot_directory.mkdir(parents=True, exist_ok=False)

    times_ms = [elapsed * 1000 for elapsed in iteration_times]
    mean_time_ms = sum(times_ms) / len(times_ms)
    median_time_ms = statistics.median(times_ms)
    standard_deviation_ms = statistics.stdev(times_ms)
    sorted_times_ms = sorted(times_ms)
    percentile_95_index = min(
        len(sorted_times_ms) - 1,
        max(0, int(len(sorted_times_ms) * 0.95) - 1),
    )
    percentile_95_ms = sorted_times_ms[percentile_95_index]

    print("\nIteration timing statistics")
    print(f"Minimum:       {min(times_ms):.3f} ms")
    print(f"Median:        {median_time_ms:.3f} ms")
    print(f"Mean:          {mean_time_ms:.3f} ms")
    print(f"Standard dev.: {standard_deviation_ms:.3f} ms")
    print(f"95th percentile: {percentile_95_ms:.3f} ms")
    print(f"Maximum:       {max(times_ms):.3f} ms")

    plt.figure(figsize=(10, 6))
    plt.hist(times_ms, bins="auto", edgecolor="black", alpha=0.8)
    plt.axvline(
        mean_time_ms,
        color="red",
        linestyle="--",
        label=f"Mean: {mean_time_ms:.2f} ms",
    )
    plt.axvline(
        mean_time_ms - standard_deviation_ms,
        color="orange",
        linestyle=":",
        label=f"Mean - SD: {mean_time_ms - standard_deviation_ms:.2f} ms",
    )
    plt.axvline(
        mean_time_ms + standard_deviation_ms,
        color="orange",
        linestyle=":",
        label=f"Mean + SD: {mean_time_ms + standard_deviation_ms:.2f} ms",
    )
    plt.xlabel("Iteration time (ms)")
    plt.ylabel("Frequency")
    plt.title("Distribution of UDP Motor Test Iteration Times")
    plt.grid(axis="y", alpha=0.3)
    plt.legend()
    plt.tight_layout()
    distribution_path = plot_directory / "iteration_time_distribution.png"
    plt.savefig(distribution_path, dpi=150)
    plt.show()
    plt.close()

    plt.figure(figsize=(10, 6))
    plt.plot(range(len(times_ms)), times_ms, linewidth=0.8)
    plt.axhline(
        mean_time_ms,
        color="red",
        linestyle="--",
        label=f"Mean: {mean_time_ms:.2f} ms",
    )
    plt.axhline(
        percentile_95_ms,
        color="purple",
        linestyle=":",
        label=f"95th percentile: {percentile_95_ms:.2f} ms",
    )
    plt.xlabel("Iteration")
    plt.ylabel("Iteration time (ms)")
    plt.title("UDP Motor Test Iteration Time by Iteration")
    plt.grid(alpha=0.3)
    plt.legend()
    plt.tight_layout()
    series_path = plot_directory / "iteration_time_series.png"
    plt.savefig(series_path, dpi=150)
    plt.show()
    plt.close()
    print(f"Plots saved to {plot_directory}")


def main() -> None:
    print(f"Testing UDP connection to OpenRB-150 at {UDP_IP}:{UDP_PORT}")
    print("-" * 40)

    client = ControlTableClient(UDP_IP, UDP_PORT, TIMEOUT, "udp_motor_log.csv")
    try:
        print(f"Setting current limit of all motors to {150} mA")
        client.write_all(DataNames.CURRENT_LIMIT, [150] * MOTOR_COUNT)
        
        print(f"Setting motors to position mode")
        client.write_all(DataNames.OPERATING_MODE, [3] * MOTOR_COUNT)

        print(f"Enabling torque on all motors")
        client.write_all(DataNames.TORQUE_ENABLE, [1] * MOTOR_COUNT)

        print(f"Setting all goal positions to {2048}")
        client.write_all(DataNames.GOAL_POSITION, [2048] * MOTOR_COUNT)

        print(
            f"Running {WARMUP_COUNT} warm-up position reads "
            "to wake up CPU/Network..."
        )
        for _ in range(WARMUP_COUNT):
            client.read_all(DataNames.PRESENT_POSITION)

        # write_position_log_header()
        iteration_times = []
        loop_start = time.perf_counter()

        print(f"Running {TEST_COUNT} test iterations...")

        for test_id in range(TEST_COUNT):
            iteration_start = time.perf_counter()
            target_positions = [
                random.randint(2000, 2100) for _ in range(MOTOR_COUNT)
            ]

            client.write_all(DataNames.GOAL_POSITION, target_positions)
            print("Wrote target positions:", target_positions)
            read_positions = client.read_all(DataNames.PRESENT_POSITION)
            print("Read positions:", read_positions)

            # print(
            #     f"Target Positions: {' '.join(map(str, target_positions))}, "
            #     "Read Positions: "
            #     f"{' '.join(str(read_positions[motor_id]) for motor_id in range(MOTOR_COUNT))}"
            # )
            append_position_log(test_id, target_positions, read_positions)
            iteration_times.append(time.perf_counter() - iteration_start)

        mismatch_found = check_previous_targets()
        if not mismatch_found:
            print("All motor positions match the previous target position.")

        loop_elapsed = time.perf_counter() - loop_start
        print(f"\nLoop completed in {loop_elapsed:.3f} seconds")
        print(f"Loop speed: {TEST_COUNT / loop_elapsed:.2f} iterations/second")
    finally:
        client.write_all(DataNames.TORQUE_ENABLE, [0] * MOTOR_COUNT)
        client.close()

    plot_iteration_times(iteration_times)

if __name__ == "__main__":
    main()
