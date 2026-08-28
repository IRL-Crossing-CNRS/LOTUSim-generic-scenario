#!/usr/bin/env python3
"""
@file log_run_csv.py
@brief Standalone CLI for the CsvRecorder observer (simulation_run.csv_recorder).

Run it on the host or on any machine sharing the ROS 2 network, alongside any
scenario — it spawns nothing and observes everything:

    python3 log_run_csv.py --world energy [--outdir csv_logs] [--rate 2.0]

For the integrated host-side equivalent, set ``"record_csv": true`` in the
scenario JSON instead (see simulation_run/csv_recorder.py).
"""

import argparse
import os
import time

import rclpy

from simulation_run.csv_recorder import CsvRecorder


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Record agent poses (+battery) of a LOTUSim world, one CSV per agent."
    )
    parser.add_argument("--world", required=True, help="World name, e.g. 'energy'")
    parser.add_argument(
        "--outdir",
        default=None,
        help="Output directory (default: csv_logs_<world>_<timestamp> in the current dir)",
    )
    parser.add_argument(
        "--prefix", default="", help="Filename prefix for the per-agent CSVs"
    )
    parser.add_argument(
        "--rate", type=float, default=2.0, help="Sampling rate in Hz (default 2.0)"
    )
    args = parser.parse_args()

    outdir = args.outdir or os.path.join(
        os.getcwd(), f"csv_logs_{args.world}_{time.strftime('%Y-%m-%d_%H-%M-%S')}"
    )

    rclpy.init()
    node = CsvRecorder(args.world, outdir, args.prefix, args.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(f"CSV files written in {outdir}/")


if __name__ == "__main__":
    main()
