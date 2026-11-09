#!/usr/bin/env python3
"""
STM32 Micromouse Blind Flood-Fill Verification Tool
===================================================
Executes blind cell-by-cell maze navigation with the STM32 microcontroller.
Python acts as the maze information provider:
  1. The complete maze layout is held exclusively in Python (never preloaded to STM32).
  2. Robot starts at (0, 0) knowing only the boundaries and target goal center.
  3. Robot requests wall data (REQ_WALLS x y dir) upon arrival at each cell.
  4. Python inspects the cell and sends relative walls (WALLS <front> <left> <right>).
  5. Robot plans its next step, drives motors by encoder ticks, and reports arrival (ARRIVED).

Usage:
    python verify_stm32_floodfill.py [--port COM5] [--maze 6] [--motors]
"""

from __future__ import annotations

import argparse
import pathlib
import sys
import time
from typing import Any, Optional

import serial
import serial.tools.list_ports

from navigation_engine import (
    NavigationEngine,
    load_maze,
    normalize_maze_preset,
    query_cell_relative_walls,
)

# ANSI color codes for terminal output
GREEN = "\033[92m"
CYAN = "\033[96m"
YELLOW = "\033[93m"
RED = "\033[91m"
BOLD = "\033[1m"
DIM = "\033[2m"
RESET = "\033[0m"


def print_banner() -> None:
    print(f"{CYAN}{BOLD}=============================================================={RESET}")
    print(f"{CYAN}{BOLD}   STM32 Micromouse Blind Cell-by-Cell Flood-Fill Tool        {RESET}")
    print(f"{CYAN}{BOLD}=============================================================={RESET}")


def get_default_port() -> str:
    """Returns COM5 if available, or first detected USB serial port."""
    ports = [p.device for p in serial.tools.list_ports.comports()]
    if "COM5" in ports:
        return "COM5"
    for p in serial.tools.list_ports.comports():
        if "bluetooth" not in p.description.lower() and "stlink" not in p.description.lower():
            return p.device
    return "COM5"


def run_blind_solve_and_verify(
    ser: serial.Serial,
    maze: Any,
    timeout: float = 60.0,
    with_motors: bool = False,
) -> tuple[bool, dict[str, Any]]:
    """
    Executes blind cell-by-cell maze navigation using the unified NavigationEngine.
    """
    mode_str = "with physical motors" if with_motors else "simulation"
    print(f"\n{BOLD}[1/2] Starting Blind Cell-by-Cell Navigation ({maze.size}x{maze.size}, {mode_str})...{RESET}")
    print(f"  {YELLOW}• Powered by Unified NavigationEngine (shared with PyQt6 GUI){RESET}")
    print(f"  {YELLOW}• Complete maze kept strictly in Python memory (zero walls preloaded to STM32){RESET}")
    print(f"  {YELLOW}• Robot requests relative walls (F, L, R) upon arrival at each cell{RESET}")
    print(f"{DIM}--- Live Blind Navigation Handshake ---{RESET}")

    engine = NavigationEngine(maze=maze, with_motors=with_motors)

    def cli_log_handler(kind: str, text: str) -> None:
        if kind == "oracle":
            parts = text.split()
            rx = int(parts[1]) if len(parts) > 1 else engine.logical_x
            ry = int(parts[2]) if len(parts) > 2 else engine.logical_y
            r_dir = parts[3].upper() if len(parts) > 3 else engine.logical_dir
            f, l, r = query_cell_relative_walls(maze, rx, ry, r_dir)
            print(f"  {YELLOW}➔ [ORACLE]{RESET} Sensed walls at ({rx}, {ry}) Dir:{r_dir} -> {BOLD}Front:{f} Left:{l} Right:{r}{RESET}")
        elif kind == "arrived":
            parts = text.split()
            rx = int(parts[1]) if len(parts) > 1 else engine.logical_x
            ry = int(parts[2]) if len(parts) > 2 else engine.logical_y
            r_dir = parts[3].upper() if len(parts) > 3 else engine.logical_dir
            print(f"  {GREEN}✔ [ORACLE]{RESET} Robot completed encoder travel to cell ({rx}, {ry}) facing {r_dir} (Arrival #{engine.arrivals_confirmed})")
        elif kind == "step":
            print(f"  {CYAN}{text}{RESET}")
        elif kind == "motor":
            print(f"  {YELLOW}{BOLD}{text}{RESET}")
        elif kind == "success":
            print(f"  {GREEN}{BOLD}{text}{RESET}")
        elif kind == "fail":
            print(f"  {RED}{BOLD}{text}{RESET}")
        elif kind == "timing":
            print(f"  {text}")

    passed, stats = engine.run_blocking(ser, timeout=timeout, on_log=cli_log_handler)
    print(f"{DIM}--- End of Blind Navigation stream ---{RESET}")
    return passed, stats


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Verify STM32 blind cell-by-cell micromouse navigation."
    )
    parser.add_argument(
        "--port",
        default=None,
        help="Serial COM port connected to STM32 USART1 (default: auto-detect COM5)",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Baud rate (default: 115200)",
    )
    parser.add_argument(
        "--maze",
        default="6",
        choices=["6", "10", "16", "5", "6x6", "10x10", "16x16", "5x5"],
        help="Maze dimension/preset: 6 (default), 10, 16, 5 (or 6x6, 10x10, etc.)",
    )
    parser.add_argument(
        "--motors",
        action="store_true",
        help="Actuate physical motors in real time during maze solving (sends BLIND_RUN)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=None,
        help="Timeout in seconds (default: 20.0s for sim, 90.0s for motors)",
    )

    args = parser.parse_args()
    print_banner()

    port = args.port or get_default_port()
    baud = args.baud
    maze_preset = args.maze
    with_motors = args.motors

    if args.timeout is not None:
        timeout = args.timeout
    else:
        timeout = 90.0 if with_motors else 20.0

    print(f"Connecting to STM32 Blue Pill on {BOLD}{port}{RESET} @ {baud} baud...")

    try:
        ser = serial.Serial(port, baud, timeout=1.0)
    except serial.SerialException as exc:
        print(f"\n{RED}{BOLD}Failed to open {port}:{RESET} {exc}")
        print("\nTroubleshooting tips:")
        print(f"  • Ensure the CP210x / USB-UART adapter is plugged in and recognized as {port}.")
        print("  • If the odometry GUI or serial monitor is open, close it to free the port.")
        print("  • Check available COM ports:")
        for p in serial.tools.list_ports.comports():
            print(f"      - {p.device}: {p.description}")
        return 1

    time.sleep(0.3)

    try:
        # Load maze in Python memory only
        maze = load_maze(maze_preset)
        print(f"Loaded maze in Python: {BOLD}{maze.size}x{maze.size}{RESET} (goal centers: {maze.get_centers()})")

        passed, stats = run_blind_solve_and_verify(ser, maze, timeout=timeout, with_motors=with_motors)

        # Print Final Verification Summary
        print(f"\n{CYAN}{BOLD}=============================================================={RESET}")
        print(f"{CYAN}{BOLD}               STM32 VERIFICATION REPORT                      {RESET}")
        print(f"{CYAN}{BOLD}=============================================================={RESET}")
        print(f"  Maze Dimension:       {maze.size}x{maze.size}")
        print(f"  Navigation Mode:      BLIND (Cell-by-cell relative wall discovery)")
        print(f"  Goal Cells:           {stats['target_centers']}")
        print(f"  Steps Executed:       {stats['step_count']} steps")
        print(f"  Wall Requests Served: {stats['wall_queries_served']}")
        print(f"  Arrivals Confirmed:   {stats['arrivals_confirmed']}")
        print(f"  Final Position:       {stats['final_pos']} " + (f"{GREEN}[GOAL]{RESET}" if stats['valid_goal_center'] else f"{RED}[NOT GOAL]{RESET}"))
        print(f"  Firmware Status:      {GREEN if stats['firmware_status'] == 'SUCCESS' else RED}{stats['firmware_status']}{RESET}")

        if stats.get("hw_total_us") is not None:
            print(f"  STM32 Solve Time:     {stats['hw_total_us']} us ({stats['hw_total_us']/1000.0:.3f} ms)")
        if stats.get("hw_avg_us") is not None:
            print(f"  Avg Time per Step:    {stats['hw_avg_us']:.2f} us (Hardware 72 MHz DWT)")

        print(f"{CYAN}{BOLD}=============================================================={RESET}")

        if passed:
            print(f"{GREEN}{BOLD}>>> VERIFICATION PASSED: STM32 FIRMWARE SUCCESSFULLY SOLVED THE MAZE! <<<{RESET}\n")
            return 0
        else:
            print(f"{RED}{BOLD}>>> VERIFICATION FAILED: STM32 DID NOT REACH GOAL OR ENCOUNTERED ERROR <<<{RESET}\n")
            return 1

    finally:
        ser.close()


if __name__ == "__main__":
    sys.exit(main())
