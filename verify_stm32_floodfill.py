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

# Add submodule path so cell and mazes can be loaded directly
CURRENT_DIR = pathlib.Path(__file__).resolve().parent
SUBMODULE_PATH = CURRENT_DIR / "FloodFill-Micromouse" / "floodfill_py"
if str(SUBMODULE_PATH) not in sys.path and SUBMODULE_PATH.exists():
    sys.path.insert(0, str(SUBMODULE_PATH))

try:
    import cell as ff_cell
    import mazes as ff_mazes

    SUBMODULE_LOADED = True
except ImportError:
    SUBMODULE_LOADED = False


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


def normalize_maze_preset(preset: str | int) -> str:
    """Normalizes preset input (e.g. 6, '6', '6x6', '6X6') to clean dimension string ('6')."""
    s = str(preset).strip().lower()
    if "x" in s:
        s = s.split("x")[0].strip()
    return s


def load_maze(preset: str | int = "6") -> Any:
    """Loads a preset maze from the FloodFill-Micromouse submodule."""
    if not SUBMODULE_LOADED:
        raise RuntimeError(
            "Could not load FloodFill-Micromouse submodule. Ensure FloodFill-Micromouse/floodfill_py exists."
        )

    norm = normalize_maze_preset(preset)

    if norm == "6":
        maze = ff_cell.create_empty_maze(6)
        ff_mazes.wall_arrangement_x6(maze)
        return maze
    elif norm == "10":
        maze = ff_cell.create_empty_maze(10)
        ff_mazes.wall_arrangement_10x10(maze)
        return maze
    elif norm == "16":
        maze = ff_cell.create_empty_maze(16)
        ff_mazes.wall_arrangement_16x16(maze)
        return maze
    elif norm == "5":
        maze = ff_cell.create_empty_maze(5)
        ff_mazes.wall_arrangement_x5(maze)
        return maze
    else:
        raise ValueError(
            f"Unsupported maze preset: '{preset}'. Choices: 6 (or 6x6), 10 (or 10x10), 16 (or 16x16), 5 (or 5x5)"
        )


def query_cell_relative_walls(maze: Any, x: int, y: int, orientation: str) -> tuple[int, int, int]:
    """
    Returns (front, left, right) relative to the robot's current cardinal heading.
    Each value is 1 if a wall exists, 0 if open.
    The complete maze is held exclusively here in Python.
    """
    cell = None
    for row in maze.cells:
        for c in row:
            if c.x == x and c.y == y:
                cell = c
                break
        if cell is not None:
            break

    if cell is None:
        return (1, 1, 1)

    cw = {
        "N": 1 if cell.is_north_wall else 0,
        "E": 1 if cell.is_east_wall else 0,
        "S": 1 if cell.is_south_wall else 0,
        "W": 1 if cell.is_west_wall else 0,
    }
    # Heading -> (Front cardinal dir, Left cardinal dir, Right cardinal dir)
    hd = {
        "N": ("N", "W", "E"),
        "E": ("E", "N", "S"),
        "S": ("S", "E", "W"),
        "W": ("W", "S", "N"),
    }
    f_dir, l_dir, r_dir = hd.get(orientation.upper(), ("N", "W", "E"))
    return (cw[f_dir], cw[l_dir], cw[r_dir])


def run_blind_solve_and_verify(
    ser: serial.Serial,
    maze: Any,
    timeout: float = 60.0,
    with_motors: bool = False,
) -> tuple[bool, dict[str, Any]]:
    """
    Executes blind cell-by-cell maze navigation.
    Python acts as the maze information provider:
      1. Robot starts at (0,0) with no preloaded maze.
      2. Robot requests wall information (REQ_WALLS x y dir).
      3. Python queries the complete maze internally and sends relative WALLS F L R.
      4. Robot decides its direction, performs encoder-based movement.
      5. Robot reports cell arrival (ARRIVED x y dir).
      6. Python updates logical coordinates and returns new walls.
    """
    mode_str = "with physical motors" if with_motors else "simulation"
    print(f"\n{BOLD}[1/2] Starting Blind Cell-by-Cell Navigation ({maze.size}x{maze.size}, {mode_str})...{RESET}")
    print(f"  {YELLOW}• Complete maze kept strictly in Python memory (zero walls preloaded to STM32){RESET}")
    print(f"  {YELLOW}• Robot requests relative walls (F, L, R) upon arrival at each cell{RESET}")

    ser.reset_input_buffer()

    # Trigger blind solve on STM32
    cmd = f"BLIND_RUN {maze.size}\r\n".encode("ascii") if with_motors else f"BLIND_START {maze.size}\r\n".encode("ascii")
    ser.write(cmd)

    start_time = time.time()
    steps_received: list[str] = []
    goal_reached = False
    success = False
    final_pos: Optional[tuple[int, int]] = None
    hw_total_us: Optional[int] = None
    wall_queries_served = 0
    arrivals_confirmed = 0

    logical_x = 0
    logical_y = 0
    logical_dir = "N"

    centers = {(c.x, c.y) for c in maze.get_centers()}

    print(f"{DIM}--- Live Blind Navigation Handshake ---{RESET}")

    while (time.time() - start_time) < timeout:
        line = ser.readline().decode("latin-1", errors="replace").strip()
        if not line:
            continue

        if line.startswith("REQ_WALLS"):
            # Format: REQ_WALLS <x> <y> <dir>
            parts = line.split()
            rx = int(parts[1]) if len(parts) > 1 else logical_x
            ry = int(parts[2]) if len(parts) > 2 else logical_y
            r_dir = parts[3].upper() if len(parts) > 3 else logical_dir

            f, l, r = query_cell_relative_walls(maze, rx, ry, r_dir)
            ser.write(f"WALLS {f} {l} {r}\r\n".encode("ascii"))
            wall_queries_served += 1
            print(f"  {YELLOW}➔ [ORACLE]{RESET} Sensed walls at ({rx}, {ry}) Dir:{r_dir} -> {BOLD}Front:{f} Left:{l} Right:{r}{RESET}")

        elif line.startswith("ARRIVED"):
            # Format: ARRIVED <x> <y> <dir>
            parts = line.split()
            logical_x = int(parts[1]) if len(parts) > 1 else logical_x
            logical_y = int(parts[2]) if len(parts) > 2 else logical_y
            logical_dir = parts[3].upper() if len(parts) > 3 else logical_dir
            arrivals_confirmed += 1
            final_pos = (logical_x, logical_y)
            print(f"  {GREEN}✔ [ORACLE]{RESET} Robot completed encoder travel to cell ({logical_x}, {logical_y}) facing {logical_dir} (Arrival #{arrivals_confirmed})")

        elif "[STEP" in line:
            print(f"  {CYAN}{line}{RESET}")
            steps_received.append(line)
        elif ">> Motors:" in line:
            print(f"  {YELLOW}{BOLD}{line}{RESET}")
        elif "[GOAL]" in line:
            print(f"  {GREEN}{BOLD}{line}{RESET}")
            goal_reached = True
            try:
                coords_part = line.split("at (")[1].split(")")[0]
                gx, gy = map(int, coords_part.split(","))
                final_pos = (gx, gy)
            except Exception:
                pass
        elif "[STATUS] SUCCESS" in line:
            print(f"  {GREEN}{BOLD}{line}{RESET}")
            success = True
            break
        elif "[STATUS] FAILED" in line or "[FAILED]" in line:
            print(f"  {RED}{BOLD}{line}{RESET}")
            success = False
            break
        elif "Total Compute:" in line:
            print(f"  {line}")
            try:
                hw_total_us = int(line.split(":")[1].split("us")[0].strip())
            except Exception:
                pass
        elif "===" in line or "---" in line or "Execution Results" in line:
            print(f"  {DIM}{line}{RESET}")

    print(f"{DIM}--- End of Blind Navigation stream ---{RESET}")

    is_valid_goal = final_pos in centers if final_pos is not None else False
    passed = goal_reached and success and is_valid_goal and wall_queries_served > 0

    stats = {
        "step_count": len(steps_received),
        "goal_reached": goal_reached,
        "final_pos": final_pos,
        "valid_goal_center": is_valid_goal,
        "target_centers": list(centers),
        "firmware_status": "SUCCESS" if success else "FAILED",
        "hw_total_us": hw_total_us,
        "hw_avg_us": (hw_total_us / len(steps_received)) if (hw_total_us and steps_received) else None,
        "wall_queries_served": wall_queries_served,
        "arrivals_confirmed": arrivals_confirmed,
    }

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
