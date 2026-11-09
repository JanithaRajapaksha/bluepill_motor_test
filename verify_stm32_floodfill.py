#!/usr/bin/env python3
"""
STM32 Micromouse Flood-Fill Verification Tool
=============================================
Uploads a maze from the FloodFill-Micromouse submodule to the STM32 microcontroller
over UART (default: COM5), triggers the embedded flood-fill solver in firmware,
streams real-time step telemetry, and verifies that the goal was successfully reached.

Usage:
    python verify_stm32_floodfill.py [--port COM5] [--baud 115200] [--maze 6x6]
    python verify_stm32_floodfill.py --bench 6
"""

from __future__ import annotations

import argparse
import os
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
    print(f"{CYAN}{BOLD}   STM32 Embedded Flood-Fill Solver & Verification Tool       {RESET}")
    print(f"{CYAN}{BOLD}=============================================================={RESET}")


def get_default_port() -> str:
    """Returns COM5 if available, or first detected USB serial port."""
    ports = [p.device for p in serial.tools.list_ports.comports()]
    if "COM5" in ports:
        return "COM5"
    # Fallback to any non-Bluetooth COM port if COM5 not listed
    for p in serial.tools.list_ports.comports():
        if "bluetooth" not in p.description.lower() and "stlink" not in p.description.lower():
            return p.device
    return "COM5"


def load_maze(preset: str = "6x6") -> Any:
    """Loads a preset maze from the FloodFill-Micromouse submodule."""
    if not SUBMODULE_LOADED:
        raise RuntimeError(
            "Could not load FloodFill-Micromouse submodule. Ensure FloodFill-Micromouse/floodfill_py exists."
        )

    if preset in ("6", "6x6"):
        maze = ff_cell.create_empty_maze(6)
        ff_mazes.wall_arrangement_x6(maze)
        return maze
    elif preset in ("10", "10x10"):
        maze = ff_cell.create_empty_maze(10)
        ff_mazes.wall_arrangement_10x10(maze)
        return maze
    elif preset in ("16", "16x16"):
        maze = ff_cell.create_empty_maze(16)
        ff_mazes.wall_arrangement_16x16(maze)
        return maze
    elif preset in ("5", "5x5"):
        maze = ff_cell.create_empty_maze(5)
        ff_mazes.wall_arrangement_x5(maze)
        return maze
    else:
        raise ValueError(f"Unsupported maze preset: {preset}. Choices: 5x5, 6x6, 10x10, 16x16")


def extract_internal_walls(maze: Any) -> list[tuple[int, int, str]]:
    """Extracts internal walls for each cell (excluding outer perimeter walls)."""
    wall_list: list[tuple[int, int, str]] = []
    size = maze.size

    for row in maze.cells:
        for c in row:
            x, y = c.x, c.y
            dirs = []
            # Only send internal walls (perimeter is automatically set by MAZE_START)
            if c.is_north_wall and y < size - 1:
                dirs.append("N")
            if c.is_east_wall and x < size - 1:
                dirs.append("E")
            if c.is_south_wall and y > 0:
                dirs.append("S")
            if c.is_west_wall and x > 0:
                dirs.append("W")

            if dirs:
                wall_list.append((x, y, "".join(dirs)))

    return wall_list


def upload_maze_to_stm32(
    ser: serial.Serial,
    maze: Any,
    verbose: bool = True,
) -> bool:
    """Uploads the maze geometry to the STM32 via line protocol."""
    size = maze.size
    walls = extract_internal_walls(maze)

    if verbose:
        print(f"\n{BOLD}[1/3] Uploading {size}x{size} maze to STM32 ({len(walls)} cell walls)...{RESET}")

    # Flush input buffer
    ser.reset_input_buffer()

    # 1. Send MAZE_START <size>
    cmd_init = f"MAZE_START {size}\r\n".encode("ascii")
    ser.write(cmd_init)
    time.sleep(0.04)

    # Read response
    resp = ser.read(ser.in_waiting or 128).decode("ascii", errors="replace")
    if "OK: MAZE_INIT" not in resp:
        # Retry once
        ser.write(cmd_init)
        time.sleep(0.08)
        resp = ser.read(ser.in_waiting or 128).decode("ascii", errors="replace")
        if "OK: MAZE_INIT" not in resp:
            print(f"{RED}Error: STM32 did not acknowledge MAZE_START. Response: {resp.strip()}{RESET}")
            return False

    if verbose:
        print(f"  {GREEN}✔{RESET} Maze initialized on STM32: {size}x{size}")

    # 2. Upload internal walls
    uploaded = 0
    for x, y, dirs in walls:
        wall_cmd = f"WALL {x} {y} {dirs}\r\n".encode("ascii")
        ser.write(wall_cmd)
        time.sleep(0.015)
        uploaded += 1

    time.sleep(0.05)
    # Clear any residual ACKs
    ser.reset_input_buffer()

    if verbose:
        print(f"  {GREEN}✔{RESET} Successfully uploaded {uploaded} wall specifications to STM32 RAM.")

    return True


def solve_and_verify(
    ser: serial.Serial,
    maze: Any,
    timeout: float = 12.0,
) -> tuple[bool, dict[str, Any]]:
    """Triggers the STM32 firmware solver and verifies execution."""
    print(f"\n{BOLD}[2/3] Triggering STM32 Flood-Fill Solver ('SOLVE')...{RESET}")

    # Clear input buffer before solve
    ser.reset_input_buffer()

    # Send SOLVE command
    ser.write(b"SOLVE\r\n")

    start_time = time.time()
    steps_received: list[str] = []
    goal_reached = False
    success = False
    final_pos: Optional[tuple[int, int]] = None
    hw_total_us: Optional[int] = None
    hw_avg_us: Optional[float] = None

    centers = {(c.x, c.y) for c in maze.get_centers()}

    print(f"{DIM}--- Streaming live telemetry from STM32 ---{RESET}")

    while (time.time() - start_time) < timeout:
        line = ser.readline().decode("latin-1", errors="replace").strip()
        if not line:
            continue

        # Print all lines from STM32
        if "[STEP" in line:
            print(f"  {CYAN}{line}{RESET}")
            steps_received.append(line)
        elif "[GOAL]" in line:
            print(f"  {GREEN}{BOLD}{line}{RESET}")
            goal_reached = True
            # Extract coordinates from e.g. "[GOAL] Reached at (3, 3) in 18 steps!"
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
        elif "Total Time:" in line:
            print(f"  {line}")
            try:
                hw_total_us = int(line.split(":")[1].split("us")[0].strip())
            except Exception:
                pass
        elif "Avg per Step:" in line:
            print(f"  {line}")
            try:
                hw_avg_us = float(line.split(":")[1].split("us")[0].strip())
            except Exception:
                pass
        elif "===" in line or "---" in line or "Execution Results" in line or "Memory Footprint" in line:
            print(f"  {DIM}{line}{RESET}")

    print(f"{DIM}--- End of STM32 stream ---{RESET}")

    # Verification criteria:
    # 1. Steps were recorded
    # 2. Goal reached flag is true
    # 3. Final position is within target centers
    # 4. Status was SUCCESS
    is_valid_goal = final_pos in centers if final_pos is not None else False
    passed = goal_reached and success and is_valid_goal and len(steps_received) > 0

    stats = {
        "step_count": len(steps_received),
        "goal_reached": goal_reached,
        "final_pos": final_pos,
        "valid_goal_center": is_valid_goal,
        "target_centers": list(centers),
        "firmware_status": "SUCCESS" if success else "FAILED",
        "hw_total_us": hw_total_us,
        "hw_avg_us": hw_avg_us,
    }

    return passed, stats


def run_benchmark_preset(
    ser: serial.Serial,
    size: int = 6,
    timeout: float = 12.0,
) -> tuple[bool, dict[str, Any]]:
    """Runs a built-in benchmark directly on the STM32 via single key or BENCH command."""
    print(f"\n{BOLD}[1/2] Triggering built-in {size}x{size} benchmark on STM32...{RESET}")
    ser.reset_input_buffer()

    # Send benchmark key (e.g. '6' for 6x6, '1' for 10x10, '2' for 16x16)
    if size == 6:
        ser.write(b"6\r\n")
    elif size == 10:
        ser.write(b"1\r\n")
    elif size == 16:
        ser.write(b"2\r\n")
    elif size == 5:
        ser.write(b"5\r\n")
    else:
        ser.write(f"BENCH {size}\r\n".encode("ascii"))

    # Reuse solver listener with preset maze
    maze = load_maze(str(size))
    return solve_and_verify(ser, maze, timeout=timeout)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Upload maze to STM32 and verify embedded flood-fill solver."
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
        default="6x6",
        choices=["5x5", "6x6", "10x10", "16x16", "5", "6", "10", "16"],
        help="Maze preset to upload and solve (default: 6x6)",
    )
    parser.add_argument(
        "--bench",
        action="store_true",
        help="Run firmware preset benchmark instead of dynamic upload",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="Timeout in seconds (default: 10.0s)",
    )

    args = parser.parse_args()
    print_banner()

    port = args.port or get_default_port()
    baud = args.baud
    maze_preset = args.maze

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
        # Load maze from FloodFill-Micromouse
        maze = load_maze(maze_preset)
        print(f"Loaded maze from submodule: {BOLD}{maze.size}x{maze.size}{RESET} (goal centers: {maze.get_centers()})")

        if args.bench:
            passed, stats = run_benchmark_preset(ser, size=maze.size, timeout=args.timeout)
        else:
            # 1. Upload maze
            if not upload_maze_to_stm32(ser, maze):
                print(f"{RED}Maze upload failed.{RESET}")
                return 1

            # 2. Solve and verify
            passed, stats = solve_and_verify(ser, maze, timeout=args.timeout)

        # 3. Print Final Verification Summary
        print(f"\n{CYAN}{BOLD}=============================================================={RESET}")
        print(f"{CYAN}{BOLD}               STM32 VERIFICATION REPORT                      {RESET}")
        print(f"{CYAN}{BOLD}=============================================================={RESET}")
        print(f"  Maze Dimension:    {maze.size}x{maze.size}")
        print(f"  Goal Cells:        {stats['target_centers']}")
        print(f"  Steps Executed:    {stats['step_count']} steps")
        print(f"  Final Position:    {stats['final_pos']} " + (f"{GREEN}[GOAL]{RESET}" if stats['valid_goal_center'] else f"{RED}[NOT GOAL]{RESET}"))
        print(f"  Firmware Status:   {GREEN if stats['firmware_status'] == 'SUCCESS' else RED}{stats['firmware_status']}{RESET}")

        if stats["hw_total_us"] is not None:
            print(f"  STM32 Solve Time:  {stats['hw_total_us']} us ({stats['hw_total_us']/1000.0:.3f} ms)")
        if stats["hw_avg_us"] is not None:
            print(f"  Avg Time per Step: {stats['hw_avg_us']:.2f} us (Hardware 72 MHz DWT)")

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
