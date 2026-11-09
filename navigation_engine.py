"""
Unified Micromouse Navigation & Oracle Engine
============================================
Shared core engine powering both the PyQt6 GUI (odometry_gui.py) and
the standalone headless CLI verification tool (verify_stm32_floodfill.py).

Centralizes:
  1. Complete ground-truth maze management (kept strictly in Python).
  2. Relative wall sensing oracle (Front, Left, Right).
  3. UART line protocol parsing (REQ_WALLS, WALLS, ARRIVED, [STEP], [GOAL], [STATUS]).
  4. Cell discovery tracking for fog-of-war visualization.
  5. Synchronous / blocking execution runner for CLI verification.
"""

from __future__ import annotations

import pathlib
import re
import sys
import time
from typing import Any, Callable, Optional

# Locate submodule path for cell and mazes
CURRENT_DIR = pathlib.Path(__file__).resolve().parent
SUBMODULE_PATH = CURRENT_DIR / "FloodFill-Micromouse" / "floodfill_py"
if str(SUBMODULE_PATH) not in sys.path and SUBMODULE_PATH.exists():
    sys.path.insert(0, str(SUBMODULE_PATH))

try:
    import cell as ff_cell
    import mazes as ff_mazes

    SUBMODULE_LOADED = True
except ImportError:
    ff_cell = None
    ff_mazes = None
    SUBMODULE_LOADED = False

# Telemetry regex patterns
TELEMETRY_PATTERN = re.compile(
    r"L:(-?\d+)\s+d:(-?\d+)\s+RPM:(-?\d+)\s+PWM:(\d+)\s+\|\s+"
    r"R:(-?\d+)\s+d:(-?\d+)\s+RPM:(-?\d+)\s+PWM:(\d+)"
)

FLOODFILL_STEP_PATTERN = re.compile(
    r"\[STEP\s+(\d+)\]\s+Pose:\(\s*(\d+),\s*(\d+)\)\s+Dir:([NESW])\s+\|\s+Act:(\S+)"
)


def normalize_maze_preset(preset: str | int) -> str:
    """Normalizes preset input (e.g. 6, '6', '6x6', '6X6') to clean dimension string ('6')."""
    s = str(preset).strip().lower()
    if "x" in s:
        s = s.split("x")[0].strip()
    return s


def load_maze(preset: str | int = "6") -> Any:
    """Loads a preset maze from the FloodFill-Micromouse submodule."""
    if not SUBMODULE_LOADED or ff_cell is None or ff_mazes is None:
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
    if maze is None:
        return (1, 1, 1)

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


class NavigationEngine:
    """
    Unified Navigation Engine managing the blind cell-by-cell maze solving session.
    Operates identically in both the PyQt6 GUI and the headless CLI tool.
    """

    def __init__(self, maze: Optional[Any] = None, with_motors: bool = False) -> None:
        self.maze: Optional[Any] = maze
        self.with_motors: bool = with_motors

        # Navigation state
        self.logical_x: int = 0
        self.logical_y: int = 0
        self.logical_dir: str = "N"

        # Discovery & metrics
        self.discovered_cells: set[tuple[int, int]] = {(0, 0)}
        self.wall_queries_served: int = 0
        self.arrivals_confirmed: int = 0
        self.steps_received: list[dict[str, Any]] = []

        # Status flags
        self.is_active: bool = False
        self.goal_reached: bool = False
        self.success: bool = False
        self.final_pos: Optional[tuple[int, int]] = None
        self.hw_total_us: Optional[int] = None
        self.start_time: float = 0.0

        # Optional callbacks for GUI / CLI decoupling
        self.on_wall_request: Optional[Callable[[int, int, str, int, int, int], None]] = None
        self.on_cell_arrival: Optional[Callable[[int, int, str], None]] = None
        self.on_step: Optional[Callable[[dict[str, Any]], None]] = None
        self.on_goal: Optional[Callable[[tuple[int, int]], None]] = None
        self.on_status: Optional[Callable[[bool, str], None]] = None
        self.on_motor_action: Optional[Callable[[str], None]] = None
        self.on_telemetry: Optional[Callable[[tuple[int, ...]], None]] = None
        self.on_log: Optional[Callable[[str], None]] = None

    def set_maze(self, maze: Optional[Any]) -> None:
        """Sets the active ground-truth maze."""
        self.maze = maze
        self.reset()

    def reset(self) -> None:
        """Resets the engine state back to initial conditions at cell (0, 0)."""
        self.logical_x = 0
        self.logical_y = 0
        self.logical_dir = "N"
        self.discovered_cells = {(0, 0)}
        self.wall_queries_served = 0
        self.arrivals_confirmed = 0
        self.steps_received.clear()
        self.is_active = False
        self.goal_reached = False
        self.success = False
        self.final_pos = None
        self.hw_total_us = None
        self.start_time = 0.0

    def start_solve(self, maze: Optional[Any] = None, with_motors: bool | None = None) -> str:
        """
        Prepares the engine for a new solve session and returns the UART start command string.
        """
        if maze is not None:
            self.maze = maze
        if with_motors is not None:
            self.with_motors = with_motors

        self.reset()
        self.is_active = True
        self.start_time = time.time()

        size = self.maze.size if self.maze is not None else 6
        cmd = f"BLIND_RUN {size}\r\n" if self.with_motors else f"BLIND_START {size}\r\n"
        return cmd

    def get_goal_centers(self) -> set[tuple[int, int]]:
        """Returns the set of goal center coordinates for the current maze."""
        if self.maze is None:
            return {(2, 2), (2, 3), (3, 2), (3, 3)}
        try:
            return {(c.x, c.y) for c in self.maze.get_centers()}
        except Exception:
            half = self.maze.size // 2
            return {(half - 1, half - 1), (half - 1, half), (half, half - 1), (half, half)}

    def process_line(self, line: str) -> Optional[str]:
        """
        Processes a raw incoming line from the STM32 Blue Pill.
        If the line requires a response (e.g. REQ_WALLS -> WALLS), returns the response string.
        Otherwise updates state, invokes relevant callbacks, and returns None.
        """
        line_clean = line.strip()
        if not line_clean:
            return None

        # 1. Robot Wall Request: REQ_WALLS <x> <y> <dir>
        if line_clean.startswith("REQ_WALLS"):
            parts = line_clean.split()
            rx = int(parts[1]) if len(parts) > 1 else self.logical_x
            ry = int(parts[2]) if len(parts) > 2 else self.logical_y
            r_dir = parts[3].upper() if len(parts) > 3 else self.logical_dir

            self.logical_x = rx
            self.logical_y = ry
            self.logical_dir = r_dir
            self.discovered_cells.add((rx, ry))

            f, l, r = query_cell_relative_walls(self.maze, rx, ry, r_dir)
            self.wall_queries_served += 1

            if self.on_wall_request:
                self.on_wall_request(rx, ry, r_dir, f, l, r)

            response = f"WALLS {f} {l} {r}\r\n"
            return response

        # 2. Robot Cell Arrival Confirmation: ARRIVED <x> <y> <dir>
        if line_clean.startswith("ARRIVED"):
            parts = line_clean.split()
            rx = int(parts[1]) if len(parts) > 1 else self.logical_x
            ry = int(parts[2]) if len(parts) > 2 else self.logical_y
            r_dir = parts[3].upper() if len(parts) > 3 else self.logical_dir

            self.logical_x = rx
            self.logical_y = ry
            self.logical_dir = r_dir
            self.final_pos = (rx, ry)
            self.discovered_cells.add((rx, ry))
            self.arrivals_confirmed += 1

            if self.on_cell_arrival:
                self.on_cell_arrival(rx, ry, r_dir)

            return None

        # 3. Solver Step Telemetry: [STEP ...]
        step_match = FLOODFILL_STEP_PATTERN.search(line_clean)
        if step_match:
            step_dict = {
                "step_num": int(step_match.group(1)),
                "col": int(step_match.group(2)),
                "row": int(step_match.group(3)),
                "dir": step_match.group(4),
                "action": step_match.group(5),
                "raw_line": line_clean,
            }
            self.steps_received.append(step_dict)
            if self.on_step:
                self.on_step(step_dict)
            return None

        # 4. Motor Action Log: >> Motors: ...
        if ">> Motors:" in line_clean:
            if self.on_motor_action:
                self.on_motor_action(line_clean)
            return None

        # 5. Goal Reached: [GOAL]
        if "[GOAL]" in line_clean:
            self.goal_reached = True
            try:
                coords_part = line_clean.split("at (")[1].split(")")[0]
                gx, gy = map(int, coords_part.split(","))
                self.final_pos = (gx, gy)
            except Exception:
                pass
            if self.on_goal and self.final_pos is not None:
                self.on_goal(self.final_pos)
            return None

        # 6. Status Success: [STATUS] SUCCESS
        if "[STATUS] SUCCESS" in line_clean:
            self.success = True
            self.is_active = False
            if self.on_status:
                self.on_status(True, "SUCCESS")
            return None

        # 7. Status Failed: [STATUS] FAILED or [FAILED]
        if "[STATUS] FAILED" in line_clean or "[FAILED]" in line_clean:
            self.success = False
            self.is_active = False
            if self.on_status:
                self.on_status(False, "FAILED")
            return None

        # 8. Timing Measurement
        if "Total Compute:" in line_clean or "Total Time:" in line_clean:
            try:
                self.hw_total_us = int(line_clean.split(":")[1].split("us")[0].strip())
            except Exception:
                pass
            return None

        # 9. Motor Encoder Telemetry: L:... | R:...
        match_telem = TELEMETRY_PATTERN.search(line_clean)
        if match_telem:
            parsed = tuple(int(v) for v in match_telem.groups())
            if self.on_telemetry:
                self.on_telemetry(parsed)
            return None

        return None

    def get_summary(self) -> dict[str, Any]:
        """Returns verification statistics dictionary."""
        centers = self.get_goal_centers()
        is_valid_goal = self.final_pos in centers if self.final_pos is not None else False
        passed = self.goal_reached and self.success and is_valid_goal and self.wall_queries_served > 0

        avg_us = (self.hw_total_us / len(self.steps_received)) if (self.hw_total_us and self.steps_received) else None

        return {
            "passed": passed,
            "step_count": len(self.steps_received),
            "goal_reached": self.goal_reached,
            "final_pos": self.final_pos,
            "valid_goal_center": is_valid_goal,
            "target_centers": list(centers),
            "firmware_status": "SUCCESS" if self.success else ("FAILED" if not self.is_active else "RUNNING"),
            "hw_total_us": self.hw_total_us,
            "hw_avg_us": avg_us,
            "wall_queries_served": self.wall_queries_served,
            "arrivals_confirmed": self.arrivals_confirmed,
            "elapsed_s": time.time() - self.start_time if self.start_time > 0 else 0.0,
        }

    def run_blocking(
        self,
        ser: Any,
        timeout: float = 60.0,
        on_log: Optional[Callable[[str, str], None]] = None,
    ) -> tuple[bool, dict[str, Any]]:
        """
        Executes a blocking solve loop using serial port `ser`.
        Used by verify_stm32_floodfill.py.
        """
        cmd = self.start_solve(self.maze, self.with_motors)
        ser.reset_input_buffer()
        ser.write(cmd.encode("ascii"))

        start = time.time()
        while (time.time() - start) < timeout:
            raw_line = ser.readline()
            if not raw_line:
                continue

            line = raw_line.decode("latin-1", errors="replace").strip()
            if not line:
                continue

            # Process line through unified engine
            response = self.process_line(line)

            # Send response if required (e.g. WALLS ...)
            if response is not None:
                ser.write(response.encode("ascii"))

            # Log to caller if handler registered
            if on_log:
                if line.startswith("REQ_WALLS"):
                    on_log("oracle", line)
                elif line.startswith("ARRIVED"):
                    on_log("arrived", line)
                elif "[STEP" in line:
                    on_log("step", line)
                elif ">> Motors:" in line:
                    on_log("motor", line)
                elif "[GOAL]" in line or "[STATUS] SUCCESS" in line:
                    on_log("success", line)
                elif "[STATUS] FAILED" in line or "[FAILED]" in line:
                    on_log("fail", line)
                elif "Total Compute:" in line:
                    on_log("timing", line)

            if not self.is_active and (self.success or self.goal_reached):
                break

        stats = self.get_summary()
        return stats["passed"], stats
