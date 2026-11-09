"""Helper module to load and configure Micromouse mazes from the FloodFill-Micromouse submodule."""

from __future__ import annotations

import pathlib
import sys
from typing import Any, Optional

# Locate the submodule directory containing cell.py and mazes.py
_CURRENT_DIR = pathlib.Path(__file__).resolve().parent
_WORKSPACE_ROOT = _CURRENT_DIR.parent
_SUBMODULE_DIR = _WORKSPACE_ROOT / "FloodFill-Micromouse" / "floodfill_py"

if str(_SUBMODULE_DIR) not in sys.path and _SUBMODULE_DIR.exists():
    sys.path.insert(0, str(_SUBMODULE_DIR))

try:
    import cell as ff_cell
    import mazes as ff_mazes

    SUBMODULE_AVAILABLE = True
except ImportError:
    ff_cell = None
    ff_mazes = None
    SUBMODULE_AVAILABLE = False


def is_available() -> bool:
    """Returns True if the FloodFill-Micromouse submodule is importable."""
    return SUBMODULE_AVAILABLE and ff_cell is not None and ff_mazes is not None


def load_empty_maze(size: int = 6) -> Any:
    """Creates a blank maze with boundary perimeter walls."""
    if not is_available():
        raise RuntimeError("FloodFill-Micromouse submodule is not available.")
    return ff_cell.create_empty_maze(size)


def load_test_maze_6x6() -> Any:
    """Loads the preset 6x6 test maze."""
    if not is_available():
        raise RuntimeError("FloodFill-Micromouse submodule is not available.")
    maze = ff_cell.create_empty_maze(6)
    ff_mazes.wall_arrangement_x6(maze)
    return maze


def load_test_maze_10x10() -> Any:
    """Loads the preset 10x10 test maze."""
    if not is_available():
        raise RuntimeError("FloodFill-Micromouse submodule is not available.")
    maze = ff_cell.create_empty_maze(10)
    ff_mazes.wall_arrangement_10x10(maze)
    return maze


def load_test_maze_16x16() -> Any:
    """Loads the standard 16x16 competition maze."""
    if not is_available():
        raise RuntimeError("FloodFill-Micromouse submodule is not available.")
    maze = ff_cell.create_empty_maze(16)
    ff_mazes.wall_arrangement_16x16(maze)
    return maze


MAZE_PRESETS = {
    "6x6 Test Maze": load_test_maze_6x6,
    "Empty 6x6 Maze": lambda: load_empty_maze(6),
    "10x10 Test Maze": load_test_maze_10x10,
    "16x16 Competition Maze": load_test_maze_16x16,
}


def load_preset(name: str) -> Any:
    """Loads a maze by preset name."""
    loader = MAZE_PRESETS.get(name)
    if loader is None:
        raise ValueError(f"Unknown maze preset '{name}'. Choices: {list(MAZE_PRESETS.keys())}")
    return loader()
