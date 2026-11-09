# Blue Pill Micromouse Drive & Flood-Fill Controller

STM32F103C8T6 (Blue Pill) firmware and companion Python tooling for a two-wheel differential-drive Micromouse. The firmware features independent 1 kHz PI wheel-speed control loops, encoder-based targeted maneuvers (1 cell, 90°/180° turns), real-time odometry telemetry streaming over USART2, and a **fully embedded C Flood-Fill maze solver** with zero dynamic memory allocation.

---

## What's New

### 1. Embedded C Flood-Fill Maze Solver on STM32
The BFS shortest-path flood-fill algorithm from [`FloodFill-Micromouse`](FloodFill-Micromouse) is now built directly into the STM32 microcontroller firmware:
- **Zero Dynamic Allocations**: Operates completely in statically-allocated buffers (`sizeof(MazeMap) = 513 bytes`, `sizeof(Navigator) = 517 bytes`), preventing stack/heap collisions.
- **Hardware Execution Timing**: Uses the ARM Cortex-M3 **72 MHz DWT cycle counter** to time every navigation decision in microseconds (~500 µs/step).
- **UART Maze Upload Protocol**: Upload custom mazes on the fly using line commands:
  - `MAZE_START <size>`: Sets boundary walls for an $N \times N$ maze.
  - `WALL <x> <y> <dirs>`: Sets internal walls (e.g., `WALL 2 0 N`, `WALL 4 0 E`).
  - `SOLVE`: Runs the flood-fill navigator, evaluates simulated sensors, and streams live telemetry until the goal is reached.
- **Built-in Benchmark Shortcuts**: Fast single-key triggers for serial terminals:
  - `'6'`: Run 6x6 test maze benchmark
  - `'1'`: Run 10x10 maze benchmark
  - `'2'`: Run 16x16 full competition maze benchmark
  - `'5'`: Run 5x5 maze benchmark
  - `'h'`: Print interactive help menu

### 2. Standalone Verification Tool (`verify_stm32_floodfill.py`)
A terminal-only Python verification tool that:
- Loads preset mazes (default: 6x6) from the `FloodFill-Micromouse` submodule.
- Connects to the STM32 over UART (default: `COM5` @ 115200 baud).
- Uploads the maze walls over serial and commands the firmware to solve.
- Streams step-by-step telemetry in real time.
- Verifies goal reach, execution step count, timing, and firmware success status.

Run it:
```powershell
# Dynamic maze upload mode (default 6x6 on COM5)
.\.venv\Scripts\python.exe verify_stm32_floodfill.py --maze 6

# Built-in firmware benchmark mode
.\.venv\Scripts\python.exe verify_stm32_floodfill.py --maze 6 --bench
```

### 3. Modularized PyQt6 Odometry & Maze GUI (`odometry_gui.py` / `gui/`)
The desktop visualization app was redesigned into a modern, modular PyQt6 package:
- `gui/canvas.py`: Hardware-accelerated 2D viewport rendering robot path, grid, and full Micromouse maze (floor, start/goal cell tints, competition red walls, white pegs, and axis indices).
- `gui/sidebar.py`: Scrollable sidebar with serial connection, geometry calibration, WASD teleop D-pad, discrete motion buttons, and maze environment controls.
- `gui/footer.py`: HUD metric cards (Pose X/Y/Heading, Encoders, Speeds).
- `gui/kinematics.py`: Differential drive forward kinematics (`DifferentialDriveOdometry`).
- `gui/maze_loader.py`: Submodule bridge to load test mazes directly from `FloodFill-Micromouse`.

Run the GUI:
```powershell
.\.venv\Scripts\python.exe odometry_gui.py
```

### 4. Modular Firmware Architecture (`Core/`)
The firmware is split into clean, modular components:
- `Core/Inc/motor_config.h`: Calibration constants, tick geometry (1 cell, 90°/180° turns), PI parameters, target speeds.
- `Core/Inc/motor_controller.h` & `Core/Src/motor_controller.c`: Dual-channel H-Bridge motor actuation, encoder sampling, 1 kHz PI speed control loop, targeted moves.
- `Core/Inc/telemetry_cmd.h` & `Core/Src/telemetry_cmd.c`: UART command dispatching and 100 ms streaming.
- `Core/Inc/floodfill_runner.h` & `Core/Src/floodfill_runner.c`: Embedded flood-fill execution harness with DWT microsecond timer.
- `Core/Src/main.c`: Clean, lean application entry point (~120 lines total).

---

## Hardware Pinout

| Function | MCU Pin | Assignment |
| :--- | :--- | :--- |
| Left encoder channel A/B | PA0 / PA1 | TIM2 CH1 / CH2 |
| Right encoder channel A/B | PA6 / PA7 | TIM3 CH1 / CH2 |
| Left motor PWM | PA8 | TIM1 CH1 |
| Right motor PWM | PA9 | TIM1 CH2 |
| Left motor direction | PB0 / PB1 | AIN1 / AIN2 |
| Right motor direction | PB10 / PB11 | BIN1 / BIN2 |
| Motor driver standby | PB13 | STBY (HIGH enables driver) |
| Serial transmit | PA2 | USART2 TX (Connect to Adapter RX) |
| Serial receive | PA3 | USART2 RX (Connect to Adapter TX) |

---

## Encoder Calibration

Measured over 20 wheel revolutions:
- **Left CPR**: `815.70`
- **Right CPR**: `815.55`
- **1 Cell Pitch**: `1060` normalized encoder ticks per wheel (~134.7 mm with 33 mm wheels)
- **90° Turn**: `692` ticks differential
- **180° Turn**: `1384` ticks differential

---

## UART Commands (USART2 @ 115200 Baud)

### Manual Teleop & Discrete Actions
| Command | Action |
| :--- | :--- |
| `W` / `S` | Continuous drive forward / backward at 150 RPM |
| `A` / `D` | Continuous pivot left / right |
| `Q` / `E` | Smooth forward-left / forward-right curves |
| `Z` / `C` | Smooth reverse-left / reverse-right curves |
| `F` | Autonomous 1-cell forward move (`CELL_TICKS`) |
| `B` | Autonomous 1-cell backward move (`-CELL_TICKS`) |
| `J` | Autonomous 90° left turn (`-TURN_90_TICKS`, `+TURN_90_TICKS`) |
| `K` | Autonomous 90° right turn (`+TURN_90_TICKS`, `-TURN_90_TICKS`) |
| `U` | Autonomous 180° turn (`-TURN_180_TICKS`, `+TURN_180_TICKS`) |
| `X` or Space | Stop both motors immediately and cancel moves |
| `R` | Reset displayed encoder totals |

### Flood-Fill Benchmark & Upload Commands
| Command | Action |
| :--- | :--- |
| `'6'` | Run 6x6 maze benchmark simulation |
| `'1'` | Run 10x10 maze benchmark simulation |
| `'2'` | Run 16x16 full competition maze benchmark |
| `'5'` | Run 5x5 maze benchmark simulation |
| `'h'` | Display interactive menu |
| `MAZE_START <size>` | Clear map and set perimeter walls for $N \times N$ maze |
| `WALL <x> <y> <dirs>` | Add internal wall (e.g. `WALL 2 0 N`, `WALL 4 0 E`) |
| `SOLVE` | Solve uploaded maze and stream live step telemetry |

---

## Build & Flash

### Option 1: Command Line (Makefile + STM32CubeProgrammer)
```powershell
# Build firmware
$toolDir = "C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740\tools\bin"
$makeExe = "C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.200.202604021615\tools\bin\make.exe"
$env:PATH = "$toolDir;$env:PATH"
Set-Location "Debug"
& $makeExe bluepill_motor_test.elf
& "$toolDir\arm-none-eabi-objcopy.exe" -O binary "bluepill_motor_test.elf" "bluepill_motor_test.bin"

# Flash via ST-Link
& "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" -c port=SWD -w "bluepill_motor_test.bin" 0x08000000 -v -rst
```

### Option 2: STM32CubeIDE
1. Open STM32CubeIDE and open this folder.
2. Select the `Debug` configuration and click **Build**.
3. Connect an ST-Link and click **Run** or **Debug**.
