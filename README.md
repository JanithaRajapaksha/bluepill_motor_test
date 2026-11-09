# Blue Pill Micromouse Drive Controller & Embedded Flood-Fill Solver

STM32CubeIDE firmware, an embedded autonomous flood-fill navigation solver, a PyQt6 2D visualization odometry GUI, a standalone terminal verification tool, and a hold-to-run keyboard utility for a two-wheel differential-drive micromouse built on the **STM32F103C8T6 (Blue Pill)**.

The firmware reads quadrature encoders on hardware timers, runs independent 1 kHz PI wheel-speed controllers, executes targeted 1-cell and 90°/180° discrete moves with closed-loop encoder correction and position hold, and embeds a zero-heap C flood-fill maze solver with microsecond timing.

---

## Recent Milestones & Implementation Log

A summary of all features and architectural improvements recently designed, implemented, and verified on the robot:

### 1. Firmware Modularization (`Core/`)
- **`motor_config.h`**: Centralized hardware pin definitions, encoder CPRs, discrete move tick targets (`CELL_TICKS = 1060`, `TURN_90_TICKS = 692`, `TURN_180_TICKS = 1384`), and 1 kHz PI gains.
- **`motor_controller.c/.h`**: Modularized low-level hardware actuation, PWM generation on `TIM1`, quadrature encoder sampling on `TIM2`/`TIM3`, 1 kHz speed control loop, and targeted position-hold moves.
- **`telemetry_cmd.c/.h`**: Decoupled UART command parser (supporting immediate teleop keys and line-based maze protocols) and 100 ms periodic telemetry formatter.
- **`main.c`**: Lean application entry point (~120 lines total).

### 2. Embedded C Flood-Fill Solver on STM32
- **Zero-Heap Navigation Engine**: Fully statically bounded C implementation (`sizeof(MazeMap) = 513` bytes, `sizeof(Navigator) = 517` bytes; zero dynamic allocations / `malloc = 0`).
  - `maze_map.c/.h`: Compact 1-byte-per-cell representation for mazes up to 16x16.
  - `floodfill_core.c/.h`: BFS shortest-path distance gradient solver using static FIFO queue buffers.
  - `sensor_interface.c/.h`: Standardized 3-channel (Front, Left, Right) relative obstacle detection.
  - `navigator.c/.h`: State machine emitting high-level navigation decisions (`MOVE_FORWARD`, `TURN_LEFT`, `TURN_RIGHT`, `TURN_AROUND`, `STOP`).
  - `mazes.c/.h`: Built-in competition benchmark presets (5x5, 6x6, 10x10, 16x16).
- **Microsecond Hardware Timing (`floodfill_runner.c/.h`)**:
  - Leverages the ARM Cortex-M3 **72 MHz DWT cycle counter** to time every single navigation step with sub-microsecond precision (~490–510 µs per step).

### 3. UART Maze Upload & Solving Protocol
- **`MAZE_START <size>`**: Initializes blank maze with outer perimeter walls.
- **`WALL <x> <y> <N|E|S|W>`**: Dynamically injects internal cell walls into STM32 RAM over UART.
- **`SOLVE`**: Solves the uploaded maze in firmware and streams live step telemetry.
- **Standalone Benchmark Keystrokes**: Single-character commands (`'6'`, `'1'`, `'2'`, `'5'`, `'h'`) for running built-in benchmark simulations in any serial terminal without PC-side software.

### 4. Headless Verification Tool (`verify_stm32_floodfill.py`)
- Standalone CLI script supporting `--maze 6` (default), `--maze 10`, `--maze 16`, `--maze 5`, `--bench`, and `--port COM5`.
- Reads golden maze layouts from the `FloodFill-Micromouse` submodule, uploads them to the Blue Pill over UART, triggers the solver, streams live steps, and asserts goal arrival and solve success.
- **Verified on Hardware**: Solved 6x6 maze in 5 steps (dynamic upload) / 19 steps (benchmark mode) with `[STATUS] SUCCESS`.

### 5. PyQt6 Modern GUI Package (`gui/` & `odometry_gui.py`)
- Rebuilt from legacy Tkinter into a modern, modular PyQt6 application:
  - `canvas.py`: Hardware-accelerated 2D viewport rendering competition red walls (`#ef4444`), white corner posts (`#f8fafc`), emerald start cell `(0,0)`, purple center goal cells, and real-time robot pose.
  - `sidebar.py`: Scrollable sidebar with serial connection, geometry calibration, maze environment controls, D-pad WASD teleop, and discrete moves.
  - `footer.py`: HUD metric cards (Pose, Odometry, Wheel RPM, Raw Encoders).
  - `serial_worker.py`: Non-blocking background `QThread` serial telemetry reader.
  - `maze_loader.py`: Submodule bridge importing preset mazes.
  - **"⚡ Solve on STM32"**: One-click button in GUI that uploads the maze, triggers the STM32 solver, and animates the robot traversing the maze cell-by-cell in real time.

---

## Hardware Pinout & Wiring

Target microcontroller: **STM32F103C8T6** (72 MHz, 64 KB Flash, 20 KB SRAM).  
Motor driver: Dual H-bridge with standby and PWM control per motor.  
Serial communications: **USART2** (`PA2`/`PA3`) @ **115200 baud** via CP210x USB-to-UART adapter.

| Function | MCU Pin | Peripheral | Project Assignment | Description |
| :--- | :---: | :--- | :--- | :--- |
| **Left Encoder A / B** | `PA0` / `PA1` | `TIM2 CH1 / CH2` | Left Wheel Encoder | Quadrature encoder input (4x decoding) |
| **Right Encoder A / B** | `PA6` / `PA7` | `TIM3 CH1 / CH2` | Right Wheel Encoder | Quadrature encoder input (inverted in software) |
| **Left Motor PWM** | `PA8` | `TIM1 CH1` | PWM A | 20 kHz PWM duty output (0–600 / 1000) |
| **Right Motor PWM** | `PA9` | `TIM1 CH2` | PWM B | 20 kHz PWM duty output (0–600 / 1000) |
| **Left Motor Direction** | `PB0` / `PB1` | GPIO Output | `AIN1` / `AIN2` | Direction logic (CH A) |
| **Right Motor Direction** | `PB10` / `PB11` | GPIO Output | `BIN1` / `BIN2` | Direction logic (CH B) |
| **Motor Driver Standby** | `PB13` | GPIO Output | `STBY` | Active-high enable |
| **USART2 TX** | `PA2` | `USART2_TX` | Serial Transmit | Transmit to PC adapter RX |
| **USART2 RX** | `PA3` | `USART2_RX` | Serial Receive | Receive from PC adapter TX |

The firmware assumes driver channel A is the left motor and channel B is the right motor. Positive motor commands use IN1 low and IN2 high; reverse commands use IN1 high and IN2 low. The actual physical forward direction depends on wiring.

> [!IMPORTANT]
> Connect the motor power supply ground, Blue Pill ground, and USB-to-UART adapter ground together. Never power DC motors directly from MCU 3.3V/5V rails.

---

## Encoder Calibration

`TIM2` is treated as the left wheel and `TIM3` as the right wheel. The right encoder count direction is inverted in software so forward movement increases both reported totals.

Measured over 20 wheel revolutions:

| Wheel | Measured counts/revolution |
| :--- | ---: |
| **Left** | 815.70 |
| **Right** | 815.55 |

These values are used to calculate wheel RPM and metric odometry:

| Parameter | Calibrated Value | Description |
| :--- | :---: | :--- |
| **Wheel Diameter** | `33.0 mm` | Calibrated rubber wheel diameter |
| **Track Width** | `112.0 mm` | Wheel contact center-to-center distance |
| **Cell Distance (`CELL_TICKS`)** | `1060 ticks` | Distance of 1 cell (~134.7 mm forward/backward) |
| **Turn 90° (`TURN_90_TICKS`)** | `692 ticks` | In-place pivot for 90° turn |
| **Turn 180° (`TURN_180_TICKS`)** | `1384 ticks` | In-place pivot for 180° turn |
| **Cell Pitch (Grid)** | `134.7 mm` | 1 Cell pitch in simulation and GUI |

The one-cell target is **1060 normalized encoder ticks per wheel** (~134.7 mm on 33 mm wheels). Calibrate it on your real floor by resetting counts, moving the robot exactly one cell, and updating `CELL_TICKS` in [`Core/Inc/motor_config.h`](Core/Inc/motor_config.h).

---

## Firmware Behavior & Control Loops

The STM32 SysTick runs at 1 kHz and calls the motor controller every 1 ms. Encoder counts are sampled each tick. RPM feedback uses a rolling 20 ms count window to reduce the quantization caused by low encoder counts per millisecond. Serial telemetry is printed separately every 100 ms.

### PI Speed Loop Settings

Configured in [`Core/Inc/motor_config.h`](Core/Inc/motor_config.h):

| Setting | Current Value | Purpose |
| :--- | ---: | :--- |
| **Normal wheel-speed target** | 150 RPM | Forward/reverse and pivot speed target |
| **Curve fast wheel target** | 165 RPM | Outer wheel target during curve maneuvers |
| **Curve slow wheel target** | 80 RPM | Inner wheel target during curve maneuvers |
| **`MOTOR_KP`** | 0.8 | Proportional speed correction |
| **`MOTOR_KI`** | 6.5 | Integral speed correction |
| **Normal feed-forward PWM** | 300 / 1000 | Starting PWM estimate (30%) |
| **Maximum PWM** | 600 / 1000 | 60% output safety limit |
| **Speed-loop period** | 1 ms | 1 kHz update |
| **RPM measurement window** | 20 ms | Encoder smoothing window |

Tune gains with the robot secured and wheels lifted first. If RPM oscillates or overshoots, reduce `MOTOR_KI`; do not increase the PWM cap unless the motor, driver, and power supply are rated for it.

### One-Cell Move and Position Hold

`F` (forward) and `B` (backward) request a targeted one-cell move. The firmware records each wheel's starting count, targets `CELL_TICKS` for both wheels, trims wheel-speed targets according to which wheel is ahead, and slows the wheels as they approach the target. Each wheel stops when it reaches its own target. When both targets are reached, the firmware enters encoder-count hold at the final counts.

While holding, the controller allows a **20 tick deadband**. If a wheel differs from its stored target by more than that, it corrects that wheel at a limited 35–60 RPM target using a lower 180/1000 feed-forward PWM. The hold remains active until `X`/space, a new movement command, or `R`. If a targeted move is not reached within 3 seconds, both motors stop and the firmware reports `CELL_TIMEOUT`.

### Encoder Telemetry Format

The firmware prints a line every 100 ms:

```text
L:<total> d:<change> RPM:<rpm> PWM:<duty> | R:<total> d:<change> RPM:<rpm> PWM:<duty>
```

- `total`: Accumulated normalized encoder count.
- `d`: Delta change since previous report.
- `RPM`: Measured wheel speed.
- `PWM`: Applied compare value (0 to 600).
- `R` resets displayed totals to zero without altering hardware timer registers.

---

## Complete Serial Command Reference

USART2 uses **115200 baud, 8 data bits, no parity, 1 stop bit**.

### 1. Manual Teleop & Curve Maneuvers
- **`W` / `w`**: Drive both wheels forward (150 RPM target).
- **`S` / `s`**: Drive both wheels backward (150 RPM target).
- **`A` / `a`**: In-place pivot left (left reverse, right forward).
- **`D` / `d`**: In-place pivot right (left forward, right reverse).
- **`Q` / `q`**: Forward-left curve (left 80 RPM, right 165 RPM).
- **`E` / `e`**: Forward-right curve (left 165 RPM, right 80 RPM).
- **`Z` / `z`**: Reverse-left curve (left -80 RPM, right -165 RPM).
- **`C` / `c`**: Reverse-right curve (left -165 RPM, right -80 RPM).

### 2. Autonomous Discrete Moves
- **`F` / `f`**: 1 Cell Forward (`+1060 ticks`).
- **`B` / `b`**: 1 Cell Backward (`-1060 ticks`).
- **`J` / `j`**: Turn 90° Left (`-692 / +692 ticks`).
- **`K` / `k`**: Turn 90° Right (`+692 / -692 ticks`).
- **`U` / `u`**: Turn 180° (`-1384 / +1384 ticks`).
- **`X` / `x` or Space**: Immediate motor stop & cancel active targeted move.
- **`R` / `r`**: Reset displayed encoder totals.

### 3. Interactive Firmware Benchmarks (Single Key)
- **`6`**: Run built-in 6x6 maze benchmark simulation.
- **`1`**: Run built-in 10x10 maze benchmark simulation.
- **`2`**: Run built-in 16x16 full competition maze benchmark.
- **`5`**: Run built-in 5x5 maze benchmark simulation.
- **`h`**: Print firmware interactive help menu.

### 4. Dynamic Maze Upload Protocol (Line-Based)
```text
MAZE_START <size>\r\n         -> STM32 responds: OK: MAZE_INIT <size>
WALL <x> <y> <N|E|S|W>\r\n    -> STM32 responds: OK: WALL
SOLVE\r\n                     -> Solves uploaded maze & streams step telemetry
BENCH <size>\r\n              -> Runs preset benchmark
```

---

## Codebase Architecture

```
bluepill_motor_test/
├── Core/                         # STM32F103 Firmware (STM32CubeIDE / Makefile)
│   ├── Inc/
│   │   ├── motor_config.h        # Calibration, speeds, discrete ticks, PI gains
│   │   ├── motor_controller.h    # Actuation, encoder sampling, 1 kHz PI loop
│   │   ├── telemetry_cmd.h       # UART command parser & 100 ms telemetry streamer
│   │   ├── floodfill_runner.h    # STM32 flood-fill execution harness (DWT timer)
│   │   ├── maze_map.h            # Static 513-byte 16x16 maze representation
│   │   ├── floodfill_core.h      # BFS distance gradient solver (0 heap / zero malloc)
│   │   ├── navigator.h           # State machine emitting high-level NavCommands
│   │   ├── sensor_interface.h    # 3-channel relative wall evaluator
│   │   └── mazes.h               # Benchmark layouts (5x5, 6x6, 10x10, 16x16)
│   └── Src/
│       ├── main.c                # Clean application entry point (~120 lines)
│       ├── motor_controller.c    # 1 kHz PI controller & targeted moves
│       ├── telemetry_cmd.c       # Teleop & line protocol parser
│       └── floodfill_runner.c    # Embedded solver execution & telemetry output
├── gui/                          # Modular PyQt6 Desktop Application Package
│   ├── config.py                 # Palette, constants, QSS stylesheets, regex patterns
│   ├── kinematics.py             # Differential-drive forward kinematics & dead reckoning
│   ├── canvas.py                 # QPainter 2D viewport (walls, pegs, start/goal, robot)
│   ├── sidebar.py                # Serial, geometry, maze controls, WASD & discrete moves
│   ├── footer.py                 # Real-time HUD metric cards
│   ├── serial_worker.py          # Background QThread for non-blocking serial telemetry
│   ├── maze_loader.py            # Submodule bridge loading mazes from FloodFill-Micromouse
│   └── main_window.py            # Main application window & live solver animation
├── FloodFill-Micromouse/         # Git Submodule (algorithm core, C lib, Python tests)
├── motor_keyboard.py             # Python hold-to-run keyboard utility
├── odometry_gui.py               # Main PyQt6 GUI launcher
└── verify_stm32_floodfill.py     # Standalone CLI maze upload & verification tool
```

---

## Standalone Terminal Verification Tool

[`verify_stm32_floodfill.py`](verify_stm32_floodfill.py) uploads any maze preset from the submodule to the STM32 over UART (**`COM5`** by default), triggers the embedded solver in firmware, streams real-time step telemetry, and verifies that the goal was reached.

```powershell
# Dynamic maze upload mode (default 6x6 on COM5)
.\.venv\Scripts\python.exe verify_stm32_floodfill.py --maze 6

# Built-in firmware benchmark mode
.\.venv\Scripts\python.exe verify_stm32_floodfill.py --maze 6 --bench

# Specify port or other maze sizes
.\.venv\Scripts\python.exe verify_stm32_floodfill.py --port COM5 --maze 10
```

### Verified Output on Physical Hardware
```text
==============================================================
   STM32 Embedded Flood-Fill Solver & Verification Tool       
==============================================================
Connecting to STM32 Blue Pill on COM5 @ 115200 baud...
Loaded maze from submodule: 6x6 (goal centers: [(2,2), (3,2), (2,3), (3,3)])

[1/3] Uploading 6x6 maze to STM32 (31 cell walls)...
  ✔ Maze initialized on STM32: 6x6
  ✔ Successfully uploaded 31 wall specifications to STM32 RAM.

[2/3] Triggering STM32 Flood-Fill Solver ('SOLVE')...
--- Streaming live telemetry from STM32 ---
  [STEP  1] Pose:( 0, 1) Dir:N | Act:MOVE_FORWARD | Time:493 us
  [STEP  2] Pose:( 0, 2) Dir:N | Act:MOVE_FORWARD | Time:514 us
  [STEP  3] Pose:( 1, 2) Dir:E | Act:TURN_RIGHT   | Time:517 us
  [STEP  4] Pose:( 2, 2) Dir:E | Act:MOVE_FORWARD | Time:514 us
  [STEP  5] Pose:( 2, 2) Dir:E | Act:STOP         | Time:505 us
  --------------------------------------------------
  [GOAL] Reached at (2, 2) in 5 steps!
  Execution Results:
  Total Time:       2543 us (2.543 ms)
  Avg per Step:     508 us
  Memory Footprint: sizeof(MazeMap)=513 bytes, sizeof(Navigator)=517 bytes
  [STATUS] SUCCESS

==============================================================
               STM32 VERIFICATION REPORT                      
==============================================================
  Maze Dimension:    6x6
  Goal Cells:        [(2, 3), (3, 2), (3, 3), (2, 2)]
  Steps Executed:    5 steps
  Final Position:    (2, 2) [GOAL]
  Firmware Status:   SUCCESS
  STM32 Solve Time:  2543 us (2.543 ms)
  Avg Time per Step: 508.00 us (Hardware 72 MHz DWT)
==============================================================
>>> VERIFICATION PASSED: STM32 FIRMWARE SUCCESSFULLY SOLVED THE MAZE! <<<
```

---

## PyQt6 Odometry & Maze GUI Application

Launch the desktop GUI:

```powershell
.\.venv\Scripts\python.exe odometry_gui.py
```

### Features
1. **Interactive 2D Viewport**:
   - Competition red walls (`#ef4444`) and white corner posts (`#f8fafc`).
   - Emerald green start cell highlight at `(0, 0)`.
   - Purple goal cell highlight for center 2x2 cluster.
   - Column/row numbers along the outer perimeter.
   - Smooth robot heading marker and continuous path drawing.
2. **MAZE ENVIRONMENT Controls**:
   - **Preset Selector**: Select `6x6 Test Maze`, `Empty 6x6 Maze`, `10x10`, `16x16`.
   - **Load Maze**: Renders maze layout in the viewport.
   - **Place at (0,0)**: Places robot at center of start cell `(0, 0)` facing North.
   - **⚡ Solve on STM32**: Uploads the maze to the STM32 over serial, triggers the embedded solver, and animates each step live on screen!
3. **Motion & Calibration Controls**:
   - D-Pad for manual teleop (`W`, `A`, `S`, `D`, `X`).
   - Discrete 1-cell moves (`F`, `B`) and turns (`J`, `K`, `U`).
   - CPR, wheel diameter, track width, and 1-cell distance (`Calc (F)`) calibration.
4. **HUD Metric Cards**:
   - Real-time pose ($X$, $Y$, $\theta$), linear/angular velocity, raw encoder totals, and wheel RPM.

---

## Python Hold-to-Run Keyboard Utility

`motor_keyboard.py` listens for PC keyboard press and release events, sends the corresponding firmware characters, and prints incoming telemetry. Hold `W/A/S/D` to drive; releasing the active movement key sends stop or resumes another held movement key. `F` is a one-shot cell command. Space or `X` stops immediately, `R` resets counts, and `Esc` exits after sending a final stop.

```powershell
py -m venv .venv
.venv\Scripts\Activate.ps1
py -m pip install -r requirements.txt
py motor_keyboard.py --port COM5
```

Close other serial terminals before running because only one application can use the COM port at a time.

---

## Building and Flashing Firmware

### Option A: Using STM32CubeIDE
1. Open STM32CubeIDE and import this folder via **File -> Open Projects from File System...**
2. Build the project (**Ctrl + B**).
3. Connect an ST-Link V2 to SWD pins (`GND`, `SWCLK`, `SWDIO`, `3.3V`).
4. Click **Run** or **Debug** to flash.

### Option B: Using Command Line (GNU ARM Toolchain)
```powershell
# 1. Build ELF and export BIN image
$toolDir = "C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740\tools\bin"
$makeExe = "C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.200.202604021615\tools\bin\make.exe"
$env:PATH = "$toolDir;$env:PATH"
Set-Location "Debug"
& $makeExe bluepill_motor_test.elf
& "$toolDir\arm-none-eabi-objcopy.exe" -O binary "bluepill_motor_test.elf" "bluepill_motor_test.bin"

# 2. Flash via STM32CubeProgrammer CLI
& "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" -c port=SWD -w "bluepill_motor_test.bin" 0x08000000 -v -rst
```

---

## Safety Guidelines

- **Bench Testing First**: Lift and secure the wheels for initial direction and PI testing.
- **Current-Limited Supply**: Use a current-limited, correctly rated motor power supply; share common ground with the MCU.
- **Emergency Stop**: The `X` key or Spacebar halts motor outputs immediately and cancels position holds.
- **Verification**: Verify motor direction, encoder polarity, counts/revolution, and one-cell ticks on the chassis before autonomous floor runs.
- **Pose Drift**: Encoder-only dead-reckoning accumulates error from tire slip, mechanical backlash, and floor irregularities. Physical wall sensors (IR/ToF) or a gyroscope should be combined with encoder odometry for competition-grade navigation.
