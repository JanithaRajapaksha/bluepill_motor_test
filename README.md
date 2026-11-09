# Blue Pill Micromouse Drive Controller

STM32CubeIDE firmware and a Windows Python keyboard utility for a two-wheel differential-drive micromouse. The firmware reads both quadrature encoders, runs independent PI wheel-speed controllers, accepts movement commands over USART2, and can execute a one-cell forward move with encoder-based wheel correction and position hold.

## Hardware

The project targets an STM32F103C8T6 (Blue Pill), an H-bridge motor driver with two direction inputs and one PWM input per motor, two quadrature encoders, and a motor power supply appropriate for the motors and driver. Do not power motors directly from the MCU pins. Connect the motor-driver ground, MCU ground, and USB-to-UART adapter ground together.

| Function | MCU pin or peripheral | Project assignment |
| --- | --- | --- |
| Left encoder channel A/B | PA0 / PA1 | TIM2 CH1 / CH2 |
| Right encoder channel A/B | PA6 / PA7 | TIM3 CH1 / CH2 |
| Left motor PWM | PA8 | TIM1 CH1 |
| Right motor PWM | PA9 | TIM1 CH2 |
| Left motor direction | PB0 / PB1 | AIN1 / AIN2 |
| Right motor direction | PB10 / PB11 | BIN1 / BIN2 |
| Motor driver standby | PB13 | STBY, high enables driver |
| Serial transmit | PA2 | USART2 TX |
| Serial receive | PA3 | USART2 RX |

The firmware assumes driver channel A is the left motor and channel B is the right motor. Positive motor commands use IN1 low and IN2 high; reverse commands use IN1 high and IN2 low. The actual physical forward direction depends on wiring. Swap the direction states or channel assignments in `Apply_Motor_Output` if necessary.

## Encoder calibration

TIM2 is treated as the left wheel and TIM3 as the right wheel. The right encoder count direction is inverted in software so forward movement should increase both reported totals.

Measured over 20 wheel revolutions:

| Wheel | Measured counts/revolution |
| --- | ---: |
| Left | 815.70 |
| Right | 815.55 |

These are the values used to calculate wheel RPM. The current one-cell target is **1060 normalized encoder ticks per wheel**. This value is an initial assumption, not yet validated against the physical maze cell pitch and floor. Calibrate it by resetting the counts, moving the robot exactly one cell on the real floor, and recording both wheel deltas. Update `CELL_TICKS` in `Core/Src/main.c` after calibration.

## Firmware behavior

The STM32 SysTick runs at 1 kHz and calls the motor controller every 1 ms. Encoder counts are sampled each tick. RPM feedback uses a rolling 20 ms count window to reduce the quantization caused by low encoder counts per millisecond. Serial telemetry is printed separately every 100 ms.

Each wheel has its own PI speed loop. Current settings in `Core/Src/main.c` are:

| Setting | Current value | Purpose |
| --- | ---: | --- |
| Normal wheel-speed target | 150 RPM | Forward/reverse and pivot speed target |
| `MOTOR_KP` | 0.8 | Proportional speed correction |
| `MOTOR_KI` | 6.5 | Integral speed correction |
| Normal feed-forward PWM | 300 / 1000 | Starting PWM estimate |
| Maximum PWM | 600 / 1000 | 60% output limit |
| Speed-loop period | 1 ms | 1 kHz update |
| RPM measurement window | 20 ms | Encoder smoothing window |

Treat these values as initial tuning settings. Tune with the robot secured and wheels lifted first. If RPM oscillates or overshoots, reduce `MOTOR_KI`; do not increase the PWM cap unless the motor, driver, and power supply are rated for it.

### Serial commands

USART2 uses **115200 baud, 8 data bits, no parity, 1 stop bit**. Firmware commands are single characters:

| Command | Action |
| --- | --- |
| `W` | Drive both wheels forward at the 150 RPM target |
| `S` | Drive both wheels backward at the 150 RPM target |
| `A` | Pivot left: left reverse, right forward |
| `D` | Pivot right: left forward, right reverse |
| `F` | Start a one-cell forward move |
| `X` or space | Stop both motors and cancel cell move/hold |
| `R` | Reset displayed encoder totals and cancel position hold; ignored during an active `F` move |

Commands are case-insensitive. `W`, `S`, `A`, and `D` remain active until another command arrives. The motors start stopped after reset or power-up.

### One-cell move and encoder hold

`F` requests a one-cell forward move. The firmware records each wheel's starting count, targets `CELL_TICKS` for both wheels, trims wheel-speed targets according to which wheel is ahead, and slows the wheels as they approach the target. Each wheel stops when it reaches its own target. When both targets are reached, the firmware enters encoder-count hold at the final counts.

While holding, the controller allows a **20 tick deadband**. If a wheel differs from its stored target by more than that, it corrects that wheel at a limited 35-60 RPM target using a lower 180/1000 feed-forward PWM. The hold remains active until `X`/space, a new movement command, or `R`. If the one-cell target is not reached within 3 seconds, both motors stop and the firmware reports `CELL_TIMEOUT`.

Encoder hold only observes wheel rotation. It cannot distinguish an intentional wheel turn from wheel slip, and it cannot measure sideways displacement or absolute chassis position. Corrections can move the chassis. Wall sensors and/or a gyro are needed for robust pose correction; test encoder hold carefully before using it on a maze.

### Encoder telemetry

The firmware prints a line every 100 ms:

```text
L:<total> d:<change> RPM:<rpm> PWM:<duty> | R:<total> d:<change> RPM:<rpm> PWM:<duty>
```

`total` is the accumulated normalized count, `d` is the change since the preceding report, `RPM` is measured wheel speed, and `PWM` is the applied compare value from 0 to 600. When accepted, `R` resets displayed totals to zero; it does not reset the hardware encoder timers.

## Python hold-to-run keyboard utility

`motor_keyboard.py` listens for PC keyboard press and release events, sends the corresponding firmware characters, and prints incoming telemetry. Hold `W/A/S/D` to drive; releasing the active movement key sends stop or resumes another movement key that remains held. `F` is a one-shot cell command and is not cancelled when the key is released. Space or `X` stops immediately, `R` resets counts, and `Esc` exits after sending a final stop command.

Connect a USB-to-UART adapter with TX/RX crossed and a common ground:

- Adapter RX to PA2 (MCU USART2 TX)
- Adapter TX to PA3 (MCU USART2 RX, required for control)
- Adapter GND to MCU/driver GND

Close other serial-terminal applications before starting the Python utility because the COM port can only be opened by one program at a time. On Windows, install and run it from PowerShell:

```powershell
py -m venv .venv
.venv\Scripts\Activate.ps1
py -m pip install -r requirements.txt
py motor_keyboard.py --port COM5
```

Replace `COM5` with the adapter's actual COM port. `--baud` may be specified if the firmware baud rate is changed; it defaults to 115200. The script requires the board to be flashed and connected. If the keyboard listener cannot access global keyboard events, run it from a normal desktop session with the required Windows permissions.

## Standalone odometry GUI

`odometry_gui.py` is a separate Tkinter application; it does not extend or launch `motor_keyboard.py`. It reads the same USART2 telemetry and displays a cell grid, the path traveled, the robot's current heading, encoder totals, and measured wheel speeds. Position is shown in millimeters, with heading in degrees clockwise from north. The GUI also provides **Forward one cell**, **Stop**, and **Reset pose + encoders** buttons.

Run it from the project virtual environment:

```powershell
py odometry_gui.py
```

In the GUI, choose the board's COM port and baud rate. The measured counts/revolution values are prefilled. Enter the actual wheel diameter and the distance between the left and right wheel contact centers (track width); these are required for metric odometry and heading. The grid-cell size defaults to 180 mm as a display reference and can be changed to match the maze. Connect after entering the dimensions.

The pose uses differential-drive odometry: each encoder's count change is converted to wheel travel using counts/revolution and wheel circumference; the difference between wheel travels updates heading using track width. The drawn path is dead-reckoned from that pose. **Reset pose + encoders** sends `X`, waits briefly for the current move to stop, sends `R` to reset the firmware's accumulated encoder totals, then starts the GUI pose at `(0, 0, 0)`. `F` starts the firmware's one-cell move and `X` stops it. Close `motor_keyboard.py` or any other serial monitor before connecting; only one application can use the COM port at a time.

Encoder-only odometry accumulates error from wheel-diameter/track calibration, tire slip, uneven floors, and mechanical play. The display is a dead-reckoned estimate, not an absolute position sensor or a full RViz replacement. Use wall sensors or a gyro to correct accumulated pose drift for reliable maze navigation.

## Build and flash

1. Open STM32CubeIDE and import this folder as an existing project.
2. Select the `Debug` configuration and build the project.
3. Connect an ST-Link debugger and use **Run** or **Debug** to flash the Blue Pill.
4. Connect the motor supply and USB-to-UART adapter, then start `motor_keyboard.py` or use another serial terminal to send commands.

The STM32F1 HAL and CMSIS sources are included under `Drivers/`. Generated `Debug/`, `Release/`, and common Python cache/virtual-environment artifacts are excluded by `.gitignore`.

## Safety and verification

- Lift and secure the wheels for initial direction and PI testing.
- Use a current-limited, correctly rated motor supply and driver; share ground with the MCU.
- Keep the stop command available. `X` stops the motors and cancels encoder hold.
- Verify motor direction, encoder polarity, counts/revolution, and one-cell ticks on the actual chassis before autonomous maze use.
- Host GCC syntax checks do not replace an STM32CubeIDE ARM build or hardware validation. PI gains, hold response, and the 1060-tick cell target still require testing on the robot.