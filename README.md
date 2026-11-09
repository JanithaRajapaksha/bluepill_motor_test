# Bluepill Motor Encoder Test

STM32CubeIDE project for controlling two DC motors and reading their quadrature encoders on an STM32F103C8T6 (Blue Pill). Encoder totals and count changes stream continuously over USART2 while the motors are controlled with serial keyboard keys.

## Encoder connections

| Motor | Timer | Encoder channel 1 | Encoder channel 2 |
| --- | --- | --- | --- |
| Left | TIM2 | PA0 | PA1 |
| Right | TIM3 | PA6 | PA7 |

The firmware treats TIM2 as the left encoder and TIM3 as the right encoder. The right encoder's reported direction is inverted in software. If the physical motor assignment differs, swap the left/right labels or timer usage in `Core/Src/main.c`.

## Keyboard motor control

Connect a USB-to-UART adapter to USART2 and open a serial terminal at **115200 baud, 8 data bits, no parity, 1 stop bit**. Configure it to send keystrokes immediately rather than waiting for Enter, then send these keys:

| Key | Action |
| --- | --- |
| W | Drive both motors forward |
| S | Drive both motors backward |
| A | Turn left in place |
| D | Turn right in place |
| Space or X | Stop both motors |
| R | Reset both displayed encoder totals to zero |

Commands remain active until another command is received. Motors start stopped. The control uses a fixed 30% PWM duty. The assumed forward direction is IN1 low and IN2 high for each motor; actual wheel direction depends on motor wiring. Raise the wheels clear of the surface and secure the chassis before powering the motor supply.

The controls assume driver channel A (AIN1/AIN2, TIM1 CH1) is the left motor and channel B (BIN1/BIN2, TIM1 CH2) is the right motor. Swap those assignments in `Set_Motor_Speed` if the wiring is reversed.

## Build and flash

1. Open STM32CubeIDE and import this folder as an existing project.
2. Select the `Debug` configuration and build the project.
3. Connect the Blue Pill with an ST-Link debugger and use **Run** or **Debug** to flash it.

The project is configured for the STM32F103C8Tx and uses the STM32F1 HAL drivers included under `Drivers/`.

## View encoder output

Connect a USB-to-UART adapter to USART2:

- PA2: USART2 TX to the adapter's RX
- PA3: USART2 RX to the adapter's TX (required for keyboard control)
- GND: common ground

The firmware prints a line every 100 ms, including while motors are moving:

```text
Left: <total> (d=<change>)  Right: <total> (d=<change>)
```

The `d` value is the signed count change over the latest reporting interval. Encoder counting is handled by the STM32 timer peripherals and continues while the main loop polls for motor-control keys.

## Hold-to-run Python controller

The Python controller sends motor commands on key-down and sends stop when the movement key is released. It also displays the encoder lines received from the board. Close other serial-monitor applications before starting it because only one application can use the COM port at a time.

On Windows, install Python dependencies and start the script with the COM port assigned to your USB-to-UART adapter:

```powershell
py -m venv .venv
.venv\Scripts\Activate.ps1
py -m pip install -r requirements.txt
py motor_keyboard.py --port COM5
```

Replace `COM5` with the correct port. Hold **W/A/S/D** to drive; releasing the active key stops, or resumes another movement key that is still held. **Space/X** stops immediately, **R** resets the encoder totals, and **Esc** exits and sends a final stop command. The controller assumes the firmware is flashed and the board is connected on USART2 at 115200 baud. Keep the wheels raised while testing.