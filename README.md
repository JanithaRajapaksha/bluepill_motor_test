# Bluepill Motor Encoder Test

STM32CubeIDE project for testing two quadrature motor encoders on an STM32F103C8T6 (Blue Pill). The firmware reports encoder totals and count changes over USART2; it does not command the motors to move.

## Encoder connections

| Motor | Timer | Encoder channel 1 | Encoder channel 2 |
| --- | --- | --- | --- |
| Left | TIM2 | PA0 | PA1 |
| Right | TIM3 | PA6 | PA7 |

The firmware treats TIM2 as the left encoder and TIM3 as the right encoder. The right encoder's reported direction is inverted in software. If the physical motor assignment differs, swap the left/right labels or timer usage in `Core/Src/main.c`.

## Build and flash

1. Open STM32CubeIDE and import this folder as an existing project.
2. Select the `Debug` configuration and build the project.
3. Connect the Blue Pill with an ST-Link debugger and use **Run** or **Debug** to flash it.

The project is configured for the STM32F103C8Tx and uses the STM32F1 HAL drivers included under `Drivers/`.

## View encoder output

Connect a USB-to-UART adapter to USART2:

- PA2: USART2 TX to the adapter's RX
- PA3: USART2 RX to the adapter's TX (not needed for this output-only test)
- GND: common ground

Open a serial terminal at **115200 baud, 8 data bits, no parity, 1 stop bit**. A line is printed every 100 ms in this format:

```text
Left: <total> (d=<change>)  Right: <total> (d=<change>)
```

Rotate each wheel or encoder shaft by hand and check that its total changes. The `d` value is the signed count change over the latest reporting interval. The motor driver is not enabled by this test; ensure the wheels can be turned safely by hand.