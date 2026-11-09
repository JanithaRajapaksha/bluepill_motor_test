/**
  ******************************************************************************
  * @file           : floodfill_runner.h
  * @brief          : Standalone Flood-Fill navigation & benchmark harness for STM32
  ******************************************************************************
  */

#ifndef FLOODFILL_RUNNER_H
#define FLOODFILL_RUNNER_H

#include "stm32f1xx_hal.h"
#include "maze_map.h"
#include "navigator.h"
#include "mazes.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the FloodFill runner with UART handle and DWT timer.
 */
void Floodfill_Runner_Init(UART_HandleTypeDef *huart);

/**
 * @brief Run blind cell-by-cell navigation requesting walls from Python over UART.
 * @param size Dimension of the maze (e.g. 6).
 * @param actuate_motors true to physically drive motors via encoders, false for simulated encoder ticks.
 * @param max_steps Safety step limit.
 * @return true if goal reached, false otherwise.
 */
bool Floodfill_Runner_Solve_Blind(uint8_t size, bool actuate_motors, uint16_t max_steps);

/**
 * @brief Print interactive help menu over UART.
 */
void Floodfill_Runner_Print_Menu(void);

#ifdef __cplusplus
}
#endif

#endif /* FLOODFILL_RUNNER_H */
