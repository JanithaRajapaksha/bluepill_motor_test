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
 * @brief Initialize a fresh blank maze of given size with perimeter walls.
 */
void Floodfill_Runner_Init_Maze(uint8_t size);

/**
 * @brief Add a wall to the uploaded ground-truth maze.
 */
bool Floodfill_Runner_Set_Wall(uint8_t x, uint8_t y, Direction dir);

/**
 * @brief Load one of the preset benchmark mazes (5, 6, 10, or 16).
 */
bool Floodfill_Runner_Build_Preset(uint8_t size);

/**
 * @brief Execute flood-fill solver on the current ground-truth maze and stream telemetry.
 * @param max_steps Maximum number of steps before aborting (e.g. 200).
 * @return true if goal reached, false otherwise.
 */
bool Floodfill_Runner_Solve(uint16_t max_steps);

/**
 * @brief Run a preset benchmark (5, 6, 10, or 16) directly.
 */
bool Floodfill_Runner_Benchmark(uint8_t size);

/**
 * @brief Print interactive help menu over UART.
 */
void Floodfill_Runner_Print_Menu(void);

#ifdef __cplusplus
}
#endif

#endif /* FLOODFILL_RUNNER_H */
