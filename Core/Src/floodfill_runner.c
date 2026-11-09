/**
  ******************************************************************************
  * @file           : floodfill_runner.c
  * @brief          : Standalone Flood-Fill navigation & benchmark harness for STM32
  ******************************************************************************
  */

#include "floodfill_runner.h"
#include <stdio.h>
#include <string.h>

static UART_HandleTypeDef *s_huart = NULL;
static MazeMap s_ground_truth;
static bool s_maze_ready = false;

static void uart_puts(const char *str)
{
  if (s_huart != NULL && str != NULL)
  {
    HAL_UART_Transmit(s_huart, (uint8_t *)str, (uint16_t)strlen(str), HAL_MAX_DELAY);
  }
}

void Floodfill_Runner_Init(UART_HandleTypeDef *huart)
{
  s_huart = huart;

  /* Enable DWT Cycle Counter for high-resolution microsecond measurement */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* Pre-populate with default 6x6 test maze */
  Floodfill_Runner_Build_Preset(6);
}

void Floodfill_Runner_Init_Maze(uint8_t size)
{
  if (size < 2 || size > MICROMOUSE_MAX_MAZE_SIZE)
  {
    uart_puts("ERR: Invalid size (must be 2-16)\r\n");
    return;
  }

  maze_map_init_empty(&s_ground_truth, size);
  s_maze_ready = true;

  char buf[64];
  snprintf(buf, sizeof(buf), "OK: MAZE_INIT %u (%ux%u)\r\n", size, size, size);
  uart_puts(buf);
}

bool Floodfill_Runner_Set_Wall(uint8_t x, uint8_t y, Direction dir)
{
  if (!s_maze_ready || x >= s_ground_truth.size || y >= s_ground_truth.size || dir >= DIR_COUNT)
  {
    return false;
  }

  maze_map_set_wall(&s_ground_truth, x, y, dir);
  return true;
}

bool Floodfill_Runner_Build_Preset(uint8_t size)
{
  if (!build_preset_maze(&s_ground_truth, size))
  {
    return false;
  }
  s_maze_ready = true;
  return true;
}

bool Floodfill_Runner_Solve(uint16_t max_steps)
{
  if (s_huart == NULL || !s_maze_ready)
  {
    uart_puts("ERR: No maze loaded\r\n[STATUS] FAILED\r\n");
    return false;
  }

  uint8_t size = s_ground_truth.size;
  char buf[384];

  snprintf(buf, sizeof(buf),
           "\r\n==================================================\r\n"
           "  STM32 Flood-Fill Solver: %ux%u Maze Test\r\n"
           "==================================================\r\n",
           size, size);
  uart_puts(buf);

  Navigator nav;
  navigator_init(&nav, size, 0, 0, DIR_NORTH);

  uint32_t total_solve_us = 0;
  uint16_t step_count = 0;
  const char dir_labels[] = {'N', 'E', 'S', 'W'};

  while (step_count < max_steps)
  {
    /* 1. Simulate 3-sensor array against ground-truth maze */
    bool f = maze_map_has_wall(&s_ground_truth, nav.x, nav.y, nav.orientation);
    bool l = maze_map_has_wall(&s_ground_truth, nav.x, nav.y, (Direction)((nav.orientation + 3U) % 4U));
    bool r = maze_map_has_wall(&s_ground_truth, nav.x, nav.y, (Direction)((nav.orientation + 1U) % 4U));

    ProximitySensorArray sensors = sensor_make_binary(f, l, r);
    DetectedWalls walls = sensor_interface_evaluate(&sensors);

    /* 2. Run navigator step and measure execution time in microseconds */
    uint32_t t_start = DWT->CYCCNT;
    NavCommand cmd = navigator_step(&nav, &walls);
    uint32_t t_step_us = (DWT->CYCCNT - t_start) / 72U;  /* STM32F103 running at 72 MHz */
    total_solve_us += t_step_us;

    step_count++;

    /* 3. Stream step telemetry */
    snprintf(buf, sizeof(buf),
             "[STEP %2u] Pose:(%2u,%2u) Dir:%c | Act:%-12s | Time:%lu us\r\n",
             step_count, nav.x, nav.y,
             dir_labels[nav.orientation],
             nav_command_to_string(cmd),
             (unsigned long)t_step_us);
    uart_puts(buf);

    /* 4. Check for goal or termination */
    if (cmd == NAV_CMD_STOP)
    {
      if (nav.state == NAV_STATE_GOAL_REACHED)
      {
        uint32_t avg_us = step_count > 0 ? (total_solve_us / step_count) : 0;
        snprintf(buf, sizeof(buf),
                 "--------------------------------------------------\r\n"
                 "[GOAL] Reached at (%u, %u) in %u steps!\r\n"
                 "Execution Results:\r\n"
                 "  Total Steps:      %u\r\n"
                 "  Total Time:       %lu us (%lu.%03lu ms)\r\n"
                 "  Avg per Step:     %lu us\r\n"
                 "  Memory Footprint: sizeof(MazeMap)=%u bytes, sizeof(Navigator)=%u bytes\r\n"
                 "[STATUS] SUCCESS\r\n"
                 "==================================================\r\n\r\n",
                 nav.x, nav.y, step_count,
                 step_count,
                 (unsigned long)total_solve_us,
                 (unsigned long)(total_solve_us / 1000U),
                 (unsigned long)(total_solve_us % 1000U),
                 (unsigned long)avg_us,
                 (unsigned)sizeof(MazeMap), (unsigned)sizeof(Navigator));
        uart_puts(buf);
        return true;
      }
      else
      {
        snprintf(buf, sizeof(buf),
                 "--------------------------------------------------\r\n"
                 "[FAILED] No path to goal from (%u, %u) after %u steps\r\n"
                 "[STATUS] FAILED\r\n"
                 "==================================================\r\n\r\n",
                 nav.x, nav.y, step_count);
        uart_puts(buf);
        return false;
      }
    }
  }

  snprintf(buf, sizeof(buf),
           "[FAILED] Step limit (%u) reached without reaching goal\r\n"
           "[STATUS] FAILED\r\n\r\n",
           max_steps);
  uart_puts(buf);
  return false;
}

bool Floodfill_Runner_Benchmark(uint8_t size)
{
  if (!Floodfill_Runner_Build_Preset(size))
  {
    char buf[64];
    snprintf(buf, sizeof(buf), "ERR: Unsupported preset size %u\r\n[STATUS] FAILED\r\n", size);
    uart_puts(buf);
    return false;
  }
  return Floodfill_Runner_Solve(size * size * 2U);
}

void Floodfill_Runner_Print_Menu(void)
{
  uart_puts(
    "\r\n==================================================\r\n"
    "  STM32 Micromouse Flood-Fill & Motor Controller\r\n"
    "==================================================\r\n"
    "Flood-Fill Benchmark Commands:\r\n"
    "  '6' : Run 6x6 Maze Benchmark Simulation\r\n"
    "  '1' : Run 10x10 Maze Benchmark Simulation\r\n"
    "  '2' : Run 16x16 Full Competition Maze Benchmark\r\n"
    "  '5' : Run 5x5 Maze Benchmark Simulation\r\n"
    "  'h' : Print this Menu\r\n"
    "\r\n"
    "Dynamic Maze Upload Commands (Line-based):\r\n"
    "  MAZE_START <size>   : Initialize blank maze (e.g. MAZE_START 6)\r\n"
    "  WALL <x> <y> <N|E|S|W> : Add wall to cell\r\n"
    "  SOLVE               : Solve uploaded maze with Flood-Fill\r\n"
    "\r\n"
    "Manual Teleop / Motor Commands:\r\n"
    "  W/S/A/D : Continuous teleop drive\r\n"
    "  F/B     : 1 Cell Fwd / 1 Cell Back\r\n"
    "  J/K/U   : 90 deg Left / 90 deg Right / 180 deg\r\n"
    "  Space/X : Stop motors\r\n"
    "  R       : Reset encoder totals\r\n"
    "==================================================\r\n\r\n"
  );
}
