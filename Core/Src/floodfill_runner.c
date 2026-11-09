/**
  ******************************************************************************
  * @file           : floodfill_runner.c
  * @brief          : Standalone Flood-Fill navigation & benchmark harness for STM32
  ******************************************************************************
  */

#include "floodfill_runner.h"
#include "motor_controller.h"
#include "motor_config.h"
#include "telemetry_cmd.h"
#include <stdio.h>
#include <string.h>

static UART_HandleTypeDef *s_huart = NULL;

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
}

static bool wait_for_move_complete(uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();
  while (Motor_Is_Move_Active())
  {
    /* Check for abort key ('x', 'X', ' ' or ESC 0x1B) over UART */
    uint8_t rx_char;
    if (s_huart != NULL && HAL_UART_Receive(s_huart, &rx_char, 1, 0) == HAL_OK)
    {
      if (rx_char == 'x' || rx_char == 'X' || rx_char == ' ' || rx_char == 0x1B)
      {
        Motor_Stop();
        uart_puts("\r\n[ABORT] User cancelled physical motion!\r\n");
        return false;
      }
    }

    /* Keep periodic telemetry streaming so GUI odometry stays live */
    Telemetry_Send_Periodic();

    if ((HAL_GetTick() - start) >= timeout_ms)
    {
      Motor_Stop();
      uart_puts("\r\n[TIMEOUT] Physical motion timed out!\r\n");
      return false;
    }

    HAL_Delay(5);
  }

  if (Motor_Check_And_Clear_Timeout())
  {
    Motor_Stop();
    uart_puts("\r\n[TIMEOUT] Move timed out!\r\n");
    return false;
  }

  return true;
}

static bool execute_motion_for_step(NavCommand cmd)
{
  switch (cmd)
  {
    case NAV_CMD_MOVE_FORWARD:
      uart_puts("  >> Motors: Advance 1 cell\r\n");
      Motor_Start_Targeted_Move(CELL_TICKS, CELL_TICKS);
      if (!wait_for_move_complete(CELL_MOVE_TIMEOUT_MS))
      {
        return false;
      }
      HAL_Delay(120);
      break;

    case NAV_CMD_TURN_RIGHT:
      uart_puts("  >> Motors: Turn 90 deg Right + Advance 1 cell\r\n");
      Motor_Start_Targeted_Move(TURN_90_TICKS, -TURN_90_TICKS);
      if (!wait_for_move_complete(CELL_MOVE_TIMEOUT_MS))
      {
        return false;
      }
      HAL_Delay(100);
      Motor_Start_Targeted_Move(CELL_TICKS, CELL_TICKS);
      if (!wait_for_move_complete(CELL_MOVE_TIMEOUT_MS))
      {
        return false;
      }
      HAL_Delay(120);
      break;

    case NAV_CMD_TURN_LEFT:
      uart_puts("  >> Motors: Turn 90 deg Left + Advance 1 cell\r\n");
      Motor_Start_Targeted_Move(-TURN_90_TICKS, TURN_90_TICKS);
      if (!wait_for_move_complete(CELL_MOVE_TIMEOUT_MS))
      {
        return false;
      }
      HAL_Delay(100);
      Motor_Start_Targeted_Move(CELL_TICKS, CELL_TICKS);
      if (!wait_for_move_complete(CELL_MOVE_TIMEOUT_MS))
      {
        return false;
      }
      HAL_Delay(120);
      break;

    case NAV_CMD_TURN_AROUND:
      uart_puts("  >> Motors: Turn 180 deg U-Turn + Advance 1 cell\r\n");
      Motor_Start_Targeted_Move(-TURN_180_TICKS, TURN_180_TICKS);
      if (!wait_for_move_complete(CELL_MOVE_TIMEOUT_MS))
      {
        return false;
      }
      HAL_Delay(100);
      Motor_Start_Targeted_Move(CELL_TICKS, CELL_TICKS);
      if (!wait_for_move_complete(CELL_MOVE_TIMEOUT_MS))
      {
        return false;
      }
      HAL_Delay(120);
      break;

    case NAV_CMD_STOP:
    default:
      Motor_Stop();
      break;
  }

  return true;
}

static bool request_and_receive_walls(uint8_t x, uint8_t y, Direction dir,
                                      DetectedWalls *walls_out, uint32_t timeout_ms)
{
  if (s_huart == NULL || walls_out == NULL)
  {
    return false;
  }

  const char dir_labels[] = {'N', 'E', 'S', 'W'};
  char req[64];
  snprintf(req, sizeof(req), "REQ_WALLS %u %u %c\r\n", x, y, dir_labels[dir]);
  uart_puts(req);

  char rx_buf[64];
  uint8_t rx_pos = 0;
  uint32_t start = HAL_GetTick();

  while ((HAL_GetTick() - start) < timeout_ms)
  {
    uint8_t ch;
    if (HAL_UART_Receive(s_huart, &ch, 1, 10) == HAL_OK)
    {
      if (ch == '\r' || ch == '\n')
      {
        if (rx_pos > 0)
        {
          rx_buf[rx_pos] = '\0';
          if (strcmp(rx_buf, "ABORT") == 0 || strcmp(rx_buf, "x") == 0)
          {
            return false;
          }

          unsigned int f = 0, l = 0, r = 0;
          if (sscanf(rx_buf, "WALLS %u %u %u", &f, &l, &r) == 3)
          {
            walls_out->wall_front = (f != 0);
            walls_out->wall_left  = (l != 0);
            walls_out->wall_right = (r != 0);
            walls_out->wall_rear  = false;
            return true;
          }
          rx_pos = 0;
        }
      }
      else if (rx_pos < sizeof(rx_buf) - 1U)
      {
        rx_buf[rx_pos++] = (char)ch;
      }
    }
  }

  uart_puts("ERR: Wall request timeout from Python oracle\r\n");
  return false;
}

bool Floodfill_Runner_Solve_Blind(uint8_t size, bool actuate_motors, uint16_t max_steps)
{
  if (s_huart == NULL || size < 2 || size > MICROMOUSE_MAX_MAZE_SIZE)
  {
    uart_puts("ERR: Invalid size or UART\r\n[STATUS] FAILED\r\n");
    return false;
  }

  char buf[384];
  snprintf(buf, sizeof(buf),
           "\r\n==================================================\r\n"
           "  STM32 Blind Flood-Fill Navigation: %ux%u Maze %s\r\n"
           "==================================================\r\n",
           size, size,
           actuate_motors ? "[LIVE MOTORS]" : "[SIMULATED]");
  uart_puts(buf);

  if (actuate_motors)
  {
    Motor_Stop();
    Motor_Reset_Encoders();
    HAL_Delay(100);
  }

  /* Initialize Navigator with an empty maze (only perimeter borders known) */
  Navigator nav;
  navigator_init(&nav, size, 0, 0, DIR_NORTH);

  uint32_t total_solve_us = 0;
  uint16_t step_count = 0;
  const char dir_labels[] = {'N', 'E', 'S', 'W'};

  while (step_count < max_steps)
  {
    /* 1. Request walls for current cell and orientation from Python */
    DetectedWalls walls;
    if (!request_and_receive_walls(nav.x, nav.y, nav.orientation, &walls, 10000))
    {
      uart_puts("[STATUS] FAILED\r\n");
      if (actuate_motors) Motor_Stop();
      return false;
    }

    /* 2. Run navigator step using robot's internal discovered map */
    uint32_t t_start = DWT->CYCCNT;
    NavCommand cmd = navigator_step(&nav, &walls);
    uint32_t t_step_us = (DWT->CYCCNT - t_start) / 72U;
    total_solve_us += t_step_us;

    step_count++;

    /* 3. Output step telemetry */
    snprintf(buf, sizeof(buf),
             "[STEP %2u] Pose:(%2u,%2u) Dir:%c | Act:%-12s | Time:%lu us | Sensed:[F:%u L:%u R:%u]\r\n",
             step_count, nav.x, nav.y,
             dir_labels[nav.orientation],
             nav_command_to_string(cmd),
             (unsigned long)t_step_us,
             walls.wall_front ? 1 : 0,
             walls.wall_left ? 1 : 0,
             walls.wall_right ? 1 : 0);
    uart_puts(buf);

    /* 4. Check for goal reaching */
    if (cmd == NAV_CMD_STOP)
    {
      if (actuate_motors) Motor_Stop();
      if (nav.state == NAV_STATE_GOAL_REACHED)
      {
        snprintf(buf, sizeof(buf),
                 "--------------------------------------------------\r\n"
                 "[GOAL] Reached at (%u, %u) in %u steps!\r\n"
                 "Execution Results:\r\n"
                 "  Total Steps:      %u\r\n"
                 "  Total Compute:    %lu us (%lu.%03lu ms)\r\n"
                 "[STATUS] SUCCESS\r\n"
                 "==================================================\r\n\r\n",
                 nav.x, nav.y, step_count,
                 step_count,
                 (unsigned long)total_solve_us,
                 (unsigned long)(total_solve_us / 1000U),
                 (unsigned long)(total_solve_us % 1000U));
        uart_puts(buf);
        return true;
      }
      else
      {
        uart_puts("[FAILED] No path to goal\r\n[STATUS] FAILED\r\n");
        return false;
      }
    }

    /* 5. Physical or simulated motion */
    if (actuate_motors)
    {
      if (!execute_motion_for_step(cmd))
      {
        Motor_Stop();
        snprintf(buf, sizeof(buf), "[FAILED] Motion aborted during step %u\r\n[STATUS] FAILED\r\n", step_count);
        uart_puts(buf);
        return false;
      }
    }
    else
    {
      HAL_Delay(40);
    }

    /* 6. Send cell-arrival confirmation to Python */
    snprintf(buf, sizeof(buf), "ARRIVED %u %u %c\r\n", nav.x, nav.y, dir_labels[nav.orientation]);
    uart_puts(buf);
  }

  if (actuate_motors) Motor_Stop();
  uart_puts("[FAILED] Max steps exceeded\r\n[STATUS] FAILED\r\n");
  return false;
}

void Floodfill_Runner_Print_Menu(void)
{
  uart_puts(
    "\r\n==================================================\r\n"
    "  STM32 Micromouse Blind Flood-Fill Controller\r\n"
    "==================================================\r\n"
    "Blind Navigation Commands (Python Maze Oracle):\r\n"
    "  BLIND_START <size>  : Blind cell-by-cell navigation (sim)\r\n"
    "  BLIND_RUN <size>    : Blind cell-by-cell with LIVE MOTORS\r\n"
    "  'm'                 : Run 6x6 with Live Motors\r\n"
    "  '6' / '1' / '2'     : Run 6x6 / 10x10 / 16x16 Blind Sim\r\n"
    "  'h'                 : Print this Menu\r\n"
    "\r\n"
    "Manual Teleop / Motor Commands:\r\n"
    "  W/S/A/D : Continuous teleop drive\r\n"
    "  F/B     : 1 Cell Fwd / 1 Cell Back\r\n"
    "  J/K/U   : 90 deg Left / 90 deg Right / 180 deg\r\n"
    "  Space/X : Stop motors / Abort motion\r\n"
    "  R       : Reset encoder totals\r\n"
    "==================================================\r\n\r\n"
  );
}
