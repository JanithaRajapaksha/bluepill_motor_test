/**
  ******************************************************************************
  * @file           : telemetry_cmd.c
  * @brief          : UART command parser (teleop, discrete moves, reset, floodfill)
  *                   and 100 ms periodic telemetry formatter.
  ******************************************************************************
  */

#include "telemetry_cmd.h"
#include "motor_controller.h"
#include "floodfill_runner.h"
#include <stdio.h>
#include <string.h>

static UART_HandleTypeDef *p_huart = NULL;
static uint32_t last_telemetry_tick = 0;

/* Line buffer for dynamic maze upload & line commands */
static char rx_line_buf[64];
static uint8_t rx_line_pos = 0;

static void parse_line_command(const char *line)
{
  while (*line == ' ' || *line == '\t')
  {
    line++;
  }
  if (*line == '\0')
  {
    return;
  }

  if (strncmp(line, "BLIND_RUN", 9) == 0 || strncmp(line, "RUN", 3) == 0 ||
      strncmp(line, "DRIVE", 5) == 0)
  {
    unsigned int sz = 6;
    if (strncmp(line, "BLIND_RUN", 9) == 0) sscanf(line + 9, "%u", &sz);
    else sscanf(line + 3, "%u", &sz);
    if (sz == 0) sz = 6;
    Floodfill_Runner_Solve_Blind((uint8_t)sz, true, 250);
  }
  else if (strncmp(line, "BLIND_START", 11) == 0 || strncmp(line, "BLIND_SOLVE", 11) == 0 ||
           strncmp(line, "BLIND", 5) == 0 || strncmp(line, "SOLVE", 5) == 0)
  {
    unsigned int sz = 6;
    if (strncmp(line, "BLIND_START", 11) == 0) sscanf(line + 11, "%u", &sz);
    else if (strncmp(line, "BLIND_SOLVE", 11) == 0) sscanf(line + 11, "%u", &sz);
    else if (strncmp(line, "BLIND", 5) == 0) sscanf(line + 5, "%u", &sz);
    else if (strncmp(line, "SOLVE", 5) == 0) sscanf(line + 5, "%u", &sz);
    if (sz == 0) sz = 6;
    Floodfill_Runner_Solve_Blind((uint8_t)sz, false, 250);
  }
  else if (strncmp(line, "HELP", 4) == 0 || strcmp(line, "H") == 0 || strcmp(line, "?") == 0)
  {
    Floodfill_Runner_Print_Menu();
  }
  else
  {
    char err[64];
    snprintf(err, sizeof(err), "ERR: Unknown command '%s'\r\n", line);
    HAL_UART_Transmit(p_huart, (uint8_t *)err, (uint16_t)strlen(err), HAL_MAX_DELAY);
  }
}

void Telemetry_Cmd_Init(UART_HandleTypeDef *huart)
{
  p_huart = huart;
  last_telemetry_tick = HAL_GetTick();

  /* Initialize Flood-Fill Runner and benchmark engine */
  Floodfill_Runner_Init(huart);
}

void Telemetry_Send_Startup_Banner(void)
{
  if (p_huart == NULL)
  {
    return;
  }

  static const uint8_t banner_msg[] =
    "\r\n=== Micromouse BluePill Ready (Blind Navigation) ===\r\n"
    "Blind Flood-Fill: '6'=6x6, '1'=10x10, '2'=16x16, 'm'=6x6 Live Motors, 'h'=menu\r\n"
    "Commands: BLIND <sz>, BLIND_RUN <sz>\r\n"
    "Teleop: W/S/A/D, F=1cell fwd, B=1cell rev, J=90L, K=90R, U=180, X=stop, R=reset\r\n\r\n";

  HAL_UART_Transmit(p_huart, (uint8_t *)banner_msg,
                    sizeof(banner_msg) - 1U, HAL_MAX_DELAY);
}

void Telemetry_Cmd_Process(void)
{
  uint8_t key;

  if (p_huart == NULL)
  {
    return;
  }

  /* Non-blocking UART receive (timeout = 0) */
  if (HAL_UART_Receive(p_huart, &key, 1, 0) == HAL_OK)
  {
    /* Handle line termination */
    if (key == '\r' || key == '\n')
    {
      if (rx_line_pos > 0)
      {
        rx_line_buf[rx_line_pos] = '\0';
        parse_line_command(rx_line_buf);
        rx_line_pos = 0;
      }
      return;
    }

    /* Handle single benchmark keys when line buffer is empty */
    if (rx_line_pos == 0)
    {
      switch (key)
      {
        case '6':
          Floodfill_Runner_Solve_Blind(6, false, 250);
          return;

        case '1':
          Floodfill_Runner_Solve_Blind(10, false, 250);
          return;

        case '2':
          Floodfill_Runner_Solve_Blind(16, false, 250);
          return;

        case '5':
          Floodfill_Runner_Solve_Blind(5, false, 250);
          return;

        case 'm':
        case 'M':
          Floodfill_Runner_Solve_Blind(6, true, 250);
          return;

        case 'h':
          Floodfill_Runner_Print_Menu();
          return;

        default:
          break;
      }
    }

    /* Handle immediate teleop / discrete motor commands (lowercase or Space) */
    switch (key)
    {
      /* --- Continuous Teleop Driving --- */
      case 'w':
        Motor_Set_Commands(1, MOTOR_TARGET_RPM, 1, MOTOR_TARGET_RPM);
        return;

      case 's':
        Motor_Set_Commands(-1, MOTOR_TARGET_RPM, -1, MOTOR_TARGET_RPM);
        return;

      case 'a':
        Motor_Set_Commands(-1, MOTOR_TARGET_RPM, 1, MOTOR_TARGET_RPM);
        return;

      case 'd':
        Motor_Set_Commands(1, MOTOR_TARGET_RPM, -1, MOTOR_TARGET_RPM);
        return;

      /* --- Smooth Curve Maneuvers --- */
      case 'q':
        Motor_Set_Commands(1, MOTOR_CURVE_SLOW_RPM, 1, MOTOR_CURVE_FAST_RPM);
        return;

      case 'e':
        Motor_Set_Commands(1, MOTOR_CURVE_FAST_RPM, 1, MOTOR_CURVE_SLOW_RPM);
        return;

      case 'z':
        Motor_Set_Commands(-1, MOTOR_CURVE_SLOW_RPM, -1, MOTOR_CURVE_FAST_RPM);
        return;

      case 'c':
        Motor_Set_Commands(-1, MOTOR_CURVE_FAST_RPM, -1, MOTOR_CURVE_SLOW_RPM);
        return;

      /* --- Autonomous Discrete Moves --- */
      case 'f':
        Motor_Start_Targeted_Move(CELL_TICKS, CELL_TICKS);
        return;

      case 'b':
        Motor_Start_Targeted_Move(-CELL_TICKS, -CELL_TICKS);
        return;

      case 'j':
        Motor_Start_Targeted_Move(-TURN_90_TICKS, TURN_90_TICKS);
        return;

      case 'k':
        Motor_Start_Targeted_Move(TURN_90_TICKS, -TURN_90_TICKS);
        return;

      case 'u':
        Motor_Start_Targeted_Move(-TURN_180_TICKS, TURN_180_TICKS);
        return;

      /* --- Stop & Safety --- */
      case 'x':
      case ' ':
        Motor_Stop();
        return;

      /* --- Reset Encoders --- */
      case 'r':
        Motor_Reset_Encoders();
        return;

      default:
        break;
    }

    /* Otherwise, buffer characters for multi-character line commands */
    if (rx_line_pos < sizeof(rx_line_buf) - 1U)
    {
      rx_line_buf[rx_line_pos++] = (char)key;
    }
  }

  /* Check if a targeted move timed out */
  if (Motor_Check_And_Clear_Timeout())
  {
    static const uint8_t timeout_msg[] = "CELL_TIMEOUT: motors stopped\r\n";
    HAL_UART_Transmit(p_huart, (uint8_t *)timeout_msg,
                      sizeof(timeout_msg) - 1U, HAL_MAX_DELAY);
  }
}

void Telemetry_Send_Periodic(void)
{
  if (p_huart == NULL)
  {
    return;
  }

  uint32_t now = HAL_GetTick();
  if ((now - last_telemetry_tick) >= MOTOR_REPORT_PERIOD_MS)
  {
    last_telemetry_tick = now;

    MotorTelemetry_t telem;
    Motor_Get_Telemetry(&telem);

    char report[128];
    int report_length = snprintf(
        report, sizeof(report),
        "L:%ld d:%ld RPM:%d PWM:%u | R:%ld d:%ld RPM:%d PWM:%u\r\n",
        (long)telem.left_encoder_total,
        (long)telem.left_report_delta,
        (int)telem.left_measured_rpm,
        (unsigned int)telem.left_pwm_duty,
        (long)telem.right_encoder_total,
        (long)telem.right_report_delta,
        (int)telem.right_measured_rpm,
        (unsigned int)telem.right_pwm_duty);

    if (report_length > 0)
    {
      HAL_UART_Transmit(p_huart, (uint8_t *)report, (uint16_t)report_length,
                        HAL_MAX_DELAY);
    }
  }
}
