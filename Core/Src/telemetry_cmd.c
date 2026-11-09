/**
  ******************************************************************************
  * @file           : telemetry_cmd.c
  * @brief          : UART command parser (WASD, curves, discrete moves, reset)
  *                   and 100 ms periodic telemetry formatter.
  ******************************************************************************
  */

#include "telemetry_cmd.h"
#include "motor_controller.h"
#include <stdio.h>

static UART_HandleTypeDef *p_huart = NULL;
static uint32_t last_telemetry_tick = 0;

void Telemetry_Cmd_Init(UART_HandleTypeDef *huart)
{
  p_huart = huart;
  last_telemetry_tick = HAL_GetTick();
}

void Telemetry_Send_Startup_Banner(void)
{
  if (p_huart == NULL)
  {
    return;
  }

  static const uint8_t banner_msg[] =
    "Keys: F=1cell fwd, B=1cell rev, J=90L, K=90R, U=180, W/S/A/D=drive, Space/X=stop, R=reset\r\n";

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
    switch (key)
    {
      /* --- Continuous Teleop Driving --- */
      case 'w':
      case 'W':
        Motor_Set_Commands(1, MOTOR_TARGET_RPM, 1, MOTOR_TARGET_RPM);
        break;

      case 's':
      case 'S':
        Motor_Set_Commands(-1, MOTOR_TARGET_RPM, -1, MOTOR_TARGET_RPM);
        break;

      case 'a':
      case 'A':
        Motor_Set_Commands(-1, MOTOR_TARGET_RPM, 1, MOTOR_TARGET_RPM);
        break;

      case 'd':
      case 'D':
        Motor_Set_Commands(1, MOTOR_TARGET_RPM, -1, MOTOR_TARGET_RPM);
        break;

      /* --- Smooth Curve Maneuvers --- */
      case 'q':
      case 'Q':
        /* Forward-left curve */
        Motor_Set_Commands(1, MOTOR_CURVE_SLOW_RPM, 1, MOTOR_CURVE_FAST_RPM);
        break;

      case 'e':
      case 'E':
        /* Forward-right curve */
        Motor_Set_Commands(1, MOTOR_CURVE_FAST_RPM, 1, MOTOR_CURVE_SLOW_RPM);
        break;

      case 'z':
      case 'Z':
        /* Reverse-left curve */
        Motor_Set_Commands(-1, MOTOR_CURVE_SLOW_RPM, -1, MOTOR_CURVE_FAST_RPM);
        break;

      case 'c':
      case 'C':
        /* Reverse-right curve */
        Motor_Set_Commands(-1, MOTOR_CURVE_FAST_RPM, -1, MOTOR_CURVE_SLOW_RPM);
        break;

      /* --- Autonomous Discrete Moves --- */
      case 'f':
      case 'F':
        /* 1 Cell Forward */
        Motor_Start_Targeted_Move(CELL_TICKS, CELL_TICKS);
        break;

      case 'b':
      case 'B':
        /* 1 Cell Backward */
        Motor_Start_Targeted_Move(-CELL_TICKS, -CELL_TICKS);
        break;

      case 'j':
      case 'J':
        /* Turn 90° Left */
        Motor_Start_Targeted_Move(-TURN_90_TICKS, TURN_90_TICKS);
        break;

      case 'k':
      case 'K':
        /* Turn 90° Right */
        Motor_Start_Targeted_Move(TURN_90_TICKS, -TURN_90_TICKS);
        break;

      case 'u':
      case 'U':
        /* Turn 180° */
        Motor_Start_Targeted_Move(-TURN_180_TICKS, TURN_180_TICKS);
        break;

      /* --- Stop & Safety --- */
      case 'x':
      case 'X':
      case ' ':
        Motor_Stop();
        break;

      /* --- Reset Encoders --- */
      case 'r':
      case 'R':
        Motor_Reset_Encoders();
        break;

      default:
        break;
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
