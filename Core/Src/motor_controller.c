/**
  ******************************************************************************
  * @file           : motor_controller.c
  * @brief          : Motor driver actuation (TB6612FNG), encoder quadrature
  *                   counting, speed estimation, closed-loop PI speed control,
  *                   and targeted autonomous moves.
  ******************************************************************************
  */

#include "motor_controller.h"
#include <stdlib.h>

/* ========================================================================== */
/* Hardware Timer Handles                                                     */
/* ========================================================================== */
static TIM_HandleTypeDef *p_htim_pwm = NULL;
static TIM_HandleTypeDef *p_htim_enc_left = NULL;
static TIM_HandleTypeDef *p_htim_enc_right = NULL;

/* ========================================================================== */
/* Internal State Variables                                                   */
/* ========================================================================== */
static volatile int32_t  left_encoder_total = 0;
static volatile int32_t  right_encoder_total = 0;
static uint16_t          left_encoder_previous = 0;
static uint16_t          right_encoder_previous = 0;

static int32_t           left_report_total = 0;
static int32_t           right_report_total = 0;

static volatile int8_t   left_motor_command = 0;
static volatile int8_t   right_motor_command = 0;
static volatile float    left_measured_rpm = 0.0f;
static volatile float    right_measured_rpm = 0.0f;
static volatile float    left_integral = 0.0f;
static volatile float    right_integral = 0.0f;
static volatile uint16_t left_pwm_duty = 0;
static volatile uint16_t right_pwm_duty = 0;

static volatile uint8_t  motor_control_enabled = 0;

/* Targeted Movement & Position Hold State */
static volatile uint8_t  cell_move_active = 0;
static volatile uint8_t  cell_hold_active = 0;
static volatile uint8_t  cell_move_timeout = 0;

static volatile int32_t  left_cell_start = 0;
static volatile int32_t  right_cell_start = 0;
static volatile int32_t  left_cell_target = 0;
static volatile int32_t  right_cell_target = 0;

static volatile int8_t   left_target_move_dir = 0;
static volatile int8_t   right_target_move_dir = 0;
static volatile int32_t  left_target_move_total = 0;
static volatile int32_t  right_target_move_total = 0;
static volatile uint32_t cell_move_start_tick = 0;

static volatile float    left_target_rpm = MOTOR_TARGET_RPM;
static volatile float    right_target_rpm = MOTOR_TARGET_RPM;

/* ========================================================================== */
/* Private Helpers                                                            */
/* ========================================================================== */

/**
  * @brief  PI speed controller with anti-windup clamping.
  */
static uint16_t Update_Motor_PI(int8_t command, float measured_rpm,
                                float target_rpm, volatile float *integral,
                                float feed_forward_duty, uint32_t elapsed_ms)
{
  float speed_error;
  float candidate_integral;
  float feed_forward;
  float output;

  if (command == 0)
  {
    *integral = 0.0f;
    return 0;
  }

  if (measured_rpm < 0.0f)
  {
    measured_rpm = -measured_rpm;
  }

  speed_error = target_rpm - measured_rpm;
  candidate_integral = *integral + MOTOR_KI * speed_error * ((float)elapsed_ms / 1000.0f);
  if (candidate_integral > MOTOR_PWM_MAX)
  {
    candidate_integral = MOTOR_PWM_MAX;
  }
  else if (candidate_integral < -MOTOR_PWM_MAX)
  {
    candidate_integral = -MOTOR_PWM_MAX;
  }

  feed_forward = feed_forward_duty;
  output = feed_forward + (MOTOR_KP * speed_error) + candidate_integral;
  if (!((output > MOTOR_PWM_MAX && speed_error > 0.0f) ||
        (output < 0.0f && speed_error < 0.0f)))
  {
    *integral = candidate_integral;
  }

  output = feed_forward + (MOTOR_KP * speed_error) + *integral;
  if (output < 0.0f)
  {
    output = 0.0f;
  }
  else if (output > MOTOR_PWM_MAX)
  {
    output = MOTOR_PWM_MAX;
  }

  return (uint16_t)output;
}

/**
  * @brief  Applies direction GPIO pins and PWM timer compare registers.
  */
static void Apply_Motor_Output(int8_t left_command, uint16_t left_duty,
                               int8_t right_command, uint16_t right_duty)
{
  if (p_htim_pwm == NULL)
  {
    return;
  }

  /* Left Motor: AIN1, AIN2, TIM1_CH1 */
  if (left_command > 0 && left_duty > 0U)
  {
    HAL_GPIO_WritePin(GPIOB, AIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, AIN2_Pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(p_htim_pwm, TIM_CHANNEL_1, left_duty);
  }
  else if (left_command < 0 && left_duty > 0U)
  {
    HAL_GPIO_WritePin(GPIOB, AIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, AIN2_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(p_htim_pwm, TIM_CHANNEL_1, left_duty);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(p_htim_pwm, TIM_CHANNEL_1, 0);
    HAL_GPIO_WritePin(GPIOB, AIN1_Pin | AIN2_Pin, GPIO_PIN_RESET);
  }

  /* Right Motor: BIN1, BIN2, TIM1_CH2 */
  if (right_command > 0 && right_duty > 0U)
  {
    HAL_GPIO_WritePin(GPIOB, BIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, BIN2_Pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(p_htim_pwm, TIM_CHANNEL_2, right_duty);
  }
  else if (right_command < 0 && right_duty > 0U)
  {
    HAL_GPIO_WritePin(GPIOB, BIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, BIN2_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(p_htim_pwm, TIM_CHANNEL_2, right_duty);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(p_htim_pwm, TIM_CHANNEL_2, 0);
    HAL_GPIO_WritePin(GPIOB, BIN1_Pin | BIN2_Pin, GPIO_PIN_RESET);
  }
}

/* ========================================================================== */
/* Public API Functions                                                       */
/* ========================================================================== */

void Motor_Controller_Init(TIM_HandleTypeDef *pwm_timer,
                           TIM_HandleTypeDef *left_encoder_timer,
                           TIM_HandleTypeDef *right_encoder_timer)
{
  p_htim_pwm = pwm_timer;
  p_htim_enc_left = left_encoder_timer;
  p_htim_enc_right = right_encoder_timer;

  /* Reset hardware encoder counters */
  __HAL_TIM_SET_COUNTER(p_htim_enc_left, 0);
  __HAL_TIM_SET_COUNTER(p_htim_enc_right, 0);

  if (HAL_TIM_Encoder_Start(p_htim_enc_left, TIM_CHANNEL_ALL) != HAL_OK ||
      HAL_TIM_Encoder_Start(p_htim_enc_right, TIM_CHANNEL_ALL) != HAL_OK)
  {
    Error_Handler();
  }

  left_encoder_previous = (uint16_t)__HAL_TIM_GET_COUNTER(p_htim_enc_left);
  right_encoder_previous = (uint16_t)__HAL_TIM_GET_COUNTER(p_htim_enc_right);

  /* Start PWM generation */
  if (HAL_TIM_PWM_Start(p_htim_pwm, TIM_CHANNEL_1) != HAL_OK ||
      HAL_TIM_PWM_Start(p_htim_pwm, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  /* Enable motor driver standby pin */
  HAL_GPIO_WritePin(GPIOB, STBY_Pin, GPIO_PIN_SET);
  Apply_Motor_Output(0, 0, 0, 0);

  motor_control_enabled = 1U;
}

void Motor_Set_Commands(int8_t left_cmd, float left_rpm,
                        int8_t right_cmd, float right_rpm)
{
  uint32_t interrupt_state = __get_PRIMASK();
  __disable_irq();

  cell_move_active = 0U;
  cell_hold_active = 0U;
  left_motor_command = left_cmd;
  right_motor_command = right_cmd;
  left_target_rpm = (left_rpm > 0.0f) ? left_rpm : MOTOR_TARGET_RPM;
  right_target_rpm = (right_rpm > 0.0f) ? right_rpm : MOTOR_TARGET_RPM;
  left_integral = 0.0f;
  right_integral = 0.0f;
  left_pwm_duty = 0;
  right_pwm_duty = 0;

  Apply_Motor_Output(left_cmd, 0, right_cmd, 0);

  if (interrupt_state == 0U)
  {
    __enable_irq();
  }
}

void Motor_Start_Targeted_Move(int32_t left_delta, int32_t right_delta)
{
  uint32_t interrupt_state = __get_PRIMASK();
  __disable_irq();

  left_cell_start = left_encoder_total;
  right_cell_start = right_encoder_total;
  left_cell_target = left_cell_start + left_delta;
  right_cell_target = right_cell_start + right_delta;
  left_target_move_total = (left_delta >= 0) ? left_delta : -left_delta;
  right_target_move_total = (right_delta >= 0) ? right_delta : -right_delta;
  left_target_move_dir = (left_delta > 0) ? 1 : ((left_delta < 0) ? -1 : 0);
  right_target_move_dir = (right_delta > 0) ? 1 : ((right_delta < 0) ? -1 : 0);

  left_motor_command = left_target_move_dir;
  right_motor_command = right_target_move_dir;
  left_target_rpm = MOTOR_TARGET_RPM;
  right_target_rpm = MOTOR_TARGET_RPM;
  left_integral = 0.0f;
  right_integral = 0.0f;
  left_pwm_duty = 0;
  right_pwm_duty = 0;
  cell_move_active = 1U;
  cell_hold_active = 0U;
  cell_move_start_tick = HAL_GetTick();
  cell_move_timeout = 0U;

  Apply_Motor_Output(left_target_move_dir, 0, right_target_move_dir, 0);

  if (interrupt_state == 0U)
  {
    __enable_irq();
  }
}

void Motor_Stop(void)
{
  Motor_Set_Commands(0, 0.0f, 0, 0.0f);
}

void Motor_Reset_Encoders(void)
{
  if (cell_move_active == 0U)
  {
    uint32_t interrupt_state = __get_PRIMASK();
    __disable_irq();

    cell_hold_active = 0U;
    left_encoder_total = 0;
    right_encoder_total = 0;
    left_report_total = 0;
    right_report_total = 0;

    if (interrupt_state == 0U)
    {
      __enable_irq();
    }
  }
}

uint8_t Motor_Check_And_Clear_Timeout(void)
{
  if (cell_move_timeout != 0U)
  {
    cell_move_timeout = 0U;
    return 1U;
  }
  return 0U;
}

uint8_t Motor_Is_Move_Active(void)
{
  return cell_move_active;
}

void Motor_Get_Telemetry(MotorTelemetry_t *telemetry)
{
  if (telemetry == NULL)
  {
    return;
  }

  uint32_t interrupt_state = __get_PRIMASK();
  __disable_irq();

  int32_t cur_left_total = left_encoder_total;
  int32_t cur_right_total = right_encoder_total;
  telemetry->left_encoder_total = cur_left_total;
  telemetry->right_encoder_total = cur_right_total;
  telemetry->left_report_delta = cur_left_total - left_report_total;
  telemetry->right_report_delta = cur_right_total - right_report_total;
  left_report_total = cur_left_total;
  right_report_total = cur_right_total;

  telemetry->left_measured_rpm = left_measured_rpm;
  telemetry->right_measured_rpm = right_measured_rpm;
  telemetry->left_pwm_duty = left_pwm_duty;
  telemetry->right_pwm_duty = right_pwm_duty;

  if (interrupt_state == 0U)
  {
    __enable_irq();
  }
}

void Motor_Control_Tick(void)
{
  static int16_t left_delta_history[MOTOR_RPM_WINDOW_MS] = {0};
  static int16_t right_delta_history[MOTOR_RPM_WINDOW_MS] = {0};
  static int32_t left_window_count = 0;
  static int32_t right_window_count = 0;
  static uint8_t history_index = 0;

  uint16_t left_current;
  uint16_t right_current;
  int16_t left_delta;
  int16_t right_delta;
  int8_t left_command;
  int8_t right_command;
  uint8_t position_hold_mode = 0U;
  float left_speed_target = left_target_rpm;
  float right_speed_target = right_target_rpm;

  if (motor_control_enabled == 0U || p_htim_enc_left == NULL || p_htim_enc_right == NULL)
  {
    return;
  }

  /* 1. Read quadrature encoder counters and handle 16-bit rollovers */
  left_current = (uint16_t)__HAL_TIM_GET_COUNTER(p_htim_enc_left);
  right_current = (uint16_t)__HAL_TIM_GET_COUNTER(p_htim_enc_right);
  left_delta = (int16_t)(left_current - left_encoder_previous);
  right_delta = (int16_t)(right_encoder_previous - right_current);
  left_encoder_previous = left_current;
  right_encoder_previous = right_current;
  left_encoder_total += left_delta;
  right_encoder_total += right_delta;

  /* 2. Moving average window for RPM measurement */
  left_window_count -= left_delta_history[history_index];
  right_window_count -= right_delta_history[history_index];
  left_delta_history[history_index] = left_delta;
  right_delta_history[history_index] = right_delta;
  left_window_count += left_delta;
  right_window_count += right_delta;
  history_index++;
  if (history_index >= MOTOR_RPM_WINDOW_MS)
  {
    history_index = 0;
  }

  left_measured_rpm = ((float)left_window_count * 60000.0f) /
                      (LEFT_COUNTS_PER_REV * (float)MOTOR_RPM_WINDOW_MS);
  right_measured_rpm = ((float)right_window_count * 60000.0f) /
                       (RIGHT_COUNTS_PER_REV * (float)MOTOR_RPM_WINDOW_MS);

  left_command = (int8_t)left_motor_command;
  right_command = (int8_t)right_motor_command;

  /* 3. Autonomous Targeted Moves (Cells and Turns) */
  if (cell_move_active != 0U)
  {
    int32_t left_progress = (left_target_move_dir >= 0) ?
        (left_encoder_total - left_cell_start) :
        (left_cell_start - left_encoder_total);
    int32_t right_progress = (right_target_move_dir >= 0) ?
        (right_encoder_total - right_cell_start) :
        (right_cell_start - right_encoder_total);

    int32_t left_remaining = left_target_move_total - left_progress;
    int32_t right_remaining = right_target_move_total - right_progress;
    float progress_error = (float)(left_progress - right_progress);
    float correction = progress_error * CELL_STRAIGHT_KP;

    if ((HAL_GetTick() - cell_move_start_tick) >= CELL_MOVE_TIMEOUT_MS)
    {
      cell_move_active = 0U;
      cell_move_timeout = 1U;
      left_command = 0;
      right_command = 0;
      left_motor_command = 0;
      right_motor_command = 0;
      left_integral = 0.0f;
      right_integral = 0.0f;
    }
    else if (left_remaining <= 0 && right_remaining <= 0)
    {
      /* Target reached: switch seamlessly to active position hold */
      cell_move_active = 0U;
      cell_hold_active = 1U;
      left_command = 0;
      right_command = 0;
      left_motor_command = 0;
      right_motor_command = 0;
      left_integral = 0.0f;
      right_integral = 0.0f;
    }
    else
    {
      left_command = (left_remaining <= 0) ? 0 : left_target_move_dir;
      right_command = (right_remaining <= 0) ? 0 : right_target_move_dir;

      /* Clamp straightness correction */
      if (correction > CELL_MAX_STRAIGHT_CORRECTION_RPM)
      {
        correction = CELL_MAX_STRAIGHT_CORRECTION_RPM;
      }
      else if (correction < -CELL_MAX_STRAIGHT_CORRECTION_RPM)
      {
        correction = -CELL_MAX_STRAIGHT_CORRECTION_RPM;
      }

      /* Deceleration ramp */
      if (left_remaining < CELL_SLOWDOWN_TICKS && left_target_move_total > 0)
      {
        left_speed_target = CELL_MIN_APPROACH_RPM +
            (MOTOR_TARGET_RPM - CELL_MIN_APPROACH_RPM) *
            ((float)left_remaining / (float)CELL_SLOWDOWN_TICKS);
      }
      if (right_remaining < CELL_SLOWDOWN_TICKS && right_target_move_total > 0)
      {
        right_speed_target = CELL_MIN_APPROACH_RPM +
            (MOTOR_TARGET_RPM - CELL_MIN_APPROACH_RPM) *
            ((float)right_remaining / (float)CELL_SLOWDOWN_TICKS);
      }

      left_speed_target -= correction;
      right_speed_target += correction;

      if (left_speed_target < CELL_MIN_APPROACH_RPM)
      {
        left_speed_target = CELL_MIN_APPROACH_RPM;
      }
      if (right_speed_target < CELL_MIN_APPROACH_RPM)
      {
        right_speed_target = CELL_MIN_APPROACH_RPM;
      }
      if (left_speed_target > MOTOR_TARGET_RPM)
      {
        left_speed_target = MOTOR_TARGET_RPM;
      }
      if (right_speed_target > MOTOR_TARGET_RPM)
      {
        right_speed_target = MOTOR_TARGET_RPM;
      }
    }
  }
  /* 4. Closed-Loop Position Hold */
  else if (cell_hold_active != 0U)
  {
    int32_t left_position_error = left_cell_target - left_encoder_total;
    int32_t right_position_error = right_cell_target - right_encoder_total;
    int32_t left_error_magnitude = (left_position_error >= 0) ?
                                  left_position_error : -left_position_error;
    int32_t right_error_magnitude = (right_position_error >= 0) ?
                                   right_position_error : -right_position_error;

    position_hold_mode = 1U;
    if (left_error_magnitude > CELL_HOLD_DEADBAND_TICKS)
    {
      left_command = (left_position_error > 0) ? 1 : -1;
      left_speed_target = (float)left_error_magnitude * CELL_HOLD_RPM_PER_TICK;
      if (left_speed_target < CELL_HOLD_MIN_RPM)
      {
        left_speed_target = CELL_HOLD_MIN_RPM;
      }
      else if (left_speed_target > CELL_HOLD_MAX_RPM)
      {
        left_speed_target = CELL_HOLD_MAX_RPM;
      }
    }
    else
    {
      left_command = 0;
    }

    if (right_error_magnitude > CELL_HOLD_DEADBAND_TICKS)
    {
      right_command = (right_position_error > 0) ? 1 : -1;
      right_speed_target = (float)right_error_magnitude * CELL_HOLD_RPM_PER_TICK;
      if (right_speed_target < CELL_HOLD_MIN_RPM)
      {
        right_speed_target = CELL_HOLD_MIN_RPM;
      }
      else if (right_speed_target > CELL_HOLD_MAX_RPM)
      {
        right_speed_target = CELL_HOLD_MAX_RPM;
      }
    }
    else
    {
      right_command = 0;
    }
  }

  /* 5. Feed-forward and PI Controller Execution */
  float left_ff = (position_hold_mode != 0U) ?
      CELL_HOLD_FEED_FORWARD_DUTY :
      (MOTOR_FEED_FORWARD_DUTY * (left_speed_target / MOTOR_TARGET_RPM));
  float right_ff = (position_hold_mode != 0U) ?
      CELL_HOLD_FEED_FORWARD_DUTY :
      (MOTOR_FEED_FORWARD_DUTY * (right_speed_target / MOTOR_TARGET_RPM));

  left_pwm_duty = Update_Motor_PI(left_command, left_measured_rpm,
                                  (left_command == 0) ? 0.0f : left_speed_target,
                                  &left_integral,
                                  left_ff,
                                  1000U / MOTOR_CONTROL_HZ);
  right_pwm_duty = Update_Motor_PI(right_command, right_measured_rpm,
                                   (right_command == 0) ? 0.0f : right_speed_target,
                                   &right_integral,
                                   right_ff,
                                   1000U / MOTOR_CONTROL_HZ);

  /* 6. Hardware Actuation */
  Apply_Motor_Output(left_command, left_pwm_duty,
                     right_command, right_pwm_duty);
}
