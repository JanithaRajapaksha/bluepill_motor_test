/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LEFT_COUNTS_PER_REV 815.70f
#define RIGHT_COUNTS_PER_REV 815.55f
#define MOTOR_TARGET_RPM 150.0f
#define MOTOR_CURVE_SLOW_RPM 60.0f
#define MOTOR_CURVE_FAST_RPM 150.0f
#define CELL_TICKS 1060L
#define TURN_90_TICKS 692L
#define TURN_180_TICKS 1384L
#define CELL_SLOWDOWN_TICKS 250L
#define CELL_MIN_APPROACH_RPM 60.0f
#define CELL_STRAIGHT_KP 0.08f
#define CELL_MAX_STRAIGHT_CORRECTION_RPM 30.0f
#define CELL_MOVE_TIMEOUT_MS 3000U
#define CELL_HOLD_DEADBAND_TICKS 20L
#define CELL_HOLD_MIN_RPM 35.0f
#define CELL_HOLD_MAX_RPM 60.0f
#define CELL_HOLD_RPM_PER_TICK 0.25f
#define CELL_HOLD_FEED_FORWARD_DUTY 180.0f
#define MOTOR_CONTROL_HZ 1000U
#define MOTOR_RPM_WINDOW_MS 20U
#define MOTOR_REPORT_PERIOD_MS 100U
#define MOTOR_PWM_MAX 600.0f
#define MOTOR_FEED_FORWARD_DUTY 300.0f
#define MOTOR_KP 0.8f
#define MOTOR_KI 6.5f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static volatile int32_t left_encoder_total = 0;
static volatile int32_t right_encoder_total = 0;
static uint16_t left_encoder_previous = 0;
static uint16_t right_encoder_previous = 0;
static int32_t left_report_total = 0;
static int32_t right_report_total = 0;
static uint32_t encoder_report_tick = 0;
static volatile int8_t left_motor_command = 0;
static volatile int8_t right_motor_command = 0;
static volatile float left_measured_rpm = 0.0f;
static volatile float right_measured_rpm = 0.0f;
static volatile float left_integral = 0.0f;
static volatile float right_integral = 0.0f;
static volatile uint16_t left_pwm_duty = 0;
static volatile uint16_t right_pwm_duty = 0;
static volatile uint8_t motor_control_enabled = 0;
static volatile uint8_t cell_move_active = 0;
static volatile uint8_t cell_hold_active = 0;
static volatile uint8_t cell_move_timeout = 0;
static volatile int32_t left_cell_start = 0;
static volatile int32_t right_cell_start = 0;
static volatile int32_t left_cell_target = 0;
static volatile int32_t right_cell_target = 0;
static volatile int8_t left_target_move_dir = 0;
static volatile int8_t right_target_move_dir = 0;
static volatile int32_t left_target_move_total = 0;
static volatile int32_t right_target_move_total = 0;
static volatile uint32_t cell_move_start_tick = 0;
static volatile float left_target_rpm = MOTOR_TARGET_RPM;
static volatile float right_target_rpm = MOTOR_TARGET_RPM;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static uint16_t Update_Motor_PI(int8_t command, float measured_rpm,
                                float target_rpm, volatile float *integral,
                                float feed_forward_duty, uint32_t elapsed_ms);
static void Set_Motor_Commands(int8_t left_command, float left_rpm,
                               int8_t right_command, float right_rpm,
                               uint8_t start_cell_move);
static void Start_Targeted_Move(int32_t left_delta, int32_t right_delta);
static void Apply_Motor_Output(int8_t left_command, uint16_t left_duty,
                               int8_t right_command, uint16_t right_duty);
void Motor_Control_Tick(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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

static void Apply_Motor_Output(int8_t left_command, uint16_t left_duty,
                               int8_t right_command, uint16_t right_duty)
{
  if (left_command > 0 && left_duty > 0U)
  {
    HAL_GPIO_WritePin(GPIOB, AIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, AIN2_Pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, left_duty);
  }
  else if (left_command < 0 && left_duty > 0U)
  {
    HAL_GPIO_WritePin(GPIOB, AIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, AIN2_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, left_duty);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    HAL_GPIO_WritePin(GPIOB, AIN1_Pin | AIN2_Pin, GPIO_PIN_RESET);
  }

  if (right_command > 0 && right_duty > 0U)
  {
    HAL_GPIO_WritePin(GPIOB, BIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, BIN2_Pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, right_duty);
  }
  else if (right_command < 0 && right_duty > 0U)
  {
    HAL_GPIO_WritePin(GPIOB, BIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, BIN2_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, right_duty);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    HAL_GPIO_WritePin(GPIOB, BIN1_Pin | BIN2_Pin, GPIO_PIN_RESET);
  }
}

static void Start_Targeted_Move(int32_t left_delta, int32_t right_delta)
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

static void Set_Motor_Commands(int8_t left_command, float left_rpm,
                               int8_t right_command, float right_rpm,
                               uint8_t start_cell_move)
{
  if (start_cell_move != 0U)
  {
    Start_Targeted_Move(CELL_TICKS, CELL_TICKS);
    return;
  }

  uint32_t interrupt_state = __get_PRIMASK();
  __disable_irq();

  left_motor_command = left_command;
  right_motor_command = right_command;
  left_target_rpm = (left_rpm > 0.0f) ? left_rpm : MOTOR_TARGET_RPM;
  right_target_rpm = (right_rpm > 0.0f) ? right_rpm : MOTOR_TARGET_RPM;
  left_integral = 0.0f;
  right_integral = 0.0f;
  left_pwm_duty = 0;
  right_pwm_duty = 0;
  cell_move_active = 0U;
  cell_hold_active = 0U;
  Apply_Motor_Output(left_command, 0, right_command, 0);

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

  if (motor_control_enabled == 0U)
  {
    return;
  }

  left_current = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
  right_current = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  left_delta = (int16_t)(left_current - left_encoder_previous);
  right_delta = (int16_t)(right_encoder_previous - right_current);
  left_encoder_previous = left_current;
  right_encoder_previous = right_current;
  left_encoder_total += left_delta;
  right_encoder_total += right_delta;

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

      if (correction > CELL_MAX_STRAIGHT_CORRECTION_RPM)
      {
        correction = CELL_MAX_STRAIGHT_CORRECTION_RPM;
      }
      else if (correction < -CELL_MAX_STRAIGHT_CORRECTION_RPM)
      {
        correction = -CELL_MAX_STRAIGHT_CORRECTION_RPM;
      }

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
  Apply_Motor_Output(left_command, left_pwm_duty,
                     right_command, right_pwm_duty);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK ||
      HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK)
  {
    Error_Handler();
  }
  left_encoder_previous = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
  right_encoder_previous = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  encoder_report_tick = HAL_GetTick();

  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ||
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_GPIO_WritePin(GPIOB, STBY_Pin, GPIO_PIN_SET);
  Apply_Motor_Output(0, 0, 0, 0);
  motor_control_enabled = 1U;
  {
    static const uint8_t controls_message[] =
      "Keys: F=1cell fwd, B=1cell rev, J=90L, K=90R, U=180, W/S/A/D=drive, Space/X=stop, R=reset\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)controls_message,
                      sizeof(controls_message) - 1U, HAL_MAX_DELAY);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint8_t key;
    int8_t requested_left = (int8_t)left_motor_command;
    int8_t requested_right = (int8_t)right_motor_command;
    float req_left_rpm = MOTOR_TARGET_RPM;
    float req_right_rpm = MOTOR_TARGET_RPM;
    uint8_t command_changed = 0U;

    if (HAL_UART_Receive(&huart2, &key, 1, 0) == HAL_OK)
    {
      switch (key)
      {
        case 'w':
        case 'W':
          requested_left = 1;
          requested_right = 1;
          req_left_rpm = MOTOR_TARGET_RPM;
          req_right_rpm = MOTOR_TARGET_RPM;
          command_changed = 1U;
          break;
        case 's':
        case 'S':
          requested_left = -1;
          requested_right = -1;
          req_left_rpm = MOTOR_TARGET_RPM;
          req_right_rpm = MOTOR_TARGET_RPM;
          command_changed = 1U;
          break;
        case 'a':
        case 'A':
          requested_left = -1;
          requested_right = 1;
          req_left_rpm = MOTOR_TARGET_RPM;
          req_right_rpm = MOTOR_TARGET_RPM;
          command_changed = 1U;
          break;
        case 'd':
        case 'D':
          requested_left = 1;
          requested_right = -1;
          req_left_rpm = MOTOR_TARGET_RPM;
          req_right_rpm = MOTOR_TARGET_RPM;
          command_changed = 1U;
          break;
        case 'q':
        case 'Q':
          /* Smooth curve forward-left: Left motor 60 RPM forward, Right motor 150 RPM forward */
          requested_left = 1;
          requested_right = 1;
          req_left_rpm = MOTOR_CURVE_SLOW_RPM;
          req_right_rpm = MOTOR_CURVE_FAST_RPM;
          command_changed = 1U;
          break;
        case 'e':
        case 'E':
          /* Smooth curve forward-right: Left motor 150 RPM forward, Right motor 60 RPM forward */
          requested_left = 1;
          requested_right = 1;
          req_left_rpm = MOTOR_CURVE_FAST_RPM;
          req_right_rpm = MOTOR_CURVE_SLOW_RPM;
          command_changed = 1U;
          break;
        case 'z':
        case 'Z':
          /* Smooth curve reverse-left: Left motor 60 RPM reverse, Right motor 150 RPM reverse */
          requested_left = -1;
          requested_right = -1;
          req_left_rpm = MOTOR_CURVE_SLOW_RPM;
          req_right_rpm = MOTOR_CURVE_FAST_RPM;
          command_changed = 1U;
          break;
        case 'c':
        case 'C':
          /* Smooth curve reverse-right: Left motor 150 RPM reverse, Right motor 60 RPM reverse */
          requested_left = -1;
          requested_right = -1;
          req_left_rpm = MOTOR_CURVE_FAST_RPM;
          req_right_rpm = MOTOR_CURVE_SLOW_RPM;
          command_changed = 1U;
          break;
        case 'f':
        case 'F':
          Start_Targeted_Move(CELL_TICKS, CELL_TICKS);
          command_changed = 0U;
          break;
        case 'b':
        case 'B':
          Start_Targeted_Move(-CELL_TICKS, -CELL_TICKS);
          command_changed = 0U;
          break;
        case 'j':
        case 'J':
          Start_Targeted_Move(-TURN_90_TICKS, TURN_90_TICKS);
          command_changed = 0U;
          break;
        case 'k':
        case 'K':
          Start_Targeted_Move(TURN_90_TICKS, -TURN_90_TICKS);
          command_changed = 0U;
          break;
        case 'u':
        case 'U':
          Start_Targeted_Move(-TURN_180_TICKS, TURN_180_TICKS);
          command_changed = 0U;
          break;
        case 'x':
        case 'X':
        case ' ':
          requested_left = 0;
          requested_right = 0;
          req_left_rpm = 0.0f;
          req_right_rpm = 0.0f;
          command_changed = 1U;
          break;
        case 'r':
        case 'R':
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
            encoder_report_tick = HAL_GetTick();
            if (interrupt_state == 0U)
            {
              __enable_irq();
            }
          }
          break;
        }
        default:
          break;
      }
      if (command_changed != 0U)
      {
        Set_Motor_Commands(requested_left, req_left_rpm,
                           requested_right, req_right_rpm,
                           0U);
      }
    }

    if (cell_move_timeout != 0U)
    {
      static const uint8_t timeout_message[] = "CELL_TIMEOUT: motors stopped\r\n";
      cell_move_timeout = 0U;
      HAL_UART_Transmit(&huart2, (uint8_t *)timeout_message,
                        sizeof(timeout_message) - 1U, HAL_MAX_DELAY);
    }

    if ((HAL_GetTick() - encoder_report_tick) >= MOTOR_REPORT_PERIOD_MS)
    {
      char report[128];
      int report_length;
      int32_t left_report_delta = left_encoder_total - left_report_total;
      int32_t right_report_delta = right_encoder_total - right_report_total;

      left_report_total = left_encoder_total;
      right_report_total = right_encoder_total;
      encoder_report_tick = HAL_GetTick();

      report_length = snprintf(report, sizeof(report),
                               "L:%ld d:%ld RPM:%ld PWM:%u | R:%ld d:%ld RPM:%ld PWM:%u\r\n",
                               (long)left_encoder_total, (long)left_report_delta,
                               (long)left_measured_rpm, (unsigned int)left_pwm_duty,
                               (long)right_encoder_total, (long)right_report_delta,
                               (long)right_measured_rpm, (unsigned int)right_pwm_duty);
      if (report_length > 0)
      {
        HAL_UART_Transmit(&huart2, (uint8_t *)report, (uint16_t)report_length, HAL_MAX_DELAY);
      }
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 4;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 4;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 4;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 4;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, AIN1_Pin|AIN2_Pin|BIN1_Pin|BIN2_Pin
                          |STBY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : AIN1_Pin AIN2_Pin BIN1_Pin BIN2_Pin
                           STBY_Pin */
  GPIO_InitStruct.Pin = AIN1_Pin|AIN2_Pin|BIN1_Pin|BIN2_Pin
                          |STBY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
