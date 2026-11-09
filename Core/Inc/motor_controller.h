/**
  ******************************************************************************
  * @file           : motor_controller.h
  * @brief          : Header for motor_controller.c
  *                   Closed-loop motor drive, encoder tracking, and motion control.
  ******************************************************************************
  */

#ifndef __MOTOR_CONTROLLER_H
#define __MOTOR_CONTROLLER_H

#include <stdint.h>
#include "main.h"
#include "motor_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief Snapshot of encoder and motor performance for telemetry.
  */
typedef struct {
  int32_t  left_encoder_total;
  int32_t  right_encoder_total;
  int32_t  left_report_delta;
  int32_t  right_report_delta;
  float    left_measured_rpm;
  float    right_measured_rpm;
  uint16_t left_pwm_duty;
  uint16_t right_pwm_duty;
} MotorTelemetry_t;

/**
  * @brief  Initializes timers, starts PWM and encoder channels, and enables standby pin.
  * @param  pwm_timer            Handle to TIM1 for PWM generation.
  * @param  left_encoder_timer   Handle to TIM2 for left wheel encoder.
  * @param  right_encoder_timer  Handle to TIM3 for right wheel encoder.
  */
void Motor_Controller_Init(TIM_HandleTypeDef *pwm_timer,
                           TIM_HandleTypeDef *left_encoder_timer,
                           TIM_HandleTypeDef *right_encoder_timer);

/**
  * @brief  1 kHz periodic control tick called from SysTick_Handler.
  *         Calculates RPM, evaluates PI loop and position hold, and sets PWM duty.
  */
void Motor_Control_Tick(void);

/**
  * @brief  Sets manual continuous motor drive commands and target speeds.
  * @param  left_cmd   Direction: +1 (Forward), -1 (Reverse), 0 (Brake/Stop)
  * @param  left_rpm   Target speed in RPM
  * @param  right_cmd  Direction: +1 (Forward), -1 (Reverse), 0 (Brake/Stop)
  * @param  right_rpm  Target speed in RPM
  */
void Motor_Set_Commands(int8_t left_cmd, float left_rpm,
                        int8_t right_cmd, float right_rpm);

/**
  * @brief  Initiates an autonomous signed targeted move (cells, 90° turns, 180° turns).
  * @param  left_delta   Target tick displacement for left wheel.
  * @param  right_delta  Target tick displacement for right wheel.
  */
void Motor_Start_Targeted_Move(int32_t left_delta, int32_t right_delta);

/**
  * @brief  Stops both motors immediately and cancels any active targeted move.
  */
void Motor_Stop(void);

/**
  * @brief  Resets accumulated encoder counts to zero.
  */
void Motor_Reset_Encoders(void);

/**
  * @brief  Checks whether a targeted move timed out and clears the flag.
  * @retval 1 if timeout occurred, 0 otherwise.
  */
uint8_t Motor_Check_And_Clear_Timeout(void);

/**
  * @brief  Returns 1 if a targeted move is currently active, 0 otherwise.
  * @retval 1 if active, 0 otherwise.
  */
uint8_t Motor_Is_Move_Active(void);

/**
  * @brief  Copies the latest encoder and speed telemetry into the provided struct.
  * @param  telemetry  Pointer to MotorTelemetry_t destination.
  */
void Motor_Get_Telemetry(MotorTelemetry_t *telemetry);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROLLER_H */
