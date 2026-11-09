/**
  ******************************************************************************
  * @file           : telemetry_cmd.h
  * @brief          : Header for telemetry_cmd.c
  *                   UART telemetry streaming and teleop/autonomous command parser.
  ******************************************************************************
  */

#ifndef __TELEMETRY_CMD_H
#define __TELEMETRY_CMD_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief  Initializes the telemetry and command parser module with a UART handle.
  * @param  huart  Pointer to UART_HandleTypeDef (e.g. &huart2).
  */
void Telemetry_Cmd_Init(UART_HandleTypeDef *huart);

/**
  * @brief  Transmits the startup control instructions message over UART.
  */
void Telemetry_Send_Startup_Banner(void);

/**
  * @brief  Non-blocking check for received UART control characters.
  *         Executes continuous driving or autonomous targeted moves.
  */
void Telemetry_Cmd_Process(void);

/**
  * @brief  Sends periodic telemetry report (100 ms interval) over UART.
  */
void Telemetry_Send_Periodic(void);

#ifdef __cplusplus
}
#endif

#endif /* __TELEMETRY_CMD_H */
