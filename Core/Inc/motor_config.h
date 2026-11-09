/**
  ******************************************************************************
  * @file           : motor_config.h
  * @brief          : Configuration parameters, geometry, gains, and tick targets
  *                   for the differential drive motor controller.
  ******************************************************************************
  */

#ifndef __MOTOR_CONFIG_H
#define __MOTOR_CONFIG_H

#include <stdint.h>

/* ========================================================================== */
/* 1. Hardware Calibration & Geometry                                        */
/* ========================================================================== */
#define LEFT_COUNTS_PER_REV              815.70f   /* Left encoder counts per wheel turn */
#define RIGHT_COUNTS_PER_REV             815.55f   /* Right encoder counts per wheel turn */

/* ========================================================================== */
/* 2. Target Speeds (RPM)                                                    */
/* ========================================================================== */
#define MOTOR_TARGET_RPM                 150.0f    /* Standard cruising speed */
#define MOTOR_CURVE_SLOW_RPM             60.0f     /* Inner wheel speed for curves */
#define MOTOR_CURVE_FAST_RPM             150.0f    /* Outer wheel speed for curves */

/* ========================================================================== */
/* 3. Discrete Autonomous Motions (Encoder Ticks)                             */
/* ========================================================================== */
/*
 * Distance per wheel rev: pi * 33.0 mm = 103.67 mm.
 * Average CPR: 815.625 counts.
 * 1 Cell (134.7 mm):  (134.7 / 103.67) * 815.625 = 1060 ticks
 * 90° Turn (Track 112 mm, arc 87.96 mm): (87.96 / 103.67) * 815.625 = 692 ticks
 * 180° Turn: 2 * 692 = 1384 ticks
 */
#define CELL_TICKS                       1060L     /* Move 1 cell forward/backward */
#define TURN_90_TICKS                    692L      /* In-place 90° spin */
#define TURN_180_TICKS                   1384L     /* In-place 180° U-turn */

/* ========================================================================== */
/* 4. Motion Profiling, Deceleration & Synchronization                       */
/* ========================================================================== */
#define CELL_SLOWDOWN_TICKS              250L      /* Ticks remaining when deceleration begins */
#define CELL_MIN_APPROACH_RPM            60.0f     /* Minimum speed during deceleration ramp */
#define CELL_STRAIGHT_KP                 0.08f     /* P-gain to synchronize left/right wheel progress */
#define CELL_MAX_STRAIGHT_CORRECTION_RPM 30.0f     /* Maximum RPM adjustment from straightness error */
#define CELL_MOVE_TIMEOUT_MS             3000U     /* Safety timeout to abort stalled moves */

/* ========================================================================== */
/* 5. Closed-Loop Position Hold (Active Braking)                              */
/* ========================================================================== */
#define CELL_HOLD_DEADBAND_TICKS         20L       /* Deadband before position correction engages */
#define CELL_HOLD_MIN_RPM                35.0f     /* Minimum hold correction RPM */
#define CELL_HOLD_MAX_RPM                60.0f     /* Maximum hold correction RPM */
#define CELL_HOLD_RPM_PER_TICK           0.25f     /* RPM per tick of position error */
#define CELL_HOLD_FEED_FORWARD_DUTY      180.0f    /* Feed-forward PWM duty during hold */

/* ========================================================================== */
/* 6. Low-Level Control Loop & PI Controller                                 */
/* ========================================================================== */
#define MOTOR_CONTROL_HZ                 1000U     /* Control loop frequency (1 kHz via SysTick) */
#define MOTOR_RPM_WINDOW_MS              20U       /* Moving average window for RPM estimation */
#define MOTOR_REPORT_PERIOD_MS           100U      /* Telemetry reporting interval over UART */
#define MOTOR_PWM_MAX                    600.0f    /* Maximum PWM duty cycle (TIM1 ARR) */
#define MOTOR_FEED_FORWARD_DUTY          300.0f    /* Baseline feed-forward PWM duty at 150 RPM */
#define MOTOR_KP                         0.8f      /* Proportional speed controller gain */
#define MOTOR_KI                         6.5f      /* Integral speed controller gain */

#endif /* __MOTOR_CONFIG_H */
