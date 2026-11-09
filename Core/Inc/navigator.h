#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include <stdint.h>
#include <stdbool.h>
#include "maze_map.h"
#include "sensor_interface.h"

/**
 * High-level abstract motion commands emitted by the decision module.
 */
typedef enum {
    NAV_CMD_STOP = 0,         // Reached goal destination or halted
    NAV_CMD_MOVE_FORWARD,     // Continue straight into next cell
    NAV_CMD_TURN_LEFT,        // Turn 90 degrees counter-clockwise and advance
    NAV_CMD_TURN_RIGHT,       // Turn 90 degrees clockwise and advance
    NAV_CMD_TURN_AROUND,      // Turn 180 degrees and advance
    NAV_CMD_COUNT
} NavCommand;

/**
 * High-level state of the navigation search.
 */
typedef enum {
    NAV_STATE_SEARCHING = 0,
    NAV_STATE_GOAL_REACHED,
    NAV_STATE_NO_PATH
} NavState;

/**
 * Robot navigation state including current pose and internal discovered map.
 * Entire structure is statically bounded (~520 bytes).
 */
typedef struct {
    uint8_t x;
    uint8_t y;
    Direction orientation;
    NavState state;
    MazeMap map;
} Navigator;

/**
 * Initialize navigator context with starting position and empty maze boundary.
 */
void navigator_init(
    Navigator *nav,
    uint8_t size,
    uint8_t start_x,
    uint8_t start_y,
    Direction start_dir
);

/**
 * Process new sensor readings from the robot's current cell, update internal map,
 * run the flood-fill gradient solver, advance internal pose, and return the
 * high-level motion command for the motor controller.
 */
NavCommand navigator_step(Navigator *nav, const DetectedWalls *walls);

/**
 * String representation of NavCommand for logging / debugging.
 */
const char* nav_command_to_string(NavCommand cmd);

#endif /* NAVIGATOR_H */

