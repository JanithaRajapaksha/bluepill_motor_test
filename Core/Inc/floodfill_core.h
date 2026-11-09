#ifndef FLOODFILL_CORE_H
#define FLOODFILL_CORE_H

#include <stdint.h>
#include <stdbool.h>
#include "maze_map.h"

/**
 * Perform breadth-first flood-fill algorithm starting from goal cells.
 * Operates purely on statically-allocated buffers in the MazeMap (zero heap usage).
 * Exploration order is strictly North, East, South, West to preserve deterministic tie-breaking.
 */
void floodfill_compute(
    MazeMap *map,
    const uint8_t goal_x[],
    const uint8_t goal_y[],
    uint8_t num_goals
);

/**
 * Determine the next cell along the shortest-path gradient.
 * Looks for accessible neighbor with distance == current_distance - 1 in order:
 * North, East, South, West.
 * Returns true if such neighbor is found, false otherwise.
 */
bool floodfill_get_next_cell(
    const MazeMap *map,
    uint8_t cur_x,
    uint8_t cur_y,
    uint8_t *next_x,
    uint8_t *next_y,
    Direction *out_dir
);

#endif /* FLOODFILL_CORE_H */

