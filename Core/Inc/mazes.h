#ifndef MAZES_H
#define MAZES_H

#include <stdint.h>
#include <stdbool.h>
#include "maze_map.h"

/**
 * Static benchmark maze configurations.
 * Statically populates an existing MazeMap instance without any dynamic allocations.
 */
void wall_arrangement_5x5(MazeMap *map);
void wall_arrangement_6x6(MazeMap *map);
void wall_arrangement_10x10(MazeMap *map);
void wall_arrangement_16x16(MazeMap *map);

/**
 * Initialize and populate a MazeMap with a preset configuration (5, 6, 10, or 16).
 * Returns true if the size is supported and initialized, false otherwise.
 */
bool build_preset_maze(MazeMap *map, uint8_t size);

#endif /* MAZES_H */
