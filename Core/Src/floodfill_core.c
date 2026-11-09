#include <stddef.h>
#include "floodfill_core.h"

// Statically allocated FIFO queue buffers (zero heap usage, re-entrant per call)
static uint8_t s_queue_x[MICROMOUSE_MAX_CELLS];
static uint8_t s_queue_y[MICROMOUSE_MAX_CELLS];

void floodfill_compute(
    MazeMap *map,
    const uint8_t goal_x[],
    const uint8_t goal_y[],
    uint8_t num_goals
) {
    if (!map || !goal_x || !goal_y || num_goals == 0) return;

    maze_map_reset_distances(map);

    uint16_t head = 0;
    uint16_t tail = 0;

    for (uint8_t i = 0; i < num_goals; ++i) {
        uint8_t gx = goal_x[i];
        uint8_t gy = goal_y[i];
        if (gx < map->size && gy < map->size) {
            uint16_t idx = maze_cell_index(map->size, gx, gy);
            map->distances[idx] = 0;
            s_queue_x[tail] = gx;
            s_queue_y[tail] = gy;
            tail++;
        }
    }

    uint8_t size = map->size;

    while (head < tail) {
        uint8_t cx = s_queue_x[head];
        uint8_t cy = s_queue_y[head];
        head++;

        uint16_t cur_idx = maze_cell_index(size, cx, cy);
        uint8_t next_dist = (uint8_t)(map->distances[cur_idx] + 1U);

        // Check neighbors in canonical order: North, East, South, West

        // 1. North
        if (cy + 1U < size && !maze_map_has_wall(map, cx, cy, DIR_NORTH)) {
            uint16_t n_idx = maze_cell_index(size, cx, cy + 1U);
            if (map->distances[n_idx] == DISTANCE_UNREACHABLE) {
                map->distances[n_idx] = next_dist;
                s_queue_x[tail] = cx;
                s_queue_y[tail] = cy + 1U;
                tail++;
            }
        }

        // 2. East
        if (cx + 1U < size && !maze_map_has_wall(map, cx, cy, DIR_EAST)) {
            uint16_t n_idx = maze_cell_index(size, cx + 1U, cy);
            if (map->distances[n_idx] == DISTANCE_UNREACHABLE) {
                map->distances[n_idx] = next_dist;
                s_queue_x[tail] = cx + 1U;
                s_queue_y[tail] = cy;
                tail++;
            }
        }

        // 3. South
        if (cy > 0U && !maze_map_has_wall(map, cx, cy, DIR_SOUTH)) {
            uint16_t n_idx = maze_cell_index(size, cx, cy - 1U);
            if (map->distances[n_idx] == DISTANCE_UNREACHABLE) {
                map->distances[n_idx] = next_dist;
                s_queue_x[tail] = cx;
                s_queue_y[tail] = cy - 1U;
                tail++;
            }
        }

        // 4. West
        if (cx > 0U && !maze_map_has_wall(map, cx, cy, DIR_WEST)) {
            uint16_t n_idx = maze_cell_index(size, cx - 1U, cy);
            if (map->distances[n_idx] == DISTANCE_UNREACHABLE) {
                map->distances[n_idx] = next_dist;
                s_queue_x[tail] = cx - 1U;
                s_queue_y[tail] = cy;
                tail++;
            }
        }
    }
}

bool floodfill_get_next_cell(
    const MazeMap *map,
    uint8_t cur_x,
    uint8_t cur_y,
    uint8_t *next_x,
    uint8_t *next_y,
    Direction *out_dir
) {
    if (!map || cur_x >= map->size || cur_y >= map->size) {
        return false;
    }

    uint16_t cur_idx = maze_cell_index(map->size, cur_x, cur_y);
    uint8_t cur_dist = map->distances[cur_idx];
    if (cur_dist == 0 || cur_dist == DISTANCE_UNREACHABLE) {
        return false; // Already at goal or unreachable
    }

    uint8_t target_dist = (uint8_t)(cur_dist - 1U);
    uint8_t size = map->size;

    // Check North
    if (cur_y + 1U < size && !maze_map_has_wall(map, cur_x, cur_y, DIR_NORTH)) {
        uint16_t n_idx = maze_cell_index(size, cur_x, cur_y + 1U);
        if (map->distances[n_idx] == target_dist) {
            if (next_x) *next_x = cur_x;
            if (next_y) *next_y = cur_y + 1U;
            if (out_dir) *out_dir = DIR_NORTH;
            return true;
        }
    }

    // Check East
    if (cur_x + 1U < size && !maze_map_has_wall(map, cur_x, cur_y, DIR_EAST)) {
        uint16_t n_idx = maze_cell_index(size, cur_x + 1U, cur_y);
        if (map->distances[n_idx] == target_dist) {
            if (next_x) *next_x = cur_x + 1U;
            if (next_y) *next_y = cur_y;
            if (out_dir) *out_dir = DIR_EAST;
            return true;
        }
    }

    // Check South
    if (cur_y > 0U && !maze_map_has_wall(map, cur_x, cur_y, DIR_SOUTH)) {
        uint16_t n_idx = maze_cell_index(size, cur_x, cur_y - 1U);
        if (map->distances[n_idx] == target_dist) {
            if (next_x) *next_x = cur_x;
            if (next_y) *next_y = cur_y - 1U;
            if (out_dir) *out_dir = DIR_SOUTH;
            return true;
        }
    }

    // Check West
    if (cur_x > 0U && !maze_map_has_wall(map, cur_x, cur_y, DIR_WEST)) {
        uint16_t n_idx = maze_cell_index(size, cur_x - 1U, cur_y);
        if (map->distances[n_idx] == target_dist) {
            if (next_x) *next_x = cur_x - 1U;
            if (next_y) *next_y = cur_y;
            if (out_dir) *out_dir = DIR_WEST;
            return true;
        }
    }

    return false;
}
