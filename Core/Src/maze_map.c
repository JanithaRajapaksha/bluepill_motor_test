#include "maze_map.h"
#include <string.h>

static const uint8_t WALL_MASKS[DIR_COUNT] = {
    WALL_NORTH, // DIR_NORTH = 0
    WALL_EAST,  // DIR_EAST  = 1
    WALL_SOUTH, // DIR_SOUTH = 2
    WALL_WEST   // DIR_WEST  = 3
};

static const uint8_t RECIPROCAL_MASKS[DIR_COUNT] = {
    WALL_SOUTH, // opposite of North is South
    WALL_WEST,  // opposite of East is West
    WALL_NORTH, // opposite of South is North
    WALL_EAST   // opposite of West is East
};

void maze_map_init(MazeMap *map, uint8_t size) {
    if (!map) return;
    if (size > MICROMOUSE_MAX_MAZE_SIZE) {
        size = MICROMOUSE_MAX_MAZE_SIZE;
    }
    map->size = size;
    memset(map->walls, 0, sizeof(map->walls));
    memset(map->distances, DISTANCE_UNREACHABLE, sizeof(map->distances));
}

void maze_map_init_empty(MazeMap *map, uint8_t size) {
    maze_map_init(map, size);
    for (uint8_t y = 0; y < size; ++y) {
        for (uint8_t x = 0; x < size; ++x) {
            if (x == 0) {
                maze_map_set_wall(map, x, y, DIR_WEST);
            }
            if (x == size - 1) {
                maze_map_set_wall(map, x, y, DIR_EAST);
            }
            if (y == 0) {
                maze_map_set_wall(map, x, y, DIR_SOUTH);
            }
            if (y == size - 1) {
                maze_map_set_wall(map, x, y, DIR_NORTH);
            }
        }
    }
}

void maze_map_set_wall(MazeMap *map, uint8_t x, uint8_t y, Direction dir) {
    if (!map || x >= map->size || y >= map->size || dir >= DIR_COUNT) {
        return;
    }

    uint16_t idx = maze_cell_index(map->size, x, y);
    map->walls[idx] |= WALL_MASKS[dir];

    // Set reciprocal wall on adjacent cell if in bounds
    switch (dir) {
        case DIR_NORTH:
            if (y + 1 < map->size) {
                map->walls[maze_cell_index(map->size, x, y + 1)] |= RECIPROCAL_MASKS[DIR_NORTH];
            }
            break;
        case DIR_EAST:
            if (x + 1 < map->size) {
                map->walls[maze_cell_index(map->size, x + 1, y)] |= RECIPROCAL_MASKS[DIR_EAST];
            }
            break;
        case DIR_SOUTH:
            if (y > 0) {
                map->walls[maze_cell_index(map->size, x, y - 1)] |= RECIPROCAL_MASKS[DIR_SOUTH];
            }
            break;
        case DIR_WEST:
            if (x > 0) {
                map->walls[maze_cell_index(map->size, x - 1, y)] |= RECIPROCAL_MASKS[DIR_WEST];
            }
            break;
        default:
            break;
    }
}

bool maze_map_has_wall(const MazeMap *map, uint8_t x, uint8_t y, Direction dir) {
    if (!map || x >= map->size || y >= map->size || dir >= DIR_COUNT) {
        return true; // Out of bounds treated as walled
    }
    uint16_t idx = maze_cell_index(map->size, x, y);
    return (map->walls[idx] & WALL_MASKS[dir]) != 0;
}

void maze_map_set_visited(MazeMap *map, uint8_t x, uint8_t y) {
    if (!map || x >= map->size || y >= map->size) return;
    map->walls[maze_cell_index(map->size, x, y)] |= CELL_VISITED;
}

bool maze_map_is_visited(const MazeMap *map, uint8_t x, uint8_t y) {
    if (!map || x >= map->size || y >= map->size) return false;
    return (map->walls[maze_cell_index(map->size, x, y)] & CELL_VISITED) != 0;
}

void maze_map_reset_distances(MazeMap *map) {
    if (!map) return;
    memset(map->distances, DISTANCE_UNREACHABLE, sizeof(map->distances));
}

uint8_t maze_map_get_centers(const MazeMap *map, uint8_t out_x[4], uint8_t out_y[4]) {
    if (!map || !out_x || !out_y) return 0;
    uint8_t mid = map->size / 2U;
    if (map->size % 2U == 1U) {
        out_x[0] = mid;
        out_y[0] = mid;
        return 1U;
    }
    out_x[0] = mid - 1U; out_y[0] = mid - 1U;
    out_x[1] = mid;      out_y[1] = mid - 1U;
    out_x[2] = mid - 1U; out_y[2] = mid;
    out_x[3] = mid;      out_y[3] = mid;
    return 4U;
}

void maze_map_get_wall_code(const MazeMap *map, uint8_t x, uint8_t y, char out_code[5]) {
    if (!out_code) return;
    out_code[0] = '\0';
    if (!map || x >= map->size || y >= map->size) return;

    uint8_t w = map->walls[maze_cell_index(map->size, x, y)];
    uint8_t idx = 0;
    if (w & WALL_EAST)  out_code[idx++] = 'E';
    if (w & WALL_NORTH) out_code[idx++] = 'N';
    if (w & WALL_SOUTH) out_code[idx++] = 'S';
    if (w & WALL_WEST)  out_code[idx++] = 'W';
    out_code[idx] = '\0';
}

