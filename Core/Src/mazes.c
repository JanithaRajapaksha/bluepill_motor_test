#include "mazes.h"

#define SET_N(m, x, y) maze_map_set_wall((m), (x), (y), DIR_NORTH)
#define SET_S(m, x, y) maze_map_set_wall((m), (x), (y), DIR_SOUTH)
#define SET_E(m, x, y) maze_map_set_wall((m), (x), (y), DIR_EAST)
#define SET_W(m, x, y) maze_map_set_wall((m), (x), (y), DIR_WEST)

void wall_arrangement_5x5(MazeMap *map) {
    if (!map) return;
    SET_E(map, 0, 0);
    SET_N(map, 2, 0);
    SET_E(map, 2, 0);
    SET_N(map, 3, 0);
    SET_N(map, 2, 1);
    SET_E(map, 2, 1);
    SET_S(map, 0, 2);
    SET_E(map, 0, 2);
    SET_E(map, 1, 2);
    SET_N(map, 3, 2);
    SET_E(map, 3, 2);
    SET_N(map, 0, 3);
    SET_E(map, 1, 4);
    SET_S(map, 3, 4);
    SET_S(map, 4, 4);
}

void wall_arrangement_6x6(MazeMap *map) {
    if (!map) return;
    SET_N(map, 2, 0);
    SET_N(map, 3, 0);
    SET_E(map, 4, 0);
    SET_W(map, 5, 0);
    SET_E(map, 0, 1);
    SET_N(map, 2, 1);
    SET_N(map, 3, 1);
    SET_W(map, 4, 1);
    SET_W(map, 5, 1);
    SET_E(map, 0, 2);
    SET_E(map, 1, 2);
    SET_E(map, 3, 2);
    SET_N(map, 4, 2);
    SET_N(map, 5, 2);
    SET_N(map, 1, 3);
    SET_E(map, 1, 3);
    SET_N(map, 2, 3);
    SET_E(map, 3, 3);
    SET_N(map, 1, 4);
    SET_N(map, 2, 4);
    SET_N(map, 3, 4);
    SET_E(map, 4, 4);
    SET_E(map, 4, 5);
}

void wall_arrangement_10x10(MazeMap *map) {
    if (!map) return;
    // Row 0
    SET_E(map, 0, 1);
    SET_N(map, 0, 3);
    SET_W(map, 0, 4);
    SET_E(map, 0, 6);
    // Row 1
    SET_E(map, 1, 0);
    SET_E(map, 1, 2);
    SET_N(map, 1, 5);
    SET_W(map, 1, 6);
    // Row 2
    SET_N(map, 2, 1);
    SET_E(map, 2, 3);
    SET_W(map, 2, 4);
    SET_W(map, 2, 6);
    // Row 3
    SET_E(map, 3, 2);
    SET_N(map, 3, 3);
    SET_N(map, 3, 4);
    SET_W(map, 3, 5);
    SET_E(map, 3, 7);
    // Row 4
    SET_E(map, 4, 2);
    SET_N(map, 4, 3);
    SET_N(map, 4, 4);
    SET_W(map, 4, 5);
    SET_W(map, 4, 7);
    // Row 5
    SET_E(map, 5, 2);
    SET_S(map, 5, 3);
    SET_S(map, 5, 4);
    SET_W(map, 5, 5);
    SET_W(map, 5, 8);
    // Row 6
    SET_N(map, 6, 3);
    SET_W(map, 6, 4);
    SET_E(map, 6, 6);
    SET_E(map, 6, 7);
    // Row 7
    SET_N(map, 7, 1);
    SET_N(map, 7, 3);
    SET_N(map, 7, 5);
    SET_W(map, 7, 8);
    // Row 8
    SET_N(map, 8, 2);
    SET_E(map, 8, 4);
    SET_E(map, 8, 6);
    SET_E(map, 8, 7);
    // Row 9
    SET_N(map, 9, 0);
    SET_W(map, 9, 5);
    SET_N(map, 9, 8);
    // Guard the center goal cells
    SET_S(map, 4, 4);
    SET_S(map, 4, 5);
    SET_N(map, 5, 4);
    SET_N(map, 5, 5);
    SET_W(map, 4, 3);
    SET_E(map, 4, 6);
    SET_W(map, 5, 3);
    SET_E(map, 5, 6);
}

void wall_arrangement_16x16(MazeMap *map) {
    if (!map) return;
    // Column 0
    SET_E(map, 0, 0);
    SET_E(map, 0, 1);
    SET_E(map, 0, 4);
    SET_E(map, 0, 5);
    SET_E(map, 0, 6);
    SET_E(map, 0, 7);
    SET_E(map, 0, 8);
    SET_E(map, 0, 9);
    SET_E(map, 0, 10);
    SET_E(map, 0, 11);
    SET_E(map, 0, 14);
    SET_E(map, 0, 15);
    // Column 1
    SET_E(map, 1, 1);
    SET_E(map, 1, 2);
    SET_N(map, 1, 2);
    SET_E(map, 1, 3);
    SET_E(map, 1, 4);
    SET_E(map, 1, 5);
    SET_E(map, 1, 6);
    SET_E(map, 1, 7);
    SET_E(map, 1, 8);
    SET_E(map, 1, 9);
    SET_E(map, 1, 10);
    SET_E(map, 1, 11);
    SET_E(map, 1, 12);
    SET_N(map, 1, 12);
    SET_E(map, 1, 13);
    SET_E(map, 1, 14);
    // Column 2
    SET_N(map, 2, 0);
    SET_N(map, 2, 1);
    SET_E(map, 2, 3);
    SET_E(map, 2, 4);
    SET_E(map, 2, 5);
    SET_E(map, 2, 6);
    SET_E(map, 2, 8);
    SET_E(map, 2, 9);
    SET_E(map, 2, 10);
    SET_E(map, 2, 11);
    SET_E(map, 2, 12);
    SET_N(map, 2, 13);
    SET_N(map, 2, 14);
    // Column 3
    SET_N(map, 3, 0);
    SET_N(map, 3, 1);
    SET_N(map, 3, 2);
    SET_E(map, 3, 4);
    SET_E(map, 3, 5);
    SET_E(map, 3, 6);
    SET_E(map, 3, 8);
    SET_E(map, 3, 9);
    SET_E(map, 3, 10);
    SET_E(map, 3, 11);
    SET_N(map, 3, 12);
    SET_N(map, 3, 13);
    SET_N(map, 3, 14);
    // Column 4
    SET_N(map, 4, 1);
    SET_N(map, 4, 2);
    SET_N(map, 4, 3);
    SET_N(map, 4, 4);
    SET_E(map, 4, 5);
    SET_E(map, 4, 6);
    SET_E(map, 4, 8);
    SET_E(map, 4, 9);
    SET_E(map, 4, 10);
    SET_N(map, 4, 11);
    SET_N(map, 4, 12);
    SET_N(map, 4, 13);
    SET_N(map, 4, 14);
    // Column 5
    SET_N(map, 5, 0);
    SET_N(map, 5, 1);
    SET_N(map, 5, 2);
    SET_N(map, 5, 3);
    SET_N(map, 5, 4);
    SET_E(map, 5, 5);
    SET_E(map, 5, 6);
    SET_E(map, 5, 8);
    SET_E(map, 5, 9);
    SET_N(map, 5, 10);
    SET_N(map, 5, 11);
    SET_N(map, 5, 12);
    SET_N(map, 5, 13);
    SET_N(map, 5, 14);
    // Column 6
    SET_N(map, 6, 0);
    SET_N(map, 6, 1);
    SET_N(map, 6, 2);
    SET_N(map, 6, 3);
    SET_N(map, 6, 4);
    SET_N(map, 6, 5);
    SET_E(map, 6, 8);
    SET_N(map, 6, 9);
    SET_N(map, 6, 10);
    SET_N(map, 6, 11);
    SET_N(map, 6, 12);
    SET_N(map, 6, 13);
    SET_N(map, 6, 14);
    // Column 7
    SET_N(map, 7, 0);
    SET_N(map, 7, 1);
    SET_E(map, 7, 1);
    SET_N(map, 7, 2);
    SET_N(map, 7, 3);
    SET_N(map, 7, 4);
    SET_N(map, 7, 5);
    SET_N(map, 7, 6);
    SET_N(map, 7, 8);
    SET_N(map, 7, 9);
    SET_N(map, 7, 10);
    SET_E(map, 7, 10);
    SET_N(map, 7, 11);
    SET_E(map, 7, 11);
    SET_N(map, 7, 12);
    SET_N(map, 7, 14);
    // Column 8
    SET_N(map, 8, 0);
    SET_N(map, 8, 2);
    SET_N(map, 8, 4);
    SET_N(map, 8, 6);
    SET_E(map, 8, 7);
    SET_E(map, 8, 8);
    SET_N(map, 8, 8);
    SET_N(map, 8, 9);
    SET_N(map, 8, 10);
    SET_N(map, 8, 11);
    SET_N(map, 8, 12);
    SET_N(map, 8, 13);
    SET_N(map, 8, 14);
    // Column 9
    SET_N(map, 9, 0);
    SET_N(map, 9, 1);
    SET_N(map, 9, 2);
    SET_N(map, 9, 3);
    SET_N(map, 9, 4);
    SET_N(map, 9, 5);
    SET_E(map, 9, 6);
    SET_E(map, 9, 7);
    SET_E(map, 9, 8);
    SET_E(map, 9, 9);
    SET_N(map, 9, 9);
    SET_N(map, 9, 10);
    SET_N(map, 9, 11);
    SET_N(map, 9, 12);
    SET_N(map, 9, 13);
    // Column 10
    SET_N(map, 10, 0);
    SET_N(map, 10, 1);
    SET_N(map, 10, 2);
    SET_N(map, 10, 3);
    SET_N(map, 10, 4);
    SET_E(map, 10, 5);
    SET_E(map, 10, 6);
    SET_E(map, 10, 8);
    SET_E(map, 10, 9);
    SET_E(map, 10, 10);
    SET_N(map, 10, 10);
    SET_N(map, 10, 11);
    SET_E(map, 10, 13);
    SET_N(map, 10, 13);
    SET_N(map, 10, 14);
    // Column 11
    SET_N(map, 11, 0);
    SET_N(map, 11, 1);
    SET_N(map, 11, 2);
    SET_N(map, 11, 3);
    SET_E(map, 11, 4);
    SET_E(map, 11, 5);
    SET_E(map, 11, 6);
    SET_E(map, 11, 7);
    SET_E(map, 11, 8);
    SET_E(map, 11, 10);
    SET_E(map, 11, 11);
    SET_N(map, 11, 11);
    SET_N(map, 11, 12);
    SET_N(map, 11, 14);
    // Column 12
    SET_N(map, 12, 0);
    SET_N(map, 12, 1);
    SET_N(map, 12, 2);
    SET_E(map, 12, 2);
    SET_E(map, 12, 3);
    SET_E(map, 12, 4);
    SET_E(map, 12, 5);
    SET_E(map, 12, 6);
    SET_E(map, 12, 7);
    SET_E(map, 12, 8);
    SET_E(map, 12, 9);
    SET_E(map, 12, 10);
    SET_E(map, 12, 11);
    SET_N(map, 12, 11);
    SET_E(map, 12, 12);
    SET_N(map, 12, 12);
    SET_N(map, 12, 13);
    // Column 13
    SET_N(map, 13, 0);
    SET_N(map, 13, 1);
    SET_E(map, 13, 2);
    SET_E(map, 13, 3);
    SET_E(map, 13, 4);
    SET_E(map, 13, 5);
    SET_E(map, 13, 7);
    SET_E(map, 13, 8);
    SET_E(map, 13, 9);
    SET_E(map, 13, 10);
    SET_E(map, 13, 11);
    SET_E(map, 13, 12);
    SET_E(map, 13, 13);
    SET_N(map, 13, 13);
    SET_N(map, 13, 14);
    // Column 14
    SET_N(map, 14, 0);
    SET_N(map, 14, 1);
    SET_E(map, 14, 2);
    SET_E(map, 14, 3);
    SET_E(map, 14, 5);
    SET_E(map, 14, 6);
    SET_E(map, 14, 7);
    SET_E(map, 14, 8);
    SET_E(map, 14, 9);
    SET_E(map, 14, 10);
    SET_E(map, 14, 11);
    SET_E(map, 14, 12);
    SET_E(map, 14, 13);
    SET_E(map, 14, 14);
    SET_N(map, 14, 14);
    // Unparsed: PRESET_CONFIGURATIONS = {
    // Unparsed: 5: wall_arrangement_x5,
    // Unparsed: 6: wall_arrangement_x6,
    // Unparsed: 10: wall_arrangement_10x10,
    // Unparsed: 16: wall_arrangement_16x16,
    // Unparsed: }
}

bool build_preset_maze(MazeMap *map, uint8_t size) {
    if (!map) return false;
    maze_map_init_empty(map, size);

    switch (size) {
        case 5:
            wall_arrangement_5x5(map);
            return true;
        case 6:
            wall_arrangement_6x6(map);
            return true;
        case 10:
            wall_arrangement_10x10(map);
            return true;
        case 16:
            wall_arrangement_16x16(map);
            return true;
        default:
            return false;
    }
}
