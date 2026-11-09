#include <stddef.h>
#include "navigator.h"
#include "floodfill_core.h"

void navigator_init(
    Navigator *nav,
    uint8_t size,
    uint8_t start_x,
    uint8_t start_y,
    Direction start_dir
) {
    if (!nav) return;
    nav->x = start_x;
    nav->y = start_y;
    nav->orientation = start_dir;
    nav->state = NAV_STATE_SEARCHING;
    maze_map_init_empty(&nav->map, size);
}

NavCommand navigator_step(Navigator *nav, const DetectedWalls *walls) {
    if (!nav) return NAV_CMD_STOP;

    // 1. Mark current cell as visited
    maze_map_set_visited(&nav->map, nav->x, nav->y);

    // 2. Project sensed relative walls to absolute cardinal directions
    if (walls) {
        Direction front_dir = nav->orientation;
        Direction left_dir  = (Direction)((nav->orientation + 3U) % 4U);
        Direction right_dir = (Direction)((nav->orientation + 1U) % 4U);

        if (walls->wall_front) {
            maze_map_set_wall(&nav->map, nav->x, nav->y, front_dir);
        }
        if (walls->wall_left) {
            maze_map_set_wall(&nav->map, nav->x, nav->y, left_dir);
        }
        if (walls->wall_right) {
            maze_map_set_wall(&nav->map, nav->x, nav->y, right_dir);
        }
    }

    // 3. Compute flood-fill distance gradient from goals
    uint8_t goal_x[4];
    uint8_t goal_y[4];
    uint8_t num_goals = maze_map_get_centers(&nav->map, goal_x, goal_y);

    floodfill_compute(&nav->map, goal_x, goal_y, num_goals);

    uint16_t cur_idx = maze_cell_index(nav->map.size, nav->x, nav->y);
    if (nav->map.distances[cur_idx] == 0) {
        nav->state = NAV_STATE_GOAL_REACHED;
        return NAV_CMD_STOP;
    }

    // 4. Find next cell along distance gradient (cur_dist - 1)
    uint8_t next_x = 0;
    uint8_t next_y = 0;
    Direction next_dir = DIR_NORTH;

    bool found = floodfill_get_next_cell(
        &nav->map, nav->x, nav->y,
        &next_x, &next_y, &next_dir
    );

    if (!found) {
        // Fallback for dead end / local minimum
        uint8_t cur_x_arr[1] = { nav->x };
        uint8_t cur_y_arr[1] = { nav->y };
        floodfill_compute(&nav->map, cur_x_arr, cur_y_arr, 1);

        found = floodfill_get_next_cell(
            &nav->map, nav->x, nav->y,
            &next_x, &next_y, &next_dir
        );

        if (!found) {
            nav->state = NAV_STATE_NO_PATH;
            return NAV_CMD_STOP;
        }
    }

    // 5. Determine high-level motion command based on orientation delta
    uint8_t turn = (uint8_t)((next_dir - nav->orientation + 4U) % 4U);
    NavCommand cmd;

    switch (turn) {
        case 0:
            cmd = NAV_CMD_MOVE_FORWARD;
            break;
        case 1:
            cmd = NAV_CMD_TURN_RIGHT;
            break;
        case 2:
            cmd = NAV_CMD_TURN_AROUND;
            break;
        case 3:
            cmd = NAV_CMD_TURN_LEFT;
            break;
        default:
            cmd = NAV_CMD_MOVE_FORWARD;
            break;
    }

    // 6. Advance navigator pose
    nav->orientation = next_dir;
    nav->x = next_x;
    nav->y = next_y;

    return cmd;
}

const char* nav_command_to_string(NavCommand cmd) {
    switch (cmd) {
        case NAV_CMD_STOP:         return "STOP";
        case NAV_CMD_MOVE_FORWARD: return "MOVE_FORWARD";
        case NAV_CMD_TURN_LEFT:    return "TURN_LEFT";
        case NAV_CMD_TURN_RIGHT:   return "TURN_RIGHT";
        case NAV_CMD_TURN_AROUND:  return "TURN_AROUND";
        default:                   return "UNKNOWN";
    }
}
