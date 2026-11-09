#include "sensor_interface.h"

static bool evaluate_single_channel(const ProximityReading *reading) {
    if (!reading) return false;

    switch (reading->type) {
        case SENSOR_TYPE_BINARY:
            return reading->binary_detected;

        case SENSOR_TYPE_DISTANCE_MM:
        case SENSOR_TYPE_DISTANCE_CM:
            // Obstacle detected if measured distance is within threshold (and valid > 0)
            return (reading->raw_value > 0 && reading->raw_value <= reading->wall_threshold);

        case SENSOR_TYPE_ANALOG_RAW:
            if (reading->higher_means_closer) {
                return (reading->raw_value >= reading->wall_threshold);
            } else {
                return (reading->raw_value <= reading->wall_threshold);
            }

        default:
            return false;
    }
}

DetectedWalls sensor_interface_evaluate(const ProximitySensorArray *sensors) {
    DetectedWalls walls = {false, false, false, false};
    if (!sensors) return walls;

    walls.wall_front = evaluate_single_channel(&sensors->front);
    walls.wall_left  = evaluate_single_channel(&sensors->left);
    walls.wall_right = evaluate_single_channel(&sensors->right);
    walls.wall_rear  = evaluate_single_channel(&sensors->rear);

    return walls;
}

ProximitySensorArray sensor_make_binary(bool front, bool left, bool right) {
    ProximitySensorArray arr = {0};
    arr.front.type = SENSOR_TYPE_BINARY;
    arr.front.binary_detected = front;

    arr.left.type = SENSOR_TYPE_BINARY;
    arr.left.binary_detected = left;

    arr.right.type = SENSOR_TYPE_BINARY;
    arr.right.binary_detected = right;

    arr.rear.type = SENSOR_TYPE_BINARY;
    arr.rear.binary_detected = false;

    return arr;
}

ProximitySensorArray sensor_make_tof_mm(
    uint16_t front_mm,
    uint16_t left_mm,
    uint16_t right_mm,
    uint16_t threshold_mm
) {
    ProximitySensorArray arr = {0};
    arr.front.type = SENSOR_TYPE_DISTANCE_MM;
    arr.front.raw_value = front_mm;
    arr.front.wall_threshold = threshold_mm;

    arr.left.type = SENSOR_TYPE_DISTANCE_MM;
    arr.left.raw_value = left_mm;
    arr.left.wall_threshold = threshold_mm;

    arr.right.type = SENSOR_TYPE_DISTANCE_MM;
    arr.right.raw_value = right_mm;
    arr.right.wall_threshold = threshold_mm;

    arr.rear.type = SENSOR_TYPE_DISTANCE_MM;
    arr.rear.raw_value = 0;
    arr.rear.wall_threshold = threshold_mm;

    return arr;
}

ProximitySensorArray sensor_make_ultrasonic_cm(
    uint16_t front_cm,
    uint16_t left_cm,
    uint16_t right_cm,
    uint16_t threshold_cm
) {
    ProximitySensorArray arr = {0};
    arr.front.type = SENSOR_TYPE_DISTANCE_CM;
    arr.front.raw_value = front_cm;
    arr.front.wall_threshold = threshold_cm;

    arr.left.type = SENSOR_TYPE_DISTANCE_CM;
    arr.left.raw_value = left_cm;
    arr.left.wall_threshold = threshold_cm;

    arr.right.type = SENSOR_TYPE_DISTANCE_CM;
    arr.right.raw_value = right_cm;
    arr.right.wall_threshold = threshold_cm;

    arr.rear.type = SENSOR_TYPE_DISTANCE_CM;
    arr.rear.raw_value = 0;
    arr.rear.wall_threshold = threshold_cm;

    return arr;
}

ProximitySensorArray sensor_make_analog_adc(
    uint16_t front_adc,
    uint16_t left_adc,
    uint16_t right_adc,
    uint16_t threshold_adc,
    bool higher_means_closer
) {
    ProximitySensorArray arr = {0};
    arr.front.type = SENSOR_TYPE_ANALOG_RAW;
    arr.front.raw_value = front_adc;
    arr.front.wall_threshold = threshold_adc;
    arr.front.higher_means_closer = higher_means_closer;

    arr.left.type = SENSOR_TYPE_ANALOG_RAW;
    arr.left.raw_value = left_adc;
    arr.left.wall_threshold = threshold_adc;
    arr.left.higher_means_closer = higher_means_closer;

    arr.right.type = SENSOR_TYPE_ANALOG_RAW;
    arr.right.raw_value = right_adc;
    arr.right.wall_threshold = threshold_adc;
    arr.right.higher_means_closer = higher_means_closer;

    arr.rear.type = SENSOR_TYPE_ANALOG_RAW;
    arr.rear.raw_value = 0;
    arr.rear.wall_threshold = threshold_adc;
    arr.rear.higher_means_closer = higher_means_closer;

    return arr;
}

