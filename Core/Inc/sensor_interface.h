#ifndef SENSOR_INTERFACE_H
#define SENSOR_INTERFACE_H

#include <stdint.h>
#include <stdbool.h>

/**
 * Supported proximity sensor data formats.
 */
typedef enum {
    SENSOR_TYPE_BINARY = 0,    // Digital obstacle detection (true = wall present)
    SENSOR_TYPE_DISTANCE_MM,   // Measured distance in millimeters (e.g. ToF VL53L0X)
    SENSOR_TYPE_DISTANCE_CM,   // Measured distance in centimeters (e.g. Ultrasonic)
    SENSOR_TYPE_ANALOG_RAW     // Raw ADC value (e.g. analog Sharp IR)
} SensorDataType;

/**
 * Single proximity sensor channel measurement.
 */
typedef struct {
    SensorDataType type;
    bool binary_detected;      // Used when type == SENSOR_TYPE_BINARY
    uint16_t raw_value;        // Distance (mm/cm) or ADC ticks
    uint16_t wall_threshold;   // Threshold to compare against raw_value
    bool higher_means_closer;  // For analog ADC: true if higher ADC value indicates closer obstacle
} ProximityReading;

/**
 * Complete sensor array reading covering robot directions.
 */
typedef struct {
    ProximityReading front;
    ProximityReading left;
    ProximityReading right;
    ProximityReading rear;
} ProximitySensorArray;

/**
 * Evaluated relative wall presence around the robot.
 */
typedef struct {
    bool wall_front;
    bool wall_left;
    bool wall_right;
    bool wall_rear;
} DetectedWalls;

/**
 * Process sensor readings into standardized boolean wall detections.
 */
DetectedWalls sensor_interface_evaluate(const ProximitySensorArray *sensors);

/**
 * Convenience helper constructors for sensor setups.
 */
ProximitySensorArray sensor_make_binary(bool front, bool left, bool right);
ProximitySensorArray sensor_make_tof_mm(
    uint16_t front_mm,
    uint16_t left_mm,
    uint16_t right_mm,
    uint16_t threshold_mm
);
ProximitySensorArray sensor_make_ultrasonic_cm(
    uint16_t front_cm,
    uint16_t left_cm,
    uint16_t right_cm,
    uint16_t threshold_cm
);
ProximitySensorArray sensor_make_analog_adc(
    uint16_t front_adc,
    uint16_t left_adc,
    uint16_t right_adc,
    uint16_t threshold_adc,
    bool higher_means_closer
);

#endif /* SENSOR_INTERFACE_H */

