/*
 File name: State_Controller.h
 Author: Angus McLennan
 Created on: Feb 27, 2024
 Modified: July 9, 2024
 Description: A state machine for use in high powered rockets to detect stages of flight

 Copyright © 2024 Angus McLennan

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef INC_STATE_CONTROLLER_H_
#define INC_STATE_CONTROLLER_H_

#include <cmath>
#include <cstdint>
#include <algorithm>
#include <vector>
#include "digital_filter.h"

/*  Filter Parameters
 The parameters defined below define the operation of the state machine. If these parameters are changed, the state machine may not operate as expected.
 DO NOT CHANGE THESE PARAMETERS
 ********** FILTER PARAMETERS START **********
 */
/* Launch detection parameters */
constexpr float LAUNCH_ACCEL_THRESHOLD = 2.0f;      // g
constexpr float LAUNCH_ACCEL_FILTER_FREQ = 100.0f;  // Hz
constexpr int LAUNCH_ACCEL_FILTER_WIDTH = 20;       // Filter width elements
constexpr float PITCH_OVER_ANGLE_THRESHOLD = 30.0f; // degrees

/* Burnout detection constants */
constexpr float BURNOUT_ACCEL_FILTER_FREQ = 100.0f; // Hz
constexpr int BURNOUT_ACCEL_FILTER_WIDTH = 10;
constexpr float MAX_MOTOR_BURN_TIME = 10.0f;    // Seconds
constexpr float BURNOUT_ACCEL_THRESHOLD = 0.0f; // g

/* Apogee detection constants */
constexpr int VERTICAL_VELOCITY_DETECT_FREQ = 40; // Hz - Frequency that apogee detect algorithm runs at. Also defines the update freq of the altitude and acceleration lp filters
constexpr int VERTICAL_VELOCITY_FILTER_FREQ = 10; // Hz - Frequency that the vertical velocity median filter is updated
constexpr int VERTICAL_VELOCITY_FILTER_WIDTH = 10;
constexpr float APOGEE_DETECT_VELOCITY_THRESHOLD = 1.0f; // m/s
constexpr float ALTITUDE_LP_FILTER_CUTOFF_FREQ = 1.0f;   // Hz
constexpr float ACCEL_LP_FILTER_CUTOFF_FREQ = 1.0f;      // Hz
constexpr float APOGEE_DETECT_ACCEL_THRESHOLD = 2.0f;    // g

// Main deploy altitude detection constants
constexpr float MAIN_DEPLOY_ALTITUDE = 160.0f; // m above the starting altitude (ground)

// Landing detection constants
constexpr float LANDING_SPEED_THRESHOLD = 1.0f;         // m/s (magnitude)
constexpr float LANDING_VELOCITY_LP_CUTOFF_FREQ = 0.1f; // Hz - Cut-off frequency of vertical velocity low pass filter used for detecting landing
constexpr float FLIGHT_TIME_TIMEOUT = 3600.0f;          // seconds

constexpr float DEG_TO_RAD(float degrees)
{
    return (degrees * 0.0174532925);
}

// Enum to define all the stages of flight
enum class FlightState
{
    IDLE_ON_PAD,
    LAUNCHED,
    BURNOUT,
    APOGEE,
    MAIN_CHUTE_ALTITUDE,
    LANDED
};

// Enum to define the axis of the accelerometer that corresponds to 'out the nose' of the rocket
enum class RocketUpAxis
{
    X_AXIS_POSITIVE,
    X_AXIS_NEGATIVE,
    Y_AXIS_POSITIVE,
    Y_AXIS_NEGATIVE,
    Z_AXIS_POSITIVE,
    Z_AXIS_NEGATIVE
};

// Enum to define the continuity state of an e-match
enum class EmatchState
{
    OPEN_CIRCUIT,
    SHORT_CIRCUIT,
    GOOD,
    EMATCH_ERROR
};

// Enum to define the arm state of an e-match
enum class ArmState
{
    DISARMED,
    ARMED
};

struct SystemState
{
    FlightState flight_state;
    RocketUpAxis up_axis;
    EmatchState drogue_ematch_state;
    EmatchState main_ematch_state;
    ArmState drogue_arm_state;
    ArmState main_arm_state;

    uint32_t launch_time;
    float starting_altitude;
    float starting_pressure;
    float starting_temperature;
    uint32_t burnout_time;
    float burnout_altitude;
    uint32_t drogue_deploy_time;
    float drogue_deploy_altitude;
    uint32_t main_deploy_time;
    float main_deploy_altitude;
    uint32_t landing_time;
    float landing_altitude;
    float available_flash_memory_kB;
    float battery_voltage;
};

struct StateMachineInternalState
{
    float angle_from_vertical;
    float filtered_launch_detect_accel;
    float filtered_burnout_detect_x_axis_accel;
    float filtered_apogee_detect_altitude;
    float filtered_apogee_detect_vertical_velocity;
    float filtered_apogee_detect_accel;
    float unfiltered_main_detect_agl_altitude;
    float filtered_landing_detect_vertical_velocity;
};

class StateController
{
public:
    uint8_t init(float starting_altitude, float starting_pressure, float starting_temperature);
    bool detectLaunchAccel(float ax, float ay, float az, float rocket_angle, uint32_t timestamp_ms);
    bool calculateUpAxis(float ax, float ay, float az);
    bool detectBurnoutAccel(float ax, float ay, float az, float altitude, uint32_t timestamp_ms);
    bool detectApogee(float ax, float ay, float az, float altitude, uint32_t timestamp_ms);
    bool detectMainDeployAltitude(float altitude, uint32_t timestamp_ms);
    bool detectLanding(float altitude, uint32_t timestamp_ms);

private:
    SystemState system_state;
    MedianFilter launch_median_filter;
    MedianFilter burnout_median_filter;
    MedianFilter apogee_median_filter;
    ExpLowPassFilter altitude_exp_lp_filter;
    ExpLowPassFilter accel_low_pass_filter;
    StateMachineInternalState internal_state_fc;
    ExpLowPassFilter landing_lp_filter;

    uint32_t last_velocity_calculation_time;
    float last_altitude;
    float accel_average[3];
    uint32_t up_axis_loop_counter;
};

#endif /* INC_STATE_CONTROLLER_H_ */
