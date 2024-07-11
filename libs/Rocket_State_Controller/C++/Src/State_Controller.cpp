/*
 State_Controller.cpp

 Created on: Feb 27, 2024
 Modified: July 9, 2024
 Author: Angus McLennan
 */

#include "Rocket_State_Controller/State_Controller.h"

uint8_t StateController::init(float starting_altitude, float starting_pressure, float starting_temperature) {
	system_state.flight_state = FlightState::IDLE_ON_PAD;

	system_state.starting_altitude = starting_altitude;
	system_state.starting_pressure = starting_pressure;
	system_state.starting_temperature = starting_temperature;

	std::fill(std::begin(accel_average), std::end(accel_average), 0.0f);

	// Initialise launch median filter
	launch_median_filter = MedianFilter(LAUNCH_ACCEL_FILTER_WIDTH, LAUNCH_ACCEL_FILTER_FREQ);

	// Initialise burnout median filter
	burnout_median_filter = MedianFilter(BURNOUT_ACCEL_FILTER_WIDTH, BURNOUT_ACCEL_FILTER_FREQ);

	// Initialise apogee vertical velocity median filter
	apogee_median_filter = MedianFilter(VERTICAL_VELOCITY_FILTER_WIDTH, VERTICAL_VELOCITY_FILTER_FREQ);

	// Initialise apogee altitude exponential low pass filter
	altitude_exp_lp_filter = ExpLowPassFilter(VERTICAL_VELOCITY_DETECT_FREQ, ALTITUDE_LP_FILTER_CUTOFF_FREQ, starting_altitude);
	last_altitude = starting_altitude;

	// Initialise apogee acceleration exponential low pass filter
	accel_low_pass_filter = ExpLowPassFilter(VERTICAL_VELOCITY_DETECT_FREQ, ACCEL_LP_FILTER_CUTOFF_FREQ, 0.0);

	// Initialise landing detection exponential low pass filter
	landing_lp_filter = ExpLowPassFilter(VERTICAL_VELOCITY_DETECT_FREQ, LANDING_VELOCITY_LP_CUTOFF_FREQ, -5.0);

	return 0;
}

bool StateController::detectLaunchAccel(float ax, float ay, float az, float rocket_angle, uint32_t timestamp_ms) {
	internal_state_fc.angle_from_vertical = rocket_angle;
	if (rocket_angle <= DEG_TO_RAD(PITCH_OVER_ANGLE_THRESHOLD) || M_PI - rocket_angle <= DEG_TO_RAD(PITCH_OVER_ANGLE_THRESHOLD)) {
		float acceleration_magnitude = std::sqrt(ax * ax + ay * ay + az * az);

		launch_median_filter.update(acceleration_magnitude, static_cast<float>(timestamp_ms) / 1000.0);

		if (launch_median_filter.isFilledUp()) {
			// Filter has been filled with valid values
			float filtered_acceleration = launch_median_filter.getMedianValue();
			internal_state_fc.filtered_launch_detect_accel = filtered_acceleration;

			if (filtered_acceleration >= LAUNCH_ACCEL_THRESHOLD) {
				system_state.flight_state = FlightState::LAUNCHED;
				system_state.launch_time = timestamp_ms;
				return true;
			}
		}
	}
	return false;
}

bool StateController::calculateUpAxis(float ax, float ay, float az) {
	if (up_axis_loop_counter < 20)
	    {
	        up_axis_loop_counter++;
	        accel_average[0] += ax;
	        accel_average[1] += ay;
	        accel_average[2] += az;
	        return false;
	    }
	    else
	    {
	        // Calculate averages
	        accel_average[0] /= up_axis_loop_counter;
	        accel_average[1] /= up_axis_loop_counter;
	        accel_average[2] /= up_axis_loop_counter;

	        // Determine the axis with largest absolute value
	        float max_acceleration = std::fabs(accel_average[0]);
	        int largest_index = 0;

	        for (int i = 1; i < 3; i++)
	        {
	            if (std::fabs(accel_average[i]) > max_acceleration)
	            {
	                max_acceleration = std::fabs(accel_average[i]);
	                largest_index = i;
	            }
	        }

	        // Assign up axis
	        if (largest_index == 0)
	        {
	            if (accel_average[largest_index] > 0)
	            {
	                system_state.up_axis = RocketUpAxis::X_AXIS_POSITIVE;
	            }
	            else
	            {
	                system_state.up_axis = RocketUpAxis::X_AXIS_NEGATIVE;
	            }
	        }
	        else if (largest_index == 1)
	        {
	            if (accel_average[largest_index] > 0)
	            {
	                system_state.up_axis = RocketUpAxis::Y_AXIS_POSITIVE;
	            }
	            else
	            {
	                system_state.up_axis = RocketUpAxis::Y_AXIS_NEGATIVE;
	            }
	        }
	        else
	        {
	            if (accel_average[largest_index] > 0)
	            {
	                system_state.up_axis = RocketUpAxis::Z_AXIS_POSITIVE;
	            }
	            else
	            {
	                system_state.up_axis = RocketUpAxis::Z_AXIS_NEGATIVE;
	            }
	        }

	        // Reset state
	        accel_average[0] = 0;
	        accel_average[1] = 0;
	        accel_average[2] = 0;
	        up_axis_loop_counter = 0;
	        return true;
	    }
	    return false;
}

bool StateController::detectBurnoutAccel(float ax, float ay, float az, float altitude, uint32_t timestamp_ms) {
	// Detect burnout with a timeout as back up
	    float time_since_launch = (float)(timestamp_ms - system_state.launch_time) / 1000.0;
	    if (time_since_launch >= MAX_MOTOR_BURN_TIME)
	    {
	        // Register burnout
	        system_state.flight_state = FlightState::BURNOUT;
	    }
	    float up_axis_reading = 0;
	    // Pass the accelerometer reading into a filter that corresponds to the up axis
	    switch (system_state.up_axis)
	    {
	    case RocketUpAxis::X_AXIS_POSITIVE:
	        up_axis_reading = ax;
	        break;
	    case RocketUpAxis::X_AXIS_NEGATIVE:
	        up_axis_reading = -ax;
	        break;
	    case RocketUpAxis::Y_AXIS_POSITIVE:
	        up_axis_reading = ay;
	        break;
	    case RocketUpAxis::Y_AXIS_NEGATIVE:
	        up_axis_reading = -ay;
	        break;
	    case RocketUpAxis::Z_AXIS_POSITIVE:
	        up_axis_reading = az;
	        break;
	    case RocketUpAxis::Z_AXIS_NEGATIVE:
	        up_axis_reading = -az;
	        break;
	    default:
	        up_axis_reading = ax;
	        break;
	    }
	    burnout_median_filter.update(up_axis_reading, static_cast<float>(timestamp_ms) / 1000.0);
	    if (burnout_median_filter.isFilledUp())
	    {
	        // Filter has been filled with valid values
	        float filtered_acceleration = burnout_median_filter.getMedianValue();
	        internal_state_fc.filtered_burnout_detect_x_axis_accel = filtered_acceleration;
	        if (filtered_acceleration <= BURNOUT_ACCEL_THRESHOLD)
	        {
	            // Register burnout
	            system_state.flight_state = FlightState::BURNOUT;
	            system_state.burnout_time = timestamp_ms;
	            system_state.burnout_altitude = altitude;
	            return true;
	        }
	    }
	    return false;
}

bool StateController::detectApogee(float ax, float ay, float az, float altitude, uint32_t timestamp_ms) {
	if ((float)(timestamp_ms - last_velocity_calculation_time) / 1000.0 >= (1.0 / VERTICAL_VELOCITY_DETECT_FREQ))
	    {
	        float dt = (timestamp_ms - last_velocity_calculation_time) / 1000.0;
	        float filtered_altitude = altitude_exp_lp_filter.update(altitude);
	        float vertical_velocity = (filtered_altitude - last_altitude) / dt;
	        apogee_median_filter.update(vertical_velocity, static_cast<float>(timestamp_ms) / 1000.0);
	        float ax2 = ax * ax;
	        float ay2 = ay * ay;
	        float az2 = az * az;
	        float acceleration_magnitude = sqrt(ax2 + ay2 + az2);
	        float filtered_accel = accel_low_pass_filter.update(acceleration_magnitude);
	        internal_state_fc.filtered_apogee_detect_accel = filtered_accel;
	        internal_state_fc.filtered_apogee_detect_altitude = filtered_altitude;
	        if (apogee_median_filter.isFilledUp())
	        {
	            float filtered_vertical_velocity =apogee_median_filter.getMedianValue();
	            // Log internal state of system
	            internal_state_fc.filtered_apogee_detect_vertical_velocity = filtered_vertical_velocity;

	            if (filtered_vertical_velocity <= APOGEE_DETECT_VELOCITY_THRESHOLD)
	            {
	                if (filtered_accel <= APOGEE_DETECT_ACCEL_THRESHOLD)
	                {
	                    // Register apogee
	                    system_state.flight_state = FlightState::APOGEE;
	                    system_state.drogue_deploy_time = timestamp_ms;
	                    system_state.drogue_deploy_altitude = altitude;
	                    return true;
	                }
	            }
	        }
	        last_velocity_calculation_time = timestamp_ms;
	        last_altitude = filtered_altitude;
	    }
	    return false;
}

bool StateController::detectMainDeployAltitude(float altitude, uint32_t timestamp_ms) {
	float altitude_agl = altitude - system_state.starting_altitude;
	internal_state_fc.unfiltered_main_detect_agl_altitude = altitude_agl;

	if (altitude_agl <= MAIN_DEPLOY_ALTITUDE) {
		system_state.flight_state = FlightState::MAIN_CHUTE_ALTITUDE;
		system_state.main_deploy_time = timestamp_ms;
		system_state.main_deploy_altitude = altitude;
		return true;
	}
	return false;
}

bool StateController::detectLanding(float altitude, uint32_t timestamp_ms) {
	if ((float)(timestamp_ms - last_velocity_calculation_time) / 1000.0 >= (1.0 / VERTICAL_VELOCITY_DETECT_FREQ))
	    {
	        float dt = (timestamp_ms - last_velocity_calculation_time) / 1000.0;
	        float vertical_velocity = (altitude - last_altitude) / dt;
	        float filtered_vertical_velocity = landing_lp_filter.update(vertical_velocity);
	        internal_state_fc.filtered_landing_detect_vertical_velocity = filtered_vertical_velocity;
	        if (fabs(filtered_vertical_velocity) <= LANDING_SPEED_THRESHOLD)
	        {
	            system_state.flight_state = FlightState::LANDED;
	            system_state.landing_time = timestamp_ms;
	            system_state.landing_altitude = altitude;
	            return true;
	        }
	        last_velocity_calculation_time = timestamp_ms;
	        last_altitude = altitude;
	    }
	    return false;
}
