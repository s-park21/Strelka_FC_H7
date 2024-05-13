#include "EKF_Full.h"

static double rt_powd_snf(double u0, double u1);

EKF_fs_Status_t EKF_fs_init(EKF_fs_t *ekf) {
	arm_mat_init_f32(&ekf->state_vec, STATE_VEC_DIMS, 1, ekf->state_vec_data);
	float qu_init[4] = { 0.7071, 0, 0.7071, 0 };
	memcpy(ekf->state_vec_data, qu_init, sizeof(qu_init));
	arm_mat_init_f32(&ekf->P, STATE_VEC_DIMS, STATE_VEC_DIMS, ekf->P_data);
	arm_mat_init_f32(&ekf->Q_accel, STATE_VEC_DIMS, STATE_VEC_DIMS, ekf->Q_accel_data);
	arm_mat_init_f32(&ekf->R_accel, 3, 3, ekf->R_accel_data);
	arm_mat_init_f32(&ekf->Q_gyro, STATE_VEC_DIMS, STATE_VEC_DIMS, ekf->Q_gyro_data);
	arm_mat_init_f32(&ekf->R_mag, 3, 3, ekf->R_mag_data);
	arm_mat_init_f32(&ekf->R_gps, 3, 3, ekf->R_gps_data);
	arm_mat_init_f32(&ekf->R_baro, 1, 1, ekf->R_baro_data);

	return EKF_FS_OK;
}

EKF_fs_Status_t EKF_fs_predict_accel(EKF_fs_t *ekf, float ax, float ay, float az, float dt) {
	// Temporary storage buffers
	float tmp_10x10_data[STATE_VEC_DIMS * STATE_VEC_DIMS];
	arm_matrix_instance_f32 tmp_10x10;
	arm_mat_init_f32(&tmp_10x10, STATE_VEC_DIMS, STATE_VEC_DIMS, tmp_10x10_data);

	// Calculate rotation matrix from current state
	float rot_mat_data[9];
	EKF_fs_EP2C(ekf->state_vec_data[0], -ekf->state_vec_data[1], -ekf->state_vec_data[2], -ekf->state_vec_data[3], rot_mat_data);
	arm_matrix_instance_f32 rot_mat;
	arm_mat_init_f32(&rot_mat, 3, 3, rot_mat_data);

	// Calculate acceleration vector in world frame (NED)
	float acc_vec_data[3] = { ax * GRAVITY_MPS, ay * GRAVITY_MPS, az * GRAVITY_MPS };
	arm_matrix_instance_f32 acc_vec;
	arm_mat_init_f32(&acc_vec, 3, 1, acc_vec_data);
	float acc_vec_world_data[3];
	arm_matrix_instance_f32 acc_vec_world;
	arm_mat_init_f32(&acc_vec_world, 3, 1, acc_vec_world_data);
	arm_status res = arm_mat_mult_f32(&rot_mat, &acc_vec, &acc_vec_world);
	if (res)
		return res;

	// Subtract gravity from acceleration vector in world frame
	float gravity_vec_data[3] = { 0, 0, -GRAVITY_MPS };
	arm_matrix_instance_f32 gravity_vec;
	arm_mat_init_f32(&gravity_vec, 3, 1, gravity_vec_data);
	res = arm_mat_sub_f32(&acc_vec_world, &gravity_vec, &acc_vec_world);
	if (res)
		return res;

	// Make prediction step by incrementing linear position and velocity using measured acceleration via Euler integration
	for (int i = 0; i < 3; i++) {
		// Predict position
		ekf->state_vec_data[i + 4] += dt * ekf->state_vec_data[i + 7] + 0.5 * dt * dt * acc_vec_world_data[i];
		// Predict velocity
		ekf->state_vec_data[i + 7] += dt * acc_vec_world_data[i];
	}

	// Generate Jacobian matrix of accelerometer prediction
	float qw = ekf->state_vec_data[0];
	float qx = ekf->state_vec_data[1];
	float qy = ekf->state_vec_data[2];
	float qz = ekf->state_vec_data[3];
	float accel_pred_jacob_data[STATE_VEC_DIMS * STATE_VEC_DIMS];
	arm_matrix_instance_f32 accel_pred_jacob;
	arm_mat_init_f32(&accel_pred_jacob, STATE_VEC_DIMS, STATE_VEC_DIMS, accel_pred_jacob_data);
	get_acc_jacob(qw, qx, qy, qz, ax, ay, az, dt, accel_pred_jacob_data);

	// Calculate jacobian transpose
	float accel_pred_jacob_transpose_data[STATE_VEC_DIMS * STATE_VEC_DIMS];
	arm_matrix_instance_f32 accel_pred_jacob_transpose;
	arm_mat_init_f32(&accel_pred_jacob_transpose, STATE_VEC_DIMS, STATE_VEC_DIMS, accel_pred_jacob_transpose_data);
	res = arm_mat_trans_f32(&accel_pred_jacob, &accel_pred_jacob_transpose);
	if (res)
		return res;

	// Multiply jacobian by P matrix
	res = arm_mat_mult_f32(&accel_pred_jacob, &ekf->P, &tmp_10x10);
	if (res)
		return res;

	// Multiply result by jacobian transpose and store temporarily
	res = arm_mat_mult_f32(&tmp_10x10, &accel_pred_jacob_transpose, &accel_pred_jacob);
	if (res)
		return res;

	// Add accelerometer Q matrix to result and store in P matrix
	res = arm_mat_add_f32(&accel_pred_jacob, &ekf->Q_accel, &ekf->P);

	return res;
}

EKF_fs_Status_t EKF_fs_predict_gyro(EKF_fs_t *ekf, float p, float q, float r, float dt) {
	// Temporary storage buffers
	float tmp_4x1_data[4];
	arm_matrix_instance_f32 tmp_4x1;
	arm_mat_init_f32(&tmp_4x1, 4, 1, tmp_4x1_data);

	float tmp_10x10_data[STATE_VEC_DIMS * STATE_VEC_DIMS];
	arm_matrix_instance_f32 tmp_10x10;
	arm_mat_init_f32(&tmp_10x10, STATE_VEC_DIMS, STATE_VEC_DIMS, tmp_10x10_data);

	// Calculate B matrix
	float B_data[12];
	arm_matrix_instance_f32 B;
	arm_mat_init_f32(&B, 4, 3, B_data);
	BmatEP(ekf->state_vec_data[0], ekf->state_vec_data[1], ekf->state_vec_data[2], ekf->state_vec_data[3], B_data);

	// Create angular velocity vector
	float w_data[] = { p, q, r };
	arm_matrix_instance_f32 w;
	arm_mat_init_f32(&w, 3, 1, w_data);

	// Multiply B matrix by angular velocity vector
	arm_status res = arm_mat_mult_f32(&B, &w, &tmp_4x1);
	if (res)
		return res;

	// Increment quaternion estimate
	for (int i = 0; i < 4; i++) {
		ekf->state_vec_data[i] += 0.5 * tmp_4x1_data[i] * dt;
	}

	// Normalise new estimate of quaternions
	EKF_fs_normalise_vector(ekf->state_vec_data, 4);

	// Generate Jacobian matrix of gyroscope prediction
	float qw = ekf->state_vec_data[0];
	float qx = ekf->state_vec_data[1];
	float qy = ekf->state_vec_data[2];
	float qz = ekf->state_vec_data[3];
	float gyro_pred_jacob_data[STATE_VEC_DIMS * STATE_VEC_DIMS];
	arm_matrix_instance_f32 gyro_pred_jacob;
	arm_mat_init_f32(&gyro_pred_jacob, STATE_VEC_DIMS, STATE_VEC_DIMS, gyro_pred_jacob_data);
	get_gyro_jacob(p, q, r, qw, qx, qy, qz, dt, gyro_pred_jacob_data);

	// Calculate jacobian transpose
	float gyro_pred_jacob_transpose_data[STATE_VEC_DIMS * STATE_VEC_DIMS];
	arm_matrix_instance_f32 gyro_pred_jacob_transpose;
	arm_mat_init_f32(&gyro_pred_jacob_transpose, STATE_VEC_DIMS, STATE_VEC_DIMS, gyro_pred_jacob_transpose_data);
	res = arm_mat_trans_f32(&gyro_pred_jacob, &gyro_pred_jacob_transpose);
	if (res)
		return res;

	// Multiply jacobian by P matrix
	res = arm_mat_mult_f32(&gyro_pred_jacob, &ekf->P, &tmp_10x10);
	if (res)
		return res;

	// Multiply result by jacobian transpose and store temporarily
	res = arm_mat_mult_f32(&tmp_10x10, &gyro_pred_jacob_transpose, &gyro_pred_jacob);
	if (res)
		return res;

	// Add accelerometer Q matrix to result and store in P matrix
	res = arm_mat_add_f32(&gyro_pred_jacob, &ekf->Q_gyro, &ekf->P);

	return res;
}

EKF_fs_Status_t EKF_fs_update_accel(EKF_fs_t *ekf, float ax, float ay, float az) {
	// Temporary storage buffers
	float tmp_10x1_data[10];
	arm_matrix_instance_f32 tmp_10x1;
	arm_mat_init_f32(&tmp_10x1, STATE_VEC_DIMS, 1, tmp_10x1_data);

	// Calculate rotation matrix from current state
	float rot_mat_data[9];
	EKF_fs_EP2C(ekf->state_vec_data[0], ekf->state_vec_data[1], ekf->state_vec_data[2], ekf->state_vec_data[3], rot_mat_data);

	// Calculate predicted acceleration - the value that the accelerometer should read given the current state
	float a_pred[3] = { rot_mat_data[2], rot_mat_data[5], rot_mat_data[8] };

	// Calculate jacobian matrix
	float qw = ekf->state_vec_data[0];
	float qx = ekf->state_vec_data[1];
	float qy = ekf->state_vec_data[2];
	float qz = ekf->state_vec_data[3];

	float acc_jacob_data[30] = { 0 };
	acc_jacob_data[0] = 2 * qy;
	acc_jacob_data[1] = -2 * qz;
	acc_jacob_data[2] = 2 * qw;
	acc_jacob_data[3] = -2 * qx;
	acc_jacob_data[10] = -2 * qx;
	acc_jacob_data[11] = -2 * qw;
	acc_jacob_data[12] = -2 * qz;
	acc_jacob_data[13] = -2 * qy;
	acc_jacob_data[20] = -2 * qw;
	acc_jacob_data[21] = 2 * qx;
	acc_jacob_data[22] = 2 * qy;
	acc_jacob_data[23] = -2 * qz;
	arm_matrix_instance_f32 acc_jacob;
	arm_mat_init_f32(&acc_jacob, 3, STATE_VEC_DIMS, acc_jacob_data);

	// Calculate K matrix
	arm_matrix_instance_f32 K;
	float K_data[STATE_VEC_DIMS * 3] = { 0 };
	arm_mat_init_f32(&K, STATE_VEC_DIMS, 3, K_data);

	EKF_fs_Status_t res = EKF_fs_calculate_K_matrix(ekf, &acc_jacob, &K, &ekf->R_accel);
	if (res)
		return res;
	// Zeroing out these because they get changed for some reason
//	K_data[24] = 0;
//	K_data[25] = 0;
//	K_data[26] = 0;

// Calculate normalised accelerometer vector
	float acc_vec_data[3] = { ax, ay, az };
	arm_matrix_instance_f32 acc_vec;
	arm_mat_init_f32(&acc_vec, 3, 1, acc_vec_data);
	EKF_fs_normalise_vector(acc_vec_data, 3);

	// Calculate difference between measured acceleration (normalised) and predicted acceleration
	for (int i = 0; i < 3; i++) {
		acc_vec_data[i] -= a_pred[i];
	}

	// Multiply Kalman gain by difference and store in tmp vector
	arm_status resl = arm_mat_mult_f32(&K, &acc_vec, &tmp_10x1);
	if (resl)
		return resl;

	// Update current state vector by adding result to it
	resl = arm_mat_add_f32(&ekf->state_vec, &tmp_10x1, &ekf->state_vec);
	if (resl)
		return resl;

	return EKF_fs_update_P_matrix(ekf, &acc_jacob, &K);
}

EKF_fs_Status_t EKF_fs_update_baro(EKF_fs_t *ekf, float current_pressure_Pa, float initial_pressure_Pa, float initial_temperature_K, float initial_altitude_m) {
	// Temporary storage buffers
	float tmp_10x1_data[10];
	arm_matrix_instance_f32 tmp_10x1;
	arm_mat_init_f32(&tmp_10x1, STATE_VEC_DIMS, 1, tmp_10x1_data);

	// Calculate T0
	float T0 = initial_temperature_K - LAPSE_RATE * initial_altitude_m;

	// Calculate predicted height given current state
	float delta_d = ekf->state_vec_data[6];
	float h_pred = pow(1 - delta_d * LAPSE_RATE / T0, -GRAVITY_MPS / (R_gas * LAPSE_RATE)) * initial_pressure_Pa;

	// Calculate barometer jacobian matrix
	float baro_jacob_data[STATE_VEC_DIMS] = { 0 };
	baro_jacob_data[6] = (initial_pressure_Pa * GRAVITY_MPS) / (R_gas * pow((1 - (LAPSE_RATE * delta_d) / (initial_temperature_K - initial_altitude_m * LAPSE_RATE)), (GRAVITY_MPS / (LAPSE_RATE * R_gas) + 1)) * (initial_temperature_K - initial_altitude_m * LAPSE_RATE));
	arm_matrix_instance_f32 baro_jacob;
	arm_mat_init_f32(&baro_jacob, 1, STATE_VEC_DIMS, baro_jacob_data);

	float K_data[STATE_VEC_DIMS];
	arm_matrix_instance_f32 K;
	arm_mat_init_f32(&K, STATE_VEC_DIMS, 1, K_data);
	EKF_fs_Status_t res = EKF_fs_calculate_K_matrix(ekf, &baro_jacob, &K, &ekf->R_baro);
	if (res)
		return res;

	// Calculate difference between measured altitude and predicted altitude
	float diff = current_pressure_Pa - h_pred;

	// Multiply K by the difference
	arm_status resl = arm_mat_scale_f32(&K, diff, &tmp_10x1);
	if (resl)
		return resl;

	// Update current state vector by adding result to it
	resl = arm_mat_add_f32(&ekf->state_vec, &tmp_10x1, &ekf->state_vec);
	if (resl)
		return resl;

	return EKF_fs_update_P_matrix(ekf, &baro_jacob, &K);
}

EKF_fs_Status_t EKF_fs_update_gps(EKF_fs_t *ekf, float current_latitude, float current_longitude, float current_altitude, float initial_latitude, float initial_longitude, float initial_altitude) {
	// Temporary storage buffers
	float tmp_10x1_data[10];
	arm_matrix_instance_f32 tmp_10x1;
	arm_mat_init_f32(&tmp_10x1, STATE_VEC_DIMS, 1, tmp_10x1_data);

	// Calculate predicted GPS position (LLA) given current state
	float gps_lat_prediction = ekf->state_vec_data[4] / (RADIUS_EARTH * cos(initial_latitude * M_PI / 180)) + initial_latitude;
	float gps_lon_prediction = ekf->state_vec_data[5] / (RADIUS_EARTH) + initial_longitude;
	float gps_pred[] = { gps_lat_prediction, gps_lon_prediction, initial_altitude - ekf->state_vec_data[6] };

	// Calculate GPS jacobian
	float gps_jacob_data[3 * 10] = { 0 };
	gps_jacob_data[4] = 1 / RADIUS_EARTH * cos((M_PI * initial_latitude) / 180);
	gps_jacob_data[15] = 1 / RADIUS_EARTH;
	gps_jacob_data[26] = -1;
	arm_matrix_instance_f32 gps_jacob;
	arm_mat_init_f32(&gps_jacob, 3, STATE_VEC_DIMS, gps_jacob_data);

	float K_data[STATE_VEC_DIMS * 3];
	arm_matrix_instance_f32 K;
	arm_mat_init_f32(&K, STATE_VEC_DIMS, 3, K_data);

	// Calculate K matrix
	EKF_fs_Status_t res = EKF_fs_calculate_K_matrix(ekf, &gps_jacob, &K, &ekf->R_gps);
	if (res)
		return res;

	// Calculate difference between measured GPS coordinates and predicted GPS coordinates
	gps_pred[0] -= current_latitude;
	gps_pred[1] -= current_longitude;
	gps_pred[2] -= current_altitude;

	arm_matrix_instance_f32 gps_diff;
	arm_mat_init_f32(&gps_diff, 3, 1, gps_pred);

	// Multiply difference by K matrix and store in temp matrix
	arm_status resl = arm_mat_mult_f32(&K, &gps_diff, &tmp_10x1);
	if (resl)
		return resl;

	// Update current state vector by adding result to it
	resl = arm_mat_add_f32(&ekf->state_vec, &tmp_10x1, &ekf->state_vec);
	if (resl)
		return resl;

	return EKF_fs_update_P_matrix(ekf, &gps_jacob, &K);
}

EKF_fs_Status_t EKF_fs_update_mag(EKF_fs_t *ekf, float mx, float my, float mz) {
	// Temporary storage buffers
	float tmp_10x1_data[10];
	arm_matrix_instance_f32 tmp_10x1;
	arm_mat_init_f32(&tmp_10x1, STATE_VEC_DIMS, 1, tmp_10x1_data);

	float earth_mag_vec_data[] = { 0.355184571848873, 0.074055414158661, -0.931858205712823 }; // The Earth magnetic field vector in Melbourne Australia May 2024
	arm_matrix_instance_f32 earth_mag_vec;
	arm_mat_init_f32(&earth_mag_vec, 3, 1, earth_mag_vec_data);

	// Calculate rotation matrix from current state
	float rot_mat_data[9];
	EKF_fs_EP2C(ekf->state_vec_data[0], ekf->state_vec_data[1], ekf->state_vec_data[2], ekf->state_vec_data[3], rot_mat_data);
	arm_matrix_instance_f32 rot_mat;
	arm_mat_init_f32(&rot_mat, 3, 3, rot_mat_data);

	// Define magnetic field vector in body coordinates
	float earth_mag_vec_body_data[3];
	arm_matrix_instance_f32 earth_mag_vec_body;
	arm_mat_init_f32(&earth_mag_vec_body, 3, 1, earth_mag_vec_body_data);

	// Rotate magnetic field vector prediction into body frame
	arm_status res = arm_mat_mult_f32(&rot_mat, &earth_mag_vec, &earth_mag_vec_body);
	if (res)
		return res;

	// Calculate magnetometer jacobian
	float mag_jacob_data[3 * STATE_VEC_DIMS];
	arm_matrix_instance_f32 mag_jacob;
	arm_mat_init_f32(&mag_jacob, 3, STATE_VEC_DIMS, mag_jacob_data);

	get_mag_jacob(ekf->state_vec_data[0], ekf->state_vec_data[1], ekf->state_vec_data[2], ekf->state_vec_data[3], mag_jacob_data);

	float K_data[STATE_VEC_DIMS * 3];
	arm_matrix_instance_f32 K;
	arm_mat_init_f32(&K, STATE_VEC_DIMS, 3, K_data);

	// Calculate K matrix
	EKF_fs_Status_t resl = EKF_fs_calculate_K_matrix(ekf, &mag_jacob, &K, &ekf->R_mag);
	if (resl)
		return resl;

	// Calculate normalised accelerometer vector
	float mag_vec_data[3] = { mx, my, mz };
	arm_matrix_instance_f32 mag_vec;
	arm_mat_init_f32(&mag_vec, 3, 1, mag_vec_data);
	EKF_fs_normalise_vector(mag_vec_data, 3);

	// Calculate difference between measured acceleration (normalised) and predicted acceleration
	for (int i = 0; i < 3; i++) {
		mag_vec_data[i] -= earth_mag_vec_body_data[i];
	}

	// Multiply Kalman gain by difference and store in tmp vector
	res = arm_mat_mult_f32(&K, &mag_vec, &tmp_10x1);
	if (res)
		return res;

	// Update current state vector by adding result to it
	res = arm_mat_add_f32(&ekf->state_vec, &tmp_10x1, &ekf->state_vec);
	if (res)
		return res;

	return EKF_fs_update_P_matrix(ekf, &mag_jacob, &K);
}

void get_acc_jacob(float qw, float qx, float qy, float qz, float a1, float a2, float a3, float dt, float jacob_a_pred[100]) {
	static const signed char iv[10] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	static const signed char iv1[10] = { 0, 1, 0, 0, 0, 0, 0, 0, 0, 0 };
	static const signed char iv2[10] = { 0, 0, 1, 0, 0, 0, 0, 0, 0, 0 };
	static const signed char iv3[10] = { 0, 0, 0, 1, 0, 0, 0, 0, 0, 0 };
	int i;
	for (i = 0; i < 10; i++) {
		jacob_a_pred[10 * i] = iv[i];
		jacob_a_pred[10 * i + 1] = iv1[i];
		jacob_a_pred[10 * i + 2] = iv2[i];
		jacob_a_pred[10 * i + 3] = iv3[i];
	}
	float b_jacob_a_pred_tmp;
	float b_jacob_a_pred_tmp_tmp;
	float b_jacob_a_pred_tmp_tmp_tmp;
	float c_jacob_a_pred_tmp;
	float c_jacob_a_pred_tmp_tmp;
	float c_jacob_a_pred_tmp_tmp_tmp;
	float d_jacob_a_pred_tmp;
	float d_jacob_a_pred_tmp_tmp;
	float e_jacob_a_pred_tmp;
	float e_jacob_a_pred_tmp_tmp;
	float f_jacob_a_pred_tmp_tmp;
	float g_jacob_a_pred_tmp_tmp;
	float h_jacob_a_pred_tmp_tmp;
	float i_jacob_a_pred_tmp_tmp;
	float j_jacob_a_pred_tmp_tmp;
	float jacob_a_pred_tmp;
	float jacob_a_pred_tmp_tmp;
	float jacob_a_pred_tmp_tmp_tmp;
	jacob_a_pred_tmp = dt * dt;
	jacob_a_pred_tmp_tmp_tmp = 981.0 * a1 * qw;
	b_jacob_a_pred_tmp_tmp_tmp = 981.0 * a3 * qy;
	c_jacob_a_pred_tmp_tmp_tmp = 981.0 * a2 * qz;
	jacob_a_pred_tmp_tmp = jacob_a_pred_tmp * ((jacob_a_pred_tmp_tmp_tmp / 100.0 + b_jacob_a_pred_tmp_tmp_tmp / 100.0) - c_jacob_a_pred_tmp_tmp_tmp / 100.0);
	b_jacob_a_pred_tmp = jacob_a_pred_tmp_tmp / 2.0;
	jacob_a_pred[4] = b_jacob_a_pred_tmp;
	b_jacob_a_pred_tmp_tmp = 981.0 * a1 * qx;
	c_jacob_a_pred_tmp_tmp = 981.0 * a2 * qy;
	d_jacob_a_pred_tmp_tmp = 981.0 * a3 * qz;
	c_jacob_a_pred_tmp = jacob_a_pred_tmp * ((b_jacob_a_pred_tmp_tmp / 100.0 + c_jacob_a_pred_tmp_tmp / 100.0) + d_jacob_a_pred_tmp_tmp / 100.0) / 2.0;
	jacob_a_pred[14] = c_jacob_a_pred_tmp;
	e_jacob_a_pred_tmp_tmp = 981.0 * a3 * qw;
	f_jacob_a_pred_tmp_tmp = 981.0 * a2 * qx;
	g_jacob_a_pred_tmp_tmp = 981.0 * a1 * qy;
	d_jacob_a_pred_tmp = jacob_a_pred_tmp * ((e_jacob_a_pred_tmp_tmp / 100.0 + f_jacob_a_pred_tmp_tmp / 100.0) - g_jacob_a_pred_tmp_tmp / 100.0);
	e_jacob_a_pred_tmp = d_jacob_a_pred_tmp / 2.0;
	jacob_a_pred[24] = e_jacob_a_pred_tmp;
	h_jacob_a_pred_tmp_tmp = 981.0 * a2 * qw;
	i_jacob_a_pred_tmp_tmp = 981.0 * a3 * qx;
	j_jacob_a_pred_tmp_tmp = 981.0 * a1 * qz;
	jacob_a_pred_tmp *= (h_jacob_a_pred_tmp_tmp / 100.0 - i_jacob_a_pred_tmp_tmp / 100.0) + j_jacob_a_pred_tmp_tmp / 100.0;
	jacob_a_pred[34] = -jacob_a_pred_tmp / 2.0;
	jacob_a_pred[44] = 1.0;
	jacob_a_pred[54] = 0.0;
	jacob_a_pred[64] = 0.0;
	jacob_a_pred[74] = dt;
	jacob_a_pred[84] = 0.0;
	jacob_a_pred[94] = 0.0;
	jacob_a_pred_tmp /= 2.0;
	jacob_a_pred[5] = jacob_a_pred_tmp;
	jacob_a_pred[15] = -d_jacob_a_pred_tmp / 2.0;
	jacob_a_pred[25] = c_jacob_a_pred_tmp;
	jacob_a_pred[35] = b_jacob_a_pred_tmp;
	jacob_a_pred[45] = 0.0;
	jacob_a_pred[55] = 1.0;
	jacob_a_pred[65] = 0.0;
	jacob_a_pred[75] = 0.0;
	jacob_a_pred[85] = dt;
	jacob_a_pred[95] = 0.0;
	jacob_a_pred[6] = e_jacob_a_pred_tmp;
	jacob_a_pred[16] = jacob_a_pred_tmp;
	jacob_a_pred[26] = -jacob_a_pred_tmp_tmp / 2.0;
	jacob_a_pred[36] = c_jacob_a_pred_tmp;
	jacob_a_pred[46] = 0.0;
	jacob_a_pred[56] = 0.0;
	jacob_a_pred[66] = 1.0;
	jacob_a_pred[76] = 0.0;
	jacob_a_pred[86] = 0.0;
	jacob_a_pred[96] = dt;
	jacob_a_pred_tmp_tmp = (jacob_a_pred_tmp_tmp_tmp / 50.0 + b_jacob_a_pred_tmp_tmp_tmp / 50.0) - c_jacob_a_pred_tmp_tmp_tmp / 50.0;
	jacob_a_pred_tmp = dt * jacob_a_pred_tmp_tmp;
	jacob_a_pred[7] = jacob_a_pred_tmp;
	b_jacob_a_pred_tmp = dt * ((b_jacob_a_pred_tmp_tmp / 50.0 + c_jacob_a_pred_tmp_tmp / 50.0) + d_jacob_a_pred_tmp_tmp / 50.0);
	jacob_a_pred[17] = b_jacob_a_pred_tmp;
	c_jacob_a_pred_tmp = (e_jacob_a_pred_tmp_tmp / 50.0 + f_jacob_a_pred_tmp_tmp / 50.0) - g_jacob_a_pred_tmp_tmp / 50.0;
	d_jacob_a_pred_tmp = dt * c_jacob_a_pred_tmp;
	jacob_a_pred[27] = d_jacob_a_pred_tmp;
	e_jacob_a_pred_tmp = (h_jacob_a_pred_tmp_tmp / 50.0 - i_jacob_a_pred_tmp_tmp / 50.0) + j_jacob_a_pred_tmp_tmp / 50.0;
	jacob_a_pred[37] = -dt * e_jacob_a_pred_tmp;
	jacob_a_pred[47] = 0.0;
	jacob_a_pred[57] = 0.0;
	jacob_a_pred[67] = 0.0;
	jacob_a_pred[77] = 1.0;
	jacob_a_pred[87] = 0.0;
	jacob_a_pred[97] = 0.0;
	e_jacob_a_pred_tmp *= dt;
	jacob_a_pred[8] = e_jacob_a_pred_tmp;
	jacob_a_pred[18] = -dt * c_jacob_a_pred_tmp;
	jacob_a_pred[28] = b_jacob_a_pred_tmp;
	jacob_a_pred[38] = jacob_a_pred_tmp;
	jacob_a_pred[48] = 0.0;
	jacob_a_pred[58] = 0.0;
	jacob_a_pred[68] = 0.0;
	jacob_a_pred[78] = 0.0;
	jacob_a_pred[88] = 1.0;
	jacob_a_pred[98] = 0.0;
	jacob_a_pred[9] = d_jacob_a_pred_tmp;
	jacob_a_pred[19] = e_jacob_a_pred_tmp;
	jacob_a_pred[29] = -dt * jacob_a_pred_tmp_tmp;
	jacob_a_pred[39] = b_jacob_a_pred_tmp;
	jacob_a_pred[49] = 0.0;
	jacob_a_pred[59] = 0.0;
	jacob_a_pred[69] = 0.0;
	jacob_a_pred[79] = 0.0;
	jacob_a_pred[89] = 0.0;
	jacob_a_pred[99] = 1.0;

	// This is an autogenerate function by matlab. Matlab stupidly flattens a matrix column by column while the STM32 inteprets a flattened matrix as rows by rows
	// To fix this, we have to rearrange the matrix above to be in row by row form.
	float rearranged_matrix[100];
	rearrange_matrix_row_form(jacob_a_pred, rearranged_matrix, 10, 10);
	memcpy(jacob_a_pred, rearranged_matrix, sizeof(rearranged_matrix));
}

void get_gyro_jacob(float w1, float w2, float w3, float qw, float qx, float qy, float qz, float dt, float jacob_w_pred[100]) {
	static const signed char iv[10] = { 0, 0, 0, 0, 1, 0, 0, 0, 0, 0 };
	static const signed char iv1[10] = { 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 };
	static const signed char iv2[10] = { 0, 0, 0, 0, 0, 0, 1, 0, 0, 0 };
	static const signed char iv3[10] = { 0, 0, 0, 0, 0, 0, 0, 1, 0, 0 };
	static const signed char iv4[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 1, 0 };
	static const signed char iv5[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };
	float a_tmp;
	float a_tmp_tmp;
	float b_a_tmp;
	float b_a_tmp_tmp;
	float b_jacob_w_pred_tmp;
	float c_a_tmp;
	float c_a_tmp_tmp;
	float c_jacob_w_pred_tmp;
	float d_a_tmp;
	float d_a_tmp_tmp;
	float d_jacob_w_pred_tmp;
	float e_jacob_w_pred_tmp;
	float f_jacob_w_pred_tmp;
	float g_jacob_w_pred_tmp;
	float h_jacob_w_pred_tmp;
	float i_jacob_w_pred_tmp;
	float j_jacob_w_pred_tmp;
	float jacob_w_pred_tmp;
	int i;
	a_tmp_tmp = (qw * w1 / 2.0 + qy * w3 / 2.0) - qz * w2 / 2.0;
	a_tmp = qx + dt * a_tmp_tmp;
	b_a_tmp_tmp = (qw * w2 / 2.0 - qx * w3 / 2.0) + qz * w1 / 2.0;
	b_a_tmp = qy + dt * b_a_tmp_tmp;
	c_a_tmp_tmp = (qw * w3 / 2.0 + qx * w2 / 2.0) - qy * w1 / 2.0;
	c_a_tmp = qz + dt * c_a_tmp_tmp;
	d_a_tmp_tmp = (qx * w1 / 2.0 + qy * w2 / 2.0) + qz * w3 / 2.0;
	d_a_tmp = qw - dt * d_a_tmp_tmp;
	jacob_w_pred_tmp = ((a_tmp * a_tmp + b_a_tmp * b_a_tmp) + c_a_tmp * c_a_tmp) + d_a_tmp * d_a_tmp;
	b_jacob_w_pred_tmp = dt * w1;
	c_jacob_w_pred_tmp = sqrt(jacob_w_pred_tmp);
	d_jacob_w_pred_tmp = dt * w2;
	e_jacob_w_pred_tmp = dt * w3;
	jacob_w_pred_tmp = 2.0 * rt_powd_snf(jacob_w_pred_tmp, 1.5);
	f_jacob_w_pred_tmp = (((2.0 * qw - 2.0 * dt * d_a_tmp_tmp) + b_jacob_w_pred_tmp * a_tmp) + d_jacob_w_pred_tmp * b_a_tmp) + e_jacob_w_pred_tmp * c_a_tmp;
	g_jacob_w_pred_tmp = 1.0 / c_jacob_w_pred_tmp;
	jacob_w_pred[0] = g_jacob_w_pred_tmp - d_a_tmp * f_jacob_w_pred_tmp / jacob_w_pred_tmp;
	c_jacob_w_pred_tmp *= 2.0;
	h_jacob_w_pred_tmp = (((2.0 * qx + 2.0 * dt * a_tmp_tmp) - b_jacob_w_pred_tmp * d_a_tmp) + d_jacob_w_pred_tmp * c_a_tmp) - e_jacob_w_pred_tmp * b_a_tmp;
	i_jacob_w_pred_tmp = -b_jacob_w_pred_tmp / c_jacob_w_pred_tmp;
	jacob_w_pred[10] = i_jacob_w_pred_tmp - d_a_tmp * h_jacob_w_pred_tmp / jacob_w_pred_tmp;
	b_a_tmp_tmp = (((2.0 * qy + 2.0 * dt * b_a_tmp_tmp) - b_jacob_w_pred_tmp * c_a_tmp) - d_jacob_w_pred_tmp * d_a_tmp) + e_jacob_w_pred_tmp * a_tmp;
	j_jacob_w_pred_tmp = -d_jacob_w_pred_tmp / c_jacob_w_pred_tmp;
	jacob_w_pred[20] = j_jacob_w_pred_tmp - d_a_tmp * b_a_tmp_tmp / jacob_w_pred_tmp;
	d_a_tmp_tmp = (((2.0 * qz + 2.0 * dt * c_a_tmp_tmp) + b_jacob_w_pred_tmp * b_a_tmp) - d_jacob_w_pred_tmp * a_tmp) - e_jacob_w_pred_tmp * d_a_tmp;
	a_tmp_tmp = -e_jacob_w_pred_tmp / c_jacob_w_pred_tmp;
	jacob_w_pred[30] = a_tmp_tmp - d_a_tmp * d_a_tmp_tmp / jacob_w_pred_tmp;
	jacob_w_pred[40] = 0.0;
	jacob_w_pred[50] = 0.0;
	jacob_w_pred[60] = 0.0;
	jacob_w_pred[70] = 0.0;
	jacob_w_pred[80] = 0.0;
	jacob_w_pred[90] = 0.0;
	b_jacob_w_pred_tmp /= c_jacob_w_pred_tmp;
	jacob_w_pred[1] = b_jacob_w_pred_tmp - a_tmp * f_jacob_w_pred_tmp / jacob_w_pred_tmp;
	jacob_w_pred[11] = g_jacob_w_pred_tmp - a_tmp * h_jacob_w_pred_tmp / jacob_w_pred_tmp;
	e_jacob_w_pred_tmp /= c_jacob_w_pred_tmp;
	jacob_w_pred[21] = e_jacob_w_pred_tmp - a_tmp * b_a_tmp_tmp / jacob_w_pred_tmp;
	jacob_w_pred[31] = j_jacob_w_pred_tmp - a_tmp * d_a_tmp_tmp / jacob_w_pred_tmp;
	jacob_w_pred[41] = 0.0;
	jacob_w_pred[51] = 0.0;
	jacob_w_pred[61] = 0.0;
	jacob_w_pred[71] = 0.0;
	jacob_w_pred[81] = 0.0;
	jacob_w_pred[91] = 0.0;
	c_jacob_w_pred_tmp = d_jacob_w_pred_tmp / c_jacob_w_pred_tmp;
	jacob_w_pred[2] = c_jacob_w_pred_tmp - b_a_tmp * f_jacob_w_pred_tmp / jacob_w_pred_tmp;
	jacob_w_pred[12] = a_tmp_tmp - b_a_tmp * h_jacob_w_pred_tmp / jacob_w_pred_tmp;
	jacob_w_pred[22] = g_jacob_w_pred_tmp - b_a_tmp * b_a_tmp_tmp / jacob_w_pred_tmp;
	jacob_w_pred[32] = b_jacob_w_pred_tmp - b_a_tmp * d_a_tmp_tmp / jacob_w_pred_tmp;
	jacob_w_pred[42] = 0.0;
	jacob_w_pred[52] = 0.0;
	jacob_w_pred[62] = 0.0;
	jacob_w_pred[72] = 0.0;
	jacob_w_pred[82] = 0.0;
	jacob_w_pred[92] = 0.0;
	jacob_w_pred[3] = e_jacob_w_pred_tmp - c_a_tmp * f_jacob_w_pred_tmp / jacob_w_pred_tmp;
	jacob_w_pred[13] = c_jacob_w_pred_tmp - c_a_tmp * h_jacob_w_pred_tmp / jacob_w_pred_tmp;
	jacob_w_pred[23] = i_jacob_w_pred_tmp - c_a_tmp * b_a_tmp_tmp / jacob_w_pred_tmp;
	jacob_w_pred[33] = g_jacob_w_pred_tmp - c_a_tmp * d_a_tmp_tmp / jacob_w_pred_tmp;
	jacob_w_pred[43] = 0.0;
	jacob_w_pred[53] = 0.0;
	jacob_w_pred[63] = 0.0;
	jacob_w_pred[73] = 0.0;
	jacob_w_pred[83] = 0.0;
	jacob_w_pred[93] = 0.0;
	for (i = 0; i < 10; i++) {
		jacob_w_pred[10 * i + 4] = iv[i];
		jacob_w_pred[10 * i + 5] = iv1[i];
		jacob_w_pred[10 * i + 6] = iv2[i];
		jacob_w_pred[10 * i + 7] = iv3[i];
		jacob_w_pred[10 * i + 8] = iv4[i];
		jacob_w_pred[10 * i + 9] = iv5[i];
	}

	// TODO: Do we need to flip this to row-row form????
//	float rearranged_matrix[100];
//	rearrange_matrix_row_form(jacob_w_pred, rearranged_matrix, 10, 10);
//	memcpy(jacob_w_pred, rearranged_matrix, sizeof(rearranged_matrix));
}

void get_mag_jacob(float qw, float qx, float qy, float qz, float jacob_mag[30]) {
	float b_jacob_mag_tmp;
	float b_jacob_mag_tmp_tmp;
	float c_jacob_mag_tmp;
	float c_jacob_mag_tmp_tmp;
	float d_jacob_mag_tmp;
	float e_jacob_mag_tmp;
	float f_jacob_mag_tmp;
	float g_jacob_mag_tmp;
	float h_jacob_mag_tmp;
	float jacob_mag_tmp;
	float jacob_mag_tmp_tmp;
	jacob_mag_tmp = 5.90515027096472E+14 * qy / 3.16848112446871E+14;
	b_jacob_mag_tmp = 4.880578184069775E+15 * qz / 3.2952203694474584E+16;
	c_jacob_mag_tmp = (5.852057180349399E+15 * qw / 8.238050923618646E+15 + jacob_mag_tmp) + b_jacob_mag_tmp;
	jacob_mag[0] = c_jacob_mag_tmp;
	d_jacob_mag_tmp = (5.852057180349399E+15 * qx / 8.238050923618646E+15 + 4.880578184069775E+15 * qy / 3.2952203694474584E+16) - 5.90515027096472E+14 * qz / 3.16848112446871E+14;
	jacob_mag[3] = d_jacob_mag_tmp;
	e_jacob_mag_tmp = 5.852057180349399E+15 * qy / 8.238050923618646E+15;
	f_jacob_mag_tmp = 4.880578184069775E+15 * qx / 3.2952203694474584E+16;
	g_jacob_mag_tmp = 5.90515027096472E+14 * qw / 3.16848112446871E+14;
	jacob_mag[6] = (g_jacob_mag_tmp + f_jacob_mag_tmp) - e_jacob_mag_tmp;
	jacob_mag_tmp_tmp = 5.90515027096472E+14 * qx / 3.16848112446871E+14;
	b_jacob_mag_tmp_tmp = 4.880578184069775E+15 * qw / 3.2952203694474584E+16;
	c_jacob_mag_tmp_tmp = 5.852057180349399E+15 * qz / 8.238050923618646E+15;
	h_jacob_mag_tmp = (b_jacob_mag_tmp_tmp - jacob_mag_tmp_tmp) - c_jacob_mag_tmp_tmp;
	jacob_mag[9] = h_jacob_mag_tmp;
	jacob_mag[12] = 0.0;
	jacob_mag[15] = 0.0;
	jacob_mag[18] = 0.0;
	jacob_mag[21] = 0.0;
	jacob_mag[24] = 0.0;
	jacob_mag[27] = 0.0;
	jacob_mag[1] = h_jacob_mag_tmp;
	e_jacob_mag_tmp = (e_jacob_mag_tmp - f_jacob_mag_tmp) - g_jacob_mag_tmp;
	jacob_mag[4] = e_jacob_mag_tmp;
	jacob_mag[7] = d_jacob_mag_tmp;
	jacob_mag[10] = (-(5.852057180349399E+15 * qw) / 8.238050923618646E+15 - jacob_mag_tmp) - b_jacob_mag_tmp;
	jacob_mag[13] = 0.0;
	jacob_mag[16] = 0.0;
	jacob_mag[19] = 0.0;
	jacob_mag[22] = 0.0;
	jacob_mag[25] = 0.0;
	jacob_mag[28] = 0.0;
	jacob_mag[2] = e_jacob_mag_tmp;
	jacob_mag[5] = (jacob_mag_tmp_tmp - b_jacob_mag_tmp_tmp) + c_jacob_mag_tmp_tmp;
	jacob_mag[8] = c_jacob_mag_tmp;
	jacob_mag[11] = d_jacob_mag_tmp;
	jacob_mag[14] = 0.0;
	jacob_mag[17] = 0.0;
	jacob_mag[20] = 0.0;
	jacob_mag[23] = 0.0;
	jacob_mag[26] = 0.0;
	jacob_mag[29] = 0.0;
	float rearranged_matrix[30];
	rearrange_matrix_row_form(jacob_mag, rearranged_matrix, 3, 10);
	memcpy(jacob_mag, rearranged_matrix, sizeof(rearranged_matrix));
}

void EKF_fs_EP2C(float qw, float qx, float qy, float qz, float C[9]) {

	float32_t q0 = qw;
	float32_t q1 = qx;
	float32_t q2 = qy;
	float32_t q3 = qz;

	C[0] = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
	C[1] = 2 * (q1 * q2 + q0 * q3);
	C[2] = 2 * (q1 * q3 - q0 * q2);
	C[3] = 2 * (q1 * q2 - q0 * q3);
	C[4] = q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3;
	C[5] = 2 * (q2 * q3 + q0 * q1);
	C[6] = 2 * (q1 * q3 + q0 * q2);
	C[7] = 2 * (q2 * q3 - q0 * q1);
	C[8] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
}

/*
 BmatEP(Q)

 B = BmatEP(Q) returns the 4x3 matrix which relates the
 body angular velocity vector w to the derivative of
 Euler parameter vector Q.

 dQ/dt = 1/2 [B(Q)] w

 */
void BmatEP(float qw, float qx, float qy, float qz, float B[12]) {
	B[0] = -qx;
	B[1] = -qy;
	B[2] = -qz;
	B[3] = qw;
	B[4] = -qz;
	B[5] = qy;
	B[6] = qz;
	B[7] = qw;
	B[8] = -qx;
	B[9] = -qy;
	B[10] = qx;
	B[11] = qw;
}

void EKF_fs_normalise_vector(float *vector, size_t len) {
	float magnitude = 0;
	// Add the squares of each element
	for (int i = 0; i < len; i++) {
		magnitude += vector[i] * vector[i];
	}
	magnitude = sqrt(magnitude);

	// Normalise each element of the input vector
	for (int i = 0; i < len; i++) {
		vector[i] /= magnitude;
	}
}

EKF_fs_Status_t EKF_fs_calculate_K_matrix(EKF_fs_t *ekf, arm_matrix_instance_f32 *J, arm_matrix_instance_f32 *K, arm_matrix_instance_f32 *R) {
	// Temporary storage buffers
	arm_matrix_instance_f32 tmp1;
	float tmp1_data[J->numRows * J->numCols];
	arm_mat_init_f32(&tmp1, J->numRows, J->numCols, tmp1_data);

	arm_matrix_instance_f32 tmp2;
	float tmp2_data[J->numRows * J->numRows];
	arm_mat_init_f32(&tmp2, J->numRows, J->numRows, tmp2_data);

	arm_matrix_instance_f32 tmp3;
	float tmp3_data[J->numRows * J->numRows];
	arm_mat_init_f32(&tmp3, J->numRows, J->numRows, tmp3_data);

	arm_matrix_instance_f32 tmp4;
	float tmp4_data[J->numCols * J->numRows];
	arm_mat_init_f32(&tmp4, J->numCols, J->numRows, tmp4_data);

	// Create jacobian transpose matrix
	arm_matrix_instance_f32 J_t;
	float J_t_data[J->numRows * J->numCols];
	arm_mat_init_f32(&J_t, J->numCols, J->numRows, J_t_data);
	arm_status res = arm_mat_trans_f32(J, &J_t);
	if (res)
		return res;

	// Multiply jacobian by P and store in temp matrix
	res = arm_mat_mult_f32(J, &ekf->P, &tmp1);
	if (res)
		return res;

	// Multiply result by jacobian transpose and store in second temp matrix
	res = arm_mat_mult_f32(&tmp1, &J_t, &tmp2);
	if (res)
		return res;

	// Add result to R matrix
	res = arm_mat_add_f32(&tmp2, R, &tmp2);
	if (res)
		return res;

	// Calculate inverse of result and store in tmp3
	res = arm_mat_inverse_f32(&tmp2, &tmp3);
	if (res)
		return res;

	// Multiply jacobian inverse by result and store in tmp4
	res = arm_mat_mult_f32(&J_t, &tmp3, &tmp4);
	if (res)
		return res;

	// Update K from the product of P and tmp4
	res = arm_mat_mult_f32(&ekf->P, &tmp4, K);

	return res;
}

EKF_fs_Status_t EKF_fs_update_P_matrix(EKF_fs_t *ekf, arm_matrix_instance_f32 *J, arm_matrix_instance_f32 *K) {
	// Temporary storage buffers
	float tmp_10x10_data[STATE_VEC_DIMS * STATE_VEC_DIMS];
	arm_matrix_instance_f32 tmp_10x10;
	arm_mat_init_f32(&tmp_10x10, STATE_VEC_DIMS, STATE_VEC_DIMS, tmp_10x10_data);

	float tmp_2_data[ekf->P.numRows * ekf->P.numCols];
	arm_matrix_instance_f32 tmp_2;
	arm_mat_init_f32(&tmp_2, ekf->P.numRows, ekf->P.numCols, tmp_2_data);

	// Multiply K by J and store in tmp10x10
	arm_status res = arm_mat_mult_f32(K, J, &tmp_10x10);
	if (res)
		return res;

	// Subtract all diagonal elements of the result from 1
	float eye_data[100];
	create_diagonal_matrix(eye_data, STATE_VEC_DIMS, STATE_VEC_DIMS, 1.0);
	arm_matrix_instance_f32 eye;
	arm_mat_init_f32(&eye, STATE_VEC_DIMS, STATE_VEC_DIMS, eye_data);

	res = arm_mat_sub_f32(&eye, &tmp_10x10, &tmp_10x10);
	if (res) {
		return res;
	}

// Multiply result by P
	res = arm_mat_mult_f32(&tmp_10x10, &ekf->P, &tmp_2);
	if (res)
		return res;

// Store result in P to update P
	memcpy(ekf->P_data, tmp_2_data, sizeof(tmp_2_data));

	return res;
}

/*
 * Function: create_identity_matrix
 * Fills the output_matrix with values to make it an identity matrix
 * Inputs:
 * float *output_matrix: The matrix that you want to be an identity. It should be a (1xn) matrix where n = rows*cols
 * int rows: Number of rows in the output matrix
 * int cols: Number of columns in the output matrix
 * float init_diag: The value that will be placed at each diagonal position of the matrix. This is 1 for identity matrix
 */
void create_diagonal_matrix(float *output_matrix, int rows, int cols, float init_diag) {
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			if (i == j) {
				output_matrix[i * cols + j] = init_diag;
			} else {
				output_matrix[i * cols + j] = 0;
			}
		}
	}
}

/* Function Definitions */
/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_powd_snf(double u0, double u1) {
	double y;
	if (rtIsNaN(u0) || rtIsNaN(u1)) {
		y = rtNaN;
	} else {
		double d;
		double d1;
		d = fabs(u0);
		d1 = fabs(u1);
		if (rtIsInf(u1)) {
			if (d == 1.0) {
				y = 1.0;
			} else if (d > 1.0) {
				if (u1 > 0.0) {
					y = rtInf;
				} else {
					y = 0.0;
				}
			} else if (u1 > 0.0) {
				y = 0.0;
			} else {
				y = rtInf;
			}
		} else if (d1 == 0.0) {
			y = 1.0;
		} else if (d1 == 1.0) {
			if (u1 > 0.0) {
				y = u0;
			} else {
				y = 1.0 / u0;
			}
		} else if (u1 == 2.0) {
			y = u0 * u0;
		} else if ((u1 == 0.5) && (u0 >= 0.0)) {
			y = sqrt(u0);
		} else if ((u0 < 0.0) && (u1 > floor(u1))) {
			y = rtNaN;
		} else {
			y = pow(u0, u1);
		}
	}
	return y;
}

/*
 * E = EKF_fs_EP2Euler321(Q) translates the Euler parameter vector
 * Q into the corresponding (3-2-1) Euler angle set.
 */
void EKF_fs_EP2Euler321(float *qu, float *euler) {
	float q0 = qu[0];
	float q1 = qu[1];
	float q2 = qu[2];
	float q3 = qu[3];
// Normalise input quaternion
	float d = sqrt(pow(q0, 2) + pow(q1, 2) + pow(q2, 2) + pow(q3, 2));
	q0 /= d;
	q1 /= d;
	q2 /= d;
	q3 /= d;
	euler[0] = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);
	euler[1] = asin(-2 * (q1 * q3 - q0 * q2));
	euler[2] = atan2(2 * (q2 * q3 + q0 * q1), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3);
}

/*
 * Function: rearrange_matrix_row_form
 * Rearranges a flattened matrix that was flattened column by column into one that is flattened row by row
 * float* in: Pointer to input matrix
 * float* out: Pointer to output matrix
 * int in_rows: Number of rows of input matrix (same as output)
 * int in_cols: Number of columns of output matrix (same as output)
 * Returns: void
 */
void rearrange_matrix_row_form(float *in, float *out, int in_rows, int in_cols) {
	for (int i = 0; i < in_rows; i++) {
		for (int j = 0; j < in_cols; j++) {
			out[i * in_rows + j] = in[j * in_cols + i];
		}
	}
}

/*
 * Function: calibrate_accelerometer
 * Calculates the offsets of a stationary accelerometer
 * float ax: Acceleration reading in X
 * float ay: Acceleration reading in Y
 * float az: Acceleration reading in Z
 * float *offxyz (output): Acceleration offset result in X, Y and Z as a vector
 */
void calibrate_accelerometer(float ax, float ay, float az, float *offxyz) {
	float acc_vec[] = {ax, ay, az};
	EKF_fs_normalise_vector(acc_vec, 3);
	offxyz[0] = ax - acc_vec[0];
	offxyz[1] = ay - acc_vec[1];
	offxyz[2] = az - acc_vec[2];
}
