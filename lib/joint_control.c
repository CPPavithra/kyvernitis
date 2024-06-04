#include <zephyr/drivers/sensor.h>
#include <kyvernitis/lib/joint_control.h>

float tau = 0.96;

/*
 * Initializes the pitch angle of a joint with the pitch computer from acceleration
 * Returns 0 if successful
 */
int arm_joint_status_init(const struct device *dev, struct ArmJointStatus *joint)
{
	struct sensor_value accel[3];
	int rc = sensor_sample_fetch(dev);
	rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);

	float faccel[3];
	for (int i = 0; i < 3; i++) {
		faccel[i] = sensor_value_to_double(&accel[i]);
	}
	joint->angle =
		(180 * atan2(-1 * faccel[0], sqrt(pow(faccel[1], 2) + pow(faccel[2], 2))) / M_PI);
	joint->prev_time = 0;

	return rc;
}

/*
 * Processes IMU data to generate Pitch angle for a joint
 * Returns 0 if successful
 */
int process_imu(const struct device *dev, struct ArmJointStatus *joint)
{
	uint64_t current_time = k_uptime_get();
	joint->dt = (current_time - joint->prev_time) / 1000.0;
	joint->prev_time = current_time;

	struct sensor_value accel[3];
	struct sensor_value gyro[3];
	int rc = sensor_sample_fetch(dev);

	if (rc == 0) {
		rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
	}
	if (rc == 0) {
		rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
	}
	if (rc == 0) {
		for (int i = 0; i < 3; i++) {
			joint->accel[i] = sensor_value_to_double(&accel[i]);
			joint->gyro[i] = sensor_value_to_double(&gyro[i]);
		};

		float pitch_accel = (180 *
				     atan2(-1 * joint->accel[0], sqrt(pow(joint->accel[1], 2) +
								      pow(joint->accel[2], 2))) /
				     M_PI);

		joint->angle =
			tau * (joint->angle + (joint->gyro[1] - joint->gyroOffset) * (joint->dt)) +
			(1 - tau) * pitch_accel;
	}

	return rc;
}

/*
 * Calibrates IMU for a joint and set offset for pitch angle
 * Returns 0 on success
 */
int calibrate_imu(const struct device *dev, struct ArmJointStatus *joint)
{
	int rc;
	struct sensor_value gyro[3];

	for (int i = 0; i < 1000; i++) {
		rc = sensor_sample_fetch(dev);

		if (rc == 0) {
			rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
		}
		if (rc == 0) {
			joint->gyroOffset += sensor_value_to_double(&gyro[1]);
			k_sleep(K_MSEC(4));
		}
	}

	if (rc == 0) {
		joint->gyroOffset /= 1000;
		rc = arm_joint_status_init(dev, joint);
		if (rc < 0) {
			joint->angle = 0;
		}
	}
	return rc;
}

/*
 * Updates angle to be achieved
 * Returns 0 on Success
 */
int update_pid(const struct device *dev, struct ArmJointStatus *joint)
{
	int ret = 0;
	ret = process_imu(dev, joint);
	struct PID *pid = &(joint->pid);

	float error = joint->desired_angle - joint->angle;
	if (error > 90) {
		return -1;
	}

	if(fabs(error) <= 1) {
		pid->pid_change = 0;
	}

	if (fabs(error) > 1) {

		pid->integral += error * joint->dt;
		pid->derivative= (error - pid->previous_error) / joint->dt;
		pid->pid_change = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * pid->derivative;

		pid->previous_error = error;
	} 

	return ret;
}

/*
 * pid_change to PWM interpolation
 */
uint32_t pid_pwm_interp(float pid_change, float *angle_range, uint32_t *pwm_range) {

	if (pid_change > angle_range[1]) {
		return pwm_range[1];
	}

	if (pid_change < angle_range[0]) {
		return pwm_range[0];
	}
	if (abs((int)angle_range * 100) == 0) {
		return (uint32_t)((pwm_range[0] + pwm_range[1]) / 2);
	}

	float dangle = angle_range[1] - angle_range[0];
	float dpwm = pwm_range[1] - pwm_range[0];

	uint32_t pwm_interp = pwm_range[0] + (dpwm / dangle) * (pid_change - angle_range[0]);

	return pwm_interp;
}
