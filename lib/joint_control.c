#include <zephyr/drivers/sensor.h>
#include <kyvernitis/lib/joint_control.h>

float tau = 0.96;

/*
 * Initializes the pitch angle of a joint with the pitch computer from acceleration
 * Returns 0 if successful
 */
int arm_joint_status_init(const struct device *dev, struct arm_joint_status *joint) {
	struct sensor_value accel[3];
	int rc = sensor_sample_fetch(dev);
	rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);

	float faccel[3];
	for(int i = 0; i < 3; i++) {
		faccel[i] = sensor_value_to_double(&accel[i]);
	}
	joint->pitch = (180 * atan2(-1*faccel[0], sqrt(pow(faccel[1],2) + pow(faccel[2],2)))/M_PI);

	return rc;
}

/*
 * Processes IMU data to generate Pitch angle for a joint
 * Returns 0 if successful
 */
int process_imu(const struct device *dev, struct arm_joint_status *joint)
{
	uint64_t current_time = k_uptime_get();
	float dt = (current_time - joint->prev_time)/ 1000.0;
	joint->prev_time = current_time;

	struct sensor_value accel[3];
	struct sensor_value gyro[3];
	int rc = sensor_sample_fetch(dev);

	if (rc == 0) {
		rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ,
					accel);
	}
	if (rc == 0) {
		rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ,
					gyro);
	}
	if (rc == 0) {	
		for(int i = 0; i < 3; i++) {
			joint->accel[i] = sensor_value_to_double(&accel[i]);
			joint->gyro[i] = sensor_value_to_double(&gyro[i]);
		};	

		float pitch_accel = (180 * atan2(-1*joint->accel[0], sqrt(pow(joint->accel[1],2) + pow(joint->accel[2],2)))/M_PI);

		joint->pitch = tau * (joint->pitch + (joint->gyro[1] - joint->gyroOffset)*dt) + (1 - tau) * pitch_accel;
	}

	return rc;
}

/* 
 * Calibrates IMU for a joint and set offset for pitch angle
 * Returns 0 on success
 */
int calibrate_imu(const struct device *dev, struct arm_joint_status *joint) {
	int rc;
	struct sensor_value gyro[3];

	for (int i = 0; i < 1000; i++) {
		rc = sensor_sample_fetch(dev);
		
		if (rc == 0) {
			rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
		}
		if(rc == 0) {
			joint->gyroOffset += sensor_value_to_double(&gyro[1]);
			k_sleep(K_MSEC(4));
		}
	}

	if(rc == 0) {
		joint->gyroOffset /= 1000;
		rc = arm_joint_status_init(dev, joint);
		if(rc < 0) {
			joint->pitch = 0;
		}
	}
	return rc;
}
