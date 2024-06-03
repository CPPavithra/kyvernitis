#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <math.h>
#include <stdio.h>

#define M_PI 3.14159265358979323846

static const char *now_str(void)
{
	static char buf[16]; /* ...HH:MM:SS.MMM */
	uint32_t now = k_uptime_get_32();
	unsigned int ms = now % MSEC_PER_SEC;
	unsigned int s;
	unsigned int min;
	unsigned int h;

	now /= MSEC_PER_SEC;
	s = now % 60U;
	now /= 60U;
	min = now % 60U;
	now /= 60U;
	h = now;

	snprintf(buf, sizeof(buf), "%u:%02u:%02u.%03u",
		 h, min, s, ms);
	return buf;
}


uint64_t prev_time = 0;
struct joint {
	float accel[3];
	float gyro[3];
	float pitch;
};

float pitch_gyro = 0;
float gyroOffset = 0;
float alpha = 0.96;
static int process_mpu6050(const struct device *dev, struct joint *joint)
{
	uint64_t current_time = k_uptime_get();
	float dt = (current_time - prev_time)/ 1000.0;
	prev_time = current_time;

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
	
	for(int i = 0; i < 3; i++) {
		joint->accel[i] = sensor_value_to_double(&accel[i]);
		joint->gyro[i] = sensor_value_to_double(&gyro[i]);
	};	

	float pitch_accel = (180 * atan2(-1*joint->accel[0], sqrt(pow(joint->accel[1],2) + pow(joint->accel[2],2)))/M_PI);

	joint->pitch = alpha * (joint->pitch + (joint->gyro[1] - gyroOffset)*dt) + (1 - alpha) * pitch_accel;

	if (rc == 0) {
		// printf("[%s]: "
		//        "  accel %f %f %f m/s/s\n"
		//        "  gyro  %f %f %f rad/s\n"
		//        "  pitch %f degrees\n",
		//        now_str(),
		//        sensor_value_to_double(&accel[0]),
		//        sensor_value_to_double(&accel[1]),
		//        sensor_value_to_double(&accel[2]),
		//        sensor_value_to_double(&gyro[0]),
		//        sensor_value_to_double(&gyro[1]),
		//        sensor_value_to_double(&gyro[2]),
		//        joint.pitch);
		printf("[%s]: %f\n", now_str(), joint->pitch );

	} else {
		printf("sample fetch/get failed: %d\n", rc);
	}

	return rc;
}


int main(void)
{
	const struct device *const mpu6050 = DEVICE_DT_GET(DT_ALIAS(imu_lower_joint));

	if (!device_is_ready(mpu6050)) {
		printf("Device %s is not ready\n", mpu6050->name);
		return 0;
	}
	struct sensor_value gyro[3];

	for (int i = 0; i < 1000; i++) {
		int rc = sensor_sample_fetch(mpu6050);

		rc = sensor_channel_get(mpu6050, SENSOR_CHAN_GYRO_XYZ,
					gyro);

		gyroOffset += sensor_value_to_double(&gyro[1]);
		k_sleep(K_MSEC(10));
	}

	gyroOffset /= 1000;

	struct joint joint;
	joint.pitch = 0;
	while (1) {
		int rc = process_mpu6050(mpu6050, &joint);

		if (rc != 0) {
			break;
		}
		k_sleep(K_MSEC(4));
	}

	/* triggered runs with its own thread after exit */
	return 0;
}
