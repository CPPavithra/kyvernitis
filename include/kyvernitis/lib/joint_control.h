#include <zephyr/device.h>
#include <math.h>

#define M_PI 3.14159265358979323846

struct arm_joint_status {
	float accel[3];
	float gyro[3];
	float gyroOffset;
	float pitch;
	uint64_t prev_time;
};

int process_imu(const struct device *dev, struct arm_joint_status *joint);
int calibrate_imu(const struct device *dev, struct arm_joint_status *joint);
