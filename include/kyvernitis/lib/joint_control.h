#include <zephyr/device.h>
#include <math.h>

#define M_PI 3.14159265358979323846

struct PID {
	const float Kp, Ki, Kd;	
	float previous_error, integral, derivative;
	float pid_change;
};

struct ArmJointStatus {
	float accel[3];
	float gyro[3];
	float gyroOffset, pitch, desired_angle, dt;
	struct PID pid;
	uint64_t prev_time;
};



int process_imu(const struct device *dev, struct ArmJointStatus *joint);
int calibrate_imu(const struct device *dev, struct ArmJointStatus *joint);
int update_pid(const struct device *dev, struct ArmJointStatus *joint);
uint32_t pid_pwm_interp(float pid_change, float *angle_range, uint32_t *pwm_range);
