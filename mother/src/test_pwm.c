/*
 * Test file for checking pwm pins
 */

#include "zephyr/sys/printk.h"
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>

#include <kyvernitis/lib/kyvernitis.h>

#define PWM_MOTOR_SETUP(pwm_dev_id)                                                                \
	{.dev_spec = PWM_DT_SPEC_GET(pwm_dev_id),                                                  \
	 .min_pulse = DT_PROP(pwm_dev_id, min_pulse),                                              \
	 .max_pulse = DT_PROP(pwm_dev_id, max_pulse)},
struct pwm_motor roboclaw[11] = {DT_FOREACH_CHILD(DT_PATH(pwmmotors), PWM_MOTOR_SETUP)};

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

int main()
{
	printk("Test: PWM\n\n");

	if (!gpio_is_ready_dt(&led)){
		printk("Error: Led not ready\n");
	}

	for (size_t i = 0U; i < ARRAY_SIZE(roboclaw); i++) {
		if (!pwm_is_ready_dt(&(roboclaw[i].dev_spec))) {
			printk("PWM: Roboclaw %s is not ready\n", roboclaw[i].dev_spec.dev->name);
			return 0;
		}
	}

	for(size_t i = 0U; i < ARRAY_SIZE(roboclaw); i++) {
		if(pwm_motor_write(&(roboclaw[i]),1500000)) {
			printk("Unable to write pwm pulse to PWM Motor : %d", i);
		}
	}

	if (gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE) < 0)
	{
		printk("Error: Led not configured\n");
		return 0;
	}

	printk("Successfully initialized all pins\n");
	
	while (1) {
		for(size_t i = 0U; i < ARRAY_SIZE(roboclaw); i++) {
			for(uint32_t pulse = 1100000; pulse < 1900000; pulse += 100000)
			{
				if(pwm_motor_write(&roboclaw[i], pulse)) {
					printk("Unable to write pwm pulse to PWM Motor : %d", i);
					return 0;
				}
				printk("Writing [%u] to PWM %d\n", pulse, i);
				k_sleep(K_MSEC(300));
			}
			for(uint32_t pulse = 1900000; pulse > 1100000; pulse -= 100000)
			{
				if(pwm_motor_write(&roboclaw[i], pulse)) {
					printk("Unable to write pwm pulse to PWM Motor : %d", i);
					return 0;
				}
				printk("Writing [%u] to PWM %d\n", pulse, i);
				k_sleep(K_MSEC(300));
			}
			// if(pwm_motor_write(&roboclaw[i], 1100000)) {
			// 		printk("Unable to write pwm pulse to PWM Motor : %d", i);
			// 		return 0;
			// }
		}
	}
}
