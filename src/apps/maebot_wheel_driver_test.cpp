#include <lcm/lcm.h>
#include <pthread.h>
#include <unistd.h>
#include <stdlib.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>

#include "lcmtypes/maebot_motor_driver_command_t.h"

struct state_t {
	// lcm stuff
	lcm_t* lcm;

	maebot_motor_driver_command_t motor_command_msg;
};

int main() {
	state_t state;
	state.lcm = lcm_create(NULL);

	for (int i = 0; i < 6; ++i) {
		state.motor_command_msg.motor_speed_left = 0.20;
		state.motor_command_msg.motor_speed_right = 0.20;
		maebot_motor_driver_command_t_publish(state.lcm, "MAEBOT_MOTOR_DRIVER_COMMAND", &state.motor_command_msg);
		usleep(5 * 1e6);

		state.motor_command_msg.motor_speed_left = 0;
		state.motor_command_msg.motor_speed_right = 0;
		maebot_motor_driver_command_t_publish(state.lcm, "MAEBOT_MOTOR_DRIVER_COMMAND", &state.motor_command_msg);
		usleep(0.5 * 1e6);

		state.motor_command_msg.motor_speed_left = 0.25;
		state.motor_command_msg.motor_speed_right = -0.25;
		maebot_motor_driver_command_t_publish(state.lcm, "MAEBOT_MOTOR_DRIVER_COMMAND", &state.motor_command_msg);
		usleep(1 * 1e6);

		state.motor_command_msg.motor_speed_left = 0;
		state.motor_command_msg.motor_speed_right = 0;
		maebot_motor_driver_command_t_publish(state.lcm, "MAEBOT_MOTOR_DRIVER_COMMAND", &state.motor_command_msg);
		usleep(1 * 1e6);
	}
}

