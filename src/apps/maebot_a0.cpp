#include <lcm/lcm.h>
#include <pthread.h>
#include <unistd.h>
#include <stdlib.h>
#include <inttypes.h>
#include <stdio.h>

#include <fstream>

#include "common/timestamp.h"
#include "lcmtypes/maebot_motor_command_t.h"
#include "lcmtypes/maebot_motor_feedback_t.h"

#define CMD_PRD 50000 //us  -> 20Hz
#define MTR_SPD 0.25f
#define MTR_STOP 0.0f
#define TICK_2_FEET 2911
#define TICK_3_FEET 4366
#define TICK_TURN 300

typedef struct {
	maebot_motor_command_t msg; // motor state.msg
	pthread_mutex_t msg_mutex; // mutex for motor command state.msg

	pthread_mutex_t tick_mutex; // mutex for state.tick
	int tick; // encoder ticks
	int initial_tick; // an intial state.tick

	lcm_t* lcm; // lcm
} state_t;

state_t state; // global state

int count;
void* diff_drive_thread(void* arg)
{
	uint64_t utime_start;
	while(1) {
		utime_start = utime_now();
		pthread_mutex_lock(&state.msg_mutex);
		// printf("sending message: %lf, \t%lf\t%d\n", state.msg.motor_left_speed, state.msg.motor_right_speed, count++);
		maebot_motor_command_t_publish(state.lcm, "MAEBOT_MOTOR_COMMAND", &state.msg);
		pthread_mutex_unlock(&state.msg_mutex);

		usleep(CMD_PRD - (utime_now() - utime_start));
	}

	return NULL;
}

static void motor_feedback_handler(const lcm_recv_buf_t *rbuf,
	const char *channel, const maebot_motor_feedback_t *msg, void *user)
{
	int res = system("clear");
	if (res)
		printf("system clear failed\n");

	pthread_mutex_lock(&state.tick_mutex);
	state.tick = msg->encoder_left_ticks;
	// printf("ticks: %d\n", state.tick);
	pthread_mutex_unlock(&state.tick_mutex);
}

void* lcm_handle_thread(void* arg) {
	while(1) {
		pthread_mutex_lock(&state.msg_mutex);
		lcm_handle(state.lcm);
		pthread_mutex_unlock(&state.msg_mutex);
		usleep(1000);
	}

	return NULL;
}

int main(int argc, char *argv[])
{
	std::ofstream log;
	log.open("out.txt");

	if (pthread_mutex_init(&state.msg_mutex, NULL)) {
		printf("state.msg mutex init failed\n\r");
		return 1;
	}

	if (pthread_mutex_init(&state.tick_mutex, NULL)) {
		printf("tick mutex init failed\n\r");
		return 1;
	}

	state.lcm = lcm_create(NULL);
	if (!state.lcm) {
		printf("lcm create failed\n\r");
		return 1;
	}

	// Init state.msg
	// no need for mutex here, as command thread hasn't started yet.
	state.msg.motor_left_speed = MTR_STOP;
	state.msg.motor_right_speed = MTR_STOP;

	// Start sending motor commands
	pthread_t diff_drive_thread_pid;
	pthread_create (&diff_drive_thread_pid, NULL, diff_drive_thread, NULL);

	maebot_motor_feedback_t_subscribe(state.lcm,
		"MAEBOT_MOTOR_FEEDBACK",
		motor_feedback_handler,
		NULL);

	pthread_t lcm_handle_thread_pid;
	pthread_create(&lcm_handle_thread_pid, NULL, lcm_handle_thread, NULL);

	pthread_mutex_lock(&state.tick_mutex);
	state.tick = 0;
	pthread_mutex_unlock(&state.tick_mutex);

	// check initial check
	while (1) {
		pthread_mutex_lock(&state.tick_mutex);
		if (state.tick != 0) {
			pthread_mutex_unlock(&state.tick_mutex);
			break;
		}
		pthread_mutex_unlock(&state.tick_mutex);
	}
	pthread_mutex_lock(&state.tick_mutex);
	state.initial_tick = state.tick;
	pthread_mutex_unlock(&state.tick_mutex);

	// go forward 2 feet
	printf("forward2\t");
	log << "forward2\n";
	pthread_mutex_lock(&state.msg_mutex);
	state.msg.motor_left_speed  = MTR_SPD;
	state.msg.motor_right_speed = MTR_SPD;
	pthread_mutex_unlock(&state.msg_mutex);
	while (1) {
		pthread_mutex_lock(&state.tick_mutex);
		if (state.tick > TICK_2_FEET + state.initial_tick) {
			pthread_mutex_unlock(&state.tick_mutex);
			break;
		}
		pthread_mutex_unlock(&state.tick_mutex);
	}

	// stop
	printf("stopping\t");
	log << "stopping\n";
	pthread_mutex_lock(&state.msg_mutex);
	state.msg.motor_left_speed  = MTR_STOP;
	state.msg.motor_right_speed = MTR_STOP;
	pthread_mutex_unlock(&state.msg_mutex);

	// update initial_tick
	state.initial_tick += TICK_2_FEET;

	// scan

	// turn
	printf("turning\t");
	log << "turning\n";
	pthread_mutex_lock(&state.msg_mutex);
	state.msg.motor_left_speed  = MTR_SPD;
	state.msg.motor_right_speed = -MTR_SPD;
	pthread_mutex_unlock(&state.msg_mutex);
	while (1) {
		pthread_mutex_lock(&state.tick_mutex);
		if (state.tick > TICK_TURN + state.initial_tick) {
			pthread_mutex_unlock(&state.tick_mutex);
			break;
		}
		pthread_mutex_unlock(&state.tick_mutex);
	}

	state.initial_tick += TICK_TURN;

	// go forward 3 feet
	printf("forward3\t");
	log << "forward3\n";
	pthread_mutex_lock(&state.msg_mutex);
	state.msg.motor_left_speed  = MTR_SPD;
	state.msg.motor_right_speed = MTR_SPD;
	pthread_mutex_unlock(&state.msg_mutex);
	while (1) {
		pthread_mutex_lock(&state.tick_mutex);
		if (state.tick > TICK_3_FEET + state.initial_tick) {
			pthread_mutex_unlock(&state.tick_mutex);
			break;
		}
		pthread_mutex_unlock(&state.tick_mutex);
	}

	state.initial_tick += TICK_3_FEET;

	// stop
	printf("stopping\t");
	log << "stopping\n";
	pthread_mutex_lock(&state.msg_mutex);
	state.msg.motor_left_speed  = MTR_STOP;
	state.msg.motor_right_speed = MTR_STOP;
	pthread_mutex_unlock(&state.msg_mutex);

	// scan


	// turn
	printf("turning\t");
	log << "turning\n";
	pthread_mutex_lock(&state.msg_mutex);
	state.msg.motor_left_speed  = MTR_SPD;
	state.msg.motor_right_speed = -MTR_SPD;
	pthread_mutex_unlock(&state.msg_mutex);
	while (1) {
		pthread_mutex_lock(&state.tick_mutex);
		if (state.tick > TICK_TURN + state.initial_tick) {
			pthread_mutex_unlock(&state.tick_mutex);
			break;
		}
		pthread_mutex_unlock(&state.tick_mutex);
	}

	state.initial_tick += TICK_TURN;

	printf("done\t");
	log << "done\n";

	log.close();
	return 0;
}
