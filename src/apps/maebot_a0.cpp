#include <lcm/lcm.h>
#include <pthread.h>
#include <unistd.h>
#include <stdlib.h>
#include <inttypes.h>
#include <stdio.h>

#include "maebot/rplidar.h"
#include "common/serial.h"
#include "common/timestamp.h"
#include "common/timestamp.h"
#include "imagesource/image_util.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"
#include "lcmtypes/maebot_motor_command_t.h"
#include "lcmtypes/maebot_motor_feedback_t.h"
#include "lcmtypes/maebot_a0_sensor_data_t.h"
#include "lcmtypes/maebot_sensor_data_t.h"
#include "lcmtypes/maebot_laser_scan_t.h"

#define CMD_PRD 50000 //us  -> 20Hz
#define MTR_SPD 0.25f
#define MTR_STOP 0.0f
#define TICK_2_FEET 2911
#define TICK_3_FEET 4366
#define TICK_TURN 300

typedef struct {
	// motor command
	maebot_motor_command_t motor_command_msg;
	pthread_mutex_t motor_command_msg_mutex;

	// camera
	image_source_t *isrc;

	// lidar
	int dev;
	bool scan_finished;
	pthread_mutex_t scan_mutex;
	int num_ranges;
	float* ranges;
	float* thetas;
	long int* times;
	float* intensities;

	// encoder
	pthread_mutex_t tick_mutex;
	int tick; // encoder ticks
	int initial_tick; // an intial tick

	lcm_t* lcm; // lcm
} state_t;

state_t state; // global state

int count;
void* diff_drive_thread(void* arg)
{
	uint64_t utime_start;
	while(1) {
		utime_start = utime_now();
		pthread_mutex_lock(&state.motor_command_msg_mutex);
		maebot_motor_command_t_publish(state.lcm, "MAEBOT_MOTOR_COMMAND", &state.motor_command_msg);
		pthread_mutex_unlock(&state.motor_command_msg_mutex);

		usleep(CMD_PRD - (utime_now() - utime_start));
	}

	return NULL;
}

static void motor_feedback_handler(const lcm_recv_buf_t *rbuf,
	const char *channel,
	const maebot_motor_feedback_t *motor_command_msg, void *user)
{
	pthread_mutex_lock(&state.tick_mutex);
	state.tick = motor_command_msg->encoder_left_ticks;
	pthread_mutex_unlock(&state.tick_mutex);
}

static void laser_data_handler(const lcm_recv_buf_t *rbuf,
	const char *channel,
	const maebot_laser_scan_t *msg, void *user) {
	pthread_mutex_lock(&state.scan_mutex);
	state.num_ranges = msg->num_ranges;
	if (state.ranges != NULL) free(state.ranges);
	state.ranges = (float*) malloc(sizeof(float) * state.num_ranges);

	if (state.thetas != NULL) free(state.thetas);
	state.thetas = (float*) malloc(sizeof(float) * state.num_ranges);

	if (state.times != NULL) free(state.times);
	state.times = (long int*) malloc(sizeof(long int) * state.num_ranges);

	if (state.intensities != NULL) free(state.intensities);
	state.intensities = (float*) malloc(sizeof(float) * state.num_ranges);

	state.scan_finished = true;
	pthread_mutex_unlock(&state.scan_mutex);
}

void* lcm_handle_thread(void* arg) {
	while(1) {
		pthread_mutex_lock(&state.motor_command_msg_mutex);
		lcm_handle(state.lcm);
		pthread_mutex_unlock(&state.motor_command_msg_mutex);
		usleep(1000);
	}

	return NULL;
}

// remember to destroy the image_u32_t
image_u32_t* take_picture(void) {
	image_source_data_t isdata;
	int res = state.isrc->get_frame(state.isrc, &isdata);
	if (res) {
		printf("error taking picture\n");
		exit(1);
	}
	image_u32_t* im = image_convert_u32(&isdata);
	state.isrc->release_frame(state.isrc, &isdata);
	return im;
}

void poll_sensors(void) {
	maebot_a0_sensor_data_t lcm_msg;

	// camera
	image_u32_t* im = take_picture();
	lcm_msg.height = im->height;
	lcm_msg.buffer_size = im->height * im->stride;
	lcm_msg.buff = (int32_t*) malloc(sizeof(int32_t) * lcm_msg.buffer_size);
	memcpy(lcm_msg.buff, im->buf, lcm_msg.buffer_size);

	// lidar

	maebot_a0_sensor_data_t_publish(state.lcm, "MAEBOT_A0_SENSOR_DATA", &lcm_msg);

	free(lcm_msg.buff);
	image_u32_destroy(im);
}

void init_state(void) {
	// initialization of state
	if (pthread_mutex_init(&state.motor_command_msg_mutex, NULL)) {
		printf("motor_command_msg mutex init failed\n");
		exit(1);
	}

	if (pthread_mutex_init(&state.tick_mutex, NULL)) {
		printf("tick mutex init failed\n");
		exit(1);
	}

	if (pthread_mutex_init(&state.scan_mutex, NULL)) {
		printf("scan mutex init failed\n");
		exit(1);
	}

	state.num_ranges = 0;
	state.ranges = NULL;
	state.thetas = NULL;
	state.times = NULL;
	state.intensities = NULL;

	state.lcm = lcm_create(NULL);
	if (!state.lcm) {
		printf("lcm create failed\n");
		exit(1);
	}

	zarray_t * urls=image_source_enumerate();
	for (int i = 0; i < zarray_size(urls); i++) {
		char *url;
		zarray_get(urls, i, &url);
		state.isrc = image_source_open(url);
		if (state.isrc == NULL) {
			printf("Unable to open device %s\n", url);
		} else {
			printf("opened %s\n", url);
			break;
		}
	}
	if (state.isrc == NULL) {
		printf("Unable to open any device\n");
		exit(1);
	}

	if (state.isrc->start(state.isrc))
		exit(-1);

	// Init state.motor_command_msg
	// no need for mutex here, as command thread hasn't started yet.
	state.motor_command_msg.motor_left_speed = MTR_STOP;
	state.motor_command_msg.motor_right_speed = MTR_STOP;

	// Start sending motor commands
	pthread_t diff_drive_thread_pid;
	pthread_create (&diff_drive_thread_pid, NULL, diff_drive_thread, NULL);

	maebot_motor_feedback_t_subscribe(state.lcm,
		"MAEBOT_MOTOR_FEEDBACK",
		motor_feedback_handler,
		NULL);

	maebot_laser_scan_t_subscribe(state.lcm,
		"MAEBOT_LASER_SCAN",
		laser_data_handler,
		NULL);

	pthread_t lcm_handle_thread_pid;
	pthread_create(&lcm_handle_thread_pid, NULL, lcm_handle_thread, NULL);

	pthread_mutex_lock(&state.tick_mutex);
	state.tick = -60000;
	pthread_mutex_unlock(&state.tick_mutex);
}

int main(int argc, char *argv[])
{
	init_state();
	for (int i = 0; i < 6; ++i)
	{
		// check initial check
		while (1) {
			pthread_mutex_lock(&state.tick_mutex);
			if (state.tick != -60000) {
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
		pthread_mutex_lock(&state.motor_command_msg_mutex);
		state.motor_command_msg.motor_left_speed  = MTR_SPD;
		state.motor_command_msg.motor_right_speed = MTR_SPD;
		pthread_mutex_unlock(&state.motor_command_msg_mutex);
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
		pthread_mutex_lock(&state.motor_command_msg_mutex);
		state.motor_command_msg.motor_left_speed  = MTR_STOP;
		state.motor_command_msg.motor_right_speed = MTR_STOP;
		pthread_mutex_unlock(&state.motor_command_msg_mutex);

		// update initial_tick
		state.initial_tick += TICK_2_FEET;

		// scan
		poll_sensors();
		printf("finished scan\t");

		// turn
		printf("turning\t");
		pthread_mutex_lock(&state.motor_command_msg_mutex);
		state.motor_command_msg.motor_left_speed  = MTR_SPD;
		state.motor_command_msg.motor_right_speed = -MTR_SPD;
		pthread_mutex_unlock(&state.motor_command_msg_mutex);
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
		pthread_mutex_lock(&state.motor_command_msg_mutex);
		state.motor_command_msg.motor_left_speed  = MTR_SPD;
		state.motor_command_msg.motor_right_speed = MTR_SPD;
		pthread_mutex_unlock(&state.motor_command_msg_mutex);
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
		pthread_mutex_lock(&state.motor_command_msg_mutex);
		state.motor_command_msg.motor_left_speed  = MTR_STOP;
		state.motor_command_msg.motor_right_speed = MTR_STOP;
		pthread_mutex_unlock(&state.motor_command_msg_mutex);

		// scan
		poll_sensors();
		printf("finished scan\t");

		// turn
		printf("turning\n");
		pthread_mutex_lock(&state.motor_command_msg_mutex);
		state.motor_command_msg.motor_left_speed  = MTR_SPD;
		state.motor_command_msg.motor_right_speed = -MTR_SPD;
		pthread_mutex_unlock(&state.motor_command_msg_mutex);
		while (1) {
			pthread_mutex_lock(&state.tick_mutex);
			if (state.tick > TICK_TURN + state.initial_tick) {
				pthread_mutex_unlock(&state.tick_mutex);
				break;
			}
			pthread_mutex_unlock(&state.tick_mutex);
		}

		state.initial_tick += TICK_TURN;
	}

	printf("done\n");
	state.isrc->close(state.isrc);
	lcm_destroy(state.lcm);
	serial_close(state.dev);
	return 0;
}
