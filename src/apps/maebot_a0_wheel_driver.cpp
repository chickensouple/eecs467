#include <lcm/lcm.h>
#include <pthread.h>
#include <unistd.h>
#include <stdlib.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>

#include <iostream>

#include "maebot/rplidar.h"
#include "common/serial.h"
#include "common/timestamp.h"
#include "common/timestamp.h"
#include "imagesource/image_util.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"
#include "imagesource/image_u8x3.h"
#include "lcmtypes/maebot_motor_command_t.h"
#include "lcmtypes/maebot_motor_feedback_t.h"
#include "lcmtypes/maebot_a0_corner_scan_t.h"
#include "lcmtypes/maebot_sensor_data_t.h"
#include "lcmtypes/maebot_laser_scan_t.h"
#include "lcmtypes/maebot_motor_driver_command_t.h"

#define CMD_PRD 50000 //us  -> 20Hz
#define MTR_SPD_L 0.20f // 0.255f
#define MTR_SPD_R 0.20f // 0.25f
#define MTR_STOP 0.0f
#define TICK_2_FEET 2911
#define TICK_3_FEET 4366
#define TICK_TURN 300 // 300 is calculated 90 degrees
#define LIDAR_BUFF_SIZE 500

typedef struct {
	// motor command
	maebot_motor_driver_command_t motor_command_msg;
	pthread_mutex_t motor_command_msg_mutex;

	// camera
	image_source_t *isrc;
	int camera_count;

	// lidar
	bool lidar_scan;
	pthread_mutex_t lidar_mutex;
	long int lidar_utime;
	int lidar_num_ranges;
	float* lidar_ranges;
	float* lidar_thetas;
	long int* lidar_times;
	float* lidar_intensities;

	// encoder
	pthread_mutex_t tick_mutex;
	int tick; // encoder ticks
	int initial_tick; // an intial tick

	lcm_t* lcm; // lcm
} state_t;

state_t state; // global state

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
	pthread_mutex_lock(&state.lidar_mutex);
	if (!state.lidar_scan) {
		pthread_mutex_unlock(&state.lidar_mutex);
		return;
	}
	pthread_mutex_unlock(&state.lidar_mutex);

	pthread_mutex_lock(&state.lidar_mutex);
	state.lidar_utime = msg->utime;
	state.lidar_num_ranges = msg->num_ranges;

	memcpy(state.lidar_ranges, msg->ranges, state.lidar_num_ranges * sizeof(float));
	memcpy(state.lidar_thetas, msg->thetas, state.lidar_num_ranges * sizeof(float));
	memcpy(state.lidar_times, msg->times, state.lidar_num_ranges * sizeof(int64_t));
	memcpy(state.lidar_intensities, msg->intensities, state.lidar_num_ranges * sizeof(float));

	state.lidar_scan = false;
	pthread_mutex_unlock(&state.lidar_mutex);
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

void take_picture(void) {
	image_source_data_t isdata;
	int res = state.isrc->get_frame(state.isrc, &isdata);
	if (res) {
		printf("error taking picture\n");
		exit(1);
	}
	image_u8x3_t* im = image_convert_u8x3(&isdata);
	state.isrc->release_frame(state.isrc, &isdata);

	char count_str[10];
	sprintf(count_str, "%d", state.camera_count++);
	char path[100] = "pictures/corner_image_";
	strcat(path, count_str);
	strcat(path, ".ppm");
	res = image_u8x3_write_pnm(im, path);
	if (res) {
		printf("error saving picture\n");
		exit(1);
	}
	image_u8x3_destroy(im);
}

void poll_sensors(void) {
	maebot_a0_corner_scan_t lcm_msg;

	// camera
	take_picture();

	// lidar
	pthread_mutex_lock(&state.lidar_mutex);
	state.lidar_scan = true;
	pthread_mutex_unlock(&state.lidar_mutex);
	while (1) {
		pthread_mutex_lock(&state.lidar_mutex);
		if (state.lidar_scan == false) {
			pthread_mutex_unlock(&state.lidar_mutex);
			break;
		}
		pthread_mutex_unlock(&state.lidar_mutex);
	}

	pthread_mutex_lock(&state.lidar_mutex);
	// get the data
	lcm_msg.lidar.utime = state.lidar_utime;
	lcm_msg.lidar.num_ranges = state.lidar_num_ranges;
	lcm_msg.lidar.ranges = (float*) malloc(sizeof(float) * state.lidar_num_ranges);
	lcm_msg.lidar.thetas = (float*) malloc(sizeof(float) * state.lidar_num_ranges);
	lcm_msg.lidar.times = (int64_t*) malloc(sizeof(int64_t) * state.lidar_num_ranges);
	lcm_msg.lidar.intensities = (float*) malloc(sizeof(float) * state.lidar_num_ranges);
	memcpy(lcm_msg.lidar.ranges, state.lidar_ranges, state.lidar_num_ranges * sizeof(float));
	memcpy(lcm_msg.lidar.thetas, state.lidar_thetas, state.lidar_num_ranges * sizeof(float));
	memcpy(lcm_msg.lidar.times, state.lidar_times, state.lidar_num_ranges * sizeof(long int));
	memcpy(lcm_msg.lidar.intensities, state.lidar_intensities, state.lidar_num_ranges * sizeof(float));

	pthread_mutex_unlock(&state.lidar_mutex);

	maebot_a0_corner_scan_t_publish(state.lcm, "MAEBOT_A0_CORNER_SCAN", &lcm_msg);

	free(lcm_msg.lidar.ranges);
	free(lcm_msg.lidar.thetas);
	free(lcm_msg.lidar.times);
	free(lcm_msg.lidar.intensities);
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

	if (pthread_mutex_init(&state.lidar_mutex, NULL)) {
		printf("scan mutex init failed\n");
		exit(1);
	}

	state.lidar_num_ranges = 0;

	state.lidar_ranges = (float*) malloc(sizeof(float) * LIDAR_BUFF_SIZE);

	state.lidar_thetas = (float*) malloc(sizeof(float) * LIDAR_BUFF_SIZE);

	state.lidar_times = (long int*) malloc(sizeof(long int) * LIDAR_BUFF_SIZE);

	state.lidar_intensities = (float*) malloc(sizeof(float) * LIDAR_BUFF_SIZE);

	state.lcm = lcm_create(NULL);
	if (!state.lcm) {
		printf("lcm create failed\n");
		exit(1);
	}

	zarray_t * urls=image_source_enumerate();
	for (int i = 0; i < zarray_size(urls); ++i) {
		char *url;
		zarray_get(urls, i, &url);
		printf("%s\n", url);
	}
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

	state.camera_count = 0;

	state.lidar_scan = false;

	// Init state.motor_command_msg
	// no need for mutex here, as command thread hasn't started yet.
	state.motor_command_msg.motor_speed_left = MTR_STOP;
	state.motor_command_msg.motor_speed_right = MTR_STOP;

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

void reset_state_tick() {
	state.initial_tick = -60000;
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
	printf("set initial: %d\n", state.tick); std::cout << std::endl;
	pthread_mutex_unlock(&state.tick_mutex);
}

int main(int argc, char *argv[])
{
	init_state();
	for (int i = 0; i < 6; ++i)
	{
		// check initial check
		reset_state_tick();

		// go forward 2 feet
		printf("forward2\t");
		std::cout << std::endl;
		pthread_mutex_lock(&state.motor_command_msg_mutex);
		state.motor_command_msg.motor_speed_left  = MTR_SPD_L;
		state.motor_command_msg.motor_speed_right = MTR_SPD_R;
		maebot_motor_driver_command_t_publish(state.lcm, 
			"MAEBOT_MOTOR_DRIVER_COMMAND", &state.motor_command_msg);
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
		printf("stopping\t"); std::cout << std::endl;
		pthread_mutex_lock(&state.motor_command_msg_mutex);
		state.motor_command_msg.motor_speed_left  = MTR_STOP;
		state.motor_command_msg.motor_speed_right = MTR_STOP;
		maebot_motor_driver_command_t_publish(state.lcm, 
			"MAEBOT_MOTOR_DRIVER_COMMAND", &state.motor_command_msg);
		pthread_mutex_unlock(&state.motor_command_msg_mutex);
		usleep(300000);

		// scan
		poll_sensors();
		printf("finished scan\t"); std::cout << std::endl;

		// turn
		reset_state_tick();
		printf("turning\t"); std::cout << std::endl;
		printf("target: %d\t", state.initial_tick + TICK_TURN); std::cout << std::endl;
		pthread_mutex_lock(&state.motor_command_msg_mutex);
		state.motor_command_msg.motor_speed_left  = MTR_SPD_L;
		state.motor_command_msg.motor_speed_right = -MTR_SPD_R;
		maebot_motor_driver_command_t_publish(state.lcm, 
			"MAEBOT_MOTOR_DRIVER_COMMAND", &state.motor_command_msg);
		pthread_mutex_unlock(&state.motor_command_msg_mutex);
		while (1) {
			pthread_mutex_lock(&state.tick_mutex);
			printf("tick: %d\t", state.tick); std::cout << std::endl;
			if (state.tick > TICK_TURN + state.initial_tick) {
				pthread_mutex_unlock(&state.tick_mutex);
				break;
			}
			pthread_mutex_unlock(&state.tick_mutex);
		}

		reset_state_tick();

		// go forward 3 feet
		printf("forward3\t"); std::cout << std::endl;
		pthread_mutex_lock(&state.motor_command_msg_mutex);
		state.motor_command_msg.motor_speed_left  = MTR_SPD_L;
		state.motor_command_msg.motor_speed_right = MTR_SPD_R;
		maebot_motor_driver_command_t_publish(state.lcm, 
			"MAEBOT_MOTOR_DRIVER_COMMAND", &state.motor_command_msg);
		pthread_mutex_unlock(&state.motor_command_msg_mutex);
		while (1) {
			pthread_mutex_lock(&state.tick_mutex);
			if (state.tick > TICK_3_FEET + state.initial_tick) {
				pthread_mutex_unlock(&state.tick_mutex);
				break;
			}
			pthread_mutex_unlock(&state.tick_mutex);
		}

		// stop
		printf("stopping\t"); std::cout << std::endl;
		pthread_mutex_lock(&state.motor_command_msg_mutex);
		state.motor_command_msg.motor_speed_left  = MTR_STOP;
		state.motor_command_msg.motor_speed_right = MTR_STOP;
		maebot_motor_driver_command_t_publish(state.lcm, 
			"MAEBOT_MOTOR_DRIVER_COMMAND", &state.motor_command_msg);
		pthread_mutex_unlock(&state.motor_command_msg_mutex);
		usleep(300000);

		// scan
		poll_sensors();
		printf("finished scan\t"); std::cout << std::endl;

		// turn
		printf("turning\n"); std::cout << std::endl;
		reset_state_tick();
		pthread_mutex_lock(&state.motor_command_msg_mutex);
		state.motor_command_msg.motor_speed_left  = MTR_SPD_L;
		state.motor_command_msg.motor_speed_right = -MTR_SPD_R;
		maebot_motor_driver_command_t_publish(state.lcm, 
			"MAEBOT_MOTOR_DRIVER_COMMAND", &state.motor_command_msg);
		pthread_mutex_unlock(&state.motor_command_msg_mutex);
		while (1) {
			pthread_mutex_lock(&state.tick_mutex);
			if (state.tick > TICK_TURN + state.initial_tick) {
				pthread_mutex_unlock(&state.tick_mutex);
				break;
			}
			pthread_mutex_unlock(&state.tick_mutex);
		}

		reset_state_tick();
	}

	printf("done\n");
	state.isrc->stop(state.isrc);
	state.isrc->close(state.isrc);
	lcm_destroy(state.lcm);
	return 0;
}
