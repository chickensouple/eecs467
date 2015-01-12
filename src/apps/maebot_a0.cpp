#include <lcm/lcm.h>
#include <pthread.h>
#include <unistd.h>

#include "common/timestamp.h"
#include "lcmtypes/maebot_motor_command_t.h"
#include "lcmtypes/maebot_motor_feedback_t.h"

#define CMD_PRD 50000 //us  -> 20Hz
#define MTR_SPD 0.25f
#define MTR_STOP 0.0f
#define TICK_2_FEET 2911
#define TICK_3_FEET 4366

maebot_motor_command_t msg;
pthread_mutex_t msg_mutex;
pthread_mutex_t tick_mutex;
int global_tick;
int global_initial_tick;

void *
diff_drive_thread (void *arg)
{
	lcm_t *lcm = lcm_create (NULL);

	uint64_t utime_start;
	while(1) {
		utime_start = utime_now ();

		pthread_mutex_lock (&msg_mutex);
		maebot_motor_command_t_publish (lcm, "MAEBOT_MOTOR_COMMAND", &msg);
		pthread_mutex_unlock (&msg_mutex);

		usleep (CMD_PRD - (utime_now() - utime_start));
	}

	return NULL;
}

static void
motor_feedback_handler (const lcm_recv_buf_t *rbuf, const char *channel,
						const maebot_motor_feedback_t *msg, void *user)
{
	int res = system ("clear");
	if (res)
		printf ("system clear failed\n");

	pthread_mutex_lock(&tick_mutex);
	global_tick = msg->encoder_left_ticks;
	pthread_mutex_unlock (&tick_mutex);
}

int
main (int argc, char *argv[])
{
	if (pthread_mutex_init (&msg_mutex, NULL)) {
		printf ("msg mutex init failed\n");
		return 1;
	}

	if (pthread_mutex_init (&tick_mutex, NULL)) {
		printf ("tick mutex init failed\n");
		return 1;
	}

	// Init msg
	// no need for mutex here, as command thread hasn't started yet.
	msg.motor_left_speed = MTR_STOP;
	msg.motor_right_speed = MTR_STOP;

	// Start sending motor commands
	pthread_t diff_drive_thread_pid;
	pthread_create (&diff_drive_thread_pid, NULL, diff_drive_thread, NULL);

	maebot_motor_feedback_t_subscribe (lcm,
		"MAEBOT_MOTOR_FEEDBACK",
		motor_feedback_handler,
		NULL);

	// // forward
	// pthread_mutex_lock (&msg_mutex);
	// msg.motor_left_speed  = MTR_SPD;
	// msg.motor_right_speed = MTR_SPD;
	// pthread_mutex_unlock (&msg_mutex);

	// usleep (500000);

	pthread_mutex_lock(&tick_mutex);
	global_tick = 0;
	pthread_mutex_unlock (&tick_mutex);

	// check initial check
	while (global_tick == 0);
	pthread_mutex_lock(&tick_mutex);
	global_initial_tick = global_tick;
	pthread_mutex_unlock (&tick_mutex);

	pthread_mutex_lock (&msg_mutex);
	msg.motor_left_speed  = MTR_SPD;
	msg.motor_right_speed = MTR_SPD;
	pthread_mutex_unlock (&msg_mutex);

	while (global_tick < TICK_2_FEET + global_initial_tick);

	// stop
	pthread_mutex_lock (&msg_mutex);
	msg.motor_left_speed  = MTR_STOP;
	msg.motor_right_speed = MTR_STOP;
	pthread_mutex_unlock (&msg_mutex);

	// scan and take whatever measurements

	// turn

	// go forward
	// wait 3 feet
	// stop
	// scan
	// turn

	return 0;
}
