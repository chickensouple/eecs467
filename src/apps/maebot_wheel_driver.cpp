#include <lcm/lcm.h>
#include <pthread.h>
#include <unistd.h>
#include <stdlib.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>

#include "common/serial.h"
#include "common/timestamp.h"
#include "common/timestamp.h"
#include "lcmtypes/maebot_motor_command_t.h"
#include "lcmtypes/maebot_motor_driver_command_t.h"
#include "lcmtypes/maebot_motor_feedback_t.h"

#define CMD_PRD 50000 //us  -> 20Hz

struct state_t {
	// lcm stuff
	lcm_t* lcm;
	pthread_t lcm_thread;
	pthread_mutex_t lcm_mutex;

	pthread_t diff_drive_thread;

	// wheel driver
	pthread_mutex_t wheel_driver_mutex;

	maebot_motor_command_t motor_command_msg;
};

state_t state;

static void motorFeedbackHandler(const lcm_recv_buf_t* rbuf,
	const char* channel,
	const maebot_motor_feedback_t* msg, void* user);

static void motorDriverCommandHandler(const lcm_recv_buf_t* rbuf,
	const char* channel,
	const maebot_motor_driver_command_t* msg, void* user);

void* lcmHandleThread(void* arg);

void* diffDriveThread(void* arg);

class WheelDriver {
public:
	double _cmdSpeedLeft, _cmdSpeedRight;
	int32_t _tickLeft, _tickRight;
	int64_t _utime;
	bool _isInitialized;
	double _metersPerTick;
	float _msgSpeedLeft, _msgSpeedRight;
	float _Kp;

public:
	WheelDriver(float Kp = 0.1,
		float msgSpeedLeft = 0, 
		float msgSpeedRight = 0,
		double metersPerTick = 0.00020944) : 
		_cmdSpeedLeft(0), _cmdSpeedRight(0),
		_isInitialized(false), _metersPerTick(metersPerTick),
		_msgSpeedLeft(msgSpeedLeft), _msgSpeedRight(msgSpeedRight),
		_Kp(Kp) { }

	~WheelDriver() { }

	void setCmdSpeeds(double cmdSpeedLeft, double cmdSpeedRight) {
		_cmdSpeedLeft = cmdSpeedLeft;
		_cmdSpeedRight = cmdSpeedRight;
	}

	bool isInitialized() {
		return _isInitialized;
	}

	void initialize(int32_t tickLeft, int32_t tickRight, int64_t utime) {
		_tickLeft = tickLeft;
		_tickRight = tickRight;
		_utime = utime;
		_isInitialized = true;
	}

	float getMsgSpeedLeft() {
		return _msgSpeedLeft;
	}

	float getMsgSpeedRight() {
		return _msgSpeedRight;
	}

	void update(int32_t tickLeft, int32_t tickRight, int64_t utime) {
		int32_t deltaTickLeft = tickLeft - _tickLeft;
		int32_t deltaTickRight = tickRight - _tickRight;
		int64_t deltaTime = utime - _utime;
		_tickLeft = tickLeft;
		_tickRight = tickRight;
		_utime = utime;

		double deltaTimeSeconds = (double) deltaTime / 1e6;
		double deltaTickLeftMeters = (double) deltaTickLeft * _metersPerTick;
		double deltaTickRightMeters = (double) deltaTickRight * _metersPerTick;

		double leftSpeed = deltaTickLeftMeters / deltaTimeSeconds;
		double rightSpeed = deltaTickRightMeters / deltaTimeSeconds;

		double leftError = (_cmdSpeedLeft - leftSpeed);
		double rightError = (_cmdSpeedRight - rightSpeed);

		_msgSpeedLeft += leftError * _Kp;
		_msgSpeedRight += rightError * _Kp;

		if (_msgSpeedLeft > 1) _msgSpeedLeft = 1;
		if (_msgSpeedLeft < -1) _msgSpeedLeft = -1;
		if (_msgSpeedRight > 1) _msgSpeedRight = 1;
		if (_msgSpeedRight < -1) _msgSpeedRight = 1;
	}
};

WheelDriver driver(0.08);

int main() {
	state.lcm = lcm_create(NULL);

	maebot_motor_feedback_t_subscribe(state.lcm,
		"MAEBOT_MOTOR_FEEDBACK",
		motorFeedbackHandler,
		NULL);

	maebot_motor_driver_command_t_subscribe(state.lcm,
		"MAEBOT_MOTOR_DRIVER_COMMAND",
		motorDriverCommandHandler,
		NULL);

	if (pthread_mutex_init(&state.lcm_mutex, NULL)) {
		printf("lcm mutex init failed\n");
		exit(1);
	}

	if (pthread_mutex_init(&state.wheel_driver_mutex, NULL)) {
		printf("wheel driver mutex init failed\n");
		exit(1);
	}

	pthread_create(&state.lcm_thread, NULL, lcmHandleThread, NULL);
	while(1);
}

static void motorFeedbackHandler(const lcm_recv_buf_t* rbuf,
	const char* channel,
	const maebot_motor_feedback_t* msg, void* user) {
	pthread_mutex_lock(&state.wheel_driver_mutex);
	if (!driver.isInitialized()) {
		driver.initialize(msg->encoder_left_ticks, 
			msg->encoder_right_ticks,
			msg->utime);
	} else {
		driver.update(msg->encoder_left_ticks, msg->encoder_right_ticks,
			msg->utime);
	}
	state.motor_command_msg.motor_left_speed = driver.getMsgSpeedLeft();
	state.motor_command_msg.motor_right_speed = driver.getMsgSpeedRight();
	maebot_motor_command_t_publish(state.lcm, "MAEBOT_MOTOR_COMMAND", &state.motor_command_msg);
	pthread_mutex_unlock(&state.wheel_driver_mutex);
}

void* lcmHandleThread(void* arg) {
	while(1) {
		pthread_mutex_lock(&state.lcm_mutex);
		lcm_handle_timeout(state.lcm, 1000 / 15);
		pthread_mutex_unlock(&state.lcm_mutex);
	}
	return NULL;
}

static void motorDriverCommandHandler(const lcm_recv_buf_t* rbuf,
	const char* channel,
	const maebot_motor_driver_command_t* msg, void* user) {
	pthread_mutex_lock(&state.wheel_driver_mutex);
	driver.setCmdSpeeds(msg->motor_speed_left, msg->motor_speed_right);
	pthread_mutex_unlock(&state.wheel_driver_mutex);
}
