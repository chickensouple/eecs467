#include <gtk/gtk.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>
#include <lcm/lcm.h>

#include <vector>

// core api
#include "vx/vx.h"
#include "vx/vx_util.h"

#include "vx/gtk/vx_gtk_display_source.h"

// drawables
#include "vx/vxo_drawables.h"

#include "common/getopt.h"
#include "imagesource/image_u32.h"
#include "imagesource/image_util.h"

#include "lcmtypes/maebot_motor_feedback_t.h"
#include "lcmtypes/maebot_a0_sensor_data_t.h"
#include "lcmtypes/maebot_sensor_data_t.h"

#define BASE_LENGTH 0.08 // meters
#define METERS_PER_TICK 0.002083333333 // meters

typedef struct
{
	lcm_t* lcm;
	pthread_mutex_t lcm_mutex;


	bool first_odo;
	double odo_x_curr, odo_y_curr, odo_heading_curr;
	int odo_left_tick, odo_right_tick;
	std::vector<float> odo_points;
	pthread_mutex_t odo_points_mutex;

	int running;

	image_u32_t *img;

	vx_application_t app;

	vx_world_t * world;
	zhash_t * layers;

	pthread_mutex_t mutex; // for accessing the arrays
	pthread_t animate_thread;
} state_t;

state_t * state;

void* lcm_handle_thread(void* arg) {
	while(1) {
		pthread_mutex_lock(&state->lcm_mutex);
		lcm_handle(state->lcm);
		pthread_mutex_unlock(&state->lcm_mutex);
		usleep(1000);
	}

	return NULL;
}

static void sensor_data_handler(const lcm_recv_buf_t* rbuf,
	const char* channel,
	const maebot_sensor_data_t *msg, void* user) {

}

static void a0_sensor_data_handler(const lcm_recv_buf_t* rbuf,
	const char* channel,
	const maebot_a0_sensor_data_t *msg, void* user) {

}

static void motor_feedback_handler(const lcm_recv_buf_t *rbuf,
	const char *channel,
	const maebot_motor_feedback_t *msg, void *user)
{
	if (state->first_odo) {
		state->odo_left_tick = msg->encoder_left_ticks;
		state->odo_right_tick = msg->encoder_right_ticks;
		state->first_odo = false;
		return;
	}

	double deltaLeft = msg->encoder_left_ticks - state->odo_left_tick;
	double deltaRight = msg->encoder_right_ticks - state->odo_right_tick;
	state->odo_left_tick = msg->encoder_left_ticks;
	state->odo_right_tick = msg->encoder_right_ticks;

	deltaLeft *= METERS_PER_TICK;
	deltaRight *= METERS_PER_TICK;

	double distance = (deltaRight + deltaLeft) / 2;
	double theta = (deltaRight - deltaLeft) / BASE_LENGTH;
	double alpha = theta / 2;
	state->odo_x_curr += distance * cos(state->odo_heading_curr + alpha);
	state->odo_y_curr += distance * sin(state->odo_heading_curr + alpha);
	state->odo_heading_curr += theta;

	pthread_mutex_lock(&state->odo_points_mutex);
	state->odo_points.push_back(state->odo_x_curr);
	state->odo_points.push_back(state->odo_y_curr);
	state->odo_points.push_back(0);
	pthread_mutex_unlock(&state->odo_points_mutex);
}

void* draw_thread(void*) {
	while (1) {
		printf("drawing\n");
		pthread_mutex_lock(&state->odo_points_mutex);
		int vec_size = state->odo_points.size();
		vx_resc_t *verts = vx_resc_copyf((state->odo_points).data(), vec_size);
		pthread_mutex_unlock(&state->odo_points_mutex);
		vx_buffer_add_back(vx_world_get_buffer(state->world,"draw_thread"), vxo_lines(verts, vec_size / 3, GL_LINES, vxo_lines_style(vx_red, 2.0f)));
		vx_buffer_swap(vx_world_get_buffer(state->world,"draw_thread"));
		usleep(1000);
	}

	return NULL;
}

static void display_finished(vx_application_t * app, vx_display_t * disp)
{
	state_t * state = (state_t *) app->impl;
	pthread_mutex_lock(&state->mutex);

	vx_layer_t * layer = NULL;

	// store a reference to the world and layer that we associate with each vx_display_t
	zhash_remove(state->layers, &disp, NULL, &layer);

	vx_layer_destroy(layer);

	pthread_mutex_unlock(&state->mutex);
}

static void display_started(vx_application_t * app, vx_display_t * disp)
{
	state_t * state = (state_t *) app->impl;

	vx_layer_t * layer = vx_layer_create(state->world);
	vx_layer_set_display(layer, disp);

	pthread_mutex_lock(&state->mutex);
	// store a reference to the world and layer that we associate with each vx_display_t
	zhash_put(state->layers, &disp, &layer, NULL, NULL);
	pthread_mutex_unlock(&state->mutex);
}

static void state_destroy(state_t * state)
{

	if (state->img != NULL)
		image_u32_destroy(state->img);

	vx_world_destroy(state->world);
	assert(zhash_size(state->layers) == 0);

	zhash_destroy(state->layers);
	free(state);

	pthread_mutex_destroy(&state->mutex);
	lcm_destroy(state->lcm);

}

static state_t * state_create()
{
	state_t * state = (state_t * ) calloc(1, sizeof(state_t));
	state->running = 1;
	state->app.impl=state;
	state->app.display_started=display_started;
	state->app.display_finished=display_finished;

	state->lcm = lcm_create(NULL);
	if (!state->lcm) {
		printf("lcm create failed\n");
		exit(1);
	}

	if (pthread_mutex_init(&state->lcm_mutex, NULL)) {
		printf("lcm mutex init failed\n");
		exit(1);
	}

	if (pthread_mutex_init(&state->odo_points_mutex, NULL)) {
		printf("odo points mutex init failed\n");
		exit(1);
	}

	state->first_odo = true;
	state->odo_y_curr = 0;
	state->odo_x_curr = 0;
	state->odo_heading_curr = 0;

	state->world = vx_world_create();
	state->layers = zhash_create(sizeof(vx_display_t*), sizeof(vx_layer_t*), zhash_ptr_hash, zhash_ptr_equals);

	pthread_mutex_init (&state->mutex, NULL);

	return state;
}

int main(int argc, char ** argv)
{
	vx_global_init(); // Call this to initialize the vx-wide lock. Required to start the GL thread or to use the program library

	state = state_create();

	// draw(state, state->world);

	// pthread_create(&state->animate_thread, NULL, render_loop, state);
	pthread_t draw_thread_pid;
	pthread_create(&draw_thread_pid, NULL, draw_thread, NULL);

	pthread_t lcm_handle_thread_pid;
	pthread_create(&lcm_handle_thread_pid, NULL, lcm_handle_thread, NULL);

	gdk_threads_init ();
	gdk_threads_enter ();

	gtk_init (&argc, &argv);

	vx_gtk_display_source_t * appwrap = vx_gtk_display_source_create(&state->app);
	GtkWidget * window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
	GtkWidget * canvas = vx_gtk_display_source_get_widget(appwrap);
	gtk_window_set_default_size (GTK_WINDOW (window), 400, 400);
	gtk_container_add(GTK_CONTAINER(window), canvas);
	gtk_widget_show (window);
	gtk_widget_show (canvas); // XXX Show all causes errors!

	g_signal_connect_swapped(G_OBJECT(window), "destroy", G_CALLBACK(gtk_main_quit), NULL);


	maebot_motor_feedback_t_subscribe(state->lcm,
		"MAEBOT_MOTOR_FEEDBACK",
		motor_feedback_handler,
		NULL);

	maebot_sensor_data_t_subscribe(state->lcm,
		"MAEBOT_SENSOR_DATA",
		sensor_data_handler,
		NULL);

	maebot_a0_sensor_data_t_subscribe(state->lcm,
		"MAEBOT_A0_SENSOR_DATA",
		a0_sensor_data_handler,
		NULL);

	gtk_main (); // Blocks as long as GTK window is open
	gdk_threads_leave ();

	vx_gtk_display_source_destroy(appwrap);

	state_destroy(state);
	vx_global_destroy();
}
