#include <gtk/gtk.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>

// core api
#include "vx/vx.h"
#include "vx/vx_util.h"

#include "vx/gtk/vx_gtk_display_source.h"
#include "vx/vx_remote_display_source.h"

// drawables
#include "vx/vxo_drawables.h"

#include "common/getopt.h"
#include "imagesource/image_u32.h"
#include "imagesource/image_util.h"

typedef struct
{
	int running;

	image_u32_t *img;

	vx_application_t app;

	vx_world_t * world;
	zhash_t * layers;

	pthread_mutex_t mutex; // for accessing the arrays
	pthread_t animate_thread;
} state_t;



static void draw(state_t * state, vx_world_t * world)
{

	if (1) {
		vx_object_t *vt = vxo_text_create(VXO_TEXT_ANCHOR_TOP_RIGHT, "<<right,#0000ff>>Heads Up!\n");
		vx_buffer_t *vb = vx_world_get_buffer(world, "text");
		vx_buffer_add_back(vb, vxo_pix_coords(VX_ORIGIN_TOP_RIGHT,vt));
		vx_buffer_swap(vb);
	}

	// Draw a texture
	if (state->img != NULL){
		image_u32_t * img = state->img;
		vx_object_t * o3 = vxo_image_texflags(vx_resc_copyui(img->buf, img->stride*img->height),
			img->width, img->height, img->stride,
			GL_RGBA, VXO_IMAGE_FLIPY,
			VX_TEX_MIN_FILTER | VX_TEX_MAG_FILTER);

		// pack the image into the unit square
		vx_buffer_t * vb = vx_world_get_buffer(world, "texture");
		vx_buffer_add_back(vb,vxo_chain(
			vxo_mat_scale(1.0/img->height),
			vxo_mat_translate3(0, - img->height, 0),
					o3));
		vx_buffer_swap(vb);
	}
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

}

static state_t * state_create()
{
	state_t * state = (state_t * ) calloc(1, sizeof(state_t));
	state->running = 1;
	state->app.impl=state;
	state->app.display_started=display_started;
	state->app.display_finished=display_finished;


	state->world = vx_world_create();
	state->layers = zhash_create(sizeof(vx_display_t*), sizeof(vx_layer_t*), zhash_ptr_hash, zhash_ptr_equals);

	pthread_mutex_init (&state->mutex, NULL);

	return state;
}


void * render_loop(void * data)
{
	state_t * state = (state_t *) data;
	int i = 1;
	while(state->running) {

		usleep(2500);
	if(i==250){
		i=-250;
	}

	vx_object_t *left_comm= vxo_chain(  vxo_mat_translate3((i/10)*0.5, 0, 0),
									vxo_mat_rotate_z(M_PI/2.0),
									vxo_mat_scale2(5,i/10), 
									vxo_rect(vxo_mesh_style(vx_blue)) );
	vx_object_t *right_comm= vxo_chain(  vxo_mat_translate3((i/10)*0.5, 10, 0),
									vxo_mat_rotate_z(M_PI/2.0),
									vxo_mat_scale2(5,i/10), 
									vxo_rect(vxo_mesh_style(vx_red)) );
	 vx_object_t *left_tick= vxo_chain(  vxo_mat_translate3((i/10)*0.5, 20, 0),
									vxo_mat_rotate_z(M_PI/2.0),
									vxo_mat_scale2(5,i/10), 
									vxo_rect(vxo_mesh_style(vx_orange)) );
	  vx_object_t *right_tick= vxo_chain(  vxo_mat_translate3((i/10)*0.5, 30, 0),
									vxo_mat_rotate_z(M_PI/2.0),
									vxo_mat_scale2(5,i/10), 
									vxo_rect(vxo_mesh_style(vx_green)) );
	   vx_object_t *imu= vxo_chain(  vxo_mat_translate3((i/10)*0.5, 40, 0),
									vxo_mat_rotate_z(M_PI/2.0),
									vxo_mat_scale2(5,i/10), 
									vxo_rect(vxo_mesh_style(vx_purple)) );
	vx_buffer_add_back(vx_world_get_buffer(state->world,"test"),left_comm);
	vx_buffer_add_back(vx_world_get_buffer(state->world,"test"),right_comm);
	 vx_buffer_add_back(vx_world_get_buffer(state->world,"test"),right_tick);
	  vx_buffer_add_back(vx_world_get_buffer(state->world,"test"),left_tick);
	   vx_buffer_add_back(vx_world_get_buffer(state->world,"test"),imu);
	vx_buffer_swap(vx_world_get_buffer(state->world,"test"));

	++i;
	}

	return NULL;
}

int main(int argc, char ** argv)
{
	vx_global_init(); // Call this to initialize the vx-wide lock. Required to start the GL thread or to use the program library

	state_t * state = state_create();


	draw(state, state->world);

	pthread_create(&state->animate_thread, NULL, render_loop, state);

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

	gtk_main (); // Blocks as long as GTK window is open
	gdk_threads_leave ();

	vx_gtk_display_source_destroy(appwrap);


	pthread_join(state->animate_thread, NULL);
	// vx_remote_display_source_destroy(cxn);

	state_destroy(state);
	vx_global_destroy();
}
