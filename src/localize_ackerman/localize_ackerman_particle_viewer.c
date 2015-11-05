#include <carmen/carmen.h>
#include <carmen/localize_ackerman_interface.h>
#include <gl_plotter.h>

gl_plotter* plotter1 = NULL;
gl_plotter* plotter2 = NULL;

double mean_x_pr = 0.0;
double mean_y_pr = 0.0;

static void
localize_ackerman_particle_prediction_handler(carmen_localize_ackerman_particle_message *particle_message) 
{ 	
	int num_particles = particle_message->num_particles;
	
	double* x_data = malloc(num_particles*sizeof(double));
	double* y_data = malloc(num_particles*sizeof(double));
	double* w_data = malloc(num_particles*sizeof(double));

	//double mean_x = particle_message->globalpos.x;
	//double mean_y = particle_message->globalpos.y;

	mean_x_pr = particle_message->globalpos.x;
	mean_y_pr = particle_message->globalpos.y;

	int i;
	for(i=0; i<num_particles; i++)
	{
		double dx = mean_x_pr - particle_message->particles[i].x;
		double dy = mean_y_pr - particle_message->particles[i].y;
		double w = particle_message->particles[i].weight;
		
		//x_data[i] = sqrt(dx*dx + dy*dy);
		//y_data[i] = w;

		x_data[i] = -dy;
		y_data[i] = dx;
		w_data[i] = w;		
		
	}

	//gl_plotter_draw_data(plotter1, x_data, y_data, num_particles);
	gl_plotter_draw_data_color_intensity(plotter1, x_data, y_data, w_data, num_particles);
	
	free(x_data);
	free(y_data);
	free(w_data);
}

static void
localize_ackerman_particle_correction_handler(carmen_localize_ackerman_particle_message *particle_message) 
{ 	
	int num_particles = particle_message->num_particles;
	
	double* x_data = malloc(num_particles*sizeof(double));
	double* y_data = malloc(num_particles*sizeof(double));
	double* w_data = malloc(num_particles*sizeof(double));

	//double mean_x = particle_message->globalpos.x;
	//double mean_y = particle_message->globalpos.y;
	
	int i;
	for(i=0; i<num_particles; i++)
	{
		double dx = mean_x_pr - particle_message->particles[i].x;
		double dy = mean_y_pr - particle_message->particles[i].y;
		double w = particle_message->particles[i].weight;

		//x_data[i] = sqrt(dx*dx + dy*dy);
		//y_data[i] = w;

		x_data[i] = -dy;
		y_data[i] = dx;
		w_data[i] = w;
		//printf("x:% lf, w:% e\n", x_data[i], w);
	}

	gl_plotter_draw_data_color_intensity(plotter2, x_data, y_data, w_data, num_particles);

	free(x_data);
	free(y_data);
	free(w_data);
}



static void
initialize(int argc, char** argv)
{
	carmen_ipc_initialize(argc, argv);
	carmen_param_check_version(argv[0]);

	//plotter1 = create_gl_plotter("Localizer Ackerman Particles Prediction", 600, 600, 0.0, 0.5, -0.1, 1.1);
	//plotter2 = create_gl_plotter("Localizer Ackerman Particles Correction", 600, 600, 0.0, 0.5, -0.1, 1.1);
	plotter1 = create_gl_plotter("Localizer Ackerman Particles Prediction", 600, 600, -3.5, 3.5, -3.5, 3.5);
	plotter2 = create_gl_plotter("Localizer Ackerman Particles Correction", 600, 600, -3.5, 3.5, -3.5, 3.5);

	
	carmen_localize_ackerman_subscribe_particle_prediction_message(	NULL,
								(carmen_handler_t)localize_ackerman_particle_prediction_handler,
								CARMEN_SUBSCRIBE_ALL);

	carmen_localize_ackerman_subscribe_particle_correction_message(	NULL,
								(carmen_handler_t)localize_ackerman_particle_correction_handler,
								CARMEN_SUBSCRIBE_ALL);

}

static void
finalize()
{
	destroy_gl_plotter(plotter1);
	destroy_gl_plotter(plotter2);
}

int
main(int argc, char** argv)
{
	initialize(argc, argv);
            
	carmen_ipc_dispatch();	                                                             
	
	carmen_ipc_disconnect();

	finalize();

	return 0;
}
