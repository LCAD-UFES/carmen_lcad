#ifndef GL_PLOTTER_H_
#define GL_PLOTTER_H_


typedef struct gl_plotter gl_plotter;

gl_plotter* create_gl_plotter(char* name, int window_width, int window_height, 
				double plot_min_x, double plot_max_x,
				double plot_min_y, double plot_max_y);


void destroy_gl_plotter(gl_plotter* plt);

void gl_plotter_draw_data(gl_plotter* plt, double* x_data, double* y_data, int data_length);
void gl_plotter_draw_data_color_intensity(gl_plotter* plt, double* x_data, double* y_data, double* color_intensity, int data_length);
int get_close_state(gl_plotter* plt);

#endif
