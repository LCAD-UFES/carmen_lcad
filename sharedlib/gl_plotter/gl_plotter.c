#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/glu.h>

#include "gl_plotter.h"
#include "Window.h"

struct gl_plotter
{
	window* w;
	double plot_min_x;
	double plot_max_x;
	double plot_min_y;
	double plot_max_y;

	int close_window;
};

static void
init_gl(int width, int height)
{
	static int init = 1;
	int zero = 0;

	if(init)
	{
		
		glutInit(&zero, NULL);
		glewInit();
		init = 0;
	}

	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClearDepth(1.0);
	
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluPerspective(45.0f, (double)width / (double)height, 0.1f, 4000.0f);	// Calculate The Aspect Ratio Of The Window
	
	glMatrixMode(GL_MODELVIEW);
}

gl_plotter*
create_gl_plotter(char* name, int window_width, int window_height, double plot_min_x, double plot_max_x, double plot_min_y, double plot_max_y)
{
	gl_plotter* plt = malloc(sizeof(gl_plotter));

	plt->w = initWindow(name, window_width, window_height);
	plt->plot_min_x = plot_min_x;
	plt->plot_max_x = plot_max_x;
	plt->plot_min_y = plot_min_y;
	plt->plot_max_y = plot_max_y;
	plt->close_window = 0;

	init_gl(window_width, window_height);

	return plt;
}

void
destroy_gl_plotter(gl_plotter* plt)
{
	destroyWindow(plt->w);

	free(plt);
}

static void
mouseFunc(int type, int button, int x, int y)
{
	type = type;
	button = button;
	x = x;
	y = y;
	//printf("mouse - type: %d, button: %d, x: %d, y: %d\n", type, button, x, y);
}

static void
keyPress(int code)
{
	code = code;
	//printf("key press: %d, type: press\n", code);
}

static void
keyRelease(int code)
{
	code = code;
	//printf("key release: %d, type: press\n", code);
}

static void
initialize_drawing(gl_plotter* plt)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glDisable(GL_DEPTH_TEST);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(plt->plot_min_x, plt->plot_max_x, plt->plot_min_y, plt->plot_max_y);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

static void
draw_points(double* x, double* y, int size)
{
	glPointSize(3.0);
	glColor3f(0.0, 0.0, 0.0);

	glBegin(GL_POINTS);

		int i;
		for(i=0; i<size; i++)
		{		
			glVertex2f(x[i], y[i]);
		}

	glEnd();
}

static void
draw_points_intensity(double* x, double* y, double* color_intensity, int size)
{
	glPointSize(3.0);

	glBegin(GL_POINTS);

		int i;
		for(i=0; i<size; i++)
		{	
			double color = color_intensity[i];
			glColor3f(1.0-color, color, 0.0);	
			glVertex2f(x[i], y[i]);
		}

	glEnd();
}

void
gl_plotter_draw_data(gl_plotter* plt, double* x_data, double* y_data, int data_length)
{
	makeWindowCurrent(plt->w);

	initialize_drawing(plt);

	draw_points(x_data, y_data, data_length);

	showWindow(plt->w);

	plt->close_window = !processWindow(plt->w, mouseFunc, keyPress, keyRelease);
}

void
gl_plotter_draw_data_color_intensity(gl_plotter* plt, double* x_data, double* y_data, double* color_intensity, int data_length)
{
	makeWindowCurrent(plt->w);

	initialize_drawing(plt);

	draw_points_intensity(x_data, y_data, color_intensity, data_length);

	showWindow(plt->w);

	plt->close_window = !processWindow(plt->w, mouseFunc, keyPress, keyRelease);
}

int
get_close_state(gl_plotter* plt)
{
	return plt->close_window;
}
