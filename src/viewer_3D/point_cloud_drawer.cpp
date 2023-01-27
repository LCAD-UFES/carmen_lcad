#include <carmen/carmen.h>

#include <GL/glew.h>
#include <GL/glut.h>
#include <GL/glu.h>

#include "point_cloud_drawer.h"
#include "viewer_3D.h"

struct point_cloud_drawer
{
	GLuint* vertex_buffer_ids;
	GLuint* color_buffer_ids;
	int* buffer_sizes;
	int num_buffers;
	int next_buffer;
};


point_cloud_drawer* create_point_cloud_drawer(int max_size)
{
	point_cloud_drawer* drawer = (point_cloud_drawer*)malloc(sizeof(point_cloud_drawer));

	drawer->num_buffers = max_size;
	drawer->next_buffer = 0;
	drawer->vertex_buffer_ids = (GLuint*)malloc((drawer->num_buffers) * sizeof(GLuint));
	drawer->color_buffer_ids = (GLuint*)malloc((drawer->num_buffers) * sizeof(GLuint));
	drawer->buffer_sizes = (int*)malloc((drawer->num_buffers) * sizeof(int));

	int i;
	for (i = 0; i<drawer->num_buffers; i++)
	{
		drawer->buffer_sizes[i] = 0;
	}

	glGenBuffers(drawer->num_buffers, drawer->vertex_buffer_ids);
	glGenBuffers(drawer->num_buffers, drawer->color_buffer_ids);
	
	return drawer;
}

static float* create_colors(point_cloud pcloud)
{
	float* colors = (float*)malloc(3 * pcloud.num_points * sizeof(float));

	int i;
	for (i = 0; i < pcloud.num_points; i++)
	{
		//double x = pcloud.points[i].x;
		//double y = pcloud.points[i].y;
/*		double z = pcloud.points[i].z - pcloud.car_position.z;

		if(z < -5.0)
		{
			colors[3*i] = 0.0;			
			colors[3*i+1] = 0.0;
			colors[3*i+2] = 0.0;
		}
		else
		{
			colors[3*i] = 0.0 - z;			
			colors[3*i+1] = 0.1 + z/10.0;
			colors[3*i+2] = (z+3.0)/6.0;
		}
*/
//		if (pcloud.point_color != NULL)
//		{
			colors[3*i] = carmen_clamp(0.0, pcloud.point_color[i].x, 1.0);
			colors[3*i+1] = carmen_clamp(0.0, pcloud.point_color[i].y, 1.0);
			colors[3*i+2] = carmen_clamp(0.0, pcloud.point_color[i].z, 1.0);
//		}
//		else
//		{
//			colors[3 * i + 0] = 1.0;
//			colors[3 * i + 1] = 1.0;
//			colors[3 * i + 2] = 1.0;
//		}
	}

	return colors;
}

void add_point_cloud(point_cloud_drawer* drawer, point_cloud pcloud)
{
	glBindBuffer(GL_ARRAY_BUFFER, drawer->vertex_buffer_ids[drawer->next_buffer]);
	glBufferData(GL_ARRAY_BUFFER, pcloud.num_points * sizeof(carmen_vector_3D_t), pcloud.points, GL_STATIC_DRAW);
	glFinish();
	glVertexPointer(3, GL_DOUBLE, 0, (char *) NULL);

	float *colors = create_colors(pcloud);

	glBindBuffer(GL_ARRAY_BUFFER, drawer->color_buffer_ids[drawer->next_buffer]);
	glBufferData(GL_ARRAY_BUFFER, 3 * pcloud.num_points * sizeof(float), colors, GL_STATIC_DRAW);
	glFinish();
	glColorPointer(3, GL_FLOAT, 0, (char *) NULL);

	free(colors);

	drawer->buffer_sizes[drawer->next_buffer] = pcloud.num_points;
	drawer->next_buffer = (drawer->next_buffer + 1) % (drawer->num_buffers);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
}


void draw_point_cloud(point_cloud_drawer* drawer)
{
	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(3);

	glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);	

	int i;
	for (i = 0; i < drawer->num_buffers; i++)
	{
		glBindBuffer(GL_ARRAY_BUFFER, drawer->vertex_buffer_ids[i]);
		glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, 0, 0);
		glBindBuffer(GL_ARRAY_BUFFER, drawer->color_buffer_ids[i]);
		glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 0, 0);
		glDrawArrays(GL_POINTS, 0, drawer->buffer_sizes[i]);
	}

	glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);

	glDisableVertexAttribArray(3);
	glDisableVertexAttribArray(0);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
}


void destroy_point_cloud_drawer(point_cloud_drawer* drawer)
{
	glDeleteBuffers(drawer->num_buffers, drawer->vertex_buffer_ids);
	glDeleteBuffers(drawer->num_buffers, drawer->color_buffer_ids);

	free(drawer->vertex_buffer_ids);
	free(drawer->color_buffer_ids);
	free(drawer->buffer_sizes);
	free(drawer);
}
