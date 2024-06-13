#include <carmen/carmen.h>
#include <carmen/mapper_interface.h>

#include <GL/glew.h>
#include <GL/glut.h>
#include <GL/glu.h>

#include "map_drawer.h"

int drawVBO = 0;


map_drawer*
create_map_drawer(int argc, char** argv)
{
	map_drawer* m_drawer = (map_drawer*)malloc(sizeof(map_drawer));

	int num_items;

	carmen_param_t param_list[] =
	{
		{"mapper", "map_grid_res", CARMEN_PARAM_DOUBLE, &(m_drawer->map_grid_resolution), 0, NULL},
	};

	num_items = sizeof(param_list)/sizeof(param_list[0]);
	carmen_param_install_params(argc, argv, param_list, num_items);

	m_drawer->max_num_maps = 1;
	m_drawer->next_map = 0;

	m_drawer->maps = (carmen_map_t*)malloc(m_drawer->max_num_maps * sizeof(carmen_map_t));
	m_drawer->lvl1_maps = (carmen_map_t*)malloc(m_drawer->max_num_maps * sizeof(carmen_map_t));
	m_drawer->vertex_buffer_ids = (GLuint*)malloc((m_drawer->max_num_maps) * sizeof(GLuint));
	m_drawer->lvl1_vertex_buffer_ids = (GLuint*)malloc((m_drawer->max_num_maps) * sizeof(GLuint));
	m_drawer->color_buffer_ids = (GLuint*)malloc((m_drawer->max_num_maps) * sizeof(GLuint));
	m_drawer->buffer_sizes = (int*)malloc((m_drawer->max_num_maps) * sizeof(int));
	m_drawer->lvl1_buffer_sizes = (int*)malloc((m_drawer->max_num_maps) * sizeof(int));

	int i;
	for(i=0; i < m_drawer->max_num_maps; i++)
	{
		m_drawer->buffer_sizes[i] = 0;
		m_drawer->lvl1_buffer_sizes[i] = 0;
		m_drawer->maps[i].config.x_size = 0;
		m_drawer->maps[i].config.y_size = 0;
		m_drawer->lvl1_maps[i].config.x_size = 0;
		m_drawer->lvl1_maps[i].config.y_size = 0;
	}

	glGenBuffers(m_drawer->max_num_maps, m_drawer->vertex_buffer_ids);
	glGenBuffers(m_drawer->max_num_maps, m_drawer->lvl1_vertex_buffer_ids);
	glGenBuffers(m_drawer->max_num_maps, m_drawer->color_buffer_ids);

	return m_drawer;
}


void
destroy_map_drawer(map_drawer* m_drawer)
{
	glDeleteBuffers(m_drawer->max_num_maps, m_drawer->vertex_buffer_ids);
	glDeleteBuffers(m_drawer->max_num_maps, m_drawer->lvl1_vertex_buffer_ids);
	glDeleteBuffers(m_drawer->max_num_maps, m_drawer->color_buffer_ids);

	free(m_drawer->vertex_buffer_ids);
	free(m_drawer->lvl1_vertex_buffer_ids);
	free(m_drawer->color_buffer_ids);
	free(m_drawer->buffer_sizes);
	free(m_drawer->lvl1_buffer_sizes);

	free(m_drawer->maps);
	free(m_drawer->lvl1_maps);

	free(m_drawer);
}

static int getTotalObstacles(carmen_map_t map, double car_x, double car_y, double zoom)
{
	int total = 0;
	double resolution = map.config.resolution;

	for (int i = 0; i < map.config.x_size; i++)
	{
		for (int j = 0; j < map.config.y_size; j++)
		{
			double x = i * resolution;
			double y = j * resolution;
			double distance = sqrt((car_x - x) * (car_x - x) + (car_y - y) * (car_y - y));
			if (distance < ((double) map.config.x_size * resolution * zoom * 3.0))
			{
				double map_value = map.complete_map[i * map.config.x_size + j];

				if (map_value > 0.5)
					total++;
			}
		}
	}

	return (total);
}

static void fillCubeData(float* vd, int offset, float x, float y, float z, float x_size, float y_size, float z_size)
{	
	float length_x = x_size / 2;
	float length_y = y_size / 2;
	float length_z = z_size / 2;

	int k = offset;
	vd[k+0] = 0.0f; 	vd[k+1] = 0.0f; 	vd[k+2] = -1.0f;	
	vd[k+6] = 0.0f; 	vd[k+7] = 0.0f; 	vd[k+8] = -1.0f;	
	vd[k+12] = 0.0f; 	vd[k+13] = 0.0f; 	vd[k+14] = -1.0f;
	vd[k+18] = 0.0f; 	vd[k+19] = 0.0f; 	vd[k+20] = -1.0f;
	vd[k+3] = x-length_x; 	vd[k+4] = y-length_y; 	vd[k+5] = z-length_z;	
	vd[k+9] = x+length_x; 	vd[k+10] = y-length_y; 	vd[k+11] = z-length_z;	
	vd[k+15] = x+length_x; 	vd[k+16] = y+length_y; 	vd[k+17] = z-length_z;	
	vd[k+21] = x-length_x; 	vd[k+22] = y+length_y; 	vd[k+23] = z-length_z;
	
	k += 24;
	vd[k+0] = 0.0f; 	vd[k+1] = 0.0f; 	vd[k+2] = 1.0f;	
	vd[k+6] = 0.0f; 	vd[k+7] = 0.0f; 	vd[k+8] = 1.0f;	
	vd[k+12] = 0.0f; 	vd[k+13] = 0.0f; 	vd[k+14] = 1.0f;
	vd[k+18] = 0.0f; 	vd[k+19] = 0.0f; 	vd[k+20] = 1.0f;
	vd[k+3] = x-length_x; 	vd[k+4] = y-length_y; 	vd[k+5] = z+length_z;	
	vd[k+9] = x+length_x; 	vd[k+10] = y-length_y; 	vd[k+11] = z+length_z;	
	vd[k+15] = x+length_x; 	vd[k+16] = y+length_y; 	vd[k+17] = z+length_z;	
	vd[k+21] = x-length_x; 	vd[k+22] = y+length_y; 	vd[k+23] = z+length_z;

	k += 24;
	vd[k+0] = 1.0f; 	vd[k+1] = 0.0f; 	vd[k+2] = 0.0f;	
	vd[k+6] = 1.0f; 	vd[k+7] = 0.0f; 	vd[k+8] = 0.0f;	
	vd[k+12] = 1.0f; 	vd[k+13] = 0.0f; 	vd[k+14] = 0.0f;
	vd[k+18] = 1.0f; 	vd[k+19] = 0.0f; 	vd[k+20] = 0.0f;
	vd[k+3] = x+length_x; 	vd[k+4] = y-length_y; 	vd[k+5] = z+length_z;	
	vd[k+9] = x+length_x; 	vd[k+10] = y-length_y; 	vd[k+11] = z-length_z;	
	vd[k+15] = x+length_x; 	vd[k+16] = y+length_y; 	vd[k+17] = z-length_z;	
	vd[k+21] = x+length_x; 	vd[k+22] = y+length_y; 	vd[k+23] = z+length_z;

	k += 24;
	vd[k+0] = -1.0f; 	vd[k+1] = 0.0f; 	vd[k+2] = 0.0f;	
	vd[k+6] = -1.0f; 	vd[k+7] = 0.0f; 	vd[k+8] = 0.0f;	
	vd[k+12] = -1.0f; 	vd[k+13] = 0.0f; 	vd[k+14] = 0.0f;
	vd[k+18] = -1.0f; 	vd[k+19] = 0.0f; 	vd[k+20] = 0.0f;
	vd[k+3] = x-length_x; 	vd[k+4] = y-length_y; 	vd[k+5] = z+length_z;	
	vd[k+9] = x-length_x; 	vd[k+10] = y-length_y; 	vd[k+11] = z-length_z;	
	vd[k+15] = x-length_x; 	vd[k+16] = y+length_y; 	vd[k+17] = z-length_z;	
	vd[k+21] = x-length_x; 	vd[k+22] = y+length_y; 	vd[k+23] = z+length_z;

	k += 24;
	vd[k+0] = 0.0f; 	vd[k+1] = 1.0f; 	vd[k+2] = 0.0f;	
	vd[k+6] = 0.0f; 	vd[k+7] = 1.0f; 	vd[k+8] = 0.0f;	
	vd[k+12] = 0.0f; 	vd[k+13] = 1.0f; 	vd[k+14] = 0.0f;
	vd[k+18] = 0.0f; 	vd[k+19] = 1.0f; 	vd[k+20] = 0.0f;
	vd[k+3] = x-length_x; 	vd[k+4] = y+length_y; 	vd[k+5] = z+length_z;	
	vd[k+9] = x-length_x; 	vd[k+10] = y+length_y; 	vd[k+11] = z-length_z;	
	vd[k+15] = x+length_x; 	vd[k+16] = y+length_y; 	vd[k+17] = z-length_z;	
	vd[k+21] = x+length_x; 	vd[k+22] = y+length_y; 	vd[k+23] = z+length_z;
	
	k += 24;		
	vd[k+0] = 0.0f; 	vd[k+1] = -1.0f; 	vd[k+2] = 0.0f;	
	vd[k+6] = 0.0f; 	vd[k+7] = -1.0f; 	vd[k+8] = 0.0f;	
	vd[k+12] = 0.0f; 	vd[k+13] = -1.0f; 	vd[k+14] = 0.0f;
	vd[k+18] = 0.0f; 	vd[k+19] = -1.0f; 	vd[k+20] = 0.0f;
	vd[k+3] = x-length_x; 	vd[k+4] = y-length_y; 	vd[k+5] = z+length_z;	
	vd[k+9] = x-length_x; 	vd[k+10] = y-length_y; 	vd[k+11] = z-length_z;	
	vd[k+15] = x+length_x; 	vd[k+16] = y-length_y; 	vd[k+17] = z-length_z;	
	vd[k+21] = x+length_x; 	vd[k+22] = y-length_y; 	vd[k+23] = z+length_z;
}

static float *create_vertex_data(carmen_map_t map, int total_size, double car_x, double car_y, double zoom)
{
	float *vd = (float *) malloc(total_size * 6 * 4 * 2 * 3 * sizeof(float));

	double resolution = map.config.resolution;
	int pos = 0;
	for (int i = 0; i < map.config.x_size; i++)
	{
		for (int j = 0; j < map.config.y_size; j++)
		{
			double x = i * resolution;
			double y = j * resolution;
			double z = 0.0;
			double distance = sqrt((car_x - x) * (car_x - x) + (car_y - y) * (car_y - y));
			if (distance < ((double) map.config.x_size * resolution * zoom * 3.0))
			{
				double map_value = map.complete_map[i * map.config.x_size + j];
				if (map_value > 0.5)
				{
					fillCubeData(vd, pos, x, y, z, resolution, resolution, 0.5);
					pos += (6 * 4 * 2 * 3);
				}
			}
		}
	}

	return vd;
}

void
add_map_message(map_drawer* m_drawer, carmen_mapper_map_message *message,
		carmen_vector_3D_t offset, carmen_pose_3D_t car_fused_pose, double map_zoom)
{
	carmen_map_t map;

	map.complete_map = message->complete_map;
	map.config = message->config;

	m_drawer->maps[m_drawer->next_map] = map;

	double offset_x = map.config.x_origin - offset.x;
	double offset_y = map.config.y_origin - offset.y;
	double car_x = car_fused_pose.position.x - offset_x;
	double car_y = car_fused_pose.position.y - offset_y;

	glBindBuffer(GL_ARRAY_BUFFER, m_drawer->vertex_buffer_ids[m_drawer->next_map]);
	if(drawVBO)
	{
		int total_size = getTotalObstacles(map, car_x, car_y, map_zoom);
		float* vertex_data = create_vertex_data(map, total_size, car_x, car_y, map_zoom);
		glBufferData(GL_ARRAY_BUFFER, total_size * (6 * 4 * 2 * 3) * sizeof(float), vertex_data, GL_STATIC_DRAW);
		free(vertex_data);
		m_drawer->buffer_sizes[m_drawer->next_map] = total_size * 6 * 4;
	}
	glBindBuffer(GL_ARRAY_BUFFER,0);
}

// Remission map test @@@Braian
//void
//add_remission_map_message(map_drawer* m_drawer, carmen_map_server_localize_map_message *message,
//		carmen_vector_3D_t offset, carmen_pose_3D_t car_fused_pose, double map_zoom)
//{
//	carmen_map_t map;
//
//	map.complete_map = message->complete_mean_remission_map;
//	map.config = message->config;
//
//	m_drawer->maps[m_drawer->next_map] = map;
//
//	double offset_x = map.config.x_origin - offset.x;
//	double offset_y = map.config.y_origin - offset.y;
//	double car_x = car_fused_pose.position.x - offset_x;
//	double car_y = car_fused_pose.position.y - offset_y;
//
//	glBindBuffer(GL_ARRAY_BUFFER, m_drawer->vertex_buffer_ids[m_drawer->next_map]);
//	if(drawVBO)
//	{
//		int total_size = getTotalObstacles(map, car_x, car_y, map_zoom);
//		float* vertex_data = create_vertex_data(map, total_size, car_x, car_y, map_zoom);
//		glBufferData(GL_ARRAY_BUFFER, total_size * (6 * 4 * 2 * 3) * sizeof(float), vertex_data, GL_STATIC_DRAW);
//		free(vertex_data);
//		m_drawer->buffer_sizes[m_drawer->next_map] = total_size * 6 * 4;
//	}
//	glBindBuffer(GL_ARRAY_BUFFER,0);
//}


void
add_map_level1_message(map_drawer* m_drawer, carmen_mapper_map_message *message,
		carmen_vector_3D_t offset, carmen_pose_3D_t car_fused_pose, double map_zoom)
{
	carmen_map_t map;

	map.complete_map = message->complete_map;
	map.config = message->config;

	m_drawer->lvl1_maps[m_drawer->next_map] = map;

	double offset_x = map.config.x_origin - offset.x;
	double offset_y = map.config.y_origin - offset.y;
	double car_x = car_fused_pose.position.x - offset_x;
	double car_y = car_fused_pose.position.y - offset_y;

	glBindBuffer(GL_ARRAY_BUFFER, m_drawer->lvl1_vertex_buffer_ids[m_drawer->next_map]);
	if (drawVBO)
	{
		int total_size = getTotalObstacles(map, car_x, car_y, map_zoom);
		float* vertex_data = create_vertex_data(map, total_size, car_x, car_y, map_zoom);
		glBufferData(GL_ARRAY_BUFFER, total_size * (6 * 4 * 2 * 3) * sizeof(float), vertex_data, GL_STATIC_DRAW);
		free(vertex_data);
		m_drawer->lvl1_buffer_sizes[m_drawer->next_map] = total_size * 6 * 4;
	}
	glBindBuffer(GL_ARRAY_BUFFER,0);
}

void
add_compact_cost_map_message(map_drawer* m_drawer, carmen_map_t map,
		carmen_vector_3D_t offset, carmen_pose_3D_t car_fused_pose, double map_zoom)
{
	m_drawer->maps[m_drawer->next_map] = map;

	double offset_x = map.config.x_origin - offset.x;
	double offset_y = map.config.y_origin - offset.y;
	double car_x = car_fused_pose.position.x - offset_x;
	double car_y = car_fused_pose.position.y - offset_y;

	glBindBuffer(GL_ARRAY_BUFFER, m_drawer->vertex_buffer_ids[m_drawer->next_map]);
	if (drawVBO)
	{
		int total_size = getTotalObstacles(map, car_x, car_y, map_zoom);
		float *vertex_data = create_vertex_data(map, total_size, car_x, car_y, map_zoom);
		glBufferData(GL_ARRAY_BUFFER, total_size * (6 * 4 * 2 * 3) * sizeof(float), vertex_data, GL_STREAM_DRAW);
		free(vertex_data);
		m_drawer->buffer_sizes[m_drawer->next_map] = total_size * 6 * 4;
	}
	m_drawer->next_map = (m_drawer->next_map + 1) % m_drawer->max_num_maps;

	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void
add_localize_map_message(map_drawer* m_drawer, carmen_map_server_localize_map_message *message,
		carmen_vector_3D_t offset, carmen_pose_3D_t car_fused_pose, double map_zoom)
{
	carmen_map_t map;

	map.complete_map = message->complete_map;
	map.config = message->config;

	m_drawer->maps[m_drawer->next_map] = map;

	double offset_x = map.config.x_origin - offset.x;
	double offset_y = map.config.y_origin - offset.y;
	double car_x = car_fused_pose.position.x - offset_x;
	double car_y = car_fused_pose.position.y - offset_y;

	glBindBuffer(GL_ARRAY_BUFFER, m_drawer->vertex_buffer_ids[m_drawer->next_map]);
	if(drawVBO)
	{
		int total_size = getTotalObstacles(map, car_x, car_y, map_zoom);
		float* vertex_data = create_vertex_data(map, total_size, car_x, car_y, map_zoom);
		glBufferData(GL_ARRAY_BUFFER, total_size * (6 * 4 * 2 * 3) * sizeof(float), vertex_data, GL_STATIC_DRAW);
		free(vertex_data);
		m_drawer->buffer_sizes[m_drawer->next_map] = total_size * 6 * 4;
	}
	glBindBuffer(GL_ARRAY_BUFFER,0);
}

//static void
//drawPlane(double length_x, double length_y, double length_z)
//{
//
//	glPushMatrix();
//
//	glBegin(GL_QUADS);
//
//	glNormal3f( 0.0f, 0.0f,1.0f);
//	glVertex3f(-length_x/2, -length_y/2, -length_z/2);
//	glNormal3f( 0.0f, 0.0f,1.0f);
//	glVertex3f(length_x/2, -length_y/2, -length_z/2);
//	glNormal3f( 0.0f, 0.0f,1.0f);
//	glVertex3f(length_x/2, length_y/2, -length_z/2);
//	glNormal3f( 0.0f, 0.0f,1.0f);
//	glVertex3f(-length_x/2, length_y/2, -length_z/2);
//	glNormal3f( 0.0f, -1.0f, 0.0f);
//
//	glEnd();
//
//	glPopMatrix();
//}


// Função que desenha o cubo completo, menos eficiente (original) @@@Braian
//static void
//drawBox(double length_x, double length_y, double length_z)
//{
//
//	glPushMatrix();
//
//		glBegin(GL_QUADS);
//
//
//			glNormal3f( 0.0f, 0.0f,-1.0f);	glVertex3f(-length_x/2, -length_y/2, -length_z/2);
//			glNormal3f( 0.0f, 0.0f,-1.0f);	glVertex3f(length_x/2, -length_y/2, -length_z/2);
//			glNormal3f( 0.0f, 0.0f,-1.0f);	glVertex3f(length_x/2, length_y/2, -length_z/2);
//			glNormal3f( 0.0f, 0.0f,-1.0f);	glVertex3f(-length_x/2, length_y/2, -length_z/2);
//
//
//			glNormal3f( 0.0f, 0.0f, 1.0f);	glVertex3f(-length_x/2, -length_y/2, length_z/2);
//			glNormal3f( 0.0f, 0.0f, 1.0f);	glVertex3f(length_x/2, -length_y/2, length_z/2);
//			glNormal3f( 0.0f, 0.0f, 1.0f);	glVertex3f(length_x/2, length_y/2, length_z/2);
//			glNormal3f( 0.0f, 0.0f, 1.0f);	glVertex3f(-length_x/2, length_y/2, length_z/2);
//
//
//			glNormal3f( 1.0f, 0.0f, 0.0f);	glVertex3f(length_x/2, -length_y/2, length_z/2);
//			glNormal3f( 1.0f, 0.0f, 0.0f);	glVertex3f(length_x/2, -length_y/2, -length_z/2);
//			glNormal3f( 1.0f, 0.0f, 0.0f);	glVertex3f(length_x/2, length_y/2, -length_z/2);
//			glNormal3f( 1.0f, 0.0f, 0.0f);	glVertex3f(length_x/2, length_y/2, length_z/2);
//
//
//			glNormal3f(-1.0f, 0.0f, 0.0f);	glVertex3f(-length_x/2, -length_y/2, length_z/2);
//			glNormal3f(-1.0f, 0.0f, 0.0f);	glVertex3f(-length_x/2, -length_y/2, -length_z/2);
//			glNormal3f(-1.0f, 0.0f, 0.0f);	glVertex3f(-length_x/2, length_y/2, -length_z/2);
//			glNormal3f(-1.0f, 0.0f, 0.0f);	glVertex3f(-length_x/2, length_y/2, length_z/2);
//
//
//			glNormal3f( 0.0f, 1.0f, 0.0f);	glVertex3f(-length_x/2, length_y/2, length_z/2);
//			glNormal3f( 0.0f, 1.0f, 0.0f);	glVertex3f(-length_x/2, length_y/2, -length_z/2);
//			glNormal3f( 0.0f, 1.0f, 0.0f);	glVertex3f(length_x/2, length_y/2, -length_z/2);
//			glNormal3f( 0.0f, 1.0f, 0.0f);	glVertex3f(length_x/2, length_y/2, length_z/2);
//
//
//			glNormal3f( 0.0f,-1.0f, 0.0f);	glVertex3f(-length_x/2, -length_y/2, length_z/2);
//			glNormal3f( 0.0f,-1.0f, 0.0f);	glVertex3f(-length_x/2, -length_y/2, -length_z/2);
//			glNormal3f( 0.0f,-1.0f, 0.0f);	glVertex3f(length_x/2, -length_y/2, -length_z/2);
//			glNormal3f( 0.0f,-1.0f, 0.0f);	glVertex3f(length_x/2, -length_y/2, length_z/2);
//
//
//		glEnd();
//
//	glPopMatrix();
//}


//static void
//draw_map_element(double x, double y, double z, double map_value, double resolution)
//{
//	if(map_value > 0.5)
//	{
//		glPushMatrix();
//
//			glTranslated(x, y, z);
//			glColor3d(1.0, 0, 0); 										// red
//			glColor3d(map_value, map_value, map_value); 				// gray scale (remission)
//			glColor3d(1 - map_value, 1 - map_value, 1 - map_value); 	// gray scale inverted (costs)
//			drawBox(resolution, resolution, 0.5);
//			drawBoxNew(resolution, resolution, 0.5);
//			drawPlane(resolution, resolution, 0.5);
//			drawPlane(x, y, 0.5);
//
//		glPopMatrix();
//	}
//}


//static void
//draw_single_map(carmen_map_t map, double car_x, double car_y, double zoom)
//{
//	glPushMatrix();
//
////		printf("Origin x:% lf, y:% lf\n", map.config.x_origin, map.config.y_origin);
//
//		double resolution = map.config.resolution;
//
//		for(int i = 0; i < map.config.x_size; i++)
//		{
//			for(int j = 0; j < map.config.y_size; j++)
//			{
//				double x = i * resolution;
//				double y = j * resolution;
//				double distance = sqrt((car_x - x) * (car_x - x) + (car_y - y) * (car_y - y));
//				if (distance < ((double) map.config.x_size * resolution * zoom * 3.0))
//				{
//					double z = 0.0;
//					double map_value = map.complete_map[i * map.config.x_size + j];
//					draw_map_element(x, y, z, map_value, resolution);
//				}
//				//printf("x:% lf, y:% lf, z:% lf\n", x, y, z);
//			}
//		}
//
//	glPopMatrix();
//}

static void
draw_single_map_optimized(carmen_map_t map, double car_x, double car_y, double zoom)
{
	double *positions;

	positions = (double *) malloc(map.config.x_size * map.config.y_size * 12 * sizeof(double));

	int num_elements = 0;

	glPushMatrix();

	//		printf("Origin x:% lf, y:% lf\n", map.config.x_origin, map.config.y_origin);

	double resolution = map.config.resolution;

	for(int i = 0; i < map.config.x_size; i++)
	{
		for(int j = 0; j < map.config.y_size; j++)
		{
			double x = i * resolution;
			double y = j * resolution;
			double distance = sqrt((car_x - x) * (car_x - x) + (car_y - y) * (car_y - y));
			if (distance < ((double) map.config.x_size * resolution * zoom * 3.0))
			{
				double z = 0.3;
				double map_value = map.complete_map[i * map.config.x_size + j];
				if(map_value > 0.5)
				{
					double size = resolution;
					positions[num_elements * 12] = x - size;
					positions[num_elements * 12 + 1] = y + size;
					positions[num_elements * 12 + 2] = -z;

					positions[num_elements * 12 + 3] = x - size;
					positions[num_elements * 12 + 4] = y - size;
					positions[num_elements * 12 + 5] = -z;

					positions[num_elements * 12 + 6] = x + size;
					positions[num_elements * 12 + 7] = y - size;
					positions[num_elements * 12 + 8] = -z;

					positions[num_elements * 12 + 9] = x + size;
					positions[num_elements * 12 + 10] = y + size;
					positions[num_elements * 12 + 11] = -z;

					num_elements++;
				}
			}
		}
	}

	glColor3d(1.0, 0, 0);
	glNormal3f( 0.0f, 0.0f,1.0f);

	glEnableClientState(GL_VERTEX_ARRAY);

	glVertexPointer(3, GL_DOUBLE, 3*sizeof(double), positions);

	glDrawArrays(GL_QUADS, 0, num_elements * 4);

	glDisableClientState(GL_VERTEX_ARRAY);

	glNormal3f( 0.0f, -1.0f, 0.0f);
	glPopMatrix();

	free(positions);
}

//static void
//draw_single_remission_map(carmen_map_t map, double car_x, double car_y, double zoom)
//{
//	double *positions;
//	double *colors;
//	double *normals;
//
//	positions = (double *) malloc(map.config.x_size * map.config.y_size * 12 * sizeof(double));
//	colors = (double *) malloc(map.config.x_size * map.config.y_size * 12 * sizeof(double));
//	normals = (double *) malloc(map.config.x_size * map.config.y_size * 12 * sizeof(double));
//
//	int num_elements = 0;
//
//	glPushMatrix();
//
//	//		printf("Origin x:% lf, y:% lf\n", map.config.x_origin, map.config.y_origin);
//
//	double resolution = map.config.resolution;
//
//	for(int i = 0; i < map.config.x_size; i++)
//	{
//		for(int j = 0; j < map.config.y_size; j++)
//		{
//			double x = i * resolution;
//			double y = j * resolution;
//			double distance = sqrt((car_x - x) * (car_x - x) + (car_y - y) * (car_y - y));
//			if (distance < ((double) map.config.x_size * resolution * zoom * 3.0))
//			{
//				double z = 0.5;
//				double map_value = map.complete_map[i * map.config.x_size + j];
//				if(map_value > 0.0)
//				{
//					positions[num_elements * 12] = x - 0.1;
//					positions[num_elements * 12 + 1] = y + 0.1;
//					positions[num_elements * 12 + 2] = -z;
//
//					positions[num_elements * 12 + 3] = x - 0.1;
//					positions[num_elements * 12 + 4] = y - 0.1;
//					positions[num_elements * 12 + 5] = -z;
//
//					positions[num_elements * 12 + 6] = x + 0.1;
//					positions[num_elements * 12 + 7] = y - 0.1;
//					positions[num_elements * 12 + 8] = -z;
//
//					positions[num_elements * 12 + 9] = x + 0.1;
//					positions[num_elements * 12 + 10] = y + 0.1;
//					positions[num_elements * 12 + 11] = z;
//
//					for(int i = 0; i < 12; i++)
//					{
//						colors[num_elements * 12 + i] = map_value;
//					}
//
//					for(int i = 0; i < 4; i++)
//					{
//						normals[num_elements * 12 + i * 3 + 0] = 0.0;
//						normals[num_elements * 12 + i * 3 + 1] = 0.0;
//						normals[num_elements * 12 + i * 3 + 2] = 1.0;
//					}
//
//					num_elements++;
//
//				}
//
//			}
//
//		}
//	}
//
//	glColor3d(1.0, 0, 0);
//
//	glEnableClientState(GL_VERTEX_ARRAY);
//	glEnableClientState(GL_COLOR_ARRAY);
//	glEnableClientState(GL_NORMAL_ARRAY);
//
//	glVertexPointer(3, GL_DOUBLE, 3*sizeof(double), positions);
//	glColorPointer(3, GL_DOUBLE, 3*sizeof(double), colors);
//	glNormalPointer(GL_DOUBLE, 3*sizeof(double), normals);
//
//	glDrawArrays(GL_QUADS, 0, num_elements * 4);
//
//	glDisableClientState(GL_VERTEX_ARRAY);
//	glDisableClientState(GL_COLOR_ARRAY);
//	glDisableClientState(GL_NORMAL_ARRAY);
//
//	glPopMatrix();
//
//	free(positions);
//	free(colors);
//	free(normals);
//}


static void
draw_map_VBO(map_drawer *m_drawer, carmen_vector_3D_t offset)
{
//	printf("Offset2 x:% lf, y:% lf, z:% lf\n", offset.x, offset.y, offset.z);

	glPushMatrix();

		glEnableVertexAttribArray(0);
		glEnableVertexAttribArray(2);

		int i;
		for (i = 0; i < m_drawer->max_num_maps; i++)
		{	
			glPushMatrix();

				double offset_x = m_drawer->maps[i].config.x_origin - offset.x;
				double offset_y = m_drawer->maps[i].config.y_origin - offset.y;
				double offset_z = 0.0;//-offset.z;

				glColor3d(1.0, 0.0, 0.0);
				glTranslated(offset_x, offset_y, offset_z);

				glBindBuffer(GL_ARRAY_BUFFER, m_drawer->vertex_buffer_ids[i]);
				glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *) 0);
				glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *) (3 * sizeof(float)));
				glDrawArrays(GL_QUADS, 0, m_drawer->buffer_sizes[i]);

			glPopMatrix();
			glPushMatrix();

				offset_x = m_drawer->maps[i].config.x_origin - offset.x;
				offset_y = m_drawer->maps[i].config.y_origin - offset.y;
				offset_z = 1.0;//-offset.z;

				glColor3d(0.0, 1.0, 0.0);
				glTranslated(offset_x, offset_y, offset_z);

				glBindBuffer(GL_ARRAY_BUFFER, m_drawer->lvl1_vertex_buffer_ids[i]);
				glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *) 0);
				glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *) (3 * sizeof(float)));
				glDrawArrays(GL_QUADS, 0, m_drawer->lvl1_buffer_sizes[i]);


			glPopMatrix();
		}	

		glDisableVertexAttribArray(2);
		glDisableVertexAttribArray(0);

		glBindBuffer(GL_ARRAY_BUFFER,0);

	glPopMatrix();
}


static void
draw_map_not_VBO(map_drawer* m_drawer, carmen_vector_3D_t offset, carmen_pose_3D_t car_fused_pose, double map_zoom)
{	
	int i;

	for (i = 0; i < m_drawer->max_num_maps; i++)
	{
		glPushMatrix();

			double offset_x = m_drawer->maps[i].config.x_origin - offset.x;
			double offset_y = m_drawer->maps[i].config.y_origin - offset.y;
			double offset_z = 0.0;//-offset.z;

			glTranslated(offset_x, offset_y, offset_z);

			double car_x = car_fused_pose.position.x - offset_x;
			double car_y = car_fused_pose.position.y - offset_y;
//			draw_single_map(m_drawer->maps[i], car_x, car_y, map_zoom);
			draw_single_map_optimized(m_drawer->maps[i], car_x, car_y, map_zoom);
//			draw_single_remission_map(m_drawer->maps[i], car_x, car_y, map_zoom); @@@Braian: Desenha o mapa de remission pela mensagem

			offset_x = m_drawer->maps[i].config.x_origin - offset.x;
			offset_y = m_drawer->maps[i].config.y_origin - offset.y;
			offset_z = 10.0;//-offset.z;

			glTranslated(offset_x, offset_y, offset_z);

//			draw_single_map(m_drawer->lvl1_maps[i], car_x, car_y, map_zoom); // @@@Braian: Aumenta o consumo de CPU do viewer_3D

		glPopMatrix();
	}
}


void
draw_map(map_drawer *m_drawer, carmen_vector_3D_t offset, carmen_pose_3D_t car_fused_pose, double map_zoom)
{
	glPushMatrix();

		if (drawVBO)
			draw_map_VBO(m_drawer, offset);
		else
			draw_map_not_VBO(m_drawer, offset, car_fused_pose, map_zoom);

	glPopMatrix();
}
