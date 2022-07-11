#ifndef MAP_DRAWER_H_
#define MAP_DRAWER_H_

struct map_drawer
{
	double map_grid_resolution;

	GLuint* vertex_buffer_ids;
	GLuint* lvl1_vertex_buffer_ids;
	GLuint* color_buffer_ids;
	int* buffer_sizes;
	int* lvl1_buffer_sizes;

	carmen_map_t* maps;
	carmen_map_t* lvl1_maps;
	int max_num_maps;
	int next_map;
};

typedef struct map_drawer map_drawer;

map_drawer* create_map_drawer(int argc, char** argv);
void destroy_map_drawer(map_drawer* m_drawer);
void add_map_message(map_drawer* m_drawer, carmen_mapper_map_message *message,
		carmen_vector_3D_t offset, carmen_pose_3D_t car_fused_pose, double map_zoom);
//void add_map_level1_message(map_drawer* m_drawer, carmen_mapper_map_message *message,
//		carmen_vector_3D_t offset, carmen_pose_3D_t car_fused_pose, double map_zoom);
void add_compact_cost_map_message(map_drawer* m_drawer, carmen_map_t map,
		carmen_vector_3D_t offset, carmen_pose_3D_t car_fused_pose, double map_zoom);
void add_localize_map_message(map_drawer* m_drawer, carmen_map_server_localize_map_message *message,
		carmen_vector_3D_t offset, carmen_pose_3D_t car_fused_pose, double map_zoom);
void draw_map(map_drawer *m_drawer, carmen_vector_3D_t offset, carmen_pose_3D_t car_fused_pose, double map_zoom);

#endif
