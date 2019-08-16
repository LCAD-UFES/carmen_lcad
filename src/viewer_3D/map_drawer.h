#ifndef MAP_DRAWER_H_
#define MAP_DRAWER_H_

typedef struct map_drawer map_drawer;

map_drawer* create_map_drawer(void);
void destroy_map_drawer(map_drawer* m_drawer);
void add_map_message(map_drawer* m_drawer, carmen_map_t map);
void add_map_level1_message(map_drawer* m_drawer, carmen_mapper_map_message *message);
void draw_map(map_drawer* m_drawer, carmen_vector_3D_t offset);

#endif
