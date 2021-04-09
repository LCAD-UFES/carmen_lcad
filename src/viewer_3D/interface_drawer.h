#ifndef INTERFACE_DRAWER_H_
#define INTERFACE_DRAWER_H_

#define SHOW_PATH_PLANS_FLAG_CODE		35
#define PLAN_TREE_FLAG_CODE				36
#define WAYPOINTS_CODE					37

typedef struct interface_drawer interface_drawer;

interface_drawer* create_interface_drawer(int window_width, int window_height);
void destroy_interface_drawer(interface_drawer* i_drawer);
void interface_mouse_func(interface_drawer* i_drawer, int type, int button, int x, int y, int window_height);
void draw_interface(interface_drawer* i_drawer, int window_width, int window_height);

#endif
