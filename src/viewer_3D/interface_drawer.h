#ifndef INTERFACE_DRAWER_H_
#define INTERFACE_DRAWER_H_

#define SHOW_PATH_PLANS_FLAG_CODE		35
#define PLAN_TREE_FLAG_CODE				36


typedef struct interface_drawer interface_drawer;

interface_drawer* create_interface_drawer(void);
void destroy_interface_drawer(interface_drawer* i_drawer);
void interface_mouse_func(interface_drawer* i_drawer, int type, int button, int x, int y);
void draw_interface(interface_drawer* i_drawer);

#endif
