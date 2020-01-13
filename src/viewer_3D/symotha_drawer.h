#ifndef SYMOTHA_DRAWER_H_
#define SYMOTHA_DRAWER_H_

#ifdef __cplusplus
extern "C" {
#endif

struct _symotha_parameters
{
	double central_lane_obstacles_safe_distance;
	double main_central_lane_obstacles_safe_distance;
	double lateral_lane_obstacles_safe_distance;
	double obstacles_safe_distance;
};

typedef struct _symotha_parameters symotha_parameters_t;


struct _symotha_drawer
{
	symotha_parameters_t symotha_params;
};

typedef struct _symotha_drawer symotha_drawer_t;

symotha_drawer_t *create_symotha_drawer(int argc, char **argv);
void destroy_symotha_drawer(symotha_drawer_t *symotha_drawer);
void draw_symotha(symotha_drawer_t *symotha_drawer, carmen_pose_3D_t car_fused_pose);

#ifdef __cplusplus
}
#endif

#endif
