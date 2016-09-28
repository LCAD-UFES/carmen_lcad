#ifndef STEHS_PLANNER_H
#define STEHS_PLANNER_H


#ifdef __cplusplus
extern "C" {
#endif


typedef struct
{

	carmen_robot_ackerman_config_t robot_config;
	unsigned int show_debug_info;

} stehs_planner_config_t, *stehs_planner_config_p;


#ifdef __cplusplus
}
#endif

#endif
