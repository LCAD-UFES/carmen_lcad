#ifndef RS_H_
#define RS_H_
#include <carmen/carmen.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	RS_TURN_RIGHT, RS_TURN_LEFT, RS_STRAIGHT, RS_FWD, RS_BWD, RS_NONE
} RS_POSSIBLE_MOVES;

typedef struct {
	int turn;
	int move;
} rs_move;

rs_move* rs_get_moves(int numero);

int fct_curve(int ty, int orientation, double val, carmen_robot_and_trailer_traj_point_t *start, double delta, carmen_robot_and_trailer_traj_point_t *points, int n);

void rs_init_parameters(double max_phi, double distance_between_front_and_rear_axles);

double reed_shepp(carmen_robot_and_trailer_traj_point_t start, carmen_robot_and_trailer_traj_point_t goal, int *numero, double *tr, double *ur, double *vr);

int constRS(int num, double t, double u, double v, carmen_robot_and_trailer_traj_point_t start, carmen_robot_and_trailer_traj_point_t *points);

#ifdef __cplusplus
}
#endif

#endif /* RS_H_ */
