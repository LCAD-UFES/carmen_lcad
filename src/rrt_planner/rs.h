/*
 * rs.h
 *
 *  Created on: 06/08/2012
 *      Author: romulo
 */

#ifndef RS_H_
#define RS_H_

typedef enum {
	RS_TURN_RIGHT, RS_TURN_LEFT, RS_STRAIGHT, RS_FWD, RS_BWD, RS_NONE
} RS_POSSIBLE_MOVES;

typedef struct {
	int turn;
	int move;
} rs_move;

typedef struct {
	rs_move *moves;
	double dist[5];
	int size;

} RS_Path_Description;

rs_move* rs_get_moves(int numero);

void rs_build_path_description(RS_Path_Description *path_desc, int numero, double tr, double ur, double vr);

int fct_curve(int ty, int orientation, double val, double *x1, double *y1, double *t1, double delta, double *pathx, double *pathy, double *patht, int n);

void rs_init_parameters(double max_phi, double distance_between_front_and_rear_axles);

double get_turning_radius();

double reed_shepp(double x1, double y1, double t1, double x2, double y2, double t2, int *numero, double *tr, double *ur, double *vr);

int constRS(int num, double t, double u, double v, double x1, double y1, double t1, double delta, double *pathx, double *pathy, double *patht);

int rs_get_path(int num, double t, double u, double v, double x1, double y1, double t1, double delta, double *pathx, double *pathy, double *patht, int *path_vector, int *path_size);
#endif /* RS_H_ */
