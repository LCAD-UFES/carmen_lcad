/*
 * tree.h
 *
 *  Created on: 13/03/2012
 *      Author: romulo
 */

#ifndef TREE_H_
#define TREE_H_

struct Tree
{
	// The tree is composed of branches represented by connected vertice pairs. All branches start in the robot.
	int	num_edges;
	carmen_robot_and_trailers_traj_point_t *p1;
	carmen_robot_and_trailers_traj_point_t *p2;
	int *mask;

	// Zero or more branches of the tree might be represented as paths as well.
	carmen_robot_and_trailers_traj_point_t **paths;
	int *paths_sizes;
	int num_paths;
};

#endif /* TREE_H_ */
