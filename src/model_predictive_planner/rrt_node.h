/*
 * rrt_node.h
 *
 *  Created on: 16/12/2011
 *      Author: rradaelli
 */

#ifndef RRT_NODE_H_
#define RRT_NODE_H_

#include "robot_state.h"
#include <map>
#include <list>
using namespace std;

class RRT_Edge;

class RRT_Node
{
public:
	RRT_Node();
	virtual ~RRT_Node();
	double		  distance(const RRT_Node &n);
	double		  distance(const Pose &p);
	bool		  add_command(Command &c, double &time);
	bool		  is_command_followed(Command &c, double &time); //verify if the command was already followed in this node
	bool operator ==(RRT_Node &node);
	bool operator !=(RRT_Node &node);
	unsigned long int get_key();
	static unsigned long int get_key(Robot_State& robot_state);
	void		  update_cvf(int i = 1);

	bool prune_node();

	/**
	 * If there is any edge with this node, remove it
	 */
	void remove_edge(RRT_Node *node);

public:
	map<unsigned long int, Command> followedCommands;

public:
	map<unsigned long int, bool> obstacle_verification_cache;
	double cost;
	double cvf; //Constraint Violation Frequency
	bool closed;
	Robot_State	   robot_state;
	list<RRT_Edge> adjacency_nodes;
	list<RRT_Edge> prev_nodes;
	RRT_Node	 *parent;
	static double n; //used by cvf
	static int	  k; //number of updates of cvf value
};

class RRT_Edge
{
public:
	RRT_Edge(RRT_Node *n1 = 0, RRT_Node *n2 = 0, double cost = 1);

	bool operator ==(const RRT_Edge &node);
	bool operator !=(const RRT_Edge &node);

	RRT_Node *n1;
	RRT_Node *n2;
	double	  cost;
	double time;
	Command	  command;
};

typedef struct
{
public:
	Robot_State p1;
	Robot_State p2;
	Command	  command;
	double time;
} RRT_Path_Edge;


typedef struct {
	carmen_robot_and_trailers_traj_point_t p1, p2;
	double v, phi;
	double time;
} Edge_Struct;


typedef struct
{
	Edge_Struct *path;
	int size;
	carmen_point_t goal;
	int last_goal;
	double timestamp;
	char  *host;
} rrt_path_message;

#define RRT_PATH_NAME "rrt_path_message_name"
#define RRT_PATH_FMT "{<{{double, double, double, int, [double:5], double, double}, {double, double, double, int, [double:5], double, double}, double, double, double}:2>, int, {double, double, double}, int, double, string}"



#endif /* RRT_NODE_H_ */
