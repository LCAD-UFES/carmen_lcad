#ifndef TRAJECTORIES_H
#define TRAJECTORIES_H

#include <cstdio>
#include <cstdlib>
#include <vector>
#include <string>
#include <map>
using namespace std;

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    map<string ,double> TrajectoryNode_map;
}TrajectoryNode;

typedef struct {
    /* data */
    map<string ,double> TimeInterpolants_map;
}TimeInterpolants;

typedef struct {
    map<string ,double> SpatialInterpolants_map;
}SpatialInterpolants;

typedef struct 
{
    vector<double> t;         // time (s)
    vector<double> s;         // longitudinal distance along path (m)
    vector<double> V;         // longitudinal velocity (m/s)
    vector<double> A;         // longitudinal acceleration (m/s^2)
    vector<double> E;         // E coordinate (m)
    vector<double> N;         // N coordinate (m)
    vector<double> head;         // heading (rad)
    vector<double> curv;         // curvature (1/m)
    vector<double> gra;         // grade (rad)
    vector<double> bank;         // bank (rad)
    vector<double> edge_L;    // left lateral deviation bound (m)
    vector<double> edge_R;    // right lateral deviation bound (m)
} TrajectoryTube;


TrajectoryNode construct_TrajectoryNode();

int searchsortedfirst (vector<double> t, double num);

int clamp (int x, int lo , int hi);

void  t_function (TrajectoryTube traj, double t);

void Base_getindex(TrajectoryTube traj, double s);

#ifdef __cplusplus
}
#endif

#endif