#ifndef TRAJECTORIES_H
#define TRAJECTORIES_H

#include <cstdio>
#include <cstdlib>
#include <vector>
#include <string>
#include <map>
#include "vehicle_dynamics.h"
using namespace std;

#ifdef __cplusplus
extern "C" {
#endif

typedef struct{
    vector<double> E;
    vector<double> N;
    vector<double> phi;
    vector<double> k;
    vector<double> theta;
    vector<double> psi;
    vector<double> edge_L;
    vector<double> edge_R;
}SpatialInterpolants;

typedef struct {
    /* data */
    map<string ,double> TimeInterpolants_map;
}TimeInterpolants;

typedef struct {
    double t;
    TimeInterpolants ti;
    SpatialInterpolants si;
}TrajectoryNode;


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

TrajectoryNode
Traj_getindex_s(TrajectoryTube traj, double s);

int searchsortedfirst (vector<double> t, double num);

//int clamp (int x, int lo , int hi);

void  t_function (TrajectoryTube traj, double t);

void Base_getindex(TrajectoryTube traj, double s);

vector<vector<double>>
path_coordinates(TrajectoryTube traj, BicycleState x);

#ifdef __cplusplus
}
#endif

#endif