#ifndef MODEL_PREDICTIVE_CONTROL_H
#define MODEL_PREDICTIVE_CONTROL_H


#include <cstdio>
#include <cstdlib>
#include <vector>
#include <string>
#include <map>
#include "trajectories.h"
#include "vehicle_dynamics.h"
#include "coupled_lat_long.h"
using namespace std;

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    int N_short;
    int N_long;
    double dt_short;
    double dt_long;
    bool use_correction_step;
    vector<double> prev_ts;
    vector<double>  ts;
    vector<double> dt;
}MPCTimeSteps;

MPCTimeSteps
make_MPCTimeSteps(int N_short, int N_long, double dt_short, double dt_long, bool use_correction_step);

void
compute_time_steps(MPCTimeSteps TS, double t0);

typedef struct
{
    map<string, double> vehicle;
    TrajectoryTube trajectory;
    VehicleModel dynamics;
    BicycleState current_state;
    BicycleControl current_control;
    int heartbeat;
    double time_offset;
    MPCTimeSteps time_steps;
    bool solved;
    VehicleModel tracking_dynamics;
    vector<LateralTrackingBicycleState> qs;
    vector<BicycleControl2> us;
    vector<LateralTrackingBicycleParams> ps;
}TrajectoryTrackingMPC;

TrajectoryTrackingMPC make_TrajectoryTrackingMPC(TrajectoryTube trajectory,VehicleModel dynamics,CoupledControlParams control_params,
                          BicycleState current_state, BicycleControl current_control, int heartbeat , double time_offset,
                          MPCTimeSteps time_steps,
                          vector<LateralTrackingBicycleState> qs, vector<BicycleControl2> us,vector<LateralTrackingBicycleParams> ps);

#ifdef __cplusplus
}
#endif

#endif