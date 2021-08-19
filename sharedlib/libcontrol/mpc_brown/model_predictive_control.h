#ifndef MODEL_PREDICTIVE_CONTROL_H
#define MODEL_PREDICTIVE_CONTROL_H


#include <cstdio>
#include <cstdlib>
#include <vector>
#include <string>
#include <map>
#include "decoupled_lat_long.h"
#include "coupled_lat_long.h"
#include "trajectories.h"
#include "vehicle_dynamics.h"

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
    std::vector<double> prev_ts;
    std::vector<double>  ts;
    std::vector<double> dt;
}MPCTimeSteps;

MPCTimeSteps
make_MPCTimeSteps(int N_short, int N_long, double dt_short, double dt_long, bool use_correction_step);

MPCTimeSteps
compute_time_steps(MPCTimeSteps TS, double t0);

typedef struct
{
    map<string, double> vehicle;
    TrajectoryTube trajectory;
    VehicleModel dynamics;
    BicycleState current_state;
    BicycleControl current_control;
    CoupledControlParams control_params;
    int heartbeat;
    double time_offset;
    MPCTimeSteps time_steps;
    bool solved;
    VehicleModel tracking_dynamics;
    vector<TrackingBicycleState> qs;
    vector<BicycleControl2> us;
    vector<TrackingBicycleParams>  ps;
}TrajectoryTrackingMPC;

typedef struct
{
    map<string, double> vehicle;
    TrajectoryTube trajectory;
    VehicleModel dynamics;
    BicycleState current_state;
    BicycleControl current_control;
    DecoupledControlParams control_params;
    int heartbeat;
    double time_offset;
    MPCTimeSteps time_steps;
    bool solved;
    VehicleModel tracking_dynamics;
    vector<LateralTrackingBicycleState> qs;
    vector<BicycleControl2> us;
    vector<LateralTrackingBicycleParams>  ps;
}LateralTrajectoryTrackingMPC;


TrajectoryTrackingMPC 
make_TrajectoryTrackingMPC(TrajectoryTube trajectory,VehicleModel dynamics,CoupledControlParams control_params,
                          BicycleState current_state, BicycleControl current_control, int heartbeat, double time_offset ,
                          MPCTimeSteps time_steps,
                          vector<TrackingBicycleState> qs, vector<BicycleControl2> us,vector<TrackingBicycleParams>  ps);

LateralTrajectoryTrackingMPC 
make_TrajectoryTrackingMPC_lateral(TrajectoryTube trajectory,VehicleModel dynamics,DecoupledControlParams control_params,
                          BicycleState current_state, BicycleControl current_control, int heartbeat, double time_offset ,
                          MPCTimeSteps time_steps,
                          vector<LateralTrackingBicycleState> qs, vector<BicycleControl2> us,vector<LateralTrackingBicycleParams>  ps);                    

#ifdef __cplusplus
}
#endif

#endif