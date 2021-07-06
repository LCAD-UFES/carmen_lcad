#include <cstdio>
#include <cmath>
#include <vector>
#include <string>
#include <map>
#include "trajectories.h"
#include "vehicle_dynamics.h"
#include "model_predictive_control.h"
using namespace std;

typedef struct { 
    map<string ,double> CoupledControlParams_map;
}CoupledControlParams;


void CoupledTrajectoryTrackingMPC(map<string ,double> vehicle,  TrajectoryTube trajectory, CoupledControlParams control_params,
                                     int N_short = 10, int N_long = 20, double dt_short = (0.01), double dt_long = (0.2), bool use_correction_step=true)
{
    VehicleModel dynamics;
    BicycleState current_state; //= zeros(BicycleState{T})
    BicycleControl current_control; //= zeros(BicycleControl{T})
    MPCTimeSteps time_steps;

    int N = 1 + N_short + N_long;
    TrackingBicycleState qs; //= rand(TrackingBicycleState{T}, N)    # not zeros so that construct_lateral_tracking_QP below works
    BicycleControl2 us; //= rand(BicycleControl2{T}, N)
    TrackingBicycleParams ps; //= zeros(TrackingBicycleParams{T}, N)
    VehicleModel tracking_dynamics = VehicleModel(VehicleModel vehicle, TrackingBicycleModel(vehicle));
    model, variables, parameters = construct_coupled_tracking_QP(tracking_dynamics, control_params, time_steps, qs, us, ps)
    TrajectoryTrackingMPC(vehicle, trajectory, dynamics, control_params,
                          current_state, current_control, 0, NaN,
                          time_steps,
                          qs, us, ps,
                          tracking_dynamics, model, variables, parameters)
}