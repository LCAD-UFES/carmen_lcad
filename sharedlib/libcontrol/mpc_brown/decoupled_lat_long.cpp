#include <cstdio>
#include <cstdlib>
#include <string>
#include <map>
#include <cmath>
#include <vector>
#include <algorithm>
#include <boost/numeric/ublas/banded.hpp>
#include <boost/numeric/ublas/io.hpp>
#include "decoupled_lat_long.h"
#include "model_predictive_control.h"
#include "vehicle_dynamics.h"
#include "trajectories.h"

using namespace boost::numeric::ublas;
using namespace std;



DecoupledControlParams
make_decoupled_control_params()
{
    DecoupledControlParams dcp;
    dcp.V_min = 1.0;
    dcp.V_max = 15.0;
    dcp.k_V = 10 / 4 / 100;
    dcp.k_s = 10 / 4 / 10000;
    dcp.delta_max = 0.344;
    dcp.Q_delta_phi = 1 / pow((10 * M_PI / 180), 2);
    dcp.Q_e = 1.0;
    dcp.W_beta = 50 / (10 * M_PI / 180);
    dcp.W_r = 50.0;
    dcp.R_delta = 0.0;
    dcp.R_tri_delta = 0.01 / pow(10 * M_PI / 180, 2);
    return dcp;
}

typedef struct{
    diagonal_matrix<double> Q_delta_phi;
}LateralTrackingQPParams;

void DecoupledTrajectoryTrackingMPC( TrajectoryTube trajectory, DecoupledControlParams control_params,
                                       int N_short=10,int  N_long=20, double dt_short=(0.01),double dt_long=0.2, bool use_correction_step=true)
{
    control_params = make_decoupled_control_params();
    VehicleModel dynamics;
    BicycleState current_state; //= zeros(BicycleState{T})
    BicycleControl current_control; //= zeros(BicycleControl{T})
    MPCTimeSteps time_steps = make_MPCTimeSteps(N_short, N_long, dt_short, dt_long, use_correction_step);
    std::vector<LateralTrackingBicycleState> qs; //= rand(TrackingBicycleState{T}, N)    # not zeros so that construct_lateral_tracking_QP below works
    std::vector<BicycleControl2> us; //= rand(BicycleControl2{T}, N)
    std::vector<LateralTrackingBicycleParams> ps; //= zeros(TrackingBicycleParams{T}, N)
    int N = 1 + N_short + N_long;
    // model, variables, parameters = construct_lateral_tracking_QP(tracking_dynamics, control_params, time_steps, qs, us, ps)
    int aux = 0;
    double caco = 0.0;
    LateralTrajectoryTrackingMPC mpc = make_TrajectoryTrackingMPC_lateral(trajectory, dynamics, control_params,
                          current_state, current_control, aux, caco ,
                          time_steps,
                          qs,  us, ps);
}
/*
function DecoupledTrajectoryTrackingMPC(vehicle::Dict{Symbol,T}, trajectory::TrajectoryTube{T}; control_params=DecoupledControlParams(),
                                        N_short=10, N_long=20, dt_short=T(0.01), dt_long=T(0.2), use_correction_step=true) where {T}
    dynamics = VehicleModel(vehicle)
    current_state = zeros(BicycleState{T})
    current_control = zeros(BicycleControl{T})
    time_steps = MPCTimeSteps(N_short, N_long, dt_short, dt_long, use_correction_step)

    N = 1 + N_short + N_long
    qs = rand(LateralTrackingBicycleState{T}, N)    # not zeros so that construct_lateral_tracking_QP below works
    us = rand(BicycleControl2{T}, N)
    ps = rand(LateralTrackingBicycleParams{T}, N)
    tracking_dynamics = VehicleModel(vehicle, LateralTrackingBicycleModel(vehicle))
    model, variables, parameters = construct_lateral_tracking_QP(tracking_dynamics, control_params, time_steps, qs, us, ps)
    TrajectoryTrackingMPC(vehicle, trajectory, dynamics, control_params,
                          current_state, current_control, 0, NaN,
                          time_steps,
                          qs, us, ps,
                          tracking_dynamics, model, variables, parameters)
end
*/