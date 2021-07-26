#include <cstdio>
#include <cmath>
#include <vector>
#include <string>
#include <map>
#include "coupled_lat_long.h"
#include "model_predictive_control.h"
#include "vehicle_dynamics.h"
#include "trajectories.h"

CoupledControlParams 
make_CoupledControlParams()
{
    CoupledControlParams CCP_m;
    CCP_m.CoupledControlParams_map["V_min"] = 1.0;
    CCP_m.CoupledControlParams_map["V_max"] = 15.0;
    CCP_m.CoupledControlParams_map["k_V"] = 10 / 4 / 100;
    CCP_m.CoupledControlParams_map["k_s"] = 10 / 4 / 10000;
    CCP_m.CoupledControlParams_map["delta_max"] = 0.344;
    CCP_m.CoupledControlParams_map["Q_delta_s"] = 1.0;
    CCP_m.CoupledControlParams_map["Q_delta_phi"] = 1.0;
    CCP_m.CoupledControlParams_map["Q_e"] = 1.0;
    CCP_m.CoupledControlParams_map["W_beta"] = 50 / (10 * M_1_PI / 180);
    CCP_m.CoupledControlParams_map["W_r"] = 50.0;
    CCP_m.CoupledControlParams_map["W_HJI"] = 500.0;
    CCP_m.CoupledControlParams_map["N_HJI"] = 3;
    CCP_m.CoupledControlParams_map["R_delta"] = 0.0;
    CCP_m.CoupledControlParams_map["R_delta_delta"] = 0.1;
    CCP_m.CoupledControlParams_map["R_Fx"] = 0.0;
    CCP_m.CoupledControlParams_map["R_delta_Fx"] = 0.5;
    return CCP_m;
}

TrajectoryTrackingMPC
CoupledTrajectoryTrackingMPC( TrajectoryTube trajectory, CoupledControlParams control_params,
                                     int N_short = 10, int N_long = 20, double dt_short = (0.01), double dt_long = (0.2), bool use_correction_step=true)
{
    VehicleModel dynamics;
    BicycleState current_state; //= zeros(BicycleState{T})
    BicycleControl current_control; //= zeros(BicycleControl{T})
    MPCTimeSteps time_steps = make_MPCTimeSteps(N_short, N_long, dt_short, dt_long, use_correction_step);

    int N = 1 + N_short + N_long;
    vector<TrackingBicycleState> qs; //= rand(TrackingBicycleState{T}, N)    # not zeros so that construct_lateral_tracking_QP below works
    vector<BicycleControl2> us; //= rand(BicycleControl2{T}, N)
    vector<TrackingBicycleParams> ps; //= zeros(TrackingBicycleParams{T}, N)
    VehicleModel tracking_dynamics; // = VehicleModel(VehicleModel vehicle, TrackingBicycleState(vehicle));
    //model, variables, parameters = construct_coupled_tracking_QP(tracking_dynamics, control_params, time_steps, qs, us, ps)
    int aux = 0;
    double caco = 0.0;
    TrajectoryTrackingMPC mpc = make_TrajectoryTrackingMPC(trajectory, dynamics, control_params,
                          current_state, current_control, aux, caco ,
                          time_steps,
                          qs,  us, ps);
    return mpc;
}


TrajectoryTrackingMPC
compute_linearization_nodes( TrajectoryTrackingMPC mpc,
                                    vector<TrackingBicycleState>  qs,
                                    vector<BicycleControl2>  us,
                                    vector<TrackingBicycleParams>  ps)
{
    TrajectoryTube traj = mpc.trajectory;
    MPCTimeSteps TS = mpc.time_steps;
    vector<double> ts = TS.ts;
    BicycleState q0 = mpc.current_state;
    vector<vector<double>> matrix = path_coordinates(traj, q0);
    vector<double> s0 = matrix[0];
    vector<double> e0 = matrix[1];
    vector<double> t0 = matrix[2];
    vector<double> delta_s;
    TrajectoryNode tn = Traj_getindex_s(traj, ts[1]);
    /*for(unsigned int i = 0; i < s0.size(); i++)
    {
        delta_s.push_back(s0[i] - tn.);
    }*/
    vector<double> delta_phi;
    for(int i = 0; i < tn.si.phi.size(); i++ )
    {
        delta_phi.push_back(q0.phi[0] - tn.si.phi[i]);
    }
    BicycleState q;
    q.N = delta_s;
    q.E = e0;
    q.Ux = q0.Ux;
    q.Uy = q0.Uy;
    q.r = q0.r;
    q.phi = q0.phi;
    TrackingBicycleParams p;
    p.V = tn.ti.TimeInterpolants_map["V"];
    p.k = tn.si.k;
}
/*
void construct_coupled_tracking_QP( VehicleModel dynamics, CoupledControlParams control_params, double time_steps, vector<TrackingBicycleState> qs, 
vector<BicycleControl2> us, double ps, double N_short, double N_long, double dt )
{
    double δ_max_  = dynamics.control_limits._delta_max, delta_l_max   = control_params.CoupledControlParams_map["delta_max"];
    double Fx_max_ = dynamics.control_limits.Fx_max, Px_max =  dynamics.control_limits.Px_max;
    double Uxt, Fxft, Fxrt, Uy_r;
    vector<double> Uy_r, delta_t, ltf_vector, delta_s;
    for (double i = 0; i < N_short+N_long; i++ )
    {
        Uxt = qs[i+1].Ux;
        ltf_vector = longitudinal_tire_forces(dynamics.longitudinal_params, us[i+1].Fx);
        δ_mint, δ_maxt, Ht, Gt = stable_limits(dynamics.bicycle_model, Uxt, Fxft, Fxrt);
        Uy_r.push_back(q[3][i + 1]);
        Uy_r.push_back(q[4][i + 1]);
        delta_t.push_back(delta[1][i]);
        delta_t.push_back(delta[1][i]);
        delta_t.push_back(delta[2][i]);
        delta_t.push_back(delta[2][i]);
    }

    /*
    for t in 1:N_short+N_long
        Uxt = qs[t+1].Ux
        Fxft, Fxrt = longitudinal_tire_forces(dynamics.longitudinal_params, us[t+1].Fx)
        δ_mint, δ_maxt, Ht, Gt = stable_limits(dynamics.bicycle_model, Uxt, Fxft, Fxrt)
        Ht = push!(H, Parameter(Array(Ht), m))[end]
        Gt = push!(G, Parameter(Array(Gt), m))[end]
        δ_mint  = push!(δ_min,  Parameter([max(δ_mint, -δ_max_) / u_normalization[1]], m))[end]
        δ_maxt  = push!(δ_max,  Parameter([min(δ_maxt, δ_max_)  / u_normalization[1]], m))[end]
        Fx_maxt = push!(Fx_max, Parameter([min(Px_max/Uxt, Fx_max_) / u_normalization[2]], m))[end]
        Uy_r = q[3:4,t+1]
        σt   = [σ[1,t],σ[1,t],σ[2,t],σ[2,t]]
        Δδt  = [Δδ[t]]
        Δδ_mint = push!(Δδ_min, Parameter([-δ̇_max*dt[t] / u_normalization[1]], m))[end]
        Δδ_maxt = push!(Δδ_max, Parameter([ δ̇_max*dt[t] / u_normalization[1]], m))[end]
        @constraint(m, [δ[t+1]] <= δ_maxt)
        @constraint(m, [δ[t+1]] >= δ_mint)
        @constraint(m, [Fx[t+1]] <= Fx_maxt)
        @constraint(m, Ht*Uy_r - Gt <= σt)
        @constraint(m, Δδt <= Δδ_maxt)
        @constraint(m, Δδt >= Δδ_mint)
    end
    
}
*/
typedef struct{
    vector<vector<double>> Q_delta_s;
    vector<vector<double>> Q_delta_phi;
    vector<vector<double>> Q_e;
    vector<vector<double>> R_delta;
    vector<vector<double>> R_delta_delta;
    vector<vector<double>> R_Fx;
    vector<vector<double>> R_delta_Fx;
    vector<double> W_beta;
    vector<double> W_r;
    vector<double> W_HJI;
    vector<double> q_curr;
    vector<double> u_curr;
    vector<vector<double>> M_HJI;
    vector<vector<double>> b_HJI;
    vector<vector<double>> A;
    vector<vector<double>> B;
    vector<vector<double>> B0;
    vector<vector<double>> Bf;
    vector<double> c;
    vector<vector<double>> H;
    vector<vector<double>> G;
    vector<vector<double>> delta_min;
    vector<vector<double>> delta_max;
    vector<vector<double>> Fx_max;
    vector<vector<double>> delta_delta_min;
    vector<vector<double>> delta_delta_max;
}TrackingQPParams;

typedef struct{
    vector<vector<double>> q;
    vector<vector<double>> u;
    vector<vector<double>> delta;
    vector<double> delta_HJI;
}TrackingQPVariables;

/*
void update_QP(TrackingBicycleState dynamics, TrackingQPParams QPP, MPCTimeSteps mpc)
{
    //dynamics = mpc.
   
    double  N_short = mpc.N_short, N_long = mpc.N_long; 
    vector <double> dt = mpc.dt;
    for(int i = 0 ; i < dynamics.delta_s.size(); i++)
    {
        dynamics.
    }
}

get_next_control(mpc::TrajectoryTrackingMPC, variables::TrackingQPVariables)
*/