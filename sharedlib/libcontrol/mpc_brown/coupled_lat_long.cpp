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

///Não sei se terminei essa função abaixo
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
    BicycleControl u0 = mpc.current_control;
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
    for(unsigned int i = 0; i < tn.si.phi.size(); i++ )
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
    if(mpc.solved)
    {
        for (unsigned int i = 1; i < TS.N_short + TS.N_long + 1; i++)
        {
            double t = TS.ts[i];
            if(t  < TS.prev_ts[TS.prev_ts.size() - 1])
            {
                
            }
        }
    }else
    {
        vector<double> s_delta_phi, c_delta_phi;
        for(unsigned int i = 0; i < delta_phi.size(); i++)
        {
            s_delta_phi.push_back(sin(delta_phi[i]));
            c_delta_phi.push_back(cos(delta_phi[i]));
        }
        vector<double> V, V_aux, V_aux_1, beta0, A_des;
        for(unsigned int i = 0; i < q0.Ux.size(); i++)
        {
            V.push_back(q0.Ux[i] * c_delta_phi[i] - q0.Uy[i] * s_delta_phi[i]); 
            beta0.push_back(atan2(q0.Ux[i], q0.Uy[i]));
        }
        vector<vector<double>> matrix_f = _lateral_tire_forces(mpc.tracking_dynamics.bicycle_model, q0, u0, 0);
        double tau;
        BicycleControl2 u;
        for(unsigned int i = 0; i < TS.N_short + TS.N_long + 1; i++)
        {
            (i == TS.N_short + TS.N_long + 1) ? tau = TS.dt[i - 1] : tau = TS.dt[i];
            TrajectoryNode aux = Traj_getindex_s(traj, ts[1]);
            for(int j = 0; j < V.size(); j ++)
            {
                A_des.push_back(aux.ti.TimeInterpolants_map["A"] + mpc.control_params.CoupledControlParams_map["k_V"]
                * (aux.ti.TimeInterpolants_map["V"] - V[j]) / tau);
                V_aux.push_back((mpc.control_params.CoupledControlParams_map["V_min"] - V[j]) / tau);
                V_aux_1.push_back((mpc.control_params.CoupledControlParams_map["V_max"] - V[j]) / tau);
            }
            A_des = min(max(A_des, V_aux),V_aux_1);
            vector<double> A;
            map<string, vector<double>> map_aux;
            if(i == 0)
            {
                for(unsigned int j = 0; j < q0.Ux.size(); j++)
                {
                    A.push_back((q0.Ux[j] - q0.r[j] * q0.Uy[j]) * c_delta_phi[j] - (q0.r[j] + q0.r[j] * q0.Ux[j]) * s_delta_phi[j]);
                }
            }else
            {
                if(i < TS.N_short + 1)
                {
                    map_aux = steady_state_estimates(mpc.tracking_dynamics, V, A_des, aux.ti.TimeInterpolants_map["k"], 1, 
                        q0.r, beta0, u0.delta_, matrix_f[0]);
                    q.Ux = q0.Ux;
                    q.Uy = q0.Uy;
                    q.r = q0.r;
                    q.E = e0;
                    for(unsigned int j = 0; j < q0.phi.size(); j++)
                    {
                        q.phi.push_back(q0.phi[j] - aux.si.phi[j]);
                    }
                    u._delta = map_aux["_delta"];
                    for(unsigned int j = 0; j < map_aux["Fxf"].size(); j++)
                    {
                        u.Fx.push_back(map_aux["Fxf"][j] + map_aux["Fxr"][j]);
                    }
                    TrackingBicycleParams p;
                    p.V = aux.ti.TimeInterpolants_map["V"];
                    p.k = aux.si.k;
                }else
                {
                    map_aux = steady_state_estimates(mpc.tracking_dynamics, V, A_des, aux.ti.TimeInterpolants_map["k"], 1, 
                        q0.r, beta0, u0.delta_, matrix_f[0]);
                    q.Ux = map_aux["Ux"];
                    q.Uy = map_aux["Uy"];
                    q.r = map_aux["r"];
                    for(unsigned int j = 0; j < map_aux["beta_"].size(); j++)
                    {
                        q.phi.push_back(-map_aux["beta_"][j]);
                    }
                    u._delta = map_aux["_delta"];
                    for(unsigned int j = 0; j < map_aux["Fxf"].size(); j++)
                    {
                        u.Fx.push_back(map_aux["Fxf"][j] + map_aux["Fxr"][j]);
                    }
                    TrackingBicycleParams p;
                    p.V = aux.ti.TimeInterpolants_map["V"];
                    p.k = aux.si.k;
                }
            }
            vector<BicycleState> qs;
            qs.push_back(q);
            vector<BicycleControl2> us;
            us.push_back(u);
            vector<TrackingBicycleParams> ps;
            ps.push_back(p);
            if(i == TS.N_short + TS.N_long)
            {
                break;
            }
            for(unsigned int j = 0; j < V.size(); j++)
            {
                V[j] = V[j] + map_aux["A"][j] * tau;
            }
            for(unsigned int j = 0; j < s0.size(); j++)
            {
                s0[j] = s0[j] + V[j] * tau + map_aux["A"][j] * tau * tau / 2;
            }
        }
       
    }
    return mpc;
}

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
    vector<double> u_normalization;
}TrackingQPVariables;

/*
TrackingQPVariables
make_TrackingQPVariables(vector<vector<double>> q, vector<vector<double>> u, vector<vector<double>> delta, vector<double> delta_HJI, 
    vector<double> u_normalization)
{

}
*/