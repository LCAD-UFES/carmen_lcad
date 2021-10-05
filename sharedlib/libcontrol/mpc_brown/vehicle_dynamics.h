#ifndef VEHICLE_DYNAMICS_H
#define VEHICLE_DYNAMICS_H


#include <cstdio>
#include <cstdlib>
#include <vector>
#include <string>
#include <map>


#ifdef __cplusplus
extern "C" {
#endif

using namespace std;


typedef struct {
    double delta_;
    double Fxf;
    double Fxr;
}BicycleControl;

typedef struct {
    // Dimensions
    double L;    // wheelbase
    double a;    // distance from front axle to CG
    double b;    // distance from rear axle to CG
    double h;    // CG height

    // Mass and yaw moment of inertia
    double G;    // gravitational acceleration
    double m;    // vehicle mass
    double Izz;  // yaw moment of inertia

    // Tire model parameters
    double mi;    // coefficient of friction
    double c_alphaf;  // front tire (pair) cornering stiffness
    double c_alphar;  // rear tire (pair) cornering stiffness

    // Longitudinal Drag Force Parameters (FxDrag = Cd0 + Cd1*Ux + Cd2*Ux^2)
    double Cd0;  // rolling resistance
    double Cd1;  // linear drag coefficint
    double Cd2;  // quadratic "aero" drag coefficint
}BicycleModelParams;

typedef struct {
    vector<double> E;    // world frame "x" position of CM
    vector<double> N;    // world frame "y" position of CM
    vector<double> phi;    // world frame heading of vehicle
    vector<double> Ux;   // body frame longitudinal speed
    vector<double> Uy;   // body frame lateral speed
    vector<double> r;   // yaw rate (dphi/dt)
}BicycleState;

typedef struct{
    double phi;    // nominal trajectory heading
    double k;    // nominal trajectory (local) curvature
    double theta;    // nominal trajectory (local) pitch grade
    double Phi;    // nominal trajectory (local) roll grade
}LocalRoadGeometry;


typedef struct {
    vector<double> delta_s;   // longitudinal error (w.r.t. nominal trajectory)
    double Ux;   // body frame longitudinal speed
    double Uy;   // body frame lateral speed
    vector<double> r;    // yaw rate
    double delta_phi;   // heading error (w.r.t. nominal trajectory)
    vector<double> e;    // lateral error (w.r.t. nominal trajectory)
}TrackingBicycleState;

typedef struct{ 
    double V;    // nominal trajectory speed
    vector<double> k;    // nominal trajectory (local) curvature
    double theta;    // nominal trajectory (local) pitch grade
    double Phi;    // nominal trajectory (local) roll grade
}TrackingBicycleParams;



typedef struct {
    double Uy;   // body frame lateral speed
    double r;    // yaw rate
    vector<double> delta_phi;   // heading error (w.r.t. nominal trajectory)
    vector<double> e;    // lateral error (w.r.t. nominal trajectory)
}LateralTrackingBicycleState;

typedef struct {
    double Ux;   // body frame lateral speed
    double k;    // yaw rate
    double theta;   // heading error (w.r.t. nominal trajectory)
    double theta_1;    // lateral error (w.r.t. nominal trajectory)
}LateralTrackingBicycleParams;

typedef struct {
    vector<double> _delta;
    vector<double> Fx;
}BicycleControl2;

typedef struct {
    double fwd_frac;
    double rwd_frac;
    double fwb_frac;
    double rwb_frac;
}LongitudinalActuationParams;


typedef struct {
    double Fx_max;
    double Fx_min;
    double Px_max;
    double _delta_max;
    double k_max;
}ControlLimits;

typedef struct {
    BicycleModelParams bicycle_model;
    LongitudinalActuationParams  longitudinal_params;
    ControlLimits control_limits;
}VehicleModel;

vector<double>
_fialatiremodel(vector<double> tan_alpha, double c_alpha, vector<double> Fy_max);

vector<double>
fialatiremodel(vector<double> alpha, double c_alpha, double mi, double Fx, vector<double> Fz);

double
_invfialatiremodel(double Fy, double c_alpha, double Fy_max);


double
invfialatiremodel(double Fy, double c_alpha, double mi, double Fx, double Fz);

vector<vector<double>>
lateral_tire_forces( BicycleModelParams B, vector<double> alpha_f, vector<double> alpha_r, 
double Fxf,double Fxr, vector<double> s_delta, vector<double> c_delta, int num_iters);

vector<vector<double>>
_lateral_tire_forces(BicycleModelParams B, BicycleState q, BicycleControl u, int num_iters);

BicycleState
make_BicycleState(BicycleModelParams B, double _phi, double Ux , double Uy, double r, 
vector<double> _delta, double Fxf, double Fxr);

TrackingBicycleState
make_TrackingBicycleState(BicycleModelParams B, double Ux, double Uy, double r, double _delta_phi,
                              vector<double> _delta, double Fxf, double Fxr,
                              double V, double k);

LateralTrackingBicycleState
make_LateralTrackingBicycleState(BicycleModelParams B, double Uy, double r,
                              vector<double> _delta, vector<double> Fxf, vector<double> Fxr,
                              double Ux, double k, double _delta_phi);

vector <double>
stable_limits(BicycleModelParams B, double Ux, double Fxf, double Fxr);

vector <double>
longitudinal_tire_forces(LongitudinalActuationParams LP, double Fx);

double
clamp (double x, double lo , double hi);

double ForwardDiff(vector<double> Ux);

BicycleControl2
apply_control_limits(ControlLimits CL, vector <double> vec, vector<double>  Ux);


map<string, vector<double>>
steady_state_estimates(VehicleModel X, vector<double> V, vector<double> A_tan, double k,
                                int num_iters, vector<double> r, vector<double> beta_zero, double _delta_zero, vector<double> Fyf_zero);

#ifdef __cplusplus
}
#endif

#endif