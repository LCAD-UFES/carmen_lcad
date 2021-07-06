#ifndef VEHICLE_DYNAMICS_H
#define VEHICLE_DYNAMICS_H


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
    double E;    // world frame "x" position of CM
    double N;    // world frame "y" position of CM
    double phi;    // world frame heading of vehicle
    double Ux;   // body frame longitudinal speed
    double Uy;   // body frame lateral speed
    double r;   // yaw rate (dphi/dt)
}BicycleState;

typedef struct{
    double phi;    // nominal trajectory heading
    double k;    // nominal trajectory (local) curvature
    double theta;    // nominal trajectory (local) pitch grade
    double Phi;    // nominal trajectory (local) roll grade
}LocalRoadGeometry;


typedef struct {
    double delta_s;   // longitudinal error (w.r.t. nominal trajectory)
    double Ux;   // body frame longitudinal speed
    double Uy;   // body frame lateral speed
    double r;    // yaw rate
    double delta_phi;   // heading error (w.r.t. nominal trajectory)
    double e;    // lateral error (w.r.t. nominal trajectory)
}TrackingBicycleState;

typedef struct{ 
    double V;    // nominal trajectory speed
    double k;    // nominal trajectory (local) curvature
    double theta;    // nominal trajectory (local) pitch grade
    double Phi;    // nominal trajectory (local) roll grade
}TrackingBicycleParams;



typedef struct {
    double Uy;   // body frame lateral speed
    double r;    // yaw rate
    double delta_phi;   // heading error (w.r.t. nominal trajectory)
    double e;    // lateral error (w.r.t. nominal trajectory)
}LateralTrackingBicycleState;

typedef struct {
    double Ux;   // body frame lateral speed
    double k;    // yaw rate
    double theta;   // heading error (w.r.t. nominal trajectory)
    double theta_1;    // lateral error (w.r.t. nominal trajectory)
}LateralTrackingBicycleParams;

typedef struct {
    double _delta;
    double Fx;
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

double _fialatiremodel(double tan_alpha, double c_alpha, double Fy_max);

double fialatiremodel(double alpha, double c_alpha, double mi, double Fx, double Fz);


double _invfialatiremodel(double Fy, double c_alpha, double Fy_max);


double invfialatiremodel(double Fy, double c_alpha, double mi, double Fx, double Fz);

vector<double> lateral_tire_forces( BicycleModelParams B, double alpha_f, double alpha_r, double Fxf, double Fxr, double s_delta, double c_delta, int num_iters);

void _lateral_tire_forces(BicycleModelParams B, vector<double> q, vector<double> u, int num_iters);

vector <double> aaa(BicycleModelParams B, double E, double  N, double phi, double Ux , double Uy, double r, 
double delta_, double Fxf, double Fxr, double  _phi_r, double k, double theta, double Phi);

vector <double> make_TrackingBicycleState(BicycleModelParams B, double delta_s, double Ux, double Uy, double r, double delta_phi, double e,
                              double delta_, double Fxf, double Fxr,
                              double V, double k, double theta, double Phi);

vector <double> make_LateralTrackingBicycleState(BicycleModelParams B, double Uy, double r, double delta_phi, double e,
                              double delta_, double Fxf, double Fxr,
                              double Ux, double k, double theta, double Phi);

vector <double> stable_limits(BicycleModelParams B, double Ux, double Fxf, double Fxr);

vector <double> longitudinal_tire_forces(LongitudinalActuationParams LP, double Fx);

double clamp (double x, double lo , double hi);


BicycleControl2 apply_control_limits(ControlLimits CL, vector <double> vec, double Ux);


map<string, double> steady_state_estimates(VehicleModel X, double V, double A_tan, double k,
                                int num_iters, double r, double beta_zero, double _delta_zero, double Fyf_zero);

#ifdef __cplusplus
}
#endif

#endif