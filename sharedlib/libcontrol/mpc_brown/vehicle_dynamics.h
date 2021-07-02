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
    double δ;
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
    double μ;    // coefficient of friction
    double Cαf;  // front tire (pair) cornering stiffness
    double Cαr;  // rear tire (pair) cornering stiffness

    // Longitudinal Drag Force Parameters (FxDrag = Cd0 + Cd1*Ux + Cd2*Ux^2)
    double Cd0;  // rolling resistance
    double Cd1;  // linear drag coefficint
    double Cd2;  // quadratic "aero" drag coefficint
}BicycleModelParams;

typedef struct {
    double E;    // world frame "x" position of CM
    double N;    // world frame "y" position of CM
    double ψ;    // world frame heading of vehicle
    double Ux;   // body frame longitudinal speed
    double Uy;   // body frame lateral speed
    double r;   // yaw rate (dψ/dt)
}BicycleState;

typedef struct{
    double ψ;    // nominal trajectory heading
    double κ;    // nominal trajectory (local) curvature
    double θ;    // nominal trajectory (local) pitch grade
    double ϕ;    // nominal trajectory (local) roll grade
}LocalRoadGeometry;


typedef struct {
    double Δs;   // longitudinal error (w.r.t. nominal trajectory)
    double Ux;   // body frame longitudinal speed
    double Uy;   // body frame lateral speed
    double r;    // yaw rate
    double Δψ;   // heading error (w.r.t. nominal trajectory)
    double e;    // lateral error (w.r.t. nominal trajectory)
}TrackingBicycleState;

typedef struct{ 
    double V;    // nominal trajectory speed
    double κ;    // nominal trajectory (local) curvature
    double θ;    // nominal trajectory (local) pitch grade
    double ϕ;    // nominal trajectory (local) roll grade
}TrackingBicycleParams;



typedef struct {
    double Uy;   // body frame lateral speed
    double r;    // yaw rate
    double Δψ;   // heading error (w.r.t. nominal trajectory)
    double e;    // lateral error (w.r.t. nominal trajectory)
}LateralTrackingBicycleState;

typedef struct {
    double Ux;   // body frame lateral speed
    double k;    // yaw rate
    double θ;   // heading error (w.r.t. nominal trajectory)
    double ϕ;    // lateral error (w.r.t. nominal trajectory)
}LateralTrackingBicycleParams;

typedef struct {
    double δ;
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
    double δ_max;
    double κ_max;
}ControlLimits;

typedef struct {
    BicycleModelParams bicycle_model;
    LongitudinalActuationParams  longitudinal_params;
    ControlLimits control_limits;
}VehicleModel;

double _fialatiremodel(double tanα, double Cα, double Fy_max);

double fialatiremodel(double α, double Cα, double μ, double Fx, double Fz);


double _invfialatiremodel(double Fy, double Cα, double Fy_max);


double invfialatiremodel(double Fy, double Cα, double μ, double Fx, double Fz);

vector<double> lateral_tire_forces( BicycleModelParams B, double αf, double αr, double Fxf, double Fxr, double sδ, double cδ, int num_iters = 3);

void _lateral_tire_forces(BicycleModelParams B, vector<double> q, vector<double> u, int num_iters=3);

vector <double> aaa(BicycleModelParams B, double E, double  N, double ψ, double Ux , double Uy, double r, 
double δ, double Fxf, double Fxr, double  ψᵣ, double κ, double θ, double ϕ);

vector <double> make_TrackingBicycleState(BicycleModelParams B, double Δs, double Ux, double Uy, double r, double Δψ, double e,
                              double δ, double Fxf, double Fxr,
                              double V, double κ, double θ, double ϕ);

vector <double> make_LateralTrackingBicycleState(BicycleModelParams B, double Uy, double r, double Δψ, double e,
                              double δ, double Fxf, double Fxr,
                              double Ux, double κ, double θ, double ϕ);

vector <double> stable_limits(BicycleModelParams B, double Ux, double Fxf, double Fxr);

vector <double> longitudinal_tire_forces(LongitudinalActuationParams LP, double Fx);

double clamp (double x, double lo , double hi);


BicycleControl2 apply_control_limits(ControlLimits CL, vector <double> vec, double Ux);


map<string, double> steady_state_estimates(VehicleModel X, double V, double A_tan, double κ,
                                int num_iters=4, double r, double β0, double δ0, double Fyf0);

#ifdef __cplusplus
}
#endif

#endif