#include <cstdio>
#include <cmath>
#include <vector>
#include <string>
#include <map>
#include "vehicle_dynamics.h"

using namespace std;

template <typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

double
_fialatiremodel(double tan_alpha, double c_alpha, double Fy_max)
{
    double tan_alpha_slide = 3 * Fy_max / c_alpha;
    double ratio = abs(tan_alpha / tan_alpha_slide);
    if (ratio <= 1)
    {
        return -c_alpha * tan_alpha * (1 - ratio + ratio * ratio /3);
    }else
    {
       return -Fy_max * sign(tan_alpha);
    }
}

//Coupled Tire Forces - Simplified Model
double
fialatiremodel(double alpha, double c_alpha, double mi, double Fx, double Fz)
{
    double F_max = mi * Fz;
    double aux;
    abs(Fx) >= F_max ? aux = 0.0 : aux = _fialatiremodel(tan(alpha), c_alpha, sqrt(F_max * F_max - Fx * Fx));
    return aux;
}

double
_invfialatiremodel(double Fy, double c_alpha, double Fy_max)
{
    double aux;
    if (abs(Fy) >= Fy_max)
    {
       aux = -(3 * Fy_max / c_alpha) * sign(Fy);
    }else
    {
        aux = -(1 + cbrt(abs(Fy) / Fy_max - 1)) * sign(Fy);
    }
    return aux;
}

double
invfialatiremodel(double Fy, double c_alpha, double mi, double Fx, double Fz)
{
    double F_max = mi * Fz;
    double Fy_max = sqrt(F_max * F_max - Fx * Fx);
    double aux = atan(_invfialatiremodel(Fy, c_alpha, Fy_max));
    return aux;
}

vector<double>
lateral_tire_forces( BicycleModelParams B, double alpha_f, double alpha_r, double Fxf, double Fxr, double s_delta, double c_delta, int num_iters)
{
    double Fyf = 0.0; 
    double Fx = Fxf * c_delta - Fyf * s_delta + Fxr;
    for (int i = 0; i < num_iters; i++)
    {
        double Fzf = (B.m * B.G * B.b - B.h * Fx) / B.L;
        double Fyf = fialatiremodel(alpha_f, B.c_alphaf, B.mi, Fxf, Fzf);
        Fx = Fxf * c_delta - Fyf*s_delta + Fxr;
    }
    double Fzr = (B.m * B.G * B.a + B.h * Fx) / B.L;
    double Fyr = fialatiremodel(alpha_r, B.c_alphar, B.mi, Fxr, Fzr);
    vector<double> result;
    result.push_back(Fyf);
    result.push_back(Fyr);
    return result;
}

void
_lateral_tire_forces(BicycleModelParams B, vector<double> q, vector<double> u, int num_iters)
{
    num_iters = 3;
    double Ux = q[3];
    double Uy = q[4];
    double r = q[5];
    double _delta = u[0];
    double Fxf = u[1];
    double Fxr =  u[2];
    double a = B.a, b = B.b;
    double s_delta = sin(_delta), c_delta = cos(_delta);
    double alpha_f = atan2(Uy + a * r, Ux) - _delta;
    double alpha_r = atan2(Uy - b * r, Ux);
    lateral_tire_forces(B, alpha_f, alpha_r, Fxf, Fxr, s_delta, c_delta, num_iters);
}



BicycleState
make_BicycleState(BicycleModelParams B, double _phi, double Ux , double Uy, double r, 
double _delta, double Fxf, double Fxr)
{
    double s_phi = sin(_phi), c_phi = cos(_phi);
    double s_delta = sin(_delta), c_delta = cos(_delta);
    double alpha_f = atan2(Uy + B.a * r, Ux) - _delta;
    double alpha_r = atan2(Uy - B.b * r, Ux);
    int num = 0;
    vector <double> Fy = lateral_tire_forces(B, alpha_f, alpha_r, Fxf, Fxr, s_delta, c_delta, num);
    double Fx_drag = -B.Cd0 - Ux * (B.Cd1 + B.Cd2 * Ux);
    double Fx_grade = 0;    // TODO: figure out how roll/pitch are ordered
    double Fy_grade = 0;
    double F_til_xf = Fxf * c_delta - Fy[0] * s_delta;
    double F_til_yf = Fy[0] * c_delta + Fxf * s_delta;
    BicycleState vec;
    vec.E.push_back(-Ux * s_phi - Uy * c_phi); // Ux*c_phi - Uy*s_phi (_phi measured from N))
    vec.N.push_back( Ux * c_phi - Uy * s_phi);
    vec.phi.push_back(r);
    vec.r.push_back((F_til_xf + Fxr + Fx_drag + Fx_grade) / B.m + r * Uy);
    vec.Ux.push_back((F_til_yf + Fy[1] + Fy_grade) / B.m - r * Ux);
    vec.Uy.push_back((B.a * F_til_yf - B.b * Fy[1]) / B.Izz);
    return vec;
}


TrackingBicycleState
make_TrackingBicycleState(BicycleModelParams B, double Ux, double Uy, double r, double _delta_phi,
                              double _delta, double Fxf, double Fxr,
                              double V, double k)
{
    double s_delta_phi = sin(_delta_phi), c_delta_phi = cos(_delta_phi);
    double s_delta = sin(_delta), c_delta = cos(_delta);
    double alpha_f = atan2(Uy + B.a * r, Ux) - _delta;
    double alpha_r = atan2(Uy - B.b * r, Ux);
    int num = 0;
    vector <double> Fy = lateral_tire_forces(B, alpha_f, alpha_r, Fxf, Fxr, s_delta, c_delta, num);
    double Fx_drag = -B.Cd0 - Ux * (B.Cd1 + B.Cd2 * Ux);
    double Fx_grade = 0;    // TODO: figure out how roll/pitch are ordered
    double Fy_grade = 0;
    double F_til_xf = Fxf * c_delta - Fy[0] * s_delta;
    double F_til_yf = Fy[0] * c_delta + Fxf * s_delta;
    TrackingBicycleState  vec;
    vec.delta_phi = (Ux * c_delta_phi - Uy * s_delta_phi - V); // Ux*c_phi - Uy*s_phi (_phi measured from N))
    vec.delta_s = ((F_til_xf + Fxr + Fx_drag + Fx_grade) / B.m + r * Uy);
    vec.e = ((F_til_yf + Fy[1] + Fy_grade) / B.m - r * Ux);
    vec.r = ((B.a * F_til_yf - B.b * Fy[1]) / B.Izz);
    vec.Ux = (r - (Ux * c_delta_phi - Uy*s_delta_phi) * k);
    vec.Uy = (Ux * s_delta_phi + Uy * c_delta_phi);
    return vec;
}   


LateralTrackingBicycleState
make_LateralTrackingBicycleState(BicycleModelParams B, double Uy, double r,
                              double _delta, double Fxf, double Fxr,
                              double Ux, double k, double _delta_phi)
{
    double s_delta_phi = sin(_delta_phi), c_delta_phi = cos(_delta_phi);
    double s_delta = sin(_delta), c_delta = cos(_delta);
    double alpha_f = atan2(Uy + B.a * r, Ux) - _delta;
    double alpha_r = atan2(Uy - B.b * r, Ux);
    int num = 0;
    vector <double> Fy = lateral_tire_forces(B, alpha_f, alpha_r, Fxf, Fxr, s_delta, c_delta, num);
    double Fy_grade = 0;
    double F_til_yf = Fy[0] * c_delta + Fxf * s_delta;
    LateralTrackingBicycleState vec;
    vec.delta_phi = ((F_til_yf + Fy[1] + Fy_grade) / B.m - r * Ux); // Ux*c_phi - Uy*s_phi (_phi measured from N))
    vec.e = ((B.a * F_til_yf - B.b * Fy[1]) / B.Izz);
    vec.r = (r - Ux * k);
    vec.Uy = (Ux * s_delta_phi + Uy * c_delta_phi);
    return vec;
}

vector <double>
stable_limits(BicycleModelParams B, double Ux, double Fxf, double Fxr)
{
    double Fx = Fxf + Fxr;
    double Fy_grade = 0;
    double Fzf = (B.m * B.G * B.b - B.h * Fx) / B.L;
    double Fzr = (B.m * B.G * B.a + B.h * Fx) / B.L;
    double Ff_max = B.mi * Fzf;
    double Fr_max = B.mi * Fzr;
    double Fyf_max = abs(Fxf) > Ff_max ? 0 : sqrt(Ff_max*Ff_max - Fxf*Fxf);
    double Fyr_max = abs(Fxr) > Fr_max ? 0 : sqrt(Fr_max*Fr_max - Fxr*Fxr);
    double tan_alphaf_slide = 3 * Fyf_max / B.c_alphaf;
    double tan_alphar_slide = 3 * Fyr_max / B.c_alphar;
    double alpha_f_slide = atan(tan_alphaf_slide);
    double alpha_r_slide = atan(tan_alphar_slide);

    // https://ddl.stanford.edu/sites/default/files/publications/2012_Thesis_Bobier_A_Phase_Portrait_Approach_to_Vehicle_Stabilization_and_Envelope_Control.pdf
    double _delta_max = atan(B.L * ( B.mi * B.G + Fy_grade / B.m) / (Ux * Ux) - tan_alphar_slide) + alpha_f_slide;    // corresponds to U̇y = 0; alpha_r < 0; alpha_f < 0; _delta > 0; r > 0
    double _delta_min = atan(B.L * (-B.mi * B.G + Fy_grade / B.m) / (Ux * Ux) + tan_alphar_slide) - alpha_f_slide;    // corresponds to U̇y = 0; alpha_r > 0; alpha_f > 0; _delta < 0; r < 0
    double rC  = ( B.mi * B.G + Fy_grade / B.m) / Ux;
    double UyC = -Ux * tan_alphar_slide + B.b * rC;
    double rD  = Ux / B.L * (tan( alpha_f_slide + _delta_max) - tan_alphar_slide);     // alpha_r > 0; alpha_f > 0; _delta > 0;
    double UyD = Ux * tan_alphar_slide + B.b *rD;
    double mCD = (rD - rC) / (UyD - UyC);
    double rE  = Ux / B.L * (tan(-alpha_f_slide + _delta_min) + tan_alphar_slide);    // alpha_r < 0; alpha_f < 0; _delta < 0;
    double UyE = -Ux * tan_alphar_slide + B.b * rE;
    double rF  = (-B.mi * B.G + Fy_grade / B.m) / Ux;
    double UyF = Ux * tan_alphar_slide + B.b * rF;
    double mEF = (rF - rE) / (UyF - UyE);
    vector<double> G_veh;
    G_veh.push_back(alpha_r_slide);
    G_veh.push_back(alpha_r_slide);
    G_veh.push_back(rC - UyC * mCD);
    G_veh.push_back(-rF + UyF * mEF);
    return G_veh;
    /*H_veh = @SMatrix [ 1/Ux -b/Ux;    # β max (≈Uy/Ux)
                      -1/Ux  b/Ux;    # β min (≈Uy/Ux)
                       -mCD     1;    # r max
                        mEF    -1]    # r min
    G_veh = SVector(αr_slide, αr_slide, rC - UyC*mCD, -rF + UyF*mEF)
    δ_min, δ_max, H_veh, G_veh*/

}

vector <double>
longitudinal_tire_forces(LongitudinalActuationParams LP, double Fx)
{
    vector <double> aux;
    if (Fx > 0) 
    { 
        aux.push_back(Fx * LP.fwd_frac);
        aux.push_back(Fx * LP.rwd_frac);
    }else
    {
        aux.push_back(Fx * LP.fwb_frac);
        aux.push_back(Fx * LP.rwb_frac);
    }
    return aux;
}

double
clamp (double x, double lo , double hi){
    if(x < lo) 
    {
        return lo;
    }else
    {
        if(x > hi)
        {
            return hi;
        }else
        {
            return x;
        }
    }
}

double ForwardDiff(vector<double> Ux)
{
    /*double a = Ux[Ux.size()- 1];
    double b = Ux[Ux.size()- 2];
    for (unsigned int i = 2; i < Ux.size(); i ++)
    {*/
    int i = 2;
    double a = (3 * Ux[i] - 4 * Ux[i-1] + Ux[i - 2])/ (2 * abs(Ux[i] - Ux[i - 1]));
    /*}
    Ux[0] = b;
    Ux[1] = a;*/
    return a;
}

BicycleControl2
apply_control_limits(ControlLimits CL, vector <double> vec, vector<double>  Ux)
{
    double Fx_max = CL.Fx_max;
    double Px_max = CL.Px_max;
    double Fx_min = CL.Fx_min;
    double _delta_max = CL._delta_max;
    double Ux_new = ForwardDiff(Ux);    // important for ForwardDiff/linearization, since Ux is technically a state variable
    BicycleControl2 BC2;
    BC2._delta = clamp(vec[0], -_delta_max, _delta_max);
    double min_num = min(vec[1], Fx_max);
    min_num = min(min_num, Px_max / Ux_new);
    BC2.Fx = max(min_num, Fx_min);
    return BC2;
}

map<string, double>
steady_state_estimates(VehicleModel X, double V, double A_tan, double k,
                                int num_iters, double r, double beta_zero, double _delta_zero, double Fyf_zero)
{
    num_iters = 4;
    double A_rad = V * V * k;
    double A_mag = hypot(A_tan, A_rad);
    double A_max = X.bicycle_model.mi * X.bicycle_model.G;
    if (A_mag > A_max)             // nominal trajectory total acceleration exceeds friction limit
    {    
        if(abs(A_rad) > A_max)    // nominal trajectory lateral acceleration exceeds friction limit
        {
            A_rad = A_max*sign(A_rad);
            A_tan = 0.0;
            // error("TODO: case when nominal trajectory lateral acceleration exceeds friction limit")
        }else    // prioritize radial acceleration for path tracking; reduce intended acceleration along path to compensate
        {       
            A_tan = sqrt(A_max * A_max - A_rad * A_rad) * sign(A_tan);
        }
    }
    double r_ponto = A_tan * k;

    double i = 1;
    double beta_= beta_zero;
    double _delta = _delta_zero;
    double Fyf = Fyf_zero;
    double Ux, Uy, Fxr, Fxf;

    while(true)
    {
        double sbeta_ = sin(beta_), cbeta_ = cos(beta_);
        double s_delta = sin(_delta), c_delta = cos(_delta);
        double Ux = V * cbeta_, Uy = V * sbeta_;
        double Fx_drag = -X.bicycle_model.Cd0 - Ux * (X.bicycle_model.Cd1 + X.bicycle_model.Cd2 * Ux);
        double Fx_grade = 0;    // TODO: figure out how roll/pitch are ordered
        double Fy_grade = 0;

        double Ax = A_tan * cbeta_ - A_rad * sbeta_;             // Ax = U̇x - r*Uy
        double Ay = A_tan * sbeta_ + A_rad * cbeta_;             // Ay = U̇y + r*Ux
        double Fx = Ax * X.bicycle_model.m - Fx_drag - Fx_grade;       // Fx = F_til_xf + Fxr = Fxf*c_delta - Fyf*s_delta + Fxr
        Fx = min(Fx, min(X.control_limits.Fx_max, X.control_limits.Px_max/Ux)*(X.longitudinal_params.rwd_frac + X.longitudinal_params.fwd_frac * c_delta) - Fyf * s_delta);    // braking force saturated by friction, not engine, limits
        double Fzr = (X.bicycle_model.m*X.bicycle_model.G*X.bicycle_model.a + X.bicycle_model.h*Fx) / X.bicycle_model.L, 
            Fzf = (X.bicycle_model.m * X.bicycle_model.G * X.bicycle_model.b - X.bicycle_model.h * Fx) / X.bicycle_model.L;
        double Fr_max = X.bicycle_model.mi * Fzr, Ff_max = X.bicycle_model.mi * Fzf;
        double aux;
        if(Fx > 0)
        {
            aux = X.longitudinal_params.rwd_frac / (X.longitudinal_params.rwd_frac + X.longitudinal_params.fwd_frac * c_delta);
        }else
        {
            aux = X.longitudinal_params.rwb_frac / (X.longitudinal_params.rwb_frac + X.longitudinal_params.fwb_frac * c_delta);
        }
        Fxr = clamp((Fx + Fyf*s_delta) * aux,
                    -Fr_max, Fr_max);
        double Fyr_max = sqrt(Fr_max * Fr_max - Fxr * Fxr);
        double Fyr = (Ay * X.bicycle_model.m - Fy_grade - r_ponto * X.bicycle_model.Izz / X.bicycle_model.a) / (1 + X.bicycle_model.b / X.bicycle_model.a);
        Fyr = clamp(Fyr, -Fyr_max, Fyr_max);    // TODO: maybe reduce A_tan in the case that Fyr exceeds Fyr_max
        double tan_alphar = _invfialatiremodel(Fyr, X.bicycle_model.c_alphar, Fyr_max);

        double F_til_xf = clamp(Fx - Fxr, -Ff_max, Ff_max);
        double F_til_yfmax = sqrt(Ff_max * Ff_max - F_til_xf * F_til_xf);
        double F_til_yf = clamp((X.bicycle_model.b * Fyr + r_ponto * X.bicycle_model.Izz) / X.bicycle_model.a, -F_til_yfmax, F_til_yfmax);
        Fxf = F_til_xf * c_delta + F_til_yf * s_delta;
        Fyf = F_til_yf * c_delta - F_til_xf * s_delta;
        double Fyf_max = sqrt(Ff_max * Ff_max - Fxf * Fxf);
        double alpha_f = atan(_invfialatiremodel(Fyf, X.bicycle_model.c_alphaf, Fyf_max));
        _delta = atan2(Uy + X.bicycle_model.a * r, Ux) - alpha_f;

        if(i == num_iters)
        {
            Ax = (Fxf * c_delta - Fyf * s_delta + Fxr + Fx_drag + Fx_grade) / X.bicycle_model.m;
            Ay = (Fyf * c_delta + Fxf * s_delta + Fyr + Fy_grade) / X.bicycle_model.m;
            A_tan = Ax * cbeta_ + Ay * sbeta_;
            break;
        }
        i = i + 1;
        beta_ = atan(tan_alphar + X.bicycle_model.b * r /Ux);
    }
    double sbeta_ = sin(beta_), cbeta_ = cos(beta_);
    Ux = V * cbeta_, Uy = V * sbeta_;
    
    map<string, double> map_aux;
    map_aux["beta_"] = beta_;
    map_aux["Ux"] = Ux;
    map_aux["Uy"] = Uy;
    map_aux["r"] = r;
    map_aux["A"] = A_tan;
    map_aux["_delta"] = _delta;
    map_aux["Fxf"] = Fxf;
    map_aux["Fxr"] = Fxr;

    return map_aux;
}