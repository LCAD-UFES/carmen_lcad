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
_fialatiremodel(double tanα, double Cα, double Fy_max)
{
    double tanα_slide = 3 * Fy_max / Cα;
    double ratio = abs(tanα / tanα_slide);
    if (ratio <= 1)
    {
        return -Cα * tanα * (1 - ratio + ratio * ratio /3);
    }else
    {
       return -Fy_max * sign(tanα);
    }
}

//Coupled Tire Forces - Simplified Model
double
fialatiremodel(double α, double Cα, double μ, double Fx, double Fz)
{
    double F_max = μ * Fz;
    double aux;
    abs(Fx) >= F_max ? aux = 0.0 : aux = _fialatiremodel(tan(α), Cα, sqrt(F_max*F_max - Fx*Fx));
    return aux;
}

double
_invfialatiremodel(double Fy, double Cα, double Fy_max)
{
    double aux;
    if (abs(Fy) >= Fy_max)
    {
       aux = -(3 * Fy_max / Cα) * sign(Fy);
    }else
    {
        aux = -(1 + cbrt(abs(Fy) / Fy_max - 1)) * sign(Fy);
    }
    return aux;
}

double
invfialatiremodel(double Fy, double Cα, double μ, double Fx, double Fz)
{
    double F_max = μ * Fz;
    double Fy_max = sqrt(F_max * F_max - Fx * Fx);
    double aux = atan(_invfialatiremodel(Fy, Cα, Fy_max));
    return aux;
}

vector<double>
lateral_tire_forces( BicycleModelParams B, double αf, double αr, double Fxf, double Fxr, double sδ, double cδ, int num_iters = 3)
{
    double Fyf = 0.0; 
    double Fx = Fxf * cδ - Fyf * sδ + Fxr;
    for (int i = 0; i < num_iters; i++)
    {
        double Fzf = (B.m * B.G * B.b - B.h * Fx) / B.L;
        double Fyf = fialatiremodel(αf, B.Cαf, B.μ, Fxf, Fzf);
        double Fx = Fxf*cδ - Fyf*sδ + Fxr;
    }
    double Fzr = (B.m * B.G * B.a + B.h * Fx) / B.L;
    double Fyr = fialatiremodel(αr, B.Cαr, B.μ, Fxr, Fzr);
    vector<double> result;
    result.push_back(Fyf);
    result.push_back(Fyr);
    return result;
}

void
_lateral_tire_forces(BicycleModelParams B, vector<double> q, vector<double> u, int num_iters=3)
{
    double Ux, Uy, r = q[3], q[4], q[5];
    double δ, Fxf, Fxr = u[0], u[1], u[2];
    double a = B.a, b = B.b;
    double sδ = sin(δ), cδ = cos(δ);
    double αf = atan2(Uy + a*r, Ux) - δ;
    double αr = atan2(Uy - b*r, Ux);
    lateral_tire_forces(B, αf, αr, Fxf, Fxr, sδ, cδ, num_iters);
}



vector <double>
aaa(BicycleModelParams B, double E, double  N, double ψ, double Ux , double Uy, double r, 
double δ, double Fxf, double Fxr, double  ψᵣ, double κ, double θ, double ϕ)
{
    double sψ = sin(ψ), cψ = cos(ψ);
    double sδ = sin(δ), cδ = cos(δ);
    double αf = atan2(Uy + B.a * r, Ux) - δ;
    double αr = atan2(Uy - B.b*r, Ux);
    vector <double> Fy = lateral_tire_forces(B, αf, αr, Fxf, Fxr, sδ, cδ);
    double Fx_drag = -B.Cd0 - Ux *(B.Cd1 + B.Cd2 * Ux);
    double Fx_grade = 0;    // TODO: figure out how roll/pitch are ordered
    double Fy_grade = 0;
    double F̃xf = Fxf * cδ - Fy[0] * sδ;
    double F̃yf = Fy[0] * cδ + Fxf * sδ;
    vector <double> vec;
    vec.push_back(-Ux * sψ - Uy * cψ); // Ux*cψ - Uy*sψ (ψ measured from N))
    vec.push_back( Ux * cψ - Uy * sψ);
    vec.push_back(r);
    vec.push_back((F̃xf + Fxr + Fx_drag + Fx_grade) / B.m + r * Uy);
    vec.push_back((F̃yf + Fy[1] + Fy_grade) / B.m - r * Ux);
    vec.push_back((B.a * F̃yf - B.b * Fy[1]) / B.Izz);
    return vec;
}


vector <double>
make_TrackingBicycleState(BicycleModelParams B, double Δs, double Ux, double Uy, double r, double Δψ, double e,
                              double δ, double Fxf, double Fxr,
                              double V, double κ, double θ, double ϕ)
{
    double sΔψ = sin(Δψ), cΔψ = cos(Δψ);
    double sδ = sin(δ), cδ = cos(δ);
    double αf = atan2(Uy + B.a * r, Ux) - δ;
    double αr = atan2(Uy - B.b * r, Ux);
    vector <double> Fy = lateral_tire_forces(B, αf, αr, Fxf, Fxr, sδ, cδ);
    double Fx_drag = -B.Cd0 - Ux * (B.Cd1 + B.Cd2 * Ux);
    double Fx_grade = 0;    // TODO: figure out how roll/pitch are ordered
    double Fy_grade = 0;
    double F̃xf = Fxf*cδ - Fy[0]*sδ;
    double F̃yf = Fy[0] *cδ + Fxf*sδ;
    vector <double> vec;
    vec.push_back(Ux*cΔψ - Uy*sΔψ - V); // Ux*cψ - Uy*sψ (ψ measured from N))
    vec.push_back((F̃xf + Fxr + Fx_drag + Fx_grade) / B.m + r * Uy);
    vec.push_back((F̃xf + Fxr + Fx_drag + Fx_grade) / B.m + r * Uy);
    vec.push_back((F̃yf + Fy[1] + Fy_grade)/B.m - r*Ux);
    vec.push_back((B.a * F̃yf - B.b * Fy[1]) / B.Izz);
    vec.push_back(r - (Ux*cΔψ - Uy*sΔψ)*κ);
    vec.push_back(Ux*sΔψ + Uy*cΔψ);
    return vec;
}   


vector <double>
make_LateralTrackingBicycleState(BicycleModelParams B, double Uy, double r, double Δψ, double e,
                              double δ, double Fxf, double Fxr,
                              double Ux, double κ, double θ, double ϕ)
{
    double sΔψ = sin(Δψ), cΔψ = cos(Δψ);
    double sδ = sin(δ), cδ = cos(δ);
    double αf = atan2(Uy + B.a * r, Ux) - δ;
    double αr = atan2(Uy - B.b * r, Ux);
    vector <double> Fy = lateral_tire_forces(B, αf, αr, Fxf, Fxr, sδ, cδ);
    double Fy_grade = 0;
    double F̃yf = Fy[0] * cδ + Fxf * sδ;
    vector <double> vec;
    vec.push_back((F̃yf + Fy[1] + Fy_grade) / B.m - r * Ux); // Ux*cψ - Uy*sψ (ψ measured from N))
    vec.push_back((B.a * F̃yf - B.b * Fy[1]) / B.Izz);
    vec.push_back(r - Ux * κ);
    vec.push_back(Ux * sΔψ + Uy * cΔψ);
    return vec;
}

vector <double>
stable_limits(BicycleModelParams B, double Ux, double Fxf, double Fxr)
{
    double Fx = Fxf + Fxr;
    double Fy_grade = 0;
    double Fzf = (B.m * B.G * B.b - B.h * Fx) / B.L;
    double Fzr = (B.m * B.G * B.a + B.h * Fx) / B.L;
    double Ff_max = B.μ * Fzf;
    double Fr_max = B.μ * Fzr;
    double Fyf_max = abs(Fxf) > Ff_max ? 0 : sqrt(Ff_max*Ff_max - Fxf*Fxf);
    double Fyr_max = abs(Fxr) > Fr_max ? 0 : sqrt(Fr_max*Fr_max - Fxr*Fxr);
    double tanαf_slide = 3 * Fyf_max / B.Cαf;
    double tanαr_slide = 3 * Fyr_max / B.Cαr;
    double αf_slide = atan(tanαf_slide);
    double αr_slide = atan(tanαr_slide);

    // https://ddl.stanford.edu/sites/default/files/publications/2012_Thesis_Bobier_A_Phase_Portrait_Approach_to_Vehicle_Stabilization_and_Envelope_Control.pdf
    double δ_max = atan(B.L * ( B.μ * B.G + Fy_grade / B.m) / (Ux * Ux) - tanαr_slide) + αf_slide;    // corresponds to U̇y = 0; αr < 0; αf < 0; δ > 0; r > 0
    double δ_min = atan(B.L * (-B.μ * B.G + Fy_grade / B.m) / (Ux * Ux) + tanαr_slide) - αf_slide;    // corresponds to U̇y = 0; αr > 0; αf > 0; δ < 0; r < 0
    double rC  = ( B.μ * B.G + Fy_grade / B.m) / Ux;
    double UyC = -Ux * tanαr_slide + B.b * rC;
    double rD  = Ux / B.L * (tan( αf_slide + δ_max) - tanαr_slide);     // αr > 0; αf > 0; δ > 0;
    double UyD = Ux * tanαr_slide + B.b *rD;
    double mCD = (rD - rC) / (UyD - UyC);
    double rE  = Ux / B.L * (tan(-αf_slide + δ_min) + tanαr_slide);    // αr < 0; αf < 0; δ < 0;
    double UyE = -Ux * tanαr_slide + B.b * rE;
    double rF  = (-B.μ * B.G + Fy_grade / B.m) / Ux;
    double UyF = Ux * tanαr_slide + B.b * rF;
    double mEF = (rF - rE) / (UyF - UyE);

}

vector <double>
longitudinal_tire_forces(LongitudinalActuationParams LP, double Fx)
{
    vector <double> aux;
    if (Fx > 0) 
    { 
        aux.push_back(Fx*LP.fwd_frac);
        aux.push_back(Fx*LP.rwd_frac);
    }else
    {
        aux.push_back(Fx*LP.fwb_frac);
        aux.push_back(Fx*LP.rwb_frac);
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

BicycleControl2
apply_control_limits(ControlLimits CL, vector <double> vec, double Ux)
{
    double Fx_max = CL.Fx_max;
    double Px_max = CL.Px_max;
    double Fx_min = CL.Fx_min;
    double δ_max = CL.δ_max;
    //double Ux = ForwardDiff.value(Ux)    // important for ForwardDiff/linearization, since Ux is technically a state variable
    BicycleControl2 BC2;
    BC2.δ = clamp(vec[0], -δ_max, δ_max);
    BC2.Fx = max(min(vec[1], Fx_max, Px_max/Ux), Fx_min);
    return BC2;
}

map<string, double>
steady_state_estimates(VehicleModel X, double V, double A_tan, double κ,
                                int num_iters=4, double r, double β0, double δ0, double Fyf0)
{
    double A_rad = V * V * κ;
    double A_mag = hypot(A_tan, A_rad);
    double A_max = X.bicycle_model.μ * X.bicycle_model.G;
    if (A_mag > A_max)             // nominal trajectory total acceleration exceeds friction limit
    {    
        if(abs(A_rad) > A_max)    // nominal trajectory lateral acceleration exceeds friction limit
        {
            A_rad = A_max*sign(A_rad);
            double A_tan = 0.0;
            // error("TODO: case when nominal trajectory lateral acceleration exceeds friction limit")
        }else    // prioritize radial acceleration for path tracking; reduce intended acceleration along path to compensate
        {       
            A_tan = sqrt(A_max * A_max - A_rad * A_rad) * sign(A_tan);
        }
    }
    double ṙ = A_tan * κ;

    double i = 1;
    double β, δ, Fyf = β0, δ0, Fyf0;
    double Ux, Uy, Fxr, Fxf;

    while(true)
    {
        double sβ = sin(β), cβ = cos(β);
        double sδ = sin(δ), cδ = cos(δ);
        double Ux = V * cβ, Uy = V * sβ;
        double Fx_drag = -X.bicycle_model.Cd0 - Ux * (X.bicycle_model.Cd1 + X.bicycle_model.Cd2 * Ux);
        double Fx_grade = 0;    // TODO: figure out how roll/pitch are ordered
        double Fy_grade = 0;

        double Ax = A_tan * cβ - A_rad * sβ;             // Ax = U̇x - r*Uy
        double Ay = A_tan * sβ + A_rad * cβ;             // Ay = U̇y + r*Ux
        double Fx = Ax * X.bicycle_model.m - Fx_drag - Fx_grade;       // Fx = F̃xf + Fxr = Fxf*cδ - Fyf*sδ + Fxr
        Fx = min(Fx, min(X.control_limits.Fx_max, X.control_limits.Px_max/Ux)*(X.longitudinal_params.rwd_frac + X.longitudinal_params.fwd_frac * cδ) - Fyf * sδ);    // braking force saturated by friction, not engine, limits
        double Fzr = (X.bicycle_model.m*X.bicycle_model.G*X.bicycle_model.a + X.bicycle_model.h*Fx) / X.bicycle_model.L, 
            Fzf = (X.bicycle_model.m * X.bicycle_model.G * X.bicycle_model.b - X.bicycle_model.h * Fx) / X.bicycle_model.L;
        double Fr_max = X.bicycle_model.μ * Fzr, Ff_max = X.bicycle_model.μ * Fzf;
        double aux;
        if(Fx > 0)
        {
            aux = X.longitudinal_params.rwd_frac / (X.longitudinal_params.rwd_frac + X.longitudinal_params.fwd_frac * cδ);
        }else
        {
            aux = X.longitudinal_params.rwb_frac / (X.longitudinal_params.rwb_frac + X.longitudinal_params.fwb_frac * cδ);
        }
        Fxr = clamp((Fx + Fyf*sδ) * aux,
                    -Fr_max, Fr_max);
        double Fyr_max = sqrt(Fr_max * Fr_max - Fxr * Fxr);
        double Fyr = (Ay * X.bicycle_model.m - Fy_grade - ṙ * X.bicycle_model.Izz / X.bicycle_model.a) / (1 + X.bicycle_model.b / X.bicycle_model.a);
        Fyr = clamp(Fyr, -Fyr_max, Fyr_max);    // TODO: maybe reduce A_tan in the case that Fyr exceeds Fyr_max
        double tanαr = _invfialatiremodel(Fyr, X.bicycle_model.Cαr, Fyr_max);

        double F̃xf = clamp(Fx - Fxr, -Ff_max, Ff_max);
        double F̃yfmax = sqrt(Ff_max * Ff_max - F̃xf * F̃xf);
        double F̃yf = clamp((X.bicycle_model.b * Fyr + ṙ * X.bicycle_model.Izz) / X.bicycle_model.a, -F̃yfmax, F̃yfmax);
        Fxf = F̃xf * cδ + F̃yf * sδ;
        Fyf = F̃yf * cδ - F̃xf * sδ;
        double Fyf_max = sqrt(Ff_max * Ff_max - Fxf * Fxf);
        double αf = atan(_invfialatiremodel(Fyf, X.bicycle_model.Cαf, Fyf_max));
        double δ = atan2(Uy + X.bicycle_model.a * r, Ux) - αf;

        if(i == num_iters)
        {
            Ax = (Fxf * cδ - Fyf * sδ + Fxr + Fx_drag + Fx_grade) / X.bicycle_model.m;
            Ay = (Fyf * cδ + Fxf * sδ + Fyr + Fy_grade) / X.bicycle_model.m;
            A_tan = Ax * cβ + Ay * sβ;
            break;
        }
        i = i + 1;
        β = atan(tanαr + X.bicycle_model.b * r /Ux);
    }
    double sβ = sin(β), cβ = cos(β);
    double Ux = V * cβ, Uy = V * sβ;
    
    map<string, double> map_aux;
    map_aux["β"] = β;
    map_aux["Ux"] = Ux;
    map_aux["Uy"] = Uy;
    map_aux["r"] = r;
    map_aux["A"] = A_tan;
    map_aux["δ"] = δ;
    map_aux["Fxf"] = Fxf;
    map_aux["Fxr"] = Fxr;

    return map_aux;
}