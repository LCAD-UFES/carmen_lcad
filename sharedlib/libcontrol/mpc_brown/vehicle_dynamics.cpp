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

vector<double>
_fialatiremodel(vector<double> tan_alpha, double c_alpha, vector<double> Fy_max)
{
    vector<double> tan_alpha_slide;
    for(unsigned int i = 0; i < Fy_max.size(); i++)
    { 
     tan_alpha_slide.push_back(3 * Fy_max[i] / c_alpha);
    }
    vector<double> ratio, vec; 
    for(unsigned int i = 0; i < tan_alpha.size(); i++)
    {
        ratio.push_back(abs(tan_alpha[i] / tan_alpha_slide[i]));
    }
    for(unsigned int i = 0; i < ratio.size(); i++)
    {
        if (ratio[i] <= 1)
        {
            vec.push_back(-c_alpha * tan_alpha[i] * (1 - ratio[i] + ratio[i] * ratio[i] /3));
        }else
        {
            vec.push_back(-Fy_max[i] * sign(tan_alpha[i]));
        }
    }
    return vec;
}

//Coupled Tire Forces - Simplified Model
vector<double>
fialatiremodel(vector<double> alpha, double c_alpha, double mi, vector<double> Fx, vector<double> Fz)
{
    vector<double> F_max, vec;
    for(unsigned int i = 0; i < Fz.size(); i ++)
    {
        F_max.push_back(mi * Fz[i]);
    }
    vector<double> aux, tan_alpha;
    for(unsigned int i = 0; i < F_max.size(); i ++)
    {
        vec.push_back(sqrt(F_max[i] * F_max[i] - Fx[i] * Fx[i]));
    }
    for(unsigned int i = 0; i < Fx.size(); i++)
    {
        if(abs(Fx[i]) >= F_max[0])
        {
            for(unsigned int i = 0; i < alpha.size(); i ++)
            {
                aux.push_back(0.0);
            }
        }else
        {
            for(unsigned int i = 0; i < alpha.size(); i ++)
            {
                tan_alpha.push_back(tan(alpha[i]));
            }
            aux = _fialatiremodel(tan_alpha, c_alpha, vec);
        }
    } 
    
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

vector<vector<double>>
lateral_tire_forces( BicycleModelParams B, vector<double> alpha_f, vector<double> alpha_r, 
vector<double> Fxf,vector<double> Fxr, vector<double> s_delta, vector<double> c_delta, int num_iters)
{
    vector<double> Fyf, Fx, Fzr, Fzf;
    for(unsigned int i = 0; i < alpha_f.size(); i++)
    { 
        Fx.push_back(Fxf[i] * c_delta[i] - Fyf[i] * s_delta[i] + Fxr[i]);
    }
    for (int i = 0; i < num_iters; i++)
    {
        for(unsigned int j = 0; j < Fx.size(); j ++)
        {
            Fzf.push_back((B.m * B.G * B.b - B.h * Fx[j]) / B.L);
        }
        Fyf = fialatiremodel(alpha_f, B.c_alphaf, B.mi, Fxf, Fzf);
        for(unsigned int j = 0; j < Fyf.size(); j ++)
        {
            Fx.push_back(Fxf[j] * c_delta[j] - Fyf[j] * s_delta[j] + Fxr[j]);
        }
    }
    for(unsigned int i = 0; i < Fyf.size(); i ++)
    {
        Fzr.push_back((B.m * B.G * B.a + B.h * Fx[i]) / B.L);
    }
    vector<double> Fyr = fialatiremodel(alpha_r, B.c_alphar, B.mi, Fxr, Fzr);
    vector<vector<double>> result;
    result.push_back(Fyf);
    result.push_back(Fyr);
    return result;
}

vector<vector<double>>
_lateral_tire_forces(BicycleModelParams B, BicycleState q, BicycleControl u, int num_iters)
{
    num_iters = 3;
    vector<double> Ux = q.Ux;
    vector<double> Uy = q.Uy;
    vector<double> r = q.r;
    vector<double> _delta = u.delta_;
    vector<double> Fxf = u.Fxf;
    vector<double> Fxr =  u.Fxr;
    double a = B.a, b = B.b;
    vector<double> s_delta, c_delta; 
    for(unsigned int i = 0; i < _delta.size(); i++)
    {
        s_delta.push_back(sin(_delta[i]));
        c_delta.push_back(cos(_delta[i]));
    }
    vector<double> alpha_f,   alpha_r;
    for (unsigned int i = 0; i < Uy.size(); i++)
    {
        alpha_f.push_back(atan2(Uy[i] + a * r[i], Ux[i]) - _delta[i]);
        alpha_r.push_back(atan2(Uy[i] - b * r[i], Ux[i]));
    }
    vector<vector<double>> matrix_f = lateral_tire_forces(B, alpha_f, alpha_r, Fxf, Fxr, s_delta, c_delta, num_iters);
    return matrix_f; 
}



BicycleState
make_BicycleState(BicycleModelParams B, double _phi, double Ux , double Uy, double r, 
vector<double> _delta, vector<double> Fxf, vector<double> Fxr)
{
    double s_phi = sin(_phi), c_phi = cos(_phi);
    vector<double> s_delta, c_delta;
    for(unsigned int i = 0; i < _delta.size(); i++)
    {
        s_delta.push_back(sin(_delta[i])); c_delta.push_back(cos(_delta[i]));
    }
    vector<double> alpha_f;
    for(unsigned int i = 0; i < _delta.size(); i++) 
        alpha_f.push_back(atan2(Uy + B.a * r, Ux) - _delta[i]);
    vector<double> alpha_r; 
    alpha_r.push_back(atan2(Uy - B.b * r, Ux));
    int num = 0;
    vector<vector<double>> Fy = lateral_tire_forces(B, alpha_f, alpha_r, Fxf, Fxr, s_delta, c_delta, num);
    double Fx_drag = -B.Cd0 - Ux * (B.Cd1 + B.Cd2 * Ux);
    double Fx_grade = 0;    // TODO: figure out how roll/pitch are ordered
    double Fy_grade = 0;
    vector<double> F_til_xf, F_til_yf;
    for(unsigned int i = 0; i < Fy[0].size(); i++)
    {
        F_til_xf.push_back(Fxf[i] * c_delta[i] - Fy[0][i] * s_delta[i]);
        F_til_yf.push_back(Fy[0][i] * c_delta[i] + Fxf[i] * s_delta[i]);
    }
    BicycleState vec;
    vec.E.push_back(-Ux * s_phi - Uy * c_phi); // Ux*c_phi - Uy*s_phi (_phi measured from N))
    vec.N.push_back( Ux * c_phi - Uy * s_phi);
    vec.phi.push_back(r);
    for(unsigned int i = 0; i < Fy[1].size(); i++)
    {
        vec.r.push_back((F_til_xf[i] + Fxr[i] + Fx_drag + Fx_grade) / B.m + r * Uy);
        vec.Ux.push_back((F_til_yf[i] + Fy[1][i] + Fy_grade) / B.m - r * Ux);
        vec.Uy.push_back((B.a * F_til_yf[i] - B.b * Fy[1][i]) / B.Izz);
    }
    
    return vec;
}


TrackingBicycleState
make_TrackingBicycleState(BicycleModelParams B, double Ux, double Uy, double r, double _delta_phi,
                              vector<double> _delta, vector<double> Fxf, vector<double> Fxr,
                              double V, double k)
{
    double s_delta_phi = sin(_delta_phi), c_delta_phi = cos(_delta_phi);
    vector<double> s_delta, c_delta;
    for(unsigned int i = 0; i < _delta.size(); i++)
    {
        s_delta.push_back(sin(_delta[i]));
        c_delta.push_back(cos(_delta[i]));
    }
    vector<double> alpha_f;
    for(unsigned int i = 0; _delta.size(); i++)
        alpha_f.push_back(atan2(Uy + B.a * r, Ux) - _delta[i]);
    vector<double> alpha_r;
    alpha_r.push_back(atan2(Uy - B.b * r, Ux));
    int num = 0;
    vector<vector<double>> Fy = lateral_tire_forces(B, alpha_f, alpha_r, Fxf, Fxr, s_delta, c_delta, num);
    double Fx_drag = -B.Cd0 - Ux * (B.Cd1 + B.Cd2 * Ux);
    double Fx_grade = 0;    // TODO: figure out how roll/pitch are ordered
    double Fy_grade = 0;
    vector<double> F_til_xf, F_til_yf;
    for(unsigned int i = 0; i < Fy[0].size(); i++)
    {
        F_til_xf.push_back(Fxf[i] * c_delta[i] - Fy[0][i] * s_delta[i]);
        F_til_yf.push_back(Fy[0][i] * c_delta[i] + Fxf[i] * s_delta[i]);
    }
    TrackingBicycleState  vec;
    vec.delta_phi = (Ux * c_delta_phi - Uy * s_delta_phi - V); // Ux*c_phi - Uy*s_phi (_phi measured from N))
    for(unsigned int i =0; i < F_til_xf.size(); i++)
    {
        vec.delta_s.push_back((F_til_xf[i] + Fxr[i] + Fx_drag + Fx_grade) / B.m + r * Uy);
    }
    
    for(unsigned int i = 0; i < Fy[1].size(); i ++)
    {
        vec.e.push_back((F_til_yf[i] + Fy[1][i] + Fy_grade) / B.m - r * Ux);
        vec.r.push_back((B.a * F_til_yf[i] - B.b * Fy[1][i]) / B.Izz);
    }
    
    vec.Ux = (r - (Ux * c_delta_phi - Uy*s_delta_phi) * k);
    vec.Uy = (Ux * s_delta_phi + Uy * c_delta_phi);
    return vec;
}   


LateralTrackingBicycleState
make_LateralTrackingBicycleState(BicycleModelParams B, double Uy, double r,
                              vector<double> _delta, vector<double> Fxf, vector<double> Fxr,
                              double Ux, double k, double _delta_phi)
{
    double s_delta_phi = sin(_delta_phi), c_delta_phi = cos(_delta_phi);
    vector<double> s_delta, c_delta;
    for(unsigned int i = 0; i < _delta.size(); i++)
    {
        s_delta.push_back(sin(_delta[i]));
        c_delta.push_back(cos(_delta[i]));
    }
    vector<double> alpha_f;
    for(unsigned int i = 0; i < _delta.size(); i++) 
        alpha_f.push_back(atan2(Uy + B.a * r, Ux) - _delta[i]);
    vector<double> alpha_r;
    alpha_r.push_back(atan2(Uy - B.b * r, Ux));
    int num = 0;
    vector<vector<double>> Fy = lateral_tire_forces(B, alpha_f, alpha_r, Fxf, Fxr, s_delta, c_delta, num);
    double Fy_grade = 0;
    vector<double> F_til_yf;
    for(unsigned int i = 0; i < Fy[0].size(); i ++)
    { 
        F_til_yf.push_back(Fy[0][i] * c_delta[i] + Fxf[i] * s_delta[i]);
    }
    LateralTrackingBicycleState vec;
    for(unsigned int i = 0; i < Fy[1].size(); i ++)
    {
        vec.delta_phi.push_back((F_til_yf[i] + Fy[1][i] + Fy_grade) / B.m - r * Ux); // Ux*c_phi - Uy*s_phi (_phi measured from N))
        vec.e.push_back((B.a * F_til_yf[i] - B.b * Fy[1][i]) / B.Izz);
    }
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
    BC2._delta.push_back(clamp(vec[0], -_delta_max, _delta_max));
    double min_num = min(vec[1], Fx_max);
    min_num = min(min_num, Px_max / Ux_new);
    BC2.Fx.push_back(max(min_num, Fx_min));
    return BC2;
}

map<string, vector<double>>
steady_state_estimates(VehicleModel X, vector<double> V, vector<double> A_tan, double k,
                                int num_iters, vector<double> r, vector<double> beta_zero, vector<double> _delta_zero, vector<double> Fyf_zero)
{
    vector<double> A_rad;
    for(int j = 0; j <= V.size(); j++)
    {
        A_rad.push_back(V[j] * V[j] * k);
    }
    vector<double> A_mag, r_ponto, beta_, _delta, Fyf;
    for(int j = 0; j < A_tan.size(); j++)
    {
        A_mag.push_back(hypot(A_tan[j], A_rad[j]));
    }
    
    double A_max = X.bicycle_model.mi * X.bicycle_model.G;
    double Ux, Uy, Fxr, Fxf;
    for(int j = 0; j < A_mag.size(); j++)
    {
        if (A_mag[j] > A_max)             // nominal trajectory total acceleration exceeds friction limit
        {    
            if(abs(A_rad[j]) > A_max)    // nominal trajectory lateral acceleration exceeds friction limit
            {
                A_rad[j] = A_max*sign(A_rad);
                A_tan[j] = 0.0;
                // error("TODO: case when nominal trajectory lateral acceleration exceeds friction limit")
            }else    // prioritize radial acceleration for path tracking; reduce intended acceleration along path to compensate
            {       
                A_tan[j] = sqrt(A_max * A_max - A_rad[j] * A_rad[j]) * sign(A_tan[j]);
            }
        }
        r_ponto.push_back(A_tan[j] * k);

        double i = 1;
        beta_.push_back(beta_zero[j]);
        vector<double> _delta = _delta_zero;
        vector<double> s_delta, c_delta; 
        Fyf.push_back(Fyf_zero[j]);
        while(true)
        {
            double sbeta_ = sin(beta_[j]), cbeta_ = cos(beta_[j]);
            s_delta.push_back(sin(_delta[j]));
            c_delta.push_back(cos(_delta[j]));
            double Ux = V[j] * cbeta_, Uy = V[j] * sbeta_;
            double Fx_drag = -X.bicycle_model.Cd0 - Ux * (X.bicycle_model.Cd1 + X.bicycle_model.Cd2 * Ux);
            double Fx_grade = 0;    // TODO: figure out how roll/pitch are ordered
            double Fy_grade = 0;

            double Ax = A_tan[j] * cbeta_ - A_rad[j] * sbeta_;             // Ax = U̇x - r*Uy
            double Ay = A_tan[j] * sbeta_ + A_rad[j] * cbeta_;             // Ay = U̇y + r*Ux
            double Fx = Ax * X.bicycle_model.m - Fx_drag - Fx_grade;       // Fx = F_til_xf + Fxr = Fxf*c_delta - Fyf*s_delta + Fxr
            Fx = min(Fx, min(X.control_limits.Fx_max, X.control_limits.Px_max/Ux)
                * (X.longitudinal_params.rwd_frac + X.longitudinal_params.fwd_frac * c_delta[j]) - Fyf[j] * s_delta[j]);    // braking force saturated by friction, not engine, limits
            double Fzr = (X.bicycle_model.m * X.bicycle_model.G*X.bicycle_model.a + X.bicycle_model.h * Fx) / X.bicycle_model.L, 
                Fzf = (X.bicycle_model.m * X.bicycle_model.G * X.bicycle_model.b - X.bicycle_model.h * Fx) / X.bicycle_model.L;
            double Fr_max = X.bicycle_model.mi * Fzr, Ff_max = X.bicycle_model.mi * Fzf;
            double aux;
            if(Fx > 0)
            {
                aux = X.longitudinal_params.rwd_frac / (X.longitudinal_params.rwd_frac + X.longitudinal_params.fwd_frac * c_delta[j]);
            }else
            {
                aux = X.longitudinal_params.rwb_frac / (X.longitudinal_params.rwb_frac + X.longitudinal_params.fwb_frac * c_delta[j]);
            }
            Fxr = clamp((Fx + Fyf[j] * s_delta[j]) * aux,
                        -Fr_max, Fr_max);
            double Fyr_max = sqrt(Fr_max * Fr_max - Fxr * Fxr);
            double Fyr = (Ay * X.bicycle_model.m - Fy_grade - r_ponto[j] * X.bicycle_model.Izz / X.bicycle_model.a) 
                / (1 + X.bicycle_model.b / X.bicycle_model.a);
            Fyr = clamp(Fyr, -Fyr_max, Fyr_max);    // TODO: maybe reduce A_tan in the case that Fyr exceeds Fyr_max
            double tan_alphar = _invfialatiremodel(Fyr, X.bicycle_model.c_alphar, Fyr_max);

            double F_til_xf = clamp(Fx - Fxr, -Ff_max, Ff_max);
            double F_til_yfmax = sqrt(Ff_max * Ff_max - F_til_xf * F_til_xf);
            double F_til_yf = clamp((X.bicycle_model.b * Fyr + r_ponto[j] * X.bicycle_model.Izz) / X.bicycle_model.a, -F_til_yfmax, F_til_yfmax);
            Fxf = F_til_xf * c_delta[j] + F_til_yf * s_delta[j];
            Fyf[j] = F_til_yf * c_delta[j] - F_til_xf * s_delta[j];
            double Fyf_max = sqrt(Ff_max * Ff_max - Fxf * Fxf);
            double alpha_f = atan(_invfialatiremodel(Fyf[j], X.bicycle_model.c_alphaf, Fyf_max));
            _delta.push_back(atan2(Uy + X.bicycle_model.a * r[j], Ux) - alpha_f);

            if(i == num_iters)
            {
                Ax = (Fxf * c_delta[j] - Fyf[j] * s_delta[j] + Fxr + Fx_drag + Fx_grade) / X.bicycle_model.m;
                Ay = (Fyf[j] * c_delta[j] + Fxf * s_delta[j] + Fyr + Fy_grade) / X.bicycle_model.m;
                A_tan[j] = Ax * cbeta_ + Ay * sbeta_;
                break;
            }
            i = i + 1;
            beta_[j] = atan(tan_alphar + X.bicycle_model.b * r[j] /Ux);
        }
        double sbeta_ = sin(beta_[j]), cbeta_ = cos(beta_[j]);
        Ux = V[j] * cbeta_, Uy = V[j] * sbeta_;
    }
    vector<double> v_Ux, v_Uy, v_Fxf, v_Fxr;
    v_Ux.push_back(Ux);
    v_Uy.push_back(Uy);
    v_Fxf.push_back(Fxf);
    v_Fxr.push_back(Fxr);

    map<string, vector<double>> map_aux;
    map_aux["beta_"] = beta_;
    map_aux["Ux"] = v_Ux;
    map_aux["Uy"] = v_Uy;
    map_aux["r"] = r;
    map_aux["A"] = A_tan;
    map_aux["_delta"] = _delta;
    map_aux["Fxf"] = v_Fxf;
    map_aux["Fxr"] = v_Fxr;

    return map_aux;
}