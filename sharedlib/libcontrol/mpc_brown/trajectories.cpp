#include <cstdio>
#include <cstdlib>
#include <string>
#include <map>
#include <cmath>
#include <vector>
#include <algorithm>
#include <boost/math/interpolators/cubic_b_spline.hpp>
#include "trajectories.h"

using namespace std;

TrajectoryNode
construct_TrajectoryNode()
{
    TrajectoryNode traj;
    return traj;
}

int
searchsortedfirst (vector<double> t, double num)
{
    for(unsigned int i = 0; i < t.size(); i++)
    {
        if(t[i] >= num)
        {
            return i;
        }
    } 
    return -1;
}


void 
t_function (TrajectoryTube traj, double t)
{
    int i = clamp(searchsortedfirst(traj.t, t), 1, traj.t.size() - 2);
    double A = (traj.V[i+1] - traj.V[i])/(traj.t[i+1] - traj.t[i]);    // potentially different from traj.A[i]
    double dt = t - traj.t[i];
    TimeInterpolants ti;
    ti.TimeInterpolants_map["s"] = traj.s[i] + traj.V[i] * dt + A * dt * dt/2;
    ti.TimeInterpolants_map["V"] = traj.V[i] + A * dt;
    ti.TimeInterpolants_map["A"] = A;
    //si = traj.interp_by_s(ti.s)
    //boost::math::cubic_b_spline<double> spline(ti.TimeInterpolants_map["s"]);
    //construct_TrajectoryNode(t, ti, si)*/

}

TrajectoryNode
Traj_getindex_s(TrajectoryTube traj, double s)
{
    int i = int(clamp(searchsortedfirst(traj.s, s) - 1, 1, traj.s.size() - 1));
    double A = (traj.V[i + 1] - traj.V[i]) / (traj.t[i + 1] - traj.t[i]);    // potentially different from traj.A[i]
    double ds = s - traj.s[i]; 
    double dt;
    if(abs(A) < 1e-3 || s > traj.s[traj.s.size()]){
        dt = ds/traj.V[i];
    }else{
        dt = (sqrt(2 * A * ds + pow(traj.V[i], 2)) - traj.V[i]) / A;
    }
    double t = traj.t[i] + dt;
    TimeInterpolants ti;
    ti.TimeInterpolants_map["s"] = s;
    ti.TimeInterpolants_map["V"] = traj.V[i] + A * dt;
    ti.TimeInterpolants_map["A"] = A;
    TrajectoryNode tn;
    tn.t = t;
    tn.ti = ti;
    //si = traj.interp_by_s(s)
    return tn;
}


vector<double> 
operator*(const vector<double>& v1, const vector<double>& v2)
{
    vector<double> aux;
    for(unsigned int i = 0; i < v1.size(); i++)
    {
        aux.push_back(v1[i] * v2[i]);
    }
    return aux;
}

vector <double> 
operator-(const vector<double>& v1, const vector<double>& v2)
{
    vector<double> aux;
    for(unsigned int i = 0; i < v1.size(); i++)
    {
        aux.push_back(v1[i] - v2[i]);
    }
    return aux;
}

vector<double> 
operator+(const vector<double>& v1, const vector<double>& v2)
{
    vector<double> aux;
    for(unsigned int i = 0; i < v1.size(); i++)
    {
        aux.push_back(v1[i] + v2[i]);
    }
    return aux;
}



vector <double> 
distance2(double a, double b, double c, double d, BicycleState x)
{
    double v =  d - b;
    double v1 = c - a; 
    vector<double> lambda;
    
    lambda.push_back(clamp(v1 * (x.E[0] - a) / (v1 * v1), 0, 1));
    lambda.push_back(clamp(v * (x.N[0] - b) / (v * v), 0, 1));

    vector<double> aux;
    for (unsigned int i = 0 ; i< lambda.size(); i ++)
    {
        aux.push_back(1 - lambda[i]);
    }
    vector<double> p;
    p.push_back(aux[0] * a + lambda[0] * c);
    p.push_back(aux[1] * b + lambda[1] * d);

    vector <double> result ;
    for (unsigned int i = 0; i < p.size(); i ++)
    {
        if (i% 2 == 0)
        {
            result.push_back((p[i] - x.E[0]) * (p[i] - x.E[0]));
        }else
        {
            result.push_back((p[i] - x.N[0]) * (p[i] - x.E[0]));
        }
        
    }
    return result;
}

vector<vector<double>>
path_coordinates(TrajectoryTube traj, BicycleState x)
{
    int imin = 0;
    vector<double> d2min, d2;
    for (unsigned int i = 0; i < traj.t.size() - 1; i ++)
    {
        d2min.push_back(INFINITY);
    }
    for (unsigned int i = 0; i < traj.t.size() - 1; i ++)
    {
        d2 = distance2(traj.E[i], traj.N[i], traj.E[i+1], traj.N[i+1], x);
        if(d2 < d2min)
        {
            d2min.clear();
            for(unsigned int i; i < d2.size(); i ++)
            {
                d2min.push_back(d2[i]);
            }
            imin = i;
        }
    }
    int i = imin;
    vector<double> v;
    v.push_back(traj.E[i+1] - traj.E[i]);
    v.push_back(traj.N[i+1] - traj.N[i]);
    vector<double> w;
    w.push_back(x.E[0] - traj.E[i]);
    w.push_back(x.N[0] - traj.N[i]);
    vector<double> ds; 
    for (unsigned int j = 0; j < d2min.size(); j++)
    {
        ds.push_back(sqrt(w[j]*w[j] - d2min[j]));
    }
    vector<double> s;
    for (unsigned int j = 0; j < ds.size(); j++)
    {
        s.push_back(traj.s[i] + ds[j]);
    }
    vector<double> cross;
    cross.push_back(v[1] - w[1]);
    cross.push_back(w[0] - v[0]);
    vector<double> e;
    for(unsigned int j = 0; j < d2min.size(); j++)
    {
        e.push_back(sqrt(d2min[j]));
    }
    double A = (traj.V[i+1] - traj.V[i])/(traj.t[i+1] - traj.t[i]);    // potentially different from traj.A[i]
    vector<double> dt;
    if(abs(A) < 1e-3)
    {
        for(unsigned int j = 0 ; j < ds.size(); j ++)
        {
            dt.push_back(ds[j] / traj.V[i]);
        }
    }else
    {
        for(unsigned int j = 0 ; j < ds.size(); j ++)
        {
            dt.push_back((sqrt(2 * A * ds[j] + pow(traj.V[i], 2)) - traj.V[i]) / A);
        }
    }
    vector<double> t;
    
    for(unsigned int j = 0 ; j < dt.size(); j ++)
    {
        t.push_back(traj.t[i] + dt[j]);
    }
    vector<vector<double>> matrix;
    matrix.push_back(s);
    matrix.push_back(e);
    matrix.push_back(t);
    return matrix;
}