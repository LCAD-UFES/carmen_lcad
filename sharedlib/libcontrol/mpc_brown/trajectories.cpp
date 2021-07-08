#include <cstdio>
#include <cstdlib>
#include <string>
#include <map>
#include <cmath>
#include <vector>
#include <algorithm>
#include <boost/math/interpolators/cubic_b_spline.hpp>
#include "trajectories.h"
#include "vehicle_dynamics.h"

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
    vector<double> aux;
    for(unsigned int i = 0; i < t.size(); i++)
    {
        aux.insert(aux.end(), t[i]);
    }
    sort(aux.begin(), aux.end());
    for(unsigned int i = 0; i < t.size(); i++) 
    {
        if (num >= aux[i])
        {
            return i;
        }
    }
    return t.size();
}

/*
int
clamp (int x, int lo , int hi){
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
*/
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
/*
void
Base_getindex(TrajectoryTube traj, double s)
{
    i = clamp(searchsortedfirst(traj.s, s, 1, length(traj), Base.Order.ForwardOrdering()) - 1, 1, length(traj)-1)
}
*/
/*function Base.getindex(traj::TrajectoryTube, s)
    i = clamp(searchsortedfirst(traj.s, s, 1, length(traj), Base.Order.ForwardOrdering()) - 1, 1, length(traj)-1)
    A = (traj.V[i+1] - traj.V[i])/(traj.t[i+1] - traj.t[i])    # potentially different from traj.A[i]
    ds = s - traj.s[i]
    if abs(A) < 1e-3 || s > traj.s[end]
        dt = ds/traj.V[i]
    else
        dt = (sqrt(2*A*ds + traj.V[i]^2) - traj.V[i])/A
    end
    t = traj.t[i] + dt
    ti = TimeInterpolants(s, traj.V[i] + A*dt, A)
    si = traj.interp_by_s(s)
    TrajectoryNode(t, ti, si)
end*/
