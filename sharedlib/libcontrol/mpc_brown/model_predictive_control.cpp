#include <cstdio>
#include <cstdlib>
#include <string>
#include <map>
#include <cmath>
#include <vector>
#include <algorithm>
#include "model_predictive_control.h"
#include <boost/math/interpolators/cubic_b_spline.hpp>

using namespace std;


MPCTimeSteps
make_MPCTimeSteps(int N_short, int N_long, double dt_short, double dt_long, bool use_correction_step)
{
    int N = 1 + N_short + N_long;
    vector<double> ts, dt;
    for (int i = 0; i < N; i++)
    {
        double aux = (double)rand() / RAND_MAX;
        ts.push_back(aux);
    }
    for (int i = 0; i < N - 1; i = i + 2)
    {
        dt.push_back(ts[i +1] - ts[i]);
    }
    
    MPCTimeSteps MPCts;
    MPCts.ts = ts;
    MPCts.dt = dt;
    MPCts.N_short = N_short;
    MPCts.N_long = N_long;
    MPCts.dt_short = dt_short;
    MPCts.dt_long = dt_long;
    MPCts.use_correction_step = use_correction_step;
    MPCts.ts = ts;
    return MPCts;
}

void
compute_time_steps(MPCTimeSteps TS, double t0)
{
    //TS.prev_ts = TS.ts;
    TS.prev_ts.clear();
    for (int i = 0 ; i < TS.prev_ts.size(); i ++ )
    {
        TS.prev_ts.push_back(TS.ts[i]);
    }
    int size_ts = TS.ts.size();
    TS.ts.clear();
    double  t0_long = t0 + TS.N_short * TS.dt_short;
    if(TS.use_correction_step)
    {
        t0_long = TS.dt_long * ceil((t0_long + TS.dt_short) / TS.dt_long - 1);    // dt_correction ∈ [dt_short, dt_long+dt_short)
    }
    vector<double> aux;
    for (int i = 0; i <= TS.N_short; i ++)
    {
        TS.ts.push_back((TS.dt_short * i) + t0);
    }
    for (int i = TS.N_short +1 ; i <= size_ts; i ++)
    {
        TS.ts.push_back((TS.dt_long * i) + t0_long);
    }
    for (int i = 0; i <= TS.N_short; i ++)
    {
        TS.ts.push_back((TS.dt_short * i) + t0);
    }
    TS.dt.clear();
    for (int i = 0; i < TS.N_short+TS.N_long; i++)
    {
        TS.dt.push_back(TS.ts[i+1] - TS.ts[i]);
    }
}

