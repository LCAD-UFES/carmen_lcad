#ifndef MODEL_PREDICTIVE_CONTROL_H
#define MODEL_PREDICTIVE_CONTROL_H


#include <cstdio>
#include <cstdlib>
#include <vector>
#include <string>
#include <map>

using namespace std;

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    int N_short;
    int N_long;
    double dt_short;
    double dt_long;
    bool use_correction_step;
    vector<double> prev_ts;
    vector<double>  ts;
    vector<double> dt;
}MPCTimeSteps;

MPCTimeSteps
make_MPCTimeSteps(int N_short, int N_long, double dt_short, double dt_long, bool use_correction_step);

void
compute_time_steps(MPCTimeSteps TS, double t0);

typedef struct
{
    map<string, double> vehicle;
    int heartbeat;
    double time_offset;
    bool solved;
}TrajectoryTrackingMPC;

#ifdef __cplusplus
}
#endif

#endif