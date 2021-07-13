#ifndef COUPLED_LAT_LONG_H
#define COUPLED_LAT_LONG_H


#include <cstdio>
#include <cstdlib>
#include <vector>
#include <string>
#include <cmath>
#include <map>
#include "trajectories.h"
#include "vehicle_dynamics.h"
#include "model_predictive_control.h"
using namespace std;

#ifdef __cplusplus
extern "C" {
#endif

using namespace std;

typedef struct { 
    map<string ,double> CoupledControlParams_map;
}CoupledControlParams;

CoupledControlParams make_CoupledControlParams();


#ifdef __cplusplus
}
#endif

#endif