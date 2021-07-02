#include <cstdio>
#include <cstdlib>
#include <string>
#include <map>
#include <cmath>
#include <vector>
#include <algorithm>
#include <boost/numeric/ublas/banded.hpp>
#include <boost/numeric/ublas/io.hpp>

using namespace boost::numeric::ublas;

typedef struct {
    double V_min;
    double V_max;
    double k_V;
    double k_s;
    double delta_max;
    double Q_delta_phi;
    double Q_e;
    double W_beta;
    double W_r;
    double R_delta;
    double R_tri_delta;
}DecoupledControlParams;

DecoupledControlParams
make_decoupled_control_params()
{
    DecoupledControlParams dcp;
    dcp.V_min = 1.0;
    dcp.V_max = 15.0;
    dcp.k_V = 10 / 4 / 100;
    dcp.k_s = 10 / 4 / 10000;
    dcp.delta_max = 0.344;
    dcp.Q_delta_phi = 1 / pow((10 * M_PI / 180), 2);
    dcp.Q_e = 1.0;
    dcp.W_beta = 50 / (10 * M_PI / 180);
    dcp.W_r = 50.0;
    dcp.R_delta = 0.0;
    dcp.R_tri_delta = 0.01 / pow(10 * M_PI / 180, 2);
    return dcp;
}

typedef struct{
    diagonal_matrix<double> Q_delta_phi;
}LateralTrackingQPParams;
