#include <VehicleModel.hpp>

using namespace hyper;

// set the base parameters
double VehicleModel::max_steering_angle = 0.5337;
double VehicleModel::axle_distance = 2.61874;
double VehicleModel::understeer = 0.0015;
double VehicleModel::kmax = 0.2;

// base constructor
VehicleModel::VehicleModel()
{}

// read the vehicle parameters from file
void VehicleModel::SetParameters(double msa, double ald, double und, double k)
{
    VehicleModel::max_steering_angle = msa;
    VehicleModel::axle_distance = ald;
    VehicleModel::understeer = und;
    VehicleModel::kmax = k;
}

// get the odometry measure
g2o::SE2 VehicleModel::GetOdometryMeasure(double v, double phi, double dt)
{
    double length = v * dt;
    double dx = 0.0, dy = 0.0, dtheta = 0.0;
    if (0.0 != phi)
    {
        double tr = VehicleModel::axle_distance / std::tan(phi / (1.0 + v * v * VehicleModel::understeer));
        // double tr = VehicleModel::axle_distance / std::tan(phi);
        dtheta = length / tr;

        double dtheta_2 = dtheta * 0.5;
        double sintheta = std::sin(dtheta_2);
        double L = 2.0 * sintheta * tr;

        dx = L * std::cos(dtheta_2);
        dy = L * sintheta;
    }
    else
    {
        dx = length;
    }
    return g2o::SE2(dx, dy, dtheta);
}

// get the next vehicle state
g2o::SE2 VehicleModel::NextState(g2o::SE2 prev, double v, double phi, double dt)
{
    return prev * GetOdometryMeasure(v, phi, dt);
}

// get the max allowed curvature
double VehicleModel::GetMaxAllowedCurvature()
{
    return kmax;
}
