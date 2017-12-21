#ifndef VIRTUAL_SCAN_PARAMETERS_H
#define VIRTUAL_SCAN_PARAMETERS_H

#include <cstddef>

namespace virtual_scan
{

/** @brief Maximum distance between two points for them to be considered part of a segment. */
extern const double D_SEGMENT;

/** @brief Maximum angle difference from a point to the tip of a segment before it's considered a new segment. */
extern const double O_SEGMENT;

extern const double D_MAX;

/** @brief Maximum obstacle speed in m/s. */
extern const double V_MAX;

/** @brief Length of the time window in time steps. */
extern const size_t T;

/** @brief Number of iterations in a MCMC cycle. */
extern const size_t N_MC;

extern const double LAMBDA_L;

extern const double LAMBDA_T;

/** @brief Weight factor for the distance between a sensor reading and the obstacle border it should be on. */
extern const double LAMBDA_1;

extern const double LAMBDA_2;

extern const double LAMBDA_3;

extern const double LAMBDA_4;

} // namespace virtual_scan

#endif
