#ifndef OMPL_PLANNER_BASE_H
#define OMPL_PLANNER_BASE_H

#include <carmen/carmen.h>

#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

void planTrajectory(const carmen_map_t &map, ob::StateSpacePtr space, const ob::ScopedState<> &start, const ob::ScopedState<> &goal);

void printTrajectory(ob::StateSpacePtr space, const ob::ScopedState<ob::SE2StateSpace> &goal);

#endif
