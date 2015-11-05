#ifndef OMPL_PLANNER_BASE_ODE_H
#define OMPL_PLANNER_BASE_ODE_H

#include <carmen/carmen.h>

#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <iostream>
#include <valarray>
#include <limits>

namespace ob = ompl::base;
namespace oc = ompl::control;

// Kinematic car model object definition.  This class does NOT use ODESolver to propagate the system.
class KinematicCarModel : public oc::StatePropagator
{
    public:
        KinematicCarModel(const oc::SpaceInformationPtr &si) ;

        virtual void propagate(const ob::State *state, const oc::Control* control, const double duration, ob::State *result) const ;

    protected:
        // Explicit Euler Method for numerical integration.
        void EulerIntegration(const ob::State *start, const oc::Control *control, const double duration, ob::State *result) const ;

        /// implement the function describing the robot motion: qdot = f(q, u)
        void ode(const ob::State *state, const oc::Control *control, std::valarray<double> &dstate) const ;

        /// implement y(n+1) = y(n) + d
        void update(ob::State *state, const std::valarray<double> &dstate) const ;

        ob::StateSpacePtr        space_;
        double                   carLength_;
        double                   timeStep_;
};

// Definition of the ODE for the kinematic car.
// This method is analogous to the above KinematicCarModel::ode function.
void KinematicCarODE (const oc::ODESolver::StateType& q, const oc::Control* control, oc::ODESolver::StateType& qdot) ;

// This is a callback method invoked after numerical integration.
void KinematicCarPostIntegration (const oc::Control* /*control*/, ob::State* result) ;

bool isStateValid(const oc::SpaceInformation *si, const ob::State *state) ;

void planWithSimpleSetup(double start_x, double start_y, double start_theta, double goal_x, double goal_y, double goal_theta) ;

#endif
