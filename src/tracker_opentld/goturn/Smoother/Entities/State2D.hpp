#ifndef STATE_2D_HPP
#define STATE_2D_HPP

#include <list>
#include <vector>

#include "../Helpers/wrap2pi.hpp" //TODO vou mudar depois =D
#include "Pose2D.hpp"

namespace smoother {

class State2D : public smoother::Pose2D {

    public:

        // PUBLIC ATTRIBUTES

        // the current speed
        double v;

        // the steering angle
        double phi;

        // the command time
        double t;

        // simple constructor
        State2D();

        // simple constructor, the input is a pose
        State2D(
                const smoother::Pose2D& pose,
                double vel = 0.0,
                double wheel_angle = 0.0,
                double t_ = 0.0,
            );

        // copy constructor
        State2D(const astar::State2D&);

        // distance between two poses
        double Distance(const State2D&);

        // distance squared between two poses
        double Distance2(const State2D&);

        // get difference between orientations (yaw)
        double GetOrientationDiff(const State2D&);

        // get difference between orientations (yaw), overloaded version
        double GetOrientationDiff(double);

        // the assignment operator
        void operator=(const State2D&);

        // alternative assignement operator
        void operator=(const Pose2D&);

        // == operator overloading
        bool operator==(const State2D&);

        // != operator overloading
        bool operator!=(const State2D&);

};

// a helper class to avoid copies
class StateList {

public:

    // EVERYTHING PUBLIC
    std::list<State2D> states;

};

typedef StateList* StateListPtr;

//
class StateArray {

public:

    // EVERYTHING PUBLIC
    std::vector<State2D> states;

    // default constructor
    StateArray();

    StateArray(unsigned int size) : states(size) {}

};

typedef StateArray* StateArrayPtr;
typedef StateArray& StateArrayRef;

}

#endif
