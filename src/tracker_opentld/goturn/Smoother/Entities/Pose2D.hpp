#ifndef HYBRID_ASTAR_POSE_2D_HPP
#define HYBRID_ASTAR_POSE_2D_HPP

#include <vector>
#include <list>

#include "Vector2D.hpp"

namespace smoother {

class Pose2D {

    public:

        // the position
		smoother::Vector2D<double> position;

        // the current orientation
        double orientation;

        // most basic constructor
        Pose2D();

        // copy constructor
        Pose2D(const Pose2D&);

        // explicit constructor
        Pose2D(const Vector2D<double>&, double);

        // explicit constructor
        Pose2D(double, double, double);

        // assignment operator overloading
        void operator=(const Pose2D&);

        // equals to operator overloading
        bool operator==(const Pose2D&);

        // different operator overloading
        bool operator!=(const Pose2D&);

};

class PoseArray {

    public:

        // the pose array
        std::vector<Pose2D> poses;

};

typedef PoseArray* PoseArrayPtr;

class PoseList {

    public:

        // the pose list
        std::list<Pose2D> poses;

};

typedef PoseList* PoseListPtr;

}
#endif
