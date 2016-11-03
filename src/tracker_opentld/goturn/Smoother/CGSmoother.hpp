/*
// Author: josiasalexandre@gmail.com

// Based on the Matt Bradley's Masters Degree thesis and work
// Copyright (c) 2012 Matt Bradley
// CHANGES:
// conjugate gradient optimization implementation, the original work used the simpler descendent gradient method
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of
// this software and associated documentation files (the "Software"), to deal in
// the Software without restriction, including without limitation the rights to
// use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
// the Software, and to permit persons to whom the Software is furnished to do so,
// subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
// FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
// IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
// CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef CONJUGATE_GRADIENT_PATH_SMOOTHER_HPP
#define CONJUGATE_GRADIENT_PATH_SMOOTHER_HPP

#include <vector>

#include "Entities/State2D.hpp"
#include "Entities/Vector2DArray.hpp"
#include "Entities/GridCellIndex.hpp"
#include <carmen/carmen.h>
#include <carmen/collision_detection.h>
//#include <carmen/obstacle_distance_mapper_interface.h>


namespace smoother {

// define the minimizer status
enum CGStatus {CGIddle, CGContinue, CGStuck, CGSuccess, CGFailure};

class CGSmoother {

    private:

        // PRIVATE ATTRIBUTES
	    // car circle to obstacle
		double	circle_radius;
        // the obstacle weight
        double wo;

        // the smooth curvature weight
        double ws;

        // the curvature weight
        double wk;

        // the max obstacle distance
        double dmax;

        // the alpha voronoi parameter
        double alpha;

        // the maximum allowed curvature
        double kmax;

        // THE VEHICLE CONTEXT ATTRIBUTES

        // the min turn radius
        double min_turn_radius;

        // the desired speed
        double max_speed;

        // the max allowed lateral acceleration
        double max_lateral_acceleration;

        // the safety factor
        double safety_factor;

        // THE MINIMIZER CONTEXT ATTRIBUTES

        // the minizer state
        CGStatus cg_status;

        // the input path
        smoother::StateArrayPtr input_path;

        // the solution vector
        smoother::Vector2DArrayPtr x;

        // the cost function evaluated at x
        double fx;

        // the gradient evaluated at x
        smoother::Vector2DArrayPtr gx;

        // the gradient norm - Evaluated at the first point x
        double gx_norm;

        // the solution vector at the next position
        smoother::Vector2DArrayPtr x1;

        // the cost function evaluated at x1
        double fx1;

        // the gradient evaluated at x1
        smoother::Vector2DArrayPtr gx1;

        // the gradient norm - Evaluated at the second point x1
        double gx1_norm;

        // the nest trial solution
        smoother::Vector2DArrayPtr trialx;

        // the best solution function value
        double ftrialx;

        // the best solution gradient
        smoother::Vector2DArrayPtr gtrialx;

        // the best solution gradient norm
        double gtrialx_norm;

        // the displacement between the next and previous gradient
        std::vector<smoother::Vector2D<double> > gx1mgx;

        // the norm of the displacement vector between the old and the best new path
        double x1mx_norm;

        // the norm of the displacement vector between the old and the the trial path
        double trialxmx_norm;

        // the direction vector (Krylov Space)
        std::vector<smoother::Vector2D<double> > s;

        // the direction vector norm
        double s_norm;

        // the directional derivative value
        double sg;

        // store the info about the free and locked positions
        std::vector<bool> locked_positions;

        // how many iterations?
        unsigned int max_iterations;

        // the problem dimension
        unsigned int dim;

        // the next step size
        double step;

        // the default step length
        double default_step_length;

        // the max step size limit
        double stepmax;

        // the min step size limit
        double stepmin;

        // the function tolerance
        double ftol;

        // the gradient tolerance
        double gtol;

        // the solution progress tolerance
        double xtol;

        // the Interpolation context attributes

        // PRIVATE METHODS

        // convert carmen_ackerman_traj_point_t to smoother::State2D
        StateArrayPtr ToState2D(std::vector<carmen_ackerman_traj_point_t> &path);

        // convert the input path to carmen_ackerman_traj_point_t
        std::vector<carmen_ackerman_traj_point_t> FromState2D(smoother::StateArrayPtr);

        // verify if a given path is unsafe
        bool UnsafePath(smoother::Vector2DArrayPtr);

        // get the greater number considering the absolute values
        double ABSMax(double a, double b, double c);

//        // get the obstacle and voronoi contribution
//        smoother::Vector2D<double> GetObstacleDerivative(
//                    const smoother::Vector2D<double>&,
//                    const smoother::Vector2D<double>&,
//                    const smoother::Vector2D<double>&
//                );

        // get the obstacle and voronoi contribution
        // overloaded version
        smoother::Vector2D<double> GetObstacleDerivative(
                    const smoother::Vector2D<double>&,
                    double nearest_obstacle_distance
                );

        // get the curvature contribution
        smoother::Vector2D<double> GetCurvatureDerivative(
                    const smoother::Vector2D<double>&,
                    const smoother::Vector2D<double>&,
                    const smoother::Vector2D<double>&
                );

        // get the smootheness contribution
        smoother::Vector2D<double> GetSmoothPathDerivative(
                    const smoother::Vector2D<double>&,
                    const smoother::Vector2D<double>&,
                    const smoother::Vector2D<double>&,
                    const smoother::Vector2D<double>&,
                    const smoother::Vector2D<double>&
                );

        // build the cost function gradient evaluated at a given input path and returns the gradient's norm
        void ComputeGradient();

        // take a fixed step at the current direction vector (s)
        void TakeStep(double factor);

        // Evaluate the function at the given point
        inline void EvaluateF(
                const smoother::Vector2D<double> &xim1,
                const smoother::Vector2D<double> &xi,
                const smoother::Vector2D<double> &xip1,
                double &obstacle,
                double &smooth,
                double &curvature
                );

        // Evaluate the obstacle, potential field and curvature gradient contributions
        inline void EvaluateG(
                const smoother::Vector2D<double> &xim2,
                const smoother::Vector2D<double> &xim1,
                const smoother::Vector2D<double> &xi,
                const smoother::Vector2D<double> &xip1,
                const smoother::Vector2D<double> &xip2,
                smoother::Vector2D<double> &gradient
                );

        inline void EvaluateG(
				const Vector2D<double> &xim1,
				const Vector2D<double> &xi,
				const Vector2D<double> &xip1,
				Vector2D<double> &gradient);

        // Evaluate the function and the obstacle/potential field gradients
        inline void EvaluateFG(
                const smoother::Vector2D<double> &xim2,
                const smoother::Vector2D<double> &xim1,
                const smoother::Vector2D<double> &xi,
                const smoother::Vector2D<double> &xip1,
                const smoother::Vector2D<double> &xip2,
                double &obstacle,
                double &smooth,
                double &curvature,
                smoother::Vector2D<double> &gradient
                );

        // custom function to evaluate the cost function and the update the gradient at the same time
        // it uses the x1 as the input array
        // the resulting cost value is saved in the internal fx1 variable
        // the gradient vector gx1 is updated and the gradient norm is saved in gx1_norm
        void EvaluateFunctionAndGradient();

        // The purpose of cstep is to compute a safeguarded step for
        // a linesearch and to update an interval of uncertainty for
        // a minimizer of the function.
        // It's the cstep function provided by the minpack library and it is adapted to our context here
        //  Argonne National Laboratory. MINPACK Project. June 1983
        // Jorge J. More', David J. Thuente
        int CStep(
                    double& stl, double& fx, double& sgl,
                    double& stu, double& fu, double& sgu,
                    double& stp, double& fp, double& sgp,
                    bool& bracket, double stmin, double stmax);

        // the More-Thuente line serch
        // based on the minpack and derivates codes
        int MTLineSearch(double lambda);

        // setup the first iteration
        bool Setup(smoother::StateArrayPtr path, bool locked);

        // update the conjugate direction -> s(i+1) = -gradient + gamma * s(i)
        void UpdateConjugateDirection(std::vector<smoother::Vector2D<double> > &s, const std::vector<smoother::Vector2D<double> > &gradient, double gamma);

        // the Polak-Ribiere Conjugate Gradient Method With Moré-Thuente Line Search
        void ConjugateGradientPR(smoother::StateArrayPtr path, bool locked = false);

        // copy the current solution to the input path
        void InputPathUpdate(smoother::Vector2DArrayPtr, smoother::StateArrayPtr);

        // show the current path in the map
        void ShowPath(smoother::StateArrayPtr, bool plot_locked = true);

        // get a bezier point given four points and the time
        inline smoother::Vector2D<double> GetBezierPoint(std::vector<smoother::Vector2D<double> > &points, double t);

        // build a set of control points between the states
        void BuildBezierControlPoints(
                const std::vector<smoother::State2D>&,
                std::vector<smoother::Vector2D<double> > &p1,
                std::vector<smoother::Vector2D<double> > &p2);

        // build a bezier curve passing through a set of states
        void DrawBezierCurve(
                const std::vector<smoother::State2D>&,
                std::vector<smoother::State2D> &,
                const std::vector<smoother::Vector2D<double> > &p1,
                const std::vector<smoother::Vector2D<double> > &p2);

        // interpolate a given path
        smoother::StateArrayPtr Interpolate(smoother::StateArrayPtr);

    public:

        // PUBLIC ATTRIBUTES
        carmen_robot_ackerman_config_t robot_config;
        carmen_obstacle_distance_mapper_message *distance_map;

        // PUBLIC METHODS

        // the basic constructor
        CGSmoother();

        // basic destructor
        ~CGSmoother();

        // smooth a given path
        std::vector<carmen_ackerman_traj_point_t> Smooth(std::vector<carmen_ackerman_traj_point_t> &input_path);
        void set_distance_map(carmen_obstacle_distance_mapper_message *distance_map);
        void set_robot_config(carmen_robot_ackerman_config_t robot_config);
};

}

#endif
