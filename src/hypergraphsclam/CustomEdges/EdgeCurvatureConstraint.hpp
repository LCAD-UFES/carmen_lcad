#ifndef HYPERGRAPHSLAM_VEHICLE_CURVATURE_CONSTRAINT_HPP
#define HYPERGRAPHSLAM_VEHICLE_CURVATURE_CONSTRAINT_HPP

#include <limits>

#include <g2o/core/base_multi_edge.h>
#include <g2o/types/slam2d/vertex_se2.h>

#include <Wrap2pi.hpp>

namespace g2o {

class EdgeCurvatureConstraint : public BaseMultiEdge<3, SE2> {

    protected:

        // the kmax value
        static constexpr double kmax = 0.22;

        // the wk value
        static constexpr double wk = 0.5;

        // the delta time
        double dt;

    public:

        // eigen new operator
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        // the base constructor
        EdgeCurvatureConstraint() : BaseMultiEdge<3, SE2>(), dt(0.0) {

            // resize the base edge
            resize(3);

        }


        void setTimeDifference(double _t) {

            dt = _t;

        }

        // get curvature value
        Eigen::Vector2d getCurvatureDerivative(const Eigen::Vector2d &xim1, const Eigen::Vector2d &xi, const Eigen::Vector2d &xip1) {

            // the resulting derivative
            Eigen::Vector2d res(0.0, 0.0);

            // get the dxi
            Eigen::Vector2d dxi(xi(0, 0) - xim1(0, 0), xi(1, 0) - xim1(1, 0));
            Eigen::Vector2d dxip1(xip1(0, 0) - xi(0, 0), xip1(1, 0) - xi(1, 0));

            // xi norm
            double dxi_norm = dxi.norm();

            // get the delta phi value
            double dphi = std::acos(std::max(-1.0, std::min((dxi(0, 0) * dxip1(0, 0) + dxi(1, 0) * dxip1(1, 0)) / (dxi_norm * dxip1.norm()), 1.0)));

            // get the curvature
            double k = dphi / dxi_norm;

            // if the curuvature k is greater than kmax then we need to add the curvature contribution
            // otherwise we set the curvature term equals to zero
            if (kmax < k) {

                // get the derivative of delta phi with respect the cosine of delta phi
                double ddphi = -1.0/std::sqrt(1.0 - std::pow(std::cos(dphi), 2));

                // the common denominator
                double inverse_denom = 1.0/(xi.norm() * xip1.norm());

                // the normalized orthogonal complements
                Eigen::Vector2d p1, p2;

                // some helpers
                double inverse_norm2 = 1.0/xip1.squaredNorm();
                double nx = -xip1(0, 0);
                double ny = -xip1(1, 0);
                double tmp = (xi(0, 0) * nx + xi(1, 0) * ny);
                double tmp1 = tmp * inverse_norm2;

                // set the first othogonal complement
                p1(0, 0) = (xi(0, 0) - nx * tmp1) * inverse_denom;
                p1(1, 0) = (xi(1, 0) - ny * tmp1) * inverse_denom;

                // reset the helpers
                inverse_norm2 = 1.0/xi.norm();
                tmp1 = tmp * inverse_norm2;

                // set the second othogonal complement
                p2(0, 0) = (nx - xi(0, 0) * tmp1) * inverse_denom;
                p2(1, 0) = (ny - xi(1, 0) * tmp1) * inverse_denom;

                // get common term in all three points
                double coeff1 = (-1.0 / dxi_norm) * ddphi;
                double coeff2 = dphi / dxi.squaredNorm();

                // reuse the k variable to get the first part of the derivative
                k = 2 * (k - kmax);

                Eigen::Vector2d ki, kim1, kip1;

                ki = (p1 + p2);

                ki(0, 0) *= -coeff1;
                ki(1, 0) *= -coeff1;

                // subtractr
                ki(0, 0) -= coeff2;
                ki(1, 0) -= coeff2;

                // apply the factor
                ki(0, 0) *= 0.5 *k;
                ki(1, 0) *= 0.5 *k;

                kim1 = p2;

                // multiply
                kim1(0, 0) *= coeff1;
                kim1(1, 0) *= coeff1;

                // add a scallar
                kim1(0, 0) += coeff2;
                kim1(1, 0) += coeff2;

                // aply the factor
                kim1(0, 0) *= 0.25 *k;
                kim1(1, 0) *= 0.25 *k;

                kip1 = p1;

                // aply the factor
                kip1(0, 0) *= coeff1 * 0.25 * k;
                kip1(1, 0) *= coeff1 * 0.25 * k;

                // add the curvature contribution
                res(0, 0) = wk * (kim1(0, 0) + ki(0, 0) + kip1(0, 0));
                res(1, 0) = wk * (kim1(1, 0) + ki(1, 0) + kip1(1, 0));

            }

            return res;

        }

        // the main error function
        void computeError() {

            // the pose estimates
            const VertexSE2* v1 = static_cast<const VertexSE2*>(_vertices[0]);
            const VertexSE2* v2 = static_cast<const VertexSE2*>(_vertices[1]);
            const VertexSE2* v3 = static_cast<const VertexSE2*>(_vertices[2]);

            // the estimates
            const SE2 &xim1(v1->estimate());
            const SE2 &xi(v2->estimate());
            const SE2 &xip1(v3->estimate());

            SE2 xim1_inverse(xim1.inverse());

            // the angle
            SE2 total_delta(xim1_inverse * xip1);

            // the current measure
            SE2 partial_delta(total_delta.toVector() * dt);

            // the translation vector
            Eigen::Vector2d translation(total_delta.translation());

            // update the partial angle
            partial_delta.setRotation(std::atan2(translation[1], translation[0]));

            // compute the error
            SE2 delta((xim1_inverse * xi).inverse() * partial_delta);

            // get the error
            _error = delta.toVector() * wk;

        }

        //
        virtual bool read(std::istream& is) {

            for (int i = 0; i < information().rows(); ++i) {

                for (int j = i; j < information().cols(); ++j) {

                    // get the information values
                    is >> information()(i, j);

                }

            }

            return true;

        }

        virtual bool write(std::ostream& os) const {

            for (int i = 0; i < information().rows(); ++i) {

                for (int j = i; j < information().cols(); ++j) {

                    // append the information matrix values
                    os << information()(i, j);

                }

            }

            return os.good();

        }


};

} // end namespace

#endif
