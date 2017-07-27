#ifndef HYPERGRAPHSLAM_VEHICLE_CURVATURE_CONSTRAINT_HPP
#define HYPERGRAPHSLAM_VEHICLE_CURVATURE_CONSTRAINT_HPP

#include <g2o/core/base_multi_edge.h>
#include <g2o/types/slam2d/vertex_se2.h>

namespace g2o {

class EdgeCurvatureConstraint : public BaseMultiEdge<3, Eigen::Vector3d> {

    protected:

        // the kmax value
        static constexpr double kmax = 0.22;

        // the wk value
        static constexpr double wk = 1.0;

    public:

        // eigen new operator
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        // the base constructor
        EdgeCurvatureConstraint() {

            // resize the base edge
            resize(3);

            // set the error values to zero
            _error(0,0) = 1e03;
            _error(1,0) = 1e03;
            _error(2,0) = 1e03;

        }

        // get curvature value
        Eigen::Vector2d GetCurvatureDerivative(const Eigen::Vector2d &xim1, const Eigen::Vector2d &xi, const Eigen::Vector2d &xip1) {

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
            const Eigen::Vector2d xim1(v1->estimate().translation());
            const Eigen::Vector2d xi(v2->estimate().translation());
            const Eigen::Vector2d xip1(v3->estimate().translation());

            // get the current displacement
            const Eigen::Vector2d dxi(xi - xim1);

            // get the next displacement
            const Eigen::Vector2d dxip1(xip1 - xi);

            // conpute the inverse norm
            double dxi_norm = dxi.norm();

            // compute the curvature error
            double k_error = 0.0 != dxi_norm ? std::fabs(std::atan2(dxip1(1, 0), dxip1(0, 0)) - std::atan2(dxi(1, 0), dxi(0, 0))) / dxi_norm : 0.0;

            // verify the curvature constraint
            k_error = kmax < k_error ? k_error - kmax : 0.0;

            // the current norm
            _error(0, 0) = k_error;
            _error(1, 0) = k_error;
            _error(2, 0) = k_error;

            //std::cout << "Error: " << _error.transpose() << std::endl;
            // _error(2, 0) = 0.0;

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
