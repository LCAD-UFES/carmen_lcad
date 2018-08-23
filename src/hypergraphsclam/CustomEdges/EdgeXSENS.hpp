#ifndef HYPERGRAPHSLAM_EDGE_XSENS_HPP
#define HYPERGRAPHSLAM_EDGE_XSENS_HPP

#include <Eigen/Core>
#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/core/base_unary_edge.h>

namespace g2o {

class EdgeXSENS : public BaseUnaryEdge<1, double, VertexSE2> {

    public:

        // eigen operator
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        // basic constructor
        EdgeXSENS() : BaseUnaryEdge<1, double, VertexSE2>() {

            resize(1);
        }

        void computeError() {

            const VertexSE2* v = dynamic_cast<const VertexSE2*>(_vertices[0]);

            SE2 m(v->estimate());

            // update the rotation
            m.setRotation(Eigen::Rotation2Dd(_measurement));

            SE2 delta = m.inverse() * (v->estimate());

            _error[0] = delta.toVector().norm();

        }

        virtual void setMeasurement(const double &m) {

            _measurement = m;

        }

        virtual bool read(std::istream& is) {

            (void) is;

            return false;
        }

        virtual bool write(std::ostream& os) const
        {

            (void) os;

            return false;

        }

};

    // the odometry calibration edges
typedef std::vector<g2o::EdgeXSENS*> EdgeXSENSPtrVector;

}

#endif
