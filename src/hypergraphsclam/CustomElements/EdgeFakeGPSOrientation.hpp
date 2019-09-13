#ifndef HYPERGRAPHSLAM_EDGE_FAKE_GPS_ORIENTATION_HPP
#define HYPERGRAPHSLAM_EDGE_FAKE_GPS_ORIENTATION_HPP

#include <Eigen/Core>
#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/core/base_multi_edge.h>

namespace g2o {

class EdgeFakeGPSOrientation : public BaseMultiEdge<3, SE2> {

    private:

        // the allowed curvature
        static constexpr double max_curvature = 0.22;

    public:

        // eigen operator
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        // basic constructor
        EdgeFakeGPSOrientation() : BaseMultiEdge<1, double>() {

            resize(3);
        }

        void computeError() {

            // get the angle between the fixed vertices
            const VertexSE2* v1 = dynamic_cast<const VertexSE2*>(_vertices[0]);
            const VertexSE2* v2 = dynamic_cast<const VertexSE2*>(_vertices[1]);
            const VertexSE2* v3 = dynamic_cast<const VertexSE2*>(_vertices[2]);

            // the estimates
            const SE2 &prev(v1->estimate());
            const SE2 &curr(v2->estimate());
            const SE2 &next(v3->estimate());

            // get the transformation from prev to next
            SE2 ptn(prev.inverse() * next);

            // the translation vector
            Eigen::Vector2d translation(ptn.translation());

            // the desired angle
            double theta = std::atan2(translation[1], translation[0]);

            SE2 delta(0.0, 0.0, mrpt::math::angDistance<double>(theta, curr.rotation().angle()););

            _error = delta.toVector();

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
