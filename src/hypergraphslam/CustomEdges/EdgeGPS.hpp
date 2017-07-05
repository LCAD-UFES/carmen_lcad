#ifndef HYPERGRAPHSLAM_EDGE_GPS_HPP
#define HYPERGRAPHSLAM_EDGE_GPS_HPP

#include <Eigen/Core>
#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/core/base_unary_edge.h>
#include <Wrap2pi.hpp>
#include <VehicleModel.hpp>

namespace g2o {

class EdgeGPS : public BaseUnaryEdge<3, SE2, VertexSE2> {

    private:

        // the gps neighbors
        SE2 left_measure, right_measure;

        // the inverse measurement
        SE2 _inverseMeasurement;

        SE2 _fakeMeasurement;
        SE2 _inverseFakeMeasurement;

        bool valid_gps;

        double kmax;

    public:

        double fake_angle;

        // eigen operator
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        // basic constructor
        EdgeGPS() : BaseUnaryEdge<3, SE2, VertexSE2>() {
            valid_gps = false;
            kmax = hyper::VehicleModel::GetMaxAllowedCurvature();
        }

        void computeError() {

            const VertexSE2* v = dynamic_cast<const VertexSE2*>(_vertices[0]);

            SE2 delta = _inverseFakeMeasurement * (v->estimate());

            _error = delta.toVector();

        }

        virtual void setMeasurement(const SE2 &m) {

            _measurement = _fakeMeasurement = m;
            _inverseMeasurement = m.inverse();

        }

        virtual g2o::SE2 fakeMeasurement() {

            return _fakeMeasurement;

        }

        virtual bool validGPS() { return valid_gps; }

        virtual bool updateNeighbors(EdgeGPS *prev, EdgeGPS *next) {

            const SE2 &xim1(prev->measurement());
            const SE2 &xip1(next->measurement());

            // the partial translation
            Eigen::Vector2d xi_xim1(_measurement.translation() - xim1.translation());
            Eigen::Vector2d xip1_xi(xip1.translation() - _measurement.translation());

            // the xiphi
            double xiphi = std::atan2(xi_xim1[1], xi_xim1[0]);
            double xip1phi = std::atan2(xip1_xi[1], xip1_xi[0]);

            double dphi = mrpt::math::angDistance<double>(xiphi, xip1phi);

            double n = xi_xim1.norm();

            if (0.0001 < n && kmax >= dphi / n) {

                // compute the total translation
                Eigen::Vector2d xip1_xim1(xip1.translation() - xim1.translation());

                // get the current angle
                fake_angle = std::atan2(xip1_xim1[1], xip1_xim1[0]);

                // update the measurement
                _fakeMeasurement.setRotation(fake_angle);

                _inverseFakeMeasurement = _fakeMeasurement.inverse();

                valid_gps = true;

            }

            return valid_gps;

        }

        virtual bool valid_gpsEdge() {

            return valid_gps;

        }

        virtual bool read(std::istream& is) {

            double data[3];

            is >> data[0] >> data[1] >> data[2];

            _measurement = SE2(data[0], data[1], data[2]);

            for (int i = 0; i < 2; i++) {

                for (int j = i; j < 2; j++) {

                    is >> information()(i, j);

                    if (i != j) {

                        information()(j, i) = information()(i, j);

                    }

                }

            }

            return true;
        }

        virtual bool write(std::ostream& os) const
        {
            os << _measurement[0] << " " << _measurement[1] << " " << _measurement[2];

            for (int i = 0; i < 2; ++i) {

                for (int j = i; j < 2; ++j) {

                    os << " " << information()(i, j);

                }

            }

            return os.good();

        }

};

    // the odometry calibration edges
typedef std::vector<g2o::EdgeGPS*> EdgeGPSPtrVector;

}

#endif
