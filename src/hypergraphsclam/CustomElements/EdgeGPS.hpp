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

        // helpers
        SE2 _fakeMeasurement;
        SE2 _inverseFakeMeasurement;

        bool valid_gps;

        double kmax;

        double max_angle_diff;

    public:

        double fake_angle;

        double real_angle;

        // eigen operator
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        // basic constructor
        EdgeGPS() : BaseUnaryEdge<3, SE2, VertexSE2>(),
            valid_gps(true),
            kmax(hyper::VehicleModel::GetMaxAllowedCurvature()),
            max_angle_diff(M_PI_4 * 0.25) {}

        void computeError() {

            const VertexSE2* v = dynamic_cast<const VertexSE2*>(_vertices[0]);

            SE2 delta = _inverseFakeMeasurement * (v->estimate());
            // SE2 delta = _inverseMeasurement * (v->estimate());

            _error = delta.toVector();

        }

        virtual void setMeasurement(const SE2 &m) {

            _measurement = m;
            _fakeMeasurement = m;
            _inverseMeasurement = m.inverse();
            _inverseFakeMeasurement = _inverseMeasurement;
            real_angle = fake_angle = m.rotation().angle();

        }

        virtual g2o::SE2 fakeMeasurement() {

            return _fakeMeasurement;

        }

        virtual bool validGPS() { return valid_gps; }

        virtual void computeFakeOrientation(const SE2 &r) {

            // get the translation diff
            Eigen::Vector2d xip1_xi(r.translation() - _measurement.translation());

            // compute the angle
            fake_angle = std::atan2(xip1_xi[1], xip1_xi[0]);

            // set the fake measurement
            _fakeMeasurement.setRotation(Eigen::Rotation2Dd(fake_angle));

            // set the fake translation
            _fakeMeasurement.setTranslation(_measurement.translation());

            // update the fake inverse measurement
            _inverseFakeMeasurement = _fakeMeasurement.inverse();

            // update the g2o default measurement values
            _measurement = _fakeMeasurement;
            _inverseFakeMeasurement = _inverseMeasurement;

        }

        virtual void computeFakeOrientation(const SE2 &l, const SE2 &r) {

            // get the translation diff
            Eigen::Vector2d xip1_xim1(r.translation() - l.translation());

            // compute the angle
            fake_angle = std::atan2(xip1_xim1[1], xip1_xim1[0]);

            // set the fake measurement
            _fakeMeasurement.setRotation(Eigen::Rotation2Dd(fake_angle));

            // set the fake translation
            _fakeMeasurement.setTranslation(_measurement.translation());

            // update the fake inverse measurement
            _inverseFakeMeasurement = _fakeMeasurement.inverse();

            // update the g2o default measurement values
            _measurement = _fakeMeasurement;
            _inverseFakeMeasurement = _inverseMeasurement;

        }

        // update neighbors
        virtual void validate(EdgeGPS *prev_gps, EdgeGPS *next_gps) {

            return;
            // direct access
            double prev_fa = prev_gps->fake_angle;
            double next_fa = next_gps->fake_angle;

            // angle distance
            double prev_diff = std::fabs(mrpt::math::angDistance<double>(fake_angle, prev_fa));
            double next_diff = std::fabs(mrpt::math::angDistance<double>(fake_angle, next_fa));

            if (max_angle_diff < prev_diff || max_angle_diff < next_diff) {

                // get the translation diff
                Eigen::Vector2d xip1_xi(next_gps->measurement().translation() - _measurement.translation());
                double current_angle = std::atan2(xip1_xi[1], xip1_xi[0]);

                double current_diff = std::fabs(mrpt::math::angDistance<double>(real_angle, current_angle));
                double real_prev_diff = std::fabs(mrpt::math::angDistance<double>(real_angle, prev_gps->real_angle));
                double real_next_diff = std::fabs(mrpt::math::angDistance<double>(real_angle, next_gps->real_angle));
                double real_diff = real_prev_diff + real_next_diff;
                double fake_diff = prev_diff + next_diff;

                fake_angle = real_diff < fake_diff && max_angle_diff > current_diff ? real_angle : prev_fa;

                _fakeMeasurement.setRotation(Eigen::Rotation2Dd(fake_angle));

                _inverseFakeMeasurement = _fakeMeasurement.inverse();

                // update the g2o default measurement values
                _measurement = _fakeMeasurement;
                _inverseMeasurement = _inverseFakeMeasurement;

            }

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
