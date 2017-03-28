#ifndef G2O_EDGE_SE2_ODOM_ACKERMAN_CALIB_HPP
#define G2O_EDGE_SE2_ODOM_ACKERMAN_CALIB_HPP

#include <g2o_types_sclam2d_api.h>
#include <odometry_measurement.h>
#include <vertex_odom_differential_params.h>

#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/core/base_multi_edge.h>
#include <VehicleModel.hpp>

namespace g2o {

class EdgeSE2OdomAckermanCalib : public BaseMultiEdge<3, Eigen::Vector3d> {

    public:

        EdgeSE2OdomAckermanCalib() : BaseMultiEdge<3, Eigen::Vector3d>() {

            resize(3);

        }

        // the mair error
        void computeError() {

            const VertexSE2* v1                        = dynamic_cast<const VertexSE2*>(_vertices[0]);
            const VertexSE2* v2                        = dynamic_cast<const VertexSE2*>(_vertices[1]);
            const VertexOdomAckermanParams* params     = dynamic_cast<const VertexOdomAckermanParams*>(_vertices[2]);

            const SE2& x1                              = v1->estimate();
            const SE2& x2                              = v2->estimate();

            // the new v
            double v = measurement()[0] * params->estimate()[0];

            // the new phi
            double phi = measurement()[1] * params->

            // get the calibrated motion given by the odometry
            SE2 odom(VehicleModel::GetOdometryMeasure(measurement()[0] * params->estimate()[0], measurement()[1] * params->estimate()[1]);

            // get the calibrated motion given by the odometry
            VelocityMeasurement calibratedVelocityMeasurment(measurement().vl() * params->estimate()(0),
                measurement().vr() * params->estimate()(1),
                measurement().dt());
            MotionMeasurement mm = OdomConvert::convertToMotion(calibratedVelocityMeasurment, params->estimate()(2));
            SE2 Ku_ij;
            Ku_ij.fromVector(mm.measurement());




            SE2 Ku_ij;

            Ku_ij.fromVector(mm.measurement());

            SE2 delta = Ku_ij.inverse() * (x1.inverse() * x2);

            _error = delta.toVector();

        }

        bool read(std::istream& is) {

            double vl, vr, dt;

            is >> vl >> vr >> dt;

            VelocityMeasurement vm(vl, vr, dt);

            setMeasurement(vm);

            for (int i = 0; i < information().rows(); ++i) {

                for (int j = i; j < information().cols(); ++j) {
                    is >> information()(i, j);
                    if (i != j)
                        information()(j, i) = information()(i, j);
                }
            }

            return true;

        }

        bool write(std::ostream& os) const {

            os << measurement().vl() << " " << measurement().vr() << " " << measurement().dt();
            for (int i = 0; i < information().rows(); ++i) {

                    for (int j = i; j < information().cols(); ++j) {

                        os << " " << information()(i, j);

                    }

            }

            return os.good();

        }

};

} // end namespace

#endif
