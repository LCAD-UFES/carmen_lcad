#ifndef G2O_EDGE_SE2_ODOM_ACKERMAN_CALIBRATION_HPP
#define G2O_EDGE_SE2_ODOM_ACKERMAN_CALIBRATION_HPP

#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/core/base_unary_edge.h>

#include <VertexOdomAckermanParams.hpp>
#include <VehicleModel.hpp>

namespace g2o {

class EdgeSE2OdomAckermanCalibration : virtual public BaseUnaryEdge<3, SE2, VertexOdomAckermanParams> {

    private:

        // the current velocity
        double v;

        // the current steering angle
        double phi;

        // the current time
        double time;

        // the inverse measurement
        SE2 _inverseMeasurement;

        // the vertices
        g2o::VertexSE2 *left, *right;

        // the current measure between both vertices
        g2o::EdgeSE2 *odom_edge;

    public:

        EdgeSE2OdomAckermanCalibration() :
            BaseUnaryEdge<3, SE2, VertexOdomAckermanParams>(),
            v(0.0), phi(0.0), time(0.0), left(nullptr), right(nullptr), odom_edge(nullptr) {

            // set the raw measure
            _measurement = hyper::VehicleModel::GetOdometryMeasure(v, phi, time);
            _inverseMeasurement = _measurement.inverse();

            // reset the error
            _error[0] = 0.0;
            _error[1] = 0.0;
            _error[2] = 0.0;

        }

        // set the raw values
        void setRawValues(double vel, double steering, double dt) {

            v = vel;
            phi = steering;
            time = dt;

        }

        // set the odometry vertices
        void setVertices(g2o::VertexSE2 *l, g2o::VertexSE2 *r, g2o::VertexOdomAckermanParams *params) {

            left = l;
            right = r;

            // set fixed
            params->setFixed(true);

            // implicit auto upcasting
            _vertices[0] = params;

        }

        // update the odometry edge
        void setOdometryEdge(g2o::EdgeSE2 *edge) {

            // update the odometry edge
            odom_edge = edge;

        }

        // set the measurement
        virtual void setMeasurement(const SE2 &m) {

            // update the measures
            _measurement = m;
            _inverseMeasurement = m.inverse();

        }

        // get the measurement from vertices
        void getMeasurementFromVertices() {

            if (nullptr != left && nullptr != right) {

                // direct access
                const SE2 &x1(left->estimate());
                const SE2 &x2(right->estimate());

                // measure
                _measurement = x1.inverse() * x2;
                _inverseMeasurement = _measurement.inverse();

            } else {

                // error
                throw std::runtime_error("Invalid vertices! It is impossible to obtain a valid measure!");

            }

        }

        // compute current measure
        SE2 getBiasedOdometryMeasure() {

            // get the current vertex
            VertexOdomAckermanParams *params = static_cast<VertexOdomAckermanParams*>(_vertices[0]);

            // get the params direct access
            Eigen::Vector3d bias(params->estimate());

            // compute the new values
            double _v = v * bias[0];
            double _phi = phi * bias[1] + bias[2];

            // compute and return the new measure
            return hyper::VehicleModel::GetOdometryMeasure(_v, _phi, time);

        }

        // update the odometry measure
        void updateOdometryMeasure() {

            if (nullptr != odom_edge) {

                // set the new measure
                _measurement = getBiasedOdometryMeasure();
                _inverseMeasurement = _measurement.inverse();

                // update the odometry edge measure
                odom_edge->setMeasurement(_measurement);

            } else {

                throw std::runtime_error("Error! Invalid odometry edge inside the calibration edge!");

            }

        }

        // the mair error
        void computeError() {

            // compute the new measure
            SE2 new_measure(getBiasedOdometryMeasure());

            // the delta value
            SE2 delta(_inverseMeasurement * new_measure);

            // update the error
            _error = delta.toVector();

        }

        // discards read and write
        virtual bool read(std::istream& is) {

            (void) is;

            return false;

        }

        virtual bool write(std::ostream& os) const {

            (void) os;

            return false;

        }

};


    // the odom ackerman calibration edges
typedef std::vector<g2o::EdgeSE2OdomAckermanCalibration*> EdgeSE2OdomAckermanCalibrationPtrVector;

// the odometry edges
typedef std::vector<g2o::EdgeSE2*> EdgeSE2PtrVector;

} // end namespace

#endif
