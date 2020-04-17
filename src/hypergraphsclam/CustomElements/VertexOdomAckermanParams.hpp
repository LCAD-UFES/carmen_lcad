#ifndef G2O_VERTEX_ODOM_ACKERMAN_PARAMS_HPP
#define G2O_VERTEX_ODOM_ACKERMAN_PARAMS_HPP

#include <g2o/core/base_vertex.h>

namespace g2o {

// define the odom ackerman vertex
// vector[0] == v multiplier
// vector[1] == phi multiplier bias
// vector[2] == phi additive bias

class VertexOdomAckermanParams: public BaseVertex<3, Eigen::Vector3d> {

    public:

        // basic constructor
        VertexOdomAckermanParams() : BaseVertex<3, Eigen::Vector3d>() {}

        // explicit constructor
        VertexOdomAckermanParams(unsigned vid, double vm = 1.0, double pm = 1.0, double pa = 0.0) : BaseVertex<3, Eigen::Vector3d>() {

            // set the current vertex id
            _id = vid;

            std::cout << "My id: " << _id << std::endl;

            // update the base vertex
            _estimate << vm, pm, pa;

        }

        virtual void setInitialEstimate(double vm = 1.0, double pm = 1.0, double pa = 0.0) {

            // update the base vertex
            _estimate << vm, pm, pa;

        }

        virtual void setToOriginImpl() {

            _estimate << 1.0, 1.0, 0.0;

        }

        virtual void oplusImpl(const double* update) {

            _estimate[0] += update[0];
            _estimate[1] += update[1];
            _estimate[2] += update[2];

        }

        bool read(std::istream& is) {

            is >> _estimate[0] >> _estimate[1] >> _estimate[2];

            return true;

        }

        bool write(std::ostream& os) const {

            os << _estimate[0] << " " << _estimate[1] << " " << _estimate[2];

            return os.good();

        }

        virtual bool setEstimateDataImpl(const double* est){

            _estimate[0] = est[0];
            _estimate[1] = est[1];
            _estimate[2] = est[2];

            return true;
        }

        virtual bool getEstimateData(double* est) const{

            est[0] = _estimate[0];
            est[1] = _estimate[1];
            est[2] = _estimate[2];

            return true;

        }

        virtual int estimateDimension() const {

            return 3;

        }

        virtual bool setMinimalEstimateDataImpl(const double* est){

            return setEstimateData(est);

        }

        virtual bool getMinimalEstimateData(double* est) const{

            return getEstimateData(est);

        }

        virtual int minimalEstimateDimension() const {

            return 3;

        }

};

}

#endif
