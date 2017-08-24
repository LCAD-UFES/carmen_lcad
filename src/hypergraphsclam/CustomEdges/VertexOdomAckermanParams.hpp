#ifndef G2O_VERTEX_ODOM_ACKERMAN_PARAMS_HPP
#define G2O_VERTEX_ODOM_ACKERMAN_PARAMS_HPP

#include <g2o_types_sclam2d_api.h>
#include <g2o/core/base_vertex.h>

namespace g2o {

// define the odom ackerman vertex
// vector[0] == v multiplier
// vector[1] == phi multiplier bias
// vector[2] == phi additive bias

class VertexOdomAckermanParams: public BaseVertex <3, Eigen::Vector3d> {

    public:

        // basic constructor
        VertexOdomDifferentialParams(double vm = 1.0, double pm = 1.0, double pa = 0.0) : BaseVertex <3, Vector3d>(vm, pm, pa) {}

        virtual void setToOriginImpl() {

          _estimate << 1.0 , 1.0, 0.0;

        }

        virtual void oplusImpl(const double* v) {

            for (int i=0; i<3; i++) {

                _estimate(i) += v[i];

            }

        }

        bool read(std::istream& is) {

            is >> _estimate(0) >> _estimate(1) >> _estimate(2);

            return true;

        }

        bool write(std::ostream& os) const {

            os << estimate()(0) << " " << estimate()(1) << " " << estimate()(2);

            return os.good();

        }

};

}

#endif
