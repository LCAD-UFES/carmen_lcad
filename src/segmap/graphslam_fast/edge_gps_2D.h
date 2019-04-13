#ifndef _EDGE_GPS_2D_H_
#define _EDGE_GPS_2D_H_

#include <Eigen/Core>
#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/core/base_unary_edge.h>
#include <stdio.h>
#include <math.h>
#include <float.h>

using namespace Eigen;

namespace g2o
{
	class EdgeGPS : public BaseUnaryEdge<3, SE2, VertexSE2>
	{
		public:
			// eigen operator
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

			EdgeGPS() : BaseUnaryEdge<3, SE2, VertexSE2>()
			{
			}

			void computeError()
			{
				const VertexSE2* v = dynamic_cast<const VertexSE2*>(_vertices[0]);
				SE2 delta = _measurement.inverse() * (v->estimate());
				_error = delta.toVector();
			}

			virtual bool read(std::istream& is)
			{
				double data[3];
				is >> data[0] >> data[1] >> data[2];
				_measurement = SE2(data[0], data[1], data[2]);

				for (int i = 0; i < 2; i++)
				{
					for (int j = i; j < 2; j++)
					{
						is >> information()(i, j);

						if (i != j)
							information()(j, i) = information()(i, j);
					}
				}

				return true;
			}

			virtual bool write(std::ostream& os) const
			{
				os << _measurement[0] << " " << _measurement[1] << " " << _measurement[2];

				for (int i = 0; i < 2; ++i)
				{
					for (int j = i; j < 2; ++j)
					{
						os << " " << information()(i, j);
					}
				}
				return os.good();
			}
	};
}

#endif
