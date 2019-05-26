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
				_car2gps = Matrix<double, 4, 4>();

				_car2gps(0, 0) = 1;
				_car2gps(0, 1) = 0;
				_car2gps(0, 2) = 0;
				_car2gps(0, 3) = 0;

				_car2gps(1, 0) = 0;
				_car2gps(1, 1) = 1;
				_car2gps(1, 2) = 0;
				_car2gps(1, 3) = 0;

				_car2gps(2, 0) = 0;
				_car2gps(2, 1) = 0;
				_car2gps(2, 2) = 1;
				_car2gps(2, 3) = 0;

				_car2gps(3, 0) = 0;
				_car2gps(3, 1) = 0;
				_car2gps(3, 2) = 0;
				_car2gps(3, 3) = 1;
			}

			void set_car2gps(Matrix<double, 4, 4> car2gps)
			{
				_car2gps = Matrix<double, 4, 4>(car2gps);
			}

			void computeError()
			{
				const VertexSE2* v = dynamic_cast<const VertexSE2*>(_vertices[0]);
				Matrix<double, 4, 1> gps_in_car_coordinate_system_homogeneous;
				Matrix<double, 4, 1> gps_in_gps_coordinate_system_homogeneous;

				SE2 pose = v->estimate();
				SE2 gps_in_car_coordinate_system = pose.inverse() * _measurement;

				gps_in_car_coordinate_system_homogeneous[0] = gps_in_car_coordinate_system[0];
				gps_in_car_coordinate_system_homogeneous[1] = gps_in_car_coordinate_system[1];
				gps_in_car_coordinate_system_homogeneous[2] = 0;
				gps_in_car_coordinate_system_homogeneous[3] = 1;

				gps_in_gps_coordinate_system_homogeneous = _car2gps * gps_in_car_coordinate_system_homogeneous;

				// if the pose is correct, the gps measurement in the gps coordinate system will be zero.
				_error = g2o::Vector3D(
						gps_in_gps_coordinate_system_homogeneous(0, 0) / gps_in_gps_coordinate_system_homogeneous(3, 0),
						gps_in_gps_coordinate_system_homogeneous(1, 0) / gps_in_gps_coordinate_system_homogeneous(3, 0),
						0.0 // angle
					);

				//SE2 delta = _measurement.inverse() * (v->estimate());
				//_error = delta.toVector();
			}

			virtual bool read(std::istream& is)
			{
				int i, j;
				double data[3];
				is >> data[0] >> data[1] >> data[2];
				_measurement = SE2(data[0], data[1], data[2]);

				for (i = 0; i < 2; i++)
				{
					for (j = i; j < 2; j++)
					{
						is >> information()(i, j);

						if (i != j)
							information()(j, i) = information()(i, j);
					}
				}

				for (i = 0; i < 4; i++)
				{
					for (j = i; j < 4; j++)
					{
						is >> _car2gps(i, j);
					}
				}

				return true;
			}

			virtual bool write(std::ostream& os) const
			{
				int i, j;

				os << _measurement[0] << " " << _measurement[1] << " " << _measurement[2];

				for (i = 0; i < 2; ++i)
				{
					for (j = i; j < 2; ++j)
					{
						os << " " << information()(i, j);
					}
				}

				for (i = 0; i < 4; ++i)
				{
					for (j = i; j < 4; ++j)
					{
						os << " " << _car2gps(i, j);
					}
				}

				return os.good();
			}


			Matrix<double, 4, 4> _car2gps;
	};
}

#endif
