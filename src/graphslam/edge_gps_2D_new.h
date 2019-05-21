#ifndef _EDGE_GPS_2D_NEW_H_
#define _EDGE_GPS_2D_NEW_H_

#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/core/base_unary_edge.h>
#include <stdio.h>
#include <math.h>
#include <float.h>
#include <carmen/tf.h>


using namespace Eigen;

namespace g2o
{
	class EdgeGPSNew : public BaseUnaryEdge<3, SE2, VertexSE2>
	{
		public:
			// eigen operator
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

			EdgeGPSNew() : BaseUnaryEdge<3, SE2, VertexSE2>()
			{
				_transformer = NULL;
			}

			void set_car2gps(tf::Transformer *transformer)
			{
				_transformer = transformer;
			}

			void computeError()
			{
				const VertexSE2 *v = dynamic_cast<const VertexSE2 *>(_vertices[0]);
				SE2 pose = v->estimate();

				tf::Transform world_to_car;
				tf::StampedTransform world_to_gps;
				world_to_car.setOrigin(tf::Vector3(pose[0], pose[1], 0.0));
				world_to_car.setRotation(tf::Quaternion(pose[2], 0.0, 0.0));
				_transformer->setTransform(tf::StampedTransform(world_to_car, tf::Time(0), "/world", "/car"));
				_transformer->lookupTransform("/world", "/gps", tf::Time(0), world_to_gps);
//				printf("%d, yaw = %lf, world_to_gps: x: %lf, y: %lf, z: %lf, mx: %lf, my: %lf\n",
//						v->id(), carmen_radians_to_degrees(pose[2]),
//						world_to_gps.getOrigin().x(), world_to_gps.getOrigin().y(), world_to_gps.getOrigin().z(),
//						_measurement[0], _measurement[1]);

				_error = g2o::Vector3D(
						world_to_gps.getOrigin().x() - _measurement[0],
						world_to_gps.getOrigin().y() - _measurement[1],
						0.0
					);
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


			tf::Transformer *_transformer;
	};
}

#endif
