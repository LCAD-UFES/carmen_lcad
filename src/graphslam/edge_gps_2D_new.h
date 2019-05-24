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
//				static int count = 0;
//				static FILE *caco;
//				if (count == 0)
//					caco = fopen("caco.txt", "w");

				const VertexSE2 *v = dynamic_cast<const VertexSE2 *>(_vertices[0]);
				SE2 pose = v->estimate();

				tf::Transform world_to_car;
				tf::StampedTransform world_to_gps;
				world_to_car.setOrigin(tf::Vector3(pose[0], pose[1], 0.0));
				world_to_car.setRotation(tf::Quaternion(pose[2], 0.0, 0.0));
				_transformer->setTransform(tf::StampedTransform(world_to_car, tf::Time(0), "/world", "/car"));
				_transformer->lookupTransform("/world", "/gps", tf::Time(0), world_to_gps);

				_error = SE2(world_to_gps.getOrigin().x() - _measurement[0],
				             world_to_gps.getOrigin().y() - _measurement[1],
				             0.0).toVector();

//				if (count < 18839)
//					fprintf(caco, "%lf %lf %lf %lf %lf %lf\n",
//							pose[0], pose[1], world_to_gps.getOrigin().x(), world_to_gps.getOrigin().y(), _measurement[0], _measurement[1]);
//				else if (count == 18839)
//					fclose(caco);
//				count++;
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
