#ifndef _EDGE_GPS_2D_H_
#define _EDGE_GPS_2D_H_

#include <Eigen/Core>
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/core/base_unary_edge.h"
#include <stdio.h>
#include <math.h>
#include <float.h>
#include <memory>
#include <string>

using namespace Eigen;

namespace g2o
{
	class EdgeGPS : public BaseUnaryEdge<3, SE2, VertexSE2>
	{
		public:
			EdgeGPS()
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

	/* Factory::registerType() recebe std::shared_ptr no g2o 2023 (apt do Ubuntu 26.04) e
	   ponteiro cru no fork antigo do LCAD (Ubuntu 20.04, /usr/local). Escolhe em tempo de
	   compilacao a forma que a versao instalada aceita. Uso:
	   g2o_register_type<HyperGraphElementCreator<EdgeGPS> >(factory, "EDGE_GPS"); */
	template <class Creator, class F>
	auto g2o_register_type_impl(F *factory, const std::string &tag, int)
		-> decltype(factory->registerType(tag, std::make_shared<Creator>()), void())
	{
		factory->registerType(tag, std::make_shared<Creator>());
	}

	template <class Creator, class F>
	void g2o_register_type_impl(F *factory, const std::string &tag, long)
	{
		factory->registerType(tag, new Creator);
	}

	template <class Creator, class F>
	void g2o_register_type(F *factory, const std::string &tag)
	{
		g2o_register_type_impl<Creator>(factory, tag, 0);
	}
}

#endif
