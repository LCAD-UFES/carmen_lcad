/*
 * particle_component.h
 *
 *  Created on: 06/10/2011
 *      Author: rradaelli
 */

#ifndef PARTICLE_COMPONENT_H_
#define PARTICLE_COMPONENT_H_

#include <carmen/carmen.h>
#include "map_component.h"

class Particle_Component : public Map_Component {
	Q_OBJECT
public:
	Particle_Component();
	virtual ~Particle_Component();
	void paint(QPainter *painter);

	public slots:
	void set_particle(carmen_localize_ackerman_particle_message particles);

	private:
	carmen_localize_ackerman_particle_message particles;
};


#endif /* PARTICLE_COMPONENT_H_ */
