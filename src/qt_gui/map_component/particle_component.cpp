/*
 * particle_component.cpp
 *
 *  Created on: 06/10/2011
 *      Author: rradaelli
 */

#include "particle_component.h"
#include <QPainter>

Particle_Component::Particle_Component()
: Map_Component() {
	set_visible(false);
	particles.num_particles = 0;
}

void Particle_Component::paint(QPainter *painter) {
	painter->setPen(Qt::red);

	for(int i=0; i<particles.num_particles; i++) {
		double x = particles.particles[i].x;
		double y = particles.particles[i].y;

		painter->drawPoint(QPointF(x/get_resolution(), get_heigth()-(y/get_resolution())));
	}
}

void Particle_Component::set_particle(carmen_localize_ackerman_particle_message particles) {
	this->particles = particles;

	double min_x, max_x, min_y, max_y;
	min_x = min_y = DBL_MAX;
	max_x = max_y = 0;

	for(int i=0; i<particles.num_particles; i++) {
		if(particles.particles[i].x/get_resolution() > max_x) {
			max_x = particles.particles[i].x/get_resolution();
		}

		if((get_heigth()-(particles.particles[i].y/get_resolution())) > max_y) {
			max_y = get_heigth()-(particles.particles[i].y/get_resolution());
		}

		if(particles.particles[i].x/get_resolution() < min_x) {
			min_x = particles.particles[i].x/get_resolution();
		}

		if((get_heigth()-(particles.particles[i].y/get_resolution())) < min_y) {
			min_y = get_heigth()-(particles.particles[i].y/get_resolution());
		}
	}

	prepareGeometryChange();

	brect.setX(min_x);
	brect.setY(min_y);
	brect.setWidth(max_x - min_x);
	brect.setHeight(max_y - min_y);

	update();


}

Particle_Component::~Particle_Component() {
	// TODO Auto-generated destructor stub
}
