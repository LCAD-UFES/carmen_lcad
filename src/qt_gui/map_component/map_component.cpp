/*
 * Map_Component.cpp
 *
 *  Created on: 05/10/2011
 *      Author: rradaelli
 */

#include "map_component.h"
#include "../carmen/carmen_state.h"

Map_Component::Map_Component() {
	setAcceptedMouseButtons(0);
}

void Map_Component::set_visible(bool b) {
	setVisible(b);
}

double Map_Component::get_heigth() {
	return Carmen_State::get_instance()->map_config->y_size;
}

double Map_Component::get_resolution() {
	return Carmen_State::get_instance()->map_config->resolution;
}

void Map_Component::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
	if(option && widget) {

	}
	paint(painter);
}

QRectF Map_Component::boundingRect() const {
	return brect;
}

Map_Component::~Map_Component() {
}
