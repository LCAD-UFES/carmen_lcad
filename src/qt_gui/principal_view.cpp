/*
 * PrincipalView.cpp
 *
 *  Created on: 09/12/2011
 *      Author: rradaelli
 */

#include "principal_view.h"
#include <QGLWidget>
#include <stdio.h>

PrincipalView::PrincipalView() {
	setViewport(new QGLWidget());

}

PrincipalView::~PrincipalView() {
	// TODO Auto-generated destructor stub
}
