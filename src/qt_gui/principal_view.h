/*
 * PrincipalView.h
 *
 *  Created on: 09/12/2011
 *      Author: rradaelli
 */

#ifndef PRINCIPALVIEW_H_
#define PRINCIPALVIEW_H_

#include <qgraphicsview.h>

class PrincipalView: public QGraphicsView {
	Q_OBJECT
public:
	PrincipalView();
	virtual ~PrincipalView();

public slots:
void zoomIn() { scale(1.2, 1.2); }
void zoomOut() { scale(1 / 1.2, 1 / 1.2); }
};

#endif /* PRINCIPALVIEW_H_ */
