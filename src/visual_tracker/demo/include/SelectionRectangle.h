/*
 * Copyright © 2011 Paul Nader 
 *
 * This file is part of QOpenTLD.
 *
 * This program is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 * 
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 * 
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef SELECTIONRECTANGLE_H_
#define SELECTIONRECTANGLE_H_

#include <QObject>
#include <QGraphicsRectItem>

class QGraphicsSceneMouseEvent;

class SelectionRectange : public QObject, public QGraphicsRectItem
{
	Q_OBJECT

public:
	SelectionRectange(QGraphicsItem *parent=0, QGraphicsScene* scene=0);
	~SelectionRectange() {};

protected:
	void mousePressEvent(QGraphicsSceneMouseEvent* event);
	void mouseReleaseEvent(QGraphicsSceneMouseEvent* event);
	void mouseDoubleClickEvent(QGraphicsSceneMouseEvent* event);
	void mouseMoveEvent(QGraphicsSceneMouseEvent* event);
	void hoverMoveEvent(QGraphicsSceneHoverEvent* event);

private:
	bool m_resizingX0;
	bool m_resizingX1;
	bool m_resizingY0;
	bool m_resizingY1;
	bool m_resizingC00;
	bool m_resizingC01;
	bool m_resizingC10;
	bool m_resizingC11;
	const unsigned m_edgeSensitivity;
};

#endif /* SELECTIONRECTANGLE_H_ */
