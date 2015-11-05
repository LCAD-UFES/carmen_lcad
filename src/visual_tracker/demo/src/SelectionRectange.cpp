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

#include "SelectionRectangle.h"
#include <QBrush>
#include <QCursor>
#include <QGraphicsSceneMouseEvent>

SelectionRectange::SelectionRectange(QGraphicsItem *parent, QGraphicsScene* scene)
: QGraphicsRectItem(parent, scene)
, m_resizingX0(false)
, m_resizingX1(false)
, m_resizingY0(false)
, m_resizingY1(false)
, m_resizingC00(false)
, m_resizingC01(false)
, m_resizingC10(false)
, m_resizingC11(false)
, m_edgeSensitivity(5)
{
	setAcceptHoverEvents(true);
	QBrush brush;
	brush.setColor(Qt::transparent);
	setBrush(brush);
}

void SelectionRectange::hoverMoveEvent(QGraphicsSceneHoverEvent* e)
{
	QRectF rect = mapRectToScene(this->rect());
	QPointF pos = e->scenePos();
	if (((pos.x() < (rect.x() + m_edgeSensitivity)) && (pos.y() < (rect.y() + m_edgeSensitivity))) ||
		((pos.x() > (rect.x() + rect.width() - m_edgeSensitivity)) && (pos.y() > (rect.y() + rect.height() - m_edgeSensitivity))))
		setCursor(Qt::SizeFDiagCursor);
	else if (((pos.x() > (rect.x() + rect.width() - m_edgeSensitivity)) && (pos.y() < (rect.y() + m_edgeSensitivity))) ||
		((pos.x() < (rect.x() + m_edgeSensitivity)) && (pos.y() > (rect.y() + rect.height() - m_edgeSensitivity))))
		setCursor(Qt::SizeBDiagCursor);
	else if ((pos.x() < (rect.x() + m_edgeSensitivity)) || (pos.x() > (rect.x() + rect.width() - m_edgeSensitivity)))
		setCursor(Qt::SizeHorCursor);
	else if ((pos.y() < (rect.y() + m_edgeSensitivity)) || (pos.y() > (rect.y() + rect.height() - m_edgeSensitivity)))
		setCursor(Qt::SizeVerCursor);
	else
		unsetCursor();
}

void SelectionRectange::mouseMoveEvent(QGraphicsSceneMouseEvent* e)
{
	QRectF rect = mapRectToScene(this->rect());
	QPointF pos = e->scenePos();

	if (m_resizingX0 || m_resizingY0 || m_resizingX1 || m_resizingY1 || m_resizingC00 || m_resizingC01 || m_resizingC10 || m_resizingC11)
	{
		if (m_resizingX0 && (pos.x() < (rect.x() + rect.width())))
		{
			QRect r(pos.x(), rect.y(), abs(rect.width() - pos.x() + rect.x()), rect.height());
			setRect(mapFromScene(r).boundingRect());
		}
		else if (m_resizingX1 && (pos.x() > rect.x()))
		{
			QRect r(rect.x(), rect.y(), abs(pos.x() - rect.x()), rect.height());
			setRect(mapFromScene(r).boundingRect());
		}
		else if (m_resizingY0 && (pos.y() < (rect.y() + rect.height())))
		{
			QRect r(rect.x(), pos.y(), rect.width(), abs(rect.height() - pos.y() + rect.y()));
			setRect(mapFromScene(r).boundingRect());
		}
		else if (m_resizingY1 && (pos.y() > rect.y()))
		{
			QRect r(rect.x(), rect.y(), rect.width(), abs(pos.y() - rect.y()));
			setRect(mapFromScene(r).boundingRect());
		}
		else if (m_resizingC00 && ((rect.width() - pos.x() + rect.x()) > 0) && ((rect.height() - pos.y() + rect.y()) > 0))
		{
			QRect r(pos.x(), pos.y(), rect.width() - pos.x() + rect.x(), rect.height() - pos.y() + rect.y());
			setRect(mapFromScene(r).boundingRect());
		}
		else if (m_resizingC11 && ((pos.x() - rect.x()) > 0) && ((pos.y() - rect.y()) > 0))
		{
			QRect r(rect.x(), rect.y(), pos.x() - rect.x(), pos.y() - rect.y());
			setRect(mapFromScene(r).boundingRect());
		}
		else if (m_resizingC01 && ((pos.x() - rect.x()) > 0) && ((rect.y() + rect.height() - pos.y()) > 0))
		{
			QRect r(rect.x(), pos.y(), pos.x() - rect.x(), rect.y() + rect.height() - pos.y());
			setRect(mapFromScene(r).boundingRect());
		}
		else if (m_resizingC10 && ((rect.x() + rect.width() - pos.x()) > 0) && ((pos.y() - rect.y()) > 0))
		{
			QRect r(pos.x(), rect.y(), rect.x() + rect.width() - pos.x(), pos.y() - rect.y());
			setRect(mapFromScene(r).boundingRect());
		}
	}
	else
	{
		QGraphicsRectItem::mouseMoveEvent(e);		
	}
	e->accept();
}

void SelectionRectange::mousePressEvent(QGraphicsSceneMouseEvent* e)
{
	if (e->button() == Qt::LeftButton)
	{
		QRectF rect = mapRectToScene(this->rect());
		QPointF pos = e->scenePos();
		if ((pos.x() < (rect.x() + m_edgeSensitivity)) && (pos.y() < (rect.y() + m_edgeSensitivity)))
			m_resizingC00 = true;
		else if ((pos.x() > (rect.x() + rect.width() - m_edgeSensitivity)) && (pos.y() > (rect.y() + rect.height() - m_edgeSensitivity)))
			m_resizingC11 = true;
		else if ((pos.x() > (rect.x() + rect.width() - m_edgeSensitivity)) && (pos.y() < (rect.y() + m_edgeSensitivity)))
			m_resizingC01 = true;
		else if ((pos.x() < (rect.x() + m_edgeSensitivity)) && (pos.y() > (rect.y() + rect.height() - m_edgeSensitivity)))
			m_resizingC10 = true;
		else if (pos.x() < (rect.x() + m_edgeSensitivity))
			m_resizingX0 = true;
		else if (pos.x() > (rect.x() + rect.width() - m_edgeSensitivity))
			m_resizingX1 = true;
		else if (pos.y() < (rect.y() + m_edgeSensitivity))
			m_resizingY0 = true;
		else if (pos.y() > (rect.y() + rect.height() - m_edgeSensitivity))
			m_resizingY1 = true;
		else setCursor(Qt::ClosedHandCursor);
	}
	QGraphicsRectItem::mousePressEvent(e);
}

void SelectionRectange::mouseReleaseEvent(QGraphicsSceneMouseEvent* e)
{
	QGraphicsRectItem::mouseReleaseEvent(e);
	if (e->button() == Qt::LeftButton)
		unsetCursor();
	setSelected(false);
	m_resizingX0 = m_resizingY0 = m_resizingX1 = m_resizingY1 = m_resizingC00 = m_resizingC01 = m_resizingC10 = m_resizingC11 = false;
}

void SelectionRectange::mouseDoubleClickEvent(QGraphicsSceneMouseEvent*)
{
	unsetCursor();
	hide();
}
