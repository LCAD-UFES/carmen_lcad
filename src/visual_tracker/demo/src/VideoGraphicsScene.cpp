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

#include "VideoGraphicsScene.h"
#include "SelectionRectangle.h"
#include <QGraphicsSceneMouseEvent>

VideoGraphicsScene::VideoGraphicsScene(QObject *parent)
: QGraphicsScene(parent)
, m_selecting(false)
{
	setBackgroundBrush(QBrush(Qt::black));
	m_selectionRectangle = new SelectionRectange(NULL, this);
	m_selectionRectangle->setPen(QPen(QColor(Qt::yellow)));
	m_selectionRectangle->setZValue(200);
	m_selectionRectangle->setFlag(QGraphicsItem::ItemIsMovable, true);
	m_selectionRectangle->setFlag(QGraphicsItem::ItemIsSelectable, true);
	m_selectionRectangle->hide();
}

void VideoGraphicsScene::mousePressEvent(QGraphicsSceneMouseEvent *e)
{
	if (e->button() == Qt::LeftButton)
	{
		if (!m_selecting)
		{
			status("");
			emit startSelection();
			m_selectionOrigin = e->scenePos();
			m_selectionRectangle->setRect(QRectF(m_selectionOrigin.x(), m_selectionOrigin.y(), 0, 0));
			emit selectionChanged(m_selectionRectangle->rect());
			m_selectionRectangle->show();
		}
		else status("");
	}

	if ((e->button() == Qt::RightButton))
	{
		m_selecting = false;
		m_selectionRectangle->hide();
		m_selectionOrigin.setX(0);
		m_selectionOrigin.setY(0);
		m_selectionFinal.setX(0);
		m_selectionFinal.setY(0);
		m_selectionRectangle->setRect(0,0,0,0);
		emit selectionChanged(m_selectionRectangle->rect());
		emit stopSelection();
	}

	QGraphicsScene::mousePressEvent(e);
}

void VideoGraphicsScene::mouseReleaseEvent(QGraphicsSceneMouseEvent *e)
{
	if (e->button() == Qt::LeftButton)
	{
		m_selectionRectangle->setSelected(false);
		if (!m_selecting && m_selectionRectangle->isVisible())
		{
			m_selecting = true;
			m_selectionFinal = e->scenePos();
			QRect selectionRect(m_selectionOrigin.x(), m_selectionOrigin.y(),
				m_selectionFinal.x()-m_selectionOrigin.x(), m_selectionFinal.y()-m_selectionOrigin.y());
			m_selectionRectangle->setRect(m_selectionRectangle->mapRectToScene(selectionRect.normalized()));
			emit selectionChanged(m_selectionRectangle->rect());
			m_selectionRectangle->show();
			status("Double click in selection to initiate detector or resize with left mouse");
		}
	}
	
	QGraphicsScene::mouseReleaseEvent(e);
}

void VideoGraphicsScene::mouseMoveEvent(QGraphicsSceneMouseEvent* e)
{
	QGraphicsScene::mouseMoveEvent(e);

	if (e->buttons() == Qt::LeftButton)
	{
		if (!m_selecting)
		{
			if (m_selectionOrigin.toPoint() == QPoint(-1, -1))
				m_selectionOrigin = e->scenePos();
			m_selectionFinal = e->scenePos();
			QRect selectionRect(m_selectionOrigin.x(), m_selectionOrigin.y(),
				m_selectionFinal.x()-m_selectionOrigin.x(), m_selectionFinal.y()-m_selectionOrigin.y());
			m_selectionRectangle->setRect(m_selectionRectangle->mapRectToScene(selectionRect.normalized()));
			m_selectionRectangle->show();
			//return;
		}
		emit selectionChanged(m_selectionRectangle->mapToScene(m_selectionRectangle->rect()).boundingRect());
	}
}

void VideoGraphicsScene::mouseDoubleClickEvent(QGraphicsSceneMouseEvent* e)
{
	QGraphicsItem* item = itemAt(e->scenePos());
	if (item == m_selectionRectangle)
	{
		status("");
		m_selectionRectangle->setSelected(false);
		m_selectionRectangle->hide();
		m_selectionRectangle->unsetCursor();
		QRectF rect = m_selectionRectangle->rect();
		QPointF p = m_selectionRectangle->scenePos();
		rect.translate(p.x(), p.y());

		emit stopSelection();
		emit regionSelected();
		m_selectionOrigin.setX(-1);
		m_selectionOrigin.setY(-1);
		m_selectionFinal.setX(0);
		m_selectionFinal.setY(0);
		m_selectionRectangle->setPos(0,0);
		emit selectionChanged(m_selectionRectangle->rect());
		m_selecting = false;
	}
}

void VideoGraphicsScene::setSelection(QRectF& rect)
{
	m_selectionRectangle->setRect(rect);
	//m_selectionRectangle->setPos(rect.topLeft());
	emit selectionChanged(m_selectionRectangle->rect());
	m_selectionRectangle->update();
	if (rect.size() != QSize(0,0))
		m_selecting = true;
}

QRectF VideoGraphicsScene::getSelection(void) const
{
	return m_selectionRectangle->rect();
}

void VideoGraphicsScene::showSelection(bool show) const
{
	if (show)
	{
	if (m_selectionRectangle->rect().size() != QSize(0,0))
		m_selectionRectangle->show();
	}
	else m_selectionRectangle->hide();
}

bool VideoGraphicsScene::selectionVisible(void) const
{
	return m_selectionRectangle->isVisible();
}