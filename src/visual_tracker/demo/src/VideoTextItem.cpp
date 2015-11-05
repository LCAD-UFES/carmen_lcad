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

#include "VideoTextItem.h"

VideoTextItem::VideoTextItem(QGraphicsItem *parent, QGraphicsScene* scene)
: QGraphicsTextItem(parent, scene)
{
}

QSizeF VideoTextItem::sizeHint(Qt::SizeHint, const QSizeF&) const
{
	QSizeF size;
	size = QGraphicsTextItem::boundingRect().size();
	return size;
}

void VideoTextItem::setGeometry(const QRectF &rect)
{
	qreal x = rect.x();
	qreal y = rect.y();
	QGraphicsTextItem::setPos(x,y);
	VideoTextItem::updateGeometry();
}

void VideoTextItem::updateGeometry()
{
	QGraphicsLayoutItem::updateGeometry();
}