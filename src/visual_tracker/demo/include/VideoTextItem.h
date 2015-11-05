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

#ifndef VIDEOTEXTITEM_H_
#define VIDEOTEXTITEM_H_

#include <QGraphicsTextItem>
#include <QGraphicsLayoutItem>

class VideoTextItem : public QGraphicsTextItem, public QGraphicsLayoutItem
{
public:
	VideoTextItem(QGraphicsItem *parent=0, QGraphicsScene* scene=0);
	~VideoTextItem() {};

protected:
	QSizeF sizeHint(Qt::SizeHint which, const QSizeF& constraint = QSizeF()) const;
	void setGeometry(const QRectF &rect);
	void updateGeometry();
};

#endif /* VIDEOTEXTITEM_H_ */
