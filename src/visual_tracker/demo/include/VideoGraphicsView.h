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

#ifndef VIDEOGRAPHICSVIEW_H_
#define VIDEOGRAPHICSVIEW_H_

#include "VideoGraphicsScene.h"
#include <QMouseEvent>
#include <QGraphicsView>

class SelectionRectange;

class VideoGraphicsView : public QGraphicsView
{
	Q_OBJECT

public:
	VideoGraphicsView(VideoGraphicsScene& scene, QWidget *parent=0);
	~VideoGraphicsView() {}

signals:
	void status(const QString message);

private:
	VideoGraphicsScene& m_videoGraphicsScene;
};



#endif /* VIDEOGRAPHICSVIEW_H_ */
