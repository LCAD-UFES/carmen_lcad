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

#ifndef IMAGEWIDGET_H_
#define IMAGEWIDGET_H_

#include <QWidget>
#include <QImage>
#include <QLabel>
#include <opencv/cv.h>

class QPaintEvent;

class ImageWidget : public QWidget
{
	Q_OBJECT

public:
	ImageWidget(QWidget *parent = 0);
	~ImageWidget();
	void putImage(IplImage* iplImage);

private:
	QLabel *m_imagelabel;
	QImage m_image;
};

#endif /* IMAGEWIDGET_H_ */
