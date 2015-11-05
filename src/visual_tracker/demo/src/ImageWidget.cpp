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

#include "ImageWidget.h"

#include <QLabel>
#include <QPixmap>
#include <QVBoxLayout>
#include <QPaintEvent>

using namespace cv;

ImageWidget::ImageWidget(QWidget *parent)
: QWidget(parent)
{
	QVBoxLayout* layout = new QVBoxLayout();
	setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	//setFixedSize(320, 240);
	layout->setContentsMargins(0,0,0,0);
	m_imagelabel = new QLabel(this);
	layout->addWidget(m_imagelabel);
	setLayout(layout);
	QPalette pal = palette();
	pal.setColor(backgroundRole(), Qt::black);
	setPalette(pal);
}

ImageWidget::~ImageWidget(void)
{  
}

void ImageWidget::putImage(IplImage* iplImage)
{
	if (iplImage->depth == IPL_DEPTH_8U)
	{
		if (iplImage->nChannels == 3)
		{
			setFixedSize(iplImage->width, iplImage->height);
			m_image = QImage((const uchar *) iplImage->imageData, iplImage->width, iplImage->height, QImage::Format_RGB888).rgbSwapped();
			m_imagelabel->setPixmap(QPixmap::fromImage(m_image.scaled(width(), height()), Qt::AutoColor));
		}
		else if (iplImage->nChannels == 1)
		{
			setFixedSize(iplImage->width, iplImage->height);
			m_image = QImage((const uchar *) iplImage->imageData, iplImage->width, iplImage->height, QImage::Format_Indexed8);
			m_imagelabel->setPixmap(QPixmap::fromImage(m_image.scaled(width(), height()), Qt::MonoOnly));
		}
	}
}