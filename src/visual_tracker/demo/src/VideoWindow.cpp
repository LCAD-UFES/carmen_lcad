/*
 * Copyright � 2011 Paul Nader 
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
#include "VideoWindow.h"
#include "VideoTextItem.h"
#include "VideoGraphicsView.h"
#include "VideoGraphicsScene.h"

#include <QDir>
#include <QTimer>
#include <QWidget>
#include <QtOpenGL/QGLWidget>
#include <QVBoxLayout>
#include <QGraphicsView>
#include <QGraphicsWidget>
#include <QGraphicsGridLayout>
#include <QGraphicsEllipseItem>

#include <carmen/carmen.h>
#include <carmen/global_graphics_qt.h>

using namespace cv;

#define LABEL_FN tr("FN: ")
#define LABEL_FPS tr("FPS: ")
#define LABEL_PEX tr("PEX: ")
#define LABEL_NEX tr("NEX: ")
#define LABEL_BBX tr("X: ")
#define LABEL_BBY tr("Y: ")
#define LABEL_BBW tr("W: ")
#define LABEL_BBH tr("H: ")

int errCallBack(int status, const char* func_name,
		const char* err_msg, const char* file_name,
		int line, void* userdata)
{
	Q_UNUSED(status);
	Q_UNUSED(func_name);
	Q_UNUSED(err_msg);
	Q_UNUSED(file_name);
	Q_UNUSED(line);
	Q_UNUSED(userdata);
#ifdef DEBUG
#ifdef WIN32
	// MSVC breakpoint to catch OpenCV errors
	__asm __emit 0xF1
#endif
#endif
	return 0;
}

VideoWindow::VideoWindow(QStatusBar& statusbar, QWidget *parent)
: QFrame(parent)
, m_frameNumber(0)
, m_frameCount(0)
, m_pexRow(1)
, m_pexCol(1)
, m_pexPosition(0)
, m_autoStart(false)
, m_fpsLimit(0)
, m_captureDevice(NULL)
, m_statusbar(statusbar)
, m_brightness(0)
, m_contrast(0)
{
	setWindowTitle("QOpenCV");
	setContentsMargins(0,0,0,0);
	setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);

	m_layout = new QHBoxLayout();
	m_layout->setSpacing(0);
	m_layout->setContentsMargins(0, 0, 0, 0);
	m_layout->setSizeConstraint(QLayout::SetMinimumSize);

	QFrame* nexFrame = new QFrame();
	QVBoxLayout* a = new QVBoxLayout();
	nexFrame->setLayout(a);
	nexFrame->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);
	a->setSpacing(0);
	a->setContentsMargins(0, 0, 0, 0);
	a->setDirection(QBoxLayout::Down);
	m_nexScene = new QGraphicsScene();
	m_nexView = new QGraphicsView(m_nexScene);
	m_nexView->setContentsMargins(0, 0, 0, 0);
	m_nexView->setAlignment(Qt::AlignTop);
	m_nexView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	a->addWidget(m_nexView);
	m_layout->addWidget(nexFrame);

	m_videoScene = new VideoGraphicsScene();
	m_cameraWidget = new ImageWidget();
	m_videoScene->addWidget(m_cameraWidget);

	m_textLayout = new QGraphicsGridLayout();
	m_textLayout->setSpacing(0);
	m_textLayout->setColumnAlignment(0, Qt::AlignBottom);
	m_textForm = new QGraphicsWidget;
	m_textForm->setLayout(m_textLayout);
	m_textForm->setZValue(100);
	m_videoScene->addItem(m_textForm);

	m_fn = new VideoTextItem();
	m_fn->setDefaultTextColor(QColor(Qt::yellow));
	m_fn->setPlainText(QString(LABEL_FN));
	m_videoScene->addItem(m_fn);
	m_textLayout->addItem(m_fn, 0, 0);
	m_fn->hide();

	m_fps = new VideoTextItem();
	m_fps->setDefaultTextColor(QColor(Qt::yellow));
	m_fps->setPlainText(QString(LABEL_FPS));
	m_videoScene->addItem(m_fps);
	m_textLayout->addItem(m_fps, 0, 1);
	m_fps->hide();

	m_pexCount = new VideoTextItem();
	m_pexCount->setDefaultTextColor(QColor(Qt::yellow));
	m_pexCount->setPlainText(QString(LABEL_PEX));
	m_videoScene->addItem(m_pexCount);
	m_textLayout->addItem(m_pexCount, 0, 2);
	m_pexCount->hide();

	m_nexCount = new VideoTextItem();
	m_nexCount->setDefaultTextColor(QColor(Qt::yellow));
	m_nexCount->setPlainText(QString(LABEL_NEX));
	m_videoScene->addItem(m_nexCount);
	m_textLayout->addItem(m_nexCount, 0, 3);
	m_nexCount->hide();

	m_info = new VideoTextItem();
	m_info->setDefaultTextColor(QColor(Qt::yellow));
	m_videoScene->addItem(m_info);
	m_textLayout->addItem(m_info, 0, 4);
	m_info->hide();

	m_bbx = new VideoTextItem();
	m_bbx->setDefaultTextColor(QColor(Qt::yellow));
	m_bbx->setPlainText(QString(LABEL_BBX));
	m_videoScene->addItem(m_bbx);
	m_textLayout->addItem(m_bbx, 1, 0);
	m_bbx->hide();

	m_bby = new VideoTextItem();
	m_bby->setDefaultTextColor(QColor(Qt::yellow));
	m_bby->setPlainText(QString(LABEL_BBY));
	m_videoScene->addItem(m_bby);
	m_textLayout->addItem(m_bby, 1, 1);
	m_bby->hide();

	m_bbw = new VideoTextItem();
	m_bbw->setDefaultTextColor(QColor(Qt::yellow));
	m_bbw->setPlainText(QString(LABEL_BBW));
	m_videoScene->addItem(m_bbw);
	m_textLayout->addItem(m_bbw, 1, 2);
	m_bbw->hide();

	m_bbh = new VideoTextItem();
	m_bbh->setDefaultTextColor(QColor(Qt::yellow));
	m_bbh->setPlainText(QString(LABEL_BBH));
	m_videoScene->addItem(m_bbh);
	m_textLayout->addItem(m_bbh, 1, 3);
	m_bbh->hide();

	m_videoFrame = new QFrame();
	m_videoFrame->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
	QVBoxLayout* b = new QVBoxLayout();
	b->setContentsMargins(0, 0, 0, 0);
	m_videoView = new VideoGraphicsView(*m_videoScene);
	m_videoView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	m_videoView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	m_videoView->setViewport(new QGLWidget);
	b->addWidget(m_videoView);

	m_confidence = new VideoTextItem();
	m_confidence->setDefaultTextColor(QColor(Qt::white));
	m_videoScene->addItem(m_confidence);
	m_confidence->setZValue(100);
	m_confidence->hide();
	m_videoFrame->setLayout(b);
	m_layout->addWidget(m_videoFrame);

	QFrame* pexFrame = new QFrame();
	QVBoxLayout* c = new QVBoxLayout();
	pexFrame->setLayout(c);
	pexFrame->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);
	c->setSpacing(0);
	c->setContentsMargins(0, 0, 0, 0);
	c->setDirection(QBoxLayout::Down);
	m_pexScene = new QGraphicsScene();
	m_pexView = new QGraphicsView(m_pexScene);
	m_pexView->setContentsMargins(0, 0, 0, 0);
	m_pexView->setAlignment(Qt::AlignTop);
	m_pexView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	c->addWidget(m_pexView);
	m_layout->addWidget(pexFrame);

	setLayout(m_layout);

	m_frame = NULL;

	m_updateTimer = new QTimer(this);
	connect(m_updateTimer, SIGNAL(timeout()), this, SLOT(updateCounters()));
	m_updateTimer->setInterval(1000);

	m_frameTimer = new QTimer(this);
	connect(m_frameTimer, SIGNAL(timeout()), this, SLOT(processFrame()));
	connect(this, SIGNAL(status(const QString)), &statusbar, SLOT(showMessage(const QString)));
	connect(&m_tld, SIGNAL(status(const QString)), &statusbar, SLOT(showMessage(const QString)));
	connect(&m_tld, SIGNAL(newPexPixmap(QPixmap)), this, SLOT(displayPexImage(QPixmap)));
	connect(&m_tld, SIGNAL(pexCount(int)), this, SLOT(pexCount(int)));
	connect(&m_tld, SIGNAL(newNexPixmap(QPixmap)), this, SLOT(displayNexImage(QPixmap)));
	connect(&m_tld, SIGNAL(nexCount(int)), this, SLOT(nexCount(int)));
	connect(&m_tld, SIGNAL(boundingRect(QRectF)), this, SLOT(boundingRect(QRectF)));
	connect(&m_tld, SIGNAL(info(QString)), this, SLOT(info(QString)));
	connect(&m_tld, SIGNAL(plot(const QVariant, QColor)), this, SLOT(plot(const QVariant, QColor)));
	connect(&m_tld, SIGNAL(clear()), this, SLOT(clearDisplay()));
	connect(&m_tld, SIGNAL(confidence(double)), this, SLOT(confidence(double)));
	connect(&m_tld, SIGNAL(newFrame(IplImage*)), this, SLOT(newFrame(IplImage*)));
	connect(m_videoView, SIGNAL(status(const QString)), &statusbar, SLOT(showMessage(const QString)));
	connect(m_videoScene, SIGNAL(status(const QString)), &statusbar, SLOT(showMessage(const QString)));
	connect(m_videoScene, SIGNAL(startSelection()), this, SLOT(stopTimers()));
	connect(m_videoScene, SIGNAL(stopSelection()), this, SLOT(startTimers()));
	connect(m_videoScene, SIGNAL(regionSelected()), this, SLOT(startDetection()));
	connect(m_videoScene, SIGNAL(selectionChanged(const QRectF)), this, SLOT(selectionChanged(const QRectF)));

	cv::redirectError(errCallBack);
	status("Select video source");
}

VideoWindow::~VideoWindow()
{
	close();
}

void VideoWindow::close()
{
	stopTimers();
	delete m_captureDevice;
}

void VideoWindow::init()
{
	QRectF selection = m_videoScene->getSelection();
	if (selection.size() == QSize(0,0))
	{
		status(tr("Press left mouse button and drag to draw selection area"));
	}
	else
	{
		m_videoScene->showSelection(true);

		if (m_autoStart)
		{
			startTimers();
			startDetection();
		}
		else status(tr("Double click on selection area to start"));
	}
}

bool VideoWindow::openCamera()
{
	close();
	clear();
	if (m_captureDevice)
	{
		free(m_captureDevice);
		m_captureDevice = NULL;
		m_frameNumber = 0;
		m_frameCount = 0;
	}
	status("Opening video camera source...");
	m_captureDevice = new CameraCaptureDevice(0);
	m_captureDevice->open();
	m_captureDevice->start();
	connect(m_captureDevice, SIGNAL(status(const QString)), &m_statusbar, SLOT(showMessage(const QString)));
	m_frame = m_captureDevice->read();
	m_cameraWidget->putImage(m_frame);
	m_videoFrame->setFixedSize(m_cameraWidget->width(), m_cameraWidget->height());
	setCountersVisible();
	m_videoScene->showSelection();
	init();
	return true;
}

bool VideoWindow::openCARMEN(IplImage *frame)
{
	close();
	clear();
	status("Opening CARMEN camera source...");
//	connect(m_captureDevice, SIGNAL(status(const QString)), &m_statusbar, SLOT(showMessage(const QString)));
	m_frame = frame;
	m_cameraWidget->putImage(m_frame);
	m_videoFrame->setFixedSize(m_cameraWidget->width(), m_cameraWidget->height());
	setCountersVisible();
	m_videoScene->showSelection();
	init();
	return true;
}

bool VideoWindow::openDirectory(QString& dirname)
{
	close();
	clear();
	if (m_captureDevice)
	{
		free(m_captureDevice);
		m_captureDevice = NULL;
	}
	status("Opening directory video source...");
	m_captureDevice = new DirectoryCaptureDevice(QDir(dirname));
	m_captureDevice->open();
	m_captureDevice->start();
	connect(m_captureDevice, SIGNAL(status(const QString)), &m_statusbar, SLOT(showMessage(const QString)));
	m_frame = m_captureDevice->read();
	m_cameraWidget->putImage(m_frame);
	m_videoFrame->setFixedSize(m_cameraWidget->width(), m_cameraWidget->height());
	setCountersVisible();
	init();
	return true;
}

bool VideoWindow::openFile(QString& filename)
{
	close();
	clear();
	if (m_captureDevice)
	{
		free(m_captureDevice);
		m_captureDevice = NULL;
	}
	status("Opening file video source...");
	m_videoSource = V_FILE;
	m_captureDevice = new FileCaptureDevice(filename);
	m_captureDevice->open();
	m_captureDevice->start();
	connect(m_captureDevice, SIGNAL(status(const QString)), &m_statusbar, SLOT(showMessage(const QString)));
	m_frame = m_captureDevice->read();
	m_cameraWidget->putImage(m_frame);
	m_videoFrame->setFixedSize(m_cameraWidget->width(), m_cameraWidget->height());
	setCountersVisible();
	init();
	return true;
}

void VideoWindow::processFrame()
{
	if (!m_captureDevice)
	{
		status("No capture device");
		return;
	}

	m_frame = m_captureDevice->read();

	if (m_frame)
	{
		// Process brightness/contrast
		adjust(m_frame);

		// Process detection
		m_tld.test(m_frame);
	}
	else stopTimers();
}

void VideoWindow::newFrame(IplImage* frame)
{
	m_cameraWidget->putImage(frame);
	if (m_videoSource == V_DIR)
		cvReleaseImage(&m_frame);
	//m_videoFrame->setFixedSize(m_cameraWidget->width(), m_cameraWidget->height());

	m_frameCount++;
	m_frameNumber++;
	m_fn->setPlainText(QString("FN: %1").arg(m_frameNumber));
	m_textLayout->invalidate();
}

void VideoWindow::setCountersVisible(bool visible)
{
	m_fn->setVisible(visible);
	m_fps->setVisible(visible);
	m_info->setVisible(visible);
	m_pexCount->setVisible(visible);
	m_nexCount->setVisible(visible);
	m_bbx->setVisible(visible);
	m_bby->setVisible(visible);
	m_bbw->setVisible(visible);
	m_bbh->setVisible(visible);
}

void VideoWindow::updateCounters()
{
	m_fps->setPlainText(QString(LABEL_FPS)+QString("%1").arg(m_frameCount));
	m_textLayout->invalidate();
	m_frameCount = 0;
}

void VideoWindow::startTimers()
{
	m_updateTimer->start();
	m_frameTimer->start();
}

void VideoWindow::stopTimers()
{
	m_updateTimer->stop();
	m_frameTimer->stop();
}

void VideoWindow::clear()
{
	m_videoScene->showSelection(false);

	m_pexScene->clear();
	m_pexView->layout()->update();
	m_pexView->update();
	m_pexPosition = 0;

	m_nexScene->clear();
	m_nexView->layout()->update();
	m_nexView->update();
}

void VideoWindow::selectionChanged(const QRectF rect)
{
	m_selection = rect;
	m_bbx->setPlainText(QString(LABEL_BBX)+QString("%1").arg(rect.x()));
	m_bby->setPlainText(QString(LABEL_BBY)+QString("%1").arg(rect.y()));
	m_bbw->setPlainText(QString(LABEL_BBW)+QString("%1").arg(rect.width()));
	m_bbh->setPlainText(QString(LABEL_BBH)+QString("%1").arg(rect.height()));
}

void VideoWindow::autoStart(bool val)
{
	m_autoStart = val;
}

void VideoWindow::fpsLimit(int val)
{
	if (val != 0) m_fpsLimit = 1000/val;
	else m_fpsLimit = 0;
	m_frameTimer->setInterval(m_fpsLimit);

}

void VideoWindow::brightness(int val)
{
	m_brightness = val;

	if (!m_frameTimer->isActive() && m_captureDevice && m_frame)
	{
		IplImage *frame;
		frame = cvCreateImage(cvSize(m_frame->width, m_frame->height), m_frame->depth, m_frame->nChannels);
		cvCopy(m_frame, frame, NULL);
		adjust(frame);
		m_cameraWidget->putImage(frame);
		cvReleaseImage(&frame);
	}
}

void VideoWindow::contrast(int val)
{
	m_contrast = val;

	if (!m_frameTimer->isActive() && m_captureDevice && m_frame)
	{
		IplImage *frame;
		frame = cvCreateImage(cvSize(m_frame->width, m_frame->height), m_frame->depth, m_frame->nChannels);
		cvCopy(m_frame, frame, NULL);
		adjust(frame);
		m_cameraWidget->putImage(frame);
		cvReleaseImage(&frame);
	}
}

void VideoWindow::adjust(IplImage* inFrame, IplImage* outFrame)
{
	// The algorithm is by Werner D. Streidt
	// (http://visca.com/ffactory/archives/5-99/msg00021.html)
	//
	double alpha, beta;
	if (m_contrast > 0)
	{
		double delta = 127.*m_contrast/100;
		alpha = 255./(255. - delta*2);
		beta = alpha*(m_brightness - delta);
	}
	else
	{
		double delta = -128.*m_contrast/100;
		alpha = (256.-delta*2)/255.;
		beta = alpha*m_brightness + delta;
	}

	Mat imgMat = inFrame;
	if (!outFrame) outFrame = inFrame;
	//Avelino - begin
	Mat outMat = (cv::Mat) outFrame;
	imgMat.convertTo(outMat, CV_8UC3, alpha, beta);
	*outFrame = (IplImage) outMat;
	//Avelino - end
}


void VideoWindow::startDetection()
{
	clear();

	if (!m_frame)
		return;

	status("Training detector...");
	m_tld.train(m_frame, m_selection);

	status("");

	//status("Initializing display...");
	//m_tld.display(0);

}

void VideoWindow::displayPexImage(QPixmap pixmap)
{
	QGraphicsPixmapItem *item = new QGraphicsPixmapItem(pixmap);
	item->setPos(0, m_pexScene->itemsBoundingRect().height());
	m_pexScene->addItem(item);
	m_pexView->ensureVisible(1, m_pexScene->itemsBoundingRect().height(), 1, 1, 0, 0);
	m_pexView->update();
}

void VideoWindow::pexCount(int count)
{
	m_pexCount->setPlainText(QString("PEX: %1").arg(count));
}

void VideoWindow::displayNexImage(QPixmap pixmap)
{
	QGraphicsPixmapItem *item = new QGraphicsPixmapItem(pixmap);
	item->setPos(0, m_nexScene->itemsBoundingRect().height());
	m_nexScene->addItem(item);
	m_nexView->ensureVisible(1, m_nexScene->itemsBoundingRect().height(), 1, 1, 0, 0);
	m_nexView->update();
}

void VideoWindow::nexCount(int count)
{
	m_nexCount->setPlainText(QString("NEX: %1").arg(count));	
}

void VideoWindow::boundingRect(QRectF rect, bool show)
{
	m_videoScene->setSelection(rect);
	m_videoScene->showSelection(show);
	m_videoView->update();
}

void VideoWindow::info(QString infoString)
{
	m_info->setPlainText(infoString);
}

void VideoWindow::plot(const QVariant item, QColor color)
{
	if (item.canConvert(QVariant::Point))
	{
		QPoint point = item.toPoint();
		QGraphicsEllipseItem* dot = new QGraphicsEllipseItem(point.x(), point.y(), 3, 3);
		dot->setBrush(QBrush(color));
		m_videoScene->addItem(dot);
		m_points.push_back(dot);
	}
	else if (item.canConvert(QVariant::List))
	{
		QList<QVariant> list = item.toList();
		QList<QVariant>::const_iterator lvit;
		for (lvit=list.begin(); lvit != list.end(); ++lvit)
		{
			QVariant item = *lvit;
			if (item.canConvert(QVariant::Point))
			{
				QPoint point = item.toPoint();
				QGraphicsEllipseItem* dot = new QGraphicsEllipseItem(point.x(), point.y(), 3, 3);
				dot->setBrush(QBrush(color));
				m_videoScene->addItem(dot);
				m_points.push_back(dot);
			}
		}
	}
}

void VideoWindow::clearDisplay()
{
	QList<QGraphicsEllipseItem*>::iterator pit;
	for (pit=m_points.begin(); pit != m_points.end(); ++pit)
	{
		QGraphicsEllipseItem* point = *pit;
		m_videoScene->removeItem(point);
		delete point;
	}
	m_points.clear();
}

void VideoWindow::confidence(double conf)
{
	if (m_videoScene->selectionVisible())
	{
		if (conf > 0)
		{
			m_confidence->setPlainText(QString("%1").arg(conf));
			m_confidence->setPos(m_videoScene->getSelection().center());
			m_confidence->show();
		}
		else m_confidence->hide();
	}
}

void VideoWindow::detectorBoundingBoxEnabled(bool val)
{
	m_tld.signalBoundingBox(val);
	m_videoScene->showSelection(val);
}

void VideoWindow::detectorPointsEnabled(bool val)
{
	m_tld.signalPoints(val);
	clearDisplay();
}

void VideoWindow::detectorConfidenceEnabled(bool val)
{
	m_tld.signalConfidence(val);
	confidence(0);
}

void VideoWindow::detectorDetectionsEnabled(bool val)
{
	m_tld.signalDetections(val);
	clearDisplay();
}

void VideoWindow::detectorPositiveEnabled(bool val)
{
	m_tld.signalPositive(val);
}

void VideoWindow::detectorNegativeEnabled(bool val)
{
	m_tld.signalNegative(val);
}

void VideoWindow::detectorFlipImageEnabled(bool val)
{
	m_tld.flipImage(val);
}

void VideoWindow::display(QRectF region)
{
	if (m_tld.trained())
		m_tld.display(region);
}

void VideoWindow::display(IplImage *frame)
{
    if (!m_tld.trained() && !m_frame)
        openCARMEN(frame);
    else
        m_tld.display(frame);
}

int VideoWindow::updateIPC(int i)
{
    carmen_ipc_sleep(0.01);
    carmen_graphics_update_ipc_callbacks_qt(this, SLOT(updateIPC(int)));
    i = 1; // only to make the compiler happy

    return i;
}
