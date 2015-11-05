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

#ifndef VIDEOWINDOW_H_
#define VIDEOWINDOW_H_

#include "DetectorObserver.h"
#include "CaptureDevice.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <QDir>
#include <QFrame>
#include <QStatusBar>
#include <QGraphicsEllipseItem>

class QTimer;
class QFrame;
class VideoTextItem;
class QGraphicsScene;
class QGraphicsView;
class ImageWidget;
class VideoGraphicsView;
class VideoGraphicsScene;
class QHBoxLayout;
class QGraphicsWidget;
class QGraphicsGridLayout;
class QGraphicsRectItem;
class QStringList;

class VideoWindow : public QFrame
{
	Q_OBJECT      

public:
	VideoWindow(QStatusBar& sb, QWidget *parent=0);
	~VideoWindow();

	bool openFile(QString& filename);
	bool openDirectory(QString& dirname);
	bool openCamera();
	bool openCARMEN(IplImage *frame);
	void init();
	void close();
	void clear();
	void adjust(IplImage* inFrame, IplImage* outFrame = NULL);
	void display(QRectF region);
	void display(IplImage *frame);

signals:
	void status(const QString message);

public slots:
    int updateIPC(int i);

public slots:
	void newFrame(IplImage* frame);
	void boundingRect(QRectF rect, bool show = true);
	void selectionChanged(const QRectF rect);
	void autoStart(bool val);
	void fpsLimit(int val);
	void brightness(int val);
	void contrast(int val);
	void detectorBoundingBoxEnabled(bool val);
	void detectorPointsEnabled(bool val);
	void detectorConfidenceEnabled(bool val);
	void detectorDetectionsEnabled(bool val);
	void detectorPositiveEnabled(bool val);
	void detectorNegativeEnabled(bool val);
	void detectorFlipImageEnabled(bool val);

private slots:
	void startTimers();
	void stopTimers();
	void setCountersVisible(bool visible = true);
	void updateCounters();
	void processFrame();
	void startDetection();
	void displayPexImage(QPixmap pixmap);
	void pexCount(int count);
	void displayNexImage(QPixmap pixmap);
	void nexCount(int count);
	void info(QString infoStr);
	void plot(const QVariant item, QColor color);
	void clearDisplay();
	void confidence(double conf);

private:
	typedef enum {V_CAMERA=0, V_FILE=1, V_DIR=2} VideoSourceType;
	QDir m_dir;
	QStringList m_inputFiles;
	unsigned int m_frameNumber;
	unsigned int m_frameCount;
	QTimer* m_frameTimer;
	QTimer* m_updateTimer;
	QFrame* m_videoFrame;
	VideoTextItem* m_fn;
	VideoTextItem* m_fps;
	VideoTextItem* m_info;
	VideoTextItem* m_pexCount;
	VideoTextItem* m_nexCount;
	VideoTextItem* m_confidence;
	VideoTextItem* m_bbx;
	VideoTextItem* m_bby;
	VideoTextItem* m_bbw;
	VideoTextItem* m_bbh;
	QGraphicsGridLayout* m_textLayout;
	QGraphicsWidget* m_textForm;
	QGraphicsScene* m_nexScene;
	QGraphicsView* m_nexView;
	int m_pexRow;
	int m_pexCol;
	QGraphicsScene* m_pexScene;
	QGraphicsView* m_pexView;
	VideoGraphicsScene* m_videoScene;
	VideoGraphicsView* m_videoView;
	ImageWidget *m_cameraWidget;
	QHBoxLayout *m_layout;
	IplImage* m_frame;
	DetectorObserver m_tld;
	VideoSourceType m_videoSource;
	qreal m_pexPosition;
	QList<QGraphicsEllipseItem*> m_points;
	bool m_autoStart;
	int m_fpsLimit;
	QRectF m_selection;
	CaptureDevice* m_captureDevice;
	QStatusBar& m_statusbar;
	int m_brightness;
	int m_contrast;
};


#endif /* VIDEOWINDOW_H_ */
