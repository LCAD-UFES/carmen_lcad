#ifndef DETECTOR_OBSERVER_H_
#define DETECTOR_OBSERVER_H_

#include <carmen/visual_tracker_messages.h>
#include <opencv/cv.h>
#include <QObject>
#include <QRectF>

class DetectorObserver : public QObject
{
	Q_OBJECT
    private:
	QRectF m_rect;
	carmen_visual_tracker_test_message m_test;
	carmen_visual_tracker_train_message m_train;
    public:
	DetectorObserver();
	~DetectorObserver();
        bool trained();
        void train(IplImage *frame, QRectF selection);
        void test(IplImage *frame);
        void display(IplImage *frame);
        void display(QRectF region);
    signals:
        void boundingRect(QRectF rect);
        void newFrame(IplImage* image);
    public slots:
        void signalBoundingBox(bool val);
        void signalPoints(bool val);
        void signalConfidence(bool val);
        void signalDetections(bool val);
        void signalPositive(bool val);
        void signalNegative(bool val);
        void flipImage(bool val);
};

#endif /* DETECTOR_OBSERVER_H_ */
