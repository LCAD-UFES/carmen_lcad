#ifndef CARMEN_IPC_H
#define CARMEN_IPC_H

#include "display.h"
#include "virtual_scan.h"

#include <QObject>
#include <QTimer>

class DifferenceScanner : public QObject
{
	Q_OBJECT

	g2d::Display *display;

	VirtualScan s1;

	QTimer *timer;

public:
	explicit DifferenceScanner(g2d::Display *display, QObject *parent = 0);

	virtual ~DifferenceScanner();

signals:

public slots:
	void update();
};

#endif // CARMEN_IPC_H
