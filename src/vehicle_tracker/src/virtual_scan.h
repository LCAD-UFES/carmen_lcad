#ifndef CARMEN_IPC_H
#define CARMEN_IPC_H

#include "display.h"

#include <QObject>
#include <QTimer>

class VirtualScan : public QObject
{
	Q_OBJECT

	g2d::Display *display;

	QTimer *timer;

public:
	explicit VirtualScan(int argc, char **argv, g2d::Display *display, QObject *parent = 0);

	virtual ~VirtualScan();

signals:

public slots:
	void retrieve();
};

#endif // CARMEN_IPC_H
