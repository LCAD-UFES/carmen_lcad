#ifndef VEHICLE_TRACKER_GUI_H
#define VEHICLE_TRACKER_GUI_H

#include "display.h"
#include "virtual_scan.h"

#include <QMainWindow>

namespace Ui
{

class VehicleTrackerGUI;

}

class VehicleTrackerGUI : public QMainWindow
{
    Q_OBJECT

	QGraphicsScene scene;

public:
	explicit VehicleTrackerGUI(int argc, char **argv, QWidget *parent = 0);

	/** \brief Class destructor. */
	~VehicleTrackerGUI();

protected:
	virtual void resizeEvent(QResizeEvent * event);

private:
    Ui::VehicleTrackerGUI *ui;

	g2d::Display display;

	VirtualScan *scan;
};

#endif // VEHICLE_TRACKER_GUI_H
