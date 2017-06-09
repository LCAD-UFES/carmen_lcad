#ifndef VEHICLE_TRACKER_GUI_H
#define VEHICLE_TRACKER_GUI_H

#include "display.h"

#include <QMainWindow>
#include <QTimer>

namespace Ui {
	class VehicleTrackerGUI;
}

/**
 * @brief Graphical User Interface (GUI) for the Vehicle Tracker module.
 */
class VehicleTrackerGUI: public QMainWindow
{
	Q_OBJECT

	/** @brief Timer object used to regularly poll the CARMEN network. */
	QTimer *timer;

	/** @brief Object encapsulating the specification of the user interface. */
	Ui::VehicleTrackerGUI *ui;

	/** @brief Scene object where geometric forms are drawn. */
	QGraphicsScene scene;

	/** @brief Collection of displayed geometric objects. */
	g2d::Display display_scan;

	/** @brief Collection of displayed geometric objects. */
	g2d::Display display_map;

	/** @brief Collection of displayed geometric objects. */
	g2d::Display display_clusters;

protected:
	/**
	 * @brief Resize event handler.
	 */
	virtual void resizeEvent(QResizeEvent * event);

public:
	/**
	 * @brief Create a new window.
	 *
	 * If `parent` is not given, becomes a top-level window.
	 */
	explicit VehicleTrackerGUI(QWidget *parent = NULL);

	/**
	 * @brief Object destructor.
	 */
	~VehicleTrackerGUI();

signals:

public slots:
	void update();
};

#endif // VEHICLE_TRACKER_GUI_H
