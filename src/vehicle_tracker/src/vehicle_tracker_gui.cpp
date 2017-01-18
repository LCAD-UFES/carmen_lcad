#include "vehicle_tracker_gui.h"
#include "ui_vehicle_tracker_gui.h"

bool is_inside(const g2d::Point &point, const g2d::Polygon &polygon)
{
	CGAL::Bounded_side side = CGAL::bounded_side_2(
		polygon.vertices_begin(),
		polygon.vertices_end(),
		point,
		g2d::Kernel()
	);

	return (side == CGAL::ON_BOUNDED_SIDE);
}

std::string relation(const g2d::Point &point, const g2d::Polygon &rect)
{
	CGAL::Bounded_side side = CGAL::bounded_side_2(
		rect.vertices_begin(),
		rect.vertices_end(),
		point,
		g2d::Kernel()
	);

	if (side == CGAL::ON_BOUNDED_SIDE)
		return "inside";

	if (side == CGAL::ON_BOUNDARY)
		return "on";

	g2d::Point center = g2d::centroid(rect);
	g2d::Field d_center = (center - CGAL::ORIGIN).squared_length();
	g2d::Field d_point = (point - CGAL::ORIGIN).squared_length();

	if (d_point < d_center)
		return "before";

	return "after";
}

void relation_message(const g2d::Point &point, const g2d::Polygon &rect)
{
	std::cout << "Point [" << point << "] is " << relation(point, rect) << " the rectangle [" << rect << "]" << std::endl;
}

VehicleTrackerGUI::VehicleTrackerGUI(int argc, char **argv, QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::VehicleTrackerGUI),
	display(&scene),
	scan(new VirtualScan(argc, argv, &display, this))
{
	ui->setupUi(this);

	scene.setItemIndexMethod(QGraphicsScene::NoIndex);
	ui->graphicsView->setScene(&scene);
	ui->graphicsView->scale(1, -1);

	display.circle(0, 0, 60);
	display.rectangle(0, 0, 4.87, 1.85, 0.0);
}

VehicleTrackerGUI::~VehicleTrackerGUI()
{
	delete ui;
}

void VehicleTrackerGUI::resizeEvent(QResizeEvent * event)
{
	QMainWindow::resizeEvent(event);

	ui->graphicsView->fitInView(scene.sceneRect());
}
