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

VehicleTrackerGUI::VehicleTrackerGUI(QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::VehicleTrackerGUI),
	display(&scene)
{
	ui->setupUi(this);

	scene.setItemIndexMethod(QGraphicsScene::NoIndex);
	ui->graphicsView->setScene(&scene);
	ui->graphicsView->scale(1, -1);

	const g2d::Point &point = display.point(0.5, 2);
	display.point(0.7, 2);
	const g2d::Polygon rect = display.rectangle(0.5, 2, 0.5, 2, 0.7);
	display.circle(0, 0, 6);

	std::cout << display.canvas.points.size() << std::endl;

	std::cout << "Point " << point << " is " << (is_inside(point, rect) ? "inside" : "outside") << " rectangle " << rect << std::endl;

	//display.canvas.points.at(1) = g2d::Point(0, 0);
	//display.points.modelChanged();
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
