/*
 * Estado do robo e de todas as camadas do mapa. A janela preenche, a vista 3D so' le'.
 */
#ifndef CARMEN_APP_SCENE_H
#define CARMEN_APP_SCENE_H

#include <QColor>
#include <QImage>
#include <QJsonObject>
#include <QPointF>
#include <QPolygonF>
#include <QRectF>
#include <QString>
#include <QVector3D>
#include <deque>
#include <map>
#include <vector>

double now_seconds();

struct RobotState
{
	/* pose */
	double x = 0, y = 0, theta = 0, v = 0, phi = 0;
	bool have_pose = false, converged = false;
	double t_pose = 0;
	/* comando e CAN */
	double v_cmd = 0, phi_cmd = 0, t_cmd = 0;
	double throttle = 0, brake = 0, steering_pct = 0;
	QString gear;
	int engine = 0, parking_brake = 0, turn_signal = 0, headlights = 0, horn = 0;
	double t_car = 0;
	/* derivados */
	double a_long = 0, a_lat = 0, last_v = 0, last_v_t = 0, last_moving = 0;
	QString gear_estimated = "N";
	std::deque<float> speed_history;
	/* IMU e GPS */
	double roll = 0, pitch = 0, t_imu = 0;
	int gps_quality = -1, gps_sats = 0;
	double gps_hdop = 0, t_gps = 0;
	/* estados */
	QString behavior = "-", task = "-", route_state = "-", mission_state = "-", mission_id;
	int autonomous = 0;
	double goal_distance = -1;
	double t_behavior = 0, t_frenet = 0, t_route = 0;
	int frenet_paths = 0, frenet_selected = -1;
	/* robo (carmen.ini) */
	double length = 0.70, width = 0.40, axle = 0.35, rear_overhang = 0.15, max_v = 1.0, max_phi = 0.6;
	std::vector<QVector3D> collision;	// circulos do robot_collision_file: x, y (do eixo traseiro), raio
};

struct Layers
{
	bool map = true, lanes = true, rddf = true, frenets = true, route = true, path = true, plan = true,
		 avoider = true, offroad = true, annotations = true, goals = true, objects = true, gps = true,
		 fused = false, particles = true, grid = true, car_model = true, labels = true,
		 collision = false, imu_axis = false, gps_axis = false, globalpos_trail = false, waypoints = false,
		 lidar = false, lidar_intensity = false, map_overlay = true,
		 camera_corner = true, camera_map = false;
};

struct MovingObject
{
	double x, y, theta, length, width, height, v;
	int id;
};

struct Annotation
{
	QString text;
	double x, y, theta;
	int type;
};

struct Scene
{
	RobotState robot;
	Layers layers;

	/* mapas (imagem ARGB e retangulo em metros) */
	QImage map_image, full_image;
	QRectF map_rect, full_rect;
	bool show_full_map = false;
	int map_version = 0, full_version = 0;	// muda a cada mapa novo (a vista 3D reenvia a textura)

	/* mapa escolhido no menu Mapas (mapper, custo, likelihood...); o offline continua em map_image */
	QString map_choice = "mapper";	// padrao: o mapa ao vivo do mapper por cima do offline
	bool layer_colored = false;		// layer_image so' com os obstaculos coloridos (transparente no resto)
	QImage layer_image;
	QRectF layer_rect;
	int layer_version = 0;

	/* nuvem do lidar no referencial do carro (m) + pose do robo quando foi medida */
	std::vector<float> lidar_xyz;
	std::vector<unsigned char> lidar_intensity;
	double lidar_x = 0, lidar_y = 0, lidar_theta = 0;
	bool have_lidar = false;

	/* cameras (JPEG do servidor). camera_left aparece no canto e a esquerda no mapa; camera_right a direita */
	struct Camera { QImage image; int version = 0; double t = 0; };
	std::map<int, Camera> cameras;
	int camera_left = 1, camera_right = -1;

	std::deque<QPointF> trail;	// globalpos recente (opcao "Global Pos")

	QPolygonF route, path, plan, avoider, offroad, goals, rddf;
	std::vector<QPolygonF> lanes, frenets;
	std::vector<double> frenet_collision;	// por caminho: fracao do comprimento onde bate (-1 = livre)
	int frenet_selected = -1;
	std::vector<Annotation> annotations;
	std::vector<MovingObject> objects;
	std::vector<QPointF> particles;
	QPointF gps, fused;
	double fused_theta = 0;
	bool have_gps = false, have_fused = false;

	QPointF final_goal;
	double final_goal_theta = 0;
	bool have_final_goal = false;

	QPolygonF preview;			// seta da pose/destino sendo enviada
	QPointF highlight;			// lugar escolhido na lista
	bool have_highlight = false;

	bool contains_map(double x, double y, double margin) const
	{
		return !map_image.isNull() && map_rect.adjusted(margin, margin, -margin, -margin).contains(QPointF(x, y));
	}
};

/* Converte [[x,y,theta,v],...] do protocolo em poligono. */
class QJsonArray;
QPolygonF json_to_polygon(const QJsonArray &pts);

/* O que a janela pede para a vista (hoje so' a 3D). */
class MapView
{
public:
	virtual ~MapView() {}
	virtual void refresh() = 0;						// redesenhar
	virtual void follow_robot() = 0;				// centraliza no robo (chamado a cada pose se "seguir" estiver ligado)
	virtual void zoom_on_robot() = 0;
	virtual void look_at(double x, double y) = 0;	// leva a vista ate' um ponto
	virtual void fit_rect(const QRectF &r) = 0;		// enquadra um retangulo (mapa completo)
};

#endif
