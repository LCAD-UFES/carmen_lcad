/*
 * Vista 3D (OpenGL classico, como o viewer_3D): mapa no chao, robo em caixa inclinada pela IMU,
 * rotas/frenets em linhas, anotacoes em postes com nome, objetos moveis em caixas.
 *
 * Camera orbital: arrastar (esq.) gira, arrastar (dir./meio) move, roda = zoom, clique = ponto no chao.
 * Tudo e' desenhado relativo a uma origem local: coordenadas UTM (~7,7e6) nao cabem em float.
 */
#ifndef CARMEN_APP_MAP_VIEW_3D_H
#define CARMEN_APP_MAP_VIEW_3D_H

#include "scene.h"
#include "car_model.h"

#include <QMatrix4x4>
#include <QOpenGLWidget>
#include <functional>

class QPainter;
class QToolButton;

class MapView3D : public QOpenGLWidget, public MapView
{
public:
	explicit MapView3D(Scene *scene);
	~MapView3D() override;

	std::function<void(double, double)> on_click;
	/* modo de escolher pose (1 = pose inicial, 2 = destino, 3 = rota ate'): clicar marca a posicao,
	   arrastar aponta; ao soltar chama on_pick(x, y, theta). 0 = navegar (arrastar gira a camera). */
	std::function<void(double, double, double)> on_pick;
	void set_pick_mode(int mode);

	void refresh() override { update(); }
	void follow_robot() override;
	void zoom_on_robot() override;
	void look_at(double x, double y) override;
	void fit_rect(const QRectF &r) override;
	void set_orbit(double d, double p) { dist = d; pitch = p; update(); }
	void top_view();				// liga/desliga a vista de cima (o olhinho no canto)
	void set_car_model(const QByteArray &obj, const double size[3], double yaw);
	bool has_car_model() const { return !car.empty(); }

	void paint_labels(QPainter &p);	// textos por cima (chamado pelo widget transparente de cima)

protected:
	void initializeGL() override;
	void resizeGL(int w, int h) override;
	void paintGL() override;
	void mousePressEvent(QMouseEvent *ev) override;
	void mouseMoveEvent(QMouseEvent *ev) override;
	void mouseReleaseEvent(QMouseEvent *ev) override;
	void wheelEvent(QWheelEvent *ev) override;

private:
	void update_matrices();
	bool ground_point(const QPointF &screen, double *wx, double *wy) const;
	bool project(double wx, double wy, double wz, QPointF *screen) const;
	void ensure_origin();
	void upload_texture(unsigned int *tex, const QImage &img);

	void draw_grid();
	void draw_map(unsigned int tex, const QRectF &r, double z);
	void draw_line(const QPolygonF &poly, const QColor &c, double width, double z);
	void draw_box(double x, double y, double z0, double theta, double length, double width, double height,
			double back, const QColor &fill, const QColor &edge, double roll = 0, double pitch = 0);
	void draw_ring(double x, double y, double radius, const QColor &c, double width);
	void draw_axes(double x, double y, double z, double theta, double roll, double pitch, double size);
	void draw_robot(double x, double y, double theta, double roll, double pitch, float alpha);
	void draw_flag(double x, double y, double theta, const QColor &c, float alpha);
	void draw_lidar();
	void draw_pick_ghost();
	void draw_camera_billboards();
	void leaveEvent(QEvent *ev) override;

	Scene *s;
	bool have_origin = false;
	double ox = 0, oy = 0;			// origem local (metros)
	double tx = 0, ty = 0;			// alvo da camera (mundo)
	double yaw = -1.2, pitch = 0.95, dist = 30.0;
	bool have_target = false;
	QMatrix4x4 proj, view;

	CarModel car;
	QToolButton *eye = nullptr;		// botao do olhinho (vista de cima)
	bool in_top_view = false;
	double saved_yaw = 0, saved_pitch = 0;

	QWidget *labels = nullptr;
	QWidget *camera_panel = nullptr;	// camera no canto superior esquerdo (como no viewer_3D_dash)
	std::map<int, std::pair<unsigned int, int> > camera_tex;	// id -> (textura, versao enviada)
public:
	void paint_camera_panel(QPainter &p, int w, int h);
	bool camera_big = false;
private:		// widget transparente por cima, so' para os textos
	unsigned int map_tex = 0, full_tex = 0, layer_tex = 0;
	int map_version = -1, full_version = -1, layer_version = -1;

	int pick_mode = 0;
	bool picking = false, have_hover = false;
	double pick_x = 0, pick_y = 0, pick_theta = 0, hover_x = 0, hover_y = 0;

	QPoint press_pos, last_drag;
	Qt::MouseButton drag_button = Qt::NoButton;
	bool dragging = false;
};

#endif
