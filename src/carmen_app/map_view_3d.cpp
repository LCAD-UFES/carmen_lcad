#include "map_view_3d.h"
#include "theme.h"

#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QToolButton>
#include <QVector4D>
#include <QWheelEvent>
#include <GL/gl.h>
#include <algorithm>
#include <cmath>

static void
gl_color(const QColor &c)
{
	glColor4f(c.redF(), c.greenF(), c.blueF(), c.alphaF());
}

/* Desenhar texto com QPainter no mesmo framebuffer do OpenGL classico nao e' confiavel (some em
   alguns drivers). Os textos ficam num widget transparente por cima, que nao pega o mouse. */
class Labels3D : public QWidget
{
public:
	explicit Labels3D(MapView3D *v) : QWidget(v), view(v)
	{
		setAttribute(Qt::WA_TransparentForMouseEvents);
		setAttribute(Qt::WA_NoSystemBackground);
		setAttribute(Qt::WA_TranslucentBackground);
	}

protected:
	void paintEvent(QPaintEvent *) override
	{
		QPainter p(this);
		view->paint_labels(p);
	}

private:
	MapView3D *view;
};

static QIcon
eye_icon(const QColor &c)
{
	QPixmap pm(64, 64);
	pm.fill(Qt::transparent);
	QPainter p(&pm);
	p.setRenderHint(QPainter::Antialiasing, true);
	QPainterPath lid;
	lid.moveTo(4, 32);
	lid.quadTo(32, 4, 60, 32);
	lid.quadTo(32, 60, 4, 32);
	p.setPen(QPen(c, 5));
	p.setBrush(Qt::NoBrush);
	p.drawPath(lid);
	p.setPen(Qt::NoPen);
	p.setBrush(c);
	p.drawEllipse(QPointF(32, 32), 10, 10);
	p.setBrush(QColor(20, 24, 32));
	p.drawEllipse(QPointF(32, 32), 4, 4);
	return QIcon(pm);
}

/* Camera no canto: um widget por cima do mapa; clique alterna entre pequena e grande. */
class CameraPanel : public QWidget
{
public:
	explicit CameraPanel(MapView3D *v) : QWidget(v), view(v)
	{
		setCursor(Qt::PointingHandCursor);
		setToolTip("Clique para aumentar/diminuir");
	}

protected:
	void paintEvent(QPaintEvent *) override
	{
		QPainter p(this);
		view->paint_camera_panel(p, width(), height());
	}
	void mousePressEvent(QMouseEvent *) override
	{
		view->camera_big = !view->camera_big;
		view->update();
	}

private:
	MapView3D *view;
};

MapView3D::MapView3D(Scene *scene) : s(scene)
{
	setMinimumSize(500, 400);
	setMouseTracking(true);
	labels = new Labels3D(this);
	labels->resize(size());
	camera_panel = new CameraPanel(this);
	camera_panel->hide();

	/* olhinho no canto: alterna para a vista de cima e volta para a anterior */
	eye = new QToolButton(this);
	eye->setObjectName("eyebtn");
	eye->setCheckable(true);
	eye->setToolTip("Vista de cima");
	eye->setIcon(eye_icon(QColor(230, 236, 245)));
	eye->setIconSize(QSize(30, 30));
	eye->setFixedSize(44, 44);
	eye->setCursor(Qt::PointingHandCursor);
	QObject::connect(eye, &QToolButton::clicked, [this]() { top_view(); });
}

void
MapView3D::set_car_model(const QByteArray &obj, const double size[3], double yaw_model)
{
	car.load(obj, size, yaw_model);
	update();
}

void
MapView3D::resizeGL(int w, int h)
{
	labels->resize(w, h);
	eye->move(w - eye->width() - 12, 12);
}

MapView3D::~MapView3D()
{
	makeCurrent();
	if (map_tex)
		glDeleteTextures(1, &map_tex);
	if (full_tex)
		glDeleteTextures(1, &full_tex);
	if (layer_tex)
		glDeleteTextures(1, &layer_tex);
	for (auto &t : camera_tex)
		glDeleteTextures(1, &t.second.first);
	doneCurrent();
}

void
MapView3D::initializeGL()
{
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_LINE_SMOOTH);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
}

/* ------------------------------------------------------------------------------------------ */
/* Camera                                                                                     */
/* ------------------------------------------------------------------------------------------ */

void
MapView3D::ensure_origin()
{
	if (have_origin)
		return;
	if (s->robot.have_pose)
	{
		ox = s->robot.x;
		oy = s->robot.y;
	}
	else if (!s->map_image.isNull())
	{
		ox = s->map_rect.center().x();
		oy = s->map_rect.center().y();
	}
	else
		return;
	have_origin = true;
	if (!have_target)
	{
		tx = ox;
		ty = oy;
		have_target = true;
	}
}

void
MapView3D::follow_robot()
{
	if (s->robot.have_pose)
	{
		tx = s->robot.x;
		ty = s->robot.y;
		have_target = true;
	}
	update();
}

void
MapView3D::zoom_on_robot()
{
	dist = 25.0;
	pitch = 0.95;
	follow_robot();
}

void
MapView3D::look_at(double x, double y)
{
	tx = x;
	ty = y;
	dist = std::min(dist, 30.0);
	have_target = true;
	update();
}

void
MapView3D::fit_rect(const QRectF &r)
{
	if (r.isEmpty())
		return;
	tx = r.center().x();
	ty = r.center().y();
	dist = std::max(r.width(), r.height()) * 0.9;
	pitch = 1.25;
	have_target = true;
	update();
}

void
MapView3D::top_view()
{
	if (!in_top_view)
	{
		saved_yaw = yaw;
		saved_pitch = pitch;
		pitch = 1.56;
		yaw = -M_PI / 2;	// norte (y) para cima
		in_top_view = true;
	}
	else
	{
		yaw = saved_yaw;
		pitch = saved_pitch;
		in_top_view = false;
	}
	eye->setChecked(in_top_view);
	update();
}

void
MapView3D::update_matrices()
{
	double aspect = std::max(1, width()) / (double) std::max(1, height());
	proj.setToIdentity();
	proj.perspective(45.0f, (float) aspect, 0.2f, 20000.0f);
	/* alvo em coordenadas locais; a camera orbita em volta dele */
	QVector3D target((float) (tx - ox), (float) (ty - oy), 0.0f);
	QVector3D eye = target + QVector3D((float) (dist * cos(pitch) * cos(yaw)), (float) (dist * cos(pitch) * sin(yaw)), (float) (dist * sin(pitch)));
	view.setToIdentity();
	view.lookAt(eye, target, QVector3D(0, 0, 1));
}

bool
MapView3D::project(double wx, double wy, double wz, QPointF *screen) const
{
	QVector4D c = proj * view * QVector4D((float) (wx - ox), (float) (wy - oy), (float) wz, 1.0f);
	if (c.w() <= 0.01f)
		return false;
	*screen = QPointF((c.x() / c.w() * 0.5 + 0.5) * width(), (1.0 - (c.y() / c.w() * 0.5 + 0.5)) * height());
	return true;
}

bool
MapView3D::ground_point(const QPointF &screen, double *wx, double *wy) const
{
	QMatrix4x4 inv = (proj * view).inverted();
	double nx = 2.0 * screen.x() / width() - 1.0, ny = 1.0 - 2.0 * screen.y() / height();
	QVector4D a = inv * QVector4D((float) nx, (float) ny, -1.0f, 1.0f);
	QVector4D b = inv * QVector4D((float) nx, (float) ny, 1.0f, 1.0f);
	QVector3D p0 = a.toVector3DAffine(), p1 = b.toVector3DAffine();
	QVector3D d = p1 - p0;
	if (fabs(d.z()) < 1e-6)
		return false;
	double t = -p0.z() / d.z();
	if (t < 0)
		return false;
	*wx = p0.x() + t * d.x() + ox;
	*wy = p0.y() + t * d.y() + oy;
	return true;
}

/* ------------------------------------------------------------------------------------------ */
/* Primitivas                                                                                 */
/* ------------------------------------------------------------------------------------------ */

void
MapView3D::upload_texture(unsigned int *tex, const QImage &img)
{
	if (img.isNull())
		return;
	QImage rgba = img.convertToFormat(QImage::Format_RGBA8888);
	if (!*tex)
		glGenTextures(1, tex);
	glBindTexture(GL_TEXTURE_2D, *tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, rgba.width(), rgba.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, rgba.constBits());
	glBindTexture(GL_TEXTURE_2D, 0);
}

void
MapView3D::draw_map(unsigned int tex, const QRectF &r, double z)
{
	if (!tex)
		return;
	/* a linha 0 da imagem e' y_origin: t = 0 em r.top() (que no QRectF e' o y menor) */
	double x0 = r.left() - ox, x1 = r.right() - ox, y0 = r.top() - oy, y1 = r.bottom() - oy;
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, tex);
	glColor4f(1, 1, 1, 1);
	glBegin(GL_QUADS);
	glTexCoord2f(0, 0); glVertex3d(x0, y0, z);
	glTexCoord2f(1, 0); glVertex3d(x1, y0, z);
	glTexCoord2f(1, 1); glVertex3d(x1, y1, z);
	glTexCoord2f(0, 1); glVertex3d(x0, y1, z);
	glEnd();
	glBindTexture(GL_TEXTURE_2D, 0);
	glDisable(GL_TEXTURE_2D);
}

void
MapView3D::draw_grid()
{
	/* grade de 5 m em volta do alvo, sumindo com a distancia */
	double step = (dist > 150) ? 50.0 : (dist > 60 ? 10.0 : 5.0);
	double half = std::max(60.0, dist * 1.5);
	double cx = floor((tx - ox) / step) * step, cy = floor((ty - oy) / step) * step;
	glLineWidth(1.0f);
	glBegin(GL_LINES);
	for (double d = -half; d <= half; d += step)
	{
		glColor4f(0.35f, 0.42f, 0.55f, 0.18f);
		glVertex3d(cx + d, cy - half, -0.03);
		glVertex3d(cx + d, cy + half, -0.03);
		glVertex3d(cx - half, cy + d, -0.03);
		glVertex3d(cx + half, cy + d, -0.03);
	}
	glEnd();
}

void
MapView3D::draw_line(const QPolygonF &poly, const QColor &c, double width, double z)
{
	if (poly.size() < 2)
		return;
	glLineWidth((float) width);
	gl_color(c);
	glBegin(GL_LINE_STRIP);
	for (const QPointF &p : poly)
		glVertex3d(p.x() - ox, p.y() - oy, z);
	glEnd();
}

void
MapView3D::draw_ring(double x, double y, double radius, const QColor &c, double width)
{
	glLineWidth((float) width);
	gl_color(c);
	glBegin(GL_LINE_LOOP);
	for (int i = 0; i < 48; i++)
	{
		double a = i * 2.0 * M_PI / 48;
		glVertex3d(x - ox + radius * cos(a), y - oy + radius * sin(a), 0.06);
	}
	glEnd();
}

void
MapView3D::draw_box(double x, double y, double z0, double theta, double length, double width, double height,
		double back, const QColor &fill, const QColor &edge, double roll, double pitch_angle)
{
	glPushMatrix();
	glTranslated(x - ox, y - oy, z0);
	glRotated(theta * 180.0 / M_PI, 0, 0, 1);
	glRotated(-pitch_angle * 180.0 / M_PI, 0, 1, 0);
	glRotated(roll * 180.0 / M_PI, 1, 0, 0);
	double x0 = -back, x1 = length - back, y0 = -width / 2, y1 = width / 2, z1 = height;
	const double v[8][3] = {{x0, y0, 0}, {x1, y0, 0}, {x1, y1, 0}, {x0, y1, 0}, {x0, y0, z1}, {x1, y0, z1}, {x1, y1, z1}, {x0, y1, z1}};
	const int faces[6][4] = {{0, 1, 2, 3}, {4, 5, 6, 7}, {0, 1, 5, 4}, {2, 3, 7, 6}, {1, 2, 6, 5}, {0, 3, 7, 4}};
	const float shade[6] = {0.55f, 1.0f, 0.75f, 0.75f, 0.9f, 0.65f};	// luz fake por face
	if (fill.alpha() > 0)
	{
		glBegin(GL_QUADS);
		for (int f = 0; f < 6; f++)
		{
			glColor4f(fill.redF() * shade[f], fill.greenF() * shade[f], fill.blueF() * shade[f], fill.alphaF());
			for (int k = 0; k < 4; k++)
				glVertex3dv(v[faces[f][k]]);
		}
		glEnd();
	}
	glLineWidth(1.5f);
	gl_color(edge);
	const int edges[12][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
	glBegin(GL_LINES);
	for (int e = 0; e < 12; e++)
	{
		glVertex3dv(v[edges[e][0]]);
		glVertex3dv(v[edges[e][1]]);
	}
	glEnd();
	glPopMatrix();
}

/* ------------------------------------------------------------------------------------------ */
/* Desenho                                                                                    */
/* ------------------------------------------------------------------------------------------ */

void
MapView3D::paintGL()
{
	ensure_origin();
	update_matrices();
	const Layers &L = s->layers;
	const RobotState &r = s->robot;


	glClearColor(pal::bg.redF(), pal::bg.greenF(), pal::bg.blueF(), 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glMatrixMode(GL_PROJECTION);
	glLoadMatrixf(proj.constData());
	glMatrixMode(GL_MODELVIEW);
	glLoadMatrixf(view.constData());

	if (have_origin)
	{
		if (map_version != s->map_version)
		{
			upload_texture(&map_tex, s->map_image);
			map_version = s->map_version;
		}
		if (full_version != s->full_version)
		{
			upload_texture(&full_tex, s->full_image);
			full_version = s->full_version;
		}
		if (layer_version != s->layer_version)
		{
			upload_texture(&layer_tex, s->layer_image);
			layer_version = s->layer_version;
		}

		if (L.grid)
			draw_grid();
		bool offline = (s->map_choice == "offline");
		if (L.map && s->show_full_map)
			draw_map(full_tex, s->full_rect, -0.02);
		if (L.map && (offline || L.map_overlay || s->layer_image.isNull()))
			draw_map(map_tex, s->map_rect, 0.0);
		if (L.map && !offline && !s->layer_image.isNull())
		{
			/* mapa escolhido no menu Mapas: por cima do offline (sobreposto) ou no lugar dele */
			if (L.map_overlay)
			{
				glDepthMask(GL_FALSE);
				glEnable(GL_TEXTURE_2D);
				glBindTexture(GL_TEXTURE_2D, layer_tex);
				glColor4f(1, 1, 1, s->layer_colored ? 1.0f : 0.6f);	// colorido ja' traz a transparencia
				double x0 = s->layer_rect.left() - ox, x1 = s->layer_rect.right() - ox, y0 = s->layer_rect.top() - oy, y1 = s->layer_rect.bottom() - oy;
				glBegin(GL_QUADS);
				glTexCoord2f(0, 0); glVertex3d(x0, y0, 0.01);
				glTexCoord2f(1, 0); glVertex3d(x1, y0, 0.01);
				glTexCoord2f(1, 1); glVertex3d(x1, y1, 0.01);
				glTexCoord2f(0, 1); glVertex3d(x0, y1, 0.01);
				glEnd();
				glBindTexture(GL_TEXTURE_2D, 0);
				glDisable(GL_TEXTURE_2D);
				glDepthMask(GL_TRUE);
			}
			else
				draw_map(layer_tex, s->layer_rect, 0.005);
		}

		/* linhas logo acima do chao (z crescente para nao brigarem no z-buffer) */
		if (L.lanes)
			for (const QPolygonF &l : s->lanes)
				draw_line(l, QColor(120, 135, 160, 190), 1.5, 0.03);
		if (L.route)
			draw_line(s->route, QColor(64, 156, 255, 210), 5.0, 0.04);
		if (L.rddf)
			draw_line(s->rddf, QColor(255, 255, 255, 160), 1.5, 0.05);
		if (L.offroad)
			draw_line(s->offroad, pal::purple, 3.0, 0.05);
		if (L.frenets)
		{
			/* leque de caminhos: verde ate' onde esta' livre, vermelho do ponto de colisao em diante
			   (o carmen_server calcula com o mapa de distancia e os circulos de colisao do robo) */
			auto draw_frenet = [&](int i, double width, const QColor &free_color, double z) {
				const QPolygonF &p = s->frenets[i];
				double hit = (i < (int) s->frenet_collision.size()) ? s->frenet_collision[i] : -1.0;
				if (hit < 0.0 || p.size() < 2)
				{
					draw_line(p, free_color, width, z);
					return;
				}
				int k = std::max(0, std::min((int) p.size() - 1, (int) std::lround(hit * (p.size() - 1))));
				if (k > 0)
					draw_line(p.mid(0, k + 1), free_color, width, z);
				draw_line(p.mid(k), QColor(255, 70, 70, 235), width + 0.5, z + 0.005);
				glPointSize(9.0f);
				glColor4f(1.0f, 0.25f, 0.25f, 1.0f);
				glBegin(GL_POINTS);
				glVertex3d(p[k].x() - ox, p[k].y() - oy, z + 0.01);
				glEnd();
			};
			for (int i = 0; i < (int) s->frenets.size(); i++)
				if (i != s->frenet_selected)
					draw_frenet(i, 2.2, QColor(46, 204, 113, 190), 0.06);
			if (s->frenet_selected >= 0 && s->frenet_selected < (int) s->frenets.size())
				draw_frenet(s->frenet_selected, 4.5, QColor(64, 200, 255, 255), 0.07);
		}
		if (L.path)
			draw_line(s->path, pal::green, 2.0, 0.08);
		if (L.plan)
			draw_line(s->plan, pal::orange, 2.5, 0.09);
		if (L.avoider)
			draw_line(s->avoider, pal::pink, 2.0, 0.10);
		if (s->preview.size() >= 2)
			draw_line(s->preview, pal::purple, 2.5, 0.12);

		if (L.goals)
		{
			glPointSize(7.0f);
			gl_color(pal::yellow);
			glBegin(GL_POINTS);
			for (const QPointF &g : s->goals)
				glVertex3d(g.x() - ox, g.y() - oy, 0.15);
			glEnd();
		}
		if (L.particles && !s->particles.empty())
		{
			glPointSize(2.5f);
			glColor4f(0.7f, 0.4f, 1.0f, 0.6f);
			glBegin(GL_POINTS);
			for (const QPointF &q : s->particles)
				glVertex3d(q.x() - ox, q.y() - oy, 0.1);
			glEnd();
		}
		if (L.annotations)
		{
			glLineWidth(1.5f);
			glBegin(GL_LINES);
			for (const Annotation &a : s->annotations)
			{
				bool place = a.text.startsWith("RDDF_PLACE");
				gl_color(place ? QColor(250, 204, 21, 220) : QColor(170, 180, 200, 170));
				glVertex3d(a.x - ox, a.y - oy, 0.0);
				glVertex3d(a.x - ox, a.y - oy, place ? 2.0 : 1.2);
			}
			glEnd();
		}
		if (L.objects)
			for (const MovingObject &o : s->objects)
				draw_box(o.x, o.y, 0.0, o.theta, o.length, o.width, std::max(0.3, o.height), o.length / 2,
						QColor(245, 166, 35, 70), pal::orange);
		if (L.gps && s->have_gps)
		{
			glLineWidth(2.0f);
			gl_color(pal::teal);
			glBegin(GL_LINES);
			double gx = s->gps.x() - ox, gy = s->gps.y() - oy;
			glVertex3d(gx - 0.6, gy, 0.2); glVertex3d(gx + 0.6, gy, 0.2);
			glVertex3d(gx, gy - 0.6, 0.2); glVertex3d(gx, gy + 0.6, 0.2);
			glVertex3d(gx, gy, 0.0); glVertex3d(gx, gy, 1.0);
			glEnd();
		}
		if (L.waypoints)
		{
			glPointSize(4.0f);
			gl_color(pal::green);
			glBegin(GL_POINTS);
			for (const QPointF &p : s->path)
				glVertex3d(p.x() - ox, p.y() - oy, 0.12);
			glEnd();
		}
		if (L.globalpos_trail && s->trail.size() >= 2)
		{
			glLineWidth(2.0f);
			glColor4f(1.0f, 0.85f, 0.3f, 0.8f);
			glBegin(GL_LINE_STRIP);
			for (const QPointF &p : s->trail)
				glVertex3d(p.x() - ox, p.y() - oy, 0.11);
			glEnd();
		}
		if (L.gps_axis && s->have_gps)
			draw_axes(s->gps.x(), s->gps.y(), 0.3, r.theta, 0, 0, 0.8);
		if (L.fused && s->have_fused)
			draw_flag(s->fused.x(), s->fused.y(), s->fused_theta, pal::accent, 0.8f);
		if (L.lidar && s->have_lidar)
			draw_lidar();
		if (s->have_final_goal)
		{
			double gx = s->final_goal.x(), gy = s->final_goal.y();
			glLineWidth(3.0f);
			gl_color(pal::red);
			glBegin(GL_LINES);
			glVertex3d(gx - ox, gy - oy, 0.0);
			glVertex3d(gx - ox, gy - oy, 2.2);
			glEnd();
			/* bandeira apontando para a orientacao do destino */
			double c = cos(s->final_goal_theta), sn = sin(s->final_goal_theta);
			glBegin(GL_TRIANGLES);
			glVertex3d(gx - ox, gy - oy, 2.2);
			glVertex3d(gx - ox, gy - oy, 1.5);
			glVertex3d(gx - ox + 0.9 * c, gy - oy + 0.9 * sn, 1.85);
			glEnd();
			draw_ring(gx, gy, 0.6, pal::red, 2.0);
		}
		if (s->have_highlight)
		{
			draw_ring(s->highlight.x(), s->highlight.y(), 1.0, pal::yellow, 3.0);
			draw_ring(s->highlight.x(), s->highlight.y(), 1.6, QColor(250, 204, 21, 120), 1.5);
		}
		if (r.have_pose)
		{
			draw_robot(r.x, r.y, r.theta, r.roll, r.pitch, 1.0f);
			if (L.collision && !r.collision.empty())
			{
				double ct = cos(r.theta), st = sin(r.theta);
				for (const QVector3D &cc : r.collision)
					draw_ring(r.x + ct * cc.x() - st * cc.y(), r.y + st * cc.x() + ct * cc.y(), cc.z(), QColor(255, 80, 80, 220), 1.5);
			}
			if (L.imu_axis)
				draw_axes(r.x, r.y, 0.6, r.theta, r.roll, r.pitch, 0.6);
			/* seta da frente */
			double c = cos(r.theta), sn = sin(r.theta), fx = r.x + c * (r.length - r.rear_overhang), fy = r.y + sn * (r.length - r.rear_overhang);
			glLineWidth(3.0f);
			glColor4f(1, 1, 1, 0.9f);
			glBegin(GL_LINES);
			glVertex3d(fx - ox, fy - oy, 0.25);
			glVertex3d(fx - ox + c * 0.8, fy - oy + sn * 0.8, 0.25);
			glEnd();
		}
	}
	if (have_origin)
	{
		draw_camera_billboards();
		draw_pick_ghost();
	}
	glDisable(GL_DEPTH_TEST);

	/* camera no canto: aparece quando ha' imagem recente da camera da esquerda */
	auto it = s->cameras.find(s->camera_left);
	bool show = s->layers.camera_corner && it != s->cameras.end() && !it->second.image.isNull();
	if (show)
	{
		const QImage &img = it->second.image;
		int w = camera_big ? std::min(width() / 2, 720) : std::min(width() / 4, 360);
		int h = w * img.height() / std::max(1, img.width()) + 22;
		camera_panel->setGeometry(12, 60, w, h);
		camera_panel->update();
	}
	camera_panel->setVisible(show);
	labels->update();
}

void
MapView3D::paint_camera_panel(QPainter &p, int w, int h)
{
	auto it = s->cameras.find(s->camera_left);
	if (it == s->cameras.end())
		return;
	p.setRenderHint(QPainter::SmoothPixmapTransform, true);
	p.setRenderHint(QPainter::Antialiasing, true);
	p.setPen(QPen(QColor(52, 60, 76), 1));
	p.setBrush(QColor(18, 22, 30, 230));
	p.drawRoundedRect(QRectF(0.5, 0.5, w - 1, h - 1), 7, 7);
	p.drawImage(QRect(4, 20, w - 8, h - 24), it->second.image);
	bool stale = now_seconds() - it->second.t > 2.0;
	p.setPen(stale ? pal::red : pal::dim);
	QFont f = p.font();
	f.setPointSizeF(7.5);
	f.setBold(true);
	p.setFont(f);
	p.drawText(QRect(8, 2, w - 16, 18), Qt::AlignLeft | Qt::AlignVCenter,
			QString("CAMERA %1%2").arg(s->camera_left).arg(stale ? "  (sem imagem)" : ""));
}

/* Paineis das cameras de pe' no mapa, a esquerda e a direita do robo, virados para quem olha. */
void
MapView3D::draw_camera_billboards()
{
	if (!s->layers.camera_map || !s->robot.have_pose)
		return;
	const int ids[2] = {s->camera_left, s->camera_right};
	double rx = -sin(yaw), ry = cos(yaw);	// direita da tela, no chao
	double scale = std::max(1.0, dist / 12.0);	// cresce um pouco com o zoom para continuar legivel
	for (int k = 0; k < 2; k++)
	{
		auto it = s->cameras.find(ids[k]);
		if (ids[k] < 0 || it == s->cameras.end() || it->second.image.isNull())
			continue;
		auto &tex = camera_tex[ids[k]];
		if (tex.second != it->second.version || !tex.first)
		{
			upload_texture(&tex.first, it->second.image);
			tex.second = it->second.version;
		}
		double w = 2.4 * scale, hgt = w * it->second.image.height() / std::max(1, it->second.image.width());
		double side = (k == 0 ? -1.0 : 1.0) * (1.4 * scale + w / 2);
		double cx = s->robot.x - ox + rx * side, cy = s->robot.y - oy + ry * side, z0 = 0.4 * scale;
		double ax = rx * w / 2, ay = ry * w / 2;
		/* "para cima" da tela: o painel fica de frente para quem olha, mesmo vendo de cima */
		double ux = -sin(pitch) * cos(yaw) * hgt, uy = -sin(pitch) * sin(yaw) * hgt, uz = cos(pitch) * hgt;
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, tex.first);
		glColor4f(1, 1, 1, 0.95f);
		glBegin(GL_QUADS);
		glTexCoord2f(0, 1); glVertex3d(cx - ax, cy - ay, z0);
		glTexCoord2f(1, 1); glVertex3d(cx + ax, cy + ay, z0);
		glTexCoord2f(1, 0); glVertex3d(cx + ax + ux, cy + ay + uy, z0 + uz);
		glTexCoord2f(0, 0); glVertex3d(cx - ax + ux, cy - ay + uy, z0 + uz);
		glEnd();
		glBindTexture(GL_TEXTURE_2D, 0);
		glDisable(GL_TEXTURE_2D);
		glLineWidth(2.0f);
		glColor4f(0.25f, 0.6f, 1.0f, 0.9f);
		glBegin(GL_LINE_LOOP);
		glVertex3d(cx - ax, cy - ay, z0);
		glVertex3d(cx + ax, cy + ay, z0);
		glVertex3d(cx + ax + ux, cy + ay + uy, z0 + uz);
		glVertex3d(cx - ax + ux, cy - ay + uy, z0 + uz);
		glEnd();
		/* haste ligando o painel ao robo */
		glColor4f(0.25f, 0.6f, 1.0f, 0.5f);
		glBegin(GL_LINES);
		glVertex3d(s->robot.x - ox, s->robot.y - oy, 0.3);
		glVertex3d(cx - (k == 0 ? -ax : ax), cy - (k == 0 ? -ay : ay), z0);
		glEnd();
	}
}

void
MapView3D::draw_robot(double x, double y, double theta, double roll, double pitch_angle, float alpha)
{
	const RobotState &r = s->robot;
	if (s->layers.car_model && !car.empty())
	{
		/* (x, y) do robo e' o eixo traseiro; o modelo esta' centrado no meio do carro */
		glPushMatrix();
		glTranslated(x - ox, y - oy, 0.0);
		glRotated(theta * 180.0 / M_PI, 0, 0, 1);
		glRotated(-pitch_angle * 180.0 / M_PI, 0, 1, 0);
		glRotated(roll * 180.0 / M_PI, 1, 0, 0);
		glTranslated(r.length / 2 - r.rear_overhang, 0.0, 0.0);
		car.draw(0.92f, 0.92f, 0.95f, alpha);
		glPopMatrix();
	}
	else
		draw_box(x, y, 0.0, theta, r.length, r.width, 0.45, r.rear_overhang,
				QColor(240, 82, 82, (int) (255 * alpha)), QColor(255, 255, 255, (int) (230 * alpha)), roll, pitch_angle);
}

void
MapView3D::draw_flag(double x, double y, double theta, const QColor &col, float alpha)
{
	glLineWidth(3.0f);
	glColor4f(col.redF(), col.greenF(), col.blueF(), alpha);
	glBegin(GL_LINES);
	glVertex3d(x - ox, y - oy, 0.0);
	glVertex3d(x - ox, y - oy, 2.2);
	glEnd();
	double c = cos(theta), sn = sin(theta);
	glBegin(GL_TRIANGLES);
	glVertex3d(x - ox, y - oy, 2.2);
	glVertex3d(x - ox, y - oy, 1.5);
	glVertex3d(x - ox + 0.9 * c, y - oy + 0.9 * sn, 1.85);
	glEnd();
	/* seta no chao para onde vai apontar */
	glBegin(GL_LINES);
	glVertex3d(x - ox, y - oy, 0.08);
	glVertex3d(x - ox + 1.2 * c, y - oy + 1.2 * sn, 0.08);
	glVertex3d(x - ox + 1.2 * c, y - oy + 1.2 * sn, 0.08);
	glVertex3d(x - ox + 1.2 * c - 0.35 * cos(theta - 0.5), y - oy + 1.2 * sn - 0.35 * sin(theta - 0.5), 0.08);
	glVertex3d(x - ox + 1.2 * c, y - oy + 1.2 * sn, 0.08);
	glVertex3d(x - ox + 1.2 * c - 0.35 * cos(theta + 0.5), y - oy + 1.2 * sn - 0.35 * sin(theta + 0.5), 0.08);
	glEnd();
	draw_ring(x, y, 0.6, QColor(col.red(), col.green(), col.blue(), (int) (255 * alpha)), 2.0);
}

void
MapView3D::draw_axes(double x, double y, double z, double theta, double roll, double pitch_angle, double size)
{
	glPushMatrix();
	glTranslated(x - ox, y - oy, z);
	glRotated(theta * 180.0 / M_PI, 0, 0, 1);
	glRotated(-pitch_angle * 180.0 / M_PI, 0, 1, 0);
	glRotated(roll * 180.0 / M_PI, 1, 0, 0);
	glLineWidth(3.0f);
	glBegin(GL_LINES);
	glColor4f(1, 0.25f, 0.25f, 1); glVertex3d(0, 0, 0); glVertex3d(size, 0, 0);
	glColor4f(0.3f, 1, 0.3f, 1);   glVertex3d(0, 0, 0); glVertex3d(0, size, 0);
	glColor4f(0.3f, 0.5f, 1, 1);   glVertex3d(0, 0, 0); glVertex3d(0, 0, size);
	glEnd();
	glPopMatrix();
}

void
MapView3D::draw_lidar()
{
	/* pontos no referencial do carro, na pose em que foram medidos; cor pela altura ou pela intensidade */
	size_t n = s->lidar_xyz.size() / 3;
	if (n == 0)
		return;
	static std::vector<float> colors;
	colors.resize(n * 3);
	for (size_t i = 0; i < n; i++)
	{
		float r, g, b;
		if (s->layers.lidar_intensity)
		{
			float f = s->lidar_intensity[i] / 255.0f;
			f = std::min(1.0f, f * 2.5f);
			r = g = b = 0.25f + 0.75f * f;
		}
		else
		{
			/* altura: azul (chao) -> verde -> amarelo -> vermelho (2 m) */
			float f = std::max(0.0f, std::min(1.0f, (s->lidar_xyz[i * 3 + 2] + 0.3f) / 2.3f));
			r = std::min(1.0f, 2.0f * f);
			g = f < 0.5f ? 0.4f + f : 1.0f - (f - 0.5f) * 1.6f;
			b = std::max(0.0f, 1.0f - 2.5f * f);
		}
		colors[i * 3] = r;
		colors[i * 3 + 1] = g;
		colors[i * 3 + 2] = b;
	}
	glPushMatrix();
	glTranslated(s->lidar_x - ox, s->lidar_y - oy, 0.0);
	glRotated(s->lidar_theta * 180.0 / M_PI, 0, 0, 1);
	glPointSize(2.0f);
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);
	glVertexPointer(3, GL_FLOAT, 0, s->lidar_xyz.data());
	glColorPointer(3, GL_FLOAT, 0, colors.data());
	glDrawArrays(GL_POINTS, 0, (GLsizei) n);
	glDisableClientState(GL_COLOR_ARRAY);
	glDisableClientState(GL_VERTEX_ARRAY);
	glPopMatrix();
}

/* Fantasma no mouse: o robo (pose inicial) ou a bandeira (destino) onde vai cair o clique; arrastando, gira. */
void
MapView3D::draw_pick_ghost()
{
	if (pick_mode == 0 || (!picking && !have_hover))
		return;
	double x = picking ? pick_x : hover_x, y = picking ? pick_y : hover_y;
	double theta = picking ? pick_theta : s->robot.theta;
	if (picking)
	{
		glLineWidth(2.5f);
		gl_color(pal::purple);
		glBegin(GL_LINES);
		glVertex3d(x - ox, y - oy, 0.1);
		glVertex3d(x - ox + 2.0 * cos(theta), y - oy + 2.0 * sin(theta), 0.1);
		glEnd();
	}
	if (pick_mode == 1)
	{
		draw_robot(x, y, theta, 0, 0, 0.55f);
		draw_ring(x, y, std::max(0.5, s->robot.length * 0.8), pal::accent, 2.0);
	}
	else
		draw_flag(x, y, theta, pick_mode == 2 ? pal::red : pal::purple, 0.75f);
}

void
MapView3D::set_pick_mode(int mode)
{
	pick_mode = mode;
	picking = false;
	setCursor(mode ? Qt::CrossCursor : Qt::ArrowCursor);
	update();
}

void
MapView3D::leaveEvent(QEvent *)
{
	have_hover = false;
	update();
}

void
MapView3D::paint_labels(QPainter &painter)
{
	const Layers &L = s->layers;
	painter.setRenderHint(QPainter::Antialiasing, true);
	if (L.annotations && L.labels && dist < 250)
	{
		QFont f = font();
		f.setPointSizeF(8);
		painter.setFont(f);
		for (const Annotation &a : s->annotations)
		{
			bool place = a.text.startsWith("RDDF_PLACE");
			QPointF sp;
			if (!project(a.x, a.y, place ? 2.1 : 1.3, &sp) || !rect().contains(sp.toPoint()))
				continue;
			QString t = a.text;
			if (t.startsWith("RDDF_ANNOTATION_TYPE_"))
				t = t.mid(21);
			else if (t.startsWith("RDDF_"))
				t = t.mid(5);
			QRectF tr = painter.fontMetrics().boundingRect(t);
			QRectF box(sp.x() - tr.width() / 2 - 5, sp.y() - tr.height() - 4, tr.width() + 10, tr.height() + 4);
			painter.setPen(Qt::NoPen);
			painter.setBrush(QColor(0, 0, 0, 140));
			painter.drawRoundedRect(box, 4, 4);
			painter.setPen(place ? QColor(250, 220, 120) : pal::text);
			painter.drawText(box, Qt::AlignCenter, t);
		}
	}
	if (L.map && s->map_choice != "offline" && L.map_overlay && s->layer_colored)
	{
		/* legenda do encaixe: cinza = offline, laranja = o que o lidar esta' vendo agora (mapper) */
		QFont lf = font();
		lf.setPointSizeF(8.5);
		painter.setFont(lf);
		QRectF box(12, height() - 62, 300, 24);
		painter.setPen(Qt::NoPen);
		painter.setBrush(QColor(18, 22, 30, 200));
		painter.drawRoundedRect(box, 6, 6);
		painter.setBrush(QColor(200, 205, 215));
		painter.drawRect(QRectF(22, height() - 55, 10, 10));
		painter.setPen(pal::text);
		painter.drawText(QRectF(38, height() - 62, 90, 24), Qt::AlignLeft | Qt::AlignVCenter, "mapa offline");
		painter.setPen(Qt::NoPen);
		painter.setBrush(QColor(90, 175, 255));
		painter.drawRect(QRectF(132, height() - 55, 10, 10));
		painter.setPen(pal::text);
		painter.drawText(QRectF(148, height() - 62, 160, 24), Qt::AlignLeft | Qt::AlignVCenter, "mapper (lidar ao vivo)");
	}
	painter.setPen(pal::dim);
	painter.drawText(QRectF(12, height() - 30, width() - 24, 22), Qt::AlignLeft | Qt::AlignVCenter,
			pick_mode ? "clique onde o robo fica e ARRASTE para onde ele aponta   |   botao direito: mover   |   roda: zoom" : "arrastar: girar   |   botao direito/meio: mover   |   roda: zoom");
	if (s->map_image.isNull())
		painter.drawText(rect(), Qt::AlignCenter, "Aguardando o mapa do robo...");
}

/* ------------------------------------------------------------------------------------------ */
/* Mouse                                                                                      */
/* ------------------------------------------------------------------------------------------ */

void
MapView3D::mousePressEvent(QMouseEvent *ev)
{
	if (pick_mode && ev->button() == Qt::LeftButton && ground_point(ev->pos(), &pick_x, &pick_y))
	{
		picking = true;
		pick_theta = s->robot.theta;
		setCursor(Qt::ClosedHandCursor);
		update();
		return;
	}
	press_pos = ev->pos();
	last_drag = ev->pos();
	drag_button = ev->button();
	dragging = false;
}

void
MapView3D::mouseMoveEvent(QMouseEvent *ev)
{
	if (picking)
	{
		double wx, wy;
		if (ground_point(ev->pos(), &wx, &wy) && hypot(wx - pick_x, wy - pick_y) > 0.15)
			pick_theta = atan2(wy - pick_y, wx - pick_x);
		update();
		return;
	}
	if (pick_mode && drag_button == Qt::NoButton)
	{
		have_hover = ground_point(ev->pos(), &hover_x, &hover_y);
		update();
		return;
	}
	if (drag_button == Qt::NoButton)
		return;
	if ((ev->pos() - press_pos).manhattanLength() > 4)
		dragging = true;
	if (!dragging)
		return;
	QPointF d = ev->pos() - last_drag;
	last_drag = ev->pos();
	if (drag_button == Qt::LeftButton && !(ev->modifiers() & Qt::ShiftModifier))
	{
		yaw -= d.x() * 0.008;
		pitch = std::max(0.08, std::min(1.56, pitch + d.y() * 0.006));
		if (in_top_view)
		{
			in_top_view = false;
			eye->setChecked(false);
		}
	}
	else
	{
		/* move o alvo no plano do chao, no referencial da camera */
		double k = dist * 0.0018;
		double fx = -cos(yaw), fy = -sin(yaw);	// para frente (da camera para o alvo)
		double rx = fy, ry = -fx;				// direita = frente x z
		tx += (-d.x() * rx + d.y() * fx) * k;
		ty += (-d.x() * ry + d.y() * fy) * k;
	}
	update();
}

void
MapView3D::mouseReleaseEvent(QMouseEvent *ev)
{
	if (picking && ev->button() == Qt::LeftButton)
	{
		picking = false;
		have_hover = false;
		setCursor(pick_mode ? Qt::CrossCursor : Qt::ArrowCursor);
		if (on_pick)
			on_pick(pick_x, pick_y, pick_theta);
		update();
		return;
	}
	if (!dragging && ev->button() == Qt::LeftButton && on_click)
	{
		double wx, wy;
		if (ground_point(ev->pos(), &wx, &wy))
			on_click(wx, wy);
	}
	drag_button = Qt::NoButton;
	dragging = false;
}

void
MapView3D::wheelEvent(QWheelEvent *ev)
{
	dist = std::max(2.0, std::min(5000.0, dist * pow(0.9985, ev->angleDelta().y())));
	update();
}
