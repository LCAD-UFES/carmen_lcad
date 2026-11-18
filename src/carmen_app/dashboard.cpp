#include "dashboard.h"
#include "theme.h"

#include <QPainter>
#include <QPainterPath>
#include <algorithm>
#include <cmath>

Dashboard::Dashboard(RobotState *state) : s(state)
{
	setMinimumWidth(330);
	setMinimumHeight(750);
}

QRectF
Dashboard::card(QPainter &p, double y, double h, const QString &title)
{
	QRectF r(10, y, width() - 20, h);
	p.setPen(Qt::NoPen);
	p.setBrush(pal::card);
	p.drawRoundedRect(r, 10, 10);
	p.setPen(pal::dim);
	QFont f = font();
	f.setPointSizeF(7.5);
	f.setBold(true);
	f.setLetterSpacing(QFont::AbsoluteSpacing, 1.2);
	p.setFont(f);
	p.drawText(QPointF(r.left() + 14, r.top() + 20), title.toUpper());
	return r;
}

void
Dashboard::text(QPainter &p, const QPointF &pos, const QString &t, double size, const QColor &c, bool bold, int align)
{
	QFont f = font();
	f.setPointSizeF(size);
	f.setBold(bold);
	f.setLetterSpacing(QFont::AbsoluteSpacing, 0);
	p.setFont(f);
	p.setPen(c);
	double h = size * 2.2 + 10;	// caixa da altura da fonte: o texto e' alinhado pela base
	QRectF r(pos.x() - (align == Qt::AlignRight ? 400 : (align == Qt::AlignHCenter ? 200 : 0)), pos.y() - h + 10, 400, h);
	p.drawText(r, align | Qt::AlignBottom, t);
}

void
Dashboard::bar(QPainter &p, const QRectF &r, double fraction, const QColor &c)
{
	p.setPen(Qt::NoPen);
	p.setBrush(pal::line);
	p.drawRoundedRect(r, r.height() / 2, r.height() / 2);
	fraction = std::max(0.0, std::min(1.0, fraction));
	if (fraction > 0)
	{
		p.setBrush(c);
		p.drawRoundedRect(QRectF(r.left(), r.top(), r.width() * fraction, r.height()), r.height() / 2, r.height() / 2);
	}
}

void
Dashboard::bipolar(QPainter &p, const QRectF &r, double value, double command, double full)
{
	p.setPen(Qt::NoPen);
	p.setBrush(pal::line);
	p.drawRoundedRect(r, r.height() / 2, r.height() / 2);
	double cx = r.center().x();
	double v = std::max(-1.0, std::min(1.0, value / full));
	p.setBrush(pal::accent);
	/* esquerda positiva (convencao do carmen: phi > 0 vira para a esquerda) */
	QRectF fill = (v >= 0) ? QRectF(cx - v * r.width() / 2, r.top(), v * r.width() / 2, r.height())
						   : QRectF(cx, r.top(), -v * r.width() / 2, r.height());
	p.drawRoundedRect(fill, r.height() / 2, r.height() / 2);
	p.setPen(QPen(pal::dim, 1));
	p.drawLine(QPointF(cx, r.top() - 3), QPointF(cx, r.bottom() + 3));
	double c = std::max(-1.0, std::min(1.0, command / full));
	double mx = cx - c * r.width() / 2;
	p.setPen(QPen(pal::orange, 2.5));
	p.drawLine(QPointF(mx, r.top() - 5), QPointF(mx, r.bottom() + 5));
}

void
Dashboard::chip(QPainter &p, const QRectF &r, const QString &label, bool on, const QColor &c)
{
	p.setPen(QPen(on ? c : pal::line, 1.2));
	p.setBrush(on ? QColor(c.red(), c.green(), c.blue(), 50) : Qt::NoBrush);
	p.drawRoundedRect(r, 6, 6);
	QFont f = font();
	f.setPointSizeF(7.5);
	f.setBold(true);
	p.setFont(f);
	p.setPen(on ? c : pal::dim);
	p.drawText(r, Qt::AlignCenter, label);
}

void
Dashboard::age_dot(QPainter &p, const QPointF &c, double t)
{
	double age = t > 0 ? now_seconds() - t : 1e9;
	QColor col = age < 1.0 ? pal::green : (age < 5.0 ? pal::orange : QColor(90, 98, 115));
	p.setPen(Qt::NoPen);
	p.setBrush(col);
	p.drawEllipse(c, 4, 4);
}

void
Dashboard::kv(QPainter &p, double y, const QString &k, const QString &v, double t)
{
	text(p, QPointF(24, y), k, 8.5, pal::dim);
	text(p, QPointF(width() - 24, y), v, 8.5, pal::text, true, Qt::AlignRight);
	if (t >= 0)
		age_dot(p, QPointF(width() - 22 - p.fontMetrics().horizontalAdvance(v) - 14, y - 6), t);
}

void
Dashboard::paintEvent(QPaintEvent *)
{
	QPainter p(this);
	p.setRenderHint(QPainter::Antialiasing, true);
	p.fillRect(rect(), pal::panel);
	double y = 10;
	const QChar deg_sign(0x00B0), sq(0x00B2);

	/* --- velocidade --- */
	QRectF r = card(p, y, 150, "Velocidade");
	text(p, QPointF(r.left() + 14, r.top() + 82), QString::number(fabs(s->v * 3.6), 'f', 1), 34, pal::text, true);
	text(p, QPointF(r.left() + 150, r.top() + 78), "km/h", 10, pal::dim);
	text(p, QPointF(r.right() - 14, r.top() + 52), QString("%1 m/s").arg(s->v, 0, 'f', 2), 10, pal::text, false, Qt::AlignRight);
	text(p, QPointF(r.right() - 14, r.top() + 72), QString("cmd %1 m/s").arg(s->v_cmd, 0, 'f', 2), 9, pal::orange, false, Qt::AlignRight);
	bar(p, QRectF(r.left() + 14, r.top() + 92, r.width() - 28, 6), fabs(s->v) / std::max(0.1, s->max_v), s->v < 0 ? pal::orange : pal::accent);
	QRectF sp(r.left() + 14, r.top() + 106, r.width() - 28, 32);
	if (s->speed_history.size() > 1)
	{
		double vmax = std::max(0.2, s->max_v);
		QPainterPath path;
		int n = (int) s->speed_history.size();
		for (int i = 0; i < n; i++)
		{
			double px = sp.left() + sp.width() * (i + 240 - n) / 239.0;
			double py = sp.bottom() - sp.height() * std::min(1.0, fabs(s->speed_history[i]) / vmax);
			if (i == 0)
				path.moveTo(px, py);
			else
				path.lineTo(px, py);
		}
		p.setPen(QPen(pal::accent, 1.5));
		p.setBrush(Qt::NoBrush);
		p.drawPath(path);
	}
	y += 160;

	/* --- direcao --- */
	r = card(p, y, 92, "Direcao");
	text(p, QPointF(r.left() + 14, r.top() + 50), QString("%1%2").arg(s->phi * 180.0 / M_PI, 0, 'f', 1).arg(deg_sign), 16, pal::text, true);
	text(p, QPointF(r.right() - 14, r.top() + 46), QString("cmd %1%2").arg(s->phi_cmd * 180.0 / M_PI, 0, 'f', 1).arg(deg_sign), 9, pal::orange, false, Qt::AlignRight);
	bipolar(p, QRectF(r.left() + 14, r.top() + 64, r.width() - 28, 8), s->phi, s->phi_cmd, std::max(0.05, s->max_phi));
	text(p, QPointF(r.left() + 14, r.top() + 88), "E", 7, pal::dim);
	text(p, QPointF(r.right() - 14, r.top() + 88), "D", 7, pal::dim, false, Qt::AlignRight);
	y += 102;

	/* --- marcha + aceleracoes --- */
	r = card(p, y, 104, "Marcha e aceleracao");
	QString gear = s->gear.isEmpty() ? s->gear_estimated : s->gear;
	const char *gears[] = {"P", "R", "N", "D", "L"};
	double gw = 30, gx = r.left() + 14;
	for (int i = 0; i < 5; i++)
		chip(p, QRectF(gx + i * (gw + 6), r.top() + 32, gw, 28), gears[i], gear == gears[i], i == 1 ? pal::orange : pal::accent);
	if (s->gear.isEmpty())
		text(p, QPointF(gx, r.top() + 78), "(estimada pelo movimento)", 7.5, pal::dim);
	text(p, QPointF(r.right() - 14, r.top() + 46), QString("long %1 m/s%2").arg(s->a_long, 0, 'f', 2).arg(sq), 9, pal::text, false, Qt::AlignRight);
	text(p, QPointF(r.right() - 14, r.top() + 64), QString("lat %1 m/s%2").arg(s->a_lat, 0, 'f', 2).arg(sq), 9, pal::text, false, Qt::AlignRight);
	text(p, QPointF(r.left() + 14, r.top() + 96), "acel", 7.5, pal::dim);
	bar(p, QRectF(r.left() + 46, r.top() + 88, r.width() / 2 - 60, 5), s->throttle / 100.0, pal::green);
	text(p, QPointF(r.center().x() + 4, r.top() + 96), "freio", 7.5, pal::dim);
	bar(p, QRectF(r.center().x() + 40, r.top() + 88, r.width() / 2 - 54, 5), s->brake / 100.0, pal::red);
	y += 114;

	/* --- indicadores --- */
	r = card(p, y, 92, "Indicadores");
	struct Chip { const char *label; bool on; QColor c; };
	const Chip chips[] = {
		{"AUTONOMO", s->autonomous != 0, pal::green}, {"MOTOR", s->engine != 0, pal::accent},
		{"FREIO MAO", s->parking_brake != 0, pal::orange}, {"FAROL", s->headlights != 0, pal::yellow},
		{"SETA E", s->turn_signal == 1 || s->turn_signal == 3, pal::orange}, {"SETA D", s->turn_signal == 2 || s->turn_signal == 3, pal::orange},
	};
	double cw = (r.width() - 28 - 12) / 3.0;
	for (int i = 0; i < 6; i++)
		chip(p, QRectF(r.left() + 14 + (i % 3) * (cw + 6), r.top() + 30 + (i / 3) * 30, cw, 24), chips[i].label, chips[i].on, chips[i].c);
	y += 102;

	/* --- estados --- */
	r = card(p, y, 246, "Estado");
	double ky = r.top() + 46;
	kv(p, ky, "Behavior", s->behavior, s->t_behavior); ky += 22;
	kv(p, ky, "Tarefa", s->task); ky += 22;
	kv(p, ky, "Rota", s->route_state, s->t_route); ky += 22;
	kv(p, ky, "Missao", s->mission_id.isEmpty() ? s->mission_state : s->mission_id + " (" + s->mission_state + ")"); ky += 22;
	kv(p, ky, "Ate' o goal", s->goal_distance >= 0 ? QString("%1 m").arg(s->goal_distance, 0, 'f', 1) : "-"); ky += 22;
	kv(p, ky, "Frenets", s->frenet_paths > 0 ? QString("%1 (escolhido %2)").arg(s->frenet_paths).arg(s->frenet_selected) : "-", s->t_frenet); ky += 22;
	kv(p, ky, "Localizacao", s->have_pose ? (s->converged ? "convergida" : "nao convergida") : "-", s->t_pose); ky += 22;
	kv(p, ky, "IMU", s->t_imu > 0 ? QString("rol %1%3  arf %2%3").arg(s->roll * 180.0 / M_PI, 0, 'f', 1).arg(s->pitch * 180.0 / M_PI, 0, 'f', 1).arg(deg_sign) : "-", s->t_imu); ky += 22;
	kv(p, ky, "GPS", s->gps_quality >= 0 ? QString("qual %1  sat %2  hdop %3").arg(s->gps_quality).arg(s->gps_sats).arg(s->gps_hdop, 0, 'f', 1) : "-", s->t_gps);
}
