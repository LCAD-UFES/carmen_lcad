/* Painel do carro (estilo do viewer_3D_dash). So' le o RobotState. */
#ifndef CARMEN_APP_DASHBOARD_H
#define CARMEN_APP_DASHBOARD_H

#include "scene.h"

#include <QWidget>

class QPainter;

class Dashboard : public QWidget
{
public:
	explicit Dashboard(RobotState *state);

protected:
	void paintEvent(QPaintEvent *) override;

private:
	QRectF card(QPainter &p, double y, double h, const QString &title);
	void text(QPainter &p, const QPointF &pos, const QString &t, double size, const QColor &c, bool bold = false,
			int align = Qt::AlignLeft);
	void bar(QPainter &p, const QRectF &r, double fraction, const QColor &c);
	void bipolar(QPainter &p, const QRectF &r, double value, double command, double full);
	void chip(QPainter &p, const QRectF &r, const QString &label, bool on, const QColor &c);
	void age_dot(QPainter &p, const QPointF &c, double t);
	void kv(QPainter &p, double y, const QString &k, const QString &v, double t = -1);

	RobotState *s;
};

#endif
