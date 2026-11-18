#include <algorithm>
#include "map_image.h"
#include "scene.h"

#include <QDateTime>
#include <QJsonArray>
#include <zlib.h>
#include <vector>

double
now_seconds()
{
	return QDateTime::currentMSecsSinceEpoch() / 1000.0;
}

QPolygonF
json_to_polygon(const QJsonArray &pts)
{
	QPolygonF poly;
	for (const QJsonValue &p : pts)
	{
		QJsonArray a = p.toArray();
		if (a.size() >= 2)
			poly << QPointF(a[0].toDouble(), a[1].toDouble());
	}
	return poly;
}

QVector<QRgb>
map_color_table()
{
	QVector<QRgb> lut(256);
	for (int v = 0; v < 255; v++)
	{
		double f = v / 254.0;
		lut[v] = qRgb(48 + (int) (f * 184), 55 + (int) (f * 180), 68 + (int) (f * 172));
	}
	lut[255] = qRgb(27, 32, 42);
	return lut;
}

bool
decode_map(const QJsonObject &meta, const QByteArray &blob, QImage *out, QRectF *rect)
{
	int xs = meta["x_size"].toInt(), ys = meta["y_size"].toInt();
	if (xs <= 0 || ys <= 0)
		return false;
	std::vector<unsigned char> grid((size_t) xs * ys);
	uLongf size = grid.size();
	if (uncompress(grid.data(), &size, (const Bytef *) blob.constData(), blob.size()) != Z_OK || size != grid.size())
		return false;

	/* O carmen guarda complete_map[x * y_size + y]: coluna = x, linha = y. */
	static const QVector<QRgb> lut = map_color_table();
	QImage img(xs, ys, QImage::Format_Indexed8);
	img.setColorTable(lut);
	for (int j = 0; j < ys; j++)
	{
		uchar *row = img.scanLine(j);
		for (int i = 0; i < xs; i++)
			row[i] = grid[(size_t) i * ys + j];
	}
	*out = img.convertToFormat(QImage::Format_ARGB32);
	double res = meta["resolution"].toDouble();
	*rect = QRectF(meta["x_origin"].toDouble(), meta["y_origin"].toDouble(), xs * res, ys * res);
	return true;
}

bool
decode_map_overlay(const QJsonObject &meta, const QByteArray &blob, const QColor &color, QImage *out, QRectF *rect)
{
	int xs = meta["x_size"].toInt(), ys = meta["y_size"].toInt();
	if (xs <= 0 || ys <= 0)
		return false;
	std::vector<unsigned char> grid((size_t) xs * ys);
	uLongf size = grid.size();
	if (uncompress(grid.data(), &size, (const Bytef *) blob.constData(), blob.size()) != Z_OK || size != grid.size())
		return false;

	/* p < 0,5: livre (transparente); p de 0,5 a 1: opacidade de ~30% a ~70% (nao esconde o offline) */
	QVector<QRgb> lut(256, qRgba(0, 0, 0, 0));
	for (int v = 128; v < 255; v++)
	{
		int alpha = 75 + (v - 128) * 105 / 126;
		lut[v] = qRgba(color.red(), color.green(), color.blue(), std::min(255, alpha));
	}
	QImage img(xs, ys, QImage::Format_Indexed8);
	img.setColorTable(lut);
	for (int j = 0; j < ys; j++)
	{
		uchar *row = img.scanLine(j);
		for (int i = 0; i < xs; i++)
			row[i] = grid[(size_t) i * ys + j];
	}
	*out = img.convertToFormat(QImage::Format_ARGB32);
	double res = meta["resolution"].toDouble();
	*rect = QRectF(meta["x_origin"].toDouble(), meta["y_origin"].toDouble(), xs * res, ys * res);
	return true;
}
