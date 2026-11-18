#include "car_model.h"

#include <GL/gl.h>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>

bool
CarModel::load(const QByteArray &obj_text, const double size[3], double yaw)
{
	std::vector<float> v;			// vertices do arquivo (x, y, z)
	std::vector<int> tri;			// indices dos triangulos
	const char *p = obj_text.constData(), *end = p + obj_text.size();

	while (p < end)
	{
		const char *eol = (const char *) memchr(p, '\n', end - p);
		if (!eol)
			eol = end;
		if (p[0] == 'v' && p[1] == ' ')
		{
			char *q;
			float x = strtof(p + 2, &q), y = strtof(q, &q), z = strtof(q, &q);
			v.push_back(x);
			v.push_back(y);
			v.push_back(z);
		}
		else if (p[0] == 'f' && p[1] == ' ')
		{
			/* f a/b/c d/e/f ...: so' o indice do vertice; poligonos viram leque de triangulos */
			int idx[64], n = 0;
			const char *q = p + 2;
			while (q < eol && n < 64)
			{
				while (q < eol && *q == ' ')
					q++;
				if (q >= eol)
					break;
				char *after;
				long i = strtol(q, &after, 10);
				if (after == q)
					break;
				idx[n++] = (i < 0) ? (int) (v.size() / 3 + i) : (int) (i - 1);
				q = after;
				while (q < eol && *q != ' ')
					q++;
			}
			for (int k = 1; k + 1 < n; k++)
			{
				tri.push_back(idx[0]);
				tri.push_back(idx[k]);
				tri.push_back(idx[k + 1]);
			}
		}
		p = eol + 1;
	}
	int nv = (int) v.size() / 3;
	if (nv == 0 || tri.empty())
		return false;

	/* glmUnitize + glmScaleXYZ(size/2): (v - centro) * size / maior_dimensao */
	float mn[3] = {1e30f, 1e30f, 1e30f}, mx[3] = {-1e30f, -1e30f, -1e30f};
	for (int i = 0; i < nv; i++)
		for (int a = 0; a < 3; a++)
		{
			mn[a] = std::min(mn[a], v[i * 3 + a]);
			mx[a] = std::max(mx[a], v[i * 3 + a]);
		}
	float largest = std::max(mx[0] - mn[0], std::max(mx[1] - mn[1], mx[2] - mn[2]));
	if (largest <= 0)
		return false;
	double c = cos(yaw), sn = sin(yaw);
	float zmin = 1e30f;
	std::vector<float> w((size_t) nv * 3);
	for (int i = 0; i < nv; i++)
	{
		double x = (v[i * 3] - (mn[0] + mx[0]) / 2) / largest * size[0];
		double y = (v[i * 3 + 1] - (mn[1] + mx[1]) / 2) / largest * size[1];
		double z = (v[i * 3 + 2] - (mn[2] + mx[2]) / 2) / largest * size[2];
		/* rotacao de 90 graus em X (Y para cima -> Z para cima): (x, y, z) -> (x, -z, y) */
		double rx = x, ry = -z, rz = y;
		/* carmodel_yaw */
		w[i * 3] = (float) (c * rx - sn * ry);
		w[i * 3 + 1] = (float) (sn * rx + c * ry);
		w[i * 3 + 2] = (float) rz;
		zmin = std::min(zmin, w[i * 3 + 2]);
	}
	for (int i = 0; i < nv; i++)
		w[i * 3 + 2] -= zmin;	// apoiado no chao

	/* triangulos soltos com normal por face (sombreamento chapado, sem precisar das normais do arquivo) */
	positions.clear();
	normals.clear();
	positions.reserve(tri.size() * 3);
	normals.reserve(tri.size() * 3);
	for (size_t t = 0; t + 2 < tri.size(); t += 3)
	{
		int a = tri[t], b = tri[t + 1], d = tri[t + 2];
		if (a < 0 || b < 0 || d < 0 || a >= nv || b >= nv || d >= nv)
			continue;
		float ux = w[b * 3] - w[a * 3], uy = w[b * 3 + 1] - w[a * 3 + 1], uz = w[b * 3 + 2] - w[a * 3 + 2];
		float vx = w[d * 3] - w[a * 3], vy = w[d * 3 + 1] - w[a * 3 + 1], vz = w[d * 3 + 2] - w[a * 3 + 2];
		float nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
		float len = sqrtf(nx * nx + ny * ny + nz * nz);
		if (len <= 0)
			continue;
		nx /= len;
		ny /= len;
		nz /= len;
		for (int k : {a, b, d})
		{
			positions.push_back(w[k * 3]);
			positions.push_back(w[k * 3 + 1]);
			positions.push_back(w[k * 3 + 2]);
			normals.push_back(nx);
			normals.push_back(ny);
			normals.push_back(nz);
		}
	}
	return !positions.empty();
}

void
CarModel::draw(float r, float g, float b, float a) const
{
	if (positions.empty())
		return;
	/* luz direcional simples vinda de cima/frente, nas duas faces (o .obj nem sempre tem a ordem certa) */
	GLfloat light_dir[4] = {0.3f, 0.5f, 1.0f, 0.0f};
	GLfloat ambient[4] = {0.35f, 0.35f, 0.38f, 1.0f};
	GLfloat diffuse[4] = {0.75f, 0.75f, 0.75f, 1.0f};
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
	glLightfv(GL_LIGHT0, GL_POSITION, light_dir);
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glEnable(GL_NORMALIZE);
	glColor4f(r, g, b, a);

	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);
	glVertexPointer(3, GL_FLOAT, 0, positions.data());
	glNormalPointer(GL_FLOAT, 0, normals.data());
	glDrawArrays(GL_TRIANGLES, 0, (GLsizei) (positions.size() / 3));
	glDisableClientState(GL_NORMAL_ARRAY);
	glDisableClientState(GL_VERTEX_ARRAY);

	glDisable(GL_COLOR_MATERIAL);
	glDisable(GL_NORMALIZE);
	glDisable(GL_LIGHT0);
	glDisable(GL_LIGHTING);
}
