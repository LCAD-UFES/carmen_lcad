/*
 * Modelo 3D do carro: o mesmo .obj do viewer_3D (parametros carmodel_* do carmen.ini), enviado pelo
 * carmen_server. Aplica as mesmas transformacoes do draw_car.cpp do viewer_3D:
 *   glmUnitize (centraliza e escala para caber em -1..1) + glmScaleXYZ(size / 2) + rotacao de 90 graus
 *   em X (o .obj vem com Y para cima, como o Blender exporta).
 * Depois apoia o modelo no chao (z minimo = 0) com o centro no meio do carro.
 */
#ifndef CARMEN_APP_CAR_MODEL_H
#define CARMEN_APP_CAR_MODEL_H

#include <QByteArray>
#include <vector>

class CarModel
{
public:
	/* obj_text: conteudo do .obj; size: carmodel_size_x/y/z; yaw: carmodel_yaw (rad) */
	bool load(const QByteArray &obj_text, const double size[3], double yaw);
	bool empty() const { return positions.empty(); }
	int triangles() const { return (int) positions.size() / 9; }

	/* desenha com o centro do carro na origem e a frente em +x (OpenGL classico, com luz) */
	void draw(float r, float g, float b, float a = 1.0f) const;

private:
	std::vector<float> positions, normals;
};

#endif
