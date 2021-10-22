#ifndef CARGO_DRAWER_H
#define CARGO_DRAWER_H

#include <carmen/carmen.h>
#include <carmen/glm.h>
#include <carmen/cargo_interface.h>
#include <iostream>
#include <vector>
#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/glu.h>

#define MAX_NUM_MODELS 10

#ifdef __cplusplus
extern "C" {
#endif

struct CargoDrawer
{
	int argc;
	char **argv;

	GLMmodel *cargoModels[MAX_NUM_MODELS];
	carmen_vector_3D_t positions[MAX_NUM_MODELS];

	int cargos_message_initialized;
	carmen_cargo_cargos_message latest_message;
};

typedef struct CargoDrawer CargoDrawer;

CargoDrawer *createCargoDrawer(int argc, char** argv);
void add_cargos_message(CargoDrawer *cargoDrawer, carmen_cargo_cargos_message *cargos_msg);
void draw_cargos(CargoDrawer *cargoDrawer, carmen_vector_3D_t offset);
void destroyCargoDrawer(CargoDrawer *cargoDrawer);

#ifdef __cplusplus
}
#endif

#endif
