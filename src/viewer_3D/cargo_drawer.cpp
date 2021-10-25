#include "cargo_drawer.h"


CargoDrawer *
createCargoDrawer(int argc, char **argv)
{
    CargoDrawer *cargoDrawer = (CargoDrawer *) malloc(sizeof (CargoDrawer));

    cargoDrawer->argc = argc;
    cargoDrawer->argv = argv;

    cargoDrawer->cargos_message_initialized = 0;

    for (int i = 0; i < MAX_NUM_MODELS; i++)
    {
    	cargoDrawer->cargoModels[i] = NULL;
    }

    return cargoDrawer;
}


void
add_cargos_message(CargoDrawer *cargoDrawer, carmen_cargo_cargos_message *cargos_msg)
{
	cargoDrawer->latest_message = *cargos_msg;
	cargoDrawer->cargos_message_initialized = 1;
}


void
draw_cargos(CargoDrawer *cargoDrawer, carmen_vector_3D_t offset)
{
	if (cargoDrawer->cargos_message_initialized)
	{
		for (int i = 0; i < cargoDrawer->latest_message.num_cargos; i++)
		{
			int cargo_type = cargoDrawer->latest_message.cargos[i].cargo_type;

//			printf("cargo_type = %d\n", cargo_type);

			if (cargo_type <= MAX_NUM_MODELS)
			{
				if (cargoDrawer->cargoModels[cargo_type - 1] == NULL)
				{
					char cargo_model_string[256];
					char *cargo_model_file = NULL;
					double cargo_size_x;

					sprintf(cargo_model_string, "%s%d", "semi_trailer_model", cargo_type);

					carmen_param_t param_list[] =
					{
						{cargo_model_string, "file_name", CARMEN_PARAM_STRING, &cargo_model_file, 0, NULL},
						{cargo_model_string, "size_x", CARMEN_PARAM_DOUBLE, &cargo_size_x, 0, NULL},
						{cargo_model_string, "x", CARMEN_PARAM_DOUBLE, &(cargoDrawer->positions[cargo_type - 1].x), 0, NULL},
						{cargo_model_string, "y", CARMEN_PARAM_DOUBLE, &(cargoDrawer->positions[cargo_type - 1].y), 0, NULL},
						{cargo_model_string, "z", CARMEN_PARAM_DOUBLE, &(cargoDrawer->positions[cargo_type - 1].z), 0, NULL},
					};

					int num_items = sizeof(param_list)/sizeof(param_list[0]);
					carmen_param_install_params(cargoDrawer->argc, cargoDrawer->argv, param_list, num_items);

//					printf("%s\n", cargo_model_string);
//					printf("%s\n", cargo_model_file);

					cargoDrawer->cargoModels[cargo_type - 1] = glmReadOBJ(cargo_model_file);
					glmUnitize(cargoDrawer->cargoModels[cargo_type - 1]);

					glmScale(cargoDrawer->cargoModels[cargo_type - 1], cargo_size_x / 2.0);
				}

				glPushMatrix();

					glTranslatef(cargoDrawer->latest_message.cargos[i].pose.x - offset.x, cargoDrawer->latest_message.cargos[i].pose.y - offset.y, -offset.z);
					glRotatef(carmen_radians_to_degrees(cargoDrawer->latest_message.cargos[i].pose.theta), 0.0f, 0.0f, 1.0f);
					glRotatef(0.0, 0.0f, 1.0f, 0.0f);
					glRotatef(0.0, 1.0f, 0.0f, 0.0f);

					glPushMatrix();

						glTranslatef(cargoDrawer->positions[cargo_type - 1].x, cargoDrawer->positions[cargo_type - 1].y, cargoDrawer->positions[cargo_type - 1].z);
						glRotatef(90.0, 1.0, 0.0, 0.0);
						glRotatef(0.0, 0.0, 1.0, 0.0);

						glColor3f(0.3,0.3,0.3);
						//glmDraw(carDrawer->carModel, GLM_SMOOTH | GLM_COLOR);
						glmDraw(cargoDrawer->cargoModels[cargo_type - 1], GLM_SMOOTH | GLM_COLOR | GLM_TEXTURE);

					glPopMatrix();

				glPopMatrix();
			}
		}
	}
}


void
destroyCargoDrawer(CargoDrawer *cargoDrawer)
{
    free(cargoDrawer);
}
