 /*********************************************************
	---   Skeleton Module Application ---
**********************************************************/

#include <carmen/carmen.h>
#include <carmen/mapper_interface.h>
#include <carmen/map_server_interface.h>
#include <carmen/dynamic_object_detector_interface.h>
#include <carmen/dynamic_object_detector_messages.h>
#include <carmen/localize_ackerman_interface.h>

#define LASER_RANGE 120.0


static int map_counter = 0;
static carmen_mapper_map_message *current_map_g = NULL;
static carmen_mapper_map_message *previous_map_g = NULL;
static carmen_dynamic_object_detector_clustered_objects_message *dynamic_objects_map_g = NULL;
static int car_global_x, car_global_y;

void
initialize_analyzed_maps()
{
	current_map_g = (carmen_mapper_map_message *) malloc (sizeof(carmen_mapper_map_message));
	current_map_g->complete_map = NULL;

	previous_map_g = (carmen_mapper_map_message *) malloc (sizeof(carmen_mapper_map_message));
	previous_map_g->complete_map = NULL;
}

void
initialize_dynamic_objects_map()
{
	dynamic_objects_map_g =
			(carmen_dynamic_object_detector_clustered_objects_message *) malloc (sizeof(carmen_dynamic_object_detector_clustered_objects_message));
	dynamic_objects_map_g->objects_map = NULL;
	dynamic_objects_map_g->objects_list = NULL;
	dynamic_objects_map_g->obj_count = 0;
}

void
initialize_dynamic_object_detector()
{
	initialize_analyzed_maps();
	initialize_dynamic_objects_map();

	map_counter = 0;
}

carmen_dynamic_object_detector_cartesian_points* to_global_pose(carmen_dynamic_object_detector_cartesian_points* p, carmen_map_config_t map_config)
{

	p->x = p->x * map_config.resolution + map_config.x_origin;
	p->y = p->y * map_config.resolution + map_config.y_origin;

	return p;
}

carmen_dynamic_object_detector_cartesian_points* to_map_pose(carmen_dynamic_object_detector_cartesian_points* p, carmen_map_config_t map_config)
{


	p->x = (p->x - map_config.x_origin) / map_config.resolution;
	p->y = (p->y - map_config.y_origin) / map_config.resolution;

	return p;
}


void
copy_current_map_to_previous_map(carmen_mapper_map_message* previous_map, carmen_mapper_map_message* current_map)
{
 
	if(previous_map != NULL && current_map != NULL)
	{
		
		for(int i = 0; i < current_map->size; i++)
			previous_map->complete_map[i] = current_map->complete_map[i];
		
		previous_map->size = current_map->size;
		
		if(current_map->config.map_name != NULL) 
		{
			free(previous_map->config.map_name);
			previous_map->config.map_name = (char*)malloc(sizeof(char)*(strlen(current_map->config.map_name)+1));
			strcpy(previous_map->config.map_name, current_map->config.map_name);
		}
		
		for(int i = 0;  i < 64; i++)
			previous_map->config.origin[i] = current_map->config.origin[i];

		previous_map->config.resolution = current_map->config.resolution;
		previous_map->config.x_origin = current_map->config.x_origin;
		previous_map->config.y_origin = current_map->config.y_origin;
		previous_map->config.x_size = current_map->config.x_size;
		previous_map->timestamp = current_map->timestamp;
		previous_map->host = current_map->host;
	}
	else
	{
		printf("dynamic_object_detector: cannot copy maps!\n");
	}
}

carmen_dynamic_object_detector_clustered_objects_message *
carmen_configure_dynamic_object_detector_clustered_message_with_grid_mapping_information(carmen_mapper_map_message* gridmap)
{
	if(dynamic_objects_map_g->objects_map == NULL)
		dynamic_objects_map_g->objects_map = (double*) malloc (gridmap->size * sizeof(double));

	dynamic_objects_map_g->map_size = gridmap->size;
	dynamic_objects_map_g->config = gridmap->config;
	dynamic_objects_map_g->timestamp = gridmap->timestamp;
	dynamic_objects_map_g->host = gridmap->host;

	return dynamic_objects_map_g;
}

double
carmen_dynamic_object_detector_analize_gridmap_cell(double current_map_cell, double previous_map_cell, double dynamic_map_cell)
{ 
  
	if(current_map_cell >= 0.7 && current_map_cell <= 1.0)			  // current cell is occupied
	{
		if(previous_map_cell >= 0.7 && previous_map_cell <= 1.0)		// previous cell is occupied
		{
			return 0.0; 	// static object
		}
		else if(previous_map_cell < 0.7 || dynamic_map_cell == 1.0)		//previous cell is free
		{
			return 1.0; 	// dynamic object
		}
		else
		{
			return -1.0;	// unknown object
		}
	}
	else 
		return -1.0;

}

void
carmen_dynamic_object_detector_publish_dynamic_object_map()
{
	
/*	
	carmen_grid_mapping_message dynamic_object_grid_map;
	dynamic_object_grid_map.complete_map = dynamic_objects_map_g->objects_map;
	dynamic_object_grid_map.size = dynamic_objects_map_g->map_size;
	dynamic_object_grid_map.config = dynamic_objects_map_g->config;
	dynamic_object_grid_map->timestamp = dynamic_objects_map_g->timestamp;
	dynamic_object_grid_map->host = dynamic_objects_map_g->host;
*/	
	carmen_map_t dynamic_object_map_temp;
	
	dynamic_object_map_temp.complete_map = dynamic_objects_map_g->objects_map;
	dynamic_object_map_temp.config.x_size = dynamic_objects_map_g->config.x_size;
	dynamic_object_map_temp.config.y_size = dynamic_objects_map_g->config.y_size;
	dynamic_object_map_temp.config.resolution = dynamic_objects_map_g->config.resolution;
	strcpy( dynamic_object_map_temp.config.origin, dynamic_objects_map_g->config.origin);
	dynamic_object_map_temp.config.map_name = dynamic_objects_map_g->config.map_name;
	dynamic_object_map_temp.config.x_origin = dynamic_objects_map_g->config.x_origin;
	dynamic_object_map_temp.config.y_origin = dynamic_objects_map_g->config.y_origin;
	
	/*
	carmen_grid_mapping_copy_map_from_message(&dynamic_object_map, &dynamic_object_grid_map);
	carmen_grid_mapping_publish_message(&dynamic_object_map, dynamic_object_grid_map.timestamp);
	*/
	carmen_map_server_publish_cost_map_message(&dynamic_object_map_temp, dynamic_objects_map_g->timestamp);
}

void carmen_dynamic_object_detector_add_point_to_dynamic_object(carmen_dynamic_object_detector_object* object, int x, int y)
{
  
	object->points = (carmen_dynamic_object_detector_cartesian_points*)realloc(object->points, sizeof(carmen_dynamic_object_detector_cartesian_points)*(object->points_count+1));
	//object->points[object->points_count] = (int*)malloc((sizeof(int)*2));
	object->points[object->points_count].x = x;
	object->points[object->points_count].y = y;
	object->points_count = object->points_count + 1;
	
	return;
}


void carmen_dynamic_object_detector_add_object_to_dynamic_map(int x, int y)
{
	carmen_dynamic_object_detector_object d_obj;
	
	d_obj.points = NULL;
	d_obj.points_count = 0;
	
	carmen_dynamic_object_detector_add_point_to_dynamic_object(&d_obj,x, y);
	
	dynamic_objects_map_g->objects_list = (carmen_dynamic_object_detector_object*)realloc(dynamic_objects_map_g->objects_list, sizeof(carmen_dynamic_object_detector_object)*(dynamic_objects_map_g->obj_count+1));
	dynamic_objects_map_g->objects_list[dynamic_objects_map_g->obj_count] = d_obj;
	
	dynamic_objects_map_g->obj_count++;
	
	return;
}

carmen_dynamic_object_detector_object* carmen_dynamic_object_detector_point_belongs_to_object(int x,int y) {


	for( int i = 0; i < dynamic_objects_map_g->obj_count; i++)
	{
		carmen_dynamic_object_detector_object* aux_obj = &dynamic_objects_map_g->objects_list[i];
		

		for( int j = 0; j < aux_obj->points_count; j++)
		{
			double dist = sqrt(pow((x - aux_obj->points[j].x),2) + pow((y - aux_obj->points[j].y),2));		 //passa por todos os pontos verificando se a distancia é menor que 0.3

			
			if(dist <= 3.0 && dist > 0.0)
			{
				return aux_obj;

			}
		}
	}
	
	return NULL;
}

void carmen_dynamic_object_detector_object_update()
{
	for( int i = 0; i < dynamic_objects_map_g->obj_count; i++)
	{
		carmen_dynamic_object_detector_object* aux_obj = &dynamic_objects_map_g->objects_list[i];
		//printf("obj: %d, %d\n", i, aux_obj->points_count);
		for( int j = 0; j< aux_obj->points_count; j++)
		{

			double dist = sqrt(pow((car_global_x - aux_obj->points[j].x),2) + pow((car_global_y - aux_obj->points[j].y),2));

			//printf("%f, %d, %d, %d, %d\n", dist, car_global_x, car_global_y, aux_obj->points[j].x, aux_obj->points[j].y);
			if(dist >= LASER_RANGE)
			{
				//printf("oi\n");
				for(int k = j; k < aux_obj->points_count-1; k++)
				{
					aux_obj->points[k] = aux_obj->points[k+1];
				}
				
				aux_obj->points = (carmen_dynamic_object_detector_cartesian_points*)realloc(aux_obj->points, sizeof(carmen_dynamic_object_detector_cartesian_points)*(aux_obj->points_count-1));
				aux_obj->points_count = aux_obj->points_count - 1;
				j--;
			}
		}

		if(aux_obj->points_count == 0)
		{
			//printf("oi2\n");
			for(int k = i; k < dynamic_objects_map_g->obj_count-1; k++)
			{
				dynamic_objects_map_g->objects_list[k] = dynamic_objects_map_g->objects_list[k+1];
			}			
			dynamic_objects_map_g->objects_list = (carmen_dynamic_object_detector_object*)realloc(dynamic_objects_map_g->objects_list, sizeof(carmen_dynamic_object_detector_object)*(dynamic_objects_map_g->obj_count-1));
			dynamic_objects_map_g->obj_count--;
			i--;
			printf("%d\n", dynamic_objects_map_g->obj_count);
		}
	}
	
	return;
}

void carmen_dynamic_object_detector_add_cell_to_dynamic_objects_list(int cell_number)
{
	carmen_dynamic_object_detector_cartesian_points p;

	p.x = ((int)cell_number%(int)dynamic_objects_map_g->config.x_size)*dynamic_objects_map_g->config.resolution;		  // transforming from a array coordinate to a x,y coordinate
	p.y = ((int)cell_number/(int)dynamic_objects_map_g->config.x_size)*dynamic_objects_map_g->config.resolution;
	
	to_global_pose(&p, dynamic_objects_map_g->config);
	
	carmen_dynamic_object_detector_object* d_obj = carmen_dynamic_object_detector_point_belongs_to_object(p.x,p.y);
	if( d_obj != NULL)
	{
		carmen_dynamic_object_detector_add_point_to_dynamic_object(d_obj, p.x, p.y);
	}
	else 
	{
		carmen_dynamic_object_detector_add_object_to_dynamic_map(p.x, p.y);
	}

	return;
}

carmen_dynamic_object_detector_clustered_objects_message *
carmen_dynamic_object_detector_run(carmen_mapper_map_message* current_map, carmen_mapper_map_message* previous_map)
{
	for(int i = 0; i < previous_map_g->size; i++)
	{
		dynamic_objects_map_g->objects_map[i] = carmen_dynamic_object_detector_analize_gridmap_cell(current_map->complete_map[i], previous_map->complete_map[i], dynamic_objects_map_g->objects_map[i]);
		int value = (int) dynamic_objects_map_g->objects_map[i];
		if(value == 1)
		{
			
			carmen_dynamic_object_detector_add_cell_to_dynamic_objects_list(i);
			
		}
	}

	carmen_dynamic_object_detector_object_update();

	return dynamic_objects_map_g;
}

/*********************************************************
		   --- Publishers ---
**********************************************************/

void
carmen_dynamic_object_detector_publish_clustered_objects_message(carmen_dynamic_object_detector_clustered_objects_message* message)
{
	IPC_RETURN_TYPE err;
//	if(dynamic_objects_map_g->objects_list == NULL)
//	  printf("kkk\n");
//	else
//	  printf("%lud, %lud\n", sizeof(*dynamic_objects_map_g->objects_list), sizeof(carmen_dynamic_object_detector_object)*(dynamic_objects_map_g->obj_count+1));
	  //printf("n = %d\n", dynamic_objects_map_g->obj_count);
	
	err = IPC_publishData(CARMEN_DYNAMIC_OBJECT_DETECTOR_CLUSTERED_OBJECTS_NAME, message);
	carmen_test_ipc_exit(err, "Could not publish", CARMEN_DYNAMIC_OBJECT_DETECTOR_CLUSTERED_OBJECTS_NAME);
}

/*********************************************************
		   --- Handlers ---
**********************************************************/

void
carmen_grid_mapping_handler(carmen_mapper_map_message* message)
{
	
	if(current_map_g->complete_map == NULL)
	{
		current_map_g->complete_map = (double *) malloc (message->size * sizeof(double));
		current_map_g->config.map_name = (char*)malloc (256 * sizeof(char));
	}
	
	if(previous_map_g->complete_map ==  NULL)
	{
		previous_map_g->complete_map = (double*)malloc(message->size * sizeof(double));
		previous_map_g->config.map_name = (char*) malloc (256 * sizeof(char));
	}
	
	copy_current_map_to_previous_map(previous_map_g, current_map_g);
	copy_current_map_to_previous_map(current_map_g, message);
	
	
	if(map_counter > 0)
	{
			if(current_map_g->config.x_origin == previous_map_g->config.x_origin && previous_map_g->config.y_origin == current_map_g->config.y_origin)
			{
				dynamic_objects_map_g = carmen_dynamic_object_detector_run(current_map_g, previous_map_g);
				carmen_dynamic_object_detector_publish_clustered_objects_message(dynamic_objects_map_g);
				carmen_dynamic_object_detector_publish_dynamic_object_map();
			}
	}
	else
	{
		dynamic_objects_map_g = carmen_configure_dynamic_object_detector_clustered_message_with_grid_mapping_information(message);
		map_counter++;
	}
	
}

void
carmen_localize_ackerman_handler(carmen_localize_ackerman_globalpos_message* message)
{
  
	if(message)
	{
		car_global_x = message->globalpos.x;
		car_global_y = message->globalpos.y;
	}
	else
	{
		car_global_x = 0;
		car_global_y = 0;
	}
  
}

void static
shutdown_module(int signo)
{
  if (signo == SIGINT)
  {
    carmen_ipc_disconnect();
    printf("dymamic_object_detector: disconnected.\n");

    exit(0);
  }
}
  
int 
main(int argc, char **argv) 
{
  /* Connect to IPC Server */
  carmen_ipc_initialize(argc, argv);
  
  /* Check the param server version */
  carmen_param_check_version(argv[0]);

  /* Register shutdown cleaner handler */
  signal(SIGINT, shutdown_module);

  /* Specific module initialization */
  initialize_dynamic_object_detector();

  /* Define messages that your module publishes */
  carmen_dynamic_object_detector_define_messages();

  /* Subscribe to sensor and filter messages */
  carmen_mapper_subscribe_message(NULL, (carmen_handler_t)carmen_grid_mapping_handler, CARMEN_SUBSCRIBE_LATEST);
  
  /* Subscribe to car positioning messages */
  carmen_localize_ackerman_subscribe_globalpos_message(NULL,(carmen_handler_t)carmen_localize_ackerman_handler, CARMEN_SUBSCRIBE_LATEST);

  /* Loop forever waiting for messages */
  carmen_ipc_dispatch();

  return (0);
}
