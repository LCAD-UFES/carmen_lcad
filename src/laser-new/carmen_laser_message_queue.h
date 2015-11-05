#ifndef CARMEN_LASER_MESSAGE_QUEUE
#define CARMEN_LASER_MESSAGE_QUEUE

#ifdef LASER_USE_PTHREAD
#include <pthread.h>
#include <semaphore.h>
#endif

#include "laser_messages.h"
#include "laser_static_messages.h"

#define CARMEN_LASER_MESSAGE_QUEQE_SIZE 4096

typedef struct {
	carmen_laser_laser_static_message readings[CARMEN_LASER_MESSAGE_QUEQE_SIZE];
#ifdef LASER_USE_PTHREAD
	pthread_mutex_t mutex;
	sem_t semaphore;
#endif
	int head, tail, size;
} carmen_laser_message_queue_t;

int carmen_laser_message_queue_add(carmen_laser_message_queue_t* queue, carmen_laser_laser_static_message* message);
int carmen_laser_message_queue_get(carmen_laser_message_queue_t* queue, carmen_laser_laser_static_message* message);
void carmen_laser_message_queue_init(carmen_laser_message_queue_t* queue);
void carmen_laser_message_queue_destroy(carmen_laser_message_queue_t* queue);

#ifdef LASER_USE_PTHREAD
int carmen_laser_message_queue_wait_get(carmen_laser_message_queue_t* queue, carmen_laser_laser_static_message* message);
#endif

#endif
