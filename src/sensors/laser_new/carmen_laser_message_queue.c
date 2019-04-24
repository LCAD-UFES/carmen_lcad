#include <stdio.h>
#include "carmen_laser_message_queue.h"

void carmen_laser_message_queue_init(carmen_laser_message_queue_t* queue){
	queue->head=queue->tail=queue->size=0;
#ifdef LASER_USE_PTHREAD
	pthread_mutex_init(&(queue->mutex),NULL);
	sem_init(&(queue->semaphore),0,0);
#endif
}

void carmen_laser_message_queue_destroy(carmen_laser_message_queue_t* queue){
#ifdef LASER_USE_PTHREAD
	pthread_mutex_destroy(&(queue->mutex));
	sem_destroy(&(queue->semaphore));
#endif
	queue->head=queue->tail=0;
}

int carmen_laser_message_queue_add(carmen_laser_message_queue_t* queue, carmen_laser_laser_static_message* message){
	int returnedSize;
	int added;
#ifdef LASER_USE_PTHREAD
	pthread_mutex_lock(&queue->mutex);
#endif
	queue->readings[queue->tail++]=*message;
	queue->size++;
	returnedSize=queue->size;
	added=1;
	if (queue->size>CARMEN_LASER_MESSAGE_QUEQE_SIZE){
		queue->size--;
		queue->head++;
		added=0;
	}
	queue->head=queue->head%CARMEN_LASER_MESSAGE_QUEQE_SIZE;
	queue->tail=queue->tail%CARMEN_LASER_MESSAGE_QUEQE_SIZE;
#ifdef LASER_USE_PTHREAD
	pthread_mutex_unlock(&queue->mutex);
	if (added)
		sem_post(&queue->semaphore);
#endif
	return returnedSize;
}

int carmen_laser_message_queue_get(carmen_laser_message_queue_t* queue, carmen_laser_laser_static_message* message){
	int returnedSize;
	if (!queue->size)
	  return -1;
#ifdef LASER_USE_PTHREAD
	pthread_mutex_lock(&queue->mutex);
#endif
	returnedSize=queue->size-1;
	if (queue->size>0){
		*message=queue->readings[queue->head++];
	}
	queue->size--;
	if (queue->size>CARMEN_LASER_MESSAGE_QUEQE_SIZE){
		queue->head++;
	}
	queue->head=queue->head%CARMEN_LASER_MESSAGE_QUEQE_SIZE;
	queue->tail=queue->tail%CARMEN_LASER_MESSAGE_QUEQE_SIZE;
#ifdef LASER_USE_PTHREAD
	pthread_mutex_unlock(&queue->mutex);
#endif
	return returnedSize;
}

#ifdef LASER_USE_PTHREAD
int carmen_laser_message_queue_wait_get(carmen_laser_message_queue_t* queue, carmen_laser_laser_static_message* message){
  int sval;
  sem_getvalue(&queue->semaphore, &sval);
  sem_wait(&queue->semaphore);
  int v=carmen_laser_message_queue_get(queue, message);
  // fprintf(stderr, "%s, sem=%d, q=%d\n", __PRETTY_FUNCTION__, sval, v);
  return v;
}
#endif

