/*********************************************************
 Visual Search Module
 *********************************************************/

#ifndef CARMEN_VISUAL_MEMORY_MESSAGES_H
#define CARMEN_VISUAL_MEMORY_MESSAGES_H

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        int image_size;
        unsigned char *reference_image;
        double timestamp;
        char *host;
    } carmen_visual_memory_message;

    typedef struct
    {
        int image_size;
        unsigned char *visual_memory;
        double timestamp;
        char *host;
    } carmen_visual_memory_output_message;

#define		CARMEN_VISUAL_MEMORY_TRAINING_MESSAGE_NAME	"carmen_visual_memory_train_message"
#define		CARMEN_VISUAL_MEMORY_TRAINING_MESSAGE_FMT	"{int,<ubyte:1>,double,string}"

#define		CARMEN_VISUAL_MEMORY_OUTPUT_MESSAGE_NAME	"carmen_visual_memory_output_message"
#define		CARMEN_VISUAL_MEMORY_OUTPUT_MESSAGE_FMT		"{int,<ubyte:1>},double,string}"

#ifdef __cplusplus
}
#endif

#endif


