#ifndef VISUAL_ODOMETRY_HANDLER_H
#define VISUAL_ODOMETRY_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _visual_odometry_handler visual_odometry_handler;

visual_odometry_handler* create_visual_odometry_handler(int argc, char** argv);
void reset_visual_odometry_handler(visual_odometry_handler* viso_handler);
void destroy_visual_odometry_handler(visual_odometry_handler* viso_handler);


#ifdef __cplusplus
}
#endif

#endif
