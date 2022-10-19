/*********************************************************
	 ---   My Module Specific Messages ---

See IPC documentation for more information:
http://www.cs.cmu.edu/~ipc/

*********************************************************/

#ifndef NAVIGATOR_GUI2_MESSAGES_H
#define NAVIGATOR_GUI2_MESSAGES_H

#ifdef __cplusplus
extern "C"
{
#endif


typedef struct {
  int width;					/* The x dimension of the image in pixels */
  int height;					/* The y dimension of the image in pixels */
  int image_size;				/* width*height*bytes_per_pixel */
  unsigned char *raw_image;
  double x_origin;				/* world coordinate x in meters of image's left corner */
  double y_origin;				/* world coordinate y in meters of image's bottom corner */
  double resolution;			/* meters per pixel */
  double timestamp;
  char *host;
} carmen_navigator_gui_map_view_message;


#define      CARMEN_NAVIGATOR_GUI_MAP_VIEW_NAME       "carmen_navigator_gui2_map_view"
#define      CARMEN_NAVIGATOR_GUI_MAP_VIEW_FMT        "{int,int,int,<ubyte:3>,double,double,double,double,string}"
//#define      CARMEN_NAVIGATOR_GUI_MAP_VIEW_FMT        "{int,int,int,<ubyte:3>,double,double,double,double,string}" // Essa mensagem, de acordo com testes, está fazendo o navigator gui travar quando orienta o final goal


#ifdef __cplusplus
}
#endif

#endif
