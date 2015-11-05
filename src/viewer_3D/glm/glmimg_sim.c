#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include "glm.h"
#include "glmint.h"

#ifdef HAVE_LIBSIMAGE
/* http://www.coin3d.org/ */
#include <simage.h>

GLubyte* 
glmReadSimage(const char* filename, GLboolean alpha, int* w, int* h, int* type)
{
    int numcomponents;
    GLubyte *buffer = simage_read_image(filename, w, h, &numcomponents);
    switch(numcomponents) {
    case 1:
	*type = GL_LUMINANCE;
	break;
    case 2:
	*type = GL_LUMINANCE_ALPHA;
	break;
    case 3:
	*type = GL_RGB;
	break;
    case 4:
	*type = GL_RGBA;
	break;
    }
    return buffer;
}
#endif
