#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include "glm.h"
#include "glmint.h"

#ifdef HAVE_DEVIL
#include <IL/il.h>
#include <stdlib.h>

GLubyte* 
glmReadDevIL(const char* filename, GLboolean alpha, int* width, int* height, int* type)
{
    ILuint image;
    ILenum format;
    int dim;
    GLubyte *data;

    ilInit();
    ilEnable(IL_ORIGIN_SET);
    ilOriginFunc(IL_ORIGIN_LOWER_LEFT);
    DBG_(__glmWarning("loading DevIL texture %s",filename));
    ilGenImages(1,&image);

    ilBindImage(image);
    if (!ilLoadImage((char*)filename)) {
	DBG_(__glmWarning("glmLoadTexture(): DevIL ilLoadImage(%s): error %s\n", filename, ilGetError()));
	ilDeleteImages(1, &image);
	return NULL;
    }
    *height = ilGetInteger(IL_IMAGE_HEIGHT);
    *width = ilGetInteger(IL_IMAGE_WIDTH);
    *type = alpha ? GL_RGBA : GL_RGB;
    format = alpha ? IL_RGBA : IL_RGB;
    dim = *width * *height * ((alpha) ? 4 : 3);
    data = (GLubyte*)malloc(sizeof(GLubyte) * dim);
    ilCopyPixels( 0, 0, 0, *width, *height, 1, format, IL_UNSIGNED_BYTE, data);
    ilDeleteImages(1, &image);
    if (ilGetError() == IL_NO_ERROR) {
	DBG_(__glmWarning("loaded DevIL texture %s",filename));
    	return data;
    }
    DBG_(__glmWarning("glmLoadTexture(): DevIL ilCopyPixels(): error %s\n", ilGetError()));
    free(data);
    return NULL;
}
#endif	/* HAVE_DEVIL */
