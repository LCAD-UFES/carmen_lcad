#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include "glm.h"
#include "glmint.h"

#ifdef HAVE_LIBSDL_IMAGE
#include <SDL/SDL_image.h>

/* following routine is from the SDL documentation */
/*
 * Return the pixel value at (x, y)
 * NOTE: The surface must be locked before calling this!
 */
static inline Uint32 getpixel(SDL_Surface *surface, int x, int y)
{
    int bpp = surface->format->BytesPerPixel;
    /* Here p is the address to the pixel we want to retrieve */
    Uint8 *p = (Uint8 *)surface->pixels + y * surface->pitch + x * bpp;

    switch(bpp) {
    case 1:
        return *p;

    case 2:
        return *(Uint16 *)p;

    case 3:
        if(SDL_BYTEORDER == SDL_BIG_ENDIAN)
            return p[0] << 16 | p[1] << 8 | p[2];
        else
            return p[0] | p[1] << 8 | p[2] << 16;

    case 4:
        return *(Uint32 *)p;

    default:
        return 0;       /* shouldn't happen, but avoids warnings */
    }
}

GLubyte* 
glmReadSDL(const char* filename, GLboolean alpha, int* width, int* height, int* type)
{
    SDL_Surface *img = NULL;
    int dim;
    int pos;
    int x,y;
    GLubyte *data;

    /* Load the 'file' to SDL_Surface */
    DBG_(__glmWarning("loading texture %s",filename));
    img = IMG_Load(filename);
    if(img == NULL) {
	__glmWarning("glmLoadTexture() failed: Unable to load texture from %s!\n%s", filename, IMG_GetError());
        return NULL;
    }

    /* Build the texture from the surface */
    dim = img->w * img->h * ((alpha) ? 4 : 3);
    data = (GLubyte*)malloc(sizeof(GLubyte) * dim);
    if(!data) {
	__glmWarning("glmLoadTexture() failed: Unable to create a texture from %s!", filename);
        return NULL;
    }
    
    /* Traverse trough surface and grab the pixels */
    pos = 0;
    
    for(y=(img->h-1); y>-1; y--) {
	for(x=0; x<img->w; x++) {
	    Uint8 r,g,b,a;
	    Uint32 color = getpixel(img, x,y);
	    
	    if(!alpha)
		SDL_GetRGB(color, img->format, &r,&g,&b);
	    else
		SDL_GetRGBA(color, img->format, &r,&g,&b,&a);

	    data[pos] = r; pos++;
	    data[pos] = g; pos++;
	    data[pos] = b; pos++;
	    if(alpha) {
		data[pos] = a; pos++;
	    }
	}
    }

    *type = (alpha) ? GL_RGBA : GL_RGB;
    *width = img->w;
    *height = img->h;
    SDL_FreeSurface(img);

    return data;
}
#endif	/* HAVE_LIBSDL_IMAGE */
