#ifndef LIBDEEPLAB_H
#define LIBDEEPLAB_H

void initialize_inference_context();
unsigned char* process_image(int width, int height, unsigned char *image);

#endif // LIBDEEPLAB_H