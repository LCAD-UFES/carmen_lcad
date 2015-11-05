#ifndef WINDOW_H_
#define WINDOW_H_

#include <stdlib.h>
#include <stdio.h>
#include <GL/glx.h>
#include <GL/gl.h>
#include <GL/glu.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct window window;

// Funcao cria a janela
window* initWindow();

// Efetivamente mostra na tela o conteudo, retorna 1 se a janela ainda existe ou 0 se a
// janela foi fechada
int showWindow(window* w);

int processWindow(window* w, void (*mouseFunc)(int type, int button, int x, int y), void (*keyPress)(int code), void (*keyRelease)(int code));

void destroyWindow(window* w);

#ifdef __cplusplus
}
#endif

#endif
