
#include <stdio.h>
#include "voice_functions.h"


int
main()
{
    init_voice();

    const char* retorno = listen();
    printf("Texto reconhecido: %s\n", retorno);
    speak((char*)"Alguns módulos estão instáveis!");

    finalize_voice();
    sleep(10);

    return 0;
}
