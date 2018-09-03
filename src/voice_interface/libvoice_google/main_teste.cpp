
#include <stdio.h>
#include "voice_functions.h"


int
main()
{
    init_voice();

    const char* retorno = listen();
    printf("Texto reconhecido: %s\n", retorno);
    speak((char*) retorno);

    finalize_voice();
    sleep(10);

    return 0;
}
