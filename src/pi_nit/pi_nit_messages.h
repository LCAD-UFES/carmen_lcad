/*********************************************************
 Pi NIT Module - Mensagens IPC

 O modulo publica as deteccoes na mensagem padrao do CARMEN
 (neural_detector_message, ver neural_detector_messages.h), de forma que
 qualquer consumidor existente (neural_image_tracker, image_path_projector,
 behavior_selector, ...) funcione sem nenhuma alteracao.

 Alem disso publica a mensagem de status abaixo, com a saude do enlace
 com o Raspberry Pi (fps, latencia, frames descartados).
 *********************************************************/

#ifndef CARMEN_PI_NIT_MESSAGES_H
#define CARMEN_PI_NIT_MESSAGES_H

#include <carmen/carmen.h>

#ifdef __cplusplus
extern "C"
{
#endif


typedef struct
{
    int    camera_id;            // numero da mensagem de camera do CARMEN
    int    connected;            // 1 se chegou resultado do Pi no ultimo segundo
    int    frames_sent;          // acumulado desde o inicio do modulo
    int    frames_received;      // acumulado (frames_sent - frames_received = descartados)
    int    detections;           // pessoas detectadas no ultimo frame
    double fps_sent;             // taxa media de envio (janela de 1s)
    double fps_received;         // taxa media de retorno (janela de 1s)
    double round_trip_ms;        // envio do frame ate a publicacao da deteccao
    double inference_ms;         // tempo dentro do Hailo, informado pelo Pi
    double timestamp;
    char  *host;
} carmen_pi_nit_status_message;

#define CARMEN_PI_NIT_STATUS_NAME    "carmen_pi_nit_status_message"
#define CARMEN_PI_NIT_STATUS_FMT     "{int, int, int, int, int, double, double, double, double, double, string}"


#ifdef __cplusplus
}
#endif

#endif
