/*********************************************************
 Pi NIT (Neural Image Tracker no Raspberry Pi) - Protocolo ZMQ

 Formato binario compartilhado entre o cliente C++ (PC do CARMEN) e o
 servidor Python (Raspberry Pi 5 + Hailo-8L).

 O mesmo layout esta descrito em pi_nit_server/pi_nit_protocol.py.
 QUALQUER alteracao aqui exige a alteracao equivalente la (e vice-versa):
 os dois lados validam PI_NIT_PROTOCOL_VERSION no handshake de cada frame.

 Fluxo:
   PC  --[pi_nit_frame_header_t + payload]-->  Pi   (porta de frames, PUSH/PULL)
   Pi  --[pi_nit_result_header_t + N x det]-->  PC  (porta de resultados, PUSH/PULL)

 Cada mensagem ZMQ e' de uma unica parte (header e payload sao concatenados
 em um unico buffer contiguo) porque o socket de frames usa ZMQ_CONFLATE,
 que nao suporta mensagens multi-part.
 *********************************************************/

#ifndef PI_NIT_PROTOCOL_H
#define PI_NIT_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define PI_NIT_PROTOCOL_VERSION     1

#define PI_NIT_FRAME_MAGIC          "PNIF"
#define PI_NIT_RESULT_MAGIC         "PNIR"

// Resolucao de entrada da rede neural no Hailo. O cliente faz letterbox da
// imagem original para este tamanho e desfaz o mapeamento nas deteccoes.
#define PI_NIT_INFERENCE_SIZE       640

// Formatos de payload aceitos pelo servidor
#define PI_NIT_FORMAT_BGR8          0   // bytes crus, width * height * 3 (OpenCV BGR)
#define PI_NIT_FORMAT_JPEG          1   // buffer JPEG (recomendado: ~15x menos rede)

// Portas padrao (o servidor faz bind, o cliente faz connect)
#define PI_NIT_DEFAULT_FRAME_PORT   5560
#define PI_NIT_DEFAULT_RESULT_PORT  5561

// Classe COCO usada para pessoa (mantida igual a do neural_image_tracker)
#define PI_NIT_CLASS_PERSON         0


// Cabecalho enviado pelo PC junto com a imagem. Todos os campos sao
// little-endian e o struct e' naturalmente alinhado (48 bytes).
typedef struct
{
    char     magic[4];      // "PNIF"
    uint16_t version;       // PI_NIT_PROTOCOL_VERSION
    uint16_t format;        // PI_NIT_FORMAT_*
    uint32_t width;         // largura da imagem enviada (= PI_NIT_INFERENCE_SIZE)
    uint32_t height;        // altura  da imagem enviada (= PI_NIT_INFERENCE_SIZE)
    uint32_t client_id;     // id aleatorio do processo cliente (detecta cliente duplicado)
    int32_t  camera_id;     // numero da mensagem de camera do CARMEN
    uint64_t frame_id;      // contador monotonico por cliente
    double   timestamp;     // timestamp da imagem original (carmen_get_time)
    uint32_t payload_len;   // bytes de imagem que seguem o header
    uint32_t reserved;      // 0
} pi_nit_frame_header_t;


// Cabecalho devolvido pelo Raspberry, seguido de num_detections registros
// pi_nit_detection_t (48 bytes de header + 24 bytes por deteccao).
typedef struct
{
    char     magic[4];          // "PNIR"
    uint16_t version;           // PI_NIT_PROTOCOL_VERSION
    uint16_t num_detections;
    uint32_t client_id;         // ecoado do frame
    int32_t  camera_id;         // ecoado do frame
    uint64_t frame_id;          // ecoado do frame
    double   image_timestamp;   // ecoado do frame
    double   server_timestamp;  // relogio do Pi no instante do envio
    float    inference_ms;      // tempo gasto no Hailo
    float    queue_ms;          // tempo que o frame esperou na fila do Pi
} pi_nit_result_header_t;


// Uma deteccao, em coordenadas da imagem 640x640 recebida pelo servidor.
typedef struct
{
    float   x1, y1;     // canto superior esquerdo
    float   x2, y2;     // canto inferior direito
    float   score;      // confianca [0, 1]
    int32_t class_id;   // classe COCO (0 = pessoa)
} pi_nit_detection_t;


// Falha de compilacao se o layout divergir do esperado pelo lado Python.
typedef char pi_nit_frame_header_size_check[(sizeof(pi_nit_frame_header_t) == 48) ? 1 : -1];
typedef char pi_nit_result_header_size_check[(sizeof(pi_nit_result_header_t) == 48) ? 1 : -1];
typedef char pi_nit_detection_size_check[(sizeof(pi_nit_detection_t) == 24) ? 1 : -1];

#ifdef __cplusplus
}
#endif

#endif
