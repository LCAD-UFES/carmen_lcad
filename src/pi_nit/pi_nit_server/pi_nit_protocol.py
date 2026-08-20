"""Protocolo binario ZMQ do pi_nit (lado Raspberry Pi).

Espelho exato de ../pi_nit_protocol.h. Qualquer alteracao aqui exige a
alteracao equivalente la (e vice-versa): os dois lados validam
PROTOCOL_VERSION em cada frame.

Layout (little-endian, tudo naturalmente alinhado):

  frame header  (48 bytes)  magic "PNIF" + metadados + imagem no mesmo buffer
  result header (48 bytes)  magic "PNIR" seguido de N deteccoes de 24 bytes
"""

import struct
from typing import List, NamedTuple, Tuple

PROTOCOL_VERSION = 1

FRAME_MAGIC = b"PNIF"
RESULT_MAGIC = b"PNIR"

INFERENCE_SIZE = 640

FORMAT_BGR8 = 0
FORMAT_JPEG = 1

DEFAULT_FRAME_PORT = 5560
DEFAULT_RESULT_PORT = 5561

CLASS_PERSON = 0

# Nome e cor (BGR) das classes COCO que o servidor costuma devolver, conforme o
# PI_NIT_CLASSES. Fica aqui para o test_client e o pi_nit_viewer desenharem do
# mesmo jeito - duas tabelas separadas acabariam divergindo.
CLASSES = {
    0: ("pessoa", (0, 255, 0)),
    1: ("bicicleta", (255, 200, 0)),
    2: ("carro", (255, 128, 0)),
    3: ("moto", (255, 0, 255)),
    5: ("onibus", (0, 200, 255)),
    7: ("caminhao", (0, 128, 255)),
}


def class_name_and_color(class_id):
    """Nome e cor de uma classe; cinza para o que nao esta na tabela."""
    return CLASSES.get(class_id, (f"classe {class_id}", (200, 200, 200)))

# char[4] uint16 uint16 uint32 uint32 uint32 int32 uint64 double uint32 uint32
FRAME_HEADER_STRUCT = struct.Struct("<4sHHIIIiQdII")
# char[4] uint16 uint16 uint32 int32 uint64 double double float float
RESULT_HEADER_STRUCT = struct.Struct("<4sHHIiQddff")
# float x1 y1 x2 y2 score, int32 class_id
DETECTION_STRUCT = struct.Struct("<5fi")

assert FRAME_HEADER_STRUCT.size == 48, FRAME_HEADER_STRUCT.size
assert RESULT_HEADER_STRUCT.size == 48, RESULT_HEADER_STRUCT.size
assert DETECTION_STRUCT.size == 24, DETECTION_STRUCT.size


class FrameHeader(NamedTuple):
    version: int
    format: int
    width: int
    height: int
    client_id: int
    camera_id: int
    frame_id: int
    timestamp: float
    payload_len: int


class Detection(NamedTuple):
    """Caixa em pixels da imagem 640x640 recebida."""

    x1: float
    y1: float
    x2: float
    y2: float
    score: float
    class_id: int


class ProtocolError(Exception):
    pass


def parse_frame(message: bytes) -> Tuple[FrameHeader, memoryview]:
    """Separa o cabecalho e o payload de imagem de uma mensagem recebida.

    Levanta ProtocolError quando a mensagem nao e' um frame valido - o
    servidor apenas registra e descarta, nunca derruba o servico.
    """
    if len(message) < FRAME_HEADER_STRUCT.size:
        raise ProtocolError(f"mensagem curta demais: {len(message)} bytes")

    (magic, version, image_format, width, height, client_id, camera_id,
     frame_id, timestamp, payload_len, _reserved) = FRAME_HEADER_STRUCT.unpack_from(message, 0)

    if magic != FRAME_MAGIC:
        raise ProtocolError(f"magic invalido: {magic!r}")
    if version != PROTOCOL_VERSION:
        raise ProtocolError(f"versao {version} incompativel (esperado {PROTOCOL_VERSION})")

    expected = FRAME_HEADER_STRUCT.size + payload_len
    if len(message) != expected:
        raise ProtocolError(f"tamanho inconsistente: {len(message)} bytes, esperado {expected}")

    header = FrameHeader(version, image_format, width, height, client_id,
                         camera_id, frame_id, timestamp, payload_len)
    payload = memoryview(message)[FRAME_HEADER_STRUCT.size:]

    return header, payload


def build_result(header: FrameHeader, detections: List[Detection],
                 server_timestamp: float, inference_ms: float, queue_ms: float) -> bytes:
    """Monta a resposta binaria com as deteccoes de um frame."""
    parts = [RESULT_HEADER_STRUCT.pack(
        RESULT_MAGIC,
        PROTOCOL_VERSION,
        len(detections),
        header.client_id,
        header.camera_id,
        header.frame_id,
        header.timestamp,
        server_timestamp,
        float(inference_ms),
        float(queue_ms),
    )]

    for detection in detections:
        parts.append(DETECTION_STRUCT.pack(
            float(detection.x1), float(detection.y1),
            float(detection.x2), float(detection.y2),
            float(detection.score), int(detection.class_id),
        ))

    return b"".join(parts)


def parse_result(message: bytes) -> Tuple[dict, List[Detection]]:
    """Le uma resposta - usado apenas por tools/test_client.py."""
    if len(message) < RESULT_HEADER_STRUCT.size:
        raise ProtocolError(f"resultado curto demais: {len(message)} bytes")

    (magic, version, num_detections, client_id, camera_id, frame_id,
     image_timestamp, server_timestamp, inference_ms, queue_ms) = RESULT_HEADER_STRUCT.unpack_from(message, 0)

    if magic != RESULT_MAGIC:
        raise ProtocolError(f"magic invalido: {magic!r}")
    if version != PROTOCOL_VERSION:
        raise ProtocolError(f"versao {version} incompativel")

    expected = RESULT_HEADER_STRUCT.size + num_detections * DETECTION_STRUCT.size
    if len(message) != expected:
        raise ProtocolError(f"tamanho inconsistente: {len(message)} bytes, esperado {expected}")

    detections = [
        Detection(*DETECTION_STRUCT.unpack_from(message, RESULT_HEADER_STRUCT.size + i * DETECTION_STRUCT.size))
        for i in range(num_detections)
    ]

    header = {
        "client_id": client_id,
        "camera_id": camera_id,
        "frame_id": frame_id,
        "image_timestamp": image_timestamp,
        "server_timestamp": server_timestamp,
        "inference_ms": inference_ms,
        "queue_ms": queue_ms,
    }

    return header, detections
