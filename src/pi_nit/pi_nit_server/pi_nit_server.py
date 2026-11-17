#!/usr/bin/env python3
"""Servico de deteccao de pessoas do pi_nit (roda no Raspberry Pi 5 + Hailo-8L).

Recebe frames 640x640 do PC do CARMEN por ZMQ, roda a rede no Hailo e devolve
as caixas detectadas. Nao faz mais nada: nao grava imagem, nao abre camera,
nao mantem estado entre frames.

  PC   --PUSH--> tcp://*:5560 (PULL)  frame  = header 48B + JPEG/BGR
  Pi   --PUSH--> tcp://*:5561 (PULL)  result = header 48B + N x 24B

As 3 cameras compartilham a mesma porta. A cada volta juntamos ate
--batch-size frames, no maximo um por camera (o mais novo), e mandamos as 3
imagens ao Hailo em uma unica inferencia. Se a inferencia atrasar, os frames
velhos sao descartados e processamos sempre os mais recentes: em percepcao
para navegacao, latencia baixa vale mais que processar todo frame.

Uso:
    python3 pi_nit_server.py --hef /opt/pi_nit/models/yolov8s_h8l.hef
    python3 pi_nit_server.py --dummy          # sem Hailo, so testa a rede
"""

import argparse
import errno
import logging
import os
import signal
import sys
import time
from typing import NamedTuple

import cv2
import numpy as np
import zmq

from hailo_person_detector import DetectorError, DummyDetector, HailoPersonDetector
from pi_nit_protocol import (CLASS_PERSON, DEFAULT_FRAME_PORT, DEFAULT_RESULT_PORT,
                             FORMAT_BGR8, FORMAT_JPEG, ProtocolError, build_result,
                             parse_frame)

logger = logging.getLogger("pi_nit.server")

running = True


def handle_signal(signum, _frame):
    global running
    logger.info("sinal %d recebido, encerrando", signum)
    running = False


def bind_or_explain(socket, bind_address: str, port: int, what: str) -> None:
    """bind() com mensagem util no lugar do traceback do ZMQ.

    Os dois erros que aparecem na pratica sao porta privilegiada (numero
    cortado ao colar o comando) e porta ja em uso (outra instancia rodando).
    """
    if port < 1024:
        raise SystemExit(
            f"erro: porta {port} para {what} e' privilegiada (abaixo de 1024) e exige root.\n"
            f"       Use uma porta acima de 1024 - os padroes sao "
            f"{DEFAULT_FRAME_PORT}/{DEFAULT_RESULT_PORT} e 5562 para a visualizacao.\n"
            f"       (numero cortado ao colar o comando? '--viewer-port 55' em vez de '5562')")

    if port > 65535:
        raise SystemExit(f"erro: porta {port} para {what} nao existe (maximo 65535)")

    try:
        socket.bind(f"tcp://{bind_address}:{port}")
    except zmq.ZMQError as exc:
        if exc.errno == errno.EADDRINUSE:
            raise SystemExit(
                f"erro: a porta {port} ({what}) ja esta em uso.\n"
                f"       Ha outro pi_nit_server rodando? Veja com:\n"
                f"         systemctl status pi_nit_server\n"
                f"         ss -ltnp | grep {port}") from exc
        if exc.errno == errno.EADDRNOTAVAIL:
            raise SystemExit(
                f"erro: o endereco {bind_address} nao existe nesta maquina "
                f"(--bind). Use 0.0.0.0 para aceitar de qualquer rede.") from exc
        raise SystemExit(f"erro ao abrir a porta {port} ({what}): {exc}") from exc


def build_viewer_socket(context, viewer_port: int, bind_address: str):
    """PUB opcional que reemite frame + deteccoes para o pi_nit_viewer.

    Fica fora do caminho critico: PUB nunca bloqueia e, com HWM 2, descarta
    sozinho quando nao ha ninguem olhando. Desligado (porta 0) nao custa nada.
    """
    if viewer_port <= 0:
        return None

    socket = context.socket(zmq.PUB)
    socket.setsockopt(zmq.SNDHWM, 2)
    socket.setsockopt(zmq.LINGER, 0)
    bind_or_explain(socket, bind_address, viewer_port, "visualizacao")
    logger.info("visualizacao publicada em tcp://%s:%d", bind_address, viewer_port)

    return socket


def build_sockets(frame_port: int, result_port: int, bind_address: str, queue_depth: int = 8):
    context = zmq.Context(io_threads=1)

    # Nada de ZMQ_CONFLATE aqui: ele guarda "a ultima mensagem" do socket
    # inteiro e, com 3 cameras dividindo a mesma porta, comeria 2 de cada 3
    # frames. Quem descarta o frame velho e' collect_batch(), que sabe ler o
    # camera_id.
    frame_socket = context.socket(zmq.PULL)
    frame_socket.setsockopt(zmq.RCVHWM, queue_depth)
    frame_socket.setsockopt(zmq.LINGER, 0)
    bind_or_explain(frame_socket, bind_address, frame_port, "frames")

    result_socket = context.socket(zmq.PUSH)
    result_socket.setsockopt(zmq.SNDHWM, 8)
    result_socket.setsockopt(zmq.LINGER, 0)
    result_socket.setsockopt(zmq.IMMEDIATE, 1)  # nao enfileira se ninguem esta conectado
    bind_or_explain(result_socket, bind_address, result_port, "resultados")

    logger.info("escutando frames em tcp://%s:%d", bind_address, frame_port)
    logger.info("publicando resultados em tcp://%s:%d", bind_address, result_port)

    return context, frame_socket, result_socket


class BatchItem(NamedTuple):
    header: object          # FrameHeader
    image: np.ndarray       # RGB uint8 640x640
    raw: bytes              # mensagem original, reemitida para o viewer
    received_at: float      # time.monotonic() da chegada


# Uma camera some da conta se ficar este tempo sem mandar frame. Precisa ser
# bem maior que o periodo de um frame (66 ms a 15 fps) para nao piscar, e curto
# o bastante para o servidor voltar a responder rapido quando uma camera cai.
CAMERA_ACTIVE_S = 2.0


def expected_cameras(state: dict, batch_size: int) -> int:
    """Quantas cameras vale a pena esperar neste batch.

    E' o numero de cameras que publicaram nos ultimos CAMERA_ACTIVE_S, limitado
    pelo batch_size com que o Hailo foi configurado. Com uma camera so devolve
    1, e a janela de espera deixa de existir na pratica.
    """
    now = time.monotonic()
    active = sum(1 for last_seen in state["cameras_seen"].values()
                 if now - last_seen <= CAMERA_ACTIVE_S)

    return max(1, min(batch_size, active))


def collect_batch(frame_socket, poller, batch_size: int, batch_window_ms: float, state: dict):
    """Junta ate batch_size frames, no maximo um (o mais novo) por camera.

    As 3 cameras publicam quase juntas, entao esperamos uma janela curta
    (batch_window_ms) depois do primeiro frame para as outras chegarem e
    mandamos as 3 imagens ao Hailo de uma vez so.

    Duas regras que fazem a diferenca:

    - Se chegar mais de um frame da MESMA camera na janela, fica so o mais
      novo. E' o descarte que o ZMQ_CONFLATE faria, mas por camera - com um
      socket compartilhado, o CONFLATE comeria os frames das outras cameras.
    - A janela so' e' esperada enquanto faltam cameras que estao mesmo
      publicando. Esperar pelo batch_size cheio custa a janela INTEIRA em
      todo frame quando ha menos cameras que batch_size: com uma camera a
      15 fps e janela de 20 ms, 66 ms viram 86 ms - a taxa cai de 15 para
      11 fps sem nenhum ganho. Por isso o alvo e' o numero de cameras vistas
      ha pouco (CAMERA_ACTIVE_S), limitado pelo batch_size.
    """
    newest_by_camera = {}
    started_at = None

    while running:
        target = expected_cameras(state, batch_size)

        remaining_ms = 200.0            # sem nada em maos, so um poll ocioso
        if started_at is not None:
            remaining_ms = batch_window_ms - (time.monotonic() - started_at) * 1000.0
            if remaining_ms <= 0.0 or len(newest_by_camera) >= target:
                break

        try:
            if not dict(poller.poll(timeout=max(0.0, remaining_ms))):
                if started_at is not None:
                    break               # janela venceu sem completar o batch
                continue                # ninguem falando: segue esperando
        except zmq.ZMQError as exc:
            if exc.errno == zmq.ETERM:
                break
            raise

        # Drena tudo o que ja chegou, nao so uma mensagem
        while True:
            try:
                message = frame_socket.recv(zmq.NOBLOCK)
            except zmq.Again:
                break

            received_at = time.monotonic()

            try:
                header, payload = parse_frame(message)
                image_rgb = decode_payload(header, payload)
            except ProtocolError as exc:
                logger.warning("frame descartado: %s", exc)
                continue

            if state["last_client_id"] is not None and header.client_id != state["last_client_id"]:
                logger.warning("novo client_id %u (antes %u) - cliente reiniciou ou ha dois clientes",
                               header.client_id, state["last_client_id"])
            state["last_client_id"] = header.client_id
            state["cameras_seen"][header.camera_id] = received_at

            previous = newest_by_camera.get(header.camera_id)
            if previous is not None and previous.header.frame_id > header.frame_id:
                continue                # chegou fora de ordem: fica o mais novo

            # O batch NUNCA pode passar do batch_size com que o Hailo foi
            # configurado. Se ja esta cheio, uma camera nova fica para a
            # proxima volta (66 ms depois, no pior caso).
            if previous is None and len(newest_by_camera) >= batch_size:
                continue

            newest_by_camera[header.camera_id] = BatchItem(header, image_rgb, message, received_at)
            if started_at is None:
                started_at = received_at

        if len(newest_by_camera) >= expected_cameras(state, batch_size):
            break

    return list(newest_by_camera.values())


def decode_payload(header, payload: memoryview) -> np.ndarray:
    """Converte o payload recebido em uma imagem RGB uint8 HxWx3."""
    if header.format == FORMAT_JPEG:
        buffer = np.frombuffer(payload, dtype=np.uint8)
        image_bgr = cv2.imdecode(buffer, cv2.IMREAD_COLOR)
        if image_bgr is None:
            raise ProtocolError("JPEG invalido")
    elif header.format == FORMAT_BGR8:
        expected = header.width * header.height * 3
        if len(payload) != expected:
            raise ProtocolError(f"payload BGR de {len(payload)} bytes, esperado {expected}")
        image_bgr = np.frombuffer(payload, dtype=np.uint8).reshape(header.height, header.width, 3)
    else:
        raise ProtocolError(f"formato {header.format} desconhecido")

    # Os modelos do Hailo Model Zoo esperam RGB
    return cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)


def main() -> int:
    parser = argparse.ArgumentParser(description="Servidor de deteccao de pessoas pi_nit (Hailo-8L)")
    parser.add_argument("--hef", default=os.environ.get("PI_NIT_HEF", "/opt/pi_nit/models/yolov8s_h8l.hef"),
                        help="caminho do modelo .hef")
    parser.add_argument("--frame-port", type=int,
                        default=int(os.environ.get("PI_NIT_FRAME_PORT", DEFAULT_FRAME_PORT)))
    parser.add_argument("--result-port", type=int,
                        default=int(os.environ.get("PI_NIT_RESULT_PORT", DEFAULT_RESULT_PORT)))
    parser.add_argument("--bind", default=os.environ.get("PI_NIT_BIND", "0.0.0.0"),
                        help="interface de bind (use o ip da rede do veículo para restringir)")
    parser.add_argument("--confidence", type=float,
                        default=float(os.environ.get("PI_NIT_CONFIDENCE", 0.35)),
                        help="confianca minima devolvida")
    parser.add_argument("--classes", default=os.environ.get("PI_NIT_CLASSES", str(CLASS_PERSON)),
                        help="classes COCO devolvidas, separadas por virgula ('all' devolve todas)")
    parser.add_argument("--backend", choices=("hailort", "gstreamer", "cpu"),
                        default=os.environ.get("PI_NIT_BACKEND", "hailort"),
                        help="hailort (padrao) | gstreamer (precisa do Tappas) | cpu (teste no PC, sem Hailo)")
    parser.add_argument("--weights", default=os.environ.get("PI_NIT_WEIGHTS", ""),
                        help="backend cpu: arquivo .pt do YOLO")
    parser.add_argument("--device", default=os.environ.get("PI_NIT_DEVICE", "cpu"),
                        help="backend cpu: 'cpu' ou '0' para GPU")
    parser.add_argument("--viewer-port", type=int, default=int(os.environ.get("PI_NIT_VIEWER_PORT", 0)),
                        help="se > 0, republica frame+deteccoes num PUB para o pi_nit_viewer")
    parser.add_argument("--batch-size", type=int, default=int(os.environ.get("PI_NIT_BATCH_SIZE", 3)),
                        help="imagens por inferencia (1 por camera). O plano usa 3")
    parser.add_argument("--batch-window-ms", type=float,
                        default=float(os.environ.get("PI_NIT_BATCH_WINDOW_MS", 20)),
                        help="quanto esperar pelas outras cameras antes de inferir")
    parser.add_argument("--postprocess-so",
                        default=os.environ.get("PI_NIT_POSTPROCESS_SO", ""),
                        help="backend gstreamer: .so de pos-processamento do Tappas")
    parser.add_argument("--postprocess-config",
                        default=os.environ.get("PI_NIT_POSTPROCESS_CONFIG", ""),
                        help="backend gstreamer: json de labels do pos-processamento")
    parser.add_argument("--dummy", action="store_true",
                        default=os.environ.get("PI_NIT_DUMMY", "0") == "1",
                        help="nao usa o Hailo; devolve uma caixa fixa (teste do enlace ZMQ)")
    parser.add_argument("--log-level", default=os.environ.get("PI_NIT_LOG_LEVEL", "INFO"))
    arguments = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, arguments.log_level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        stream=sys.stdout,
    )

    class_filter = None
    if arguments.classes.strip().lower() != "all":
        class_filter = [int(value) for value in arguments.classes.split(",") if value.strip() != ""]

    if arguments.dummy:
        detector = DummyDetector(score_threshold=arguments.confidence)
    else:
        try:
            if arguments.backend == "gstreamer":
                from gstreamer_person_detector import (DEFAULT_POSTPROCESS_SO,
                                                       GStreamerPersonDetector)
                detector = GStreamerPersonDetector(
                    arguments.hef, arguments.confidence, class_filter,
                    postprocess_so_path=arguments.postprocess_so or DEFAULT_POSTPROCESS_SO,
                    postprocess_config_path=arguments.postprocess_config)
            elif arguments.backend == "cpu":
                from cpu_person_detector import CpuPersonDetector
                if not arguments.weights:
                    logger.error("backend cpu exige --weights (ex.: .../weights/yolov8x.pt)")
                    return 1
                detector = CpuPersonDetector(arguments.weights, arguments.confidence,
                                             class_filter, arguments.device)
            else:
                detector = HailoPersonDetector(arguments.hef, arguments.confidence, class_filter,
                                               arguments.batch_size)
        except Exception as exc:  # noqa: BLE001 - a causa completa vai para o journal
            logger.error("falha ao inicializar o backend '%s': %s", arguments.backend, exc,
                         exc_info=not isinstance(exc, DetectorError))
            return 1

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    context, frame_socket, result_socket = build_sockets(
        arguments.frame_port, arguments.result_port, arguments.bind,
        queue_depth=max(4, arguments.batch_size * 2))
    viewer_socket = build_viewer_socket(context, arguments.viewer_port, arguments.bind)

    poller = zmq.Poller()
    poller.register(frame_socket, zmq.POLLIN)

    frames = 0
    batches = 0
    detections_total = 0
    inference_ms_total = 0.0
    window_started_at = time.monotonic()
    state = {"last_client_id": None, "cameras_seen": {}}

    while running:
        batch = collect_batch(frame_socket, poller, arguments.batch_size,
                              arguments.batch_window_ms, state)

        if batch:
            headers = [item.header for item in batch]
            images = [item.image for item in batch]

            try:
                per_image = detector.infer_batch(images)
            except Exception as exc:  # noqa: BLE001 - um frame ruim nao pode matar o servico
                logger.error("erro na inferencia do batch de %d frame(s): %s", len(batch), exc)
                per_image = None

            if per_image is not None:
                batches += 1
                for item, (detections, inference_ms) in zip(batch, per_image):
                    queue_ms = (time.monotonic() - item.received_at) * 1000.0
                    result = build_result(item.header, detections, time.time(),
                                          inference_ms, queue_ms)

                    try:
                        result_socket.send(result, zmq.NOBLOCK)
                    except zmq.Again:
                        logger.warning("resultado do frame %d descartado: cliente nao esta lendo",
                                       item.header.frame_id)

                    if viewer_socket is not None:
                        # topico = camera, para o viewer poder filtrar por camera
                        try:
                            viewer_socket.send_multipart(
                                [f"cam{item.header.camera_id}".encode(), item.raw, result], zmq.NOBLOCK)
                        except zmq.Again:
                            pass   # ninguem olhando: descarta sem reclamar

                    frames += 1
                    detections_total += len(detections)
                    inference_ms_total += inference_ms

                del headers, images

        elapsed = time.monotonic() - window_started_at
        if elapsed >= 5.0:
            if frames > 0:
                per_batch = frames / batches if batches else 0.0
                logger.info("%.1f fps | %.1f ms por imagem | %.2f imagens por batch | %.2f pessoa(s)/frame",
                            frames / elapsed, inference_ms_total / frames,
                            per_batch, detections_total / frames)

                # Batch maior que o numero de cameras nao acelera nada: o
                # detector completa o batch repetindo a ultima imagem e joga
                # fora os resultados extras. Com 1 camera e batch 3, dois
                # tercos do acelerador trabalham a toa.
                if arguments.batch_size > 1 and per_batch < arguments.batch_size - 0.1:
                    logger.warning(
                        "batch de %d configurado mas chegam %.1f imagem(ns) por vez: "
                        "o acelerador processa %d copias e descarta %d. "
                        "Ajuste PI_NIT_BATCH_SIZE para %d (= numero de cameras).",
                        arguments.batch_size, per_batch, arguments.batch_size,
                        arguments.batch_size - round(per_batch), max(1, round(per_batch)))
            else:
                logger.info("nenhum frame recebido nos ultimos %.0f s", elapsed)
            frames = 0
            batches = 0
            detections_total = 0
            inference_ms_total = 0.0
            window_started_at = time.monotonic()

    logger.info("encerrando")
    detector.close()
    frame_socket.close()
    result_socket.close()
    if viewer_socket is not None:
        viewer_socket.close()
    context.term()

    return 0


if __name__ == "__main__":
    sys.exit(main())
