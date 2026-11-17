#!/usr/bin/env python3
"""Cliente de teste do pi_nit - imita o lado do CARMEN, sem precisar do CARMEN.

Serve para validar o Raspberry sozinho: manda uma imagem (arquivo, webcam ou
sintetica) na mesma taxa que o CARMEN mandaria e imprime as deteccoes.

Exemplos:
    # servidor rodando na propria maquina, imagem sintetica
    python3 test_client.py --host 127.0.0.1

    # uma foto com pessoas, apontando para o Raspberry
    python3 test_client.py --host 192.168.1.20 --image /tmp/pessoas.jpg

    # webcam local a 15 fps, com janela
    python3 test_client.py --host 192.168.1.20 --camera 0 --show
"""

import argparse
import os
import struct
import sys
import time

import cv2
import numpy as np
import zmq

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from pi_nit_protocol import (class_name_and_color,
                             DEFAULT_FRAME_PORT, DEFAULT_RESULT_PORT, FORMAT_JPEG,
                             FRAME_HEADER_STRUCT, FRAME_MAGIC, INFERENCE_SIZE,
                             PROTOCOL_VERSION, parse_result)


def letterbox(image, size=INFERENCE_SIZE):
    """Mesmo letterbox do cliente C++: escala mantendo proporcao e preenche com cinza."""
    scale = min(size / image.shape[1], size / image.shape[0])
    new_width = min(int(image.shape[1] * scale + 0.5), size)
    new_height = min(int(image.shape[0] * scale + 0.5), size)
    pad_x = (size - new_width) // 2
    pad_y = (size - new_height) // 2

    canvas = np.full((size, size, 3), 114, dtype=np.uint8)
    canvas[pad_y:pad_y + new_height, pad_x:pad_x + new_width] = cv2.resize(
        image, (new_width, new_height), interpolation=cv2.INTER_LINEAR)

    return canvas, scale, pad_x, pad_y


def crop_roi(image, roi_top, roi_bottom):
    """Mesmo crop_roi() do cliente C++ (pi_nit_client_driver.cpp): corta o topo
    e/ou a base da imagem ANTES do letterbox. < 1 e' fracao da altura, >= 1 e'
    pixel, 0 desliga. Devolve a imagem cortada e crop_y (linhas cortadas do
    topo), que o cliente soma de volta na coordenada Y da deteccao.

    So' existe aqui para o -roi_top/-roi_bottom poder ser testado no
    simulador (backend cpu, sem Hailo) antes de ter acesso ao CARMEN para
    compilar e testar o lado C++ de verdade."""
    if roi_top <= 0.0 and roi_bottom <= 0.0:
        return image, 0

    rows = image.shape[0]
    top = int(roi_top * rows + 0.5) if roi_top < 1.0 else int(roi_top)
    bottom = int(roi_bottom * rows + 0.5) if roi_bottom < 1.0 else int(roi_bottom)

    if (top + bottom) >= rows:
        return image, 0

    return image[top:rows - bottom, :], top


def build_frame(image_640, frame_id, client_id, camera_id, timestamp, jpeg_quality):
    ok, encoded = cv2.imencode(".jpg", image_640, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
    if not ok:
        raise RuntimeError("falha ao codificar JPEG")
    payload = encoded.tobytes()

    header = FRAME_HEADER_STRUCT.pack(
        FRAME_MAGIC, PROTOCOL_VERSION, FORMAT_JPEG,
        INFERENCE_SIZE, INFERENCE_SIZE,
        client_id, camera_id, frame_id, timestamp, len(payload), 0,
    )

    return header + payload


def synthetic_image():
    """Imagem de teste: um retangulo claro sobre fundo escuro (nao e' uma pessoa
    de verdade; serve para o modo --dummy e para medir latencia da rede)."""
    image = np.full((480, 640, 3), 40, dtype=np.uint8)
    cv2.rectangle(image, (260, 120), (380, 440), (200, 200, 200), -1)
    return image


def main():
    parser = argparse.ArgumentParser(description="Cliente de teste do pi_nit")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--frame-port", type=int, default=DEFAULT_FRAME_PORT)
    parser.add_argument("--result-port", type=int, default=DEFAULT_RESULT_PORT)
    parser.add_argument("--image", help="arquivo de imagem a enviar em loop")
    parser.add_argument("--video", help="arquivo de video a enviar (use --loop para repetir)")
    parser.add_argument("--loop", action="store_true", help="reinicia o video ao chegar no fim")
    parser.add_argument("--camera", type=int, help="indice da webcam local")
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--frames", type=int, default=0, help="0 = infinito")
    parser.add_argument("--jpeg-quality", type=int, default=80)
    parser.add_argument("--camera-id", type=int, default=3,
                        help="id da camera; com --simulate-cameras vira o primeiro de N ids")
    parser.add_argument("--simulate-cameras", type=int, default=1,
                        help="manda o mesmo frame como se fossem N cameras (testa o batch)")
    parser.add_argument("--roi-top", type=float, default=0.0,
                        help="corta o topo da imagem antes do letterbox (mesma convencao do "
                             "-roi_top do cliente C++): <1 fracao da altura, >=1 pixel, 0 desliga")
    parser.add_argument("--roi-bottom", type=float, default=0.0,
                        help="idem, cortando a base (-roi_bottom do cliente C++)")
    parser.add_argument("--show", action="store_true")
    arguments = parser.parse_args()

    context = zmq.Context()

    # Mesmas opcoes do cliente C++: um frame em voo por camera e ZMQ_DONTWAIT.
    # Nada de ZMQ_CONFLATE - ele guarda "a ultima mensagem" do socket inteiro
    # e comeria os frames das outras cameras.
    frame_socket = context.socket(zmq.PUSH)
    frame_socket.setsockopt(zmq.SNDHWM, max(1, arguments.simulate_cameras))
    frame_socket.setsockopt(zmq.LINGER, 0)
    frame_socket.connect(f"tcp://{arguments.host}:{arguments.frame_port}")

    result_socket = context.socket(zmq.PULL)
    result_socket.setsockopt(zmq.RCVHWM, 4)
    result_socket.setsockopt(zmq.LINGER, 0)
    result_socket.connect(f"tcp://{arguments.host}:{arguments.result_port}")

    print(f"conectado em {arguments.host} ({arguments.frame_port}/{arguments.result_port})")

    capture = None
    if arguments.camera is not None:
        capture = cv2.VideoCapture(arguments.camera)
    elif arguments.video:
        capture = cv2.VideoCapture(arguments.video)
    if capture is not None and not capture.isOpened():
        print(f"erro: nao consegui abrir {arguments.video or arguments.camera}", file=sys.stderr)
        return 1

    still_image = cv2.imread(arguments.image) if arguments.image else None
    if arguments.image and still_image is None:
        print(f"erro: nao consegui abrir {arguments.image}", file=sys.stderr)
        return 1

    camera_ids = [arguments.camera_id + offset for offset in range(max(1, arguments.simulate_cameras))]
    if len(camera_ids) > 1:
        print(f"simulando {len(camera_ids)} cameras: ids {camera_ids}")
    if arguments.roi_top > 0.0 or arguments.roi_bottom > 0.0:
        print(f"roi_top={arguments.roi_top} roi_bottom={arguments.roi_bottom} "
              f"(recorte antes do letterbox, igual ao cliente C++)")

    client_id = struct.unpack("<I", os.urandom(4))[0] or 1
    pending = {}
    frame_id = 0
    sent = 0
    dropped = 0
    received = 0
    period = 1.0 / arguments.fps if arguments.fps > 0 else 0.0
    started_at = time.time()

    # Com a janela aberta, imprimir cada frame atrapalha mais do que ajuda: o
    # numero que interessa vai desenhado na propria imagem, e o terminal so'
    # resume uma vez por segundo.
    janela_started_at = time.time()
    janela_frames = 0
    fps_retorno = 0.0
    encerrar = False

    try:
        while arguments.frames == 0 or sent < arguments.frames:
            loop_started_at = time.time()

            if capture is not None:
                ok, image = capture.read()
                if not ok:
                    if arguments.loop:
                        capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
                        ok, image = capture.read()
                    if not ok:
                        print("fim do video/webcam")
                        break
            elif still_image is not None:
                image = still_image
            else:
                image = synthetic_image()

            cropped, crop_y = crop_roi(image, arguments.roi_top, arguments.roi_bottom)
            image_640, scale, pad_x, pad_y = letterbox(cropped)

            # O mesmo frame vai como se viesse de N cameras: e' assim que se
            # exercita o batch do servidor sem ter 3 cameras de verdade.
            for camera_id in camera_ids:
                frame_id += 1
                try:
                    frame_socket.send(build_frame(image_640, frame_id, client_id,
                                                  camera_id, time.time(),
                                                  arguments.jpeg_quality), zmq.NOBLOCK)
                except zmq.Again:
                    dropped += 1   # fila cheia: o servidor esta atrasado, descarta
                    continue
                pending[frame_id] = (time.time(), scale, pad_x, pad_y, image, crop_y)
                sent += 1

            # Le tudo que ja voltou
            while True:
                try:
                    message = result_socket.recv(zmq.NOBLOCK)
                except zmq.Again:
                    break

                header, detections = parse_result(message)
                entry = pending.pop(header["frame_id"], None)
                if entry is None:
                    continue

                sent_at, scale, pad_x, pad_y, original, crop_y = entry
                round_trip_ms = (time.time() - sent_at) * 1000.0
                received += 1

                # Desfaz letterbox e depois o recorte de ROI (mesma ordem e
                # mesma formula do process_pending_results() em
                # pi_nit_client_driver.cpp): X nao muda com o ROI, so' Y soma
                # o crop_y de volta.
                boxes = []
                for detection in detections:
                    boxes.append((
                        (detection.x1 - pad_x) / scale, (detection.y1 - pad_y) / scale + crop_y,
                        (detection.x2 - pad_x) / scale, (detection.y2 - pad_y) / scale + crop_y,
                        detection.score, detection.class_id,
                    ))

                janela_frames += 1
                agora = time.time()
                if agora - janela_started_at >= 1.0:
                    fps_retorno = janela_frames / (agora - janela_started_at)
                    janela_started_at = agora
                    janela_frames = 0
                    if arguments.show:
                        print(f"{fps_retorno:4.1f} fps | rtt {round_trip_ms:5.1f} ms | "
                              f"hailo {header['inference_ms']:5.1f} ms | {len(boxes)} objeto(s)")

                if not arguments.show:
                    print(f"frame {header['frame_id']:5d} | rtt {round_trip_ms:6.1f} ms | "
                          f"hailo {header['inference_ms']:5.1f} ms | fila {header['queue_ms']:5.1f} ms | "
                          f"{len(boxes)} deteccao(oes)")
                    for x1, y1, x2, y2, score, class_id in boxes:
                        nome, _cor = class_name_and_color(class_id)
                        print(f"        {nome} conf {score:.2f} "
                              f"[{x1:.0f},{y1:.0f} -> {x2:.0f},{y2:.0f}]")
                else:
                    canvas = original.copy()
                    for x1, y1, x2, y2, score, class_id in boxes:
                        nome, cor = class_name_and_color(class_id)
                        cv2.rectangle(canvas, (int(x1), int(y1)), (int(x2), int(y2)), cor, 2)
                        cv2.putText(canvas, f"{nome} {score:.2f}", (int(x1), int(y1) - 6),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, cor, 2)

                    # Faixa preta no topo: o que importa para julgar o enlace
                    cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 26), (0, 0, 0), -1)
                    cv2.putText(canvas, f"{arguments.host}   {fps_retorno:4.1f} fps   "
                                        f"hailo {header['inference_ms']:4.1f} ms   "
                                        f"rtt {round_trip_ms:5.1f} ms   {len(boxes)} objeto(s)",
                                (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

                    cv2.imshow("pi_nit - deteccao ao vivo (q para sair)", canvas)
                    if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                        encerrar = True

            if encerrar:
                break

            # Esquece frames sem resposta ha mais de 2 s
            now = time.time()
            for stale in [key for key, value in pending.items() if now - value[0] > 2.0]:
                del pending[stale]

            elapsed = time.time() - loop_started_at
            if period > elapsed:
                time.sleep(period - elapsed)

    except KeyboardInterrupt:
        pass

    if arguments.show:
        cv2.destroyAllWindows()

    total = time.time() - started_at
    print(f"\nenviados {sent} | descartados no envio {dropped} | recebidos {received} | "
          f"{received / total if total else 0:.1f} fps de retorno")
    if received == 0:
        print("nenhuma resposta: confira se o pi_nit_server esta rodando e se as portas "
              "5560/5561 estao liberadas no firewall", file=sys.stderr)

    frame_socket.close()
    result_socket.close()
    context.term()

    return 0 if received > 0 else 1


if __name__ == "__main__":
    sys.exit(main())
