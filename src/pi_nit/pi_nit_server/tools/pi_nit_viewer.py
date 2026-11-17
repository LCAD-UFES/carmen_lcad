#!/usr/bin/env python3
"""Mostra na tela, em tempo real, as imagens das cameras com as deteccoes.

E' o item (c) do plano: rodando no Raspberry ligado na rede onde o log esta
tocando, mostra as 3 cameras ao vivo em um mosaico, com as caixas de pessoa
desenhadas e o timestamp de cada frame.

Nao interfere na deteccao: le de um socket PUB separado (porta 5562), que so
existe se o servidor for iniciado com PI_NIT_VIEWER_PORT=5562. Se o viewer
nao acompanhar, o PUB descarta - a cadeia CARMEN->Hailo->CARMEN nao sente.

    # no Raspberry (servidor com a visualizacao ligada)
    sudo sed -i 's/^PI_NIT_VIEWER_PORT=.*/PI_NIT_VIEWER_PORT=5562/' /etc/pi_nit/pi_nit_server.conf
    sudo systemctl restart pi_nit_server

    # na tela do Raspberry (ou de qualquer PC da rede)
    python3 pi_nit_viewer.py --host 127.0.0.1
    python3 pi_nit_viewer.py --host 192.168.1.20 --cameras 3,4,5
"""

import argparse
import os
import sys
import time
from datetime import datetime

import cv2
import numpy as np
import zmq

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from pi_nit_protocol import FORMAT_JPEG, class_name_and_color, parse_frame, parse_result

DEFAULT_VIEWER_PORT = 5562


def decode(header, payload):
    if header.format == FORMAT_JPEG:
        return cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
    return np.frombuffer(payload, dtype=np.uint8).reshape(header.height, header.width, 3).copy()


def draw(image, detections, camera_id, image_timestamp, inference_ms, fps):
    for detection in detections:
        top_left = (int(detection.x1), int(detection.y1))
        bottom_right = (int(detection.x2), int(detection.y2))
        nome, cor = class_name_and_color(detection.class_id)
        cv2.rectangle(image, top_left, bottom_right, cor, 2)
        cv2.putText(image, f"{nome} {detection.score:.2f}", (top_left[0], top_left[1] - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, cor, 1)

    # Faixa preta no topo com os dados que importam para depurar o enlace
    cv2.rectangle(image, (0, 0), (image.shape[1], 46), (0, 0, 0), -1)
    clock = datetime.fromtimestamp(image_timestamp).strftime("%H:%M:%S.%f")[:-3]
    cv2.putText(image, f"camera {camera_id}   {clock}", (8, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(image, f"{len(detections)} objeto(s)   {inference_ms:.0f} ms   {fps:.1f} fps", (8, 38),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    return image


def mosaic(frames, camera_order, tile_width):
    """Monta o mosaico lado a lado, na ordem pedida, com placeholder para o que
    ainda nao chegou (assim a janela nao muda de tamanho quando uma camera cai)."""
    tiles = []
    for camera_id in camera_order:
        image = frames.get(camera_id)
        if image is None:
            image = np.full((tile_width, tile_width, 3), 30, dtype=np.uint8)
            cv2.putText(image, f"camera {camera_id}", (10, tile_width // 2 - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (90, 90, 90), 2)
            cv2.putText(image, "sem imagem", (10, tile_width // 2 + 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (90, 90, 90), 2)
        else:
            scale = tile_width / image.shape[1]
            image = cv2.resize(image, (tile_width, int(image.shape[0] * scale)))

        tiles.append(image)

    height = max(tile.shape[0] for tile in tiles)
    padded = []
    for tile in tiles:
        if tile.shape[0] < height:
            tile = np.vstack([tile, np.zeros((height - tile.shape[0], tile.shape[1], 3), np.uint8)])
        padded.append(tile)

    return np.hstack(padded)


def main():
    parser = argparse.ArgumentParser(description="Viewer das cameras do pi_nit")
    parser.add_argument("--host", default="127.0.0.1", help="ip do Raspberry")
    parser.add_argument("--port", type=int, default=DEFAULT_VIEWER_PORT)
    parser.add_argument("--cameras", default="", help="ids a mostrar, ex.: 3,4,5 (vazio = todas)")
    parser.add_argument("--tile-width", type=int, default=480, help="largura de cada camera no mosaico")
    parser.add_argument("--save", default="", help="grava o mosaico em um .mp4")
    arguments = parser.parse_args()

    wanted = [int(value) for value in arguments.cameras.split(",") if value.strip()] or None

    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.setsockopt(zmq.RCVHWM, 4)
    socket.setsockopt(zmq.LINGER, 0)
    socket.setsockopt_string(zmq.SUBSCRIBE, "")
    socket.connect(f"tcp://{arguments.host}:{arguments.port}")

    print(f"conectado em tcp://{arguments.host}:{arguments.port}")
    print("aguardando frames... (q ou ESC para sair)")

    poller = zmq.Poller()
    poller.register(socket, zmq.POLLIN)

    frames = {}
    seen_order = []
    last_frame_at = {}
    fps = {}
    writer = None
    started_at = time.time()
    total = 0

    try:
        while True:
            events = dict(poller.poll(timeout=100))

            if socket in events:
                _topic, frame_message, result_message = socket.recv_multipart()

                header, payload = parse_frame(frame_message)
                result_header, detections = parse_result(result_message)

                camera_id = header.camera_id
                if wanted is not None and camera_id not in wanted:
                    continue

                now = time.time()
                if camera_id in last_frame_at:
                    interval = now - last_frame_at[camera_id]
                    if interval > 0:
                        # media movel simples, so para a legenda
                        fps[camera_id] = 0.7 * fps.get(camera_id, 0.0) + 0.3 * (1.0 / interval)
                last_frame_at[camera_id] = now

                image = decode(header, payload)
                if image is None:
                    continue

                frames[camera_id] = draw(image, detections, camera_id, header.timestamp,
                                         result_header["inference_ms"], fps.get(camera_id, 0.0))
                if camera_id not in seen_order:
                    seen_order.append(camera_id)
                    seen_order.sort()
                total += 1

            if not frames:
                continue

            canvas = mosaic(frames, wanted if wanted is not None else seen_order, arguments.tile_width)
            cv2.imshow("pi_nit - cameras", canvas)

            if arguments.save:
                if writer is None:
                    writer = cv2.VideoWriter(arguments.save, cv2.VideoWriter_fourcc(*"mp4v"),
                                             15.0, (canvas.shape[1], canvas.shape[0]))
                writer.write(canvas)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break

    except KeyboardInterrupt:
        pass

    elapsed = time.time() - started_at
    print(f"\n{total} frames em {elapsed:.1f} s ({total / elapsed if elapsed else 0:.1f} fps somados)")
    if total == 0:
        print("nada chegou. O servidor foi iniciado com PI_NIT_VIEWER_PORT diferente de 0?",
              file=sys.stderr)

    if writer is not None:
        writer.release()
    cv2.destroyAllWindows()
    socket.close()
    context.term()

    return 0 if total > 0 else 1


if __name__ == "__main__":
    sys.exit(main())
