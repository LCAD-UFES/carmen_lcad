"""Backend de CPU/GPU do PC - para testar TODA a cadeia sem o Raspberry.

Nao roda no Raspberry (o Pi 5 nao aguenta YOLO em CPU a 15 fps). Existe para
que o enlace ZMQ, o letterbox, o mapeamento de coordenadas, o tracking e a
publicacao IPC possam ser validados no PC, com deteccoes de pessoa DE VERDADE,
antes de o Hailo estar pronto.

Usa o ultralytics que ja esta instalado no venv do multiple_object_tracker:

    $CARMEN_HOME/data/pi_nit/venv/bin/python3 pi_nit_server.py \
        --backend cpu --weights $CARMEN_HOME/data/pi_nit/weights/yolov8x.pt

As classes sao as mesmas do COCO (0 = pessoa), entao o que o CARMEN recebe e'
indistinguivel do que vai receber do Hailo.
"""

import logging
import time
from typing import List, Optional, Sequence, Tuple

import numpy as np

from pi_nit_protocol import CLASS_PERSON, Detection

logger = logging.getLogger("pi_nit.detector.cpu")


class CpuPersonDetector:
    def __init__(self, weights: str, score_threshold: float = 0.35,
                 class_filter: Optional[Sequence[int]] = (CLASS_PERSON,),
                 device: str = "cpu"):
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                "ultralytics nao encontrado. Rode com o Python do venv do "
                "multiple_object_tracker, ou instale com 'pip install ultralytics'."
            ) from exc

        self.score_threshold = score_threshold
        self.class_filter = list(class_filter) if class_filter else None
        self.input_width = 640
        self.input_height = 640
        self._device = device

        logger.info("carregando %s em %s (pode demorar na primeira vez)", weights, device)
        self._model = YOLO(weights)
        logger.warning("BACKEND CPU/GPU DO PC: isto NAO e' o Hailo, e' so para teste")

    def infer(self, image_rgb: np.ndarray) -> Tuple[List[Detection], float]:
        return self.infer_batch([image_rgb])[0]

    def infer_batch(self, images_rgb: List[np.ndarray]) -> List[Tuple[List[Detection], float]]:
        if not images_rgb:
            return []

        started_at = time.perf_counter()

        results = self._model.predict(
            source=[image[:, :, ::-1] for image in images_rgb],   # ultralytics espera BGR
            imgsz=self.input_width,
            conf=self.score_threshold,
            classes=self.class_filter,
            device=self._device,
            verbose=False,
        )

        per_image_ms = (time.perf_counter() - started_at) * 1000.0 / len(images_rgb)

        per_image: List[Tuple[List[Detection], float]] = []
        for result in results:
            detections: List[Detection] = []
            for box in result.boxes:
                x1, y1, x2, y2 = [float(value) for value in box.xyxy[0].tolist()]
                detections.append(Detection(
                    x1=x1, y1=y1, x2=x2, y2=y2,
                    score=float(box.conf[0]),
                    class_id=int(box.cls[0]),
                ))
            per_image.append((detections, per_image_ms))

        return per_image

    def close(self) -> None:
        self._model = None
