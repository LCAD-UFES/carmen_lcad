"""Inferencia de deteccao de pessoas no Hailo-8L (AI HAT+ 13 TOPS).

Usa a API Python do HailoRT (hailo_platform) diretamente sobre o frame
640x640 que chegou por ZMQ - sem GStreamer, porque aqui a imagem nao vem de
uma camera local e sim da rede, e queremos correspondencia exata entre frame
enviado e deteccao devolvida.

O HEF precisa ter o NMS na propria chip (e' o caso dos modelos compilados do
Hailo Model Zoo, ex.: yolov8s_h8l.hef). Nesse formato a saida ja vem como
uma lista por classe de caixas [y_min, x_min, y_max, x_max, score]
normalizadas em [0, 1], que so precisam ser multiplicadas pela resolucao.

Modo --dummy: nao usa o Hailo, devolve uma caixa fixa. Serve para validar
todo o caminho ZMQ (CARMEN <-> Raspberry) antes de o acelerador estar pronto.
"""

import logging
import time
from typing import List, Optional, Sequence, Tuple

import numpy as np

from pi_nit_protocol import CLASS_PERSON, Detection

logger = logging.getLogger("pi_nit.detector")


class DetectorError(Exception):
    pass


class HailoPersonDetector:
    """Mantem o dispositivo configurado e ativo entre chamadas.

    Reconfigurar a rede a cada frame custaria dezenas de milissegundos, entao
    o network group e' ativado uma unica vez no __init__ e reaproveitado.
    """

    def __init__(self, hef_path: str, score_threshold: float = 0.35,
                 class_filter: Optional[Sequence[int]] = (CLASS_PERSON,),
                 batch_size: int = 3):
        self.hef_path = hef_path
        self.score_threshold = score_threshold
        self.class_filter = set(class_filter) if class_filter else None
        self.batch_size = max(1, batch_size)
        self.input_width = 0
        self.input_height = 0

        self._stack = None
        self._pipeline = None
        self._input_name = None
        self._output_name = None

        self._open()

    # ------------------------------------------------------------------ setup

    def _open(self) -> None:
        try:
            from contextlib import ExitStack

            from hailo_platform import (HEF, ConfigureParams, FormatType,
                                        HailoStreamInterface, InferVStreams,
                                        InputVStreamParams, OutputVStreamParams,
                                        VDevice)
        except ImportError as exc:
            raise DetectorError(
                "hailo_platform nao encontrado. Instale o HailoRT e as bindings Python "
                "(veja README.md, secao 'Instalando o HailoRT'). Para testar sem o "
                "acelerador use --dummy."
            ) from exc

        self._stack = ExitStack()

        try:
            hef = HEF(self.hef_path)
            device = self._stack.enter_context(VDevice())

            configure_params = ConfigureParams.create_from_hef(hef, interface=HailoStreamInterface.PCIe)

            # Batch: as 3 cameras vao ao acelerador em uma unica transferencia
            # PCIe, em vez de tres. O HEF precisa suportar o batch pedido; se
            # nao suportar, caimos para 1 e seguimos (so mais lento).
            for params in configure_params.values():
                try:
                    params.batch_size = self.batch_size
                except Exception as exc:  # noqa: BLE001 - varia com a versao do HailoRT
                    logger.warning("batch_size=%d nao aceito (%s), usando 1", self.batch_size, exc)
                    self.batch_size = 1

            network_group = device.configure(hef, configure_params)[0]

            input_info = hef.get_input_vstream_infos()[0]
            output_info = hef.get_output_vstream_infos()[0]
            self._input_name = input_info.name
            self._output_name = output_info.name

            self.input_height, self.input_width = int(input_info.shape[0]), int(input_info.shape[1])

            input_params = _make_vstream_params(InputVStreamParams, network_group, FormatType.UINT8)
            output_params = _make_vstream_params(OutputVStreamParams, network_group, FormatType.FLOAT32)

            self._pipeline = self._stack.enter_context(
                InferVStreams(network_group, input_params, output_params))

            # Em HailoRT com scheduler habilitado a ativacao e' automatica e
            # activate() lanca excecao; nesse caso seguimos sem ativar.
            try:
                self._stack.enter_context(network_group.activate(network_group.create_params()))
            except Exception as exc:  # noqa: BLE001 - depende da versao do HailoRT
                logger.info("network_group.activate() nao usado (scheduler ativo): %s", exc)

        except Exception:
            self._stack.close()
            self._stack = None
            raise

        logger.info("Hailo pronto: %s | entrada %dx%d | saida '%s'",
                    self.hef_path, self.input_width, self.input_height, self._output_name)

    def close(self) -> None:
        if self._stack is not None:
            self._stack.close()
            self._stack = None
            self._pipeline = None

    # -------------------------------------------------------------- inferencia

    def infer(self, image_rgb: np.ndarray) -> Tuple[List[Detection], float]:
        """Roda a rede em uma imagem RGB uint8 HxWx3."""
        return self.infer_batch([image_rgb])[0]

    def infer_batch(self, images_rgb: List[np.ndarray]) -> List[Tuple[List[Detection], float]]:
        """Roda a rede em ate batch_size imagens de uma vez.

        Devolve, na mesma ordem da entrada, (deteccoes em pixels, tempo em ms).
        O tempo e' o do batch inteiro dividido pelo numero de imagens - e' o
        numero que interessa para saber se fecha 15 fps por camera.
        """
        if self._pipeline is None:
            raise DetectorError("detector fechado")
        if not images_rgb:
            return []

        # O HailoRT espera exatamente o batch com que a rede foi configurada.
        # Se so chegaram 2 das 3 cameras, completamos repetindo a ultima
        # imagem e descartamos o resultado extra - custa alguns ms e evita
        # ter que reconfigurar a rede a cada variacao.
        wanted = len(images_rgb)
        padded = images_rgb
        if wanted < self.batch_size:
            padded = images_rgb + [images_rgb[-1]] * (self.batch_size - wanted)

        started_at = time.perf_counter()
        raw = self._pipeline.infer({self._input_name: np.stack(padded, axis=0)})
        elapsed_ms = (time.perf_counter() - started_at) * 1000.0
        per_image_ms = elapsed_ms / wanted

        output = raw[self._output_name]

        results: List[Tuple[List[Detection], float]] = []
        for index, image in enumerate(images_rgb):
            height, width = image.shape[0], image.shape[1]
            results.append((self._parse_nms_output(output, width, height, index), per_image_ms))

        return results

    def _parse_nms_output(self, raw, width: int, height: int, batch_index: int = 0) -> List[Detection]:
        """Converte a saida NMS-por-classe do Hailo em Detection (pixels).

        Formatos aceitos (variam com a versao do HailoRT):
          - lista[batch] de lista[classe] de array (n, 5)
          - ndarray (batch, classes, max_det, 5)
        Cada caixa e' [y_min, x_min, y_max, x_max, score] normalizada.
        """
        per_class = raw[batch_index] if len(raw) > batch_index else []

        if isinstance(per_class, np.ndarray) and per_class.ndim == 1:
            # Saida crua (sem NMS na chip): nao suportada de proposito - decodificar
            # YOLO na CPU do Pi derrubaria os 15 fps.
            raise DetectorError(
                "o HEF nao tem NMS embarcado (saida crua). Use um modelo do Hailo Model "
                "Zoo compilado com NMS, ex.: yolov8s_h8l.hef"
            )

        detections: List[Detection] = []

        for class_id, boxes in enumerate(per_class):
            if boxes is None or len(boxes) == 0:
                continue
            if self.class_filter is not None and class_id not in self.class_filter:
                continue

            for box in np.asarray(boxes).reshape(-1, 5):
                score = float(box[4])
                if score < self.score_threshold:
                    continue

                y1, x1, y2, x2 = float(box[0]), float(box[1]), float(box[2]), float(box[3])
                detections.append(Detection(
                    x1=max(0.0, x1 * width),
                    y1=max(0.0, y1 * height),
                    x2=min(float(width), x2 * width),
                    y2=min(float(height), y2 * height),
                    score=score,
                    class_id=class_id,
                ))

        return detections


def _make_vstream_params(params_class, network_group, format_type):
    """InputVStreamParams.make mudou de assinatura entre versoes do HailoRT."""
    try:
        return params_class.make(network_group, format_type=format_type)
    except TypeError:
        return params_class.make(network_group, quantized=False, format_type=format_type)


class DummyDetector:
    """Substituto do Hailo para testar o caminho ZMQ sem o acelerador."""

    def __init__(self, score_threshold: float = 0.35, **_ignored):
        self.score_threshold = score_threshold
        self.input_width = 640
        self.input_height = 640
        logger.warning("MODO DUMMY: nenhuma inferencia real esta sendo feita")

    def infer(self, image_rgb: np.ndarray) -> Tuple[List[Detection], float]:
        return self.infer_batch([image_rgb])[0]

    def infer_batch(self, images_rgb: List[np.ndarray]) -> List[Tuple[List[Detection], float]]:
        # Imita o Hailo: o batch inteiro custa quase o mesmo que uma imagem
        time.sleep(0.02)
        per_image_ms = 20.0 / max(1, len(images_rgb))

        results = []
        for image in images_rgb:
            height, width = image.shape[0], image.shape[1]
            results.append(([Detection(
                x1=width * 0.40, y1=height * 0.25,
                x2=width * 0.60, y2=height * 0.90,
                score=0.99, class_id=CLASS_PERSON,
            )], per_image_ms))

        return results

    def close(self) -> None:
        pass
