"""Backend alternativo: inferencia via GStreamer (hailonet + hailofilter).

Este backend reaproveita exatamente o mesmo trecho de pipeline que ja esta
validado em reference/camera_pipeline.py (hailonet -> hailofilter ->
identity com callback, lendo as caixas com hailo.get_roi_from_buffer). A
unica diferenca e' a fonte: em vez de rtspsrc, um appsrc alimentado pelos
frames que chegam por ZMQ.

Quando usar cada backend:

  hailort (padrao)  - so precisa do HailoRT + bindings Python. E' o caminho
                      certo no Ubuntu 24.04 Server, onde o Tappas nao vem
                      pronto. Correspondencia frame->deteccao exata.

  gstreamer         - precisa do hailo-tappas-core instalado (os .so de
                      pos-processamento). Vale a pena se voce ja tem esse
                      ambiente funcionando e quer usar o mesmo caminho de
                      pos-processamento/labels de outros projetos.

Selecione com PI_NIT_BACKEND=gstreamer no /etc/pi_nit/pi_nit_server.conf.
"""

import logging
import queue
import time
from typing import List, Optional, Sequence, Tuple

import numpy as np

from pi_nit_protocol import CLASS_PERSON, Detection

logger = logging.getLogger("pi_nit.detector.gst")

# Caminho padrao do pos-processamento YOLO do Tappas em aarch64
DEFAULT_POSTPROCESS_SO = (
    "/usr/lib/aarch64-linux-gnu/hailo/tappas/post_processes/libyolo_hailortpp_postprocess.so"
)


class GStreamerPersonDetector:
    """Pipeline appsrc -> hailonet -> hailofilter, usado de forma sincrona.

    Empurramos um frame por vez e esperamos o callback correspondente, entao
    nao ha ambiguidade entre frame enviado e deteccao devolvida.
    """

    def __init__(self, hef_path: str, score_threshold: float = 0.35,
                 class_filter: Optional[Sequence[int]] = (CLASS_PERSON,),
                 postprocess_so_path: str = DEFAULT_POSTPROCESS_SO,
                 postprocess_config_path: str = "",
                 function_name: str = "filter",
                 timeout_s: float = 1.0):
        try:
            import gi

            gi.require_version("Gst", "1.0")
            from gi.repository import Gst

            import hailo
        except (ImportError, ValueError) as exc:
            raise RuntimeError(
                "backend gstreamer indisponivel: faltam os bindings GStreamer (python3-gi) "
                "e/ou o modulo 'hailo' do Tappas. Use PI_NIT_BACKEND=hailort."
            ) from exc

        self._Gst = Gst
        self._hailo = hailo
        self.score_threshold = score_threshold
        self.class_filter = set(class_filter) if class_filter else None
        self.input_width = 640
        self.input_height = 640
        self._timeout_s = timeout_s
        self._results: "queue.Queue[List[Detection]]" = queue.Queue(maxsize=4)

        Gst.init(None)

        pipeline_description = (
            f"appsrc name=src is-live=true do-timestamp=true format=time block=true "
            f"caps=video/x-raw,format=RGB,width={self.input_width},height={self.input_height},framerate=0/1 "
            f"! queue leaky=downstream max-size-buffers=2 max-size-bytes=0 max-size-time=0 "
            f"! videoconvert "
            f"! hailonet hef-path={hef_path} scheduling-algorithm=1 vdevice-group-id=pi_nit "
            f"! queue leaky=no max-size-buffers=2 max-size-bytes=0 max-size-time=0 "
            f"! hailofilter function-name={function_name} so-path={postprocess_so_path} "
            f"config-path={postprocess_config_path} "
            f"! identity name=callback_tap signal-handoffs=true "
            f"! fakesink sync=false async=false"
        )

        logger.info("pipeline: %s", pipeline_description)
        self._pipeline = Gst.parse_launch(pipeline_description)
        self._appsrc = self._pipeline.get_by_name("src")
        self._pipeline.get_by_name("callback_tap").connect("handoff", self._on_frame)

        bus = self._pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_bus_message)

        self._pipeline.set_state(Gst.State.PLAYING)
        logger.info("Hailo pronto via GStreamer: %s", hef_path)

    # ------------------------------------------------------------- callbacks

    def _on_bus_message(self, _bus, message) -> None:
        if message.type == self._Gst.MessageType.ERROR:
            error, debug = message.parse_error()
            logger.error("GStreamer ERROR: %s | %s", error.message, debug)
        elif message.type == self._Gst.MessageType.WARNING:
            warning, debug = message.parse_warning()
            logger.warning("GStreamer WARNING: %s | %s", warning.message, debug)

    def _on_frame(self, _identity, buffer) -> None:
        """Roda na thread do GLib. Nao pode bloquear nem levantar excecao."""
        try:
            roi = self._hailo.get_roi_from_buffer(buffer)
            detections: List[Detection] = []

            for raw in roi.get_objects_typed(self._hailo.HAILO_DETECTION):
                score = float(raw.get_confidence())
                if score < self.score_threshold:
                    continue

                class_id = _label_to_coco_id(raw.get_label())
                if self.class_filter is not None and class_id not in self.class_filter:
                    continue

                box = raw.get_bbox()
                # xmax = xmin + width: raw_bbox.xmax() pode nao existir em
                # algumas versoes do hailort (mesmo cuidado do camera_pipeline.py)
                xmin = box.xmin()
                ymin = box.ymin()
                detections.append(Detection(
                    x1=xmin * self.input_width,
                    y1=ymin * self.input_height,
                    x2=(xmin + box.width()) * self.input_width,
                    y2=(ymin + box.height()) * self.input_height,
                    score=score,
                    class_id=class_id,
                ))

            self._results.put_nowait(detections)
        except queue.Full:
            logger.warning("fila de resultados cheia, deteccao descartada")
        except Exception as exc:  # noqa: BLE001 - o callback nao pode propagar para o GLib
            logger.error("erro no callback de deteccao: %s", exc)

    # ------------------------------------------------------------ inferencia

    def infer_batch(self, images_rgb: List[np.ndarray]) -> List[Tuple[List[Detection], float]]:
        """O hailonet do Tappas faz o proprio batching interno, entao aqui
        empurramos uma imagem por vez. Sem ganho de batch - se o batch de 3
        for essencial, use o backend hailort."""
        return [self.infer(image) for image in images_rgb]

    def infer(self, image_rgb: np.ndarray) -> Tuple[List[Detection], float]:
        # Descarta resultados atrasados de frames anteriores (timeout)
        while not self._results.empty():
            self._results.get_nowait()

        data = np.ascontiguousarray(image_rgb).tobytes()
        buffer = self._Gst.Buffer.new_wrapped(data)

        started_at = time.perf_counter()
        flow_return = self._appsrc.emit("push-buffer", buffer)
        if flow_return != self._Gst.FlowReturn.OK:
            raise RuntimeError(f"appsrc push-buffer falhou: {flow_return}")

        try:
            detections = self._results.get(timeout=self._timeout_s)
        except queue.Empty as exc:
            raise RuntimeError(f"sem resposta do pipeline em {self._timeout_s:.1f} s") from exc

        return detections, (time.perf_counter() - started_at) * 1000.0

    def close(self) -> None:
        if self._pipeline is not None:
            self._pipeline.set_state(self._Gst.State.NULL)
            self._pipeline = None


# Labels COCO na ordem dos ids, para converter o texto devolvido pelo
# hailofilter no id numerico que o CARMEN espera (0 = pessoa).
_COCO_LABELS = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush",
]

_LABEL_TO_ID = {label: index for index, label in enumerate(_COCO_LABELS)}


def _label_to_coco_id(label: str) -> int:
    """-1 para rotulos fora do COCO (modelos customizados)."""
    return _LABEL_TO_ID.get(str(label).strip().lower(), -1)
