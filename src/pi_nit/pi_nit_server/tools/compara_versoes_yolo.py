#!/usr/bin/env python3
"""Compara detecao de pessoa entre varias versoes/tamanhos do YOLO.

Roda cada peso nos MESMOS frames do video de teste (mesma ideia da tabela
que ja existe no README.md, so' que automatizada e com mais versoes) e
imprime uma tabela markdown com:

  - latencia por imagem (mediana, min, max) no hardware onde RODAR ESTE
    SCRIPT - CPU/GPU do PC, NUNCA o Hailo-8L do Raspberry;
  - deteccoes de pessoa, confianca media, quantas passam de --confidence.

Serve para responder "qual rede detecta melhor" (isso SIM viaja para o
Hailo: mAP e ranking entre modelos sao praticamente os mesmos entre chips,
so' a escala de tempo muda). NAO serve para prever o fps no Raspberry - pra
isso, veja a tabela oficial do Hailo Model Zoo em hailo8l/, linkada no
README.md deste modulo. Junte os dois: este script escolhe as 2-3
candidatas por deteccao, a tabela do Model Zoo decide qual delas fecha os
45 fps (3 cameras x 15 fps) no 8L.

Uso:
    python3 compara_versoes_yolo.py
    python3 compara_versoes_yolo.py --weights yolov8n.pt,yolov8s.pt,yolo11n.pt
    python3 compara_versoes_yolo.py --video /caminho/outro.avi --amostras 80
"""

import argparse
import statistics
import time

import cv2
import numpy as np

DEFAULT_WEIGHTS = "yolov8n.pt,yolov8s.pt,yolov8m.pt,yolov5su.pt,yolov10n.pt,yolov10s.pt,yolo11n.pt,yolo11s.pt"


def amostra_frames(caminho_video: str, quantidade: int):
    """Le 'quantidade' frames espacados uniformemente ao longo do video.

    Os MESMOS frames sao usados para todo peso testado - senao a diferenca
    de deteccao pode ser so' porque um modelo pegou um trecho mais facil.
    """
    captura = cv2.VideoCapture(caminho_video)
    if not captura.isOpened():
        raise SystemExit(f"nao consegui abrir {caminho_video}")

    total = int(captura.get(cv2.CAP_PROP_FRAME_COUNT))
    if total <= 0:
        raise SystemExit(f"{caminho_video} nao tem frames (ou o codec nao abriu)")

    passo = max(1, total // quantidade)
    indices = list(range(0, total, passo))[:quantidade]

    frames = []
    for indice in indices:
        captura.set(cv2.CAP_PROP_POS_FRAMES, indice)
        ok, frame = captura.read()
        if ok:
            frames.append(frame)
    captura.release()

    return frames


def testa_peso(nome_peso: str, frames, classes, confianca_min: float):
    """Roda um peso em todos os frames, um de cada vez (imgsz=640).

    Um por vez, nao em batch: e' o que isola a latencia do MODELO, que e' a
    parte que se traduz (em ranking, nao em valor absoluto) para o Hailo. O
    ganho de rodar em batch de 3 ja esta medido e documentado no README
    principal (94 ms -> 10 ms com yolov8n na GPU do PC).
    """
    from ultralytics import YOLO

    modelo = YOLO(nome_peso)

    tempos_ms = []
    deteccoes = []
    confiancas = []

    # uma chamada "fria" fora da medicao - a primeira sempre paga alocacao
    modelo.predict(source=frames[0], imgsz=640, conf=confianca_min, classes=classes,
                   device="cpu", verbose=False)

    for frame in frames:
        inicio = time.perf_counter()
        resultado = modelo.predict(source=frame, imgsz=640, conf=confianca_min,
                                   classes=classes, device="cpu", verbose=False)[0]
        tempos_ms.append((time.perf_counter() - inicio) * 1000.0)

        caixas = resultado.boxes
        deteccoes.append(len(caixas))
        confiancas.extend(float(c) for c in caixas.conf.tolist())

    parametros_m = None
    try:
        parametros_m = sum(p.numel() for p in modelo.model.parameters()) / 1e6
    except Exception:
        pass

    return {
        "peso": nome_peso,
        "parametros_m": parametros_m,
        "ms_mediana": statistics.median(tempos_ms),
        "ms_min": min(tempos_ms),
        "ms_max": max(tempos_ms),
        "deteccoes_total": sum(deteccoes),
        "deteccoes_por_frame": sum(deteccoes) / len(frames),
        "conf_media": statistics.mean(confiancas) if confiancas else 0.0,
        "acima_040": sum(1 for c in confiancas if c >= 0.40),
    }


def imprime_tabela(resultados, frames_testados: int, confianca_min: float):
    print(f"\n{frames_testados} frames testados | conf minima {confianca_min} | "
          f"1 imagem por vez, CPU deste PC (nao e' o Hailo)\n")

    cabecalho = (f"{'peso':22s}{'params M':>10s}{'ms/img (mediana)':>18s}"
                 f"{'min':>8s}{'max':>8s}{'deteccoes':>11s}{'conf media':>12s}{'>=0.40':>8s}")
    print(cabecalho)
    print("-" * len(cabecalho))

    for r in sorted(resultados, key=lambda x: x["ms_mediana"]):
        params = f"{r['parametros_m']:.1f}" if r["parametros_m"] else "?"
        print(f"{r['peso']:22s}{params:>10s}{r['ms_mediana']:>18.1f}"
              f"{r['ms_min']:>8.1f}{r['ms_max']:>8.1f}{r['deteccoes_total']:>11d}"
              f"{r['conf_media']:>12.3f}{r['acima_040']:>8d}")

    print("\nLembrete: a coluna de tempo e' CPU do PC, so' serve para comparar os "
          "pesos ENTRE si (o mais rapido aqui tende a ser o mais rapido no Hailo "
          "tambem). O tempo REAL no Raspberry sai da tabela oficial do Hailo Model "
          "Zoo (hailo8l/*.hef), nao daqui.")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--video", default="../data/pi_nit/pedestres.avi",
                        help="video de teste (padrao: o mesmo do COMO_TESTAR.md)")
    parser.add_argument("--weights", default=DEFAULT_WEIGHTS,
                        help="pesos .pt separados por virgula (baixam sozinhos na 1a vez)")
    parser.add_argument("--classes", default="0", help="classes COCO, separadas por virgula")
    parser.add_argument("--confidence", type=float, default=0.25,
                        help="confianca minima (0.25 e' o que o ND/pi_nit usam por padrao)")
    parser.add_argument("--amostras", type=int, default=60,
                        help="quantos frames do video usar (mesmos para todo peso)")
    argumentos = parser.parse_args()

    classes = [int(c) for c in argumentos.classes.split(",") if c.strip()]
    pesos = [p.strip() for p in argumentos.weights.split(",") if p.strip()]

    print(f"lendo {argumentos.amostras} frames de {argumentos.video}...")
    frames = amostra_frames(argumentos.video, argumentos.amostras)
    print(f"{len(frames)} frames carregados\n")

    resultados = []
    for peso in pesos:
        print(f"==> {peso}")
        try:
            resultados.append(testa_peso(peso, frames, classes, argumentos.confidence))
        except Exception as exc:  # noqa: BLE001 - um peso que falha nao derruba os outros
            print(f"    falhou: {exc}")

    if not resultados:
        raise SystemExit("nenhum peso terminou com sucesso")

    imprime_tabela(resultados, len(frames), argumentos.confidence)


if __name__ == "__main__":
    main()
