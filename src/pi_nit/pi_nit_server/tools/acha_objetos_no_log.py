#!/usr/bin/env python3
"""Diz em que trechos de um log da IARA existem pessoas, carros, etc.

Os logs quase nao tem pedestre: no iara_20260730-3 sao 29 frames com pessoa em
1129 amostrados. Sem saber disso, a tela vazia no playback parece defeito do
detector. Este script varre as imagens de camera do log com o MESMO detector e
o MESMO corte de confianca do pi_nit_client_driver, e imprime os trechos em
segundos, prontos para o playback_control:

    ./playback_control -message 't 174 : 180' -autostart on -speed 1.0

Uso:
    python3 acha_objetos_no_log.py /dados/logs_iara/iara_20260730-3
    python3 acha_objetos_no_log.py <log> --classes 0,2 --confidence 0.3
"""

import argparse
import glob
import os

import numpy as np
from ultralytics import YOLO

# Nomes COCO das classes que interessam a percepcao do CARMEN
NOMES = {0: "pessoa", 1: "bicicleta", 2: "carro", 3: "moto",
         5: "onibus", 7: "caminhao"}

# As imagens do log sao BGR cru, sem cabecalho: o tamanho do arquivo e' a
# unica pista da resolucao.
RESOLUCOES = {640 * 480 * 3: (480, 640), 1280 * 720 * 3: (720, 1280)}


def carrega(caminho):
    dados = np.fromfile(caminho, dtype=np.uint8)
    forma = RESOLUCOES.get(dados.size)
    if forma is None:
        return None
    return dados.reshape(forma[0], forma[1], 3)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("log", help="pasta do log (a que contem o .txt_camera1)")
    parser.add_argument("--camera", default="camera1", help="qual pasta de camera varrer")
    parser.add_argument("--weights",
                        default=os.path.expanduser("$CARMEN_HOME/data/pi_nit/weights/yolov8n.pt"))
    parser.add_argument("--classes", default="0", help="classes COCO, separadas por virgula")
    parser.add_argument("--confidence", type=float, default=0.40,
                        help="mesmo padrao do pi_nit_client_driver")
    parser.add_argument("--passo", type=int, default=3, help="varre 1 imagem a cada N")
    parser.add_argument("--lote", type=int, default=8, help="imagens por inferencia")
    parser.add_argument("--intervalo", type=float, default=3.0,
                        help="segundos sem ninguem que separam dois trechos")
    argumentos = parser.parse_args()

    classes = [int(c) for c in argumentos.classes.split(",") if c.strip()]
    padrao = os.path.join(argumentos.log, f"*_{argumentos.camera}", "*", "*", "*.image")
    arquivos = sorted(glob.glob(padrao))[::argumentos.passo]

    if not arquivos:
        raise SystemExit(f"nenhuma imagem encontrada em {padrao}")

    print(f"{len(arquivos)} imagens amostradas | classes {classes} | conf {argumentos.confidence}\n")

    modelo = YOLO(argumentos.weights)
    achados = []

    for inicio in range(0, len(arquivos), argumentos.lote):
        grupo = arquivos[inicio:inicio + argumentos.lote]
        imagens, validos = [], []
        for caminho in grupo:
            imagem = carrega(caminho)
            if imagem is not None:
                imagens.append(imagem)
                validos.append(caminho)

        if not imagens:
            continue

        resultados = modelo.predict(source=imagens, imgsz=640, conf=argumentos.confidence,
                                    classes=classes, device=0, verbose=False)
        for caminho, resultado in zip(validos, resultados):
            if not len(resultado.boxes):
                continue
            instante = float(os.path.basename(caminho).split("_")[0])
            achados.append((
                instante,
                [int(caixa.cls[0]) for caixa in resultado.boxes],
                max(float(caixa.conf[0]) for caixa in resultado.boxes),
            ))

    if not achados:
        print("nenhum objeto dessas classes no log")
        return

    zero = float(os.path.basename(arquivos[0]).split("_")[0])
    print(f"{len(achados)} imagens com objeto (de {len(arquivos)} amostradas)\n")
    print(f"{'trecho para o playback_control':34s}{'dur':>7s}{'objetos':>9s}{'conf max':>10s}   classes")

    comeco, anterior = achados[0][0], achados[0][0]
    quantidades, confiancas, classes_vistas = [], [], set()

    def imprime_trecho():
        if len(quantidades) < 2:
            return
        nomes = ", ".join(sorted(NOMES.get(c, str(c)) for c in classes_vistas))
        print(f"  t {comeco - zero:6.0f} : {anterior - zero:<6.0f}"
              f"{'':14s}{anterior - comeco:5.1f}s{max(quantidades):9d}{max(confiancas):10.2f}   {nomes}")

    for instante, classes_do_frame, melhor in achados:
        if instante - anterior > argumentos.intervalo:
            imprime_trecho()
            comeco = instante
            quantidades, confiancas, classes_vistas = [], [], set()
        quantidades.append(len(classes_do_frame))
        confiancas.append(melhor)
        classes_vistas.update(classes_do_frame)
        anterior = instante

    imprime_trecho()


if __name__ == "__main__":
    main()
