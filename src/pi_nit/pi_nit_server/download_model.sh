#!/usr/bin/env bash
#
# Baixa o modelo de deteccao compilado para o Hailo-8L.
#
#   sudo ./download_model.sh                 # yolov8s (padrao, melhor equilibrio)
#   sudo ./download_model.sh yolov8n         # mais rapido, um pouco menos preciso
#   sudo ./download_model.sh yolov8m         # mais preciso, nao fecha 15 fps no 8L
#   sudo ./download_model.sh yolov11s        # candidata forte: mAP mais alto que o
#                                             # yolov8s com throughput parecido - ver
#                                             # a comparacao em ../README.md#qual-
#                                             # versao-do-yolo-usar antes de trocar
#   sudo ./download_model.sh yolov10n        # so' se sobrar pouca folga de fps (mais
#                                             # cameras, por exemplo)
#
# Os HEFs vem do Hailo Model Zoo publico e ja trazem o NMS embarcado, que e' o
# que o backend hailort espera. A tabela completa de fps/mAP por versao (medida
# no chip, nao estimada) esta em:
#   https://github.com/hailo-ai/hailo_model_zoo/blob/master/docs/public_models/HAILO8L/HAILO8L_object_detection.rst

set -euo pipefail

MODEL="${1:-yolov8s}"
MODEL_DIR="${MODEL_DIR:-/opt/pi_nit/models}"
ZOO_VERSION="${ZOO_VERSION:-v2.14.0}"

DESTINATION="${MODEL_DIR}/${MODEL}_h8l.hef"

# Duas origens publicas conhecidas. A primeira e' o Model Zoo oficial; a
# segunda e' o bucket usado pelos exemplos do Raspberry Pi 5.
CANDIDATE_URLS=(
	"https://hailo-model-zoo.s3.eu-west-2.amazonaws.com/ModelZoo/Compiled/${ZOO_VERSION}/hailo8l/${MODEL}.hef"
	"https://hailo-csdata.s3.eu-west-2.amazonaws.com/resources/hefs/h8l_rpi/${MODEL}_h8l.hef"
)

mkdir -p "${MODEL_DIR}"

for url in "${CANDIDATE_URLS[@]}"; do
	echo "==> tentando ${url}"
	if curl -fL --progress-bar -o "${DESTINATION}.part" "${url}"; then
		mv "${DESTINATION}.part" "${DESTINATION}"
		chmod 0644 "${DESTINATION}"
		echo
		echo "modelo salvo em ${DESTINATION}"
		echo "aponte o PI_NIT_HEF de /etc/pi_nit/pi_nit_server.conf para esse caminho"
		exit 0
	fi
	rm -f "${DESTINATION}.part"
done

echo >&2
echo "erro: nenhuma origem respondeu." >&2
echo "Baixe o .hef manualmente (Hailo Model Zoo -> hailo8l -> ${MODEL}) e copie" >&2
echo "para ${DESTINATION}. Atencao: precisa ser a versao 'hailo8l', a de 'hailo8'" >&2
echo "nao roda no acelerador de 13 TOPS." >&2
exit 1
