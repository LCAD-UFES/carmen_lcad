#!/bin/bash
# Deteccao ao vivo com a webcam do notebook, rodando no Raspberry.
#
# Abre a webcam, manda os frames para o Hailo do Pi e mostra as caixas em uma
# janela, com o fps, o tempo de inferencia e a ida e volta da rede. Nao precisa
# do central nem do param_daemon: fala ZMQ direto com o pi_nit_server.
#
#   ./webcam_ao_vivo.sh                      # Pi em 192.168.1.20, webcam 0, 15 fps
#   ./webcam_ao_vivo.sh 192.168.1.136        # outro IP
#   ./webcam_ao_vivo.sh 192.168.1.20 1 30    # webcam 1, 30 fps
#
# 'q' ou ESC fecha a janela.
set -euo pipefail

PI_HOST="${1:-192.168.1.20}"
WEBCAM="${2:-0}"
FPS="${3:-15}"

AQUI="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# O venv do multiple_object_tracker ja tem opencv e pyzmq; se nao existir,
# tenta o python do sistema.
PYTHON="${CARMEN_HOME:-${HOME}/carmen_lcad}/data/pi_nit/venv/bin/python3"
[[ -x "${PYTHON}" ]] || PYTHON=python3

if [[ ! -e "/dev/video${WEBCAM}" ]]; then
	echo "erro: /dev/video${WEBCAM} nao existe. Webcams disponiveis:" >&2
	ls /dev/video* 2>/dev/null >&2 || echo "  nenhuma" >&2
	exit 1
fi

echo "==> conferindo o Raspberry em ${PI_HOST}"
if ! ping -c 1 -W 2 "${PI_HOST}" >/dev/null 2>&1; then
	echo "erro: ${PI_HOST} nao responde ao ping" >&2
	exit 1
fi

# Se der para entrar por ssh, ja diz se o servico esta de pe - poupa o
# diagnostico de "nenhuma resposta" so' depois de abrir a janela.
if estado=$(ssh -o BatchMode=yes -o ConnectTimeout=4 "pi@${PI_HOST}" \
		'systemctl is-active pi_nit_server' 2>/dev/null); then
	echo "    servico pi_nit_server: ${estado}"
	if [[ "${estado}" != "active" ]]; then
		echo "    ligue com: ssh pi@${PI_HOST} 'sudo systemctl start pi_nit_server'" >&2
		exit 1
	fi
fi

echo "==> webcam ${WEBCAM} -> ${PI_HOST}:5560 a ${FPS} fps  ('q' fecha)"
exec "${PYTHON}" "${AQUI}/test_client.py" \
	--host "${PI_HOST}" \
	--camera "${WEBCAM}" \
	--fps "${FPS}" \
	--show
