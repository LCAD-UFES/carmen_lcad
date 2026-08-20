#!/usr/bin/env bash
#
# Instala o servico pi_nit_server no Raspberry Pi 5.
#
# Pre-requisito: HailoRT e o driver PCIe ja instalados e funcionando
# (`hailortcli fw-control identify` precisa responder). Veja o README.md.
#
# Uso:
#   sudo ./install.sh
#
# O script e' idempotente: pode ser rodado de novo para atualizar o codigo
# sem perder a configuracao de /etc/pi_nit/pi_nit_server.conf.

set -euo pipefail

INSTALL_DIR=/opt/pi_nit
APP_DIR="${INSTALL_DIR}/app"
VENV_DIR="${INSTALL_DIR}/venv"
MODEL_DIR="${INSTALL_DIR}/models"
CONFIG_DIR=/etc/pi_nit
SERVICE_USER=pi_nit
SOURCE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ ${EUID} -ne 0 ]]; then
	echo "erro: rode com sudo" >&2
	exit 1
fi

echo "==> instalando pacotes do sistema"
apt-get update -qq
apt-get install -y --no-install-recommends \
	python3 python3-venv python3-dev python3-pip \
	libzmq3-dev libgl1 libglib2.0-0

echo "==> criando usuario de servico '${SERVICE_USER}'"
if ! id -u "${SERVICE_USER}" >/dev/null 2>&1; then
	useradd --system --no-create-home --shell /usr/sbin/nologin "${SERVICE_USER}"
fi

echo "==> criando diretorios"
install -d -o "${SERVICE_USER}" -g "${SERVICE_USER}" "${INSTALL_DIR}" "${APP_DIR}" "${MODEL_DIR}"
install -d "${CONFIG_DIR}"

echo "==> copiando a aplicacao para ${APP_DIR}"
install -o "${SERVICE_USER}" -g "${SERVICE_USER}" -m 0644 \
	"${SOURCE_DIR}/pi_nit_server.py" \
	"${SOURCE_DIR}/pi_nit_protocol.py" \
	"${SOURCE_DIR}/hailo_person_detector.py" \
	"${SOURCE_DIR}/gstreamer_person_detector.py" \
	"${SOURCE_DIR}/cpu_person_detector.py" \
	"${SOURCE_DIR}/requirements.txt" \
	"${SOURCE_DIR}/README.md" \
	"${APP_DIR}/"
chmod 0755 "${APP_DIR}/pi_nit_server.py"

# Ferramentas de teste/visualizacao (rodam na tela do Raspberry)
install -d -o "${SERVICE_USER}" -g "${SERVICE_USER}" "${APP_DIR}/tools"
install -o "${SERVICE_USER}" -g "${SERVICE_USER}" -m 0755 \
	"${SOURCE_DIR}/tools/test_client.py" \
	"${SOURCE_DIR}/tools/pi_nit_viewer.py" \
	"${SOURCE_DIR}/tools/pi_nit_check.sh" \
	"${SOURCE_DIR}/tools/pi_nit_dummy.sh" \
	"${APP_DIR}/tools/"

# Atalho curto: 'pi_nit_check' de qualquer lugar, inclusive por ssh sem aspas
ln -sf "${APP_DIR}/tools/pi_nit_check.sh" /usr/local/bin/pi_nit_check

# --system-site-packages: o hailo_platform vem do wheel instalado no Python do
# sistema junto com o HailoRT, e precisa ser visivel de dentro do venv.
if [[ ! -d "${VENV_DIR}" ]]; then
	echo "==> criando o venv em ${VENV_DIR}"
	python3 -m venv --system-site-packages "${VENV_DIR}"
	chown -R "${SERVICE_USER}:${SERVICE_USER}" "${VENV_DIR}"
fi

echo "==> instalando dependencias Python"
"${VENV_DIR}/bin/pip" install --upgrade pip -q
"${VENV_DIR}/bin/pip" install -q -r "${SOURCE_DIR}/requirements.txt"

echo "==> conferindo o hailo_platform dentro do venv"
if "${VENV_DIR}/bin/python3" -c "import hailo_platform" 2>/dev/null; then
	echo "    ok: hailo_platform visivel"
else
	echo "    AVISO: hailo_platform NAO encontrado."
	echo "    O servico so vai rodar em modo --dummy ate voce instalar o wheel do"
	echo "    HailoRT (veja o README.md, secao 'Instalando o HailoRT')."
fi

# A configuracao nunca e' sobrescrita: o arquivo de referencia vai ao lado.
if [[ ! -f "${CONFIG_DIR}/pi_nit_server.conf" ]]; then
	echo "==> instalando a configuracao em ${CONFIG_DIR}/pi_nit_server.conf"
	install -m 0644 "${SOURCE_DIR}/pi_nit_server.conf" "${CONFIG_DIR}/pi_nit_server.conf"
else
	echo "==> configuracao existente preservada (referencia nova em ${CONFIG_DIR}/pi_nit_server.conf.new)"
	install -m 0644 "${SOURCE_DIR}/pi_nit_server.conf" "${CONFIG_DIR}/pi_nit_server.conf.new"
fi

echo "==> instalando a unit do systemd"
install -m 0644 "${SOURCE_DIR}/pi_nit_server.service" /etc/systemd/system/pi_nit_server.service
systemctl daemon-reload
systemctl enable pi_nit_server.service

echo
echo "instalacao concluida."
echo
# O hailo-all do Raspberry Pi OS ja instala modelos em /usr/share/hailo-models.
# So falta modelo se nao houver nem la' nem em /opt/pi_nit/models.
SHIPPED_MODEL=""
for candidate in yolov8s_h8l yolov6n_h8l yolox_s_leaky_h8l_rpi yolov5s_personface_h8l; do
	if [[ -f "/usr/share/hailo-models/${candidate}.hef" ]]; then
		SHIPPED_MODEL="/usr/share/hailo-models/${candidate}.hef"
		break
	fi
done

if [[ -f "${MODEL_DIR}/yolov8s_h8l.hef" ]]; then
	echo "  modelo: ${MODEL_DIR}/yolov8s_h8l.hef"
elif [[ -n "${SHIPPED_MODEL}" ]]; then
	echo "  modelo encontrado em ${SHIPPED_MODEL} (veio do hailo-all)"
	if grep -q "^PI_NIT_HEF=${SHIPPED_MODEL}$" "${CONFIG_DIR}/pi_nit_server.conf" 2>/dev/null; then
		echo "  PI_NIT_HEF ja aponta para ele"
	else
		echo "  aponte o PI_NIT_HEF para ele:"
		echo "    sudo sed -i 's|^PI_NIT_HEF=.*|PI_NIT_HEF=${SHIPPED_MODEL}|' ${CONFIG_DIR}/pi_nit_server.conf"
	fi
	echo
else
	echo "  FALTA O MODELO. Rode:  sudo ${SOURCE_DIR}/download_model.sh"
	echo
fi
echo "  iniciar:  sudo systemctl start pi_nit_server"
echo "  ver log:  journalctl -u pi_nit_server -f"
echo "  estado :  systemctl status pi_nit_server"
echo
