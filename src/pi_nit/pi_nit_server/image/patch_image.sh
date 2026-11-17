#!/usr/bin/env bash
#
# Edita uma imagem JA GERADA, direto no PC, sem gravar no cartao e sem
# precisar ligar o Raspberry. Serve para ajustar modelo, IP, senha ou
# qualquer chave da configuracao sem refazer o build (que leva ~40 min).
#
#   sudo ./patch_image.sh --list                     # mostra o que tem la' dentro
#   sudo ./patch_image.sh                            # conserta o PI_NIT_HEF sozinho
#   sudo ./patch_image.sh --hef /usr/share/hailo-models/yolov6n_h8l.hef
#   sudo ./patch_image.sh --ip 192.168.1.30 --hostname pi-nit-2
#   sudo ./patch_image.sh --set PI_NIT_VIEWER_PORT=5562 --set PI_NIT_FPS=10
#
# Sem --image, usa a imagem mais recente de /var/tmp/pi_nit_image.
#
set -euo pipefail

WORK_DIR="${WORK_DIR:-/var/tmp/pi_nit_image}"
IMAGE=""
NEW_HEF=""
NEW_IP=""
NEW_PREFIX=""
NEW_GATEWAY=""
NEW_HOSTNAME=""
NEW_PASSWORD=""
LIST_ONLY=0
declare -a SET_KEYS=()

# Detectores aceitos, em ordem de preferencia. Precisa ser _h8l (Hailo-8L de
# 13 TOPS; um _h8 e' do Hailo-8 de 26 TOPS e nao roda) e precisa ser deteccao,
# nao pose nem segmentacao - o parser espera a saida NMS de deteccao.
PREFERRED_MODELS=(yolov8s_h8l yolov6n_h8l yolox_s_leaky_h8l_rpi yolov5s_personface_h8l)

usage()
{
	sed -n '2,18p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
	cat <<EOF

Opcoes:
  --image <arq.img>    imagem a editar (padrao: a mais recente em ${WORK_DIR})
  --list               so mostra a configuracao e os modelos disponiveis
  --hef <caminho>      valor do PI_NIT_HEF (caminho DENTRO do Raspberry)
  --ip <a.b.c.d>       novo IP fixo
  --prefix <n>         mascara em bits (usado com --ip)
  --gateway <a.b.c.d>  novo gateway (usado com --ip)
  --hostname <nome>    novo hostname
  --password <senha>   nova senha do usuario
  --set CHAVE=VALOR    qualquer chave do pi_nit_server.conf (pode repetir)
EOF
	exit 0
}

while [[ $# -gt 0 ]]; do
	case "$1" in
		--image)     IMAGE="$2"; shift 2 ;;
		--hef)       NEW_HEF="$2"; shift 2 ;;
		--ip)        NEW_IP="$2"; shift 2 ;;
		--prefix)    NEW_PREFIX="$2"; shift 2 ;;
		--gateway)   NEW_GATEWAY="$2"; shift 2 ;;
		--hostname)  NEW_HOSTNAME="$2"; shift 2 ;;
		--password)  NEW_PASSWORD="$2"; shift 2 ;;
		--set)       SET_KEYS+=("$2"); shift 2 ;;
		--list)      LIST_ONLY=1; shift ;;
		-h|--help)   usage ;;
		*) echo "opcao desconhecida: $1" >&2; exit 1 ;;
	esac
done

if [[ ${EUID} -ne 0 ]]; then
	echo "erro: rode com sudo (o script monta a imagem)" >&2
	exit 1
fi

if [[ -z "${IMAGE}" ]]; then
	IMAGE=$(ls -t "${WORK_DIR}"/*.img 2>/dev/null | head -1 || true)
	[[ -n "${IMAGE}" ]] || { echo "erro: nenhuma imagem em ${WORK_DIR}; use --image" >&2; exit 1; }
	echo "==> imagem: ${IMAGE}"
fi
[[ -f "${IMAGE}" ]] || { echo "erro: ${IMAGE} nao existe" >&2; exit 1; }

# ------------------------------------------------------------------ montagem

LOOP_DEVICE=""
BOOT_MOUNT=$(mktemp -d)
ROOT_MOUNT=$(mktemp -d)

cleanup()
{
	set +e
	mountpoint -q "${BOOT_MOUNT}" && umount "${BOOT_MOUNT}"
	mountpoint -q "${ROOT_MOUNT}" && umount "${ROOT_MOUNT}"
	[[ -n "${LOOP_DEVICE}" ]] && losetup -d "${LOOP_DEVICE}"
	rmdir "${BOOT_MOUNT}" "${ROOT_MOUNT}" 2>/dev/null
}
trap cleanup EXIT

LOOP_DEVICE=$(losetup --find --show --partscan "${IMAGE}")
for _ in $(seq 20); do [[ -b "${LOOP_DEVICE}p2" ]] && break; sleep 0.3; done
[[ -b "${LOOP_DEVICE}p2" ]] || { echo "erro: ${LOOP_DEVICE}p2 nao apareceu" >&2; exit 1; }

mount "${LOOP_DEVICE}p1" "${BOOT_MOUNT}"
mount "${LOOP_DEVICE}p2" "${ROOT_MOUNT}"

CONFIG="${ROOT_MOUNT}/etc/pi_nit/pi_nit_server.conf"
[[ -f "${CONFIG}" ]] || { echo "erro: ${CONFIG#${ROOT_MOUNT}} nao existe - esta imagem foi gerada no modo firstboot? (a configuracao so aparece depois do primeiro boot)" >&2; exit 1; }

# ------------------------------------------------------------------ inspecao

echo
echo "--- modelos disponiveis na imagem ---"
find "${ROOT_MOUNT}/usr/share/hailo-models" "${ROOT_MOUNT}/opt/pi_nit/models" \
	-name "*.hef" 2>/dev/null | sed "s|${ROOT_MOUNT}||" | sort || true

CURRENT_HEF=$(sed -n 's/^PI_NIT_HEF=//p' "${CONFIG}" | head -1)
echo
echo "--- configuracao atual ---"
grep -E "^PI_NIT_" "${CONFIG}" | grep -vE "^\s*#" || true
echo
if [[ -n "${CURRENT_HEF}" && -f "${ROOT_MOUNT}${CURRENT_HEF}" ]]; then
	echo "PI_NIT_HEF aponta para um arquivo que EXISTE: ${CURRENT_HEF}"
else
	echo "ATENCAO: PI_NIT_HEF aponta para '${CURRENT_HEF}', que NAO existe na imagem."
fi

if [[ ${LIST_ONLY} -eq 1 ]]; then
	echo
	exit 0
fi

# ------------------------------------------------------------------ edicoes

CHANGED=0

# Modelo: se nao foi pedido nada e o atual esta quebrado, conserta sozinho
if [[ -z "${NEW_HEF}" && ( -z "${CURRENT_HEF}" || ! -f "${ROOT_MOUNT}${CURRENT_HEF}" ) ]]; then
	for candidate in "${PREFERRED_MODELS[@]}"; do
		if [[ -f "${ROOT_MOUNT}/usr/share/hailo-models/${candidate}.hef" ]]; then
			NEW_HEF="/usr/share/hailo-models/${candidate}.hef"
			echo
			echo "==> escolhendo automaticamente ${NEW_HEF}"
			break
		fi
	done
	[[ -z "${NEW_HEF}" ]] && echo "aviso: nenhum detector conhecido na imagem; passe --hef" >&2
fi

if [[ -n "${NEW_HEF}" ]]; then
	if [[ ! -f "${ROOT_MOUNT}${NEW_HEF}" ]]; then
		echo "erro: ${NEW_HEF} nao existe dentro da imagem" >&2
		exit 1
	fi
	sed -i "s|^PI_NIT_HEF=.*|PI_NIT_HEF=${NEW_HEF}|" "${CONFIG}"
	echo "  PI_NIT_HEF = ${NEW_HEF}"
	CHANGED=1
fi

for entry in ${SET_KEYS[@]+"${SET_KEYS[@]}"}; do
	key="${entry%%=*}"
	value="${entry#*=}"
	if grep -q "^${key}=" "${CONFIG}"; then
		sed -i "s|^${key}=.*|${key}=${value}|" "${CONFIG}"
	else
		printf '%s=%s\n' "${key}" "${value}" >> "${CONFIG}"
	fi
	echo "  ${key} = ${value}"
	CHANGED=1
done

NM_FILE="${ROOT_MOUNT}/etc/NetworkManager/system-connections/pi-nit-eth0.nmconnection"
if [[ -n "${NEW_IP}" ]]; then
	[[ -f "${NM_FILE}" ]] || { echo "erro: ${NM_FILE##*/} nao existe na imagem" >&2; exit 1; }
	prefix="${NEW_PREFIX:-$(sed -n 's|^address1=[^/]*/\([0-9]*\).*|\1|p' "${NM_FILE}" | head -1)}"
	gateway="${NEW_GATEWAY:-$(sed -n 's|^address1=[^,]*,\(.*\)|\1|p' "${NM_FILE}" | head -1)}"
	sed -i "s|^address1=.*|address1=${NEW_IP}/${prefix:-24},${gateway:-192.168.1.1}|" "${NM_FILE}"
	echo "  ip = ${NEW_IP}/${prefix:-24} (gateway ${gateway:-192.168.1.1})"
	CHANGED=1
fi

if [[ -n "${NEW_HOSTNAME}" ]]; then
	echo "${NEW_HOSTNAME}" > "${ROOT_MOUNT}/etc/hostname"
	sed -i "s/127\.0\.1\.1.*/127.0.1.1\t${NEW_HOSTNAME}/" "${ROOT_MOUNT}/etc/hosts"
	echo "  hostname = ${NEW_HOSTNAME}"
	CHANGED=1
fi

if [[ -n "${NEW_PASSWORD}" ]]; then
	command -v openssl >/dev/null || { echo "erro: falta o openssl" >&2; exit 1; }
	user=$(cut -d: -f1 "${BOOT_MOUNT}/userconf.txt" 2>/dev/null || echo pi)
	printf '%s:%s\n' "${user}" "$(openssl passwd -6 "${NEW_PASSWORD}")" > "${BOOT_MOUNT}/userconf.txt"
	echo "  senha de '${user}' trocada"
	CHANGED=1
fi

sync

echo
if [[ ${CHANGED} -eq 1 ]]; then
	echo "imagem atualizada: ${IMAGE}"
	echo "pode gravar no cartao."
else
	echo "nada a alterar."
fi
