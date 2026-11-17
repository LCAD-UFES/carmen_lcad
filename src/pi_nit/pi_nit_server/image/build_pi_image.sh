#!/usr/bin/env bash
#
# Gera uma imagem do Raspberry Pi OS ja customizada para o pi_nit:
# usuario/senha, SSH ligado, IP fixo, PCIe do Hailo habilitado, aplicacao
# instalada e servico habilitado no boot. Grava no cartao e liga - nada para
# configurar depois.
#
# RODA NO PC (x86), NAO no Raspberry. Precisa de sudo (monta a imagem).
#
#   sudo ./build_pi_image.sh
#   sudo ./build_pi_image.sh --ip 192.168.1.30 --hostname pi-nit-2
#   sudo ./build_pi_image.sh --offline        # ja instala o hailo-all na imagem
#
# Modos:
#
#   firstboot (padrao)  A imagem sai pequena e rapida de gerar. Na PRIMEIRA
#                       vez que o Raspberry ligar, ele instala o hailo-all e a
#                       aplicacao sozinho e reinicia. Exige que o Pi tenha
#                       INTERNET nesse primeiro boot (uns 5 min).
#
#   --offline           Ja instala tudo dentro da imagem, usando emulacao
#                       arm64. O Raspberry nunca precisa de internet, mas a
#                       geracao demora ~30 min e exige qemu-user-static.
#
set -euo pipefail

# ------------------------------------------------------------------ parametros

RASPIOS_URL="https://downloads.raspberrypi.com/raspios_oldstable_lite_arm64/images/raspios_oldstable_lite_arm64-2026-06-19/2026-06-18-raspios-bookworm-arm64-lite.img.xz"

PI_USER="pi"
PI_PASSWORD="1q2w3e4r"
PI_HOSTNAME="pi-nit"
PI_IP="192.168.1.20"
PI_PREFIX="24"
PI_GATEWAY="192.168.1.1"
PI_DNS="8.8.8.8"
OUTPUT=""
OFFLINE=0
EXTRA_GB=3
COMPRESS=0

SOURCE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"   # .../pi_nit_server
WORK_DIR="${WORK_DIR:-/var/tmp/pi_nit_image}"

usage()
{
	sed -n '2,30p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
	cat <<EOF

Opcoes:
  --ip <a.b.c.d>       IP fixo na eth0            (padrao ${PI_IP})
  --prefix <n>         mascara em bits            (padrao ${PI_PREFIX})
  --gateway <a.b.c.d>  gateway                    (padrao ${PI_GATEWAY})
  --dns <a.b.c.d>      servidor DNS               (padrao ${PI_DNS})
  --hostname <nome>    hostname                   (padrao ${PI_HOSTNAME})
  --user <nome>        usuario                    (padrao ${PI_USER})
  --password <senha>   senha                      (padrao ${PI_PASSWORD})
  --output <arq.img>   arquivo de saida
  --offline            instala o hailo-all dentro da imagem (precisa de qemu)
  --compress           gera tambem um .img.xz para guardar/enviar
  --work-dir <dir>     area de trabalho           (padrao ${WORK_DIR})
EOF
	exit 0
}

while [[ $# -gt 0 ]]; do
	case "$1" in
		--ip)        PI_IP="$2"; shift 2 ;;
		--prefix)    PI_PREFIX="$2"; shift 2 ;;
		--gateway)   PI_GATEWAY="$2"; shift 2 ;;
		--dns)       PI_DNS="$2"; shift 2 ;;
		--hostname)  PI_HOSTNAME="$2"; shift 2 ;;
		--user)      PI_USER="$2"; shift 2 ;;
		--password)  PI_PASSWORD="$2"; shift 2 ;;
		--output)    OUTPUT="$2"; shift 2 ;;
		--work-dir)  WORK_DIR="$2"; shift 2 ;;
		--offline)   OFFLINE=1; shift ;;
		--compress)  COMPRESS=1; shift ;;
		-h|--help)   usage ;;
		*) echo "opcao desconhecida: $1" >&2; exit 1 ;;
	esac
done

[[ -z "${OUTPUT}" ]] && OUTPUT="${WORK_DIR}/pi_nit-${PI_HOSTNAME}-$(date +%Y%m%d).img"

# ------------------------------------------------------------------ verificacoes

if [[ ${EUID} -ne 0 ]]; then
	echo "erro: rode com sudo (o script monta a imagem)" >&2
	exit 1
fi

for tool in curl xz losetup parted openssl; do
	command -v "${tool}" >/dev/null || { echo "erro: falta o comando '${tool}'" >&2; exit 1; }
done

if [[ ${OFFLINE} -eq 1 ]] && ! command -v qemu-aarch64-static >/dev/null; then
	echo "erro: --offline precisa do qemu-user-static:" >&2
	echo "      sudo apt install qemu-user-static binfmt-support" >&2
	exit 1
fi

mkdir -p "${WORK_DIR}"

# No modo firstboot nao mexemos no filesystem, entao a imagem fica no tamanho
# original (~2,5 GB). No modo --offline ela cresce para caber o hailo-all.
AVAILABLE_MB=$(df -Pm "${WORK_DIR}" | awk 'NR==2 {print $4}')
if [[ ${OFFLINE} -eq 1 ]]; then
	NEEDED_MB=$(( 3500 + EXTRA_GB * 1024 ))
else
	NEEDED_MB=3500
fi
if [[ ${AVAILABLE_MB} -lt ${NEEDED_MB} ]]; then
	echo "erro: precisa de ~${NEEDED_MB} MB livres em ${WORK_DIR}, ha ${AVAILABLE_MB} MB." >&2
	echo "      use --work-dir apontando para um disco com espaco." >&2
	exit 1
fi

# ------------------------------------------------- ext4 do Raspberry Pi OS 2026
#
# A imagem 2026 usa a feature ext4 'orphan_file' (aparece como FEATURE_C12).
# E' uma feature COMPAT: o kernel que nao a conhece simplesmente a ignora e
# monta o filesystem normalmente - por isso injetar arquivos funciona com
# qualquer e2fsprogs. Quem recusa e' o e2fsck, que precisa entender tudo para
# checar, e o resize2fs, que depende dele.
#
# Ou seja: so precisamos de um e2fsprogs novo (>= 1.47) quando vamos
# REDIMENSIONAR, e isso so acontece no modo --offline.

E2FSCK=e2fsck
RESIZE2FS=resize2fs
E2FSPROGS_VERSION=1.47.0

e2fsprogs_too_old()
{
	local have
	have=$(e2fsck -V 2>&1 | sed -n '1s/.*e2fsck \([0-9][0-9.]*\).*/\1/p')
	[[ -z "${have}" ]] && return 0
	[[ "$(printf '%s\n1.47\n' "${have}" | sort -V | head -1)" != "1.47" ]]
}

build_e2fsprogs()
{
	local dir="${WORK_DIR}/e2fsprogs-${E2FSPROGS_VERSION}"

	if [[ ! -x "${dir}/build/e2fsck/e2fsck" ]]; then
		echo "==> o e2fsprogs do sistema e' antigo demais para esta imagem:"
		echo "    $(e2fsck -V 2>&1 | head -1)"
		echo "    (o Ubuntu 20.04 so tem 1.45.5; a imagem exige >= 1.47)"
		echo "==> compilando o e2fsprogs ${E2FSPROGS_VERSION} - uns 2 min, so desta vez"

		command -v make >/dev/null || \
			{ echo "erro: falta 'make' (sudo apt install build-essential)" >&2; exit 1; }

		local tarball="${WORK_DIR}/e2fsprogs-${E2FSPROGS_VERSION}.tar.gz"
		if [[ ! -f "${tarball}" ]]; then
			curl -fL --progress-bar -o "${tarball}.part" \
				"https://mirrors.edge.kernel.org/pub/linux/kernel/people/tytso/e2fsprogs/v${E2FSPROGS_VERSION}/e2fsprogs-${E2FSPROGS_VERSION}.tar.gz"
			mv "${tarball}.part" "${tarball}"
		fi

		rm -rf "${dir}"
		tar -xzf "${tarball}" -C "${WORK_DIR}"
		mkdir -p "${dir}/build"
		( cd "${dir}/build" && ../configure --disable-nls >/dev/null && make -j"$(nproc)" >/dev/null )
	fi

	E2FSCK="${dir}/build/e2fsck/e2fsck"
	RESIZE2FS="${dir}/build/resize/resize2fs"

	[[ -x "${E2FSCK}" && -x "${RESIZE2FS}" ]] || \
		{ echo "erro: a compilacao do e2fsprogs falhou" >&2; exit 1; }

	echo "==> usando $("${E2FSCK}" -V 2>&1 | head -1)"
}

# Resolvido ANTES do download de 443 MB, para o erro aparecer em 2 s e nao em 2 min
if [[ ${OFFLINE} -eq 1 ]] && e2fsprogs_too_old; then
	build_e2fsprogs
fi

# ------------------------------------------------------------------ limpeza

LOOP_DEVICE=""
BOOT_MOUNT="${WORK_DIR}/mnt_boot"
ROOT_MOUNT="${WORK_DIR}/mnt_root"

cleanup()
{
	set +e
	if [[ -n "${LOOP_DEVICE}" ]]; then
		for path in "${ROOT_MOUNT}/boot/firmware" "${ROOT_MOUNT}/dev/pts" "${ROOT_MOUNT}/dev" \
		            "${ROOT_MOUNT}/proc" "${ROOT_MOUNT}/sys" "${BOOT_MOUNT}" "${ROOT_MOUNT}"; do
			mountpoint -q "${path}" && umount -l "${path}"
		done
		losetup -d "${LOOP_DEVICE}"
	fi
}
trap cleanup EXIT

# ------------------------------------------------------------------ 1. baixar

ARCHIVE="${WORK_DIR}/$(basename "${RASPIOS_URL}")"
if [[ ! -f "${ARCHIVE}" ]]; then
	echo "==> baixando o Raspberry Pi OS ($(basename "${ARCHIVE}"))"
	curl -fL --progress-bar -o "${ARCHIVE}.part" "${RASPIOS_URL}"
	mv "${ARCHIVE}.part" "${ARCHIVE}"
else
	echo "==> usando a imagem ja baixada em ${ARCHIVE}"
fi

echo "==> descompactando para ${OUTPUT}"
xz -dc "${ARCHIVE}" > "${OUTPUT}"

# ------------------------------------------------------------------ 2. crescer

# No modo firstboot NAO tocamos no filesystem: o proprio Raspberry Pi OS
# expande a raiz para o tamanho do cartao no primeiro boot (o cmdline.txt tem
# init=/usr/lib/raspberrypi-sys-mods/firstboot). Redimensionar aqui seria
# trabalho jogado fora - e ainda exigiria um e2fsprogs novo.
if [[ ${OFFLINE} -eq 1 ]]; then
	echo "==> aumentando a particao raiz em ${EXTRA_GB} GB (preciso instalar dentro da imagem)"
	truncate -s "+${EXTRA_GB}G" "${OUTPUT}"
	parted -s "${OUTPUT}" resizepart 2 100%
else
	echo "==> filesystem mantido no tamanho original"
	echo "    (o Raspberry expande a raiz sozinho no primeiro boot)"
fi

LOOP_DEVICE=$(losetup --find --show --partscan "${OUTPUT}")
BOOT_PARTITION="${LOOP_DEVICE}p1"
ROOT_PARTITION="${LOOP_DEVICE}p2"

# O kernel pode demorar alguns instantes para criar os nos das particoes
for _ in $(seq 20); do [[ -b "${ROOT_PARTITION}" ]] && break; sleep 0.3; done
[[ -b "${ROOT_PARTITION}" ]] || { echo "erro: ${ROOT_PARTITION} nao apareceu" >&2; exit 1; }

if [[ ${OFFLINE} -eq 1 ]]; then
	"${E2FSCK}" -p -f "${ROOT_PARTITION}"
	"${RESIZE2FS}" "${ROOT_PARTITION}"
fi

# ------------------------------------------------------------------ 3. montar

mkdir -p "${BOOT_MOUNT}" "${ROOT_MOUNT}"
mount "${BOOT_PARTITION}" "${BOOT_MOUNT}"
mount "${ROOT_PARTITION}" "${ROOT_MOUNT}"

# ------------------------------------------------------------------ 4. boot

echo "==> configurando usuario, SSH e PCIe"

# userconf.txt: o Raspberry Pi OS cria o usuario no primeiro boot a partir daqui
PASSWORD_HASH=$(openssl passwd -6 "${PI_PASSWORD}")
printf '%s:%s\n' "${PI_USER}" "${PASSWORD_HASH}" > "${BOOT_MOUNT}/userconf.txt"

# arquivo 'ssh' vazio liga o servidor SSH no primeiro boot
touch "${BOOT_MOUNT}/ssh"

# PCIe do AI HAT+ (sem isto o Hailo nao aparece no lspci)
if ! grep -q "pi_nit" "${BOOT_MOUNT}/config.txt" 2>/dev/null; then
	cat >> "${BOOT_MOUNT}/config.txt" <<'EOF'

# --- pi_nit: Hailo AI HAT+ -------------------------------------------------
# Sem estas duas linhas o acelerador nao aparece no lspci.
# Se o enlace PCIe ficar instavel (erros no dmesg), troque gen=3 por gen=2.
dtparam=pciex1
dtparam=pciex1_gen=3
EOF
fi

# ------------------------------------------------------------------ 5. rootfs

echo "==> hostname ${PI_HOSTNAME} e IP fixo ${PI_IP}/${PI_PREFIX}"

echo "${PI_HOSTNAME}" > "${ROOT_MOUNT}/etc/hostname"
sed -i "s/127\.0\.1\.1.*/127.0.1.1\t${PI_HOSTNAME}/" "${ROOT_MOUNT}/etc/hosts"

# Bookworm usa NetworkManager. autoconnect-priority alto para ganhar de
# qualquer perfil automatico que apareca depois.
NM_DIR="${ROOT_MOUNT}/etc/NetworkManager/system-connections"
mkdir -p "${NM_DIR}"
cat > "${NM_DIR}/pi-nit-eth0.nmconnection" <<EOF
[connection]
id=pi-nit-eth0
type=ethernet
interface-name=eth0
autoconnect=true
autoconnect-priority=100

[ipv4]
method=manual
address1=${PI_IP}/${PI_PREFIX},${PI_GATEWAY}
dns=${PI_DNS};

[ipv6]
method=disabled
EOF
chmod 600 "${NM_DIR}/pi-nit-eth0.nmconnection"

echo "==> copiando a aplicacao para /opt/pi_nit/src"
mkdir -p "${ROOT_MOUNT}/opt/pi_nit/src"
cp -a "${SOURCE_DIR}/." "${ROOT_MOUNT}/opt/pi_nit/src/"
rm -rf "${ROOT_MOUNT}/opt/pi_nit/src/image" \
       "${ROOT_MOUNT}/opt/pi_nit/src/__pycache__" \
       "${ROOT_MOUNT}/opt/pi_nit/src/tools/__pycache__"
chmod +x "${ROOT_MOUNT}/opt/pi_nit/src/install.sh" \
         "${ROOT_MOUNT}/opt/pi_nit/src/download_model.sh"

# ------------------------------------------------------------------ 6. instalar

install_inside_chroot()
{
	echo "==> instalando dentro da imagem (emulacao arm64, ~30 min)"
	cp "$(command -v qemu-aarch64-static)" "${ROOT_MOUNT}/usr/bin/"
	mount --bind /dev "${ROOT_MOUNT}/dev"
	mount --bind /dev/pts "${ROOT_MOUNT}/dev/pts"
	mount -t proc proc "${ROOT_MOUNT}/proc"
	mount -t sysfs sys "${ROOT_MOUNT}/sys"
	mount --bind "${BOOT_MOUNT}" "${ROOT_MOUNT}/boot/firmware"

	# Criamos o usuario AQUI, e nao via userconf.txt.
	#
	# O userconf.txt so e' aplicado pelo fluxo de primeiro boot do Raspberry
	# Pi OS (userconf-service). Esse fluxo pode nao rodar - ja aconteceu de o
	# arquivo ficar intocado no /boot e o sistema cair no dialogo interativo
	# do tty8, pedindo usuario e senha na tela. Resultado: a senha
	# documentada nao funciona e ninguem entra.
	#
	# No modo --offline temos o chroot na mao, entao criamos o usuario de
	# forma deterministica. Os grupos sao os mesmos que o Raspberry Pi OS da'
	# ao primeiro usuario.
	local groups="adm,dialout,cdrom,sudo,audio,video,plugdev,games,users,input,netdev,gpio,i2c,spi,render"

	# LC_ALL=C so para calar os 'cannot change locale' do chroot
	chroot "${ROOT_MOUNT}" /bin/bash -euxc "
		export DEBIAN_FRONTEND=noninteractive LC_ALL=C LANG=C
		apt-get update
		apt-get install -y hailo-all python3-venv python3-dev libzmq3-dev libgl1 libglib2.0-0

		if ! id -u '${PI_USER}' >/dev/null 2>&1; then
			# So os grupos que existem nesta imagem
			existing=''
			for g in \$(echo '${groups}' | tr ',' ' '); do
				getent group \"\$g\" >/dev/null && existing=\"\${existing:+\$existing,}\$g\"
			done
			useradd -m -s /bin/bash \${existing:+-G \"\$existing\"} '${PI_USER}'
		fi
		echo '${PI_USER}:${PI_PASSWORD}' | chpasswd

		/opt/pi_nit/src/install.sh
		systemctl enable pi_nit_server.service

		# depmod para TODOS os kernels da imagem.
		#
		# O DKMS roda o depmod so para o kernel que ele considera corrente -
		# dentro do chroot, o do PC. Sem isto o hailo_pci.ko fica no lugar
		# certo mas o modules.dep do kernel do Raspberry nao o conhece: o udev
		# nao carrega o driver quando o acelerador aparece, e /dev/hailo0
		# nunca e' criado. Falha silenciosa e chata de achar.
		for k in \$(ls /lib/modules); do
			[ -d \"/lib/modules/\$k/kernel\" ] && depmod -a \"\$k\" || true
		done

		apt-get clean
	"

	# O userconf.txt fica como rede de seguranca (o fluxo de primeiro boot o
	# aplica se rodar), mas o usuario ja existe independentemente dele.
	verify_user_created
	rm -f "${ROOT_MOUNT}/usr/bin/qemu-aarch64-static"

	verify_hailo_module
	provision_model
}


provision_model()
{
	# Uma imagem --offline que precisa de internet no Raspberry para baixar o
	# modelo nao e' offline. Entao o modelo entra aqui, no PC, que tem rede.
	local models_dir="${ROOT_MOUNT}/opt/pi_nit/models"
	local config="${ROOT_MOUNT}/etc/pi_nit/pi_nit_server.conf"
	local shipped=""

	mkdir -p "${models_dir}"

	# 1) O hailo-all do Raspberry Pi OS ja traz varios modelos prontos. A
	#    escolha e' por nome, em ordem de preferencia - NAO da' para pegar
	#    "o primeiro *h8l*" porque a ordem alfabetica devolveria o
	#    resnet_v1_50_h8l (classificacao) em vez de um detector.
	#
	#    Precisa ser: (a) _h8l, compilado para o Hailo-8L de 13 TOPS - um HEF
	#    _h8 e' do Hailo-8 de 26 TOPS e nao roda aqui; (b) detector, nao pose
	#    nem segmentacao, porque o parser espera a saida NMS de deteccao.
	for candidate in yolov8s_h8l yolov6n_h8l yolox_s_leaky_h8l_rpi yolov5s_personface_h8l; do
		if [[ -f "${ROOT_MOUNT}/usr/share/hailo-models/${candidate}.hef" ]]; then
			shipped="${ROOT_MOUNT}/usr/share/hailo-models/${candidate}.hef"
			break
		fi
	done

	if [[ -n "${shipped}" ]]; then
		local name
		name=$(basename "${shipped}")
		echo "==> modelo ja veio no hailo-all: /usr/share/hailo-models/${name}"
		sed -i "s|^PI_NIT_HEF=.*|PI_NIT_HEF=/usr/share/hailo-models/${name}|" "${config}"
		echo "    PI_NIT_HEF apontado para ele"
		return
	fi

	# 2) Nao veio nenhum: baixamos agora e gravamos dentro da imagem.
	echo "==> nenhum modelo no hailo-all; baixando o yolov8s_h8l.hef para dentro da imagem"
	if MODEL_DIR="${models_dir}" bash "${SOURCE_DIR}/download_model.sh" yolov8s; then
		sed -i "s|^PI_NIT_HEF=.*|PI_NIT_HEF=/opt/pi_nit/models/yolov8s_h8l.hef|" "${config}"
	else
		echo >&2
		echo "aviso: nao consegui baixar o modelo. A imagem sobe, mas o servico vai" >&2
		echo "       falhar ate' voce rodar no Raspberry (com internet):" >&2
		echo "         sudo /opt/pi_nit/src/download_model.sh" >&2
	fi
}


verify_user_created()
{
	# Sem isto, um erro no useradd so aparece quando ninguem consegue entrar
	# no Raspberry - com o cartao ja gravado e o Pi montado no carro.
	if ! grep -q "^${PI_USER}:" "${ROOT_MOUNT}/etc/passwd"; then
		echo "erro: o usuario '${PI_USER}' NAO foi criado dentro da imagem." >&2
		exit 1
	fi

	local hash
	hash=$(sed -n "s/^${PI_USER}:\([^:]*\):.*/\1/p" "${ROOT_MOUNT}/etc/shadow")
	if [[ -z "${hash}" || "${hash}" == "!"* || "${hash}" == "*" ]]; then
		echo "erro: o usuario '${PI_USER}' existe mas esta sem senha valida." >&2
		exit 1
	fi

	echo "==> usuario '${PI_USER}' criado, com senha, nos grupos:"
	echo "    $(awk -F: -v u="${PI_USER}" '$4 ~ "(^|,)"u"(,|$)" {printf "%s ", $1}' \
	          "${ROOT_MOUNT}/etc/group")"
}


verify_hailo_module()
{
	# Dentro do chroot o 'uname -r' e' o kernel do PC, nao o do Raspberry. Se o
	# DKMS compilar so para o kernel errado, a imagem sobe normalmente e o
	# acelerador simplesmente nao aparece - falha silenciosa, das piores de
	# achar depois. Entao conferimos aqui.
	local found_pi_kernel=0
	local kernel

	echo "==> conferindo para quais kernels o hailo_pci foi compilado"
	for modules_dir in "${ROOT_MOUNT}"/lib/modules/*/; do
		[[ -d "${modules_dir}" ]] || continue
		kernel=$(basename "${modules_dir}")

		if compgen -G "${modules_dir}updates/dkms/hailo_pci.ko*" >/dev/null; then
			# Ter o arquivo nao basta: sem o depmod, o modules.dep nao conhece
			# o modulo e o udev nunca o carrega.
			if grep -q "hailo_pci" "${modules_dir}modules.dep" 2>/dev/null; then
				echo "    ok  ${kernel}"
			else
				echo "    ok  ${kernel} (arquivo presente, mas fora do modules.dep - rodando depmod)"
				chroot "${ROOT_MOUNT}" depmod -a "${kernel}" 2>/dev/null || \
					echo "        AVISO: depmod falhou para ${kernel}"
			fi
			# Pi 5 = *-rpi-2712, Pi 4 = *-rpi-v8
			[[ "${kernel}" == *rpi* ]] && found_pi_kernel=1
		else
			echo "    --  ${kernel} (sem o modulo - normal se nao for o kernel do Pi)"
		fi
	done

	if [[ ${found_pi_kernel} -eq 0 ]]; then
		echo >&2
		echo "erro: o hailo_pci NAO foi compilado para nenhum kernel do Raspberry." >&2
		echo "      A imagem ate' sobe, mas o acelerador nunca vai aparecer no lspci." >&2
		echo "      Solucao mais simples: gere sem --offline e deixe o Raspberry" >&2
		echo "      instalar no primeiro boot, onde o kernel e' o certo." >&2
		exit 1
	fi
}

install_on_first_boot()
{
	echo "==> preparando a instalacao para o primeiro boot"

	cat > "${ROOT_MOUNT}/usr/local/sbin/pi_nit_firstboot.sh" <<'EOF'
#!/usr/bin/env bash
# Roda UMA vez, no primeiro boot, e se desabilita no fim.
set -euo pipefail
exec > >(tee -a /var/log/pi_nit_firstboot.log) 2>&1

echo "=== pi_nit firstboot em $(date) ==="

echo "--> esperando a rede"
for _ in $(seq 60); do
	getent hosts deb.debian.org >/dev/null 2>&1 && break
	sleep 5
done

export DEBIAN_FRONTEND=noninteractive
echo "--> instalando o hailo-all (driver + HailoRT + Python + modelos)"
apt-get update
apt-get install -y hailo-all python3-venv python3-dev libzmq3-dev libgl1 libglib2.0-0

echo "--> instalando o pi_nit_server"
/opt/pi_nit/src/install.sh

# Os modelos do hailo-all ficam aqui; aponta a configuracao para eles
if [[ -f /usr/share/hailo-models/yolov8s_h8l.hef ]]; then
	sed -i 's|^PI_NIT_HEF=.*|PI_NIT_HEF=/usr/share/hailo-models/yolov8s_h8l.hef|' \
		/etc/pi_nit/pi_nit_server.conf
fi

systemctl enable pi_nit_server.service
systemctl disable pi-nit-firstboot.service
touch /opt/pi_nit/.firstboot_done

echo "=== pi_nit firstboot concluido, reiniciando ==="
sleep 2
reboot
EOF
	chmod 755 "${ROOT_MOUNT}/usr/local/sbin/pi_nit_firstboot.sh"

	cat > "${ROOT_MOUNT}/etc/systemd/system/pi-nit-firstboot.service" <<'EOF'
[Unit]
Description=pi_nit - instalacao no primeiro boot
After=network-online.target
Wants=network-online.target
ConditionPathExists=!/opt/pi_nit/.firstboot_done

[Service]
Type=oneshot
ExecStart=/usr/local/sbin/pi_nit_firstboot.sh
RemainAfterExit=yes
TimeoutStartSec=3600

[Install]
WantedBy=multi-user.target
EOF

	mkdir -p "${ROOT_MOUNT}/etc/systemd/system/multi-user.target.wants"
	ln -sf ../pi-nit-firstboot.service \
		"${ROOT_MOUNT}/etc/systemd/system/multi-user.target.wants/pi-nit-firstboot.service"
}

if [[ ${OFFLINE} -eq 1 ]]; then
	install_inside_chroot
else
	install_on_first_boot
fi

# ------------------------------------------------------------------ 7. fechar

sync
cleanup
trap - EXIT
LOOP_DEVICE=""

echo
echo "imagem pronta: ${OUTPUT}"
ls -lh "${OUTPUT}"

if [[ ${COMPRESS} -eq 1 ]]; then
	echo "==> compactando (demora alguns minutos)"
	xz -T0 -9 -k -f "${OUTPUT}"
	ls -lh "${OUTPUT}.xz"
fi

cat <<EOF

Grave no cartao com o Raspberry Pi Imager (Use custom -> ${OUTPUT})
e NAO use a engrenagem de configuracao: ja esta tudo dentro da imagem.

  usuario  : ${PI_USER}
  senha    : ${PI_PASSWORD}
  hostname : ${PI_HOSTNAME}
  ip       : ${PI_IP}/${PI_PREFIX}  (gateway ${PI_GATEWAY})
  ssh      : ssh ${PI_USER}@${PI_IP}

A deteccao ja vai configurada em /etc/pi_nit/pi_nit_server.conf:

  PI_NIT_CLASSES=0,2,3,5,7   pessoa, carro, moto, onibus, caminhao
                             (o obj_id publicado e' o id COCO menos 1, que e'
                             a convencao do multiple_object_tracker)
  PI_NIT_BATCH_SIZE=1        TEM que ser igual ao numero de cameras; com um
                             batch maior o Hailo repete a imagem para completar
                             o lote e descarta o excedente
  PI_NIT_VIEWER_PORT=0       ponha 5562 para ver as cameras com o
                             tools/pi_nit_viewer.py

Depois de gravar, confira o conjunto todo com:

  ssh ${PI_USER}@${PI_IP} pi_nit_check
  ssh ${PI_USER}@${PI_IP} 'journalctl -u pi_nit_server -f'
EOF

if [[ ${OFFLINE} -eq 0 ]]; then
	cat <<EOF

ATENCAO - modo firstboot: no PRIMEIRO boot o Raspberry precisa de INTERNET.
Ele instala tudo sozinho (~5 min) e reinicia. Acompanhe com:

  ssh ${PI_USER}@${PI_IP} 'tail -f /var/log/pi_nit_firstboot.log'

Se a rede do veículo nao tiver internet, gere a imagem com --offline.
EOF
fi
