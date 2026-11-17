#!/usr/bin/env bash
#
# Cria (ou conserta) o usuario de login direto no cartao SD ja gravado, sem
# regravar a imagem e sem ligar o Raspberry.
#
#   sudo ./fix_user_on_card.sh
#   sudo ./fix_user_on_card.sh --user pi --password 1q2w3e4r
#   sudo ./fix_user_on_card.sh --root /media/miguel/rootfs
#
# Para que serve: o Raspberry Pi OS so cria o primeiro usuario no fluxo de
# primeiro boot, lendo /boot/firmware/userconf.txt. Quando esse fluxo nao
# roda, o arquivo fica intocado e o sistema cai no dialogo interativo do
# tty8 - e a senha documentada simplesmente nao funciona. Este script
# resolve pelo lado do PC.
#
# Como funciona: entra no sistema do cartao com chroot + qemu-aarch64-static
# e usa o useradd/chpasswd DO PROPRIO cartao. E' mais seguro do que editar
# /etc/passwd e /etc/shadow na mao.
#
set -euo pipefail

PI_USER="pi"
PI_PASSWORD="1q2w3e4r"
ROOT_MOUNT=""
STANDARD_GROUPS="adm,dialout,cdrom,sudo,audio,video,plugdev,games,users,input,netdev,gpio,i2c,spi,render"

usage()
{
	sed -n '2,18p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
	cat <<EOF

Opcoes:
  --root <dir>        raiz do cartao (padrao: detecta em /media/*/rootfs)
  --user <nome>       usuario a criar        (padrao ${PI_USER})
  --password <senha>  senha                  (padrao ${PI_PASSWORD})
EOF
	exit 0
}

while [[ $# -gt 0 ]]; do
	case "$1" in
		--root)     ROOT_MOUNT="$2"; shift 2 ;;
		--user)     PI_USER="$2"; shift 2 ;;
		--password) PI_PASSWORD="$2"; shift 2 ;;
		-h|--help)  usage ;;
		*) echo "opcao desconhecida: $1" >&2; exit 1 ;;
	esac
done

if [[ ${EUID} -ne 0 ]]; then
	echo "erro: rode com sudo (escrever no cartao exige root)" >&2
	exit 1
fi

command -v qemu-aarch64-static >/dev/null || {
	echo "erro: falta o qemu-aarch64-static:" >&2
	echo "      sudo apt install qemu-user-static binfmt-support" >&2
	exit 1
}

# ------------------------------------------------------------------ o cartao

if [[ -z "${ROOT_MOUNT}" ]]; then
	ROOT_MOUNT=$(ls -d /media/*/rootfs 2>/dev/null | head -1 || true)
	[[ -n "${ROOT_MOUNT}" ]] || { echo "erro: nao achei o cartao montado; use --root" >&2; exit 1; }
	echo "==> cartao: ${ROOT_MOUNT}"
fi

[[ -f "${ROOT_MOUNT}/etc/passwd" ]] || { echo "erro: ${ROOT_MOUNT} nao parece a raiz de um sistema" >&2; exit 1; }
[[ -f "${ROOT_MOUNT}/etc/rpi-issue" ]] || echo "aviso: nao achei /etc/rpi-issue; este cartao e' mesmo Raspberry Pi OS?"

mountpoint -q "${ROOT_MOUNT}" || { echo "erro: ${ROOT_MOUNT} nao esta montado" >&2; exit 1; }

echo "==> antes:"
awk -F: '$3>=1000 && $3<65000 {print "    usuario existente: "$1" (uid "$3")"}' "${ROOT_MOUNT}/etc/passwd"

# ------------------------------------------------------------------ chroot

QEMU_COPIED=0

cleanup()
{
	set +e
	for path in "${ROOT_MOUNT}/dev/pts" "${ROOT_MOUNT}/dev" "${ROOT_MOUNT}/proc" "${ROOT_MOUNT}/sys"; do
		mountpoint -q "${path}" && umount -l "${path}"
	done
	[[ ${QEMU_COPIED} -eq 1 ]] && rm -f "${ROOT_MOUNT}/usr/bin/qemu-aarch64-static"
	sync
}
trap cleanup EXIT

cp "$(command -v qemu-aarch64-static)" "${ROOT_MOUNT}/usr/bin/"
QEMU_COPIED=1

mount --bind /dev  "${ROOT_MOUNT}/dev"
mount --bind /dev/pts "${ROOT_MOUNT}/dev/pts"
mount -t proc proc "${ROOT_MOUNT}/proc"
mount -t sysfs sys "${ROOT_MOUNT}/sys"

echo "==> criando/ajustando o usuario '${PI_USER}' dentro do cartao"

chroot "${ROOT_MOUNT}" /bin/bash -euc "
	export LC_ALL=C LANG=C

	if id -u '${PI_USER}' >/dev/null 2>&1; then
		echo '    usuario ja existe; so redefinindo a senha'
	else
		existing=''
		for g in \$(echo '${STANDARD_GROUPS}' | tr ',' ' '); do
			getent group \"\$g\" >/dev/null && existing=\"\${existing:+\$existing,}\$g\"
		done
		useradd -m -s /bin/bash \${existing:+-G \"\$existing\"} '${PI_USER}'
		echo \"    criado, nos grupos: \$existing\"
	fi

	echo '${PI_USER}:${PI_PASSWORD}' | chpasswd
"

# ------------------------------------------------------------------ conferencia

echo "==> conferindo"

grep -q "^${PI_USER}:" "${ROOT_MOUNT}/etc/passwd" || {
	echo "erro: o usuario '${PI_USER}' nao aparece em /etc/passwd" >&2; exit 1; }

HASH=$(sed -n "s/^${PI_USER}:\([^:]*\):.*/\1/p" "${ROOT_MOUNT}/etc/shadow")
[[ -n "${HASH}" && "${HASH}" != "!"* && "${HASH}" != "*" ]] || {
	echo "erro: '${PI_USER}' esta sem senha valida" >&2; exit 1; }

# Confere a senha recalculando o hash com o MESMO salt.
#
# O salt e' tudo antes do ultimo '$' - nao os tres primeiros campos. Um hash
# pode vir como $6$salt$hash mas tambem como $6$rounds=N$salt$hash, e cortar
# em campo fixo quebraria o segundo caso.
#
# Esta conferencia e' um extra: ela depende de o libcrypt DESTA maquina
# suportar o mesmo algoritmo do cartao. Se nao suportar, avisamos - mas nao
# derrubamos o script, porque o usuario ja foi criado pelo chpasswd do
# proprio sistema do cartao, que e' quem manda.
SALT="${HASH%\$*}"
ALGO=$(echo "${HASH}" | cut -d'$' -f2)

if command -v python3 >/dev/null; then
	python3 - "${HASH}" "${SALT}" "${PI_PASSWORD}" "${ALGO}" <<'PYEOF'
import crypt, sys
have, salt, password, algo = sys.argv[1:5]
names = {"1": "MD5", "5": "SHA256", "6": "SHA512", "y": "yescrypt", "7": "scrypt"}
print(f"    algoritmo do hash: ${algo}$ ({names.get(algo, 'desconhecido')})")
try:
    calc = crypt.crypt(password, salt)
except Exception as exc:
    print(f"    aviso: nao consegui recalcular o hash aqui ({exc})")
    raise SystemExit(0)
if calc == have:
    print("    senha confere")
elif calc is None:
    print(f"    aviso: o libcrypt desta maquina nao suporta ${algo}$ - nao da' para conferir aqui.")
    print("           Isso NAO quer dizer que a senha esta errada: quem gravou foi o")
    print("           chpasswd do proprio cartao. Teste entrando no Raspberry.")
else:
    print("    AVISO: o hash recalculado nao bateu.")
    print(f"           gravado : {have[:len(salt)+12]}...")
    print(f"           esperado: {calc[:len(salt)+12]}...")
    print("           Se os prefixos acima forem iguais ate' o ultimo '$', o salt")
    print("           esta certo e a diferenca e' de versao do libcrypt - teste no")
    print("           Raspberry antes de concluir que a senha esta errada.")
PYEOF
fi

grep -q "^sudo:.*[:,]${PI_USER}\(,\|$\)" "${ROOT_MOUNT}/etc/group" \
	&& echo "    esta no grupo sudo" \
	|| echo "    AVISO: nao esta no grupo sudo"

echo
echo "pronto. Desmonte o cartao com seguranca e ligue o Raspberry:"
echo "    usuario: ${PI_USER}"
echo "    senha  : ${PI_PASSWORD}"
