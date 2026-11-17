#!/usr/bin/env bash
#
# Diagnostico completo do Raspberry, em um comando so.
#
#   ssh pi@192.168.1.20 'bash pi_nit_check.sh'
#
# Nao precisa de root nem de argumento. Verifica, em ordem de dependencia:
# PCIe -> driver -> dispositivo -> HailoRT -> modelo -> aplicacao -> servico.
# A primeira linha com [X] e' a causa; o resto abaixo dela e' consequencia.

# No Debian, lsmod/modinfo/depmod/ss vivem em /usr/sbin, que NAO entra no PATH
# de uma sessao ssh nao-interativa de usuario comum. Sem isto, 'command not
# found' vira 'modulo ausente' e o diagnostico aponta para o lugar errado.
PATH="$PATH:/sbin:/usr/sbin"

ok()    { printf '  [ok] %s\n' "$1"; }
fail()  { printf '  [X]  %s\n' "$1"; }
warn()  { printf '  [!]  %s\n' "$1"; }
title() { printf '\n== %s\n' "$1"; }

title "hardware PCIe"

if dmesg 2>/dev/null | grep -q "1000110000.pcie: link up"; then
	ok "enlace PCIe do conector externo ativo"
elif dmesg 2>/dev/null | grep -q "1000110000.pcie: link down"; then
	fail "PCIe 'link down' - nada responde no conector externo"
	echo "       O controlador subiu (o dtparam=pciex1 esta certo), mas nao ha"
	echo "       dispositivo do outro lado. Com o Pi DESLIGADO confira:"
	echo "        1. o HAT encaixado ate' o fim nos 40 pinos"
	echo "        2. o cabo flat do PCIe nas duas pontas, travas pressionadas"
	echo "        3. fonte de 27 W oficial (fonte fraca deixa o link down)"
else
	warn "nao achei o controlador 1000110000.pcie no dmesg"
	echo "       Falta 'dtparam=pciex1' em /boot/firmware/config.txt?"
fi

if [ -d /proc/device-tree/hat ]; then
	ok "EEPROM do HAT+ lida: $(tr -d '\0' < /proc/device-tree/hat/product 2>/dev/null)"
else
	fail "sem /proc/device-tree/hat - o HAT nao responde nos 40 pinos"
	echo "       Este caminho e' independente do PCIe. Os dois mudos = placa"
	echo "       nao conectada (ou nao presente)."
fi

if lspci -nn 2>/dev/null | grep -qi "1e60\|hailo"; then
	ok "Hailo no barramento: $(lspci -nn | grep -i '1e60\|hailo')"
else
	fail "nenhum dispositivo Hailo no lspci"
fi

title "driver"

if lsmod 2>/dev/null | grep -q hailo_pci; then
	ok "modulo hailo_pci carregado"
else
	warn "hailo_pci nao carregado (esperado enquanto o PCIe estiver down:"
	echo "       o udev so carrega o driver quando o dispositivo aparece)"

	# 'modinfo <nome>' resolve pelo modules.dep, entao falha em DOIS casos
	# diferentes: arquivo ausente, ou arquivo presente sem depmod. Separar os
	# dois evita mandar reinstalar o hailo-all a toa.
	KO=$(find "/lib/modules/$(uname -r)" -name 'hailo_pci.ko*' 2>/dev/null | head -1)

	if [ -n "$KO" ]; then
		ok "arquivo do modulo presente: $KO"
		if ! command -v modinfo >/dev/null; then
			warn "modinfo indisponivel - nao da' para conferir o modules.dep"
		elif modinfo hailo_pci >/dev/null 2>&1; then
			echo "       e resolvivel pelo nome - so falta o hardware"
		else
			fail "o arquivo existe mas 'modinfo hailo_pci' nao o encontra"
			echo "       falta rodar o depmod para este kernel:"
			echo "         sudo depmod -a"
		fi
	else
		fail "nenhum hailo_pci.ko para o kernel $(uname -r)"
		echo "       compilado para outros kernels:"
		find /lib/modules -name 'hailo_pci.ko*' 2>/dev/null \
			| sed 's|/lib/modules/||; s|/.*||; s/^/         /' | sort -u
		echo "       conserto:  sudo dkms autoinstall"
	fi
fi

if [ -e /dev/hailo0 ]; then
	ok "/dev/hailo0 presente"
else
	fail "/dev/hailo0 ausente - o servico nao vai iniciar (ConditionPathExists)"
fi

title "HailoRT"

if command -v hailortcli >/dev/null; then
	if timeout 15 hailortcli fw-control identify >/tmp/.hailo_id 2>&1; then
		ok "acelerador respondeu:"
		sed 's/^/       /' /tmp/.hailo_id | head -6
	else
		fail "hailortcli nao conseguiu falar com o acelerador"
		sed 's/^/       /' /tmp/.hailo_id | head -4
	fi
	rm -f /tmp/.hailo_id
else
	fail "hailortcli nao instalado (falta o hailo-all)"
fi

title "aplicacao"

CONF=/etc/pi_nit/pi_nit_server.conf
if [ -f "$CONF" ]; then
	ok "configuracao em $CONF"
	grep -E "^PI_NIT_(HEF|BACKEND|BATCH_SIZE|VIEWER_PORT|FRAME_PORT|RESULT_PORT|DUMMY)=" "$CONF" \
		| sed 's/^/       /'

	HEF=$(sed -n 's/^PI_NIT_HEF=//p' "$CONF" | head -1)
	if [ -f "$HEF" ]; then
		ok "modelo existe: $HEF"
	else
		fail "PI_NIT_HEF aponta para '$HEF', que nao existe"
		echo "       modelos disponiveis:"
		ls /usr/share/hailo-models/*.hef /opt/pi_nit/models/*.hef 2>/dev/null | sed 's/^/         /'
	fi

	if grep -q "^PI_NIT_VIEWER_PORT=0" "$CONF"; then
		warn "visualizacao desligada; para ligar:"
		echo "       sudo sed -i s/PORT=0/PORT=5562/ $CONF"
		echo "       sudo systemctl restart pi_nit_server"
	fi
else
	fail "$CONF nao existe - o install.sh nao rodou?"
fi

VENV=/opt/pi_nit/venv/bin/python3
if [ -x "$VENV" ]; then
	MISSING=$("$VENV" - <<'PYEOF' 2>/dev/null
import importlib, sys
falta = [m for m in ("hailo_platform", "zmq", "cv2", "numpy")
         if not importlib.util.find_spec(m)]
print(",".join(falta))
PYEOF
)
	if [ -z "$MISSING" ]; then
		ok "venv com hailo_platform, zmq, cv2 e numpy"
	else
		fail "faltam modulos no venv: $MISSING"
		echo "       hailo_platform faltando = venv sem --system-site-packages"
	fi
else
	fail "$VENV nao existe"
fi

title "servico"

STATE=$(systemctl is-active pi_nit_server 2>/dev/null)
ENABLED=$(systemctl is-enabled pi_nit_server 2>/dev/null)

if [ "$STATE" = "active" ]; then
	ok "pi_nit_server rodando (boot automatico: $ENABLED)"
	echo "       portas escutando:"
	ss -ltn 2>/dev/null | grep -E "556[0-9]" | sed 's/^/         /' \
		|| echo "         (nenhuma porta 556x - o servico subiu mas nao fez bind?)"
else
	fail "pi_nit_server: $STATE (boot automatico: $ENABLED)"
	echo "       ultimas linhas do log:"
	journalctl -u pi_nit_server -n 6 --no-pager 2>/dev/null | sed 's/^/         /'
fi

title "resumo"
if [ -e /dev/hailo0 ] && [ "$STATE" = "active" ]; then
	echo "  tudo no lugar. Teste do PC:"
	echo "    python3 tools/test_client.py --host $(hostname -I | awk '{print $1}') \\"
	echo "        --video pedestres.avi --loop --simulate-cameras 3 --show"
else
	echo "  resolva o primeiro [X] la' em cima - o resto abaixo dele e' consequencia."
fi
echo
