#!/usr/bin/env bash
#
# Sobe o servidor em modo --dummy, no proprio Raspberry, sem o acelerador.
#
#   ssh pi@192.168.1.20 bash pi_nit_dummy.sh
#
# Para que serve: validar TODA a cadeia CARMEN <-> Raspberry - rede, protocolo
# binario, batch, letterbox, mapeamento de coordenadas, tracking e publicacao
# IPC - antes de o Hailo estar funcionando. A unica coisa que nao e' exercitada
# e' a inferencia: cada frame volta com uma caixa fixa no centro.
#
# Nao usa o systemd de proposito: a unit tem ConditionPathExists=/dev/hailo0 e
# seria pulada justamente na situacao em que este script e' util.
#
# Ctrl+C encerra.

set -euo pipefail

APP=/opt/pi_nit/app
VENV=/opt/pi_nit/venv/bin/python3
CONF=/etc/pi_nit/pi_nit_server.conf

[ -x "$VENV" ] || { echo "erro: $VENV nao existe" >&2; exit 1; }
[ -f "$APP/pi_nit_server.py" ] || { echo "erro: $APP/pi_nit_server.py nao existe" >&2; exit 1; }

# Reaproveita as portas do arquivo de configuracao, para o teste bater com o
# que o CARMEN vai usar em producao.
FRAME_PORT=$(sed -n 's/^PI_NIT_FRAME_PORT=//p'  "$CONF" 2>/dev/null | head -1)
RESULT_PORT=$(sed -n 's/^PI_NIT_RESULT_PORT=//p' "$CONF" 2>/dev/null | head -1)
VIEWER_PORT=$(sed -n 's/^PI_NIT_VIEWER_PORT=//p' "$CONF" 2>/dev/null | head -1)
BATCH=$(sed -n 's/^PI_NIT_BATCH_SIZE=//p'        "$CONF" 2>/dev/null | head -1)

if systemctl is-active --quiet pi_nit_server 2>/dev/null; then
	echo "aviso: o pi_nit_server esta rodando e vai disputar as portas."
	echo "       pare com:  sudo systemctl stop pi_nit_server"
	exit 1
fi

echo "modo DUMMY - nenhuma inferencia real, so o caminho de dados"
echo "portas: frames ${FRAME_PORT:-5560} | resultados ${RESULT_PORT:-5561} | viewer ${VIEWER_PORT:-0}"
echo
echo "no PC, em outro terminal:"
echo "  python3 tools/test_client.py --host $(hostname -I | awk '{print $1}') \\"
echo "      --video \$CARMEN_HOME/data/pi_nit/pedestres.avi --loop \\"
echo "      --simulate-cameras 3 --fps 15 --show"
echo

exec "$VENV" "$APP/pi_nit_server.py" --dummy --bind 0.0.0.0 \
	--frame-port "${FRAME_PORT:-5560}" \
	--result-port "${RESULT_PORT:-5561}" \
	--viewer-port "${VIEWER_PORT:-0}" \
	--batch-size "${BATCH:-3}"
