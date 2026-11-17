#!/bin/bash
# Testa um modelo novo no Hailo-8L de verdade, sem deixar a producao
# desligada por acidente.
#
# Resolve a pergunta que fica em aberto na comparacao de versoes do YOLO no
# README.md ("Qual versao do YOLO usar"): os numeros de fps/mAP de la sao do
# Model Zoo, nao do NOSSO Hailo-8L - e a diferenca e' grande (veja a secao
# "Medido no Hailo-8L real" do README: 53-68% do fps oficial, medido).
#
# IMPORTANTE - descoberto rodando isto pela primeira vez (11/08/2026): o
# Hailo-8L so aceita UM processo por vez (hailo_platform.VDevice e'
# exclusivo). Nao da' para comparar dois modelos ao vivo, um em producao e
# outro em teste, no mesmo acelerador. Por isso este script PARA o
# pi_nit_server durante a medicao - a producao fica fora do ar pelo tempo do
# teste (segundos a poucos minutos) e volta sozinha no final, mesmo se o
# script for interrompido com Ctrl+C ou falhar no meio.
#
# Uso (no Raspberry, como usuario com sudo):
#   ./valida_novo_modelo.sh yolov11s
#   ./valida_novo_modelo.sh yolov11s v2.14.0        # forca a versao do Model Zoo
#
# O que ele faz, em ordem, parando no primeiro erro (mas sempre religando a
# producao antes de sair, mesmo em erro):
#   1. baixa o HEF do Model Zoo (reaproveita download_model.sh)
#   2. confere se o HailoRT instalado consegue nem CARREGAR o HEF
#      (hailortcli parse-hef) - e' aqui que aparece incompatibilidade de
#      versao entre o Dataflow Compiler que gerou o HEF e o firmware do Pi
#   3. para a producao (se estiver ativa), mede o fps PURO do modelo
#      (hailortcli run, sem rede nem ZMQ) e religa a producao
#   4. SO' SE pi_nit_server.py estiver ao lado deste script (a pasta
#      completa do projeto, nao so as ferramentas novas): oferece rodar a
#      pipeline completa (ZMQ, batch, JPEG) tambem - ainda exclusivo, ainda
#      com a producao parada por esse tempo
set -euo pipefail

MODEL="${1:?uso: ./valida_novo_modelo.sh <modelo, ex: yolov11s> [zoo_version]}"
ZOO_VERSION="${2:-v2.14.0}"

MODEL_DIR="${MODEL_DIR:-/opt/pi_nit/models}"
HEF_PATH="${MODEL_DIR}/${MODEL}_h8l.hef"

AQUI="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVER_DIR="$(dirname "${AQUI}")"
VENV="${VENV:-/opt/pi_nit/venv/bin/python3}"
[[ -x "${VENV}" ]] || VENV=python3

echo "==> 1/4 baixando ${MODEL} (Model Zoo ${ZOO_VERSION})"
MODEL_DIR="${MODEL_DIR}" ZOO_VERSION="${ZOO_VERSION}" \
	sudo -E "${SERVER_DIR}/download_model.sh" "${MODEL}"

echo
echo "==> 2/4 o firmware deste Pi consegue carregar o HEF?"
echo "    (firmware instalado: $(hailortcli fw-control identify 2>/dev/null | grep 'Firmware Version' || echo '?'))"
if ! hailortcli parse-hef "${HEF_PATH}"; then
	echo >&2
	echo "erro: hailortcli nao conseguiu ler ${HEF_PATH}." >&2
	echo "       Normalmente e' incompatibilidade de versao: o HEF foi" >&2
	echo "       compilado com um Dataflow Compiler mais novo que o" >&2
	echo "       firmware instalado (veja a versao impressa acima). Duas" >&2
	echo "       saidas:" >&2
	echo "         a) procure um ZOO_VERSION mais antigo com HEF do mesmo" >&2
	echo "            modelo: ./valida_novo_modelo.sh ${MODEL} v2.14.0" >&2
	echo "         b) compile localmente a partir do .onnx (nao precisa" >&2
	echo "            de GPU - a Dataflow Compiler roda em CPU): veja" >&2
	echo "            https://docs.ultralytics.com/integrations/hailo/" >&2
	echo "            Isso pode ser feito no WSL2 do PC, sem depender do" >&2
	echo "            Pi para nada alem de instalar o resultado." >&2
	exit 1
fi
echo "    OK - o firmware carrega o HEF"

# ---------------------------------------------------------------- producao
# O Hailo-8L e' exclusivo: so' um processo por vez consegue abrir o
# dispositivo. Para medir, a producao PRECISA sair do ar por um instante.
# Este bloco garante que ela volta, aconteca o que acontecer dali para
# frente (erro, Ctrl+C, o que for) - so' religa se estava ativa antes.
PRODUCAO_ESTAVA_ATIVA=0
if systemctl is-active --quiet pi_nit_server 2>/dev/null; then
	PRODUCAO_ESTAVA_ATIVA=1
fi

religar_producao() {
	if [[ "${PRODUCAO_ESTAVA_ATIVA}" -eq 1 ]]; then
		echo
		echo "==> religando a producao (pi_nit_server)"
		sudo systemctl start pi_nit_server
		sleep 1
		if systemctl is-active --quiet pi_nit_server; then
			echo "    OK - producao ativa de novo"
		else
			echo "    ERRO: producao NAO voltou. Rode 'sudo systemctl status" >&2
			echo "    pi_nit_server' e 'journalctl -u pi_nit_server -n 50' AGORA." >&2
		fi
	fi
}
trap religar_producao EXIT

if [[ "${PRODUCAO_ESTAVA_ATIVA}" -eq 1 ]]; then
	echo
	echo "==> parando a producao pelo tempo do teste (Hailo-8L e' exclusivo)"
	sudo systemctl stop pi_nit_server
	sleep 1
	if systemctl is-active --quiet pi_nit_server; then
		echo "erro: a producao nao parou, abortando sem medir nada." >&2
		exit 1
	fi
	echo "    OK - producao parada, vai religar sozinha ao final deste script"
fi

echo
echo "==> 3/4 fps puro do modelo (sem rede, sem ZMQ, so' o Hailo)"
hailortcli run "${HEF_PATH}" 2>&1 | tee "/tmp/${MODEL}_hailortcli_run.log" || true
echo "    (log completo em /tmp/${MODEL}_hailortcli_run.log)"

echo
echo "==> 4/4 pipeline completa (ZMQ, batch, JPEG) - opcional"
if [[ ! -f "${SERVER_DIR}/pi_nit_server.py" ]]; then
	echo "    pulado: pi_nit_server.py nao esta em ${SERVER_DIR}"
	echo "    (esta pasta so' tem as ferramentas de teste, nao o projeto"
	echo "    inteiro - copie pi_nit_server.py, pi_nit_protocol.py e"
	echo "    hailo_person_detector.py para ca' se quiser este passo)."
	echo "    O numero do passo 3/4 ja' e' o que decide throughput no chip;"
	echo "    este passo so' acrescenta o overhead de rede/JPEG por cima."
else
	echo "    ainda exclusivo - a producao continua parada durante este passo"
	LOG="/tmp/${MODEL}_pi_nit_server.log"
	"${VENV}" "${SERVER_DIR}/pi_nit_server.py" --hef "${HEF_PATH}" --bind 0.0.0.0 \
		> "${LOG}" 2>&1 &
	SERVER_PID=$!
	sleep 2
	if ! kill -0 "${SERVER_PID}" 2>/dev/null; then
		echo "    erro: o servidor de teste nao subiu, veja ${LOG}:" >&2
		tail -n 20 "${LOG}" >&2
	else
		echo "    subiu (pid ${SERVER_PID}) nas portas de producao (5560/5561)"
		echo "    em outra maquina: python3 ${SERVER_DIR}/tools/test_client.py \\"
		echo "        --host $(hostname -I | awk '{print $1}') --camera 0 --fps 15 --frames 200"
		echo "    Ctrl+C aqui quando terminar de medir."
		wait "${SERVER_PID}" || true
	fi
fi

echo
echo "==> feito. A producao volta agora (veja a mensagem de religamento acima/abaixo)."
