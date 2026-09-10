#!/bin/bash
# init_iara.bat - sobe as interfaces CAN da IARA e a rota multicast do JAUS.
#
# Detecta sozinho as interfaces presentes, entao funciona em qualquer maquina
# (o nome da placa de rede muda de PC para PC: enp4s0, enp130s0, eth0...).
#
# Uso:
#   ./init_iara.bat                          # detecta tudo
#   ./init_iara.bat can0 can1                # sobe so essas CANs
#   CAN_BITRATE=500000 ./init_iara.bat       # muda a taxa (padrao 250000)
#   JAUS_IFACE="enp130s0 wlp128s20f3" ./init_iara.bat   # forca a(s) interface(s) do JAUS

CAN_BITRATE=${CAN_BITRATE:-250000}
ERRO=0

sudo -v || exit 1

echo '=== Iniciando CAN / IMU ==='

# Quais CANs subir: as passadas na linha de comando ou todas as que o kernel achou.
if [ $# -gt 0 ]
then
    CANS="$@"
else
    CANS=$(ip -o link show type can 2>/dev/null | awk -F': ' '{print $2}')
fi

if [ -z "$CANS" ]
then
    echo "  ERRO: nenhuma interface CAN encontrada."
    echo "        O conversor USB-CAN nao foi enumerado pelo kernel."
    echo "        Conecte-o DIRETO numa porta USB do notebook (sem hub) e confira com:"
    echo "            lsusb ; ip -br link"
    ERRO=1
else
    for CAN in $CANS
    do
        if ! ip link show "$CAN" > /dev/null 2>&1
        then
            echo "  ERRO: $CAN nao existe (adaptador desconectado?)"
            ERRO=1
            continue
        fi
        sudo ip link set down "$CAN" 2>/dev/null
        if sudo ip link set "$CAN" type can bitrate "$CAN_BITRATE" && sudo ip link set up "$CAN"
        then
            echo "  OK: $CAN a $CAN_BITRATE bps"
        else
            echo "  ERRO: falhou ao configurar $CAN"
            ERRO=1
        fi
    done
fi

echo '=== Iniciando route do JAUS ==='

# Qual interface usar: a forcada pelo usuario, senao a da rede da IARA
# (192.168.0.1 / 192.168.1.1, ver README_COMO_CONTROLAR_IARA_VIA_LAPTOP.md),
# senao qualquer interface cabeada no ar.
if [ -n "$JAUS_IFACE" ]
then
    IFACES="$JAUS_IFACE"
else
    IFACES=$(ip -o -4 addr show | awk '$4 ~ /^192\.168\.[01]\.1\// {print $2}' | sort -u)
fi

if [ -z "$IFACES" ]
then
    IFACES=$(ip -o link show up | awk -F': ' '{print $2}' | grep -E '^(en|eth)' | head -1)
fi

if [ -z "$IFACES" ]
then
    echo "  ERRO: nenhuma interface de rede cabeada no ar; cabo da IARA conectado?"
    echo "        Interfaces disponiveis:"
    ip -br addr | sed 's/^/            /'
    ERRO=1
else
    METRICA=100
    for IFACE in $IFACES
    do
        if ! ip link show "$IFACE" > /dev/null 2>&1
        then
            echo "  ERRO: interface $IFACE nao existe nesta maquina"
            ERRO=1
            continue
        fi
        if sudo ip route replace 224.0.0.0/4 dev "$IFACE" metric "$METRICA"
        then
            echo "  OK: multicast 224.0.0.0/4 via $IFACE (metric $METRICA)"
        else
            echo "  ERRO: falhou ao adicionar rota multicast em $IFACE"
            ERRO=1
        fi
        METRICA=$((METRICA + 10))
    done
fi

echo '=== Estado final ==='
ip -br link | grep -E '^(can|vcan)' | sed 's/^/  /'
ip route show 224.0.0.0/4 | sed 's/^/  /'

exit $ERRO
