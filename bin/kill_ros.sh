#!/bin/bash
#
# kill_ros.sh — derruba tudo que é ROS 1 nesta máquina, sem precisar de .ini.
#
# Por que existe: quando o proccontrol morre (ou você mata ele), os nós que
# ele subiu NÃO morrem junto — viram órfãos reparentados pro `systemd --user`
# e continuam segurando a porta 11311 e o IPC. O `carmenkillall.sh` só resolve
# isso se você tiver o .ini na mão e ele não conhece os nós ROS.
#
# Ordem importa: o roslaunch morre PRIMEIRO. Se você matar os filhos com o
# roslaunch vivo, ele não colhe ninguém e você fica com uma fileira de
# <defunct> (zumbi de verdade) até o pai sair.
#
# Uso:  ./kill_ros.sh          # pergunta antes
#       ./kill_ros.sh -y       # mata direto
#       ./kill_ros.sh -l       # só lista, não mata

set -u

# Nós do sc_lio_sam + o que o ROS sobe sozinho.
PATTERNS='roslaunch|rosmaster|roscore|rosout|rviz|robot_state_publisher|ipc_bridge_node|pointcloud_node|gps_localization_node|fused_odometry_from_slam_node|lio_sam_(imageProjection|featureExtraction|imuPreintegration|mapOptmization)|ltslam_ltslam|removert_'

# Exclui a si mesmo E toda a ancestralidade (o bash que chamou o script casa
# com o padrao porque a linha de comando dele contem o nome do script).
self_chain() {
    local p=$$
    while [ "$p" -gt 1 ]; do
        echo "$p"
        p=$(ps -o ppid= -p "$p" 2>/dev/null | tr -d ' ')
        [ -z "$p" ] && break
    done
}

list_pids() {
    local skip; skip=$(self_chain | tr '\n' '|')"0"
    pgrep -f -- "$PATTERNS" | grep -Ev "^($skip)$"
}

show() {
    local pids; pids=$(list_pids)
    [ -z "$pids" ] && { echo "Nenhum processo ROS rodando."; return 1; }
    echo "Processos ROS encontrados:"
    ps -o pid=,etime=,args= -p $(echo "$pids" | tr '\n' ' ') 2>/dev/null | cut -c1-140
    return 0
}

case "${1:-}" in
    -l|--list) show; exit 0 ;;
    -y|--yes)  ASSUME_YES=1 ;;
    "")        ASSUME_YES=0 ;;
    *)         echo "Uso: $0 [-y|--yes] [-l|--list]"; exit 1 ;;
esac

show || exit 0

if [ "$ASSUME_YES" -ne 1 ]; then
    read -r -p "Matar todos? [s/N] " r
    [[ "$r" =~ ^[SsYy]$ ]] || { echo "Abortado."; exit 0; }
fi

# 1) roslaunch primeiro, pra ele derrubar e colher os próprios filhos
for p in $(list_pids); do
    ps -o args= -p "$p" 2>/dev/null | grep -q roslaunch && kill "$p" 2>/dev/null
done
sleep 3

# 2) SIGTERM no que sobrou
for p in $(list_pids); do kill "$p" 2>/dev/null; done
sleep 3

# 3) SIGKILL nos teimosos
rest=$(list_pids)
if [ -n "$rest" ]; then
    echo "Insistindo com SIGKILL em: $(echo "$rest" | tr '\n' ' ')"
    for p in $rest; do kill -9 "$p" 2>/dev/null; done
    sleep 1
fi

echo
if list_pids > /dev/null; then
    echo "AINDA SOBROU:"; show
    exit 1
fi
echo "Todos os processos ROS derrubados."
z=$(ps -eo stat | grep -c '^Z')
echo "Zumbis (<defunct>) no sistema: $z"
[ "$z" -gt 0 ] && echo "  (se forem de ROS, o pai deles ainda está vivo — rode 'ps -eo pid,ppid,stat,args | awk \$3~/^Z/')"
exit 0
