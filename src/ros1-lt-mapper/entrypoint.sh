#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash

MODE="$1"
ROS_MASTER_PORT=11311
LOCKFILE=/shared/roscore.lock

is_roscore_up() {
    (echo > /dev/tcp/localhost/${ROS_MASTER_PORT}) >/dev/null 2>&1
}

start_roscore_if_needed() {
    mkdir -p /shared
    exec 200>"${LOCKFILE}"

    if ! flock -n 200; then
        echo "[roscore] outro container está inicializando o master, aguardando..."
        for i in $(seq 1 30); do
            is_roscore_up && { echo "[roscore] master disponível."; return; }
            sleep 0.5
        done
        echo "[roscore] timeout esperando o master subir."
        exit 1
    fi

    if is_roscore_up; then
        echo "[roscore] master já ativo em localhost:${ROS_MASTER_PORT}, não inicia outro."
    else
        echo "[roscore] nenhum master encontrado, iniciando roscore..."
        roscore &
        for i in $(seq 1 30); do
            if is_roscore_up; then
                echo "[roscore] master no ar."
                break
            fi
            sleep 0.5
        done
        if ! is_roscore_up; then
            echo "[roscore] falha ao iniciar o master."
            exit 1
        fi
    fi

    flock -u 200
}

start_roscore_if_needed

case "$MODE" in
    scliosam)
        echo "[run] subindo SC-LIO-SAM..."
        exec roslaunch lio_sam run.launch
        ;;
    ltslam)
        echo "[run] subindo LT-mapper (ltslam)..."
        exec roslaunch ltslam run.launch
        ;;
    ltremovert)
        echo "[run] subindo LT-mapper (ltremovert)..."
        exec roslaunch ltremovert run_ltmapper.launch
        ;;
    *)
        echo "[run] modo não reconhecido ou nenhum passado, abrindo bash."
        exec /bin/bash
        ;;
esac