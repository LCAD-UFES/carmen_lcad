#!/bin/bash

if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <source_pose_dataset.txt> <target_dir>"
    exit 1;
fi

linhas=`cat $1| grep -v label | sed 's/\ /_/g'`

for i in $linhas; do
    declare -a lista
    lista=(`echo $i| sed 's/_/\ /g' | awk '{ print $1" "$2" "$NF}'`)
    arquivo=`echo ${lista[0]}`
    arquivo_L=$arquivo
    arquivo_R=`echo $arquivo| sed 's/\.l\./\.r\./g'`
    destino=$2
    mkdir -p $destino
    rotulo=`echo ${lista[1]}`
    indice=`echo ${lista[2]}`
    ln -sf $arquivo_L $destino/$indice"L_B"$rotulo"E.png"
    ln -sf $arquivo_R $destino/$indice"R_B"$rotulo"E.png"
done

echo "o comando a seguir lista arquivos que devem ser removidos pois apontam para locais inexistentes"
find  $destino -type l -exec file {} \; | grep broken > $destino/log.txt

