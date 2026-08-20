#!/bin/bash
# Converte o poses_opt do sc_lio_sam para o formato que o graphslam_publish do CARMEN le.
#
# O graphslam_publish do CARMEN (src/graphslam/graphslam_publish_main.cpp) le sempre
# 4 colunas: "x y theta timestamp". O sc_lio_sam grava pose 6D em 10 colunas. Sem a
# conversao o fscanf desalinha o arquivo inteiro e as poses publicadas viram lixo,
# sem nenhuma mensagem de erro.
#
# Timestamp usado: sempre o *timestamp do vertice*, nunca o do lidar.
#
# Formatos de entrada reconhecidos (mesmos do parser de origem):
#   4  colunas: x y theta t_vertice
#   6  colunas: x y theta t_lidar t_odom t_vertice
#   7  colunas: x y theta t_vertice t_lidar t_odom t_gps
#   10 colunas: x y z roll pitch yaw t_vertice t_lidar t_odom t_gps
#
# Uso: ./poses_opt_to_carmen.sh <poses_opt.dat> [saida.dat] [log.txt]
#   saida.dat  padrao: <entrada sem extensao>_carmen.dat
#   log.txt    opcional: confere se os timestamps caem dentro do log e sugere MAP_X/MAP_Y

set -u

if [ $# -lt 1 ]; then
	sed -n '2,20p' "$0" | sed 's/^# \?//'
	exit 1
fi

IN=$1
OUT=${2:-${IN%.*}_carmen.dat}
LOG=${3:-}

[ -r "$IN" ] || { echo "erro: nao consigo ler '$IN'" >&2; exit 1; }

awk '
	/^[[:space:]]*(#|$)/ { next }
	NF == 4  { printf "%.6f %.6f %.6f %.6f\n", $1, $2, $3, $4; ok++; next }
	NF == 6  { printf "%.6f %.6f %.6f %.6f\n", $1, $2, $3, $6; ok++; next }
	NF == 7  { printf "%.6f %.6f %.6f %.6f\n", $1, $2, $3, $4; ok++; next }
	NF == 10 { printf "%.6f %.6f %.6f %.6f\n", $1, $2, $6, $7; ok++; next }
	{ bad++ }
	END {
		if (ok == 0) { print "erro: nenhuma linha com 4, 6, 7 ou 10 colunas em " FILENAME > "/dev/stderr"; exit 1 }
		if (bad > 0) printf "aviso: %d linha(s) ignorada(s) por numero de colunas inesperado\n", bad > "/dev/stderr"
	}
' "$IN" > "$OUT" || exit 1

echo "$IN -> $OUT"
echo

awk '
	{
		if (NR == 1) { x0 = minx = maxx = $1; y0 = miny = maxy = $2; t0 = $4 }
		else         { d += sqrt(($1-px)^2 + ($2-py)^2) }
		px = $1; py = $2; t1 = $4
		if ($1 < minx) minx = $1; if ($1 > maxx) maxx = $1
		if ($2 < miny) miny = $2; if ($2 > maxy) maxy = $2
		if (NR > 1 && $4 <= tprev) fora++
		tprev = $4
	}
	END {
		printf "poses          : %d\n", NR
		printf "primeira pose  : x %.2f  y %.2f\n", x0, y0
		printf "percurso       : %.1f m   (bbox %.1f x %.1f m)\n", d, maxx-minx, maxy-miny
		printf "timestamps     : %.3f .. %.3f  (%.1f s)\n", t0, t1, t1-t0
		if (fora > 0) printf "aviso          : %d timestamp(s) nao crescente(s) — o graphslam_publish so casa pose com dt > 0\n", fora
		printf "MAPX %d\nMAPY %d\nT0 %.3f\nT1 %.3f\n", int(x0), int(y0), t0, t1 > "/dev/stderr"
	}
' "$OUT" 2> /tmp/.poses_opt_stats.$$

MAPX=$(awk '/^MAPX/{print $2}' /tmp/.poses_opt_stats.$$)
MAPY=$(awk '/^MAPY/{print $2}' /tmp/.poses_opt_stats.$$)
T0=$(awk '/^T0/{print $2}'   /tmp/.poses_opt_stats.$$)
T1=$(awk '/^T1/{print $2}'   /tmp/.poses_opt_stats.$$)
grep -v '^MAPX\|^MAPY\|^T0\|^T1' /tmp/.poses_opt_stats.$$ >&2
rm -f /tmp/.poses_opt_stats.$$

echo "data           : $(date -d @"${T0%.*}" '+%Y-%m-%d %H:%M:%S') .. $(date -d @"${T1%.*}" '+%H:%M:%S')"

if [ -n "$LOG" ]; then
	echo
	if [ ! -r "$LOG" ]; then
		echo "aviso: nao consigo ler o log '$LOG'" >&2
	else
		read -r L0 L1 <<< "$(grep '^ROBOTVELOCITY_ACK' "$LOG" | awk 'NR==1{a=$4} {b=$4} END{print a, b}')"
		if [ -z "${L0:-}" ]; then
			echo "aviso: nenhum ROBOTVELOCITY_ACK em '$LOG' — nao da pra conferir a janela" >&2
		else
			echo "log            : $(basename "$LOG")  $L0 .. $L1"
			awk -v t0="$T0" -v t1="$T1" -v l0="$L0" -v l1="$L1" 'BEGIN {
				ov = (t1 < l1 ? t1 : l1) - (t0 > l0 ? t0 : l0)
				if (ov <= 0) {
					print "ERRO           : as poses NAO se sobrepoem ao log — epoca/log errados,"
					print "                 o graphslam_publish nao vai casar nenhuma pose."
					exit 1
				}
				printf "sobreposicao   : %.1f s = %.0f%% das poses, %.0f%% do log\n", ov, 100*ov/(t1-t0), 100*ov/(l1-l0)
				if (100*ov/(l1-l0) < 90)
					printf "aviso          : as poses cobrem so %.0f%% do log; o mapa para onde elas acabam\n", 100*ov/(l1-l0)
			}' || exit 1
		fi
	fi
fi

echo
echo "no .ini:"
echo "  SET MAP_X=$MAPX"
echo "  SET MAP_Y=$MAPY"
echo "  SET POSES_PATH=$(readlink -f "$OUT")"
echo "  pub_poses  graphslam  1  0  ./graphslam_publish \${POSES_PATH}"
echo "e lembre que o mapper so recebe pose via ./localize_ackerman -mapping_mode on"
