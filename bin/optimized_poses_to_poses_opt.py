#!/usr/bin/env python3
"""
Gera um poses_opt.dat (10 colunas) a partir da saida crua do SC-LIO-SAM
(optimized_poses.txt + times.txt), carimbando cada pose com o timestamp EXATO do
scan de lidar correspondente no log do CARMEN.

Por que isso e' necessario: o times.txt do SC-LIO-SAM traz o tempo do keyframe, que
nao e' bit-a-bit o timestamp da mensagem VELODYNE_VARIABLE_SCAN do log (no EE a
diferenca e' de alguns milissegundos). O graphslam_publish_globalpos exige casamento
exato (1 us), entao uma pose carimbada com o tempo do keyframe nunca casaria.

Em producao quem resolve isso e' a ponte /carmen/scan_time_reference, consumida pelo
writeLocalizationPose() do mapOptmization: la' o poses_opt.dat ja' sai com o timestamp
CARMEN de cada scan. Este script serve para reaproveitar um mapeamento antigo, de que
so' sobrou o optimized_poses.txt + times.txt.

ATENCAO: a cobertura fica limitada aos KEYFRAMES (tipicamente ~25% dos scans). Os scans
sem pose nao serao publicados -- e' o comportamento correto do modulo, mas o mapa sai
mais esparso do que uma corrida com a ponte de timestamp.

Uso:
  ./optimized_poses_to_poses_opt.py <dir_sc_lio_sam> <log_carmen.txt> <saida.dat> \
      [--origin-x X] [--origin-y Y] [--lidar-id N] [--max-dt S]

  dir_sc_lio_sam  diretorio com optimized_poses.txt e times.txt
  --origin-x/y    origem do mundo somada as coordenadas locais do SC-LIO-SAM
                  (os posesOriginX/posesOriginY do params_*.yaml). Padrao: 0
  --lidar-id      id do lidar no log (padrao: 5)
  --max-dt        descarta o keyframe se o scan mais proximo estiver a mais que isso
                  (segundos, padrao: 0.1)
"""
import argparse, bisect, math, os, sys


def main():
    ap = argparse.ArgumentParser(add_help=False)
    ap.add_argument("slam_dir")
    ap.add_argument("log")
    ap.add_argument("out")
    ap.add_argument("--origin-x", type=float, default=0.0)
    ap.add_argument("--origin-y", type=float, default=0.0)
    ap.add_argument("--lidar-id", type=int, default=5)
    ap.add_argument("--max-dt", type=float, default=0.1)
    ap.add_argument("-h", "--help", action="store_true")
    a = ap.parse_args()
    if a.help:
        print(__doc__)
        return 0

    poses_path = os.path.join(a.slam_dir, "optimized_poses.txt")
    times_path = os.path.join(a.slam_dir, "times.txt")
    for p in (poses_path, times_path, a.log):
        if not os.path.isfile(p):
            print(f"erro: nao encontrei '{p}'", file=sys.stderr)
            return 1

    times = [float(x) for x in open(times_path) if x.strip()]
    mats = [[float(v) for v in l.split()] for l in open(poses_path) if l.strip()]
    if len(times) != len(mats):
        print(f"erro: times.txt tem {len(times)} linhas e optimized_poses.txt tem {len(mats)}",
              file=sys.stderr)
        return 1
    if mats and len(mats[0]) != 12:
        print(f"erro: optimized_poses.txt deveria ter 12 colunas por linha (matriz 3x4), tem {len(mats[0])}",
              file=sys.stderr)
        return 1

    tag = f"VELODYNE_VARIABLE_SCAN_IN_FILE{a.lidar_id}"
    scans = sorted(float(l.split()[5]) for l in open(a.log) if l.startswith(tag))
    if not scans:
        print(f"erro: nenhuma mensagem '{tag}' em '{a.log}'", file=sys.stderr)
        return 1

    def nearest(t):
        i = bisect.bisect_left(scans, t)
        best = None
        for j in (i - 1, i):
            if 0 <= j < len(scans):
                if best is None or abs(scans[j] - t) < abs(best - t):
                    best = scans[j]
        return best

    rows, used, worst, dropped = [], set(), 0.0, 0
    for t, m in zip(times, mats):
        s = nearest(t)
        if s is None or abs(s - t) > a.max_dt or s in used:
            dropped += 1
            continue
        used.add(s)
        worst = max(worst, abs(s - t))
        # matriz 3x4 linha-a-linha: [R00 R01 R02 tx | R10 R11 R12 ty | R20 R21 R22 tz]
        tx, ty, tz = m[3], m[7], m[11]
        yaw = math.atan2(m[4], m[0])
        roll = math.atan2(m[9], m[10])
        pitch = math.asin(max(-1.0, min(1.0, -m[8])))
        rows.append((a.origin_x + tx, a.origin_y + ty, tz, roll, pitch, yaw, s))

    if not rows:
        print("erro: nenhum keyframe casou com um scan do log (epoca errada?)", file=sys.stderr)
        return 1

    rows.sort(key=lambda r: r[6])
    with open(a.out, "w") as f:
        for x, y, z, r, p, yw, s in rows:
            # x y z roll pitch yaw t_vertice t_lidar t_odom t_gps
            f.write(f"{x:.6f} {y:.6f} {z:.6f} {r:.6f} {p:.6f} {yw:.6f} "
                    f"{s:.9f} {s:.9f} -1.000000 -1.000000\n")

    xs = [r[0] for r in rows]
    ys = [r[1] for r in rows]
    print(f"{a.slam_dir} + {os.path.basename(a.log)} -> {a.out}")
    print(f"  keyframes             : {len(times)}  (descartados: {dropped})")
    print(f"  poses gravadas        : {len(rows)}")
    print(f"  scans no log          : {len(scans)}   cobertura: {100*len(rows)/len(scans):.0f}%")
    print(f"  maior ajuste aplicado : {worst*1000:.1f} ms")
    print(f"  bbox                  : x {min(xs):.2f}..{max(xs):.2f}   y {min(ys):.2f}..{max(ys):.2f}")
    print(f"  no .ini               : SET MAP_X={int(rows[0][0])}   SET MAP_Y={int(rows[0][1])}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
