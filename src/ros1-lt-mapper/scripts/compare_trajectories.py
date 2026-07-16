#!/usr/bin/env python3
"""
Compare two aligned robot trajectories and report how far apart they are.

Input files are plain text, one 3x4 pose matrix per line (row-major):

    r00 r01 r02 tx  r10 r11 r12 ty  r20 r21 r22 tz

This is the format produced by LT-mapper / SC-LIO-SAM intersession loop
output (see map_*_intersession_loops.txt).

Metric
------
The two trajectories do NOT have the same number of poses and are not
necessarily sampled at the same points along the path, so a simple
index-to-index comparison (like ATE/RPE) is not meaningful here. Instead
this script treats each trajectory as a 3D point set/curve and computes a
point-to-curve (nearest-neighbor) distance in both directions:

  - Chamfer distance: mean nearest-neighbor distance, computed A->B and
    B->A, then averaged. This is the standard "how close are these two
    curves on average" metric.
  - Hausdorff distance: the worst-case nearest-neighbor distance (max over
    both directions). This tells you the single point where the two paths
    disagree the most.

Both are reported per-direction and combined, along with mean/median/RMSE/
std/max for each direction, so you can see whether the mismatch is a
uniform offset or a localized divergence (e.g. one path drifting at the
start/end, or a loop-closure error).
"""
import argparse
import sys

import numpy as np


def load_translations(path):
    """Read a file of 3x4 pose matrices (12 floats/line) and return an
    (N, 3) array of translations (tx, ty, tz)."""
    poses = np.loadtxt(path, dtype=np.float64)
    if poses.ndim == 1:
        poses = poses.reshape(1, -1)
    if poses.shape[1] != 12:
        raise ValueError(
            f"{path}: expected 12 values/line (3x4 pose matrix), "
            f"got {poses.shape[1]}"
        )
    poses = poses.reshape(-1, 3, 4)
    translations = poses[:, :, 3]
    return translations


def nearest_neighbor_distances(src, dst, chunk_size=512):
    """For each point in src, distance to its nearest point in dst.
    Chunked to bound peak memory for large trajectories."""
    n = src.shape[0]
    out = np.empty(n, dtype=np.float64)
    for start in range(0, n, chunk_size):
        end = min(start + chunk_size, n)
        diff = src[start:end, None, :] - dst[None, :, :]
        dist = np.sqrt(np.sum(diff * diff, axis=2))
        out[start:end] = dist.min(axis=1)
    return out


def summarize(name, dists):
    print(f"  {name}:")
    print(f"    mean   = {dists.mean():.4f} m")
    print(f"    median = {np.median(dists):.4f} m")
    print(f"    rmse   = {np.sqrt(np.mean(dists ** 2)):.4f} m")
    print(f"    std    = {dists.std():.4f} m")
    print(f"    max    = {dists.max():.4f} m  (Hausdorff, this direction)")


def plot(path_a, path_b, dists_a_to_b, out_file):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    ax = axes[0]
    ax.plot(path_a[:, 0], path_a[:, 1], "-", lw=1, color="tab:blue", label="path A")
    ax.plot(path_b[:, 0], path_b[:, 1], "-", lw=1, color="tab:orange", label="path B")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Aligned trajectories (top-down)")
    ax.axis("equal")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    sc = ax.scatter(path_a[:, 0], path_a[:, 1], c=dists_a_to_b, cmap="viridis", s=6)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Path A colored by distance to nearest point on path B")
    ax.axis("equal")
    fig.colorbar(sc, ax=ax, label="distance [m]")

    fig.tight_layout()
    fig.savefig(out_file, dpi=150)
    print(f"\nSaved plot to {out_file}")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("path_a", help="First trajectory file (3x4 pose/line)")
    parser.add_argument("path_b", help="Second trajectory file (3x4 pose/line)")
    parser.add_argument("--plot", metavar="OUT.png", default=None,
                         help="Save a comparison plot to this file")
    args = parser.parse_args()

    a = load_translations(args.path_a)
    b = load_translations(args.path_b)

    print(f"path A: {args.path_a} ({a.shape[0]} poses)")
    print(f"path B: {args.path_b} ({b.shape[0]} poses)")

    d_a_to_b = nearest_neighbor_distances(a, b)
    d_b_to_a = nearest_neighbor_distances(b, a)

    print("\nDirected distances:")
    summarize("A -> B", d_a_to_b)
    summarize("B -> A", d_b_to_a)

    chamfer = 0.5 * (d_a_to_b.mean() + d_b_to_a.mean())
    hausdorff = max(d_a_to_b.max(), d_b_to_a.max())

    print("\nCombined metrics:")
    print(f"  Chamfer distance   = {chamfer:.4f} m")
    print(f"  Hausdorff distance = {hausdorff:.4f} m")

    if args.plot:
        plot(a, b, d_a_to_b, args.plot)

    return 0


if __name__ == "__main__":
    sys.exit(main())
