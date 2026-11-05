# LT-mapper in ROS2

The native packages are `src/ltslam` and `src/removert` in
`~/carmen_lcad/src/sc_lio_sam_ros2`. They use ROS2 Lyrical and run without
ROS1 or topic bridges. The original ROS1 sources in
`~/carmen_lcad/src/sc_lio_sam/src/lt-mapper` remain only as a comparison reference.

## Build

```bash
cd ~/carmen_lcad/src/sc_lio_sam_ros2
source /opt/ros/lyrical/setup.bash
MAKEFLAGS=-j2 colcon build --base-paths src --packages-select ltslam removert --parallel-workers 1
source ./setup.bash
```

Dependencies are declared in the package manifests: rclcpp, PCL, OpenCV,
pcl_conversions, cv_bridge, image_transport and OpenMP; LT-SLAM also uses GTSAM
4.2. The GTSAM compatibility configuration follows the working SC-LIO-SAM build.
No new sensor/playback entries are needed in the vehicle process files: these
are file-based processing tools, run after saving the sessions.

## Align sessions

```bash
ros2 launch ltslam run.launch.py \
  params_file:=/path/to/ltslam.yaml \
  save_directory:=/tmp/ltmapper-aligned
```

The YAML retains the `ltslam:` section and its original keys, including
`sessions_dir`, `central_sess_name`, `query_sess_name` and the Scan Context/ICP
settings. Each input directory contains `Scans/*.pcd`, `SCDs/*.scd` and
`singlesession_posegraph.g2o`. The result includes the local and central-frame
trajectories before and after inter-session optimization.

`save_directory` is required explicitly by the launch. LT-SLAM recreates this
output directory; give it a dedicated destination. Inputs, their ancestors,
the filesystem root, the home directory and the running directory are rejected
as output destinations. Example configs have their output paths blank.
The inherited `query_anchor_init_yaw_deg` parameter remains in degrees; other
pose translations use metres. Its legacy numeric value was preserved.

## Clean the map and detect changes

```bash
ros2 launch removert run_ltmapper.launch.py \
  params_file:=/path/to/removert.yaml \
  save_pcd_directory:=/tmp/ltmapper-cleaned
```

Use the original `removert:` section with the central/query scan directories,
aligned pose files, 16-element `ExtrinsicLiDARtoPoseBase`, keyframe selection,
range-image resolutions and removal thresholds. The extrinsic matrix is
row-major. Pose files accept the existing KITTI-style matrices.
Outputs retain the original names, including `updated_map.pcd`, static/dynamic
maps and the per-scan `scans_updated*`, `scans_pd*`, `scans_nd_strong` directories.
Output/input overlap is rejected before processing.

The Removert launch enables `rviz:=true keep_alive:=true` by default.
Use `rviz:=false` for headless execution. RViz2 uses hardware
OpenGL by default. Cloud topics retain the `removert` frame and their existing
names. Range-image topics retain BGR8 encoding and image_transport, including
raw and installed compressed transports.

LT-SLAM and Removert remain alive after processing by default, preserving the
ROS1 lifecycle. Use `keep_alive:=false` for batch execution; reconstruction
always exits after saving its output files. Callbacks use a single-thread executor. Existing
OpenMP kernels remain; no worker threads were added. `use_sim_time` is optional
and false by default. File-based computation itself does not wait for `/clock`.

## Reconstruct SC-LIO-SAM localization features

```bash
ros2 launch removert reconstruct_feature_map.launch.py \
  params_file:=/path/to/reconstruct.yaml \
  output_dir:=/tmp/ltmapper-localization
```

The YAML contains both `removert:` for scan directories, poses and extrinsics,
and `map_reconstruct:` for geometric feature thresholds, voxel sizes and
optional precomputed global map. Outputs are `cloudCorner.pcd`, `cloudSurf.pcd`,
`cloudGlobal.pcd`, `trajectory.pcd` and `transformations.pcd`, with their original
point fields for the SC-LIO-SAM map loader. Geometric PCA classification is
preserved; this is the reconstruction utility already present in this project.

The launches read the existing section-based YAMLs and convert names to ROS2
parameters such as `ltslam.sessions_dir` and `removert.keyframe_gap`. Direct
`ros2 run` calls require standard ROS2 parameter files with `ros__parameters`.
The launch arguments `save_directory`, `save_pcd_directory` and `output_dir`
override the corresponding YAML settings.

## Validation on 2026-11-04

All three executables compiled and ran in ROS2. Two overlapping test sessions
were created from the first 12 keyframes, descriptors and graph nodes of a
local saved SC-LIO-SAM map. The source map was read only; all generated data
and outputs were under `/tmp/ltmapper-validation-p79oggq9`.

An isolated build of the original ROS1 sources used the same libraries and
inputs. ROS1/ROS2 comparisons passed:

- LT-SLAM: 8 trajectory files, including accepted inter-session SC constraints.
- Removert: 74 map/per-scan files, matching names, PCD metadata and numeric fields.
- Reconstruction: all 5 output PCD files and their numeric fields.

PCD comparisons use absolute tolerance 1e-5 and relative tolerance 1e-6;
trajectory comparisons use 1e-5. OpenMP was limited to one thread in both
versions for the differential comparison. The original code has a hardcoded
16-thread range projection with unsynchronized writes, so normal multi-core
runs are not guaranteed to produce identical per-scan results. This behavior
was preserved and is not introduced by the ROS2 migration.

Native checks also received all four image topics and cloud messages, checked
BGR8 encoding, cloud fields and frame, opened RViz2 without plugin errors, and
confirmed clean Removert/RViz2 shutdown with simulated time paused. Both
LT-SLAM and Removert retained their default post-processing spin and shut down
cleanly with paused simulated time. The reconstructed map was loaded by the
SC-LIO-SAM ROS2 mapper with both IARA and ARGOS settings; late subscribers
received the global map and the saved files remained unchanged. Invalid
extrinsics and protected output paths are checked before processing.

This validates the port on reduced overlapping sessions. It is not a full
alignment/cleaning validation of every real multi-session dataset or a visual
assessment of all produced maps.

To reproduce the differential test with a ROS1 reference build:

```bash
python3 tests/ltmapper_fixture.py /path/to/saved/sc-lio-sam-map
python3 tests/ltmapper_reference_build.py
# Build /tmp/ltmapper-ros1-reference with Noetic sourced (see its CMakeLists.txt).
python3 tests/integration_ltmapper.py /tmp/ltmapper-validation-<id> \
  --reference /tmp/ltmapper-ros1-reference/build
```

The test uses ROS1 port 11458 and ROS2 domain 158. Run it separately from any
other task using those identifiers. Reference builds and test data in `/tmp`
are temporary and may need to be regenerated.

Additional repeatable native checks, with `setup.bash` sourced:

```bash
python3 tests/ltmapper_parameters.py /tmp/ltmapper-validation-<id>
python3 tests/ltmapper_publications.py /tmp/ltmapper-validation-<id>
python3 tests/ltmapper_lifecycle.py /tmp/ltmapper-validation-<id>
python3 tests/mapping_localization_smoke.py /tmp/ltmapper-validation-<id>/reconstruct_ros2
```

The publication check opens RViz2 and needs an X11 display. Native checks use
ROS domains 159–161; the map-loader test uses domain 148. Source parameter and
point data remain outside generated output directories.

Validation logs on this machine: `/tmp/ltmapper-differential-final.log`,
`/tmp/ltmapper-publications-final.log`, `/tmp/ltmapper-parameters.log`,
`/tmp/ltmapper-lifecycle.log`, `/tmp/ltmapper-sclio-map-load.log`.
