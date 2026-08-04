# Installing Carmen on Ubuntu 24.04

This is the new installation guide for Ubuntu 24.04. It does not replace the old 16.04/18.04/20.04 notes.

Validated on:

- Ubuntu 24.04.4 LTS
- custom OpenCV 3.2 in `/usr/local/carmen_opencv_3_2`

Constraints used in this installation:

- do not downgrade system packages;
- do not replace packages that are already installed unless strictly needed;
- do not touch NVIDIA driver, CUDA, or cuDNN;
- prefer additive installs from Ubuntu 24.04 itself;
- treat old vehicle/vendor stacks as optional layers.

## 1. Packages for the 24.04 baseline

These package groups install on Ubuntu 24.04 without upgrading or removing existing packages.

Core optional layers:

- `libpcl-dev`
- `pcl-tools`
- `libvtk9-dev`
- `libvtk9-qt-dev`
- `libg2o-dev`
- `protobuf-compiler`
- `libgoogle-glog-dev`
- `liblmdb-dev`
- `libleveldb-dev`
- `libsnappy-dev`
- `libopenblas-dev`
- `libatlas-base-dev`

This was done without changing CUDA/NVIDIA and without any downgrade workflow.

## 2. Recommended installation strategy for Ubuntu 24.04

Use a layered installation:

1. Install the Ubuntu 24.04 development packages.
2. Keep legacy dependencies in custom prefixes when necessary.
3. Rebuild Carmen on 24.04 instead of trying to reproduce the old binary ABI.
4. Leave vehicle/vendor stacks for the end.

This strategy is important because the repository still contains assumptions from older Ubuntu releases:

- hardcoded Python 3.5/3.6/3.8 include and link paths;
- hardcoded PCL 1.8 / VTK 5.x / VTK 6.x paths;
- old Boost link names such as `-lboost_thread-mt` and `-lboost_signals`;
- Qt 4 style linking in some modules;
- multiple legacy global symbols that newer linkers no longer tolerate.

## 3. Ubuntu 24.04 packages

Baseline packages:

```bash
sudo apt update
sudo apt install -y \
  build-essential g++ git swig byacc flex doxygen \
  cmake cmake-curses-gui cmake-qt-gui pkg-config \
  gimp meld vim tcsh wget \
  freeglut3-dev \
  libgtk2.0-dev libgtk-3-dev libgtkglext1 libgtkglext1-dev \
  libimlib2 libimlib2-dev imagemagick libmagick++-dev \
  libwrap0 libwrap0-dev tcpd \
  libncurses-dev libgsl-dev \
  libdc1394-dev libdc1394-utils libraw1394-11 libraw1394-dev \
  libglade2-0 libglade2-dev \
  libcurl4-openssl-dev \
  libkml-dev liburiparser1 liburiparser-dev \
  libusb-1.0-0 libusb-1.0-0-dev libusb-dev \
  libxi-dev libxi6 libxmu-dev libxmu6 \
  libforms-dev libgflags-dev \
  libespeak-dev libfftw3-dev \
  libavcodec-dev libavformat-dev libswscale-dev \
  libgstreamer-plugins-base1.0-dev \
  libjpeg-dev libpng-dev libpng++-dev libtiff5-dev \
  libeigen3-dev libboost-all-dev libflann-dev \
  mpi-default-dev openmpi-bin openmpi-common \
  libproj-dev libsuitesparse-dev libgtest-dev \
  qtbase5-dev qtbase5-dev-tools qtchooser qttools5-dev \
  libasound2-dev mpg123 portaudio19-dev libjsoncpp-dev \
  libglew-dev libudev-dev
```

Point cloud / VTK / graph optimization layer:

```bash
sudo apt install -y \
  libpcl-dev pcl-tools \
  libvtk9-dev libvtk9-qt-dev \
  libg2o-dev
```

Caffe-like dependencies:

```bash
sudo apt install -y \
  protobuf-compiler libgoogle-glog-dev liblmdb-dev \
  libleveldb-dev libsnappy-dev libopenblas-dev libatlas-base-dev
```

## 4. What should not be copied from the old guides

Do not reintroduce the old package names below into the Ubuntu 24.04 recipe:

- `libqt4-dev`
- `qt4-qmake`
- `libvtk6*`
- `libgsl23`
- `libgsl0-dev`
- `libncurses5`
- `libncurses5-dev`
- `libcurl4-nss-dev`
- `libjasper1`
- `libjasper-dev`
- Python 2 packages such as `python-dev`, `python-pip`, `python-numpy`
- pinned CUDA 11.x / cuDNN 8 instructions from old notes

For Ubuntu 24.04:

- use Qt 5 from the distro;
- use VTK 9 from the distro;
- use Python 3.12 from the distro;
- keep CUDA as a separate optional decision.

## 5. OpenCV 3.2 on Ubuntu 24.04

The repository still depends on old OpenCV headers such as:

- `opencv/cv.h`
- `opencv/highgui.h`
- `pkg-config --cflags opencv`

The correct 24.04 strategy is to keep OpenCV 3.2 in a custom prefix.

Expected prefix:

- `/usr/local/carmen_opencv_3_2/lib/pkgconfig/opencv.pc`
- `/usr/local/carmen_opencv_3_2/lib/libopencv_core.so.3.2`

Do not replace the system OpenCV. Rebuild OpenCV 3.2 on Ubuntu 24.04 and keep it isolated.

## 6. Required environment

For interactive shells, keep `CARMEN_HOME` exported to the active repository:

```bash
export CARMEN_HOME=$HOME/carmen_lcad
export LD_LIBRARY_PATH=/usr/local/carmen_opencv_3_2/lib:/usr/local/lib:$LD_LIBRARY_PATH
```

Important update from this migration:

- `src/Makefile.conf` now auto-detects `/usr/local/carmen_opencv_3_2/lib/pkgconfig/opencv.pc`;
- the build no longer depends on manually exporting `PKG_CONFIG_PATH` just so `pkg-config opencv` works;
- the generated executables now embed the OpenCV 3.2 `RUNPATH`, so `localize_ackerman` and `mapper` no longer fail with `code = 127` due to missing `libopencv*.so.3.2`.

Python config note:

- if a Linuxbrew Python is first in `PATH`, plain `python3-config` resolves to it (e.g. Python 3.14);
- for Carmen on Ubuntu 24.04, use `/usr/bin/python3-config`.

Because of that, the Ubuntu 24.04 migration should explicitly prefer:

```bash
/usr/bin/python3-config --includes
/usr/bin/python3-config --embed --libs
```

## 7. Build flow that worked

Conservative configure:

```bash
cd $CARMEN_HOME/src
./configure --nocuda --nopython
```

Build:

```bash
make -k -j"$(nproc)"
```

During validation in this repository, the following patterns were necessary:

- replace hardcoded Python 3.8 config calls with `/usr/bin/python3-config`;
- replace old Boost link flags with `-lboost_thread -lboost_system -pthread`;
- replace old PCL/VTK include assumptions with `pkg-config`;
- replace old Qt 4 style linking with Qt 5 `pkg-config` or Qt 5 package flags;
- fix legacy multiple-definition globals exposed by modern GCC/binutils.

## 8. Repository adjustments already validated on 24.04

The following classes of repository fixes were required and already validated in this tree:

- Qt 5 migration in GUI-related pieces such as `global`, `car_panel_gui`, and `navigator_gui2`
- system Python 3.12 config use in:
  - `map_server`
  - `navigator_gui2`
  - `behavior_selector`
  - `ford_escape_hybrid`
  - `rddf`
  - `mapper`
  - `voice_interface`
- modern PCL / Boost link updates in:
  - `viewer_3D`
  - `graphslam`
  - `moving_objects`
  - `moving_objects_simulator`
  - `localize_ackerman`
  - `odometry_calibration`
  - `stereo_velodyne`
- OpenCV / FFmpeg compatibility fixes in camera code
- duplicate-global cleanup in modules such as:
  - `logger`
  - `sensors/laser_new`
  - `libstereo`
  - `libstereovgram`
  - `behavior_selector`
  - `obstacle_avoider`
- OpenJAUS `ojTorc` header fix for `GEAR_NUMBER`

## 9. Baseline modules that build

The 24.04 build already covers a large part of the codebase, including:

- `navigator_gui2`
- `navigator_gui`
- `tracker`
- `stereo_mapping`
- `mapper`
- `obstacle_distance_mapper`
- `map_server`
- `car_panel_gui`
- `stereo_velodyne`
- `motion_planner`
- `behavior_selector`
- `base_ackerman`
- `road_mapper`
- `can_dump`
- `ford_escape_hybrid`
- `graphslam`
- several support libraries under `sharedlib/`

Also validated:

- `bumblebee_basic` baseline now skips the FlyCapture runtime when that SDK is absent;
- this is the correct default behavior for Ubuntu 24.04 baseline.

## 10. Current build status

The current repository state passes a full:

```bash
make -C src -k -j4
```

with the Ubuntu 24.04 environment described in this guide.

During the final validation pass, the only repeated special-case message was:

- FlyCapture SDK not found; skipping `bumblebee_basic` runtime on Ubuntu 24.04 baseline

That skip is expected and does not block the baseline 24.04 build.

Runtime validation also confirmed:

- `proccontrol process-navigate-volta-da-ufes.ini` no longer respawns `localize_ackerman` and `mapper` with `code = 127`;
- `src/proccontrol/proccontrol.c` now derives `CARMEN_HOME` from the proccontrol binary itself before spawning child modules;
- this prevents a stale shell environment from passing an outdated absolute `CARMEN_HOME` to modules like `navigator_gui2`.
- `src/navigator_gui2/navigator_gui2_main.cpp` no longer forces `gtk_window_maximize()` on startup;
- on Ubuntu 24.04 GNOME/Wayland, that change keeps the window frame on-screen so the window manager buttons remain visible.

Operational note:

- `central` still needs to be running before `proccontrol`; that is part of the normal Carmen architecture on this stack.

Dataset note:

- historical process files such as `bin/process-navigate-volta-da-ufes.ini` still reference map datasets that are not present in this checkout;
- example: `../data/map_voltadaufes-20160323-210` is not present in this checkout;
- with the current 24.04 fixes, `map_server` now fails explicitly with `map_path directory not found` instead of silently running without a map;
- `navigator_gui2` also reports the missing `map_path` directly instead of only opening with an empty canvas.

The two legacy migration points that were explicitly fixed in this validation were:

- OpenJAUS / TORC runtime propagation for:
  - `sharedlib/OpenJAUS/ojTorc`
  - `sharedlib/OpenJAUS/ojIARASim`
  - `src/can_dump`
  - `src/ford_escape_hybrid`
- g2o + modern PCL API updates for:
  - `src/graphslam`
  - related graphslam variants that still used the old factory registration style

These are codebase migration fixes for Ubuntu 24.04, not package downgrades.

## 11. What still belongs to the optional legacy layer

These should not block the first Ubuntu 24.04 installation:

- FlyCapture 2.x
- OpenJAUS / TORC / `ojIARASim`
- vehicle-specific `ford_escape_hybrid`
- legacy CAN playback/simulation paths tied to the OpenJAUS stack
- old CUDA-specific modules
- modules that still depend on hardcoded Python 3.8 or vendor SDKs

## 12. Helper script

The 24.04 helper script is:

- [bin/install-carmen-ubuntu-24-04.sh](../bin/install-carmen-ubuntu-24-04.sh)

Examples:

```bash
./bin/install-carmen-ubuntu-24-04.sh --dry-run
./bin/install-carmen-ubuntu-24-04.sh --apply --with-pcl
./bin/install-carmen-ubuntu-24-04.sh --apply --with-pcl --with-caffe-deps
```

The script is intentionally conservative:

- it does not downgrade packages;
- it does not update NVIDIA driver;
- it does not install CUDA;
- it keeps optional layers separate.

## 13. Practical recommendation

For Ubuntu 24.04, split the installation into two phases.

Phase A:

- Ubuntu 24.04 baseline packages
- OpenCV 3.2 custom prefix
- CPU build of Carmen
- GUI, mapper, path planning, obstacle, localization, and point-cloud baseline

Phase B:

- vehicle-specific stacks
- OpenJAUS / TORC / `ojIARASim`
- optional vendor SDKs
- optional GPU stacks

This keeps the 24.04 installation reproducible without downgrading the system.

## 14. Current status

Ubuntu 24.04 is no longer blocked by missing distro packages or by the previously failing OpenJAUS / graphslam paths.

What remains optional:

- FlyCapture-dependent runtime paths
- old vendor SDK integrations that are not part of the baseline 24.04 installation
- CUDA-specific accelerators that should remain independent from the core CPU build
