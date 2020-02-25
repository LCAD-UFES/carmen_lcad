1) Installing dependencies:
This module depends on the Epik's Multi-Target Tracking (MTT) library:
$ sudo dpkg -i mtt-1.0.0-Linux.deb

2) Building:
$ make clean
$ make -j 4

3) Testing:
$ cd ~/carmen_lcad/bin/
$ ./central

Finally, execute in another terminal:
$ ./proccontrol process-volta_da_ufes_playback_viewer_3D_virtual_scan.ini

