/*********************************************************
	 ---   My Module Specific Messages ---

See IPC documentation for more information:
http://www.cs.cmu.edu/~ipc/

*********************************************************/
#ifndef CARMEN_STEREO_VELODYNE_MESSAGES_H
#define CARMEN_STEREO_VELODYNE_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

#define      CARMEN_STEREO_VELODYNE_SCAN_MESSAGE_NAME       "carmen_stereo_velodyne_scan_message"
#define      CARMEN_STEREO_VELODYNE_SCAN_MESSAGE_FMT        "{int, <{int, <short:1>, <byte:1>, double}:1>, double, string}"

#ifdef __cplusplus
}
#endif

#endif

// @}
