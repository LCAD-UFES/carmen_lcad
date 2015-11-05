#ifndef GLOBAL_GRAPHICS_QT_H
#define GLOBAL_GRAPHICS_QT_H

#include <carmen/global.h>
#include <QSocketNotifier>

  typedef struct {
    int fd;
    int callback_id;
    int ok;
  } carmen_graphics_callback_qt;

  void carmen_graphics_update_ipc_callbacks_qt(QObject* parent, const char* updateIPC);

#ifdef __cplusplus
extern "C" {
#endif

  fd_set *x_ipcGetConnections(void);
  int x_ipcGetMaxConnection(void);

#ifdef __cplusplus
}
#endif

#endif
