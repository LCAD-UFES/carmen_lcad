#include "carmen.h"
#include "global.h"
#include <sys/types.h>
#include "global_graphics_qt.h"

#ifdef __APPLE__
#include <limits.h>
#include <float.h>
#define MAXDOUBLE DBL_MAX
#else
#include <values.h>
#endif

void carmen_graphics_update_ipc_callbacks_qt(QObject* parent, const char* updateIPC)
{
  fd_set *open_fds;
  int max_connection;
  int index;
  int callback_index;
  carmen_graphics_callback_qt *callback;
  carmen_graphics_callback_qt new_callback;
  static carmen_list_t *existing_callbacks = NULL;
  static int num_callbacks = 0;
  QSocketNotifier *qt_socket = NULL;

  if (existing_callbacks == NULL)
    existing_callbacks =
      carmen_list_create(sizeof(carmen_graphics_callback_qt), 10);

  for (index = 0; index < num_callbacks; index++) {
    callback = (carmen_graphics_callback_qt*) carmen_list_get(existing_callbacks, index);
    callback->ok = 0;
  }

  open_fds = x_ipcGetConnections();
  max_connection = x_ipcGetMaxConnection();
  for (index = 0; index <= max_connection; index++) {
    if (FD_ISSET(index, open_fds)) {
      for (callback_index = 0; callback_index < num_callbacks;
	   callback_index++) {
	callback = (carmen_graphics_callback_qt*) carmen_list_get(existing_callbacks, callback_index);
	if (index == callback->fd) {
	  callback->ok = 1;
	  break;
	}
      }
      if (callback_index == existing_callbacks->length) {

	qt_socket = new QSocketNotifier(index, QSocketNotifier::Read);

	QObject::connect(qt_socket, SIGNAL(activated(int)), parent, updateIPC);

	new_callback.fd = index;
	new_callback.ok = 1;
	new_callback.callback_id = qt_socket->socket();
	  //gdk_input_add(index, GDK_INPUT_READ, callback_Func, NULL);
	carmen_list_add(existing_callbacks, &new_callback);
      }
    } /* End of if (FD_ISSET(index, open_fds)) */
  } /* End of for (index = 0; index <= max_connection; index++) */

  /*for (index = 0; index < num_callbacks; index++) {
    callback = carmen_list_get(existing_callbacks, index);
    if (callback->ok == 0) {
      gdk_input_remove(callback->callback_id);
      carmen_list_delete(existing_callbacks, index);
      index--;
    }
  }*/
}

