#include "main_window.h"
#include <QtGui>
#include <QApplication>
#include "carmen/carmen_thread.h"
#include <carmen/global_graphics_qt.h>
#include "map_item.h"
#include <QGLWidget>

int main(int argc, char *argv[])
{
	bool two_thread = false;

	if(argc>1) {
		two_thread = atoi(argv[1]);
	}

	QApplication a(argc, argv);
	Carmen_Thread carmen_thread(argc, argv);

	Main_Window main_window;
	main_window.show();

	carmen_thread.register_handlers();

	if(two_thread)
		carmen_thread.start();
	else
		carmen_graphics_update_ipc_callbacks_qt(&carmen_thread, SLOT(updateIPC(int)));

	main_window.resize(1024, 728);

	return a.exec();
}


