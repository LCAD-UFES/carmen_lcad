/*
 * carmen_app -- interface remota do carmen (roda no notebook, sem carmen nenhum instalado).
 *
 * Conecta ao carmen_server do robo por ZMQ (protocolo em src/carmen_server/PROTOCOLO.md):
 *   SUB tcp://HOST:5560   dados    (pose, mapa, rotas, frenets, grafo, painel, processos, saida)
 *   REQ tcp://HOST:5561   comandos (pose, destino, rota, GO/STOP, missoes, liga/desliga modulo)
 *
 * Uso:  carmen_app [--host IP] [--pub-port 5560] [--cmd-port 5561]
 *
 * Arquivos: connection (ZMQ), scene.h (estado), map_image (decodifica mapas), map_view_3d, car_model
 * (vistas), dashboard (painel), main_window (janela e comandos), theme (cores e estilo).
 */

#include "connection.h"
#include "main_window.h"
#include "theme.h"

#include <QApplication>
#include <QSurfaceFormat>
#include <QTabWidget>
#include <QTimer>
#include <cstdio>

int
main(int argc, char **argv)
{
	/* OpenGL de compatibilidade (a vista 3D usa o pipeline classico, como o viewer_3D) e MSAA */
	QSurfaceFormat fmt;
	fmt.setRenderableType(QSurfaceFormat::OpenGL);
	fmt.setProfile(QSurfaceFormat::CompatibilityProfile);
	fmt.setDepthBufferSize(24);
	fmt.setSamples(4);
	QSurfaceFormat::setDefaultFormat(fmt);

	QApplication app(argc, argv);
	apply_theme(app);

	std::string host = "192.168.123.18";
	int pub_port = 5560, cmd_port = 5561;
	/* teste: --screenshot ARQ.png [--tab N] [--place N] [--full-map] [--top] [--close] [--options] salva a janela e sai */
	QString screenshot;
	int screenshot_tab = 0, screenshot_place = 0;
	QString screenshot_map;
	bool screenshot_overlay = false, screenshot_pick = false, screenshot_cameras = false;
	bool screenshot_full = false, screenshot_top = false, screenshot_close = false, screenshot_options = false;
	QStringList args = app.arguments();
	for (int i = 1; i < args.size(); i++)
	{
		if (args[i] == "--host" && i + 1 < args.size())           host = args[++i].toStdString();
		else if (args[i].startsWith("--host="))                    host = args[i].mid(7).toStdString();
		else if (args[i] == "--pub-port" && i + 1 < args.size())   pub_port = args[++i].toInt();
		else if (args[i] == "--cmd-port" && i + 1 < args.size())   cmd_port = args[++i].toInt();
		else if (args[i] == "--screenshot" && i + 1 < args.size()) screenshot = args[++i];
		else if (args[i] == "--tab" && i + 1 < args.size())        screenshot_tab = args[++i].toInt();
		else if (args[i] == "--place" && i + 1 < args.size())      screenshot_place = args[++i].toInt();
		else if (args[i] == "--full-map")                          screenshot_full = true;
		else if (args[i] == "--map" && i + 1 < args.size())        screenshot_map = args[++i];
		else if (args[i] == "--overlay")                           screenshot_overlay = true;
		else if (args[i] == "--cameras")                           screenshot_cameras = true;
		else if (args[i] == "--pick")                              screenshot_pick = true;
		else if (args[i] == "--close")                             screenshot_close = true;
		else if (args[i] == "--options")                           screenshot_options = true;
		else if (args[i] == "--top")                                screenshot_top = true;
		else if (args[i] == "-h" || args[i] == "--help")
		{
			printf("uso: carmen_app [--host IP] [--pub-port 5560] [--cmd-port 5561]\n"
				   "  --host  IP do robo com o carmen_server (padrao: 192.168.123.18)\n");
			return 0;
		}
	}

	Connection conn(host, pub_port, cmd_port);
	MainWindow win(&conn);
	win.show();

	if (!screenshot_map.isEmpty())
		QTimer::singleShot(2500, [&win, screenshot_map, screenshot_overlay]() { win.test_map(screenshot_map, screenshot_overlay); });
	if (screenshot_cameras)
		QTimer::singleShot(2500, [&win]() { win.test_cameras(); });
	if (screenshot_pick)
		QTimer::singleShot(3000, [&win]() { win.test_pick(); });
	if (screenshot_close)
		QTimer::singleShot(4500, [&win]() { win.test_close(); });
	if (screenshot_options)
		QTimer::singleShot(4500, [&win]() { win.test_options(); });
	if (screenshot_top)
		QTimer::singleShot(4000, [&win]() { win.test_top_view(); });
	if (!screenshot.isEmpty())
	{
		QTimer::singleShot(500, [&win, screenshot_tab]() { win.tabs->setCurrentIndex(screenshot_tab); });
		if (screenshot_full)
			QTimer::singleShot(3000, [&win]() { win.test_full_map(); });
		if (screenshot_place > 0)
			QTimer::singleShot(5000, [&win, screenshot_place]() { win.test_select_place(screenshot_place); });
		QTimer::singleShot(7500, [&win, &app, screenshot]() {
			win.grab().save(screenshot);
			app.quit();
		});
	}
	return app.exec();
}
