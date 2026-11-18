/* Paleta e estilo (tema escuro) do app. */
#ifndef CARMEN_APP_THEME_H
#define CARMEN_APP_THEME_H

#include <QColor>

class QApplication;

namespace pal
{
	const QColor bg(18, 22, 30);
	const QColor panel(26, 31, 41);
	const QColor card(33, 39, 51);
	const QColor line(52, 60, 76);
	const QColor text(222, 226, 233);
	const QColor dim(132, 142, 160);
	const QColor accent(64, 156, 255);
	const QColor green(46, 204, 113);
	const QColor red(240, 82, 82);
	const QColor orange(245, 166, 35);
	const QColor yellow(250, 204, 21);
	const QColor purple(178, 102, 255);
	const QColor teal(38, 198, 218);
	const QColor pink(255, 105, 180);
}

void apply_theme(QApplication &app);

#endif
