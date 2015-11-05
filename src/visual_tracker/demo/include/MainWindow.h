/*
 * Copyright � 2011 Paul Nader 
 *
 * This file is part of QOpenTLD.
 *
 * This program is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 * 
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 * 
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef MAINWINDOW_H_
#define MAINWINDOW_H_

#include <QMainWindow>
#include <QMenuBar>
#include "VideoWindow.h"
#include "SettingsDialog.h"
#include "AboutDialog.h"

class QAction;
class QSettings;
class QCloseEvent;

class MainWindow : public QMainWindow
{
	Q_OBJECT      

public:
	MainWindow();
	~MainWindow();

	void closeEvent(QCloseEvent *event);
	void display(QRectF region);
	void display(IplImage *frame);
	VideoWindow* getVideoWindow() const {return m_videoWindow;};

protected slots:
	void menuTriggered(QAction*);
	void setGeometry(QByteArray);

private:
	QAction* m_fileAction;
	QAction* m_cameraAction;
    QAction* m_bbeeAction;
    QAction* m_directoryAction;
	QAction* m_settingsAction;
	QAction* m_aboutAction;
	VideoWindow* m_videoWindow;
	QStatusBar* m_statusBar;
	QSettings* m_settings;
	SettingsDialog* m_settingsDialog;
	AboutDialog* m_aboutDialog;
};


#endif /* MAINWINDOW_H_ */
