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

#include "MainWindow.h"
#include <QFileDialog>
#include <QSettings>
#include <QCloseEvent>

MainWindow::MainWindow()
: QMainWindow()
, m_fileAction(NULL)
, m_cameraAction(NULL)
{
	m_settings = new QSettings(QString("Paul Nader"), QString("QOpenTLD"));
	m_statusBar = new QStatusBar(this);
	setStatusBar(m_statusBar);
	m_videoWindow = new VideoWindow(*m_statusBar, this);
	setCentralWidget(m_videoWindow);
	m_settingsDialog = new SettingsDialog(*m_settings);
	m_aboutDialog = new AboutDialog();

	QMenuBar* menuBar = this->menuBar();
	QMenu* sourceMenu = menuBar->addMenu(QObject::tr("&Source"));
	m_fileAction = sourceMenu->addAction("&File");
	m_directoryAction = sourceMenu->addAction("&Directory");
	m_cameraAction = sourceMenu->addAction("&Camera");
	QMenu* optionsMenu = menuBar->addMenu(QObject::tr("&Options"));
	m_settingsAction = optionsMenu->addAction("&Settings");
	QMenu* aboutMenu = menuBar->addMenu(QObject::tr("&Help"));
	m_aboutAction = aboutMenu->addAction("&About QOpenTLD");
	connect(menuBar, SIGNAL(triggered(QAction *)), this, SLOT(menuTriggered(QAction *)));
	connect(m_settingsDialog, SIGNAL(boundingRect(QRectF,bool)), m_videoWindow, SLOT(boundingRect(QRectF,bool)));
	connect(m_settingsDialog, SIGNAL(autoStart(bool)), m_videoWindow, SLOT(autoStart(bool)));
	connect(m_settingsDialog, SIGNAL(fpsLimit(int)), m_videoWindow, SLOT(fpsLimit(int)));
	connect(m_settingsDialog, SIGNAL(geometry(QByteArray)), this, SLOT(setGeometry(QByteArray)));
	connect(m_settingsDialog, SIGNAL(brightness(int)), m_videoWindow, SLOT(brightness(int)));
	connect(m_settingsDialog, SIGNAL(contrast(int)), m_videoWindow, SLOT(contrast(int)));
	connect(m_settingsDialog, SIGNAL(detectorBoundingBoxEnabled(bool)), m_videoWindow, SLOT(detectorBoundingBoxEnabled(bool)));
	connect(m_settingsDialog, SIGNAL(detectorPointsEnabled(bool)), m_videoWindow, SLOT(detectorPointsEnabled(bool)));
	connect(m_settingsDialog, SIGNAL(detectorConfidenceEnabled(bool)), m_videoWindow, SLOT(detectorConfidenceEnabled(bool)));
	connect(m_settingsDialog, SIGNAL(detectorPositiveEnabled(bool)), m_videoWindow, SLOT(detectorPositiveEnabled(bool)));
	connect(m_settingsDialog, SIGNAL(detectorNegativeEnabled(bool)), m_videoWindow, SLOT(detectorNegativeEnabled(bool)));
	connect(m_settingsDialog, SIGNAL(detectorFlipImageEnabled(bool)), m_videoWindow, SLOT(detectorFlipImageEnabled(bool)));

	m_settingsDialog->accept();
}

MainWindow::~MainWindow()
{
}

void MainWindow::menuTriggered(QAction *action)
{
	if (action) 
	{
		if (action == m_fileAction)
		{
			QString fileName = QFileDialog::getOpenFileName(this, tr("Open Video"), ".", tr("Video Files (*.avi *.wmv *.mpeg)"));
			if (fileName.size() > 0)
			{
				m_settingsDialog->accept();
				m_videoWindow->openFile(fileName);
			}
			return;
		}

		if (action == m_directoryAction)
		{
			QString dirName = QFileDialog::getExistingDirectory(this, tr("Open Directory"), ".", QFileDialog::ShowDirsOnly|QFileDialog::DontResolveSymlinks);
			if (dirName.size() > 0)
			{
				m_settingsDialog->accept();
				m_videoWindow->openDirectory(dirName);
			}
			return;
		}

		if (action == m_cameraAction)
		{
			m_settingsDialog->accept();
			m_videoWindow->openCamera();
			return;
		}

		if (action == m_settingsAction)
		{
			m_settingsDialog->exec();
			return;
		}

		if (action == m_aboutAction)
		{
			m_aboutDialog->exec();
			return;
		}
	}
}

void MainWindow::setGeometry(QByteArray g)
{
	restoreGeometry(g);
}

void MainWindow::closeEvent(QCloseEvent *event)
{
	m_settings->setValue(GEOMETRY, saveGeometry());
	QWidget::closeEvent(event);
}

void MainWindow::display(QRectF region)
{
	m_videoWindow->display(region);
}

void MainWindow::display(IplImage *frame)
{
	m_videoWindow->display(frame);
}
