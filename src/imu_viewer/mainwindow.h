#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <iostream>
#include <QtWidgets/QMainWindow>
#include <qgridlayout.h>
#include <objectgl.h>
#include <QMenuBar>
#include <QMessageBox>
#include <carmen/carmen.h>
#include <carmen/xsens_messages.h>
#include <carmen/pi_imu_messages.h>
#include <carmen/pi_imu_interface.h>
#include <carmen/param_interface.h>

#define         DEVICE_NAME     "/dev/ttyACM0"


class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0, int w=600, int h=400, carmen_xsens_global_quat_message *data = 0, carmen_xsens_global_message* pose = 0);
    ~MainWindow();

    bool connect();

protected slots:
    // Redraw the scene
    void  onTimer_UpdateDisplay();

    // Get raw data from Arduini
    void onTimer_ReadData();


    // Open the about dialog box
    void handleAbout();

protected:

    // Overload of the resize event
    void resizeEvent(QResizeEvent *);

private:

    // Layout of the window
    QGridLayout *gridLayout;
    QWidget *gridLayoutWidget;

    // Central widget (where the openGL window is drawn)
    QWidget *centralWidget;

    // OpenGL object
    ObjectOpenGL  *Object_GL;
};

class MainWindowPI : public QMainWindow
{
    Q_OBJECT
public:
     MainWindowPI(QWidget *parent = 0, int w=600, int h=400, carmen_pi_imu_message_t *data = 0, carmen_xsens_global_message *pose = 0);
     ~MainWindowPI();


     bool connect();

protected slots:

    void onTimer_ReadDataPI();

    // Open the about dialog box
    void handleAbout();

    void onTimer_UpdateDisplayPI();

protected:

    // Overload of the resize event
    void resizeEvent(QResizeEvent *);

private:

    // Layout of the window
    QGridLayout *gridLayout;
    QWidget *gridLayoutWidget;

    // Central widget (where the openGL window is drawn)
    QWidget *centralWidget;

    // OpenGL object
    ObjectOpenGL  *Object_GL_PI;
};

#endif // MAINWINDOW_H
