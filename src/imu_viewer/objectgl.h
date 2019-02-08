#ifndef OBJECTGL_H
#define OBJECTGL_H

#include <QGLWidget>
#include <QtGui>
#include <iostream>


//using namespace std;

class ObjectOpenGL : public QGLWidget
{
    Q_OBJECT

public:
    ObjectOpenGL(QWidget *parent = 0);                          // Constructor
    ~ObjectOpenGL();                                            // Destructor


    void setAcceleromter(double acc_x, double acc_y, double acc_z) {ax=acc_x; ay=acc_y; az=acc_z; }
    void setGyroscope(double gyro_x, double gyro_y, double gyro_z) {gx=gyro_x; gy=gyro_y; gz=gyro_z; }
    void setMagnetometer(double mag_x, double mag_y, double mag_z) {mx=mag_x; my=mag_y; mz=mag_z; }
    void setAngles(double anx, double any, double anz) {angle_x=anx; angle_y=any; angle_z=anz; }

public slots:
    void FrontView(void);                    // Standard view : front view
    void RearView(void);                     // Standard view : read view
    void LeftView(void);                     // Standard view : left view
    void RightView(void);                    // Standard view : right view
    void TopView(void);                      // Standard view : top view
    void BottomView(void);                   // Standard view : bottom view
    void IsometricView(void);                // Standard view : isometric view

    void SetXRotation(int angle);            // Update the rotation around X
    void SetYRotation(int angle);            // Update the rotation around Y
    void SetZRotation(int angle);            // Update the rotaiton around Z

protected:
    void initializeGL();                     // Initialize OpenGL parameters
    void paintGL();                          // Redraw the scene
    void resizeGL(int width, int height);    // Resize the open GL scene
    void mousePressEvent(QMouseEvent *event);// Called on mouse press event
    void mouseMoveEvent(QMouseEvent *event); // Called on mouse move event
    void wheelEvent(QWheelEvent *event);     // Call on mouse wheel event


signals:
    void xRotationChanged(int angle);        // Signals for new orientations
    void yRotationChanged(int angle);
    void zRotationChanged(int angle);

private:
    void Draw_Frame();                       // Draw the orthonormal frame
    void NormalizeAngle(int *angle);         // Normalized the angle between 0 and 360x16
	void Draw_Box();
    
	QColor BackGround_Color;                   // Color of the background
    QColor Axis_X_Color;                       // X axis's color
    QColor Axis_Y_Color;                       // Y axis's color
    QColor Axis_Z_Color;                       // Z axis's color
    QColor Points_Color;                       // Z axis's color

    QSize  WindowSize;                         // Size (in pixel) of the OpenGL window
    QPoint LastPos;                            // Last position of the mouse (in pixel)
    GLfloat dx;                                 // X Translation  (just for display)
    GLfloat dy;                                 // Y Translation  (just for display)
    GLfloat Zoom;                               // Zoom of the object
    int xRot;                               // Rotation around X
    int yRot;                               // Rotation around Y
    int zRot;                               // Rotation around Z

    // Accelerometer last data
    double ax,ay,az;

    // Gyroscope last data
    double gx,gy,gz;

    // Magnetometer last data
    double mx,my,mz;

    // Eurler angles
    double angle_x,angle_y,angle_z;

};

#endif // OBJECTGL_H
