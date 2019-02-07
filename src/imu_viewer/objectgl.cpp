#include "objectgl.h"



// -------------------------------------------------
// Initialization
// -------------------------------------------------


ObjectOpenGL::ObjectOpenGL(QWidget *parent) :
        QGLWidget(parent)
{
    // Initialize each color
    BackGround_Color    =QColor::fromRgb(50 ,50 ,100);
    Axis_X_Color        =QColor::fromRgb(255,64  ,64  ,128);                       // Color of the X axis : red
    Axis_Y_Color        =QColor::fromRgb(64  ,255,64  ,128);                       // Color of the Y axis : green
    Axis_Z_Color        =QColor::fromRgb(64  , 64 ,255,128);                       // Color of the Z axis : blue
    Points_Color        =QColor::fromRgb(0  ,0  ,0  ,255);                       // Color of the points

    // Set initial value of angles
    angle_x=angle_y=angle_z=0;
    ax=ay=az=gx=gy=gz=mx=my=mz=0;

    // Start display in the isometric view
    //IsometricView();
    TopView();
}


ObjectOpenGL::~ObjectOpenGL()
{
    makeCurrent();
}


// Initialize OpenGl
void
ObjectOpenGL::initializeGL()
{
    // Intitialize Open GL
    qglClearColor(BackGround_Color);                            // Set backGround color
    glShadeModel(GL_FLAT);
    glEnable(GL_DEPTH_TEST);                                    // Depth buffer enabled (Hide invisible items)
    glEnable(GL_CULL_FACE);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_BLEND);
    glEnable(GL_COLOR_MATERIAL);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);          // For transparency
    glColorMaterial(GL_FRONT,GL_DIFFUSE);
    glEnable(GL_NORMALIZE);
}

// -------------------------------------------------
// Draw the scene
// -------------------------------------------------


// Redraw the openGl window
void
ObjectOpenGL::paintGL(  )
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    // Set the lights
    GLfloat LightAmbient[]={0.4f,0.4f,0.4f,1.0f};
    GLfloat LightDiffuse[]={0.8f,0.8f,0.8f,1.0f};
    glLightfv(GL_LIGHT0,GL_AMBIENT,LightAmbient);
    glLightfv(GL_LIGHT0,GL_DIFFUSE,LightDiffuse);
    int LightPos[4]={0,0,10,1};
    glLightiv(GL_LIGHT0,GL_POSITION,LightPos);

    // Move the display according to the current orientation
    glRotated(xRot / 16.0, 1.0, 0.0, 0.0);
    glRotated(yRot / 16.0, 0.0, 1.0, 0.0);
    glRotated(zRot / 16.0, 0.0, 0.0, 1.0);

    glDisable(GL_LIGHTING);

    // Invert the Y-axis for an orthonormal frame
    glScalef(1,-1,1);

    // Draw the frame
//    Draw_Frame();

    // Start display of the items
    glPushMatrix();                                             // The following properties are only for the object



    // Zoom according to the view's parameters
    glScalef(Zoom,Zoom,Zoom);                                        // ReZoom the object


    // Light independant (color is constant)

    glLineWidth(5.0);

#define RAW_ACC
#define RAW_GYRO
#define RAW_MAG

#ifdef RAW_ACC
    // Accelerometer
    glBegin(GL_LINES);
    qglColor(QColor::fromRgb(255 ,51 ,255));
    glVertex3d(0,0,0);
    glVertex3d(ax,ay,az);
    glEnd();
#endif

#ifdef RAW_GYRO
    // Gyroscopes
    glBegin(GL_LINES);
    qglColor(QColor::fromRgb(255,255 ,0));
    glVertex3d(0,0,0);
    glVertex3d(gx,gy,gz);
    glEnd();
#endif

#ifdef RAW_MAG
    // Magnetometer
    glBegin(GL_LINES);
    qglColor(QColor::fromRgb(32,32,32));
    glVertex3d(0,0,0);
    glVertex3d(mx,my,mz);
    glEnd();
#endif
    Draw_Box();

    glEnable(GL_LIGHTING);                  // Re enable the light


    // End of the object
    glPopMatrix();


    // Update the view
    glViewport(0, 0,WindowSize.width(), WindowSize.height());
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    GLfloat Ratio=(GLfloat)WindowSize.width()/(GLfloat)WindowSize.height();
    glOrtho((-0.5+dx)*Ratio,
            ( 0.5+dx)*Ratio ,
            +0.5+dy,
            -0.5+dy,
            -1500.0, 1500.0);
    glMatrixMode(GL_MODELVIEW);
}


void
ObjectOpenGL::Draw_Box()
{

    glPushMatrix();

    glRotated(angle_z , 0.0, 0.0, 1.0);
    glRotated(angle_y, 0.0, 1.0, 0.0);
    glRotated(-angle_x, 1.0, 0.0, 0.0);

    glPushMatrix();
    glScalef(4.,4.,4.);
    Draw_Frame();
    glPopMatrix();
	
/*glRotated(angle_z , 0.0, 0.0, 1.0);
    glRotated(angle_y, 0.0, 1.0, 0.0);
    glRotated(-angle_x, 1.0, 0.0, 0.0);*/

    // Bottom
    glBegin(GL_POLYGON);
    qglColor(QColor::fromRgb(0,0,255,128));
    glVertex3d(-0.8 ,-0.5   ,-0.2);
    glVertex3d(-0.8 ,0.5    ,-0.2);
    glVertex3d(0.8  ,0.5    ,-0.2);
    glVertex3d(0.8  ,-0.5   ,-0.2);
    glEnd();

    // Top
    glBegin(GL_POLYGON);
    qglColor(QColor::fromRgb(0,255,0,128));
    glVertex3d(-0.8 ,-0.5   ,0.2);
    glVertex3d(0.8  ,-0.5   ,0.2);
    glVertex3d(0.8  ,0.5    ,0.2);
    glVertex3d(-0.8 ,0.5    ,0.2);
    glEnd();

    glBegin(GL_POLYGON);
    qglColor(QColor::fromRgb(255,0,0,128));
    glVertex3d(0.8  ,-0.5    ,0.2);
    glVertex3d(0.8  ,-0.5    ,-0.2);
    glVertex3d(0.8  ,0.5     ,-0.2);
    glVertex3d(0.8  ,0.5     ,0.2);
    glEnd();

    glBegin(GL_POLYGON);
    qglColor(QColor::fromRgb(255,255,0,128));
    glVertex3d(-0.8  ,-0.5    ,0.2);
    glVertex3d(-0.8  ,0.5     ,0.2);
    glVertex3d(-0.8  ,0.5     ,-0.2);
    glVertex3d(-0.8  ,-0.5    ,-0.2);
    glEnd();

    glBegin(GL_POLYGON);
    qglColor(QColor::fromRgb(255,0,255,128));
    glVertex3d(-0.8  ,0.5    ,0.2);
    glVertex3d(0.8   ,0.5     ,0.2);
    glVertex3d(0.8   ,0.5     ,-0.2);
    glVertex3d(-0.8  ,0.5    ,-0.2);
    glEnd();

    glBegin(GL_POLYGON);
    qglColor(QColor::fromRgb(0,255,255,128));
    glVertex3d(-0.8  ,-0.5    ,0.2);
    glVertex3d(-0.8  ,-0.5    ,-0.2);
    glVertex3d(0.8   ,-0.5     ,-0.2);
    glVertex3d(0.8   ,-0.5     ,0.2);
    glEnd();

    qglColor(QColor::fromRgb(255,255,255,255));
    glPointSize(10.0);

    glBegin(GL_POINTS);
    glVertex3d(-0.8 ,-0.5   ,-0.2);
    glVertex3d(-0.8 ,0.5    ,-0.2);
    glVertex3d(0.8  ,0.5    ,-0.2);
    glVertex3d(0.8  ,-0.5   ,-0.2);
    glVertex3d(-0.8 ,-0.5   ,0.2);
    glVertex3d(-0.8 ,0.5    ,0.2);
    glVertex3d(0.8  ,0.5    ,0.2);
    glVertex3d(0.8  ,-0.5   ,0.2);
    glEnd();

    glPopMatrix();

}

// Draw the frame (X,Y and Z axis)
void
ObjectOpenGL::Draw_Frame()
{
    glLineWidth(10.0);
    // X-axis
    glBegin(GL_LINES);
    qglColor(Axis_X_Color);
    glVertex3d(0,0,0);
    glVertex3d(0.25, 0, 0);
    glEnd();
    // Y-axis
    glBegin(GL_LINES);
    qglColor(Axis_Y_Color);
    glVertex3d(0,0,0);
    glVertex3d(0, 0.25, 0);
    glEnd();
    // Z-axis
    glBegin(GL_LINES);
    qglColor(Axis_Z_Color);
    glVertex3d(0,0,0);
    glVertex3d(0, 0, 0.25);
    glEnd();
}


// -------------------------------------------------
// Standard views
// -------------------------------------------------

void
ObjectOpenGL::FrontView()
{
    SetXRotation(0);
    SetYRotation(0);
    SetZRotation(0);
    Zoom=1;
    dx=dy=0;

}

void
ObjectOpenGL::RearView()
{
    SetXRotation(0);
    SetYRotation(180*16);
    SetZRotation(0);
    Zoom=1;
    dx=dy=0;
}

void
ObjectOpenGL::LeftView()
{
    SetXRotation(0);
    SetYRotation(90*16);
    SetZRotation(0);
    Zoom=1;
    dx=dy=0;
}

void
ObjectOpenGL::RightView()
{
    SetXRotation(0);
    SetYRotation(-90*16);
    SetZRotation(0);
    Zoom=1;
    dx=dy=0;
}

void
ObjectOpenGL::TopView()
{
    SetXRotation(-90*16);
    SetYRotation(0);
    SetZRotation(0);
    Zoom=0.5;
   dx=dy=0;
}

void
ObjectOpenGL::BottomView()
{
    SetXRotation(90*16);
    SetYRotation(0);
    SetZRotation(0);
    Zoom=0.5;
    dx=dy=0;
}

void
ObjectOpenGL::IsometricView()
{
    SetXRotation(62*16);
    SetYRotation(0); //-45*16);
    SetZRotation(45*16);
    Zoom=0.5;
    dx=dy=0;
}



// -------------------------------------------------
// Events (Resize, mouse ...)
// -------------------------------------------------

// Resize the openGL window according to width and height
void ObjectOpenGL::resizeGL(int width, int height)
{
    WindowSize=QSize(width,height);
}


// OpenGL angles are contained in the interval [0 : 360*16]
// Normalize the angle in this interval
void
ObjectOpenGL::NormalizeAngle(int *angle)
{
    while (*angle < 0)
        *angle += 360 * 16;
    while (*angle >= 360 * 16)
        *angle -= 360 * 16;
}



// Update the rotation around X if necessary
void
ObjectOpenGL::SetXRotation(int angle)
{
    NormalizeAngle(&angle);
    if (angle != xRot)
    {
        xRot = angle;
        emit xRotationChanged(angle);
        updateGL();
    }
}

// Upadate the rotation around Y if necessary
void ObjectOpenGL::SetYRotation(int angle)
{
    NormalizeAngle(&angle);
    if (angle != yRot)
    {
        yRot = angle;
        emit yRotationChanged(angle);
        updateGL();
    }
}


// Update the rotation around Z if necessary
void
ObjectOpenGL::SetZRotation(int angle)
{
    NormalizeAngle(&angle);
    if (angle != zRot)
    {
        zRot = angle;
        emit zRotationChanged(angle);
        updateGL();
    }
}


// The user has pressed a button
// Memorizes the last mouse position
void ObjectOpenGL::mousePressEvent(QMouseEvent *event){
    // Right button (Rotate)
    if(event->buttons()==Qt::RightButton)
        LastPos = event->pos();
    // Left button (Move)
    if(event->buttons()==Qt::LeftButton)
        LastPos = event->pos();
}

// Wheel event : Change the Zoom
void
ObjectOpenGL::wheelEvent(QWheelEvent *event)
{

    if(event->delta()<0)
        Zoom/= 1-(event->delta()/120.0)/10.0;
    if(event->delta()>0)
        Zoom*= 1+(event->delta()/120.0)/10.0;
}

// Mouse move event
// Update the view if necessary
void
ObjectOpenGL::mouseMoveEvent(QMouseEvent *event)
{
    // Left button : move
    if(event->buttons()==Qt::LeftButton)
    {
        // Compute the difference with the previous position and scale to [-0.5 ; 0.5]
        dx+= -(event->x() - LastPos.x() )/(double)WindowSize.width();
        dy+= -(event->y() - LastPos.y() )/(double)WindowSize.height();
        // Update the view according to the new position
        //        resizeGL(WindowSize.width(),WindowSize.height());
        LastPos = event->pos();
    }

    // Right button (Rotate)
    if(event->buttons()==Qt::RightButton)
    {
        // Get the difference with the previous position
        int dx_mouse = event->x() - LastPos.x();
        int dy_mouse = event->y() - LastPos.y();
        // Update the rotation
        SetXRotation(xRot - 4 * dy_mouse);
        SetYRotation(yRot + 4 * dx_mouse);

        // Memorize previous position
        LastPos = event->pos();
    }
}


