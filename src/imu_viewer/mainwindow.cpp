#include "mainwindow.h"

carmen_xsens_global_quat_message *data_copy;
carmen_xsens_global_message *pose_copy, *pose_pi_copy;
carmen_pi_imu_message_t *imu_msg_copy;

// Constructor of the main window
// Create window properties, menu etc ...
MainWindow::MainWindow(QWidget *parent,int w, int h, carmen_xsens_global_quat_message *data, carmen_xsens_global_message *pose)
    : QMainWindow(parent)
{        
    // Set the window size
    this->resize(w,h);
    this->setWindowTitle("Object viewer");
    // Create a layout in the main window
    centralWidget = new QWidget(this);
    gridLayoutWidget = new QWidget(centralWidget);
    gridLayoutWidget->setGeometry(QRect(0, 0, this->width(), this->height()));
    gridLayout = new QGridLayout(gridLayoutWidget);

    // Create the openGL display for the map
    Object_GL = new ObjectOpenGL(gridLayoutWidget);
    Object_GL->setObjectName(QString::fromUtf8("ObjectOpenGL"));
    Object_GL->setGeometry(QRect(0, 0, this->width(), this->height()));

    // Insert the Open Gl display into the layout
    gridLayout->addWidget(Object_GL, 0, 0, 1, 1);
    setCentralWidget(centralWidget);

    // Create the menubar
    QMenu *FileMenu = menuBar()->addMenu("&File");
    FileMenu->addSeparator();
    FileMenu->addAction("Quit", qApp, SLOT (quit()), QKeySequence(tr("Ctrl+Q")));

    // Add menu items
    QMenu* ViewMenu = menuBar()->addMenu("&View");
    ViewMenu->addAction("Front view",       Object_GL, SLOT (FrontView()),  QKeySequence(tr("Ctrl+f")));
    ViewMenu->addAction("Rear view",        Object_GL, SLOT (RearView()),   QKeySequence(tr("Ctrl+e")));
    ViewMenu->addAction("Left view",        Object_GL, SLOT (LeftView()),   QKeySequence(tr("Ctrl+l")));
    ViewMenu->addAction("Right view",       Object_GL, SLOT (RightView()),  QKeySequence(tr("Ctrl+r")));
    ViewMenu->addAction("Top view",         Object_GL, SLOT (TopView()),    QKeySequence(tr("Ctrl+t")));
    ViewMenu->addAction("Bottom view",      Object_GL, SLOT (BottomView()), QKeySequence(tr("Ctrl+b")));
    FileMenu->addSeparator();
    ViewMenu->addAction("Isometric",        Object_GL, SLOT (IsometricView()), QKeySequence(tr("Ctrl+i")));
    QMenu* AboutMenu = menuBar()->addMenu("?");
    AboutMenu->addAction("About Convert_STL_2_Cube", this, SLOT (handleAbout()));
	data_copy = data;
	pose_copy = pose;
	//Timer (used for repainting the GL Window and treating IPC every 50 ms)
    QTimer *timerDisplay = new QTimer();
    timerDisplay->connect(timerDisplay, SIGNAL(timeout()),this, SLOT(onTimer_UpdateDisplay()));
    timerDisplay->start(75);
    // Timer for reading raw data (every 10ms)
    QTimer *timerArduino = new QTimer();
    timerArduino->connect(timerArduino, SIGNAL(timeout()),this, SLOT(onTimer_ReadData()));
    timerArduino->start(10);
}


// Constructor of the main window
// Create window properties, menu etc ...
MainWindowPI::MainWindowPI(QWidget *parent,int w, int h, carmen_pi_imu_message_t *imu_msg, carmen_xsens_global_message *pose_pi)
    : QMainWindow(parent)
{        
    // Set the window size
    this->resize(w,h);
    this->setWindowTitle("Object viewer PI");
    // Create a layout in the main window
    centralWidget = new QWidget(this);
    gridLayoutWidget = new QWidget(centralWidget);
    gridLayoutWidget->setGeometry(QRect(0, 0, this->width(), this->height()));
    gridLayout = new QGridLayout(gridLayoutWidget);


    // Create the openGL display for the map
    Object_GL_PI = new ObjectOpenGL(gridLayoutWidget);
    Object_GL_PI->setObjectName(QString::fromUtf8("ObjectOpenGL"));
    Object_GL_PI->setGeometry(QRect(0, 0, this->width(), this->height()));

    // Insert the Open Gl display into the layout
    gridLayout->addWidget(Object_GL_PI, 0, 0, 1, 1);
    setCentralWidget(centralWidget);

    // Create the menubar
    QMenu *FileMenu = menuBar()->addMenu("&File");
    FileMenu->addSeparator();
    FileMenu->addAction("Quit", qApp, SLOT (quit()), QKeySequence(tr("Ctrl+Q")));

    // Add menu items
    QMenu* ViewMenu = menuBar()->addMenu("&View");
    ViewMenu->addAction("Front view",       Object_GL_PI, SLOT (FrontView()),  QKeySequence(tr("Ctrl+f")));
    ViewMenu->addAction("Rear view",        Object_GL_PI, SLOT (RearView()),   QKeySequence(tr("Ctrl+e")));
    ViewMenu->addAction("Left view",        Object_GL_PI, SLOT (LeftView()),   QKeySequence(tr("Ctrl+l")));
    ViewMenu->addAction("Right view",       Object_GL_PI, SLOT (RightView()),  QKeySequence(tr("Ctrl+r")));
    ViewMenu->addAction("Top view",         Object_GL_PI, SLOT (TopView()),    QKeySequence(tr("Ctrl+t")));
    ViewMenu->addAction("Bottom view",      Object_GL_PI, SLOT (BottomView()), QKeySequence(tr("Ctrl+b")));
    FileMenu->addSeparator();
    ViewMenu->addAction("Isometric",        Object_GL_PI, SLOT (IsometricView()), QKeySequence(tr("Ctrl+i")));
    QMenu* AboutMenu = menuBar()->addMenu("?");
    AboutMenu->addAction("About Convert_STL_2_Cube", this, SLOT (handleAbout()));
    printf("oi\n");
    imu_msg_copy = imu_msg;
    pose_pi_copy = pose_pi;
    printf("oi2\n");
	//Timer (used for repainting the GL Window and treating IPC every 50 ms)
    QTimer *timerDisplay = new QTimer();
    timerDisplay->connect(timerDisplay, SIGNAL(timeout()),this, SLOT(onTimer_UpdateDisplayPI()));
    timerDisplay->start(75);
    printf("oi3\n");
    // Timer for reading raw data (every 10ms)
    QTimer *timerArduino = new QTimer();
    timerArduino->connect(timerArduino, SIGNAL(timeout()),this, SLOT(onTimer_ReadDataPI()));
    timerArduino->start(10);
    printf("oi4\n");

}


MainWindow::~MainWindow()
{
}

MainWindowPI::~MainWindowPI()
{
}



void 
MainWindow::resizeEvent(QResizeEvent *)
{
    Object_GL->resize(centralWidget->width(),centralWidget->height());
    gridLayoutWidget->setGeometry(QRect(0, 0, centralWidget->width(), centralWidget->height()));
}

void
MainWindowPI::resizeEvent(QResizeEvent *)
{
    Object_GL_PI->resize(centralWidget->width(),centralWidget->height());
    gridLayoutWidget->setGeometry(QRect(0, 0, centralWidget->width(), centralWidget->height()));
}

void 
MainWindow::onTimer_UpdateDisplay()
{
    Object_GL->updateGL();
	carmen_ipc_sleep(0.033333333);
}

void 
MainWindowPI::onTimer_UpdateDisplayPI()
{
    Object_GL_PI->updateGL();
	carmen_ipc_sleep(0.033333333);
}

// Timer event : get raw data from Arduino
void 
MainWindow::onTimer_ReadData()
{
//	static double pitch = -M_PI;

	Object_GL->setAcceleromter(data_copy->m_acc.x, data_copy->m_acc.y , data_copy->m_acc.z);
	Object_GL->setGyroscope(data_copy->m_gyr.x, data_copy->m_gyr.y, data_copy->m_gyr.z);
	Object_GL->setMagnetometer(data_copy->m_mag.x, data_copy->m_mag.y, data_copy->m_mag.z);

//	pose_copy->m_pitch = pitch;
//	pitch += 0.1;
//	if (pitch > M_PI)
//		pitch = -M_PI;

	Object_GL->setAngles(pose_copy->m_roll * 180. / M_PI , pose_copy->m_pitch * 180. / M_PI, pose_copy->m_yaw  * 180. / M_PI);
}

void 
MainWindowPI::onTimer_ReadDataPI()
{
//	static double pitch = -M_PI;

	Object_GL_PI->setAcceleromter(imu_msg_copy->imu_vector.accel.x, imu_msg_copy->imu_vector.accel.y , imu_msg_copy->imu_vector.accel.z);
	Object_GL_PI->setGyroscope(imu_msg_copy->imu_vector.gyro.x, imu_msg_copy->imu_vector.gyro.y, imu_msg_copy->imu_vector.gyro.z);
	Object_GL_PI->setMagnetometer(imu_msg_copy->imu_vector.magnetometer.x, imu_msg_copy->imu_vector.magnetometer.y, imu_msg_copy->imu_vector.magnetometer.z);

//	pose_copy->m_pitch = pitch;
//	pitch += 0.1;
//	if (pitch > M_PI)
//		pitch = -M_PI;

	Object_GL_PI->setAngles(pose_pi_copy->m_roll * 180. / M_PI , pose_pi_copy->m_pitch * 180. / M_PI, pose_pi_copy->m_yaw  * 180. / M_PI);
}


// Open the 'about' dialog box
void 
MainWindow::handleAbout()
{
    QMessageBox::information(this,"About OpenGL Frame","<H2>OpenGL Frame</H2>2011<BR>Supported by the Cart-O-Matic project (ANR CAROTTE)<BR>University of Angers (France,Europe)<BR>Designed by Philippe Lucidarme <BR> Contact: philippe.lucidarme@univ-angers.fr. ");
}

// Open the 'about' dialog box
/*void
MainWindowPI::handleAbout()
{
    QMessageBox::information(this,"About OpenGL Frame","<H2>OpenGL Frame</H2>2011<BR>Supported by the Cart-O-Matic project (ANR CAROTTE)<BR>University of Angers (France,Europe)<BR>Designed by Philippe Lucidarme <BR> Contact: philippe.lucidarme@univ-angers.fr. ");
}*/

