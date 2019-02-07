#include "mainwindow.h"

carmen_xsens_global_quat_message* data_copy;
carmen_xsens_global_message* pose_copy;

// Constructor of the main window
// Create window properties, menu etc ...
MainWindow::MainWindow(QWidget *parent,int w, int h, carmen_xsens_global_quat_message* data, carmen_xsens_global_message* pose)
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





// Desctructor
MainWindow::~MainWindow()
{}




// On resize event, the items in the window are resized
void 
MainWindow::resizeEvent(QResizeEvent *)
{
    Object_GL->resize(centralWidget->width(),centralWidget->height());
    gridLayoutWidget->setGeometry(QRect(0, 0, centralWidget->width(), centralWidget->height()));
}




// Timer event : repain the Open Gl window
void 
MainWindow::onTimer_UpdateDisplay()
{
    Object_GL->updateGL();
	carmen_ipc_sleep(0.033333333);
}

#define         RATIO_ACC       (4./32767.)
#define         RATIO_GYRO      ((1000./32767.)*(M_PI/180.))
//#define         RATIO_GYRO      (1000./32767.)
#define         RATIO_MAG       (48./32767.)

// Timer event : get raw data from Arduino
void 
MainWindow::onTimer_ReadData()
{

	   Object_GL->setAcceleromter(data_copy->m_acc.x, data_copy->m_acc.y , data_copy->m_acc.z);
       Object_GL->setGyroscope(data_copy->m_gyr.x, data_copy->m_gyr.y, data_copy->m_gyr.z);
       Object_GL->setMagnetometer(data_copy->m_mag.x, data_copy->m_mag.y, data_copy->m_mag.z);


       //std::cout << R31 << "\t" << phi*180./M_PI << "\t" << theta*180./M_PI << "\t" << psi*180./M_PI << std::endl;
       Object_GL->setAngles(pose_copy->m_roll * 180. / M_PI , pose_copy->m_pitch * 180. / M_PI, pose_copy->m_yaw  * 180. / M_PI);
       std::cout << "x  " << pose_copy->m_roll* 180. / M_PI <<  "\t" << "y  " << pose_copy->m_pitch*180./M_PI  <<  "\t" << "z  "<<  "\t"
    		   << pose_copy->m_yaw*180./M_PI << "\t"<< std::endl;
}



// Open the 'about' dialog box
void 
MainWindow::handleAbout()
{
    QMessageBox::information(this,"About OpenGL Frame","<H2>OpenGL Frame</H2>2011<BR>Supported by the Cart-O-Matic project (ANR CAROTTE)<BR>University of Angers (France,Europe)<BR>Designed by Philippe Lucidarme <BR> Contact: philippe.lucidarme@univ-angers.fr. ");
}


// Connect to the serial device (Arduino)

bool
MainWindow::connect()
{
    return true;
}


