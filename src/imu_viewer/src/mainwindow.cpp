#include "mainwindow.h"

carmen_xsens_global_message* data1;
// Constructor of the main window
// Create window properties, menu etc ...
MainWindow::MainWindow(QWidget *parent,int w, int h, carmen_xsens_global_message* data)
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
	data1 = data;

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
       // Convert into floats
       float ax,ay,az;
       float gx,gy,gz;
       float mx,my,mz;

        
		ax = data1->m_acc.x;
		ay = data1->m_acc.y;
		az = data1->m_acc.z;

		gx = data1->m_gyr.x;
		gy = data1->m_gyr.x;
		gz = data1->m_gyr.x;
/*
		mx = data1->mag.x;
		my = data1->mag.y;
		mz = data1->mag.z;
*/
	   Object_GL->setAcceleromter(ax,ay,az);
       Object_GL->setGyroscope(gx,gy,gz);
 //      Object_GL->setMagnetometer(mx,my,mz);

//       MadgwickAHRSupdate(gx,gy,gz,ax,ay,az,mx,my,mz);
      MadgwickAHRSupdateIMU(gx,gy,gz,ax,ay,az);
//        std::cout << q0 << " \t" << q1 << " \t" << q2 << " \t" << q3 << std::endl;

       double R11 = 2.*q0*q0 -1 +2.*q1*q1;
       double R21 = 2.*(q1*q2 - q0*q3);
       double R31 = 2.*(q1*q3 + q0*q2);
       double R32 = 2.*(q2*q3 - q0*q1);
       double R33 = 2.*q0*q0 -1 +2.*q3*q3;

       double phi = atan2(R32, R33 );
       double theta = -atan(R31 / sqrt(1-R31*R31) );
       double psi = atan2(R21, R11 );

       //std::cout << R31 << "\t" << phi*180./M_PI << "\t" << theta*180./M_PI << "\t" << psi*180./M_PI << std::endl;
       Object_GL->setAngles(data1->m_roll* 180. / M_PI  , data1->m_pitch * 180. / M_PI , data1->m_yaw * 180./M_PI );
	std::cout << "x  " << data1->m_roll* 180. / M_PI <<  "\t" << "y  " << data1->m_pitch*180./M_PI  <<  "\t" << "z  "<<  "\t" << data1->m_yaw*180./M_PI << "\t"<< std::endl;
       /*std::cout << ax << "\t" << ay << "\t" << az << "\t";
       std::cout << gx << "\t" << gy << "\t" << gz << "\t";
       std::cout << mx << "\t" << my << "\t" << mz << "\t";
       std::cout << std::endl;*/
		// Timer for reading raw data (every 10ms)
    
		
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


