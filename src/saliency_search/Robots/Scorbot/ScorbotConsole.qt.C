#include "Robots/Scorbot/ScorbotConsole.qt.H"
#include "QtGui/qboxlayout.h"
#include "Image/DrawOps.H"

ScorbotConsole::ScorbotConsole(OptionManager& mgr, 
    const std::string& descrName,
    const std::string& tagName) :
	ModelComponent(mgr, descrName, tagName)
{
	jointStringMap[ScorbotIce::Base]     = "Base";
	jointStringMap[ScorbotIce::Shoulder] = "Shoulder";
	jointStringMap[ScorbotIce::Elbow]    = "Elbow";
	jointStringMap[ScorbotIce::Wrist1]   = "Wrist1";
	jointStringMap[ScorbotIce::Wrist2]   = "Wrist2";
	jointStringMap[ScorbotIce::Gripper]  = "Gripper";
	jointStringMap[ScorbotIce::Slider]   = "Slider";

	itsSettings = new QSettings("iLab", "ScorbotConsole");

	itsMaxPlotLength = 100;

	int IceError = false;
	try
	{
		int argc = 1;
		char* argv[1];
		argv[0] = new char[128];
		sprintf(argv[0], "app-ScorbotConsole");
		ic = Ice::initialize(argc, argv);
		Ice::ObjectPrx base = ic->stringToProxy(
				"ScorbotServer:default -p 10000 -h ihead");
		itsScorbot = ScorbotIce::ScorbotPrx::checkedCast(base);
		if(!itsScorbot)
			throw "Invalid Proxy";
	}
	catch(const Ice::Exception& ex)
	{
		std::cerr << ex << std::endl;
		IceError = true;
	}
	catch(const char* msg)
	{
		std::cerr << msg << std::endl;
		IceError = false;
	}

	if(IceError)
		exit(-1);


	itsScorbotThread = new ScorbotInterfaceThread(this,&itsScorbot);
	connect(itsScorbotThread, SIGNAL(gotGravityCompensation(float)), this, SLOT(gotGravityCompensation(float)));
	connect(itsScorbotThread, SIGNAL(gotEncoderVal(int, int)), this, SLOT(gotEncoderVal(int, int)));
	connect(itsScorbotThread, SIGNAL(gotPWMVal(int, float)), this, SLOT(gotPWMVal(int, float)));
	connect(itsScorbotThread, SIGNAL(gotTargetPos(int, int)), this, SLOT(gotTargetPos(int, int)));
	connect(itsScorbotThread, SIGNAL(updatedAllValues()), this, SLOT(updatePlots()));
}

// ######################################################################
ScorbotConsole::~ScorbotConsole()
{
	if(ic)
		ic->destroy();
}


// ######################################################################
void ScorbotConsole::loadParams()
{
	//Save all of our settings
	for(int jointIdx=(int)ScorbotIce::Base; jointIdx<=(int)ScorbotIce::Slider; jointIdx++)
	{
		ScorbotIce::JointType joint = (ScorbotIce::JointType)jointIdx;
		QString jointName = jointStringMap[joint];

		if(!itsSettings->contains(jointName+"/pGain")) continue;

		float pGain     = itsSettings->value(jointName + "/pGain").toDouble();
		float iGain     = itsSettings->value(jointName + "/iGain").toDouble();
		float dGain     = itsSettings->value(jointName + "/dGain").toDouble();
		float maxI      = itsSettings->value(jointName + "/maxI").toDouble();
		float maxPWM    = itsSettings->value(jointName + "/maxPWM").toDouble();
		float pwmOffset = itsSettings->value(jointName + "/pwmOffset").toDouble();

		itsScorbot->setControlParams(joint, pGain, iGain, dGain, maxI, maxPWM, pwmOffset);
	}

	int32 foreArmMass, upperArmMass;
	float gravityScale;
	foreArmMass  = itsSettings->value("ForeArmMass").toInt();
	upperArmMass = itsSettings->value("UpperArmMass").toInt();
	gravityScale = itsSettings->value("GravityScale").toDouble();
	itsScorbot->setGravityParameters(upperArmMass, foreArmMass, gravityScale);


}

// ######################################################################
void ScorbotConsole::saveParams()
{
	//Save all of our settings
	for(int jointIdx=(int)ScorbotIce::Base; jointIdx<=(int)ScorbotIce::Slider; jointIdx++)
	{
		ScorbotIce::JointType joint = (ScorbotIce::JointType)jointIdx;
		float pGain, iGain, dGain, maxI, maxPWM, pwmOffset;
		itsScorbot->getPIDVals(joint, pGain, iGain, dGain, maxI, maxPWM, pwmOffset);

		QString jointName = jointStringMap[joint];

		itsSettings->setValue(jointName + "/pGain",     pGain);
		itsSettings->setValue(jointName + "/iGain",     iGain);
		itsSettings->setValue(jointName + "/dGain",     dGain);
		itsSettings->setValue(jointName + "/maxI",      maxI);
		itsSettings->setValue(jointName + "/maxPWM",    maxPWM);
		itsSettings->setValue(jointName + "/pwmOffset", pwmOffset);
	}
	int32 foreArmMass, upperArmMass;
	float gravityScale;
	itsScorbot->getGravityParameters(upperArmMass, foreArmMass, gravityScale);
	itsSettings->setValue("ForeArmMass", foreArmMass);
	itsSettings->setValue("UpperArmMass", upperArmMass);
	itsSettings->setValue("GravityScale", gravityScale);
	itsSettings->sync();
	delete itsSettings;

}

// ######################################################################
void ScorbotConsole::stop1()
{

	//saveParams();

	itsScorbot->setEnabled(false);
	itsScorbotThread->running = false;
	LINFO("Waiting for shutdown...");
	while(!itsScorbotThread->isFinished())
		usleep(10000);
}

void ScorbotConsole::start2() 
{
	//loadParams();

  QWidget* centralWidget = new QWidget(this);
	QVBoxLayout *mainVertLayout  = new QVBoxLayout(centralWidget);
  QHBoxLayout *mainHorizLayout = new QHBoxLayout(centralWidget);
	mainVertLayout->addLayout(mainHorizLayout);

	//Create joint labels
	QGridLayout *controlLayout = new QGridLayout(centralWidget);
	controlLayout->setVerticalSpacing(0);

	int row    = 0;
	int column = 0;

	//Create the horizontal control names
	QLabel* jointLbl    = new QLabel("<b>Joint</b>",           this);
	jointLbl->setAlignment(Qt::AlignHCenter);    
	QLabel* pwmLbl      = new QLabel("<b>PWM<b>",             this);
	pwmLbl->setAlignment(Qt::AlignHCenter);      
	QLabel* encoderLbl  = new QLabel("<b>Encoder Val<b>",     this);
	encoderLbl->setAlignment(Qt::AlignHCenter);  
	QLabel* desiredLbl  = new QLabel("<b>Desired Encoder<b>", this);
	desiredLbl->setAlignment(Qt::AlignHCenter);  
	QLabel* durationLbl = new QLabel("<b>Duration<b>",       this);
	durationLbl->setAlignment(Qt::AlignHCenter); 
	QLabel* setLbl      = new QLabel("<b>Set Position<b>",   this);
	setLbl->setAlignment(Qt::AlignHCenter);      
	
	controlLayout->addWidget(jointLbl,    row, column++);
	controlLayout->addWidget(pwmLbl,      row, column++);
	controlLayout->addWidget(encoderLbl,  row, column++);
	controlLayout->addWidget(desiredLbl,  row, column++);
	controlLayout->addWidget(durationLbl, row, column++);
	controlLayout->addWidget(setLbl,      row, column++);
	controlLayout->setRowStretch(0, 0);

	//Create the vertical axis labels
	row=1;	column=0;
	controlLayout->addWidget(new QLabel("<b>Base<b>", this)    , row++, column);
	controlLayout->addWidget(new QLabel("<b>Shoulder<b>", this), row++, column);
	controlLayout->addWidget(new QLabel("<b>Elbow<b>", this)   , row++, column);
	controlLayout->addWidget(new QLabel("<b>Wrist1<b>", this)  , row++, column);
	controlLayout->addWidget(new QLabel("<b>Wrist2<b>", this)  , row++, column);
	controlLayout->addWidget(new QLabel("<b>Gripper<b>", this) , row++, column);
	controlLayout->addWidget(new QLabel("<b>Slider<b>", this)  , row++, column);
	
	//Create the pwm labels
	row=1;	column++;
	pwmLbls[ScorbotIce::Base]     = new QLabel(centralWidget);
	controlLayout->addWidget(pwmLbls[ScorbotIce::Base],     row++, column);
	pwmLbls[ScorbotIce::Shoulder] = new QLabel(centralWidget);
	controlLayout->addWidget(pwmLbls[ScorbotIce::Shoulder], row++, column);
	pwmLbls[ScorbotIce::Elbow]    = new QLabel(centralWidget);
	controlLayout->addWidget(pwmLbls[ScorbotIce::Elbow],    row++, column);
	pwmLbls[ScorbotIce::Wrist1]   = new QLabel(centralWidget);
	controlLayout->addWidget(pwmLbls[ScorbotIce::Wrist1],   row++, column);
	pwmLbls[ScorbotIce::Wrist2]   = new QLabel(centralWidget);
	controlLayout->addWidget(pwmLbls[ScorbotIce::Wrist2],   row++, column);
	pwmLbls[ScorbotIce::Gripper]  = new QLabel(centralWidget);
	controlLayout->addWidget(pwmLbls[ScorbotIce::Gripper],  row++, column);
	pwmLbls[ScorbotIce::Slider]   = new QLabel(centralWidget);
	controlLayout->addWidget(pwmLbls[ScorbotIce::Slider],   row++, column);
	for(QMap<ScorbotIce::JointType, QLabel*>::iterator it=pwmLbls.begin(); it!=pwmLbls.end(); ++it)
	{ it.value()->setMaximumWidth(50); it.value()->setMinimumWidth(50); }
	
	//Create the encoder labels
	row=1; column++;
	encoderLbls[ScorbotIce::Base]     = new QLabel(centralWidget);
	controlLayout->addWidget(encoderLbls[ScorbotIce::Base],     row++, column);
	encoderLbls[ScorbotIce::Shoulder] = new QLabel(centralWidget);
	controlLayout->addWidget(encoderLbls[ScorbotIce::Shoulder], row++, column);
	encoderLbls[ScorbotIce::Elbow]    = new QLabel(centralWidget);
	controlLayout->addWidget(encoderLbls[ScorbotIce::Elbow],    row++, column);
	encoderLbls[ScorbotIce::Wrist1]   = new QLabel(centralWidget);
	controlLayout->addWidget(encoderLbls[ScorbotIce::Wrist1],   row++, column);
	encoderLbls[ScorbotIce::Wrist2]   = new QLabel(centralWidget);
	controlLayout->addWidget(encoderLbls[ScorbotIce::Wrist2],   row++, column);
	encoderLbls[ScorbotIce::Gripper]  = new QLabel(centralWidget);
	controlLayout->addWidget(encoderLbls[ScorbotIce::Gripper],  row++, column);
	encoderLbls[ScorbotIce::Slider]   = new QLabel(centralWidget);
	controlLayout->addWidget(encoderLbls[ScorbotIce::Slider],   row++, column);
	for(QMap<ScorbotIce::JointType, QLabel*>::iterator it=encoderLbls.begin(); it!=encoderLbls.end(); ++it)
	{ it.value()->setMaximumWidth(50); it.value()->setMinimumWidth(50); }

	//Create the encoder edits
	row=1; column++;
	encoderEdits[ScorbotIce::Base]     = new QLineEdit(centralWidget);
	controlLayout->addWidget(encoderEdits[ScorbotIce::Base],     row++, column);
	encoderEdits[ScorbotIce::Shoulder] = new QLineEdit(centralWidget);
	controlLayout->addWidget(encoderEdits[ScorbotIce::Shoulder], row++, column);
	encoderEdits[ScorbotIce::Elbow]    = new QLineEdit(centralWidget);
	controlLayout->addWidget(encoderEdits[ScorbotIce::Elbow],    row++, column);
	encoderEdits[ScorbotIce::Wrist1]   = new QLineEdit(centralWidget);
	controlLayout->addWidget(encoderEdits[ScorbotIce::Wrist1],   row++, column);
	encoderEdits[ScorbotIce::Wrist2]   = new QLineEdit(centralWidget);
	controlLayout->addWidget(encoderEdits[ScorbotIce::Wrist2],   row++, column);
	encoderEdits[ScorbotIce::Gripper]  = new QLineEdit(centralWidget);
	controlLayout->addWidget(encoderEdits[ScorbotIce::Gripper],  row++, column);
	encoderEdits[ScorbotIce::Slider]   = new QLineEdit(centralWidget);
	controlLayout->addWidget(encoderEdits[ScorbotIce::Slider],   row++, column);

	//Create the duration edits
	row=1; column++;
	durationEdits[ScorbotIce::Base]     = new QLineEdit(centralWidget);
	controlLayout->addWidget(durationEdits[ScorbotIce::Base],     row++, column);
	durationEdits[ScorbotIce::Shoulder] = new QLineEdit(centralWidget);
	controlLayout->addWidget(durationEdits[ScorbotIce::Shoulder], row++, column);
	durationEdits[ScorbotIce::Elbow]    = new QLineEdit(centralWidget);
	controlLayout->addWidget(durationEdits[ScorbotIce::Elbow],    row++, column);
	durationEdits[ScorbotIce::Wrist1]   = new QLineEdit(centralWidget);
	controlLayout->addWidget(durationEdits[ScorbotIce::Wrist1],   row++, column);
	durationEdits[ScorbotIce::Wrist2]   = new QLineEdit(centralWidget);
	controlLayout->addWidget(durationEdits[ScorbotIce::Wrist2],   row++, column);
	durationEdits[ScorbotIce::Gripper]  = new QLineEdit(centralWidget);
	controlLayout->addWidget(durationEdits[ScorbotIce::Gripper],  row++, column);
	durationEdits[ScorbotIce::Slider]   = new QLineEdit(centralWidget);
	controlLayout->addWidget(durationEdits[ScorbotIce::Slider],   row++, column);

	//Create the set position buttons
	row=1; column++;
	QSignalMapper* posButtonMapper = new QSignalMapper(this);
	setPosButtons[ScorbotIce::Base]     = new QPushButton("Set Position", centralWidget);
	controlLayout->addWidget(setPosButtons[ScorbotIce::Base],     row++, column);
	posButtonMapper->setMapping(setPosButtons[ScorbotIce::Base], (int)ScorbotIce::Base);
	connect(setPosButtons[ScorbotIce::Base], SIGNAL(pressed()), posButtonMapper, SLOT(map()));

	setPosButtons[ScorbotIce::Shoulder] = new QPushButton("Set Position", centralWidget);
	controlLayout->addWidget(setPosButtons[ScorbotIce::Shoulder], row++, column);
	posButtonMapper->setMapping(setPosButtons[ScorbotIce::Shoulder], (int)ScorbotIce::Shoulder);
	connect(setPosButtons[ScorbotIce::Shoulder], SIGNAL(pressed()), posButtonMapper, SLOT(map()));

	setPosButtons[ScorbotIce::Elbow]    = new QPushButton("Set Position", centralWidget);
	controlLayout->addWidget(setPosButtons[ScorbotIce::Elbow],    row++, column);
	posButtonMapper->setMapping(setPosButtons[ScorbotIce::Elbow], (int)ScorbotIce::Elbow);
	connect(setPosButtons[ScorbotIce::Elbow], SIGNAL(pressed()), posButtonMapper, SLOT(map()));
	
	setPosButtons[ScorbotIce::Wrist1]   = new QPushButton("Set Position", centralWidget);
	controlLayout->addWidget(setPosButtons[ScorbotIce::Wrist1],   row++, column);
	posButtonMapper->setMapping(setPosButtons[ScorbotIce::Wrist1], (int)ScorbotIce::Wrist1);
	connect(setPosButtons[ScorbotIce::Wrist1], SIGNAL(pressed()), posButtonMapper, SLOT(map()));

	setPosButtons[ScorbotIce::Wrist2]   = new QPushButton("Set Position", centralWidget);
	controlLayout->addWidget(setPosButtons[ScorbotIce::Wrist2],   row++, column);
	posButtonMapper->setMapping(setPosButtons[ScorbotIce::Wrist2], (int)ScorbotIce::Wrist2);
	connect(setPosButtons[ScorbotIce::Wrist2], SIGNAL(pressed()), posButtonMapper, SLOT(map()));

	setPosButtons[ScorbotIce::Gripper]  = new QPushButton("Set Position", centralWidget);
	controlLayout->addWidget(setPosButtons[ScorbotIce::Gripper],  row++, column);
	posButtonMapper->setMapping(setPosButtons[ScorbotIce::Gripper], (int)ScorbotIce::Gripper);
	connect(setPosButtons[ScorbotIce::Gripper], SIGNAL(pressed()), posButtonMapper, SLOT(map()));

	setPosButtons[ScorbotIce::Slider]   = new QPushButton("Set Position", centralWidget);
	controlLayout->addWidget(setPosButtons[ScorbotIce::Slider],   row++, column);
	posButtonMapper->setMapping(setPosButtons[ScorbotIce::Slider], (int)ScorbotIce::Slider);
	connect(setPosButtons[ScorbotIce::Slider], SIGNAL(pressed()), posButtonMapper, SLOT(map()));

	connect(posButtonMapper, SIGNAL(mapped(int)), this, SLOT(setPosition(int)));

	//Gravity Compensation Label
	controlLayout->addWidget(new QLabel("<b>Gravity Compensation<b>", this), ++row, 0);
	gravityCompLbl = new QLabel(this);
	gravityCompLbl->setMaximumWidth(50); gravityCompLbl->setMinimumWidth(50);
	controlLayout->addWidget(gravityCompLbl, row++, 1);

  //Create the extra buttons
	row=1; column=7;
	//Reset Encoder Button
	QPushButton* resetEncoderButton = new QPushButton("Reset Encoders",centralWidget);
	controlLayout->addWidget(resetEncoderButton, row++, column);
	connect(resetEncoderButton, SIGNAL(pressed()), this, SLOT(resetEncoders()));

	//Enable Motors Button
	enableMotorsButton = new QPushButton("Enable Motors", centralWidget);
	controlLayout->addWidget(enableMotorsButton, row++, column);
	connect(enableMotorsButton, SIGNAL(pressed()), this, SLOT(toggleMotorsEnabled()));
	itsMotorsEnabled = false;
	itsScorbot->setEnabled(false);

	//Print Position Code Buton
	QPushButton* printPosCodeButton = new QPushButton("Print Position Code", centralWidget);
	controlLayout->addWidget(printPosCodeButton, row++, column);
	connect(printPosCodeButton, SIGNAL(pressed()), this, SLOT(printPosCode()));

	mainHorizLayout->addLayout(controlLayout);

	mainHorizLayout->addSpacing(10);
	mainHorizLayout->addStretch(10);

	//Make the PID layout
	row=1; column = 0;
	QGridLayout *pidLayout = new QGridLayout(this);
	itsAxisComboBox = new QComboBox(this);
	itsAxisComboBox->addItem("Base",     0);
	itsAxisComboBox->addItem("Shoulder", 1);
	itsAxisComboBox->addItem("Elbow",    2);
	itsAxisComboBox->addItem("Wrist1",   3);
	itsAxisComboBox->addItem("Wrist2",   4);
	itsAxisComboBox->addItem("Gripper",  5);
	itsAxisComboBox->addItem("Slider",   6);
	connect(itsAxisComboBox, SIGNAL(activated(int)), this, SLOT(pidAxisSelected(int)));
	pidLayout->addWidget(itsAxisComboBox, 0, 1);

	pGainEdit     = new QLineEdit(this);
	pidLayout->addWidget(new QLabel("P Gain", this),     row, 0);
	pidLayout->addWidget(pGainEdit,     row++, 1);

	iGainEdit     = new QLineEdit(this);
	pidLayout->addWidget(new QLabel("I Gain", this),     row, 0);
	pidLayout->addWidget(iGainEdit,     row++, 1);

	dGainEdit     = new QLineEdit(this);
	pidLayout->addWidget(new QLabel("D Gain", this),     row, 0);
	pidLayout->addWidget(dGainEdit,     row++, 1);
	maxIEdit      = new QLineEdit(this);
	pidLayout->addWidget(new QLabel("Max I", this),      row, 0);
	pidLayout->addWidget(maxIEdit,      row++, 1);
	maxPWMEdit    = new QLineEdit(this);
	pidLayout->addWidget(new QLabel("Max PWM", this),    row, 0);
	pidLayout->addWidget(maxPWMEdit,    row++, 1);

	pwmOffsetEdit = new QLineEdit(this);
	pidLayout->addWidget(new QLabel("PWM Offset", this), row, 0);
	pidLayout->addWidget(pwmOffsetEdit, row++, 1);

	foreArmMassEdit      = new QLineEdit(this);
	pidLayout->addWidget(new QLabel("Fore Arm Mass", this), row, 0);
	pidLayout->addWidget(foreArmMassEdit, row++, 1);

	upperArmMassEdit     = new QLineEdit(this); 
	pidLayout->addWidget(new QLabel("Upper Arm Mass", this), row, 0);
	pidLayout->addWidget(upperArmMassEdit, row++, 1);

	gravityCompScaleEdit = new QLineEdit(this);
	pidLayout->addWidget(new QLabel("Gravity Scale", this), row, 0);
	pidLayout->addWidget(gravityCompScaleEdit, row++, 1);

	QPushButton* setPIDButton = new QPushButton("Set PID", this);
	connect(setPIDButton, SIGNAL(pressed()), this, SLOT(setPIDVals()));
	pidLayout->addWidget(setPIDButton, row++, 1);
	mainHorizLayout->addLayout(pidLayout);
	pidAxisSelected(0);

	//Create the plot
	itsGraphicsScene = new QGraphicsScene(this);
	itsGraphicsView = new QGraphicsView(itsGraphicsScene);
	itsImageDisplay = new ImageGraphicsItem;
	itsGraphicsScene->addItem(itsImageDisplay);
	itsGraphicsView->show();
	mainVertLayout->addWidget(itsGraphicsView);
	itsGraphicsView->setMinimumSize(640, 550);



  //Set the main layout to display
  setCentralWidget(centralWidget);
  centralWidget->setLayout(mainVertLayout);

	itsScorbotThread->start();
}

// ######################################################################
void ScorbotConsole::resetEncoders()
{
	itsScorbot->resetEncoders();
}

// ######################################################################
void ScorbotConsole::gotEncoderVal(int joint, int val)
{
	
	ScorbotIce::JointType _joint = (ScorbotIce::JointType)joint;
	encoderLbls[_joint]->setNum(val);
	addData(jointStringMap[_joint]+"_Encoder", val);
}

// ######################################################################
void ScorbotConsole::addData(QString plotName, double data)
{
	plotData[plotName].push_back(data);
	if(plotData[plotName].size() > itsMaxPlotLength)
		plotData[plotName].erase(plotData[plotName].begin());
}

// ######################################################################
void ScorbotConsole::gotPWMVal(int joint, float val)
{
	ScorbotIce::JointType _joint = (ScorbotIce::JointType)joint;
	pwmLbls[_joint]->setNum(val);
	addData(jointStringMap[_joint]+"_PWM", val*1000);
}

// ######################################################################
void ScorbotConsole::gotTargetPos(int joint, int val)
{
	ScorbotIce::JointType _joint = (ScorbotIce::JointType)joint;
	addData(jointStringMap[_joint]+"_TargetPos", val);
}

// ######################################################################
void ScorbotConsole::pidAxisSelected(int index)
{
	ScorbotIce::JointType joint = (ScorbotIce::JointType)itsAxisComboBox->itemData(index).toInt();

	itsScorbotThread->setSelectedJoint(joint);

	float pGain, iGain, dGain, maxI, maxPWM, pwmOffset, gravityCompScale;
	int32 foreArmMass, upperArmMass;
	itsScorbot->getPIDVals(joint, pGain, iGain, dGain, maxI, maxPWM, pwmOffset);
	usleep(1000);
	itsScorbot->getGravityParameters(upperArmMass, foreArmMass, gravityCompScale);

	pGainEdit->setText(QString::number(pGain));
	iGainEdit->setText(QString::number(iGain));
	dGainEdit->setText(QString::number(dGain));
	maxIEdit->setText(QString::number(maxI));
	maxPWMEdit->setText(QString::number(maxPWM));
	pwmOffsetEdit->setText(QString::number(pwmOffset));
	foreArmMassEdit->setText(QString::number(foreArmMass));
	upperArmMassEdit->setText(QString::number(upperArmMass));
	gravityCompScaleEdit->setText(QString::number(gravityCompScale));

	plotDataColor.clear();
	plotDataCheck.clear();
	plotData.clear();

	//Initialize Plot Colors
	plotDataColor[jointStringMap[joint]+"_Encoder"]   = PixRGB<byte> (255,0,0);
	plotDataColor[jointStringMap[joint]+"_Desired"]   = PixRGB<byte> (0,0,255);
	plotDataColor[jointStringMap[joint]+"_PWM"]       = PixRGB<byte> (0,255,0);
	plotDataColor[jointStringMap[joint]+"_TargetPos"] = PixRGB<byte> (0,255,255);
	plotDataColor["Gravity_Comp"] 										= PixRGB<byte> (255,255,255);
	plotDataCheck[jointStringMap[joint]+"_Encoder"]   = true;
	plotDataCheck[jointStringMap[joint]+"_Desired"]   = true;
	plotDataCheck[jointStringMap[joint]+"_PWM"]       = true;
	plotDataCheck[jointStringMap[joint]+"_TargetPos"] = true;
	plotDataCheck["Gravity_Comp"] 										= true;
}

// ######################################################################
void ScorbotConsole::setPIDVals()
{
	ScorbotIce::JointType joint = (ScorbotIce::JointType)itsAxisComboBox->itemData(itsAxisComboBox->currentIndex()).toInt();

	itsScorbot->setControlParams(joint,
			pGainEdit->text().toDouble(),
			iGainEdit->text().toDouble(),
			dGainEdit->text().toDouble(),
			maxIEdit->text().toDouble(),
			maxPWMEdit->text().toDouble(),
			pwmOffsetEdit->text().toDouble()
			);

	itsScorbot->setGravityParameters(
			upperArmMassEdit->text().toInt(),
			foreArmMassEdit->text().toInt(),
			gravityCompScaleEdit->text().toDouble());
}

// ######################################################################
void ScorbotConsole::setPosition(int joint)
{
	ScorbotIce::JointType _joint = (ScorbotIce::JointType)joint;
	itsScorbot->setJoint(_joint, encoderEdits[_joint]->text().toInt(), durationEdits[_joint]->text().toInt()); 
	plotData[jointStringMap[_joint]+"_Desired"] = (std::vector<float>(itsMaxPlotLength, encoderEdits[_joint]->text().toInt()));
}

// ######################################################################
void ScorbotConsole::toggleMotorsEnabled()
{
	itsMotorsEnabled = !itsMotorsEnabled;
	itsScorbot->setEnabled(itsMotorsEnabled);
	if(itsMotorsEnabled)
		enableMotorsButton->setText("Disable Motors");
	else
		enableMotorsButton->setText("Enable Motors");
}

// ######################################################################
void ScorbotConsole::printPosCode()
{
	std::cout << "------------------POSITION------------------" << std::endl;
	ScorbotIce::encoderValsType pos = itsScorbot->getEncoders();
	for(ScorbotIce::encoderValsType::iterator posIt=pos.begin(); posIt!=pos.end(); ++posIt)
		std::cout << "encoders[ScorbotIce::" << jointStringMap[posIt->first].toStdString() << "] = " << posIt->second << ";" << std::endl;
	std::cout << "--------------------------------------------" << std::endl;
}

// ######################################################################
void ScorbotConsole::plotLine(std::vector<float> data, PixRGB<byte> color)
{
	if(data.size() == 0) return;

	QPainterPath path;
	path.moveTo(0, data[0]);
	for(size_t i=1; i<data.size(); i++)
		path.lineTo(i, data[i]);
	itsGraphicsScene->addPath(path, QPen(QColor(color.red(), color.green(), color.blue())));
}

// ######################################################################
void ScorbotConsole::updatePlots()
{
	itsGraphicsView->setBackgroundBrush(QBrush(Qt::darkGray));

	//Clear all previous plots
	QList<QGraphicsItem*> items = itsGraphicsScene->items();
	for(int i=0; i<items.size(); i++)
	{
		itsGraphicsScene->removeItem(items[i]);
		delete items[i];
	}

	int textPos = -5000;
	int textOffset=500;
	QMap<QString, bool>::iterator dataCheckIt = plotDataCheck.begin();
	for(; dataCheckIt != plotDataCheck.end(); ++dataCheckIt)
	{
    if (dataCheckIt.value() && plotData.contains(dataCheckIt.key()))
		{
			PixRGB<byte> color = plotDataColor[dataCheckIt.key()];
			QGraphicsTextItem *text = itsGraphicsScene->addText(dataCheckIt.key());
			text->setDefaultTextColor(QColor(color.red(), color.green(), color.blue()));
			text->setPos(1, textPos);
			text->setFlags(QGraphicsItem::ItemIgnoresTransformations);
			textPos+=textOffset;
			plotLine(plotData[dataCheckIt.key()], color);
		}
	}
	itsGraphicsScene->addLine(QLineF(0, -5000, 0, 5000), QPen(QColor(128, 0, 0)));
	itsGraphicsScene->addLine(QLineF(0, 0, itsMaxPlotLength, 0), QPen(QColor(128, 0, 0)));

	itsGraphicsView->fitInView(itsGraphicsScene->itemsBoundingRect());
	itsGraphicsView->ensureVisible(itsGraphicsScene->itemsBoundingRect());
	itsGraphicsView->centerOn(itsMaxPlotLength/2, 0);
}


// ######################################################################
void ScorbotConsole::gotGravityCompensation(float compensation)
{
	plotData["Gravity_Comp"].push_back(compensation*10000);
	if(plotData["Gravity_Comp"].size() > itsMaxPlotLength)
		plotData["Gravity_Comp"].erase(plotData["Gravity_Comp"].begin());

	gravityCompLbl->setNum(compensation);
}

