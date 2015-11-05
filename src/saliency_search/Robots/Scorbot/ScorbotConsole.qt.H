#ifndef ROBOTS_SCORBOT_SCORBOTCONSOLE_H
#define ROBOTS_SCORBOT_SCORBOTCONSOLE_H

#include <Ice/Ice.h>
#include <QtGui/QtGui>
#include <QtCore/QThread>
#include "Component/ModelComponent.H"
#include "Robots/Scorbot/Scorbot.ice.H"
#include "Qt4/ImageGraphicsItem.qt.H"
#include "QtUtil/ImageConvert4.H"

class ScorbotInterfaceThread;

// ######################################################################
class ScorbotConsole : public QMainWindow, public ModelComponent
{
  Q_OBJECT;

  public:
    ScorbotConsole(OptionManager& mgr, 
        const std::string& descrName = "ScorbotConsole",
        const std::string& tagName   = "ScorbotInterface");

		~ScorbotConsole();

		void stop1();
		void start2();
		QMap<ScorbotIce::JointType, QString> jointStringMap;
		void plotLine(std::vector<float> plot, PixRGB<byte> color);

  public slots:
		void gotEncoderVal(int joint, int val);
		void gotPWMVal(int joint, float val);
		void gotTargetPos(int joint, int val);
		void gotGravityCompensation(float comp);
		void pidAxisSelected(int index);
		void setPIDVals();
		void resetEncoders();
		void setPosition(int joint);
		void toggleMotorsEnabled();
		void addData(QString plotName, double data);
		void updatePlots();
		void printPosCode(); //!< Prints some hacky C++ to stdout

  private:
		void saveParams();
		void loadParams();

		ScorbotInterfaceThread *itsScorbotThread;
		QMap<ScorbotIce::JointType, QLabel*>      encoderLbls;
		QMap<ScorbotIce::JointType, QLabel*>      pwmLbls;
		QMap<ScorbotIce::JointType, QLineEdit*>   encoderEdits;
		QMap<ScorbotIce::JointType, QLineEdit*>   durationEdits;
		QMap<ScorbotIce::JointType, QPushButton*> setPosButtons;

		QComboBox *itsAxisComboBox;

		QLineEdit *pGainEdit, *iGainEdit, *dGainEdit, *maxIEdit, *maxPWMEdit,
		          *pwmOffsetEdit, *foreArmMassEdit, *upperArmMassEdit, *gravityCompScaleEdit;
	  QLabel *gravityCompLbl;

		QPushButton* enableMotorsButton;

		QMap<QString, std::vector<float> > plotData;
		QMap<QString, PixRGB<byte> > plotDataColor;
		QMap<QString, bool> plotDataCheck;
		QGraphicsView* itsGraphicsView;
		QGraphicsScene* itsGraphicsScene;
		ImageGraphicsItem* itsImageDisplay;

		bool itsMotorsEnabled;

		size_t itsMaxPlotLength;

		QSettings* itsSettings;


		Ice::CommunicatorPtr ic;
		ScorbotIce::ScorbotPrx itsScorbot;
};

// ######################################################################
class ScorbotInterfaceThread : public QThread
{
	Q_OBJECT;
	public:

		ScorbotInterfaceThread(ScorbotConsole* window, ScorbotIce::ScorbotPrx *scorbot) :
			itsScorbot(scorbot), itsWindow(window), sleepTime(3) 
	{

	}

		bool running;

		void run()
		{
			running = true;
			while(running)
			{
				//Update the current encoder values
				ScorbotIce::encoderValsType encoders = (*itsScorbot)->getEncoders();
				ScorbotIce::encoderValsType::iterator encIdx = encoders.begin();
				for(; encIdx != encoders.end(); ++encIdx)
					emit(gotEncoderVal((int)encIdx->first, encIdx->second));
				
				//Update the current PWM values
				ScorbotIce::pwmValsType PWMs = (*itsScorbot)->getPWMs();
				ScorbotIce::pwmValsType::iterator pwmIdx = PWMs.begin();
				for(; pwmIdx != PWMs.end(); ++pwmIdx)
					emit(gotPWMVal((int)pwmIdx->first, pwmIdx->second));

				//Update the target velocity and position values for the current joint
				int32 targetPos, targetVel;
				float gravityCompensation;
				(*itsScorbot)->getTuningVals(itsSelectedJoint, targetPos, targetVel, gravityCompensation);

				emit(gotTargetPos((int)itsSelectedJoint, targetPos));
				emit(gotGravityCompensation(gravityCompensation));


				emit(updatedAllValues());
				this->msleep(100);
			}
		}

		void setSelectedJoint(ScorbotIce::JointType joint) { itsSelectedJoint = joint; }

	signals:
		void gotEncoderVal(int, int);
		void gotPWMVal(int, float);
		void gotTargetPos(int, int);
		void gotGravityCompensation(float);

		void updatedAllValues();

	private:
		ScorbotIce::ScorbotPrx *itsScorbot;
		ScorbotConsole* itsWindow;
		ScorbotIce::JointType itsSelectedJoint;
		int sleepTime;

};

#endif // ROBOTS_SCORBOT_SCORBOTCONSOLE_H

