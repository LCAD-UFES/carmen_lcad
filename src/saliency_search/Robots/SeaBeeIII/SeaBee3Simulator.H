#include "Component/ModelManager.H"
#include "Component/OptionManager.H"
#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameInfo.H"
#include "Raster/GenericFrame.H"
#include "Image/Image.H"
#include "GUI/XWinManaged.H"
#include "GUI/ImageDisplayStream.H"
#include "Util/Timer.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Robots/RobotBrain/RobotBrainComponent.H"
#include "Ice/RobotBrainObjects.ice.H"
#include "Ice/RobotSimEvents.ice.H"
#include "GUI/ViewPort.H"
#include "Ice/IceImageUtils.H"
#include <IceUtil/Thread.h>
#include <ode/ode.h>
#include <ode/collision.h>
#include "Controllers/PID.H"
#include "Util/Angle.H"

class SeaBee3Simulator : public RobotBrainComponent
{

  public:
    SeaBee3Simulator(OptionManager &mgr,
        const std::string& descrName = "SeaBee3Simulator",
        const std::string& tagName = "SeaBee3Simulator");

    ~SeaBee3Simulator();

    virtual void evolve();

    virtual void updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
        const Ice::Current&);

    virtual void registerTopics();

    //Get a camera image from one of our virtual cameras
    Image<PixRGB<byte> > getFrame(int camera);

    dWorldID        getWorld() { return world; }
    dJointGroupID getContactgroup() { return contactgroup; }


    void handleWinEvents(XEvent& event);

    void updateSensors(const dReal *pos, const dReal *R);

    void getSensors(float &xPos, float &yPos, float &depth,
        float &roll, float &pitch, float &yaw);

    void simLoop();

    void setThrusters(double forwardLeftThruster, double forwardRightThruster,
        double verticalLeftThruster, double verticalRightThruster,
        double forwardStrafeThruster, double backStrafeThruster);

  private:
    bool Initialized;
    //Generate the 3D model of the submarine
    void makeSub();
    //Draw the submarine
    void drawSub();

    //arena
    void drawArena();
    void drawWaterSurface();
    void drawGate(const double *pos);
    void drawBuoy(const double *pos);
    void drawBin(const double ori, const double *pos);
    void drawPipeline(const double ori, const double *pos);
    void drawPinger(const double *pos);
    void drawPool();

    void applyHydrodynamicForces(dReal viscosity);

    //Update the PID controllers if they are active, and set the
    //proper motor variables
    void updateControllers();

    dWorldID        world;
    dSpaceID        space;
    dGeomID          ground;
    dJointGroupID contactgroup;

    dBodyID itsSubBody;
    dGeomID itsSubGeom;

    double itsPoolDepth;
    double itsPoolLength;
    double itsPoolWidth;

    double itsWaterLevel;
    double itsSubLength;
    double itsSubRadius;
    double itsSubWeight;

    ViewPort *vp;

    double itsForwardLeftThrusterVal;
    double itsForwardRightThrusterVal;
    double itsFrontStrafeThrusterVal;
    double itsBackStrafeThrusterVal;
    double itsLeftVertThrusterVal;
    double itsRightVertThrusterVal;



    bool itsWorldView;
    bool itsShowWorld;
    XWinManaged *itsWorldDisp;

    pthread_mutex_t itsDispLock;

    //A timer to keep track of the amount of time since the last
    //set of camera messages was sent out. New messages are only
    //sent out once the elapsed time surpasses itsCameraUpdateTime
    Timer itsCameraTimer;
    double itsCameraUpdateTime;

    //A timer to keep track of the amount of time since the last
    //BeeStem message was sent out. A new message is only sent
    //out once the elapsed time surpasses itsBeeStemUpdateTime
    Timer itsBeeStemTimer;
    double itsBeeStemUpdateTime;

    //Virtual BeeStem related variables
    double itsSimXPos;
    double itsSimYPos;
    double itsSimDepth;
    double itsSimRoll;
    double itsSimPitch;
    double itsSimYaw;

    int itsDepth;
    int itsDesiredHeading;
    int itsDesiredDepth;
    int itsDesiredSpeed;
    int itsHeadingK;
    int itsHeadingP;
    int itsHeadingD;
    int itsHeadingI;
    int itsHeadingOutput;
    int itsDepthK;
    int itsDepthP;
    int itsDepthD;
    int itsDepthI;
    int itsDepthOutput;

    bool itsPIDsEnabled;

    PID<float> *itsDepthPID;
    int itsTargetDepth;

    PID<Angle> *itsHeadingPID;
    int itsTargetHeading;

    int itsTargetSpeed;

    IceUtil::Mutex itsStemMutex;

    Texture *arenaWallTexture;
    Texture *waterSurfaceTexture;

    IceUtil::Mutex itsJSMutex;

    std::vector<int> itsJSValues;
  std::vector<int> itsButValues;

};

