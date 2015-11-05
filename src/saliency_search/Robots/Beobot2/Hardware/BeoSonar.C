#include "Robots/Beobot2/Hardware/BeoSonar.H"
#include "Ice/BeobotEvents.ice.H"

const ModelOptionCateg MOC_BeoSonar = {
        MOC_SORTPRI_3, "Beobot Sonar Related Options" };

const ModelOptionDef OPT_SerialDev =
{ MODOPT_ARG(std::string), "SerialDev", &MOC_BeoSonar, OPTEXP_CORE,
        "The serial device file",
        "serial-dev", '\0', "/dev/ttyUSBX", "/dev/ttyUSB1"};

// ######################################################################
BeoSonar::BeoSonar(int id, OptionManager& mgr,
                   const std::string& descrName, const std::string& tagName) :
  RobotBrainComponent(mgr, descrName, tagName),
  itsSerial(new Serial(mgr)),
  itsData(5), itsOfs(new OutputFrameSeries(mgr)),
  itsDisplayUpdateRate(.05),
  itsSerialDev(&OPT_SerialDev, this, 0)
{
  addSubComponent(itsSerial);
  addSubComponent(itsOfs);
  itsData[0].angle = -60;
  itsData[1].angle = -30;
  itsData[2].angle = 0;
  itsData[3].angle = 30;
  itsData[4].angle = 60;
}

// ######################################################################
BeoSonar::~BeoSonar()
{
}

void BeoSonar::start3()
{
  itsSerial->configure (itsSerialDev.getVal().c_str(), 115200, "8N1", false, false, 0);

        //Check to ensure that our ttyUSB devices are set up correctly by polling their IDs
        LINFO("Checking for cooling board");
        //unsigned char cmd[1] = {0};
        std::vector<unsigned char> propIDvec = itsSerial->readFrame(0, 1, .5);

        std::string propID(propIDvec.begin(), propIDvec.end());

        if(propID == "")
                LFATAL("ERROR! Unrecognized device on %s", itsSerialDev.getVal().c_str());
        else if(propID != "coolingboard")
                LFATAL("ERROR! Incorrect device on %s: %s", itsSerialDev.getVal().c_str(), propID.c_str());

        LINFO("%s found on %s", propID.c_str(), itsSerialDev.getVal().c_str());
}

void BeoSonar::registerTopics()
{
        registerPublisher("SonarReport");
}

void BeoSonar::evolve()
{
        //unsigned char cmd[1] = {97};

        //Request a set of sonar readings from the cooling board
        std::vector<unsigned char> frame = itsSerial->readFrame(97, 1, .2);

        //If we recieved the expected 20 bytes back (4 bytes for each of the 5 sonars)
        //then let's parse them. The resulting values are the number of clock ticks
        //at 80Mhz that it took for the sonar pulse to leave the sonar, bounce off of the
        //nearest object, and return
        if(frame.size() == 20)
        {
                unsigned int time0 = ((0x0FF & frame[0])  << 24) |
                                                                                            ((0x0FF & frame[1])  << 16) |
                                                                                                  ((0x0FF & frame[2])  << 8)  |
                                                                                                  ((0x0FF & frame[3])  << 0);

                unsigned int time1 = ((0x0FF & frame[4])  << 24) |
                                            ((0x0FF & frame[5])  << 16) |
                                            ((0x0FF & frame[6])  << 8)  |
                                            ((0x0FF & frame[7])  << 0);

                unsigned int time2 = ((0x0FF & frame[8])  << 24) |
                                            ((0x0FF & frame[9])  << 16) |
                                            ((0x0FF & frame[10]) << 8)  |
                                            ((0x0FF & frame[11]) << 0);

                unsigned int time3 = ((0x0FF & frame[12]) << 24) |
                                            ((0x0FF & frame[13]) << 16) |
                                            ((0x0FF & frame[14]) << 8)  |
                                            ((0x0FF & frame[15]) << 0);

                unsigned int time4 = ((0x0FF & frame[16]) << 24) |
                                            ((0x0FF & frame[17]) << 16) |
                                            ((0x0FF & frame[18]) << 8)  |
                                            ((0x0FF & frame[19]) << 0);


                //Convert the number of clock ticks found above to meters
                double distance0 = (double(time0) / double(80000000.0)) * double(343.0) / 2.0;
                double distance1 = (double(time1) / double(80000000.0)) * double(343.0) / 2.0;
                double distance2 = (double(time2) / double(80000000.0)) * double(343.0) / 2.0;
                double distance3 = (double(time3) / double(80000000.0)) * double(343.0) / 2.0;
                double distance4 = (double(time4) / double(80000000.0)) * double(343.0) / 2.0;

                BeobotEvents::SonarMessage *msg = new BeobotEvents::SonarMessage;
                msg->distances.resize(5);
                msg->angles.resize(5);
                msg->distances[0] = distance0; msg->angles[0] = -60;
                msg->distances[1] = distance1; msg->angles[1] = -30;
                msg->distances[2] = distance2; msg->angles[2] =   0;
                msg->distances[3] = distance3; msg->angles[3] =  30;
                msg->distances[4] = distance4; msg->angles[4] =  60;
                this->publish("SonarReport", msg);

                //Plot the distances
                itsData[0].dist = distance0;
                itsData[1].dist = distance1;
                itsData[2].dist = distance2;
                itsData[3].dist = distance3;
                itsData[4].dist = distance4;
                plotDists();
        }
        else
        {
                LERROR("Invalid Frame Size Recieved from Sonar!");
        }
        usleep(100000);
}

// ######################################################################
void BeoSonar::updateMessage(const RobotSimEvents::EventMessagePtr& eMsg,
    const Ice::Current&)
{
}

// ######################################################################
void BeoSonar::plotDists()
{
        if(itsDisplayTimer.getSecs() > itsDisplayUpdateRate)
        {
                itsDisplayTimer.reset();

                Image<PixRGB<byte> > dispImg(512,512,ZEROS);
                double rad;
                double dist_scale = 70.0;


                for (unsigned int i=0;i<itsData.size();i++)
                {
                        PixRGB<byte> drawColor(0,255,0);

                        rad = std::min(itsData[i].dist,10.0);
                        rad = rad*dist_scale;
                        Point2D<int> pt;
                        pt.i = 256 - (int) rad*sin(itsData[i].angle*M_PI/180.0);
                        pt.j = 512 - (int) rad*cos(itsData[i].angle*M_PI/180.0);

                        if(i < itsData.size()-1)
                        {
                                Point2D<int> nxt_pt;
                                double rad2 = std::min(itsData[i+1].dist,10.0);
                                rad2 *= dist_scale;
                                nxt_pt.i = 256 - (int) rad2*sin(itsData[i+1].angle*M_PI/180.0);
                                nxt_pt.j = 512 - (int) rad2*cos(itsData[i+1].angle*M_PI/180.0);
                                drawLine(dispImg, pt, nxt_pt, PixRGB<byte>(0,0,255));
                        }


                        if(itsData[i].dist >= 10.0)
                                drawColor = PixRGB<byte>(255,0,0);

                        drawCircle(dispImg, pt, 3, PixRGB<byte>(255,0,0));
                        drawLine(dispImg, Point2D<int>(256,512),pt,
                                        drawColor);
                        char buffer[128];
                        sprintf(buffer, "%0.2fm", itsData[i].dist);
                        writeText(dispImg, pt, buffer,
                                        PixRGB<byte>(255,255,255), PixRGB<byte>(0,0,0), SimpleFont::FIXED(6), true);
                }

                itsOfs->writeRGB(dispImg, "SonarData", FrameInfo("SonarData",SRC_POS));
                itsOfs->updateNext();
        }
}

