#include "Robots/Beobot2/Testing/TestSIFT.ice.H"
#include "SIFT/VisualObjectDB.H"
#include <Ice/Ice.h>

class SIFTMatcherI : public TestSIFT::SIFTMatcher
{
  public:
    SIFTMatcherI(char const* dbfile)
      : TestSIFT::SIFTMatcher()
    {
      std::cout << "Loading Database " << dbfile << "...";
      itsSIFTdb.loadFrom(dbfile, false);
      std::cout << "Done, Loaded " << itsSIFTdb.numObjects() << " objects" << std::endl;

      std::cout << "Building KDTree...";
      itsSIFTdb.buildKDTree();
      std::cout << "Done" << std::endl;

      std::cout << "Waiting for server..." << std::endl;
    }

    TestSIFT::idSequence matchKeypoints(TestSIFT::keypointSequence const& keypoints, Ice::Current const&)
    {
      LINFO("Got %Zu Keypoints", keypoints.size());
      std::vector<rutz::shared_ptr<Keypoint> > VOkeypoints;
      for(size_t keyIdx=0; keyIdx<keypoints.size(); keyIdx++)
      {
        //Create a new keypoint
        TestSIFT::keypoint const* const key = &keypoints.at(keyIdx);
        VOkeypoints.push_back(
            rutz::shared_ptr<Keypoint>(new Keypoint(
                key->oriFV, key->x, key->y, key->s, key->o, key->m
                )
              )
            );
      }

      rutz::shared_ptr<VisualObject> vo(
          new VisualObject(
          "MyVisualObject",            //Name
          "NULL",                      //Image File Name
          Image<PixRGB<byte> >(),      //Image
          Point2D<int>(-1,-1),         //Attention point
          std::vector<float>(),        //Pre-attentive features
          VOkeypoints,                 //Our list of keypoints
          false,                       //useColor
          false));                     //computeKP

      //Find a match for the keypoints
      std::vector<rutz::shared_ptr<VisualObjectMatch> > matches;
      uint n_matches = itsSIFTdb.getObjectMatches(vo, matches);
      LINFO("Got %d matches: Best: %s", n_matches, 
          (matches.size() > 0 ? matches.at(0)->getVoTest()->getName().c_str() : "NONE" ));

      return TestSIFT::idSequence();
    }

  private:
    VisualObjectDB itsSIFTdb;
};

char* progName; 
void printHelp()
{
  std::cout << "Usage: " << progName << " [--port PORT] [--siftdb SIFTDBFILEPATH]" << std::endl;
}

int main(int argc, char const* argv[])
{
  progName = (char*)argv[0];
  int port = TestSIFT::DEFAULTWORKERPORT;
  char const* siftdb = "sift.db";


  ////////////////////////////////////////////////////////////////////////
  // Hacky command line option grabber (don't judge!)
  ////////////////////////////////////////////////////////////////////////
  if((argc-1)%2) { 
    std::cout << "Bad Command Line Arguments!" << std::endl;
    printHelp();
    exit(0); 
  }
  for(int i=1; i< argc; ++i)
  {
    if(strncmp(argv[i], "--", 2) != 0)
    {
      std::cout << "Bad Command Line Arguments! (" << argv[i] << ")" << std::endl;
      printHelp();
      exit(0);
    }

    if(strcmp(argv[i], "--port") == 0)
      port = atoi(argv[++i]);
    else if(strcmp(argv[i], "--siftdb") == 0)
      siftdb = argv[++i];
    else if(strcmp(argv[i], "--help") == 0)
      printHelp();
    else
    {
      std::cout << "Unknown Command Line Arguments! (" << argv[i] << ")" << std::endl;
      printHelp();
      exit(0);
    }
  }

  ////////////////////////////////////////////////////////////////////////
  // Load up the Ice runtime, and create a new SIFTMatcher on the adapter
  // The details here are unimportannt - check out the implementation of
  // SIFTMatcher above for the interesting bits
  ////////////////////////////////////////////////////////////////////////
  int status = 0;
  Ice::CommunicatorPtr ic;
  try
  {
    std::cout << "Starting SIFT Matching Worker" << std::endl;
    std::cout << "Port: " << port << std::endl;
    std::cout << "DB File: " << siftdb << std::endl;

		Ice::PropertiesPtr props = Ice::createProperties(argc,(char**)argv);
		props->setProperty("Ice.MessageSizeMax", "1048576");
		props->setProperty("Ice.Warn.Connections", "1");

		Ice::InitializationData id;
		id.properties = props;

    ic = Ice::initialize(id);

    char connectionBuffer[256];
    if(argc == 2)
      port = atoi(argv[1]);
    sprintf(connectionBuffer, "default -p %d", port);
    Ice::ObjectAdapterPtr adapter = ic->createObjectAdapterWithEndpoints(
        "SIFTMatcher", connectionBuffer);
    std::cout << "Connection opened on port " << port << std::endl;

    Ice::ObjectPtr object = new SIFTMatcherI(siftdb);
    adapter->add(object, ic->stringToIdentity("SIFTMatcher"));
    adapter->activate();
    ic->waitForShutdown();
  }
  catch(Ice::Exception const& e)
  {
    std::cerr << e << std::endl;
    status=1;
  }
  catch(char const* msg)
  {
    std::cerr << msg << std::endl;
    status=1;
  }

  if(ic)
  {
    try
    { 
      std::cout << "Shutting Down...";
      ic->destroy();
      std::cout << "Success" << std::endl;
    }
    catch(Ice::Exception const& e)
    {
      std::cerr << "Failed: " << e << std::endl;
      status = 1;
    }
  }
    return status;
}
