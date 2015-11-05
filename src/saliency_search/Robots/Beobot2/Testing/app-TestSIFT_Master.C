#include "Media/FrameSeries.H"
#include "Component/ModelManager.H"
#include "Image/Image.H"
#include "Image/PixelsTypes.H"
#include "Robots/Beobot2/Testing/TestSIFT.ice.H"
#include "Util/WorkThreadServer.H"
#include "Util/Timer.H"
#include "SIFT/VisualObject.H"


#include <Ice/Ice.h>
#include <fstream>

////////////////////////////////////////////////////////////////////////
// Definition for a matching job which spawns a request to a remote 
// worker and waits for the reply
////////////////////////////////////////////////////////////////////////
class MatchJob : public JobServer::Job
{
  public:
    MatchJob(TestSIFT::SIFTMatcherPrx matcherProxy,
        std::vector<rutz::shared_ptr<Keypoint> >::iterator kp_beginning,
        std::vector<rutz::shared_ptr<Keypoint> >::iterator kp_ending
        )
      : Job()
    {
      itsMatcherPrx = matcherProxy;

      std::vector<rutz::shared_ptr<Keypoint> >::iterator kpIt;
      for(kpIt = kp_beginning; kpIt < kp_ending; ++kpIt)
      {
        rutz::shared_ptr<Keypoint> key = *kpIt;

        //Copy all of the keypoint data into the Ice structure
        TestSIFT::keypoint newKey;
        newKey.x = key->getX();
        newKey.y = key->getY();
        newKey.s = key->getS();
        newKey.o = key->getO();
        newKey.m = key->getM();
        newKey.oriFV = key->getOriFV();
        itsKeypoints.push_back(newKey);
      }
    }

    void run()
    {
      //Send the list of keypoints to the remote worker, and wait until 
      //it returns with some results

			try
			{
				TestSIFT::idSequence ids = itsMatcherPrx->matchKeypoints(itsKeypoints);
				LDEBUG("Got Result of size: %Zu", ids.size());
			}
			catch(const Ice::Exception& ex)
			{  std::cerr << "Failed: " << ex << std::endl;   }
			catch(const char* msg)
			{   std::cerr << "Failed: " << msg << std::endl; }


    }

    char const* jobType() const { return "MatchJob"; }

    TestSIFT::SIFTMatcherPrx itsMatcherPrx;
    TestSIFT::keypointSequence itsKeypoints;
};

int main(int argc, char const* argv[])
{
  ////////////////////////////////////////////////////////////////////////
  // Start up our model components 
  ////////////////////////////////////////////////////////////////////////
  ModelManager mgr;
  nub::ref<InputFrameSeries> ifs(new InputFrameSeries(mgr));
  mgr.addSubComponent(ifs);

//  nub::ref<OutputFrameSeries> ofs(new OutputFrameSeries(mgr));
//  mgr.addSubComponent(ofs);

  mgr.parseCommandLine(argc, argv, "logfile <list of workers specified as IP[:PORT]>", 2,-1);
  mgr.start();

  ////////////////////////////////////////////////////////////////////////
  // Open the log file for writing
  ////////////////////////////////////////////////////////////////////////
  //Get the log file name as the first extra argument
  std::string logfilename = mgr.getExtraArg(0);
  std::ofstream logfile(logfilename.c_str());


  ////////////////////////////////////////////////////////////////////////
  // Create a list of worker proxies
  ////////////////////////////////////////////////////////////////////////
  std::vector<TestSIFT::SIFTMatcherPrx> workers;

  Ice::CommunicatorPtr ic;
  try
  {
    int dummy_argc=1;
    const char* dummy_argv[1] = {"dummy"};

    std::cout << "Initializing Ice Runtime...";

    Ice::PropertiesPtr props = Ice::createProperties(dummy_argc,(char**)dummy_argv);
    props->setProperty("Ice.MessageSizeMax", "1048576");
    props->setProperty("Ice.Warn.Connections", "1");
    Ice::InitializationData id;
    id.properties = props;

    ic = Ice::initialize(id);

    std::cout << "Success!" << std::endl;

    int numWorkers = mgr.numExtraArgs()-1;
    for(int workerID=0; workerID<numWorkers; ++workerID)
    {
      //Get the worker IP and port separately, using the default port if not specified
      std::string workerString = mgr.getExtraArg(workerID+1);
      size_t colonIdx = workerString.find(":");
      std::string workerName   = workerString.substr(0, colonIdx);
      uint workerPort = TestSIFT::DEFAULTWORKERPORT;
      if(colonIdx != std::string::npos) 
        workerPort = atoi(workerString.substr(colonIdx+1, std::string::npos).c_str());

      //Try opening proxy connections to the worker services
      std::cout << "Opening Connection to " << workerName << ":" << workerPort << "...";
      char connectionBuffer[256];
      sprintf(connectionBuffer,
          "SIFTMatcher:default -h %s -p %d", workerName.c_str(), workerPort);
      Ice::ObjectPrx base = ic->stringToProxy(connectionBuffer);
      workers.push_back(TestSIFT::SIFTMatcherPrx::checkedCast(base));
      std::cout << "Opened" << std::endl;
    }
  }
  catch(const Ice::Exception& ex)
  {  std::cerr << "Failed: " << ex << std::endl;   }
  catch(const char* msg)
  {   std::cerr << "Failed: " << msg << std::endl; }

  ////////////////////////////////////////////////////////////////////////
  // Read input frames, extract the SIFT keypoints, and send them out to be
  // matched by the workers
  ////////////////////////////////////////////////////////////////////////
  LINFO("Beginning Processing With %Zu Nodes", workers.size());

  //Create a jobserver to handle the workers - one thread for each worker
  WorkThreadServer threadServer("SIFT Match Job Server", workers.size());

  Timer timer(1000000);
  FrameState state = FRAME_NEXT;
  while(state == FRAME_NEXT)
  {
    //Read in the new input image
    state = ifs->updateNext();
    Image<PixRGB<byte> > img = ifs->readRGB();
    if(img.initialized())
    {
      //ofs->writeRGB(img, "Input");
    }
    else 
      break;

    //Compute SIFT keypoints from the image
    VisualObject siftMatch("img", "img", img);
    std::vector<rutz::shared_ptr<Keypoint> > keypoints =
      siftMatch.getKeypoints();

    int kp_per_worker = keypoints.size() / workers.size();

    //Assign the computed keypoints to the workers
    rutz::shared_ptr<JobServer::Job> jobs[workers.size()];
    for(size_t workerIdx=0; workerIdx<workers.size(); ++workerIdx)
    {
      int keyIdx_start = kp_per_worker*workerIdx;
      int keyIdx_end   = kp_per_worker*(workerIdx+1);
      if(workerIdx == workers.size()-1)
        keyIdx_end = keypoints.size();

      LDEBUG("Worker %Zu gets %d - %d / %Zu",
          workerIdx,
          keyIdx_start,
          keyIdx_end,
          keypoints.size());

      jobs[workerIdx] = rutz::shared_ptr<JobServer::Job>(
          new MatchJob(
            workers[workerIdx],
            keypoints.begin() + keyIdx_start,
            keypoints.begin() + keyIdx_end
            )
          );
    }

    //Enqueue the jobs all at once, and start the timer
    timer.reset();
    threadServer.enqueueJobs(jobs, workers.size());

    //Wait for all jobs to finish, and stop the timer
    threadServer.flushQueue(10000, false);
    double time = timer.getSecs();

    logfile << time << std::endl;
    logfile.flush();
    LINFO("Done Frame in %fs", time);
  }
  LINFO("FINISHED");
}
