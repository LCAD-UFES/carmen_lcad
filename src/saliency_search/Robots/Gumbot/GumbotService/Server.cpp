
#include <IceE/IceE.h>
#include <GumbotI.h>

#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

using namespace std;
using namespace Robots;


int run(int argc, char* argv[], const Ice::CommunicatorPtr& communicator)
{
  Ice::ObjectAdapterPtr adapter =
    communicator->createObjectAdapterWithEndpoints("GumbotAdapter", "default -p 10000");
  Ice::ObjectPtr object = new GumbotI(1);
  adapter->add(object, communicator->stringToIdentity("GumbotService"));
  adapter->activate();
  communicator->waitForShutdown();
  return EXIT_SUCCESS;
}

  int
main(int argc, char* argv[])
{
  int status;
  bool daemon = false;
  bool noclose = false;
  Ice::CommunicatorPtr communicator;


  try
  {
    Ice::InitializationData initData;
    //initData.properties = Ice::createProperties();
    //initData.properties->load("config");
    //initData.properties->setProperty("Ice.Override.Timeout", "100");
    communicator = Ice::initialize(argc, argv, initData);

    if (daemon)
    {
      printf("Running as a daemon\n");
      //Become a daemon
      // fork off the parent process
      pid_t pid = fork();
      if (pid < 0)
      {
        printf("Can not fork\n");
        exit(1);
      }

      if (pid > 0)
        exit(0); //exit the parent process

      // Change the file mask
      umask(0);

      //Create a new system id so that the kernel wont think we are an orphan.
      pid_t sid = setsid();
      if (sid < 0)
      {
        printf("Can not become independent\n");
        exit(1);
      }

      if (!noclose)
      {
        fclose(stdin);
        fclose(stdout);
        fclose(stderr);
      }

    }

    status = run(argc, argv, communicator);
  }
  catch(const Ice::Exception& ex)
  {
    fprintf(stderr, "%s\n", ex.toString().c_str());
    status = EXIT_FAILURE;
  }

  if(communicator)
  {
    try
    {
      communicator->destroy();
    }
    catch(const Ice::Exception& ex)
    {
      fprintf(stderr, "%s\n", ex.toString().c_str());
      status = EXIT_FAILURE;
    }
  }

  return status;
}

