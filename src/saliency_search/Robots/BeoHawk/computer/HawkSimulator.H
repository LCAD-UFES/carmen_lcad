// File:    HawkSimulator.H
// Author:  Prithvi Balaram <balaram@usc.edu>
// Date:    April 2010

// TODO: Build Voxel class for displaying the map in OpenGL
// TODO: Build basic OpenGL Simulation display class

#include <string>
#include <Ice/Ice.h>
#include "Robots/BeoHawk/core/HawkMessages.ice.H"
#include "Robots/BeoHawk/computer/HawkAgent.H"

class HawkSimulator : public HawkAgent {
 public:
  // These functions must be defined in all HawkAgents
  HawkSimulator(std::string myName, int argc, char* argv[]);

  // Scheduler, CS201 agent style
  bool scheduler();

  // Register some messages that we're listening for over ICE
  void registerTopics();

  // Catch an ICE message -- equivalent of CS201 messaging
  void catchMessage(const HawkMessages::MessagePtr& msg, const Ice::Current&);

 private:
  // These functions are specific to the individual agents
  enum States {LOOP};
  States state;

  // Publish the Sensor Data Message over ICE
  void sendSensorDataMessage();

  // Get the readings from LRF based upon current position
  void laserScan();

  // Start displaying the OpenGL Simulation
  void startDisplay();

  // Stop displaying the OpenGL Simulation
  void stopDisplay();

  // Update the OpenGL Simulation with some new information
  void updateDisplay();
};
