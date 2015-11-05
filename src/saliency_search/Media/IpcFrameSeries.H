#ifndef IPCFRAMESERIES_H_
#define IPCFRAMESERIES_H_

#include "Component/ModelParam.H"
#include "Media/MediaSimEvents.H"
#include "Raster/GenericFrame.H"
#include "Simulation/SimModule.H"
#include "Simulation/SimEvents.H"
#include "Util/SimTime.H"
#include "Neuro/WinnerTakeAll.H"
#include "Neuro/WTAwinner.H"
#include "Neuro/NeuroSimEvents.H"
#include "Util/SimTime.H"

#include <carmen/carmen.h>
#include <carmen/bumblebee_basic_messages.h>
#include <carmen/bumblebee_basic_interface.h>
#include <carmen/stereo_messages.h>
#include <carmen/stereo_interface.h>
#include <carmen/stereo_util.h>

class InputFrameSeries;

class IpcInputFrameSeries : public SimModule
{
public:
  //! Constructor
  IpcInputFrameSeries(OptionManager& mgr, int camera = 8, int list_size = 5, int widht = 640, int height = 480,
                      const std::string& descrName = "Ipc Input Frame Series",
                      const std::string& tagName = "IpcInputFrameSeries");

  //! Destructor
  virtual ~IpcInputFrameSeries();

  carmen_handler_t publish_callback;
  carmen_handler_t handler_callback;

protected:
  //! Callback function triggered at each clock tick
  SIMCALLBACK_DECLARE(IpcInputFrameSeries, SimEventClockTick);

  //! Callback for when a new point from WinnerTakeAll is avaiable
  SIMCALLBACK_DECLARE(IpcInputFrameSeries, SimEventWTAwinner);

};

#endif /* IPCFRAMESERIES_H_ */
