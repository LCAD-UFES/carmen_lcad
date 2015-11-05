#ifndef cytonHardwareInterface_H_
#define cytonHardwareInterface_H_
//     Copyright (c) 2008-2010 Energid Technologies. All rights reserved. ////
//
// Filename:    cytonHardwareInterface.h
//
// Description: Cyton hardware interface class.  This class abstracts internal
//              workings of the device driver to provide control over Cyton
//              hardware.  It talks through the plugin class, which is also
//              used by the viewer.
//
// Contents:    
//
/////////////////////////////////////////////////////////////////////////
#include <cytonTypes.h> // Pull in definitions of variable types.

namespace Ec { class Plugin; }  ///< Forward declaration of plugin class.    
class cytonBaseIO;

/// Namespace for all Cyton-specific code.
namespace cyton
{

class CYTON_DECL hardwareInterface
{
public:
   /// Bitfield options that hold whether or not to override parameters
   /// within the configuration file when initializing hardware.  Currently
   /// the only override parameters are the port device and resetOnShutdown flag.
   enum OverrideEnum
   {
      OverrideNone  = 0x0, ///< Pull everything from config file
      OverridePort  = 0x1, ///< User specified port
      OverrideReset = 0x2, ///< User specified reset
   };
   typedef EcU32 OverrideType; ///< Variable type to hold override options.
   
   /** Constructor.  Does not initialize hardware.
    * @param[in] pluginName Name of hardware plugin to utilize
    * @param[in] configFile Optional hardware configuration file
    */
   explicit hardwareInterface
      (
      const EcString& pluginName,
      const EcString& configFile = ""
      );

   /** Destructor.  Shuts down device driver if loaded.
    */
   virtual ~hardwareInterface
      (
      );

   /// Specify a port to use for the connection to the hardware.
   /// @param[in] port String name of port to use.  Platform dependent
   virtual void setPort
      (
      const EcString& port
      );

   /// Examine current hardware configuration to list available ports.
   /// @return EcStringVector A vector of string representing the port names
   ///                        of the devices available.  Platform dependent.
   ///                        Empty list returned if not available, or plugin not loaded.
   virtual EcStringVector availablePorts
      (
      ) const;

   /// Flag indicating whether or not to reset Cyton joints to their
   /// initialization position before powering down.
   /// @param[in] resetOnShutdown Whether or not to reset on power down
   virtual void setResetOnShutdown
      (
      const EcBoolean resetOnShutdown
      );

   /// Accessor to retrieve state of whether reset will occur before power down.
   /// @return EcBoolean EcTrue if a reset will occur or EcFalse if not
   virtual EcBoolean resetOnShutdown
      (
      ) const;

   /// Initialize hardware, which includes reading in configuration file, opening
   /// the port and resetting hardware to a known good state.
   /// @return EcBoolean Success or failure of initialization
   virtual EcBoolean init
      (
      );

   /// Send a reset command to the hardware to move joints back to resting position.
   /// @return EcBoolean Success or failure of reset command
   virtual EcBoolean reset
      (
      );

   /// Unloads plugin device driver.  
   /// @return EcBoolean Success or failure of shutdown command
   virtual EcBoolean shutdown
      (
      );

   /// Sends commands to Cyton hardware to move joints to a specified location.
   /// A time difference is calculated from the previous command to determine
   /// the rate at which to move the joints.
   /// @param[in] timeNow         Current time
   /// @param[in] jointCommands   Vector of joint angles to move servos to
   /// @param[in] stateType       Optional unit conversion for input jointCommands
   /// @param[in] jointVelocities Vector of joint velocities
   /// @return    EcBoolean       Success or failure of set command
   virtual EcBoolean setJointCommands
      (
      const EcReal timeNow,
      const EcRealVector& jointCommands,
      const StateType stateType = JointAngleInRadians,
      const EcRealVector& jointVelocities = EcRealVector()
      );

   /// Retrieve servo information.  Depending on the stateType parameter
   /// it will return the last commanded position (default) or any of the
   /// configuration parameters for the servos (joint bias, min angle,
   /// max angle, reset angle, max joint rate, joint scale).
   /// @param[out] jointStates Vector of returned values
   /// @param[in] stateType Type and unit of requested values
   /// @return EcBoolean Success or failure of query command
   virtual EcBoolean getJointStates
      (
      EcRealVector& jointStates,
      const StateType stateType = JointAngleInRadians
      ) const;

   /// Wait for the last command to finish, up to a specified maximum
   /// time in milliseconds.
   /// @param[in] timeoutInMS Maximum time to wait in milliseconds before failing
   /// @return    EcBoolean   Success or failure of wait command
   virtual EcBoolean waitUntilCommandFinished
      (
      const EcU32 timeoutInMS
      ) const;

   /// Retrieve the number of joints currently configured.
   /// @return EcU32 Number of joints in the loaded system
   virtual EcU32 numJoints
      (
      ) const;

   /// Give the ability to rate limit the joints.  If enabled,
   /// it will limit the arm at 25% of max rate.
   /// @param[in] lowRate   Turn rate limiting on or off
   /// @return    EcBoolean Success or failure of command
   virtual EcBoolean setLowRate
      (
      const EcBoolean lowRate
      );

   /// Retrieve a handle to the loaded plugin.
   /// @return Ec::Plugin* The loaded plugin
   Ec::Plugin* plugin
      (
      );

   /// Sends commands to Cyton hardware to move joints to a specified location.
   /// @param[in] jointCommands   Vector of joint angles to move servos to
   /// @param[in] jointVelocities Vector of joint velocities
   /// @return    EcBoolean       Success or failure of set command
   virtual EcBoolean setJointCommands
      (
      const EcRealVector& jointCommands,
      const EcRealVector& jointVelocities 
      );

private:
   /// Load the internal plugin to interface with the Cyton hardware.
   /// @return EcBoolean Success or failure of load command
   EcBoolean loadPlugin
      (
      ) const;

   EcString m_PortName;             ///< Platform-dependent description of port device
   EcString m_PluginName;           ///< Name of plugin to use
   EcString m_ConfigFile;           ///< Configuration file for system
   EcBoolean m_ResetOnShutdown;     ///< Whether to reset joints before powering down
   OverrideType m_OverrideConfig;   ///< If port or reset specified, override config file
   mutable cytonBaseIO* m_pPlugin;  ///< Handle to base device driver to process commands
};

} // namespace cyton

#endif // cytonHardwareInterface_H_
