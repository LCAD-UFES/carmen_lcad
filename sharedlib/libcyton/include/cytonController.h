#ifndef cytonController_H_
#define cytonController_H_
//     Copyright (c) 2008-2010 Energid Technologies. All rights reserved. ////
//
// Filename:    cytonController.h
//
// Description: Cyton controller class. Combine ActinSE simulation and 
//    Cyton hardware interface so Cyton can be commanded in task space mode.
//
// Contents:    
//
/////////////////////////////////////////////////////////////////////////
#include <actinSE/ControlSystem.h>
#include <actinSE/CoordinateSystemTransformation.h>
#include <actinSE/EndEffector.h>
#include <cytonHardwareInterface.h>

namespace cyton ///< Namespace for all Cyton-specific code.
{

class CYTON_DECL CytonController
{
public:
   /// constructor
   CytonController
      (
      );

   /// destructor
   ~CytonController
      (
      );

   /// get the name of the simulation file
   const EcString& simulationFile
      (
      ) const;

   /// set the name of the simulation file
   void setSimulationFile
      (
      const EcString& value
      );

   /// get the name of the plugin file (only needed when hardware mode is true)
   const EcString& pluginFile
      (
      ) const;

   /// set the name of the plugin file (only needed when hardware mode is true)
   void setPluginFile
      (
      const EcString& value
      );

   /// get the name of the configuration file (only needed when hardware mode is true)
   const EcString& configFile
      (
      ) const;

   /// set the name of the configuration file (only needed when hardware mode is true)
   void setConfigFile
      (
      const EcString& value
      );

   /// get the end-effector index
   EcU32 endEffectorIndex
      (
      ) const;

   /// set the end-effector index
   void setEndEffectorIndex
      (
      EcU32 index
      );

   /// get the hardware mode
   EcBoolean hardwareMode
      (
      ) const;

   /// set the hardware mode
   void setHardwareMode
      (
      EcBoolean mode
      );

   /// set the rendering mode
   void setRenderingMode
      (
      EcBoolean mode
      );

   /// get the tolerance. It's used to determine when/if the end-effector reaches the goal
   EcReal tolerance
      (
      ) const;

   /// set the tolerance. It's used to determine when/if the end-effector reaches the goal
   void setTolerance
      (
      EcReal tolerance
      );

   /// Specify a port to use for the connection to the hardware.
   /// @param[in] port String name of port to use.  Platform dependent
   virtual void setPort
      (
      const EcString& port
      );

   /// initialize the controller. 
   /**
   \return True if successful or false otherwise
   */
   EcBoolean init
      (
      );

   /// shutdown the hardware
   void shutdown
      (
      );

   /// reset the robot to the initial joint angles
   void reset
      (
      );

   /// move the end-effector to a point within the given timeout period
   EcBoolean moveTo
      (
      const actinSE::Array3& point, 
      EcReal timeoutPeriod=10.0
      );

   /// move the joints to the desired angles within the given timeout period
   EcBoolean moveJointsTo
      (
      const EcRealVector& jointAngles,
      EcReal timeoutPeriod=10.0
      );

   /// get the last actual of the end-effector
   const actinSE::Array3& lastActualPoint
      (
      ) const;

private:
   EcString m_PluginFile;
   EcString m_ConfigFile;
   EcString m_SimulationFile;
   EcBoolean m_HardwareMode;
   EcBoolean m_RenderingMode;
   EcReal m_Tolerance;
   EcString m_Port;
   actinSE::ControlSystem* m_pControlSystem;
   EcU32 m_EndEffectorIndex;
   cyton::hardwareInterface* m_pHardwareInterface; 

   // utility members
   actinSE::EndEffectorVector* m_pEeVec;
   actinSE::CoordinateSystemTransformation m_DesiredPose;   // this is use for point/frame EE
   actinSE::CoordinateSystemTransformation m_ActualPose;
   std::vector<actinSE::CoordinateSystemTransformation> m_DesiredJointPoses; // this is use for linear-constraint EEs
   EcRealVector* m_pJointAngles;

   // private methods
   EcBoolean moveCommandLoop
      (
      EcBoolean jointMode,
      EcReal timeoutPeriod=10.0
      );

   /// check whether the goal position has been reached
   EcBoolean goalReached
      (
      EcBoolean jointMode
      );
};

} // namespace cyton

#endif // cytonController_H_
