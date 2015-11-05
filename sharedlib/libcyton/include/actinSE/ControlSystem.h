#ifndef actinSE_ControlSystem_H_
#define actinSE_ControlSystem_H_
//     Copyright (c) 2009-2010 Energid Technologies. All rights reserved. ////
//
// Filename:    ControlSystem.h
//
// Description: Basic control system class.
//
// Contents:    class actinSE::ControlSystem
//
/////////////////////////////////////////////////////////////////////////
#include "actinTypes.h"

/// ActinSE namespace encapsulates all classes
namespace actinSE
{

/// Basic interface to the Cyton control system
class EC_ACTINSE_DECL ControlSystem
{
public:
   /// Parameters to describe the control system.  They can be either a Get(G),
   /// Set(S) or Set and Get(SG).
   /// The second argument corresponds to the parameter type that it expects.
   enum ParamTypeEnum
   {
      Rendering,         ///< (SG, #EcBoolean) Enable or disable rendering window
      SimulationTime,    ///< (SG, #EcReal) the current time in the simulation
      NumJoints,         ///< ( G, #EcU32) Number of joints in this system
      JointAngles,       ///< (SG, #EcRealVector) vector of joint angles
      JointPose,         ///< (SG, actinSE::Array3,
                         ///<      actinSE::Orientation,
                         ///<      actinSE::CoordinateSystemTransformation)
                         ///<      Joint position, orientation or both
      JointVelocities,   ///< (SG, #EcRealVector) vector of joint velocities
      BasePose,          ///< (SG, actinSE::Array3,
                         ///<      actinSE::Orientation,
                         ///<      actinSE::CoordinateSystemTransformation)
                         ///<      Base position, orientation or both
      EndEffectors,      ///< (SG, actinSE::EndEffectorVector) EEs for this system
      CraigDHParameters, ///< ( G, actinSE::Array3Vector) A, alpha, and D params
   };

   /// Constructor
   ControlSystem
      (
      );

   /// Destructor
   ~ControlSystem
      (
      );

   /// Copy constructor
   /// @param orig Instance to copy
   ControlSystem
      (
      const ControlSystem& orig
      );

   /// Assignment operator
   /// @param  orig           Instance to equate to
   /// @return ControlSystem& Resultant instance
   ControlSystem& operator=
      (
      const ControlSystem& orig
      );

   /// Equality operator
   /// @param  rhs       Other object to compare against
   /// @return EcBoolean Whether the comparison is successful or not
   EcBoolean operator==
      (
      const ControlSystem& rhs
      ) const;
   
   /// Load in a control system from a file.  It will load and initialize the control sytem.
   /// @param  fileName  File to load in
   /// @return EcBoolean Success or failure of command
   EcBoolean loadFromFile
      (
      const EcString& fileName
      );

   /// Save off the control system to a file.  This will store all relevant data for reuse.
   /// @param  fileName  File to save to
   /// @return EcBoolean Success or failure of command
   EcBoolean saveToFile
      (
      const EcString& fileName
      );
   
   /// Generalized method to set a control system parameter.  The second
   /// template parameter doesn't need to be explicitly specified as it will
   /// be inferred from the variable being passed in.   
   /// @tparam prm       Parameter to set
   /// @param  value     What to set the parameter to
   /// @return EcBoolean Success or failure of command
   template <ParamTypeEnum prm, typename ParamType> EcBoolean setParam
      (
      const ParamType& value
      );

   /// Generalized method to retrieve a control system parameter.  The second
   /// template parameter doesn't need to be explicitly specified as it will
   /// be inferred from the variable being passed in.   
   /// @tparam prm       Parameter to retrieve
   /// @param  value     Variable type returned
   /// @return EcBoolean Success or failure of command
   template <ParamTypeEnum prm, typename ParamType> EcBoolean getParam
      (
      ParamType& value
      ) const;

   /// Generalized method to retrieve a control system parameter.  The second
   /// template parameter doesn't need to be explicitly specified as it will
   /// be inferred from the variable being passed in.  Only available for
   /// #JointPose.
   /// @tparam prm       Parameter to retrieve
   /// @param  value     Variable type returned
   /// @param  subIndex  Index into desired parameter vector
   /// @return EcBoolean Success or failure of command
   template <ParamTypeEnum prm, typename ParamType> EcBoolean getParam
      (
      const EcU32 subIndex,
      ParamType& value
      ) const;

   /// Generalized method to retrieve a control system parameter.  It is a
   /// convenience accessor to get a const reference to the internal value
   /// without error checking.  Only available for #JointAngles, #JointVelocities,
   /// and #BasePose.
   /// @tparam prm       Parameter to retrieve
   /// @tparam ParamType Variable type returned
   /// @return ParamType Const reference value of parameter
   template <ParamTypeEnum prm, typename ParamType> const ParamType& param
      (
      ) const;

   /// Generalized method to retrieve a control system parameter.  It is a 
   /// convenience accessor to get-by-value the desired type without error
   /// error checking.  Allowed types are: #Rendering, #SimulationTime,
   /// #NumJoints, #EndEffectorVector and #CraigDHParameters.
   /// @tparam prm       Parameter to retrieve
   /// @tparam ParamType Variable type returned
   /// @return ParamType Value of parameter
   template <ParamTypeEnum prm, typename ParamType> ParamType param
      (
      );

   /// Reset all joints and poses to initial positions.  
   /// @return EcBoolean Success or failure of command
   EcBoolean reset
      (
      );

   /// Calculate new joint information.  The results are then available from
   /// the accessor param methods using either #JointAngles or #JointVelocities.
   /// @param  timeInSeconds Time to calculate state at
   /// @return EcBoolean     Success or failure of command
   EcBoolean calculateToNewTime
      (
      const EcReal timeInSeconds
      );

private:
   ControlSystemImpl* m_pImpl;
};

// MSVC requires specialized templates to declare their storage type
// before they are defined.  This gets around compiler error C3416.
#ifdef _MSC_VER
#  include "ControlSystemTemplateInst.ipp"
#endif

} // namespace actinSE

#endif // actinSE_ControlSystem_H_
