#ifndef actinSE_EndEffector_H_
#define actinSE_EndEffector_H_
//     Copyright (c) 2009-2010 Energid Technologies. All rights reserved. ////
//
// Filename:    EndEffector.h
//
// Description: Describes an end-effector interface for use with the
//              actinSE::ControlSystem class.
//
// Contents:    class actinSE::EndEffector
//
// Notes:       Currently most of the methods require that the end-effector
//              be attached to an existing control system.
//
/////////////////////////////////////////////////////////////////////////
#include "actinTypes.h"

/// ActinSE namespace encapsulates all classes
namespace actinSE
{

/// End-effector interface
/**
 An end-effector is the means by which a user may issue high-level
 movement commands.  An end-effector is typically applied to the last,
 or most distal joint.  The desired location can then be specified
 in system coordinates.  When running the desired locations through
 the actinSE::ControlSystem, it will attempt to satisfy the desired command
 given the constraints.  The constraints may be other end-effectors
 in the system, physical limitations, etc.
*/
class EC_ACTINSE_DECL EndEffector
{
public:
   /// During construction, used to determine the type of end-effector to
   /// configure.  It ultimately dictates the number of degrees of freedom(DoF)
   /// as well as the number of degrees of constraint(DoC).
   enum EETypeEnum
   {
      UnknownEndEffector = 0,      ///< Uninitialized end-effector
      PointEndEffector,            ///< 3 DoC in position
      OrientationEndEffector,      ///< 3 DoC in orientation
      FrameEndEffector,            ///< 6 DoC in position and orientation
      LinearConstraintEndEffector  ///< 1 DoC
   };

   /// Parameters to describe the end-effector.  They can be either a Get (G),
   /// Set(S) or Set and Get(SG).
   /// The second argument corresponds to the parameter type that it expects.
   enum ParamTypeEnum
   {
      DegreesOfConstraint, ///< ( G, #EcU32) Number of degrees of constraint
      ActualPose,          ///< ( G, actinSE::Array3,
                           ///<      actinSE::Orientation,
                           ///<      actinSE::CoordinateSystemTransformation)
                           ///<      Actual position, orientation or both
      RelativeLink,        ///< (SG, #EcU32|#EcString) Relative link using an index or string name
      MotionThreshold,     ///< (SG, #EcReal) General threshold value
      DesiredPose,         ///< (SG, actinSE::Array3,
                           ///<      actinSE::Orientation,
                           ///<      actinSE::CoordinateSystemTransformation)
                           ///<      Desired position, orientation or both
      DesiredVelocity,     ///< (SG, #EcReal|#EcRealVector) Desired velocity in each DoC
      Gain,                ///< (SG, #EcReal) Specified gain (LinearConstraint)
      HardConstraint,      ///< (SG, #EcBoolean) Whether EE is hard constrained or not
   };

   /// State bit flags which describe the end-effector.
   enum EEStateFlagsEnum
   {
      EmptyStateFlags    = 0x0, ///< Blank, empty state
      RelativeLinkFlag   = 0x1, ///< Whether the pose is relative to a link
      HardConstrained    = 0x2, ///< If this EE is hard constrained or not
      Attached           = 0x4, ///< Whether EE is attached to a control system
   };
   /// Variable type to hold the state flag bits.
   typedef EcU32 EEStateFlags;

   /// Constructor
   /// @param eeType End-effector type to use
   explicit EndEffector
      (
      const EETypeEnum eeType
      );

   /// Destructor
   ~EndEffector
      (
      );

   /// Copy constructor
   /// @param orig Instance to copy
   EndEffector
      (
      const EndEffector& orig
      );

   /// Assignment operator
   /// @param  orig         Instance to equate to
   /// @return EndEffector& Resultant instance
   EndEffector& operator=
      (
      const EndEffector& orig
      );

   /// Equality operator
   /// @param  rhs       Other object to compare against
   /// @return EcBoolean Whether the comparison is successful or not
   EcBoolean operator==
      (
      const EndEffector& rhs
      ) const;

   /// Generalized method to set an end-effector parameter.  The second
   /// template parameter doesn't need to be explicitly specified as it will
   /// be inferred from the variable being passed in.   
   /// @tparam prm       Parameter to set
   /// @param  value     What to set the parameter to
   /// @return EcBoolean Success or failure of command
   template <ParamTypeEnum prm, typename ParamType> EcBoolean setParam
      (
      const ParamType& value
      );

   /// Generalized method to retrieve an end-effector parameter.  The second
   /// template parameter doesn't need to be explicitly specified as it will
   /// be inferred from the variable being passed in.
   /// @tparam prm       Parameter to set
   /// @param  value     Value to retrieve
   /// @return EcBoolean Success or failure of command
   template <ParamTypeEnum prm, typename ParamType> EcBoolean getParam
      (
      ParamType& value
      ) const;

   /// Generalized method to retrieve an end-effector parameter.  It is a 
   /// convenience accessor to get-by-value the desired type without error
   /// error checking.
   /// @tparam prm       Parameter to retrieve
   /// @tparam ParamType Variable type returned
   /// @return ParamType Value of parameter
   template <ParamTypeEnum prm, typename ParamType> ParamType param
      (
      );

   /// Gets the current state flags associated with this end-effector.
   /// @return EEStateFlags Bit flags representing the state of the end-effector
   EEStateFlags stateFlags
      (
      ) const;

   /// Get the EE type
   /// @return EETypeEnum The configured end-effector type
   EETypeEnum endEffectorType
      (
      ) const;

   /// String description of the current end-effector.
   /// @return EcString Name of type of end-effector
   EcString name
      (
      ) const;
   
private:
   friend struct ControlSystemImpl;
   EndEffectorImpl* m_pImpl;
};

// MSVC requires specialized templates to declare their storage type
// before they are defined.  This gets around compiler error C3416.
#ifdef _MSC_VER
#  include "EndEffectorTemplateInst.ipp"
#endif

} // namespace actinSE

#endif // actinSE_EndEffector_H_
