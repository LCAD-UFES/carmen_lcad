#ifndef cytonTypes_H_
#define cytonTypes_H_
//     Copyright (c) 2008-2010 Energid Technologies. All rights reserved. ////
//
// Filename:    cytonTypes.h
//
// Description: This header provides a minimal set of definitions for use
//              with the Cyton arm.  
//
// Contents:    
//
/////////////////////////////////////////////////////////////////////////

/// Define some basic types used in Cyton code for those that do not
/// have full set of headers.
#ifdef EC_HAVE_ACTIN
#  include <foundCore/ecConstants.h>
#else // !EC_HAVE_ACTIN
#  include <actinSE/actinTypes.h>
#endif

/// Namespace for all Cyton-specific code.
namespace cyton
{

/// List of unit and variable types that can be retrieved.  Used within calls to
/// setJointCommands and getJointState.  Default units are radians with no bias or scale.
/// Note - gripper values in radian or degree mode are always given in meters.
/// The only conversion done is when using normalized units.
enum StateTypeEnum
{
   JointAngleInRadians  = 0x000, ///< Command joint angle
   BiasAngleInRadians   = 0x001, ///< Bias angle applied to joint
   MinAngleInRadians    = 0x002, ///< Minimum jont angle
   MaxAngleInRadians    = 0x004, ///< Maximum joint angle
   InitAngleInRadians   = 0x008, ///< Reset joint angle
   JointScale           = 0x010, ///< Direction and scale of joint movement
   JointVelocity        = 0x020, ///< Joint velocity (rad/sec)
   JointTorque          = 0x040, ///< Joint torque
   JointBaseMask        = 0x0ff, ///< Mask to select joint value w/o modifiers

   AngleWithBias        = 0x100, ///< Modifier to add Bias angle
   AngleWithScale       = 0x200, ///< Modifier to scale angle
   AngleInDegrees       = 0x400, ///< Modifier to specify angles in degrees
   AngleNormalized      = 0x800, ///< Modifier to specify angle in range [-1,1]

   /// Convenience enumerations for default units.
   JointAngle           = JointAngleInRadians,
   BiasAngle            = BiasAngleInRadians,
   MinAngle             = MinAngleInRadians,
   MaxAngle             = MaxAngleInRadians,
   InitAngle            = InitAngleInRadians,

   /// Angles in degrees.
   JointAngleInDegrees  = JointAngle|AngleInDegrees,
   BiasAngleInDegrees   = BiasAngle |AngleInDegrees,
   MinAngleInDegrees    = MinAngle  |AngleInDegrees,
   MaxAngleInDegrees    = MaxAngle  |AngleInDegrees,
   InitAngleInDegrees   = InitAngle |AngleInDegrees,

   /// Normalized joint angles between [-1,1]
   JointAngleNormalized = JointAngle|AngleNormalized,
   BiasAngleNormalized  = BiasAngle |AngleNormalized,
   MinAngleNormalized   = MinAngle  |AngleNormalized,
   MaxAngleNormalized   = MaxAngle  |AngleNormalized,
   InitAngleNormalized  = InitAngle |AngleNormalized,

   /// Joint angles including Bias and Scale value.
   JointAngleInRadiansBiasScale = JointAngleInRadians|AngleWithBias|AngleWithScale,
   JointAngleInDegreesBiasScale = JointAngleInDegrees|AngleWithBias|AngleWithScale,
};
typedef EcU32 StateType; ///< Specify a specific type for the enumeration.

#if defined(CYTON_SOURCE) || defined(EC_PLUGIN_SOURCE)
#  define CYTON_DECL          EC_DECL_EXPORTS
#  define CYTON_TEMPLATE_DECL extern
#else
#  define CYTON_DECL          EC_DECL_IMPORTS
#  define CYTON_TEMPLATE_DECL
#endif
   
} // namespace cyton

#endif // cytonTypes_H_
