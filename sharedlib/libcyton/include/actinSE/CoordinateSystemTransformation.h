#ifndef actinSE_CoordinateSystemTransformation_H_
#define actinSE_CoordinateSystemTransformation_H_
//     Copyright (c) 2001-2010 Energid Technologies. All rights reserved. ////
//
// Filename:    CoordinateSystemTransformation.h
//
// Description: A rotation and a translation to describe a new frame.
//
// Contents:    class actinSE::CoordinateSystemTransformation
//
/////////////////////////////////////////////////////////////////////////
#include "actinTypes.h"
#include "Orientation.h"
#include "Array3.h"

/// ActinSE namespace encapsulates all classes
namespace actinSE
{

/// A rotation and a translation describing a new frame
/** Holds a class to describe 3D rigid-body transformation.
*/
class EC_ACTINSE_DECL CoordinateSystemTransformation
{
public:
   /// Descriptor of the type of transformation
   enum ModeEnum
   {
      NO_CHANGE,      ///< Default, uninitialized transformation
      ARBITRARY,      ///< Contains both translation and rotation
      NO_TRANSLATION, ///< Contains only rotation component
      NO_ROTATION     ///< Contains only translation component
   };

   /// Default constructor
   CoordinateSystemTransformation
      (
      );

   /// Constructor with a translation and orientation
   /// @param trans  Translation component
   /// @param orient Orientation component
   CoordinateSystemTransformation
      (
      const Array3& trans,
      const Orientation& orient
      );

   /// Constructor with a translation (implicit conversion used in code base)
   /// @param trans Translation component
   CoordinateSystemTransformation
      (
      const Array3& trans
      );

   /// Constructor with an orientation (implicit conversion used in code base)
   /// @param orient Orientation component
   CoordinateSystemTransformation
      (
      const Orientation& orient
      );

   /// Destructor
   ~CoordinateSystemTransformation
      (
      );

   /// Copy constructor
   /// @param orig Instance to copy
   CoordinateSystemTransformation
      (
      const CoordinateSystemTransformation& orig
      );

   /// Assignment operator
   /// @param  orig                            Instance to equate to
   /// @return CoordinateSystemTransformation& Returned instance
   CoordinateSystemTransformation& operator=
      (
      const CoordinateSystemTransformation& orig
      );

   /// Equality operator
   /// @param  orig      Instance to compare against
   /// @return EcBoolean EcTrue if they are equal, EcFalse otherwise
   EcBoolean operator==
      (
      const CoordinateSystemTransformation& orig
      ) const;

   /// Gets the mode that the transform is in.
   /// @return ModeEnum Current mode setting
   ModeEnum mode
      (
      ) const;

   /// Get translation component.
   /// @return const Array3& Translation component
   inline const Array3& translation
      (
      ) const
   {
      return m_Translation;
   }

   /// Set translation component.  For vectors, it is faster to use
   /// setTranslation() on the vector quantity than to set the three
   /// elements individually.
   /// @param value Translation component
   void setTranslation
      (
      const Array3& value
      );

   /// Gets the orientation component.
   /// @return const Orientation& Orientation component
   inline const Orientation& orientation
      (
      ) const
   {
      return m_Orientation;
   }

   /// Set orientation component.
   /// @param value Orientation to set
   void setOrientation
      (
      const Orientation& value
      );

   /// Outboard transform by a translation and rotation.
   /// @param translation Translation component
   /// @param orientation Orientation component
   void outboardTransformBy
      (
      const Array3& translation,
      const Orientation& orientation
      );

   /// Outboard transform by a translation only.
   /// @param translation Translation component
   void outboardTransformBy
      (
      const Array3& translation
      );

   /// Conversion to a composite
   /// @param xform2
   CoordinateSystemTransformation& operator*=
      (
      const CoordinateSystemTransformation& xform2
      );

   /// Transformation composition
   /// @param xform2
   CoordinateSystemTransformation operator*
      (
      const CoordinateSystemTransformation& xform2
      ) const;

   /// Array3 transformation
   /// @param  vec    Array3 to transform by
   /// @return Array3 Resultant vector
   Array3 operator*
      (
      const Array3& vec
      ) const;

   /// Array3 transformation in place
   /// @param vec Array3 to transform by
   void transform
      (
      Array3& vec
      ) const;

   /// Transforms a vector and puts the result in the second argument
   /// The same vector may be used for to and from.
   /// @param from Array3 to transform by
   /// @param to   Resultant Array3
   void transform
      (
      const Array3& from,
      Array3& to
      ) const;

   /// Transforms two vectors - more efficient than transforming individually
   /// The same vector may be used for to and from.
   void transform
      (
      const Array3& firstFrom,
      Array3& firstTo,
      const Array3& secondFrom,
      Array3& secondTo
      ) const;

   /// Test for approximate equality of transformations.
   /// @param  xform2    Comparison transform
   /// @param  tol       Tolerance to use 
   /// @return EcBoolean EcTrue if equal, EcFalse otherwise
   EcBoolean approxEq
      (
      const CoordinateSystemTransformation& xform2, 
      EcReal tol
      ) const;

   /// Get the inverse of the transformation
   /// @return CoordindateSystemTranformation Inverse transform
   CoordinateSystemTransformation inverse
      (
      ) const;

   /// Invert this transformation in place
   /// @return CoordinateSystemTransformation& Inverted transform
   CoordinateSystemTransformation& invert
      (
      );

   /// Interpolation between two xforms
   /// @param  coordSysxForm1 Start transform
   /// @param  coordSysxForm2 End transform
   /// @param  factor         Interpolation factor
   /// @return EcBoolean      Success or failure of command
   EcBoolean interpolation
      (
      const CoordinateSystemTransformation& coordSysxForm1,
      const CoordinateSystemTransformation& coordSysxForm2,
      const EcReal& factor
      );

protected:
   Array3      m_Translation; ///< Offset of transformation in non-rotated frame
   Orientation m_Orientation; ///< Rotation of the transformation
   ModeEnum    m_Mode;        ///< Transformation mode setting
};

} // namespace actinSE

#endif // actinSE_CoordinateSystemTransformation_H_
