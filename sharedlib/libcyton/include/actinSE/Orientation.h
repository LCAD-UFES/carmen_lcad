#ifndef actinSE_Orientation_H_
#define actinSE_Orientation_H_
//     Copyright (c) 2001-2010 Energid Technologies. All rights reserved. ////
//
// Filename:    Orientation.h
//
// Description: Description of a 3D rotation 
//
// Contents:    class actinSE::Orientation
//
/////////////////////////////////////////////////////////////////////////
#include "actinTypes.h"

/// ActinSE namespace encapsulates all classes
namespace actinSE
{

/// forward declarations.
class Array3;

/// a tolerance value for the sum of the magnitude squared of the 
/// quaternion entries.
const EcReal EcTOLERANCE = 1e-12;

/// Description of a 3D rotation 
/** Holds a representation of a 3D Orientation.  It stores the data as 
 a quaternion but provides utility functions for working with DCM 
 matrices and Euler angles.  The quaternion representation is that for
 which (1,0,0,0) corresponds to no rotation.
*/
class EC_ACTINSE_DECL Orientation 
{
public:
   /// constructor
   Orientation
      (
      );

   /// constructor from four reals: w, x, y, and z
   Orientation
      (
      const EcReal w, 
      const EcReal x, 
      const EcReal y, 
      const EcReal z
      );

   /// destructor
   ~Orientation
      (
      );

   /// copy constructor
   Orientation
      (
      const Orientation& orig
      );

   /// assignment operator
   Orientation& operator=
      (
      const Orientation& orig
      );

   /// equality operator
   EcBoolean operator==
      (
      const Orientation& orig
      ) const;

   /// Transforms this orientation by another as in this = this * orient2.
   /// @param  orient2      The orientation used to transform this orientation.
   /// @return Orientation& This orientation object after transformation.
   Orientation& operator*=
      (
      const Orientation& orient2
      );

   /// returns a transformation equal to this transformed on the right by another
   /// @param  orient2     The orientation used in transformation with this orientation.
   /// @return Orientation An orientation = this * orient2.
   Orientation operator*
      (
      const Orientation& orient2
      ) const;

   /// transforms a vector and return the result
   /// @param  vec    A vector used in transformation.
   /// @return Array3 The transformed vector.
   Array3 operator*
      (
      const Array3& vec
      ) const;

   /// transforms a vector in place.
   /// @param vec A vector to be transformed.
   void transform
      (
      Array3& vec
      ) const;

   /// transforms a vector and puts the result in the second argument
   /// The same vector may be used for to and from.
   /// @param from A vector used in transformation.
   /// @param to   The result of the transformation.
   void transform
      (
      const Array3& from,
      Array3& to
      ) const;

   /// transforms two vectors - more efficient than transforming individually
   /// The same vector may be used for to and from.
   void transform
      (
      const Array3& firstFrom,
      Array3& firstTo,
      const Array3& secondFrom,
      Array3& secondTo
      ) const;

   /// tests for approximate equality of orientations.
   /// (note: not of quaterions, which may be +/-)
   EcBoolean approxEq
      (
      const Orientation& orient2, 
      const EcReal tol=EcTOLERANCE
      ) const;

   /// Finds the angle and axis between this orientation and another.
   /// @param  q2        The other orientation.
   /// @param  theta     Upon return, the angle in radians between the two orientation.
   /// @param  axis      Upon return, the axis between the two orientation.
   /// @return EcBoolean Success or failure of command
   EcBoolean angleAxisBetween
      (
      const Orientation& q2,
      EcReal& theta,  
      Array3& axis    
      ) const;

   /// Finds hyperspherical surface interpolation between the two quaternions
   /// and sets the result in this orientation.
   /// @param orient1 The first orientation. Can be itself.
   /// @param orient2 The second orientation.
   /// @param factor A real number between 0 and 1 will from orient1 to orient2. A number outside of 0 and 1 will extrapolate the two quaternions.
   void interpolation
      (
      const Orientation& orient1, 
      const Orientation& orient2, 
      const EcReal factor
      );

   /// Gets the inverse of the orientation
   /// @return Orientation The inverse of this orientation.
   Orientation inverse
      (
      ) const;

   /// Inverts this in place
   /// @return Orientation This orientation after inversion.
   Orientation& invert
      (
      );

   /// sets a quaternion value directly
   /// @param w
   /// @param x
   /// @param y
   /// @param z
   inline void set
      (
      const EcReal w, 
      const EcReal x, 
      const EcReal y, 
      const EcReal z
      )
   {
      m_Quaternion[0] = w;
      m_Quaternion[1] = x;
      m_Quaternion[2] = y;
      m_Quaternion[3] = z;
      normalize();
   }

   /// sets from yaw, pitch, roll Euler angles
   /// @param psi
   /// @param theta
   /// @param phi
   void setFrom321Euler
      (
      const EcReal psi, 
      const EcReal theta, 
      const EcReal phi
      );

   /// gets yaw, pitch, roll Euler angles
   /// @param psi
   /// @param theta
   /// @param phi
   void get321Euler
      (
      EcReal& psi, 
      EcReal& theta, 
      EcReal& phi
      ) const;

   /// sets from roll, pitch, yaw Euler angles
   /// @param phi
   /// @param theta
   /// @param psi
   void setFrom123Euler
      (
      const EcReal phi, 
      const EcReal theta, 
      const EcReal psi
      );

   /// gets roll, pitch, yaw Euler angles
   /// @param phi
   /// @param theta
   /// @param psi
   void get123Euler
      (
      EcReal& phi, 
      EcReal& theta, 
      EcReal& psi
      ) const;

   /// sets the rotation based on an angle and an axis
   /// @param angle
   /// @param axis
   void setFromAngleAxis
      (
      const EcReal angle,
      const Array3& axis
      );

   /// gets the angle and axis for an Orientation
   /// @param angle Resulting angle
   /// @param axis  Resulting vector
   void getAngleAxis
      (
      EcReal& angle,
      Array3& axis
      ) const;

   /// sets the rotation from a Rodrigues vector (also called Gibbs vector)
   /// p = k tan(theta/2), for axis k and angle theta
   /// @param vector Rodrigues vector to use
   void setFromRodriguesVector
      (
      const Array3& vector
      );

   /// gets the Rodrigues vector (also called Gibbs vector)
   /// p = k tan(theta/2), for axis k and angle theta
   /// returns an approximation when theta=Pi.
   /// @param vector Resulting vector
   void getRodriguesVector
      (
      Array3& vector
      );

   /// gets any quaternion element by index
   /// @param  index   Index to select
   /// @return EcReal& Value at index
   const EcReal& operator[]
      (
      const EcU32 index
      ) const;

   /// gets the rows of a DCM matrix corresponding to the Orientation
   /// @param row0
   /// @param row1
   /// @param row2
   void getDcmRows
      (
      Array3& row0, 
      Array3& row1, 
      Array3& row2
      ) const;

   /// sets the Orientation from DCM rows
   /// @param row0
   /// @param row1
   /// @param row2
   void setFromDcmRows
      (
      const Array3& row0, 
      const Array3& row1, 
      const Array3& row2
      );

   /// gets the columns of a DCM matrix corresponding to the Orientation
   /// @param col0
   /// @param col1
   /// @param col2
   void getDcmColumns
      (
      Array3& col0, 
      Array3& col1, 
      Array3& col2
      ) const;

   /// sets the Orientation from DCM columns
   /// @param col0
   /// @param col1
   /// @param col2
   void setFromDcmColumns
      (
      const Array3& col0, 
      const Array3& col1, 
      const Array3& col2
      );

   /// gets the outboard x-axis
   /// @return Array3 The first column of the DCM matrix
   Array3 xAxis
      (
      ) const;

   /// gets the outboard y-axis
   /// @return Array3 The second column of the DCM matrix
   Array3 yAxis
      (
      ) const;

   /// gets the outboard z-axis
   /// @return Array3 The third column of the DCM matrix
   Array3 zAxis
      (
      ) const;

protected:
   /// takes the magnitude of the internal representation
   /// @return EcReal Magnitude^2
   EcReal magSquared
      (
      ) const;

   /// normalizes the internal representation
   void normalize
      (
      );

   EcReal m_Quaternion[4]; ///< Internal quaternion representation
};

} // namespace actinSE

#endif // actinSE_Orientation_H_
