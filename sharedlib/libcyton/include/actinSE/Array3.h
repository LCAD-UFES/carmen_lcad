#ifndef actinSE_Array3_H_
#define actinSE_Array3_H_
//     Copyright (c) 2001-2010 Energid Technologies. All rights reserved. ////
//
// Filename:    Array3.h
//
// Description: A three-element vector.
//
// Contents:    class actinSE::Array3
//
/////////////////////////////////////////////////////////////////////////
#include "actinTypes.h"
#include <iosfwd>

/// ActinSE namespace encapsulates all classes
namespace actinSE
{

/// A three-element vector
/**
 Holds a representation of a 3-element vector.  It provides utilities such
 as magnitude, dot product, and cross product.  This class has inline
 methods for speed optimization.
*/
class EC_ACTINSE_DECL Array3 
{
public:
   /// constructor
   Array3
      (
      );

   /// constructor from three reals
   Array3
      (
      const EcReal x, 
      const EcReal y, 
      const EcReal z
      );

   /// destructor
   ~Array3
      (
      );

   /// copy constructor
   Array3
      (
      const Array3& orig
      );

   /// assignment operator
   Array3& operator=
      (
      const Array3& orig
      );

   /// equality operator
   EcBoolean operator==
      (
      const Array3& orig
      ) const;

   /// inequality operator
   EcBoolean operator!=
   (
    const Array3& orig
    ) const;

   /// add another vector to this vector and set this vector to the result
   inline Array3& operator+=
      (
      const Array3& v2
      )
   {
      m_Vector[0] += v2.m_Vector[0];
      m_Vector[1] += v2.m_Vector[1];
      m_Vector[2] += v2.m_Vector[2];
      return *this;
   }

   /// subtract another vector from this vector and set this vector to the result
   inline Array3& operator-=
      (
      const Array3& v2
      )
   {
      m_Vector[0] -= v2.m_Vector[0];
      m_Vector[1] -= v2.m_Vector[1];
      m_Vector[2] -= v2.m_Vector[2];
      return *this;
   }

   /// multiply this vector times a scalar and set this vector to the result
   inline Array3& operator*=
      (
      EcReal s
      )
   {
      m_Vector[0] *= s;
      m_Vector[1] *= s;
      m_Vector[2] *= s;
      return *this;
   }

   /// returns a vector equal to this vector plus another
   Array3 operator+
      (
      const Array3& v2
      ) const;

   /// returns a vector equal to this vector minus another
   Array3 operator-
      (
      const Array3& v2
      ) const;

   /// returns a vector which has been multiplied by 
   /// the scalar a on an element by element basis
   inline Array3 operator*
      (
      const EcReal a
      ) const
   {
      return Array3(m_Vector[0]*a,m_Vector[1]*a,m_Vector[2]*a);
   }

   /// returns a vector which has been divided by 
   /// the scalar a on an element by element basis
   Array3 operator/
      (
      const EcReal a
      ) const;

   /// returns a vector equal to this vector cross another (vector cross product)
   Array3 cross
      (
      const Array3& v2
      ) const;

   /// returns a vector equal to this vector dot another (vector dot product)
   EcReal dot
      (
      const Array3& v2
      ) const;

   /// returns the magnitude of this vector
   EcReal mag
      (
      ) const;

   
   /// returns the product of the three elements
   EcReal prod
      (
      ) const;

   /// returns the magnitude squared of this vector (a fast operation)
   EcReal magSquared
      (
      ) const;

   /// returns a unit vector in the same direction as this vector
   Array3 unitVector
      (
      ) const;

   /// normalizes this vector
   Array3& normalize
      (
      );

   /// tests that each element of this vector is within a tolerance of another 
   EcBoolean approxEq
      (
      const Array3& v2, 
      const EcReal tol
      ) const;

   /// find the Euclidean distance to another point
   EcReal distanceTo
      (
      const Array3& vec
      ) const;

   /// find the Euclidean distance squared to another point
   EcReal distanceSquaredTo
      (
      const Array3& vec
      ) const;

   /// compute a vector which points from this vector (point) to the other vector (point) with a given magnitude.
   /**
   \param[in] destination The destination point
   \param[in] mag The magnitude of the new vector
   \param[out] result The new vector
   */
   void computeDirectionalVector
      (
      const Array3& destination,
      const EcReal mag,
      Array3& result
      ) const;

   /// sets the z value of the vector
   inline void set
      (
      const EcReal x, 
      const EcReal y, 
      const EcReal z
      )
   {
      m_Vector[0] = x;
      m_Vector[1] = y;
      m_Vector[2] = z;
   }

   /// returns a value by index (0, 1, or 2) - const version.
   inline const EcReal& operator[]
      (
      const EcU32 index
      ) const
   {
      return m_Vector[index];
   }

   /// returns a value by index (0, 1, or 2) - nonconst version.
   inline EcReal& operator[]
      (
      const EcU32 index
      )
   {
      return m_Vector[index];
   }

protected:
   EcReal m_Vector[3];                  ///< Internal vector format
};

// -----------------------------------------------
// extra operators below
// -----------------------------------------------

/// multiplies a vector times a scalar, scalar first
inline Array3 operator*
   (
   const EcReal s, 
   const Array3& v1
   )
{
   return Array3(v1[0]*s,v1[1]*s,v1[2]*s);
}

/// returns the negative of a vector
inline Array3 operator- 
   (
   const Array3& v1
   )
{
   return Array3(-v1[0],-v1[1],-v1[2]);
}

/// Convenience stream printing.  Displays as a[0], a[1], a[2]
EC_ACTINSE_DECL std::ostream&
operator<<
   (
   std::ostream& os,
   const Array3& a
   );

} // namespace actinSE

#endif // actinSE_Array3_H_
