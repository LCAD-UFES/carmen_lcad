/**
   \file  Robots/LoBot/misc/LoScanMatch.C
   \brief Implementation of Lu and Milios's Iterative Dual Correspondence
   laser range finder scan matching.
*/

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   //
// by the University of Southern California (USC) and the iLab at USC.  //
// See http://iLab.usc.edu for information about this project.          //
// //////////////////////////////////////////////////////////////////// //
// Major portions of the iLab Neuromorphic Vision Toolkit are protected //
// under the U.S. patent ``Computation of Intrinsic Perceptual Saliency //
// in Visual Environments, and Applications'' by Christof Koch and      //
// Laurent Itti, California Institute of Technology, 2001 (patent       //
// pending; application number 09/912,225 filed July 23, 2001; see      //
// http://pair.uspto.gov/cgi-bin/final/home.pl for current status).     //
// //////////////////////////////////////////////////////////////////// //
// This file is part of the iLab Neuromorphic Vision C++ Toolkit.       //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is free software; you can   //
// redistribute it and/or modify it under the terms of the GNU General  //
// Public License as published by the Free Software Foundation; either  //
// version 2 of the License, or (at your option) any later version.     //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is distributed in the hope  //
// that it will be useful, but WITHOUT ANY WARRANTY; without even the   //
// implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      //
// PURPOSE.  See the GNU General Public License for more details.       //
//                                                                      //
// You should have received a copy of the GNU General Public License    //
// along with the iLab Neuromorphic Vision C++ Toolkit; if not, write   //
// to the Free Software Foundation, Inc., 59 Temple Place, Suite 330,   //
// Boston, MA 02111-1307 USA.                                           //
// //////////////////////////////////////////////////////////////////// //
//
// Primary maintainer for this file: mviswana usc edu
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/slam/LoScanMatch.C $
// $Id: LoScanMatch.C 13572 2010-06-16 18:42:45Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/slam/LoScanMatch.H"
#include "Robots/LoBot/config/LoConfigHelpers.H"
#include "Robots/LoBot/misc/LoExcept.H"
#include "Robots/LoBot/util/LoMath.H"
#include "Robots/LoBot/util/triple.hh"

// INVT utilities
#include "Util/log.H"

// GSL headers
#ifdef INVT_HAVE_LIBGSL
#include <gsl/gsl_eigen.h>
#include <gsl/gsl_matrix.h>
#endif

// Standard C++ headers
#include <vector>
#include <algorithm>
#include <iterator>
#include <functional>
#include <limits>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- KNOB TWIDDLING -----------------------------

// DEVNOTE: For some reason, this anonymous namespace does not seem to
// work properly. The parameters all seem to be zero. Not sure why.
// Anyhoo, removing the anonymous namespace to get this to work...
//namespace {

// Quick helper to return settings from scan_match section of config file
template<typename T>
inline T conf(const std::string& key, T default_value)
{
   return get_conf<T>("scan_match", key, default_value) ;
}

// Override for retrieving triples
template<typename T>
inline triple<T, T, T>
conf(const std::string& key, const triple<T, T, T>& default_value)
{
   return get_conf<T>("scan_match", key, default_value) ;
}

/// This local class encapsulates various parameters that can be used to
/// tweak different aspects of scan matching.
class Params : public singleton<Params> {
   /// Since different laser range finders have varying capabilities and
   /// because it would be nice to be able to use this implementation of
   /// the ICP algorithm with devices other than just the Hokuyo attached
   /// to lobot, we require users to specify the relevant LRF
   /// characteristics with three numbers.
   ///
   /// The first of these numbers specifies the start angle for the range
   /// readings. Usually, this would be a negative number indicating that
   /// the laser range finder sweeps from the right to the left.
   ///
   /// The second number specifies the angular step between consecutive
   /// range readings. For lobot's Hokuyo, this is one degree; but for
   /// other LRF's, it could be 0.5 degrees.
   ///
   /// Finally, the third number in the above triumvirate specifies the
   /// total number of range readings.
   ///
   /// DEVNOTE: The third number should actually be an integer. But we
   /// declare it float for the sake of convenience (so as to avoid
   /// writing a special API for retrieving triple<float, float, int>
   /// from the config file).
   triple<float, float, float> m_lrf_specs ;

   /// In each iteration of ICP, the algorithm searches a small angular
   /// range around each LRF reading to find the best correspondence
   /// between the current and previous LRF scans. This setting specifies
   /// the above-mentioned angular search range.
   int m_lrf_search_range ;

   /// When finding correspondences between the current and previous
   /// scans, we reject any wherein the LRF range measurements exceed
   /// this threshold.
   float m_outlier_threshold ;

   /// The ICP algorithm works by iteratively solving for the rotation
   /// and translation required to register a data shape with a model
   /// shape. Initially, the transformation is assumed to be zero.
   /// However, we can seed the initial transformation using the robot's
   /// odometry so that the algorithm has a better estimate to start off
   /// with. This can reduce the amount of work the algorithm has to do
   /// to register the previous LRF scan with the current one.
   ///
   /// By default, this flag is off so that the implementation follows
   /// the classic ICP description by Besl and McKay, viz., starting off
   /// with zero registration.
   bool m_seed_using_odometry ;

   /// Since the ICP algorithm uses multiple iterations to converge to a
   /// solution, it could (potentially) go on forever. To prevent an
   /// infinite loop, we use this setting to specify the maximum number
   /// of iterations to use.
   int m_max_iterations ;

   /// In general, the ICP algorithm should converge within a few
   /// iterations rather than going on for the maximum set above. In each
   /// iteration, the algorithm will compute a transformation and a total
   /// error between the two scans when that transformation is applied.
   /// The algorithm will terminate when the difference in this error for
   /// two consecutive iterations falls below the threshold specified by
   /// the following setting.
   float m_tau ;

   /// Private constructor because this is a singleton.
   Params() ;
   friend class singleton<Params> ;

public:
   /// Accessing the various parameters
   //@{
   static float lrf_start() {return instance().m_lrf_specs.first      ;}
   static float lrf_res()   {return instance().m_lrf_specs.second     ;}
   static int   lrf_num()   {return int(instance().m_lrf_specs.third) ;}
   static int   lrf_search_range()   {return instance().m_lrf_search_range   ;}
   static float outlier_threshold()  {return instance().m_outlier_threshold  ;}
   static bool  seed_using_odometry(){return instance().m_seed_using_odometry;}
   static int   max_iterations()     {return instance().m_max_iterations     ;}
   static float tau()                {return instance().m_tau                ;}
   //@}
} ;

// Parameter initialization
Params::Params()
   : m_lrf_specs(conf("lrf_specs", make_triple(-119.0f, 1.0f, 255.0f))),
     m_lrf_search_range(clamp(conf("lrf_search_range", 15), 0, 360)),
     m_outlier_threshold(clamp(conf("outlier_threshold", 100.0f),
                               0.001f, 1000.0f)),
     m_seed_using_odometry(conf("seed_using_odometry", false)),
     m_max_iterations(clamp(conf("max_iterations", 30), 5, 100)),
     m_tau(clamp(conf("tau", 0.1f), 1e-6f, 1e+6f))
{}

//} // end of local anonymous namespace encapsulating above helpers

//--------------------- SCANS AND TRANSFORMATIONS -----------------------

// Initializing a scan's range reading given its polar and Cartesian
// coordinates.
Scan::RangeReading::RangeReading(float r, float x, float y)
   : m_r(r), m_x(x), m_y(y)
{}

// Quick function object to convert a polar range reading into
// corresponding Cartesian coordinates.
namespace {

class polar_to_cartesian :
         public std::binary_function<float, Pose, Scan::RangeReading> {
   typedef result_type R ;
   typedef first_argument_type  A1 ;
   typedef second_argument_type A2 ;
   mutable float t ;
public:
   polar_to_cartesian() ;
   R operator()(A1 r, const A2& pose) const ;
} ;

polar_to_cartesian::polar_to_cartesian()
   : t(Params::lrf_start())
{}

polar_to_cartesian::R
polar_to_cartesian::
operator()(polar_to_cartesian::A1 r, const polar_to_cartesian::A2& P) const
{
   R range_reading(r,                                // polar
                   P.x() + r * cos(P.theta() + t),   // Cartesian x
                   P.y() + r * sin(P.theta() + t)) ; // Cartesian y
   t += Params::lrf_res() ;
   return range_reading ;
}

} // end of local anonymous namespace encapsulating above helper

// Initialize a scan given its pose and the range readings
Scan::Scan(const Pose& P, const std::vector<int>& R)
   : m_pose(P)
{
   m_range_readings.reserve(R.size()) ;
   std::transform(R.begin(), R.end(), std::back_inserter(m_range_readings),
                  std::bind2nd(polar_to_cartesian(), P)) ;
}

Scan::Scan(const Pose& P, const std::vector<float>& R)
   : m_pose(P)
{
   m_range_readings.reserve(R.size()) ;
   std::transform(R.begin(), R.end(), std::back_inserter(m_range_readings),
                  std::bind2nd(polar_to_cartesian(), P)) ;
}

// Scan clean-up
Scan::~Scan(){}

// A transformation encapsulates the rotation and translation required to
// register one scan with another.
Transformation::Transformation(float w, float Tx, float Ty)
   : m_w(w), m_Tx(Tx), m_Ty(Ty)
{}

// Convenience constructor for initializing a transformation using a
// triple.
Transformation::Transformation(const triple<float, float, float>& T)
   : m_w(T.first), m_Tx(T.second), m_Ty(T.third)
{}

// Quick helper to compute the pose difference between two Scans and
// return the result via a Transformation.
static Transformation operator-(const Scan& a, const Scan& b)
{
   return Transformation(a.theta() - b.theta(), a.x() - b.x(), a.y() - b.y()) ;
}

//----------------------- POINT CORRESPONDENCES -------------------------

namespace {

// The Iterative Closest Point algorithm works by finding corresponding
// points between two scans without knowing the transformation required
// to convert one scan to the other and then using these correspondence
// pairs to iteratively solve for the unknown transformation.
//
// This local class encapsulates the correspondence. It is simply a Nx1
// matrix wherein C[i] = j implies that the i-th point in one scan
// corresponds to the j-th point in the other. A value of -1 for C[i]
// indicates that there is no correspondence for the i-th point.
class Correspondence {
   // The Nx1 correspondence matrix is simply a one-dimensional array.
   std::vector<int> C ;

public:
   // On initialization, all correspondence pairs are set to -1.
   Correspondence(int N) ;

   // Retrieving the i-th correspondence pair.
   int operator[](int i) const {return C[i] ;}

   // Setting the i-th correspondence pair.
   int& operator[](int i) {return C[i] ;}
} ;

Correspondence::Correspondence(int N)
{
   C.reserve(N) ;
   std::fill_n(std::back_inserter(C), N, -1) ;
}

// This function finds the model point closest to the i-th data point.
// Only the LRF angular range specified by [min, max] is searched.
int closest_point(const Scan& data, const Scan& model, int i, int min, int max)
{
   int cp = -1 ;
   float min_d = std::numeric_limits<float>::max() ;

   const Scan::RangeReading& D = data[i] ;
   for (int j = min; j <= max; ++j)
   {
      const Scan::RangeReading& M = model[j] ;
      if (M.r() < 0) // bad LRF reading
         continue ;

      float d = sqrtf(sqr(D.x() - M.x()) + sqr(D.y() - M.y())) ;
      if (d < min_d) {
         cp = j ;
         min_d = d ;
      }
   }
   return cp ;
}

// This function finds the correspondence pairs between the data and
// model shapes.
Correspondence find_correspondence(const Scan& data, const Scan& model)
{
   const int N = Params::lrf_num() ;
   const int n = round(Params::lrf_search_range()/Params::lrf_res()) ;

   Correspondence C(N) ;
   for (int i = 0; i < N; ++i)
   {
      const float r = data[i].r() ;
      if (r < 0) // bad LRF reading
         continue ;

      int j = closest_point(data, model,
                            i, std::max(i - n, 0), std::min(i + n, N - 1)) ;
      if (abs(r - model[j].r()) < Params::outlier_threshold())
         C[i] = j ;
      //LERROR("C[%3d] = %3d", i, C[i]) ;
   }
   return C ;
}

} // end of local anononymous namespace encapsulating above helpers

//----------------- QUATERNION BASED TRANSFORMATIONS --------------------

namespace {

// Besl and McKay represent the rotation for registering a data shape
// with a model shape using a quaternion. The translation vector is then
// tacked on to the quaternion to produce a so-called registration
// vector. The first four components of this vector are the
// quaternion-based rotation and the last three components specify the
// translation component of the transformation.
//
// We use this helper class to encapsulate the registration vector
// described above. This class also holds the mean-square error
// associated with the transformation between the data and model shapes.
class ICPTransformation {
   // The 7-component registration vector: first four components are the
   // quaternion used to represent the rotation between data and model
   // shapes; the next three components are the translation vector
   // between the two shapes.
   float q[7] ;

   // The mean-square error in the distances between the corresponding
   // points of the two shapes when the data shape is registered with the
   // model shape using the transformation specified by the registration
   // vector.
   float mserr ;

public:
   // This constructor initializes the registration vector so that the
   // rotation and translation are zero.
   ICPTransformation() ;

   // This constructor initializes the registration vector and
   // mean-square error using the supplied parameters.
   ICPTransformation(float q[7], float mserr) ;

   // This constructor initializes the registration vector using the
   // supplied z-axis rotation and 2D translation. Usually, these values
   // would be obtained from odometry. Thus, this constructor "seeds" the
   // ICP algorithm using odometry rather than zero.
   ICPTransformation(const Transformation&) ;

   // Copy and assignment.
   ICPTransformation(const ICPTransformation&) ;
   ICPTransformation& operator=(const ICPTransformation&) ;

   // Return the mean-square error associated with this registration.
   float error() const {return mserr ;}

   // A helper function to transform an LRF scan by applying the
   // registration vector held by this object. The transformed scan is
   // returned via a new Scan object.
   Scan apply(const Scan&) const ;

   // This function converts the quaternion-based rotation into the
   // corresponding Euler angle. It also extracts the translation
   // component of the transformation from the registration vector.
   //
   // NOTE: Since we are only interested in 2D for the LRF scan matching
   // procedure, the resulting transformation only needs one Euler angle
   // (corresponding to a rotation about the z-axis) and two translation
   // components (z-component is zero).
   Transformation convert() const ;
} ;

// Initialize registration vector for zero rotation and zero translation.
// This constructor is used at the start of the ICP algorithm.
ICPTransformation::ICPTransformation()
   : mserr(-1)
{
   q[0] = 1 ;
   q[1] = q[2] = q[3] = q[4] = q[5] = q[6] = 0 ;
}

// Initialize registration vector using supplied parameters. This
// constructor is used when we solve for the rotation and translation
// between data and model shapes in each iteration of the ICP algorithm.
ICPTransformation::ICPTransformation(float quat[7], float err)
   : mserr(err)
{
   q[0] = quat[0] ;
   q[1] = quat[1] ;
   q[2] = quat[2] ;
   q[3] = quat[3] ;
   q[4] = quat[4] ;
   q[5] = quat[5] ;
   q[6] = quat[6] ;
}

// Initialize registration vector using supplied rotation about z-axis
// and 2D translation vector.
ICPTransformation::ICPTransformation(const Transformation& t)
   : mserr(-1)
{
   q[0] = cos(t.w()/2) ;
   q[1] = 0 ;
   q[2] = 0 ;
   q[3] = sin(t.w()/2) ;
   q[4] = t.Tx() ;
   q[5] = t.Ty() ;
   q[6] = 0 ;
}

// Copy constructor
ICPTransformation::ICPTransformation(const ICPTransformation& that)
   : mserr(that.mserr)
{
   q[0] = that.q[0] ;
   q[1] = that.q[1] ;
   q[2] = that.q[2] ;
   q[3] = that.q[3] ;
   q[4] = that.q[4] ;
   q[5] = that.q[5] ;
   q[6] = that.q[6] ;
}

// Assignment operator
ICPTransformation& ICPTransformation::operator=(const ICPTransformation& that)
{
   if (this != &that) {
      q[0]  = that.q[0] ;
      q[1]  = that.q[1] ;
      q[2]  = that.q[2] ;
      q[3]  = that.q[3] ;
      q[4]  = that.q[4] ;
      q[5]  = that.q[5] ;
      q[6]  = that.q[6] ;
      mserr = that.mserr ;
   }
   return *this ;
}

// Apply rotation and translation to each point in the given LRF scan and
// return a new Scan as the result.
Scan ICPTransformation::apply(const Scan& s) const
{
   float R11 = sqr(q[0]) + sqr(q[1]) - sqr(q[2]) - sqr(q[3]) ;
   float R12 = 2 * (q[1]*q[2] - q[0]*q[3]) ;
   float R21 = 2 * (q[1]*q[2] + q[0]*q[3]) ;
   float R22 = sqr(q[0]) + sqr(q[2]) - sqr(q[1]) - sqr(q[3]) ;

   const int N = Params::lrf_num() ;

   Scan t(s) ;
   for (int i = 0; i < N; ++i)
   {
      Scan::RangeReading& r = t[i] ;
      r = Scan::RangeReading(r.r(),
                             R11 * r.x() + R12 * r.y() + q[4],
                             R21 * r.x() + R22 * r.y() + q[5]) ;
   }
   return t ;
}

// Convert the quaternion to an Euler angle specifying rotation about
// z-axis and extract the 2D component of the translation from the
// registration vector.
//
// NOTE: See
// http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// for explanation of these formulae.
Transformation ICPTransformation::convert() const
{
   /*
   float x = atan(2*(q[0]*q[1] + q[2]*q[3]), 1 - 2*(sqr(q[1]) + sqr(q[2]))) ;
   float y = asin(2*(q[0]*q[2] - q[3]*q[1])) ;
   // */
   float z = atan(2*(q[0]*q[3] + q[1]*q[2]), 1 - 2*(sqr(q[2]) + sqr(q[3]))) ;
   //LERROR("rotation = [%g %g %g]", x, y, z) ;

   return Transformation(z, q[4], q[5]) ;
}

} // end of local anonymous namespace encapsulating above helper class

//---------------------- ITERATIVE CLOSEST POINT ------------------------

// This function solves for the rotation and translation between two
// scans using the supplied point correspondences. Section III (pages 241
// through 243) of the Besl-McKay paper describes the math implemented
// here.
//
// NOTE: The paper works for 3D shapes. In our case, we only need 2D.
// Therefore, the formulae used in this function are slightly different
// from the math in the paper. Basically, the stuff required for 3D is
// taken out and only the necessary 2D stuff is computed.
#ifdef INVT_HAVE_LIBGSL

static ICPTransformation
solve(const Scan& data, const Scan& model, const Correspondence& C)
{
   const int N = Params::lrf_num() ;

   // First, we compute the center of mass of the data and model shapes.
   // The CoM of the data shape is denoted mu_p and that of the model is
   // denoted mu_x. See equation (23) on page 243.
   //
   // In this loop, we can also compute the cross-covariance matrix,
   // which is referred to as sigma_px in the paper and denoted as E_px
   // here. This is equation (24) on page 243.
   //
   // NOTE: Since we only need 2D, we ignore the z-components of the CoM
   // points mu_p and mu_x. Furthermore, we ignore the third row and
   // third column of the cross-covariance matrix (they are all zeros).
   int n = 0 ; // total number of correspondences
   float mu_p_x  = 0, mu_p_y  = 0, mu_x_x  = 0, mu_x_y  = 0 ;
   float E_px_11 = 0, E_px_12 = 0, E_px_21 = 0, E_px_22 = 0 ;
   for (int i = 0; i < N; ++i)
      if (C[i] >= 0) // i-th data point corresponds to something on model shape
      {
         ++n ;

         const Scan::RangeReading& D = data[i] ;
         const Scan::RangeReading& M = model[C[i]] ;

         // Equation (23), page 243
         mu_p_x += D.x() ;
         mu_p_y += D.y() ;
         mu_x_x += M.x() ;
         mu_x_y += M.y() ;

         // Equation (24), page 243
         E_px_11 += D.x() * M.x() ;
         E_px_12 += D.x() * M.y() ;
         E_px_21 += D.y() * M.x() ;
         E_px_22 += D.y() * M.y() ;
      }

   // Now we have the sums. We need the averages for the two CoM's. This
   // will complete equation (23).
   mu_p_x  /= n ;
   mu_p_y  /= n ;
   mu_x_x  /= n ;
   mu_x_y  /= n ;

   // For the cross-covariance matrix, we have the sum in the first term
   // of equation (24). Dividing by the total number of correspondences
   // will give us the first term.
   E_px_11 /= n ;
   E_px_12 /= n ;
   E_px_21 /= n ;
   E_px_22 /= n ;

   // Finally, we subtract the CoM's to complete equation (24)...
   E_px_11 -= mu_p_x * mu_x_x ;
   E_px_12 -= mu_p_x * mu_x_y ;
   E_px_21 -= mu_p_y * mu_x_x ;
   E_px_22 -= mu_p_y * mu_x_y ;

   // Now, we formulate the symmetric 4x4 matrix Q(E_px) as shown in
   // equation (25), page 243.
   //
   // NOTE: Since we're working in 2D, the four corners and the four
   // central elements of Q are the only relevant ones. The remaining
   // eight elements of the matrix are zero.
   //
   // NOTE 2: We have skipped a few things getting to this formulation.
   // First, we have to compute E_px - transpose(E_px). Then, we have to
   // use elements (2,3), (3,1) and (1,2) of the resulting matrix to
   // create a column vector denoted as Delta. Finally, trace(E_px),
   // Delta, transpose(Delta) and E_px + transpose(E_px) - trace(E_px) * I_3
   // are used to formulate Q.
   //
   // Taking advantage of the fact that we're dealing with 2D rather than
   // 3D, it turns out that only the third element of the column vector
   // Delta is significant (the other two are zero). Furthermore, as the
   // third row and third column of the cross-covariance matrix E_px
   // contain all zeros, the math for formulating Q can be done by hand.
   //
   // Putting it all together, gives us the following value for the
   // matrix Q.
   gsl_matrix* Q = gsl_matrix_alloc(4, 4) ;
   gsl_matrix_set_zero(Q) ;
   gsl_matrix_set(Q, 0, 0,  E_px_11 + E_px_22) ;
   gsl_matrix_set(Q, 0, 3,  E_px_12 - E_px_21) ;
   gsl_matrix_set(Q, 1, 1,  E_px_11 - E_px_22) ;
   gsl_matrix_set(Q, 1, 2,  E_px_12 + E_px_21) ;
   gsl_matrix_set(Q, 2, 1,  E_px_21 + E_px_12) ;
   gsl_matrix_set(Q, 2, 2,  E_px_22 - E_px_11) ;
   gsl_matrix_set(Q, 3, 0,  E_px_12 - E_px_21) ;
   gsl_matrix_set(Q, 3, 3, -E_px_11 - E_px_22) ;

   // Once we have Q(E_px), we can solve for the rotation as the
   // eigenvector corresponding to the maximum eigenvalue of the matrix
   // Q. See the paragraph between equations (25) and (26) on page 243 of
   // the Besl-McKay paper.
   gsl_eigen_symmv_workspace* w = gsl_eigen_symmv_alloc(4) ;
   gsl_vector* eigen_values  = gsl_vector_alloc(4) ;
   gsl_matrix* eigen_vectors = gsl_matrix_alloc(4, 4) ;

   gsl_eigen_symmv(Q, eigen_values, eigen_vectors, w) ;
   gsl_eigen_symmv_sort(eigen_values, eigen_vectors, GSL_EIGEN_SORT_VAL_DESC) ;

   float q[7] ;
   q[0] = static_cast<float>(gsl_matrix_get(eigen_vectors, 0, 0)) ;
   q[1] = static_cast<float>(gsl_matrix_get(eigen_vectors, 1, 0)) ;
   q[2] = static_cast<float>(gsl_matrix_get(eigen_vectors, 2, 0)) ;
   q[3] = static_cast<float>(gsl_matrix_get(eigen_vectors, 3, 0)) ;

   gsl_matrix_free(eigen_vectors) ;
   gsl_vector_free(eigen_values) ;
   gsl_eigen_symmv_free(w) ;
   gsl_matrix_free(Q) ;

   // This is the rotation matrix associated with the above quaternion.
   // See equation (21) on page 243.
   //
   // NOTE: Since we're dealing with 2D, the third row and third column
   // of the rotation matrix R are of no consequence to us. Therefore, we
   // can get by with just the top-left four elements of R.
   float R11 = sqr(q[0]) + sqr(q[1]) - sqr(q[2]) - sqr(q[3]) ;
   float R12 = 2 * (q[1]*q[2] - q[0]*q[3]) ;
   float R21 = 2 * (q[1]*q[2] + q[0]*q[3]) ;
   float R22 = sqr(q[0]) + sqr(q[2]) - sqr(q[1]) - sqr(q[3]) ;

   // To find the translation between the data shape and the model, we
   // rotate the CoM of the data shape and then subtract the result from
   // the CoM of the model shape. See equation (26) on page 243.
   //
   // NOTE: Due to our concern with only 2D, the z-component of the
   // translation vector is zero.
   q[4] = mu_x_x - (R11 * mu_p_x + R12 * mu_p_y) ;
   q[5] = mu_x_y - (R21 * mu_p_x + R22 * mu_p_y) ;
   q[6] = 0 ;

   // Finally, we need to figure out the amount of error between the data
   // shape and the model corresponding to the transformation computed
   // above. We do that by applying the rotation and translation to each
   // point in the data shape and then finding the Euclidean distance to
   // the corresponding point on the model shape. The sum of the squares
   // of each of these distances will be the mean-square error that we're
   // interested in.
   //
   // This loop implements equation (22) on page 243 of the Besl-McKay
   // paper. As usual, we have omitted the z-component of the
   // transformation because, for Robolocust's scan matching, we're
   // working in two dimensions.
   float  err = 0 ;
   for (int i = 0; i < N; ++i)
      if (C[i] >= 0) // i-th data point corresponds to something on model shape
      {
         const Scan::RangeReading& D = data[i] ;
         const Scan::RangeReading& M = model[C[i]] ;

         err += sqr(M.x() - (R11 * D.x() + R12 * D.y()) - q[4])
              + sqr(M.y() - (R21 * D.x() + R22 * D.y()) - q[5]) ;
      }
   err /= n ;

   /*
   LERROR("q = [%8.3f %8.3f %8.3f %8.3f] [%8.3f %8.3f %8.3f]",
          q[0], q[1], q[2], q[3], q[4], q[5], q[6]) ;
   // */

   // That's it: we have the quaternion-based rotation and translation
   // plus the mean-square error associated with this transformation. We
   // bundle these two things together and return the result for use in
   // step b of the ICP algorithm (see section A, page 244 of the
   // Besl-McKay paper).
   return ICPTransformation(q, err) ;
}

#else // GNU Scientific Library not available

// Can't do eigenvalue and eigenvector computations
static ICPTransformation solve(const Scan&, const Scan&, const Correspondence&)
{
   throw missing_libs(MISSING_LIBGSL) ;
}

#endif // INVT_HAVE_LIBGSL

// This function implements the Iterative Closest Point algorithm
// described by Besl and McKay. The goal for Robolocust is to match two
// laser range finder scans and return the transformation required to
// properly register the previous scan with reference to the current one.
//
// In terms of the paper's terminology: we consider the current scan to
// be the model shape and the previous one to be the data shape; we want
// to register the data (previous scan) to the model (current scan). The
// resulting transformation required to go from the previous scan to the
// current one should then be able to correct for odometric drift/errors.
Transformation match_scans(const Scan& curr, const Scan& prev)
{
   // ICP init: the transformation required to register the data shape to
   // the model shape starts off with zero rotation and zero translation.
   // Additionally, the intermediate data shape that converges onto the
   // model shape as the algorithm progresses is intialized using the
   // original data shape.
   //
   // NOTE: In the paper, the authors use P_k and P_[k+1] to denote the
   // fact that the "intermediate" data shape changes with each iteration
   // of the algorithm. Here, we simply use the variable P and omit the
   // subscripts.
   ICPTransformation q ;
   Scan P = prev ;

   // Optionally, instead of zeros, we can use raw odometry to "seed" the
   // initial transformation.
   if (Params::seed_using_odometry()) {
      q = curr - prev ;
      P = q.apply(prev) ;
   }

   // We will need to keep track of the mean square errors in the
   // transformation computed in each iteration of the ICP algorithm so
   // that we know when we can terminate (when the error drops below some
   // preset threshold).
   float prev_error = std::numeric_limits<float>::max() ;

   // ICP loop: now, we solve for the rotation and translation required
   // to align the data shape (prev scan) to the model shape (current
   // scan) until the mean square error associated with transformation
   // drops below some preset threshold. To ensure that this loop doesn't
   // go on forever (in case the algorithm never converges), we cap the
   // maximum number of iterations.
   for (int i = 1; i <= Params::max_iterations(); ++i)
   {
      // Steps a and b (section A, page 244): find the correspondence
      // pairs between the data and model shapes and then use these
      // correspondences to compute the appropriate rotation and
      // translation required for the registration.
      q = solve(prev, curr, find_correspondence(P, curr)) ;

      // Step c (section A, page 244): apply the above registration to
      // the data shape to get the new intermediate, registered data
      // shape, viz., P_[k+1].
      P = q.apply(prev) ;

      // Step d (section A, page 244): terminate the iteration when the
      // change in the mean-square error falls below a preset threshold
      // tau.
      float error = abs(prev_error - q.error()) ;
      /*
      Transformation t = q.convert() ;
      LERROR("iter %3d: [%8.3f%9.3f%7.1f] [%9.3e %10.3e]",
             i, t.Tx(), t.Ty(), t.w(), q.error(), error) ;
      // */
      if (error < Params::tau())
         break ;

      // Setup for next iteration
      prev_error = q.error() ;
   }

   // Either ICP converged within the max allowable iterations or it
   // didn't. Either ways, return the latest transformation.
   return q.convert() ;
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
