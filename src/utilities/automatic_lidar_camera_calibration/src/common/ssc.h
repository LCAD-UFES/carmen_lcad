#ifndef __SSC_H__
#define __SSC_H__

#include <string.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>

/* 6-DOF pose
 * defined as a column vector of X = [x, y, z, r, p, h]' 
*/
#define SSC_DOF_X 0
#define SSC_DOF_Y 1
#define SSC_DOF_Z 2
#define SSC_DOF_R 3
#define SSC_DOF_P 4
#define SSC_DOF_H 5

#define SSC_DOF_XYZ 0
#define SSC_DOF_RPH 3

#define SSC_XYZ_X 0 // x,y,z indexing within xyz subvector
#define SSC_XYZ_Y 1
#define SSC_XYZ_Z 2

#define SSC_RPH_R 0 // r,p,h indexing within rph subvector
#define SSC_RPH_P 1
#define SSC_RPH_H 2

// 6-DOF pose accessor macros
#define SSC_X(v) (v[SSC_DOF_X])
#define SSC_Y(v) (v[SSC_DOF_Y])
#define SSC_Z(v) (v[SSC_DOF_Z])
#define SSC_R(v) (v[SSC_DOF_R])
#define SSC_P(v) (v[SSC_DOF_P])
#define SSC_H(v) (v[SSC_DOF_H])
#define SSC_XYZ(v) (&v[SSC_DOF_XYZ])
#define SSC_RPH(v) (&v[SSC_DOF_RPH])

#ifdef __cplusplus
extern "C" {
#endif

// convenient type definitions
typedef double ssc_pose_t[6];      // ssc_pose_t   foo == double foo[6]
typedef double ssc_Jminus_t[6*6];  // ssc_Jminus_t foo == double foo[6*6]
typedef double ssc_Jplus_t[6*12];  // ssc_Jplus_t  foo == double foo[6*12]
typedef double ssc_Jtail_t[6*12];  // ssc_Jtail_t  foo == double foo[6*12]

// 6-DOF pose accessor helpers
static inline void
ssc_pose_get (const double X[6], double xyzrph[6])
{
    xyzrph[0] = X[SSC_DOF_X];
    xyzrph[1] = X[SSC_DOF_Y];
    xyzrph[2] = X[SSC_DOF_Z];
    xyzrph[3] = X[SSC_DOF_R];
    xyzrph[4] = X[SSC_DOF_P];
    xyzrph[5] = X[SSC_DOF_H];
}

static inline void
ssc_pose_set (ssc_pose_t X, const double xyzrph[6])
{
    X[SSC_DOF_X] = xyzrph[0];
    X[SSC_DOF_Y] = xyzrph[1];
    X[SSC_DOF_Z] = xyzrph[2];
    X[SSC_DOF_R] = xyzrph[3];
    X[SSC_DOF_P] = xyzrph[4];
    X[SSC_DOF_H] = xyzrph[5];
}

static inline void
ssc_pose_get_i (const double X[6],
                double x, double y, double z,
                double r, double p, double h)
{
    x = X[SSC_DOF_X];
    y = X[SSC_DOF_Y];
    z = X[SSC_DOF_Z];
    r = X[SSC_DOF_R];
    p = X[SSC_DOF_P];
    h = X[SSC_DOF_H];
}

static inline void
ssc_pose_set_i (ssc_pose_t X, 
                const double x, const double y, const double z, 
                const double r, const double p, const double h)
{
    X[SSC_DOF_X] = x;
    X[SSC_DOF_Y] = y;
    X[SSC_DOF_Z] = z;
    X[SSC_DOF_R] = r;
    X[SSC_DOF_P] = p;
    X[SSC_DOF_H] = h;
}

static inline void
ssc_pose_get_xyz (const ssc_pose_t X, double xyz[3])
{
    xyz[0] = X[SSC_DOF_X];
    xyz[1] = X[SSC_DOF_Y];
    xyz[2] = X[SSC_DOF_Z];
}

static inline void
ssc_pose_set_xyz (ssc_pose_t X, const double xyz[3])
{
    X[SSC_DOF_X] = xyz[0];
    X[SSC_DOF_Y] = xyz[1];
    X[SSC_DOF_Z] = xyz[2];
}

static inline void
ssc_pose_get_rph (const ssc_pose_t X, double rph[3])
{
    rph[0] = X[SSC_DOF_R];
    rph[1] = X[SSC_DOF_P];
    rph[2] = X[SSC_DOF_H];
}

static inline void
ssc_pose_set_rph (ssc_pose_t X, const double rph[3])
{
    X[SSC_DOF_R] = rph[0];
    X[SSC_DOF_P] = rph[1];
    X[SSC_DOF_H] = rph[2];
}


static inline void
ssc_pose_get_xyz_i (const ssc_pose_t X, double x, double y, double z)
{
    x = X[SSC_DOF_X];
    y = X[SSC_DOF_Y];
    z = X[SSC_DOF_Z];
}

static inline void
ssc_pose_set_xyz_i (ssc_pose_t X, const double x, const double y, const double z)
{
    X[SSC_DOF_X] = x;
    X[SSC_DOF_Y] = y;
    X[SSC_DOF_Z] = z;
}

static inline void
ssc_pose_get_rph_i (const ssc_pose_t X, double r, double p, double h)
{
    r = X[SSC_DOF_R];
    p = X[SSC_DOF_P];
    h = X[SSC_DOF_H];
}

static inline void
ssc_pose_set_rph_i (ssc_pose_t X, const double r, const double p, const double h)
{
    X[SSC_DOF_R] = r;
    X[SSC_DOF_P] = p;
    X[SSC_DOF_H] = h;
}

/*
 * Jacobians in ssc_inverse, head2tail, and tail2tail are stored in row-major
 * order such that the (i,j) index is accessed as J(i*6+j) for ssc_inverse
 * and J(i*12+j) for ssc_head2tail and ssc_tail2tail.
 */


/*
% SSC_INVERSE  6-DOF coordinate frame relationship.
%   [X_ji,J_MINUS] = SSC_INVERSE(X_ij) returns the Smith, Self, and
%   Cheeseman inverse coordinate frame relationship:
%     X_ji = (-)X_ij
%          = [x_ji,y_ji,z_ji,r_ji,p_ji,h_ji]'
%   where
%     X_ij is the 6-DOF representation of frame j w.r.t. frame i, X_ji is
%     the 6-DOF representation of frame i w.r.t. frame j, J_MINUS is the
%     Jacobian of the inverse operation, i.e. J_MINUS = d(X_ji)/d(X_ij)
%     evaluated at X_ij.
%
%   The above notation and associated Jacobian are based upon*: R. Smith,
%   M. Self, and P. Cheeseman.  "Estimating Uncertain Spatial Relationships
%   in Robotics".
%
%   Set Jminus = NULL to not compute Jacobian.
*/
int
ssc_inverse (ssc_pose_t x_ji, ssc_Jminus_t Jminus, const ssc_pose_t x_ij);


/*
% SSC_HEAD2TAIL  6-DOF coordinate frame composition.
%   [X_ik,J_PLUS] = SSC_HEAD2TAIL(X_ij,X_jk) returns the Smith, Self, and
%   Cheesman head-to-tail coordinate frame composition:
%     X_ik = X_ij (+) X_jk
%          = [x_ik,y_ik,z_ik,r_ik,p_ik,h_ik]'  
%   where
%     X_ij is the 6-DOF representation of frame j w.r.t. frame i, X_jk is
%     the 6-DOF representation of frame k w.r.t. frame j, X_ik is the 6-DOF
%     representation of frame k w.r.t. frame i, J_PLUS is the Jacobian of
%     the composition operation, i.e. J_PLUS = d(X_ik)/d(X_ij,X_jk)
%     evaluated at X_ij, X_jk.
%
%   The above notation and associated Jacobian are based upon*: R. Smith,
%   M. Self, and P. Cheeseman.  "Estimating Uncertain Spatial Relationships
%   in Robotics".
%
%   Set Jplus = NULL to not compute Jacobian.
*/
int
ssc_head2tail (ssc_pose_t x_ik, ssc_Jplus_t Jplus, const ssc_pose_t x_ij, const ssc_pose_t x_jk);


/*
% SSC_TAIL2TAIL  6-DOF coordinate frame composition.
%   [X_jk,J] = SSC_TAIL2TAIL(X_ij,X_ik) returns the Smith, Self, and
%   Cheeseman tail-to-tail coordinate frame composition:
%     X_jk = (-)X_ij (+) X_ik
%          = [x_jk,y_jk,z_jk,r_jk,p_jk,h_jk]'
%   where
%     X_ij is the 6-DOF representation of frame j w.r.t. frame i, X_ik is
%     the 6-DOF representation of frame k w.r.t. frame i, X_jk is the 6-DOF
%     representation of frame k w.r.t. frame j, J is the Jacobian of the
%     tail-to-tail operation, i.e. J = d(X_jk)/d(X_ij,X_ik) evaluated at
%     X_ij, X_ik.
%
%   The above notation and associated Jacobian are based upon*: R. Smith,
%   M. Self, and P. Cheeseman.  "Estimating Uncertain Spatial Relationships
%   in Robotics".
%
%   Set J = NULL to not compute Jacobian.
*/
int
ssc_tail2tail (ssc_pose_t x_jk, ssc_Jtail_t J, const ssc_pose_t x_ij, const ssc_pose_t x_ik);



#ifdef __cplusplus
}
#endif

#endif // __SSC_H__
