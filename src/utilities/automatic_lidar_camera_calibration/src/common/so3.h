#ifndef __SO3_H__
#define __SO3_H__

#include "fasttrig.h"

#ifdef __cplusplus
extern "C" {
#endif


/*
% ROTX  Compute a rotation matrix about the X-axis.
%   R = ROTX(PHI) returns [3x3] rotation matrix R.  Note: PHI
%   is measured in radians.  PHI measures orientation of coordinate
%   frame 2 relative to coordinate frame 1.  Multiplication by rotation
%   matrix R rotates a vector in coordinate frame 1 into coordinate
%   frame 2.
*/
static inline void
so3_rotx (double Rx[9], const double phi)
{
    double s, c;
    fsincos (phi, &s, &c);
    const double data[] = { 1,  0, 0,
                            0,  c, s,
                            0, -s, c };
    memcpy (Rx, data, 9 * sizeof(double));
}

/*
% ROTY  Compute a rotation matrix about the Y-axis.
%   R = ROTY(THETA) returns [3x3] rotation matrix R.  Note: THETA
%   is measured in radians.  THETA measures orientation of coordinate
%   frame 2 relative to coordinate frame 1.  Multiplication by rotation
%   matrix R rotates a vector in coordinate frame 1 into coordinate
%   frame 2.
*/
static inline void
so3_roty (double Ry[9], const double theta)
{
    double s, c;
    fsincos (theta, &s, &c);
    const double data[] = { c, 0, -s,
                            0, 1,  0,
                            s, 0,  c };
    memcpy (Ry, data, 9 * sizeof(double));
}

/*
% ROTZ  Compute a rotation matrix about the Z-axis.
%   R = ROTZ(PSI) returns [3x3] rotation matrix R.  Note: PSI
%   is measured in radians.  PSI measures orientation of coordinate
%   frame 2 relative to coordinate frame 1.  Multiplication by rotation
%   matrix R rotates a vector in coordinate frame 1 into coordinate
%   frame 2.
*/
static inline void
so3_rotz (double Rz[9], const double psi)
{
    double s, c;
    fsincos (psi, &s, &c);
    const double data[] = { c, s, 0,
                           -s, c, 0,
                            0, 0, 1 };
    memcpy (Rz, data, 9 * sizeof(double));
}


/*
% ROTXYZ  Compute a rotation matrix about the XYZ-axes.
%   R = ROTXYZ(RPH) returns [3x3] rotation matrix R where RPH
%   is a 3-vector of Euler angles [roll,pitch,heading] measured in
%   radians.  RPH measures orientation of coordinate frame 2
%   relative to coordinate frame 1.  Multiplication by rotation
%   matrix R rotates a vector in coordinate frame 2 into coordinate
%   frame 1.
*/
static inline void
so3_rotxyz (double R[9], const double rph[3])
{
    double sr, sp, sh, cr, cp, ch;
    fsincos (rph[0], &sr, &cr);
    fsincos (rph[1], &sp, &cp);
    fsincos (rph[2], &sh, &ch);
    const double data[] = {  ch*cp, -sh*cr + ch*sp*sr,  sh*sr + ch*sp*cr,
                             sh*cp,  ch*cr + sh*sp*sr, -ch*sr + sh*sp*cr,
                            -sp,     cp*sr,             cp*cr             };
    memcpy (R, data, 9 * sizeof(double));
}


/*
%ROT2RPH  Convert rotation matrix into Euler roll,pitch,heading.
%   RPH = ROT2RPH(R) computes 3-vector of Euler angles
%   [roll,pitch,heading] from [3x3] rotation matrix R.  Angles are
%   measured in radians.
%
*/
static inline void
so3_rot2rph (const double R[9], double rph[3])
{
#define R(i,j) (R[i*3+j])
    // heading
    rph[2] = fatan2 (R(1,0), R(0,0));
    double sh, ch;
    fsincos (rph[2], &sh, &ch);

    // pitch
    rph[1] = fatan2 (-R(2,0), R(0,0)*ch + R(1,0)*sh);

    // roll
    rph[0] = fatan2 (R(0,2)*sh - R(1,2)*ch, -R(0,1)*sh + R(1,1)*ch);
#undef R
}

/*
% DROTX  Compute the derivative of a rotation matrix about the X-axis.
%   dRx = DROTZ(PHI) returns [3x3] rotation matrix dRx.  Note: PHI
%   is measured in radians.  PHI measures orientation of coordinate
%   frame 2 relative to coordinate frame 1.
*/
static inline void
so3_drotx (double dRx[3], const double phi)
{
    double s, c;
    fsincos (phi, &s, &c);
    const double data[9] = { 0,  0,  0,
                             0, -s,  c,
                             0, -c, -s };
    memcpy (dRx, data, 9 * sizeof(double));
}


/*
% DROTY  Compute the derivative of a rotation matrix about the Y-axis.
%   dRy = DROTY(THETA) returns [3x3] rotation matrix dRy.  Note: THETA
%   is measured in radians.  THETA measures orientation of coordinate
%   frame 2 relative to coordinate frame 1.
*/
static inline void
so3_droty (double dRy[9], const double theta)
{
    double s, c;
    fsincos (theta, &s, &c);
    const double data[9] = { -s,  0, -c,
                              0,  0,  c,
                              c,  0, -s };
    memcpy (dRy, data, 9 * sizeof(double));
}


/*
% DROTZ  Compute the derivative of a rotation matrix about the Z-axis.
%   dRz = DROTZ(PSI) returns [3x3] rotation matrix dRz.  Note: PSI
%   is measured in radians.  PSI measures orientation of coordinate
%   frame 2 relative to coordinate frame 1.
*/
static inline void
so3_drotz (double dRz[9], const double theta)
{
    double s, c;
    fsincos (theta, &s, &c);
    const double data[9] = { -s,  c,  0,
                             -c, -s,  0,
                              0,  0,  0 };
    memcpy (dRz, data, 9 * sizeof(double));
}

/*
% QUAT2ROT quaternion to rotation matrix.
%   R = QUAT2ROT(q) takes a 4-vector unit quaternion reprsented by q, 
%   (i.e. q = [q0;qx;qy;qz]) and returns the corresponding [3 x 3] 
%   orthonormal rotation matrix R.
*/
static inline void
so3_quat2rot (const double q[4], double R[9])
{
    const double q0 = q[0], qx = q[1], qy = q[2], qz = q[3];

    const double q0q0 = q0*q0;
    const double qxqx = qx*qx;
    const double qyqy = qy*qy;
    const double qzqz = qz*qz;

    const double q0qx = q0*qx;
    const double q0qz = q0*qz;
    const double q0qy = q0*qy;
    const double qxqy = qx*qy;
    const double qxqz = qx*qz;
    const double qyqz = qy*qz;

    const double data[9] = 
        { q0q0 + qxqx - qyqy - qzqz,  2*(qxqy - q0qz),            2*(qxqz + q0qy),
          2*(qxqy + q0qz),            q0q0 - qxqx + qyqy - qzqz,  2*(qyqz - q0qx),
          2*(qxqz - q0qy),            2*(qyqz + q0qx),            q0q0 - qxqx - qyqy + qzqz };
    memcpy (R, data, 9 * sizeof(double));
}

/*
% QUAT2RPH converts unit quaternion to Euler RPH.
%   rph = QUAT2RPH(q) returns a [3 x 1] Euler xyz representation
%   equivalent to the [4 x 1] unit quaternion (provided q is
%   not near an Euler singularity).
*/
static inline void
so3_quat2rph (const double q[4], double rph[3])
{
    double R[9];
    so3_quat2rot (q, R);
    so3_rot2rph (R, rph);
}

#ifdef __cplusplus
}
#endif

#endif // __SO3_H__
