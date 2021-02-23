"""
Primitive operations for 3x3 orthonormal and 4x4 homogeneous matrices.

Python implementation by: Luis Fernando Lara Tobar and Peter Corke.
Based on original Robotics Toolbox for Matlab code by Peter Corke.
Permission to use and copy is granted provided that acknowledgement of
the authors is made.

@author: Luis Fernando Lara Tobar and Peter Corke
@revisor: Avelino Forechi updated to the most recent Matlab version
"""

from numpy import *
from utility import *
from numpy.linalg import norm


def rotx(theta):
    """
    Rotation about X-axis
    
    @type theta: number
    @param theta: the rotation angle
    @rtype: 3x3 orthonormal matrix
    @return: rotation about X-axis

    @see: L{roty}, L{rotz}, L{rotvec}
    """
    
    ct = cos(theta)
    st = sin(theta)
    return mat([[1,  0,    0],
                [0,  ct, -st],
                [0,  st,  ct]])


def roty(theta):
    """
    Rotation about Y-axis
    
    @type theta: number
    @param theta: the rotation angle
    @rtype: 3x3 orthonormal matrix
    @return: rotation about Y-axis

    @see: L{rotx}, L{rotz}, L{rotvec}
    """
    
    ct = cos(theta)
    st = sin(theta)

    return mat([[ct,   0,   st],
                [0,    1,    0],
                [-st,  0,   ct]])


def rotz(theta):
    """
    Rotation about Z-axis
    
    @type theta: number
    @param theta: the rotation angle
    @rtype: 3x3 orthonormal matrix
    @return: rotation about Z-axis

    @see: L{rotx}, L{roty}, L{rotvec}
    """
    
    ct = cos(theta)
    st = sin(theta)

    return mat([[ct,  -st,  0],
                [st,   ct,  0],
                [ 0,    0,  1]])


def trotx(theta):
    """
    Rotation about X-axis
    
    @type theta: number
    @param theta: the rotation angle
    @rtype: 4x4 homogeneous matrix
    @return: rotation about X-axis

    @see: L{troty}, L{trotz}, L{rotx}
    """
    return r2t(rotx(theta))


def troty(theta):
    """
    Rotation about Y-axis
    
    @type theta: number
    @param theta: the rotation angle
    @rtype: 4x4 homogeneous matrix
    @return: rotation about Y-axis

    @see: L{troty}, L{trotz}, L{roty}
    """
    return r2t(roty(theta))


def trotz(theta):
    """
    Rotation about Z-axis
    
    @type theta: number
    @param theta: the rotation angle
    @rtype: 4x4 homogeneous matrix
    @return: rotation about Z-axis

    @see: L{trotx}, L{troty}, L{rotz}
    """
    return r2t(rotz(theta))


def tr2rpy(m):
    """
    Extract RPY angles.
    Returns a vector of RPY angles corresponding to the rotational part of 
    the homogeneous transform.  The 3 angles correspond to rotations about
    the Z, Y and X axes respectively.
    
    @type m: 3x3 or 4x4 matrix
    @param m: the rotation matrix
    @rtype: 1x3 matrix
    @return: RPY angles [S{theta} S{phi} S{psi}]
    
    @see:  L{rpy2tr}, L{tr2eul}
    """
    m = mat(m)
    if ishomog(m)==False:
        error('input must be a homogeneous transform')
    rpy = mat(zeros((1,3)))
    if abs(abs(m[2,0])-1)<finfo(float).eps:
        # singularity
        rpy[0,0] = 0
        rpy[0,1] = -arcsin(m[2,0])
        if m[2,0] < 0:
            rpy[0,2] = -arctan2(m[0,1], m[0,2])
        else:
            rpy[0,2] = arctan2(-m[0, 1], -m[0, 2])
    else:
        rpy[0,0] = arctan2(m[2,1],m[2,2])
        rpy[0,1] = arctan2(-m[2,0]*cos(rpy[0,0]), m[2,2])
        rpy[0,2] = arctan2(m[1,0], m[0,0])
    return rpy


def rpy2r(roll, pitch=None,yaw=None,order='vehicle'):
    """
    Rotation from RPY angles.
    
    Two call forms:
        - R = rpy2r(S{theta}, S{phi}, S{psi})
        - R = rpy2r([S{theta}, S{phi}, S{psi}])
    These correspond to rotations about the Z, Y, X axes respectively.

    @type roll: number or list/array/matrix of angles
    @param roll: roll angle, or a list/array/matrix of angles
    @type pitch: number
    @param pitch: pitch angle
    @type yaw: number
    @param yaw: yaw angle
    @rtype: 4x4 homogenous matrix
    @return: R([S{theta} S{phi} S{psi}])

    @see:  L{tr2rpy}, L{rpy2r}, L{tr2eul}

    """
    n=1
    if pitch==None and yaw==None:
        roll= mat(roll)
        if numcols(roll) != 3:
            error('bad arguments')
        n = numrows(roll)
        pitch = roll[:,1]
        yaw = roll[:,2]
        roll = roll[:,0]
    if n>1:
        R = []
        for i in range(0,n):
            if order=='vehicle':
                r = rotz(yaw[i,0]) * roty(pitch[i,0]) * rotx(roll[i,0])
            else:
                r = roty(yaw[i, 0]) * rotx(pitch[i, 0]) * rotz(roll[i, 0])
            R.append(r)
        return R
    try:
        if order == 'vehicle':
            r = rotz(yaw[0,0]) * roty(pitch[0,0]) * rotx(roll[0,0])
        else:
            r = roty(yaw[0, 0]) * rotx(pitch[0, 0]) * rotz(roll[0, 0])
        return r
    except:
        if order == 'vehicle':
            r = rotz(yaw) * roty(pitch) * rotx(roll)
        else:
            r = roty(yaw) * rotx(pitch) * rotz(roll)
        return r


def rpy2tr(roll, pitch=None, yaw=None):
    """
    Rotation from RPY angles.
    
    Two call forms:
        - R = rpy2tr(r, p, y)
        - R = rpy2tr([r, p, y])
    These correspond to rotations about the Z, Y, X axes respectively.

    @type roll: number or list/array/matrix of angles
    @param roll: roll angle, or a list/array/matrix of angles
    @type pitch: number
    @param pitch: pitch angle
    @type yaw: number
    @param yaw: yaw angle
    @rtype: 4x4 homogenous matrix
    @return: R([S{theta} S{phi} S{psi}])

    @see:  L{tr2rpy}, L{rpy2r}, L{tr2eul}

    """
    return r2t( rpy2r(roll, pitch, yaw) )


def tr2t(T):
    if ishomog(T)==False:
        error('input must be a homogeneous transform')
    return T[0:3,3]


def t2r(T):
    """
    Return rotational submatrix of a homogeneous transformation.
    @type T: 4x4 homogeneous transformation
    @param T: the transform matrix to convert
    @rtype: 3x3 orthonormal rotation matrix
    @return: rotation submatrix
    """    
    
    if ishomog(T)==False:
        error('input must be a homogeneous transform')
    return T[0:3,0:3]


def r2t(R):
    """
    Convert a 3x3 orthonormal rotation matrix to a 4x4 homogeneous transformation::
    
        T = | R 0 |
            | 0 1 |
            
    @type R: 3x3 orthonormal rotation matrix
    @param R: the rotation matrix to convert
    @rtype: 4x4 homogeneous matrix
    @return: homogeneous equivalent
    """
    
    return concatenate( (concatenate( (R, zeros((3,1))),1), mat([0,0,0,1])) )


def rt2tr(R, t):
    """
    Convert rotation and translation to homogeneous transform

    TR = RT2TR(R, t) is a homogeneous transformation matrix (N+1xN+1) formed
    from an orthonormal rotation matrix R (NxN) and a translation vector t
    (Nx1).  Works for R in SO(2) or SO(3):
    - If R is 2x2 and t is 2x1, then TR is 3x3
    - If R is 3x3 and t is 3x1, then TR is 4x4

    @see also L{T2R}, L{R2T}, L{TR2RT}.

    @param R: 3x3 orthonormal rotation matrix
    @param t: 3x1 vector
    @return: T 4x4 homogeneous transformation
    """
    if numcols(R) != numrows(R):
        error('R must be square')

    if numrows(R) != numrows(t):
        error('R and t must have the same number of rows')

    h = hstack( (zeros(numcols(R)), 1) )
    T = hstack( (R, t) )
    T = vstack( (T, h) )

    return T


def cam2tr():
    '''
    ---------------------------------------
        Y              |
        |              |
        |              |
        |              |
        W--------- X   |   C--------- Z
       /               |   | \
      /                |   |  \
     /                 |   |   \
    Z                  |   X    Y
    ---------------------------------------
    :return: 4x4 homogeneous matrix
    '''
    transf = mat([[0, -1, 0],  # camera x-axis wrt world
                  [0, 0, -1],  # camera y-axis wrt world
                  [1, 0, 0]])  # camera z-axis wrt world
    transf = hstack((transf, [[0],  # translation along x
                                 [0],  # translation along y
                                 [0]]  # translation along z
                        ))
    transf = vstack((transf, [0, 0, 0, 1]))  # homogeneous coordinate
    return transf


def euler_to_quat(roll, pitch, yaw):
    '''
    https://openslam.informatik.uni-freiburg.de/data/svn/MTK/trunk/cpp/example/relation/euler_stable.hpp
    Euler to Quaternion conversion.
    :param roll:
           pitch:
           yaw:
    :return: quaternion
    '''
    sr = math.sin(roll * 0.5)
    cr = math.cos(roll * 0.5)
    sp = math.sin(pitch * 0.5)
    cp = math.cos(pitch * 0.5)
    sy = math.sin(yaw * 0.5)
    cy = math.cos(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return array((x, y, z, w))


def quat_to_euler(quaternion):
    '''
    https://openslam.informatik.uni-freiburg.de/data/svn/MTK/trunk/cpp/example/relation/euler_stable.hpp
    Convert Quaternion quat to Euler angles, gracefully handling the singularity for abs(pitch) = PI/2.
    Passing the result of this method to the method above should always result in quat or -quat assuming quat was normalized.
    This method also has no problems handling non-normalized Quaternions.
    :param quaternion:
    :return:
    '''
    # Get yaw angle:
    qx = quaternion[0]  # x
    qy = quaternion[1]  # y
    qz = quaternion[2]  # z
    qw = quaternion[3]  # w

    qx2 = qx * qx
    qy2 = qy * qy
    qz2 = qz * qz
    qw2 = qw * qw
    # for abs(pitch) = PI/2 this will lead to atan2(0,0)
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), qw2 + qx2 - qy2 - qz2)

    # Now rotate the original Quaternion backwards by yaw:
    c = math.cos(yaw / 2.0)
    s = math.sin(yaw / 2.0)
    px = c * qx + s * qy
    py = c * qy - s * qx
    pz = c * qz - s * qw
    pw = c * qw + s * qz
    px2 = px * px
    py2 = py * py
    pz2 = pz * pz
    pw2 = pw * pw

    # Now calculating pitch and roll does not have singularities anymore:
    pitch = math.atan2(2 * (py * pw - px * pz), px2 + pw2 - py2 - pz2)
    roll = math.atan2(2 * (px * pw - py * pz), py2 + pw2 - px2 - pz2)
    return array((roll, pitch, yaw))