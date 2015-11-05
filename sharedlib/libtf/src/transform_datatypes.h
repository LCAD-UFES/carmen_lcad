/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Tully Foote */

#ifndef TF_TRANSFORM_DATATYPES_H
#define TF_TRANSFORM_DATATYPES_H

#include <string>
#include "tf_time.h"
#include "linear_math/Transform.h"

namespace tf
{

typedef tf::Vector3 Point;
typedef tf::Transform Pose;

static const double QUATERNION_TOLERANCE = 0.1f;

/** \brief The data type which will be cross compatable with geometry_msgs
 * This is the tf datatype equivilant of a MessageStamped */
template <typename T>
class Stamped : public T{
 public:
  Time stamp_; ///< The timestamp associated with this data
  std::string frame_id_; ///< The frame_id associated this data

  /** Default constructor */
  Stamped() :frame_id_ ("NO_ID_STAMPED_DEFAULT_CONSTRUCTION"){}; //Default constructor used only for preallocation

  /** Full constructor */
  Stamped(const T& input, const Time& timestamp, const std::string & frame_id) :
    T (input), stamp_ ( timestamp ), frame_id_ (frame_id){ } ;
  
  /** Set the data element */
  void setData(const T& input){*static_cast<T*>(this) = input;};
};

/** \brief Comparison Operator for Stamped datatypes */
template <typename T> 
bool operator==(const Stamped<T> &a, const Stamped<T> &b) {
  return a.frame_id_ == b.frame_id_ && a.stamp_ == b.stamp_ && static_cast<const T&>(a) == static_cast<const T&>(b);
};


/** \brief The Stamped Transform datatype used by tf */
class StampedTransform : public tf::Transform
{
public:
  Time stamp_; ///< The timestamp associated with this transform
  std::string frame_id_; ///< The frame_id of the coordinate frame  in which this transform is defined
  std::string child_frame_id_; ///< The frame_id of the coordinate frame this transform defines
  StampedTransform(const tf::Transform& input, const Time& timestamp, const std::string & frame_id, const std::string & child_frame_id):
    tf::Transform (input), stamp_ ( timestamp ), frame_id_ (frame_id), child_frame_id_(child_frame_id){ };

  /** \brief Default constructor only to be used for preallocation */
  StampedTransform() { };

  /** \brief Set the inherited Traonsform data */
  void setData(const tf::Transform& input){*static_cast<tf::Transform*>(this) = input;};

};

/** \brief Comparison operator for StampedTransform */
static inline bool operator==(const StampedTransform &a, const StampedTransform &b) {
  return a.frame_id_ == b.frame_id_ && a.child_frame_id_ == b.child_frame_id_ && a.stamp_ == b.stamp_ && static_cast<const tf::Transform&>(a) == static_cast<const tf::Transform&>(b);
};

/** \brief construct a Quaternion from Fixed angles
 * \param roll The roll about the X axis
 * \param pitch The pitch about the Y axis
 * \param yaw The yaw about the Z axis
 * \return The quaternion constructed
 */
static inline tf::Quaternion createQuaternionFromRPY(double roll,double pitch,double yaw){
  Quaternion q;
  q.setRPY(roll, pitch, yaw);
  return q;
}

static inline Quaternion createQuaternionFromYaw(double yaw){
   Quaternion q;
   q.setRPY(0.0, 0.0, yaw);
   return q;
 }

/** \brief construct an Identity Quaternion
 * \return The quaternion constructed
 */
static inline tf::Quaternion createIdentityQuaternion()
{
  Quaternion q;
  q.setRPY(0,0,0);
  return q;
};

}
#endif //TF_TRANSFORM_DATATYPES_H
