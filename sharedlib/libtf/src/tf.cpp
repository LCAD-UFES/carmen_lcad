#include "tf.h"
#include "angles.h"
#include <sys/time.h>

using namespace tf;

// Must provide storage for non-integral static const class members.
// Otherwise you get undefined symbol errors on OS X (why not on Linux?).
// Thanks to Rob for pointing out the right way to do this.
constexpr double tf::Transformer::DEFAULT_CACHE_TIME;


enum WalkEnding
{
  Identity,
  TargetParentOfSource,
  SourceParentOfTarget,
  FullPath,
};

struct CanTransformAccum
{
  CompactFrameID gather(TimeCache* cache, Time time, std::string* error_string)
  {
    return cache->getParent(time, error_string);
  }

  void accum(bool source __attribute__((unused)))
  {
  }

  void finalize(WalkEnding end __attribute__((unused)), Time _time __attribute__((unused)))
  {
  }

  TransformStorage st;
};

struct TransformAccum
{
  TransformAccum()
  : source_to_top_quat(0.0, 0.0, 0.0, 1.0)
  , source_to_top_vec(0.0, 0.0, 0.0)
  , target_to_top_quat(0.0, 0.0, 0.0, 1.0)
  , target_to_top_vec(0.0, 0.0, 0.0)
  , result_quat(0.0, 0.0, 0.0, 1.0)
  , result_vec(0.0, 0.0, 0.0)
  {
  }

  CompactFrameID gather(TimeCache* cache, Time time, std::string* error_string)
  {
    if (!cache->getData(time, st, error_string))
    {
      return 0;
    }

    return st.frame_id_;
  }

  void accum(bool source)
  {
    if (source)
    {
      source_to_top_vec = quatRotate(st.rotation_, source_to_top_vec) + st.translation_;
      source_to_top_quat = st.rotation_ * source_to_top_quat;
    }
    else
    {
      target_to_top_vec = quatRotate(st.rotation_, target_to_top_vec) + st.translation_;
      target_to_top_quat = st.rotation_ * target_to_top_quat;
    }
  }

  void finalize(WalkEnding end, Time _time)
  {
    switch (end)
    {
    case Identity:
      break;
    case TargetParentOfSource:
      result_vec = source_to_top_vec;
      result_quat = source_to_top_quat;
      break;
    case SourceParentOfTarget:
      {
        tf::Quaternion inv_target_quat = target_to_top_quat.inverse();
        tf::Vector3 inv_target_vec = quatRotate(inv_target_quat, -target_to_top_vec);
        result_vec = inv_target_vec;
        result_quat = inv_target_quat;
        break;
      }
    case FullPath:
      {
        tf::Quaternion inv_target_quat = target_to_top_quat.inverse();
        tf::Vector3 inv_target_vec = quatRotate(inv_target_quat, -target_to_top_vec);

     	result_vec = quatRotate(inv_target_quat, source_to_top_vec) + inv_target_vec;
        result_quat = inv_target_quat * source_to_top_quat;
      }
      break;
    };

    time = _time;
  }

  TransformStorage st;
  Time time;
  tf::Quaternion source_to_top_quat;
  tf::Vector3 source_to_top_vec;
  tf::Quaternion target_to_top_quat;
  tf::Vector3 target_to_top_vec;

  tf::Quaternion result_quat;
  tf::Vector3 result_vec;
};


std::string assert_resolved(const std::string& prefix, const std::string& frame_id)
{
  if (frame_id.size() > 0)
    if (frame_id[0] != '/')
      printf("TF operating on not fully resolved frame id %s, resolving using local prefix %s\n", frame_id.c_str(), prefix.c_str());
  return tf::resolve(prefix, frame_id);
};

std::string tf::resolve(const std::string& prefix, const std::string& frame_name)
{
  //  printf ("resolveping prefix:%s with frame_name:%s\n", prefix.c_str(), frame_name.c_str());
  if (frame_name.size() > 0)
    if (frame_name[0] == '/')
    {
      return frame_name;
    }
  if (prefix.size() > 0)
  {
    if (prefix[0] == '/')
    {
      std::string composite = prefix;
      composite.append("/");
      composite.append(frame_name);
      return composite;
    }
    else
    {
      std::string composite;
      composite = "/";
      composite.append(prefix);
      composite.append("/");
      composite.append(frame_name);
      return composite;
    }

  }
  else
 {
    std::string composite;
    composite = "/";
    composite.append(frame_name);
    return composite;
  }
};



Transformer::Transformer(bool interpolating,
                                Duration cache_time):
  cache_time(cache_time),
  interpolating (interpolating), 
  using_dedicated_thread_(false),
  fall_back_to_wall_time_(false)
{
  max_extrapolation_distance_.fromNSec(DEFAULT_MAX_EXTRAPOLATION_DISTANCE);
  frameIDs_["NO_PARENT"] = 0;
  frames_.push_back(NULL);// new TimeCache(interpolating, cache_time, max_extrapolation_distance));//unused but needed for iteration over all elements
  frameIDs_reverse.push_back("NO_PARENT");

  return;
}

Transformer::~Transformer()
{
  /* deallocate all frames */
  boost::recursive_mutex::scoped_lock lock(frame_mutex_);
  for (std::vector<TimeCache*>::iterator  cache_it = frames_.begin(); cache_it != frames_.end(); ++cache_it)
  {
    delete (*cache_it);
  }

};


void Transformer::clear()
{
  boost::recursive_mutex::scoped_lock lock(frame_mutex_);
  if ( frames_.size() > 1 )
  {
    for (std::vector< TimeCache*>::iterator  cache_it = frames_.begin() + 1; cache_it != frames_.end(); ++cache_it)
    {
      (*cache_it)->clearList();
    }
  }
}


template<typename F>
int Transformer::walkToTopParent(F& f, Time time, CompactFrameID target_id, CompactFrameID source_id, std::string* error_string) const
{
  // Short circuit if zero length transform to allow lookups on non existant links
  if (source_id == target_id)
  {
    f.finalize(Identity, time);
    return NO_ERROR;
  }

  //If getting the latest get the latest common time
  if (time == Time())
  {
    int retval = getLatestCommonTime(target_id, source_id, time, error_string);
    if (retval != NO_ERROR)
    {
      return retval;
    }
  }

  // Walk the tree to its root from the source frame, accumulating the transform
  CompactFrameID frame = source_id;
  CompactFrameID top_parent = frame;
  uint32_t depth = 0;
  while (frame != 0)
  {
    TimeCache* cache = getFrame(frame);

    if (!cache)
    {
      // There will be no cache for the very root of the tree
      top_parent = frame;
      break;
    }

    CompactFrameID parent = f.gather(cache, time, 0);
    if (parent == 0)
    {
      // Just break out here... there may still be a path from source -> target
      top_parent = frame;
      break;
    }

    // Early out... target frame is a direct parent of the source frame
    if (frame == target_id)
    {
      f.finalize(TargetParentOfSource, time);
      return NO_ERROR;
    }

    f.accum(true);

    top_parent = frame;
    frame = parent;

    ++depth;
    if (depth > MAX_GRAPH_DEPTH)
    {
      if (error_string)
      {
        std::stringstream ss;
        ss << "The tf tree is invalid because it contains a loop." << std::endl
           << allFramesAsString() << std::endl;
        *error_string = ss.str();
      }
      return LOOKUP_ERROR;
    }
  }

  // Now walk to the top parent from the target frame, accumulating its transform
  frame = target_id;
  depth = 0;
  while (frame != top_parent)
  {
    TimeCache* cache = getFrame(frame);

    if (!cache)
    {
      break;
    }

    CompactFrameID parent = f.gather(cache, time, error_string);
    if (parent == 0)
    {
      if (error_string)
      {
        std::stringstream ss;
        ss << *error_string << ", when looking up transform from frame [" << lookupFrameString(source_id) << "] to frame [" << lookupFrameString(target_id) << "]";
        *error_string = ss.str();
      }

      return EXTRAPOLATION_ERROR;
    }

    // Early out... source frame is a direct parent of the target frame
    if (frame == source_id)
    {
      f.finalize(SourceParentOfTarget, time);
      return NO_ERROR;
    }

    f.accum(false);

    frame = parent;

    ++depth;
    if (depth > MAX_GRAPH_DEPTH)
    {
      if (error_string)
      {
        std::stringstream ss;
        ss << "The tf tree is invalid because it contains a loop." << std::endl
           << allFramesAsString() << std::endl;
        *error_string = ss.str();
      }
      return LOOKUP_ERROR;
    }
  }

  if (frame != top_parent)
  {
    createConnectivityErrorString(source_id, target_id, error_string);
    return CONNECTIVITY_ERROR;
  }

  f.finalize(FullPath, time);

  return NO_ERROR;
}



bool Transformer::setTransform(const StampedTransform& transform, const std::string& authority)
{
  StampedTransform mapped_transform((tf::Transform)transform, transform.stamp_, transform.frame_id_, transform.child_frame_id_);
  mapped_transform.child_frame_id_ = assert_resolved(tf_prefix_, transform.child_frame_id_);
  mapped_transform.frame_id_ = assert_resolved(tf_prefix_, transform.frame_id_);

  bool error_exists = false;
  if (mapped_transform.child_frame_id_ == mapped_transform.frame_id_)
  {
    fprintf(stderr,"TF_SELF_TRANSFORM: Ignoring transform from authority \"%s\" with frame_id and child_frame_id  \"%s\" because they are the same\n",  authority.c_str(), mapped_transform.child_frame_id_.c_str());
    error_exists = true;
  }

  if (mapped_transform.child_frame_id_ == "/")//empty frame id will be mapped to "/"
  {
	  fprintf(stderr,"TF_NO_CHILD_FRAME_ID: Ignoring transform from authority \"%s\" because child_frame_id not set\n", authority.c_str());
    error_exists = true;
  }

  if (mapped_transform.frame_id_ == "/")//empty parent id will be mapped to "/"
  {
	  fprintf(stderr,"TF_NO_FRAME_ID: Ignoring transform with child_frame_id \"%s\"  from authority \"%s\" because frame_id not set\n", mapped_transform.child_frame_id_.c_str(), authority.c_str());
    error_exists = true;
  }

  if (std::isnan(mapped_transform.getOrigin().x()) || std::isnan(mapped_transform.getOrigin().y()) || std::isnan(mapped_transform.getOrigin().z())||
      std::isnan(mapped_transform.getRotation().x()) ||       std::isnan(mapped_transform.getRotation().y()) ||       std::isnan(mapped_transform.getRotation().z()) ||       std::isnan(mapped_transform.getRotation().w()))
  {
	  fprintf(stderr,"TF_NAN_INPUT: Ignoring transform for child_frame_id \"%s\" from authority \"%s\" because of a nan value in the transform (%f %f %f) (%f %f %f %f)",
              mapped_transform.child_frame_id_.c_str(), authority.c_str(),
              mapped_transform.getOrigin().x(), mapped_transform.getOrigin().y(), mapped_transform.getOrigin().z(),
              mapped_transform.getRotation().x(), mapped_transform.getRotation().y(), mapped_transform.getRotation().z(), mapped_transform.getRotation().w()
              );
    error_exists = true;
  }

  if (error_exists)
    return false;

  {
    boost::recursive_mutex::scoped_lock lock(frame_mutex_);
    CompactFrameID frame_number = lookupOrInsertFrameNumber(mapped_transform.child_frame_id_);
    TimeCache* frame = getFrame(frame_number);
    if (frame == NULL)
    {
    	frames_[frame_number] = new TimeCache(cache_time);
    	frame = frames_[frame_number];
    }

    if (frame->insertData(TransformStorage(mapped_transform, lookupOrInsertFrameNumber(mapped_transform.frame_id_), frame_number)))
    {
      frame_authority_[frame_number] = authority;
    }
    else
    {
      fprintf(stderr,"TF_OLD_DATA ignoring data from the past for frame %s at time %g according to authority %s\nPossible reasons are listed at\n", mapped_transform.child_frame_id_.c_str(), mapped_transform.stamp_.toSec(), authority.c_str());
      return false;
    }
  }

  {
    boost::mutex::scoped_lock lock(transforms_changed_mutex_);
    transforms_changed_();
  }

  return true;
};


void Transformer::lookupTransform(const std::string& target_frame, const std::string& source_frame,
                     const Time& time, StampedTransform& transform) const
{
	  std::string mapped_tgt = assert_resolved(tf_prefix_, target_frame);
	  std::string mapped_src = assert_resolved(tf_prefix_, source_frame);

	  if (mapped_tgt == mapped_src) {
		  transform.setIdentity();
		  transform.child_frame_id_ = mapped_src;
		  transform.frame_id_       = mapped_tgt;
		  transform.stamp_          = now();
		  return;
	  }

	  boost::recursive_mutex::scoped_lock lock(frame_mutex_);

	  CompactFrameID target_id = lookupFrameNumber(mapped_tgt);
	  CompactFrameID source_id = lookupFrameNumber(mapped_src);

	  std::string error_string;
	  TransformAccum accum;
	  int retval = walkToTopParent(accum, time, target_id, source_id, &error_string);
	  if (retval != NO_ERROR)
	  {
	    switch (retval)
	    {
	    case CONNECTIVITY_ERROR:
	      throw ConnectivityException(error_string);
	    case EXTRAPOLATION_ERROR:
	      throw ExtrapolationException(error_string);
	    case LOOKUP_ERROR:
	      throw LookupException(error_string);
	    default:
	      fprintf(stderr,"Unknown error code: %d\n", retval);
	      break;
	    }
	  }

	  transform.setOrigin(accum.result_vec);
	  transform.setRotation(accum.result_quat);
	  transform.child_frame_id_ = mapped_src;
	  transform.frame_id_       = mapped_tgt;
	  transform.stamp_          = accum.time;
};


void Transformer::lookupTransform(const std::string& target_frame,const Time& target_time, const std::string& source_frame,
                     const Time& source_time, const std::string& fixed_frame, StampedTransform& transform) const
{
  tf::StampedTransform temp1, temp2;
  lookupTransform(fixed_frame, source_frame, source_time, temp1);
  lookupTransform(target_frame, fixed_frame, target_time, temp2);
  transform.setData( temp2 * temp1);
  transform.stamp_ = temp2.stamp_;
  transform.frame_id_ = target_frame;
  transform.child_frame_id_ = source_frame;

};

bool Transformer::waitForTransform(const std::string& target_frame, const std::string& source_frame,
                                   const Time& time,
                                   const Duration& timeout, const Duration& polling_sleep_duration,
                                   std::string* error_msg) const
{
  if (!using_dedicated_thread_)
  {
    std::string error_string = "Do not call waitForTransform unless you are using another thread for populating data. Without a dedicated thread it will always timeout.  If you have a seperate thread servicing tf messages, call setUsingDedicatedThread(true)";
    fprintf(stderr, "%s\n", error_string.c_str());
    
    if (error_msg) 
      *error_msg = error_string;
    return false;
  }
  Time start_time = now();
  std::string mapped_tgt = assert_resolved(tf_prefix_, target_frame);
  std::string mapped_src = assert_resolved(tf_prefix_, source_frame);

  while (ok() && (now() - start_time) < timeout)
  {
	  if (frameExists(mapped_tgt) && frameExists(mapped_src) && (canTransform(mapped_tgt, mapped_src, time, error_msg)))
		  return true;

	  usleep(polling_sleep_duration.sec * 1000000 + polling_sleep_duration.nsec / 1000); //hack to avoid calling Time::now() in Duration.sleep
  }
  return false;
}

bool Transformer::canTransformNoLock(CompactFrameID target_id, CompactFrameID source_id,
                    const Time& time, std::string* error_msg) const
{
  if (target_id == 0 || source_id == 0)
  {
    return false;
  }

  CanTransformAccum accum;
  if (walkToTopParent(accum, time, target_id, source_id, error_msg) == NO_ERROR)
  {
    return true;
  }

  return false;
}

bool Transformer::canTransformInternal(CompactFrameID target_id, CompactFrameID source_id,
                                  const Time& time, std::string* error_msg) const
{
  boost::recursive_mutex::scoped_lock lock(frame_mutex_);
  return canTransformNoLock(target_id, source_id, time, error_msg);
}

bool Transformer::canTransform(const std::string& target_frame, const std::string& source_frame,
                           const Time& time, std::string* error_msg) const
{
	std::string mapped_tgt = assert_resolved(tf_prefix_, target_frame);
	std::string mapped_src = assert_resolved(tf_prefix_, source_frame);

	if (mapped_tgt == mapped_src)
		return true;

	boost::recursive_mutex::scoped_lock lock(frame_mutex_);

  if (!frameExists(mapped_tgt) || !frameExists(mapped_src))
	  return false;

  CompactFrameID target_id = lookupFrameNumber(mapped_tgt);
  CompactFrameID source_id = lookupFrameNumber(mapped_src);

  return canTransformNoLock(target_id, source_id, time, error_msg);
}



bool Transformer::canTransform(const std::string& target_frame,const Time& target_time, const std::string& source_frame,
                               const Time& source_time, const std::string& fixed_frame,
                               std::string* error_msg) const
{
  return canTransform(target_frame, fixed_frame, target_time) && canTransform(fixed_frame, source_frame, source_time, error_msg);
};

bool Transformer::waitForTransform(const std::string& target_frame,const Time& target_time, const std::string& source_frame,
                                   const Time& source_time, const std::string& fixed_frame,
                                   const Duration& timeout, const Duration& polling_sleep_duration,
                                   std::string* error_msg) const
{
  return waitForTransform(target_frame, fixed_frame, target_time, timeout, polling_sleep_duration, error_msg) && waitForTransform(fixed_frame, source_frame, source_time, timeout, polling_sleep_duration, error_msg);
};


bool Transformer::getParent(const std::string& frame_id, Time time, std::string& parent) const
{
  std::string mapped_frame_id = assert_resolved(tf_prefix_, frame_id);
  tf::TimeCache* cache;
  try
  {
    cache = getFrame(lookupFrameNumber(mapped_frame_id));
  }
  catch  (tf::LookupException &ex)
  {
	fprintf(stderr,"Transformer::getParent: %s\n",ex.what());
    return false;
  }

  TransformStorage temp;
  if (! cache->getData(time, temp)) {
    printf("Transformer::getParent: No data for parent of %s\n", mapped_frame_id.c_str());
    return false;
  }
  if (temp.frame_id_ == 0) {
    printf("Transformer::getParent: No parent for %s\n", mapped_frame_id.c_str());
    return false;
  }

  parent = lookupFrameString(temp.frame_id_);
  return true;
};


bool Transformer::frameExists(const std::string& frame_id_str) const
{
  boost::recursive_mutex::scoped_lock lock(frame_mutex_);
  std::string frame_id_resolveped = assert_resolved(tf_prefix_, frame_id_str);
  
  return frameIDs_.count(frame_id_resolveped);
}

void Transformer::setExtrapolationLimit(const Duration& distance)
{
  max_extrapolation_distance_ = distance;
}

void Transformer::createConnectivityErrorString(CompactFrameID source_frame, CompactFrameID target_frame, std::string* out) const
{
  if (!out)
  {
    return;
  }
  *out = std::string("Could not find a connection between '"+lookupFrameString(target_frame)+"' and '"+
                     lookupFrameString(source_frame)+"' because they are not part of the same tree."+
                     "Tf has two or more unconnected trees.");
}

struct TimeAndFrameIDFrameComparator
{
  TimeAndFrameIDFrameComparator(CompactFrameID id)
  : id(id)
  {}

  bool operator()(const P_TimeAndFrameID& rhs) const
  {
    return rhs.second == id;
  }

  CompactFrameID id;
};

int Transformer::getLatestCommonTime(const std::string &source_frame, const std::string &target_frame, Time& time, std::string* error_string) const
{
	  std::string mapped_tgt = assert_resolved(tf_prefix_, target_frame);
	  std::string mapped_src = assert_resolved(tf_prefix_, source_frame);

	  if (!frameExists(mapped_tgt) || !frameExists(mapped_src)) {
		  time = Time();
		  return LOOKUP_ERROR;
	  }

	  CompactFrameID source_id = lookupFrameNumber(mapped_src);
	  CompactFrameID target_id = lookupFrameNumber(mapped_tgt);
	  return getLatestCommonTime(source_id, target_id, time, error_string);
}


int Transformer::getLatestCommonTime(CompactFrameID target_id, CompactFrameID source_id, Time & time, std::string * error_string) const
{
  if (source_id == target_id)
  {
    //Set time to latest timestamp of frameid in case of target and source frame id are the same
    time = now();
    return NO_ERROR;
  }

  std::vector<P_TimeAndFrameID> lct_cache;

  // Walk the tree to its root from the source frame, accumulating the list of parent/time as well as the latest time
  // in the target is a direct parent
  CompactFrameID frame = source_id;
  P_TimeAndFrameID temp;
  uint32_t depth = 0;
  Time common_time = TIME_MAX;
  while (frame != 0)
  {
    TimeCache* cache = getFrame(frame);

    if (!cache)
    {
      // There will be no cache for the very root of the tree
      break;
    }

    P_TimeAndFrameID latest = cache->getLatestTimeAndParent();

    if (latest.second == 0)
    {
      // Just break out here... there may still be a path from source -> target
      break;
    }

    if (!latest.first.isZero())
    {
      common_time = std::min(latest.first, common_time);
    }

    lct_cache.push_back(latest);

    frame = latest.second;

    // Early out... target frame is a direct parent of the source frame
    if (frame == target_id)
    {
      time = common_time;
      if (time == TIME_MAX)
      {
        time = Time();
      }
      return NO_ERROR;
    }

    ++depth;
    if (depth > MAX_GRAPH_DEPTH)
    {
      if (error_string)
      {
        std::stringstream ss;
        ss<<"The tf tree is invalid because it contains a loop." << std::endl
          << allFramesAsString() << std::endl;
        *error_string = ss.str();
      }
      return LOOKUP_ERROR;
    }
  }

  // Now walk to the top parent from the target frame, accumulating the latest time and looking for a common parent
  frame = target_id;
  depth = 0;
  common_time = TIME_MAX;
  CompactFrameID common_parent = 0;
  while (true)
  {
    TimeCache* cache = getFrame(frame);

    if (!cache)
    {
      break;
    }

    P_TimeAndFrameID latest = cache->getLatestTimeAndParent();

    if (latest.second == 0)
    {
      break;
    }

    if (!latest.first.isZero())
    {
      common_time = std::min(latest.first, common_time);
    }

    std::vector<P_TimeAndFrameID>::iterator it = std::find_if(lct_cache.begin(), lct_cache.end(), TimeAndFrameIDFrameComparator(latest.second));
    if (it != lct_cache.end()) // found a common parent
    {
      common_parent = it->second;
      break;
    }

    frame = latest.second;

    // Early out... source frame is a direct parent of the target frame
    if (frame == source_id)
    {
      time = common_time;
      if (time == TIME_MAX)
      {
        time = Time();
      }
      return NO_ERROR;
    }

    ++depth;
    if (depth > MAX_GRAPH_DEPTH)
    {
      if (error_string)
      {
        std::stringstream ss;
        ss<<"The tf tree is invalid because it contains a loop." << std::endl
          << allFramesAsString() << std::endl;
        *error_string = ss.str();
      }
      return LOOKUP_ERROR;
    }
  }

  if (common_parent == 0)
  {
    createConnectivityErrorString(source_id, target_id, error_string);
    return CONNECTIVITY_ERROR;
  }

  // Loop through the source -> root list until we hit the common parent
  {
    std::vector<P_TimeAndFrameID>::iterator it = lct_cache.begin();
    std::vector<P_TimeAndFrameID>::iterator end = lct_cache.end();
    for (; it != end; ++it)
    {
      if (!it->first.isZero())
      {
        common_time = std::min(common_time, it->first);
      }

      if (it->second == common_parent)
      {
        break;
      }
    }
  }

  if (common_time == TIME_MAX)
  {
    common_time = Time();
  }

  time = common_time;
  return NO_ERROR;
}

//@todo - Fix this to work with new data structures
void Transformer::chainAsVector(const std::string & target_frame __attribute__((unused)), Time target_time __attribute__((unused)), const std::string & source_frame __attribute__((unused)), Time source_time __attribute__((unused)), const std::string& fixed_frame __attribute__((unused)), std::vector<std::string>& output) const
{
  std::string error_string;

  output.clear(); //empty vector

  std::stringstream mstream;
  boost::recursive_mutex::scoped_lock lock(frame_mutex_);

  TransformStorage temp;

  ///regular transforms
  for (unsigned int counter = 1; counter < frames_.size(); counter ++)
  {
    TimeCache* frame_ptr = getFrame(CompactFrameID(counter));
    if (frame_ptr == NULL)
      continue;
    CompactFrameID frame_id_num;
    if (frame_ptr->getData(Time(), temp))
        frame_id_num = temp.frame_id_;
      else
      {
        frame_id_num = 0;
      }
      output.push_back(frameIDs_reverse[frame_id_num]);
  }
}

std::string Transformer::allFramesAsString() const
{
  std::stringstream mstream;
  boost::recursive_mutex::scoped_lock lock(frame_mutex_);

  TransformStorage temp;

  ///regular transforms
  for (unsigned int counter = 1; counter < frames_.size(); counter ++)
  {
    TimeCache* frame_ptr = getFrame(CompactFrameID(counter));
    if (frame_ptr == NULL)
      continue;
    CompactFrameID frame_id_num;
    if(  frame_ptr->getData(Time(), temp))
      frame_id_num = temp.frame_id_;
    else
    {
      frame_id_num = 0;
    }
    mstream << "Frame "<< frameIDs_reverse[counter] << " exists with parent " << frameIDs_reverse[frame_id_num] << "." <<std::endl;
  }

  return mstream.str();
}

std::string Transformer::allFramesAsDot() const
{
  std::stringstream mstream;
  mstream << "digraph G {" << std::endl;
  boost::recursive_mutex::scoped_lock lock(frame_mutex_);

  TransformStorage temp;

  Time current_time = now();

  if (frames_.size() ==1)
    mstream <<"\"no tf data recieved\"";

  mstream.precision(3);
  mstream.setf(std::ios::fixed,std::ios::floatfield);
    
   //  for (std::vector< TimeCache*>::iterator  it = frames_.begin(); it != frames_.end(); ++it)
  for (unsigned int counter = 1; counter < frames_.size(); counter ++)//one referenced for 0 is no frame
  {
    unsigned int frame_id_num;
    if(  getFrame(counter)->getData(Time(), temp))
      frame_id_num = temp.frame_id_;
    else
    {
      frame_id_num = 0;
    }
    if (frame_id_num != 0)
    {
      std::string authority = "no recorded authority";
      std::map<unsigned int, std::string>::const_iterator it = frame_authority_.find(counter);
      if (it != frame_authority_.end())
        authority = it->second;

      double rate = getFrame(counter)->getListLength() / std::max((getFrame(counter)->getLatestTimestamp().toSec() -
                                                                   getFrame(counter)->getOldestTimestamp().toSec() ), 0.0001);

      mstream << std::fixed; //fixed point notation
      mstream.precision(3); //3 decimal places
      mstream << "\"" << frameIDs_reverse[frame_id_num] << "\"" << " -> "
              << "\"" << frameIDs_reverse[counter] << "\"" << "[label=\""
        //<< "Time: " << current_time.toSec() << "\\n"
              << "Broadcaster: " << authority << "\\n"
              << "Average rate: " << rate << " Hz\\n"
              << "Most recent transform: " << (current_time - getFrame(counter)->getLatestTimestamp()).toSec() << " sec old \\n"
        //    << "(time: " << getFrame(counter)->getLatestTimestamp().toSec() << ")\\n"
        //    << "Oldest transform: " << (current_time - getFrame(counter)->getOldestTimestamp()).toSec() << " sec old \\n"
        //    << "(time: " << (getFrame(counter)->getOldestTimestamp()).toSec() << ")\\n"
              << "Buffer length: " << (getFrame(counter)->getLatestTimestamp()-getFrame(counter)->getOldestTimestamp()).toSec() << " sec\\n"
              <<"\"];" <<std::endl;
    }
  }
  
  for (unsigned int counter = 1; counter < frames_.size(); counter ++)//one referenced for 0 is no frame
  {
    unsigned int frame_id_num;
    if(  getFrame(counter)->getData(Time(), temp))
      frame_id_num = temp.frame_id_;
    else
      {
	frame_id_num = 0;
      }

    if(frameIDs_reverse[frame_id_num]=="NO_PARENT")
    {
      mstream << "edge [style=invis];" <<std::endl;
      mstream << " subgraph cluster_legend { style=bold; color=black; label =\"view_frames Result\";\n"
              << "\"Recorded at time: " << current_time.toSec() << "\"[ shape=plaintext ] ;\n "
	      << "}" << "->" << "\"" << frameIDs_reverse[counter]<<"\";" <<std::endl;
    }
  }
  mstream << "}";
  return mstream.str();
}


bool Transformer::ok() const { return true; }

void Transformer::getFrameStrings(std::vector<std::string> & vec) const
{
  vec.clear();

  boost::recursive_mutex::scoped_lock lock(frame_mutex_);

  TransformStorage temp;

  //  for (std::vector< TimeCache*>::iterator  it = frames_.begin(); it != frames_.end(); ++it)
  for (unsigned int counter = 1; counter < frames_.size(); counter ++)
  {
    vec.push_back(frameIDs_reverse[counter]);
  }
  return;
}

tf::TimeCache* Transformer::getFrame(unsigned int frame_id) const
{
  if (frame_id == 0) /// @todo check larger values too
    return NULL;
  else
    return frames_[frame_id];
};


void Transformer::transformQuaternion(const std::string& target_frame, const Stamped<Quaternion>& stamped_in, Stamped<Quaternion>& stamped_out) const
{
  tf::assertQuaternionValid(stamped_in);

  StampedTransform transform;
  lookupTransform(target_frame, stamped_in.frame_id_, stamped_in.stamp_, transform);

  stamped_out.setData( transform * stamped_in);
  stamped_out.stamp_ = transform.stamp_;
  stamped_out.frame_id_ = target_frame;
};


void Transformer::transformVector(const std::string& target_frame,
                                  const Stamped<tf::Vector3>& stamped_in,
                                  Stamped<tf::Vector3>& stamped_out) const
{
  StampedTransform transform;
  lookupTransform(target_frame, stamped_in.frame_id_, stamped_in.stamp_, transform);

  /** \todo may not be most efficient */
  tf::Vector3 end = stamped_in;
  tf::Vector3 origin = tf::Vector3(0,0,0);
  tf::Vector3 output = (transform * end) - (transform * origin);
  stamped_out.setData( output);

  stamped_out.stamp_ = transform.stamp_;
  stamped_out.frame_id_ = target_frame;
};


void Transformer::transformPoint(const std::string& target_frame, const Stamped<Point>& stamped_in, Stamped<Point>& stamped_out) const
{
  StampedTransform transform;
  lookupTransform(target_frame, stamped_in.frame_id_, stamped_in.stamp_, transform);

  stamped_out.setData(transform * stamped_in);
  stamped_out.stamp_ = transform.stamp_;
  stamped_out.frame_id_ = target_frame;
};

void Transformer::transformPose(const std::string& target_frame, const Stamped<Pose>& stamped_in, Stamped<Pose>& stamped_out) const
{
  StampedTransform transform;
  lookupTransform(target_frame, stamped_in.frame_id_, stamped_in.stamp_, transform);

  stamped_out.setData(transform * stamped_in);
  stamped_out.stamp_ = transform.stamp_;
  stamped_out.frame_id_ = target_frame;
};


void Transformer::transformQuaternion(const std::string& target_frame, const Time& target_time,
                                      const Stamped<Quaternion>& stamped_in,
                                      const std::string& fixed_frame,
                                      Stamped<Quaternion>& stamped_out) const
{
  tf::assertQuaternionValid(stamped_in);
  StampedTransform transform;
  lookupTransform(target_frame, target_time,
                  stamped_in.frame_id_,stamped_in.stamp_,
                  fixed_frame, transform);

  stamped_out.setData( transform * stamped_in);
  stamped_out.stamp_ = transform.stamp_;
  stamped_out.frame_id_ = target_frame;
};


void Transformer::transformVector(const std::string& target_frame, const Time& target_time,
                                  const Stamped<Vector3>& stamped_in,
                                  const std::string& fixed_frame,
                                  Stamped<Vector3>& stamped_out) const
{
  StampedTransform transform;
  lookupTransform(target_frame, target_time,
                  stamped_in.frame_id_,stamped_in.stamp_,
                  fixed_frame, transform);

  /** \todo may not be most efficient */
  tf::Vector3 end = stamped_in;
  tf::Vector3 origin = tf::Vector3(0,0,0);
  tf::Vector3 output = (transform * end) - (transform * origin);
  stamped_out.setData( output);

  stamped_out.stamp_ = transform.stamp_;
  stamped_out.frame_id_ = target_frame;
};


void Transformer::transformPoint(const std::string& target_frame, const Time& target_time,
                                 const Stamped<Point>& stamped_in,
                                 const std::string& fixed_frame,
                                 Stamped<Point>& stamped_out) const
{
  StampedTransform transform;
  lookupTransform(target_frame, target_time,
                  stamped_in.frame_id_,stamped_in.stamp_,
                  fixed_frame, transform);

  stamped_out.setData(transform * stamped_in);
  stamped_out.stamp_ = transform.stamp_;
  stamped_out.frame_id_ = target_frame;
};

void Transformer::transformPose(const std::string& target_frame, const Time& target_time,
                                const Stamped<Pose>& stamped_in,
                                const std::string& fixed_frame,
                                Stamped<Pose>& stamped_out) const
{
  StampedTransform transform;
  lookupTransform(target_frame, target_time,
                  stamped_in.frame_id_,stamped_in.stamp_,
                  fixed_frame, transform);

  stamped_out.setData(transform * stamped_in);
  stamped_out.stamp_ = transform.stamp_;
  stamped_out.frame_id_ = target_frame;
};

boost::signals::connection Transformer::addTransformsChangedListener(boost::function<void(void)> callback)
{
  boost::mutex::scoped_lock lock(transforms_changed_mutex_);
  return transforms_changed_.connect(callback);
}

void Transformer::removeTransformsChangedListener(boost::signals::connection c)
{
  boost::mutex::scoped_lock lock(transforms_changed_mutex_);
  c.disconnect();
}
