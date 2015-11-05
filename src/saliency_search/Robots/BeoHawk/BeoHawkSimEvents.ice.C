// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1
// Generated from file `BeoHawkSimEvents.ice'

#include <BeoHawkSimEvents.ice.H>
#include <Ice/LocalException.h>
#include <Ice/ObjectFactory.h>
#include <Ice/BasicStream.h>
#include <IceUtil/Iterator.h>
#include <IceUtil/ScopedArray.h>

#ifndef ICE_IGNORE_VERSION
#   if ICE_INT_VERSION / 100 != 303
#       error Ice version mismatch!
#   endif
#   if ICE_INT_VERSION % 100 > 50
#       error Beta header file detected
#   endif
#   if ICE_INT_VERSION % 100 < 1
#       error Ice patch level mismatch!
#   endif
#endif

::Ice::Object* IceInternal::upCast(::RobotSimEvents::BeoHawkEyeSpyMessage* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::RobotSimEvents::BeoHawkEyeSpyMessage* p) { return p; }

::Ice::Object* IceInternal::upCast(::RobotSimEvents::ControlCameraMessage* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::RobotSimEvents::ControlCameraMessage* p) { return p; }

::Ice::Object* IceInternal::upCast(::RobotSimEvents::CameraImageMessage* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::RobotSimEvents::CameraImageMessage* p) { return p; }

::Ice::Object* IceInternal::upCast(::RobotSimEvents::ControlDriveVisionMessage* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::RobotSimEvents::ControlDriveVisionMessage* p) { return p; }

::Ice::Object* IceInternal::upCast(::RobotSimEvents::ExecuteMissionMessage* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::RobotSimEvents::ExecuteMissionMessage* p) { return p; }

::Ice::Object* IceInternal::upCast(::RobotSimEvents::SlamDataMessage* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::RobotSimEvents::SlamDataMessage* p) { return p; }

::Ice::Object* IceInternal::upCast(::RobotSimEvents::ControlLandMessage* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::RobotSimEvents::ControlLandMessage* p) { return p; }

::Ice::Object* IceInternal::upCast(::RobotSimEvents::ControlMoveMessage* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::RobotSimEvents::ControlMoveMessage* p) { return p; }

::Ice::Object* IceInternal::upCast(::RobotSimEvents::SensorDataMessage* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::RobotSimEvents::SensorDataMessage* p) { return p; }

void
RobotSimEvents::__read(::IceInternal::BasicStream* __is, ::RobotSimEvents::BeoHawkEyeSpyMessagePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::RobotSimEvents::BeoHawkEyeSpyMessage;
        v->__copyFrom(proxy);
    }
}

void
RobotSimEvents::__read(::IceInternal::BasicStream* __is, ::RobotSimEvents::ControlCameraMessagePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::RobotSimEvents::ControlCameraMessage;
        v->__copyFrom(proxy);
    }
}

void
RobotSimEvents::__read(::IceInternal::BasicStream* __is, ::RobotSimEvents::CameraImageMessagePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::RobotSimEvents::CameraImageMessage;
        v->__copyFrom(proxy);
    }
}

void
RobotSimEvents::__read(::IceInternal::BasicStream* __is, ::RobotSimEvents::ControlDriveVisionMessagePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::RobotSimEvents::ControlDriveVisionMessage;
        v->__copyFrom(proxy);
    }
}

void
RobotSimEvents::__read(::IceInternal::BasicStream* __is, ::RobotSimEvents::ExecuteMissionMessagePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::RobotSimEvents::ExecuteMissionMessage;
        v->__copyFrom(proxy);
    }
}

void
RobotSimEvents::__read(::IceInternal::BasicStream* __is, ::RobotSimEvents::SlamDataMessagePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::RobotSimEvents::SlamDataMessage;
        v->__copyFrom(proxy);
    }
}

void
RobotSimEvents::__read(::IceInternal::BasicStream* __is, ::RobotSimEvents::ControlLandMessagePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::RobotSimEvents::ControlLandMessage;
        v->__copyFrom(proxy);
    }
}

void
RobotSimEvents::__read(::IceInternal::BasicStream* __is, ::RobotSimEvents::ControlMoveMessagePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::RobotSimEvents::ControlMoveMessage;
        v->__copyFrom(proxy);
    }
}

void
RobotSimEvents::__read(::IceInternal::BasicStream* __is, ::RobotSimEvents::SensorDataMessagePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::RobotSimEvents::SensorDataMessage;
        v->__copyFrom(proxy);
    }
}

bool
RobotSimEvents::Location::operator==(const Location& __rhs) const
{
    if(this == &__rhs)
    {
        return true;
    }
    if(x != __rhs.x)
    {
        return false;
    }
    if(y != __rhs.y)
    {
        return false;
    }
    if(z != __rhs.z)
    {
        return false;
    }
    if(theta != __rhs.theta)
    {
        return false;
    }
    return true;
}

bool
RobotSimEvents::Location::operator<(const Location& __rhs) const
{
    if(this == &__rhs)
    {
        return false;
    }
    if(x < __rhs.x)
    {
        return true;
    }
    else if(__rhs.x < x)
    {
        return false;
    }
    if(y < __rhs.y)
    {
        return true;
    }
    else if(__rhs.y < y)
    {
        return false;
    }
    if(z < __rhs.z)
    {
        return true;
    }
    else if(__rhs.z < z)
    {
        return false;
    }
    if(theta < __rhs.theta)
    {
        return true;
    }
    else if(__rhs.theta < theta)
    {
        return false;
    }
    return false;
}

void
RobotSimEvents::Location::__write(::IceInternal::BasicStream* __os) const
{
    __os->write(x);
    __os->write(y);
    __os->write(z);
    __os->write(theta);
}

void
RobotSimEvents::Location::__read(::IceInternal::BasicStream* __is)
{
    __is->read(x);
    __is->read(y);
    __is->read(z);
    __is->read(theta);
}

bool
RobotSimEvents::Lrf::operator==(const Lrf& __rhs) const
{
    if(this == &__rhs)
    {
        return true;
    }
    if(angle != __rhs.angle)
    {
        return false;
    }
    if(distance != __rhs.distance)
    {
        return false;
    }
    return true;
}

bool
RobotSimEvents::Lrf::operator<(const Lrf& __rhs) const
{
    if(this == &__rhs)
    {
        return false;
    }
    if(angle < __rhs.angle)
    {
        return true;
    }
    else if(__rhs.angle < angle)
    {
        return false;
    }
    if(distance < __rhs.distance)
    {
        return true;
    }
    else if(__rhs.distance < distance)
    {
        return false;
    }
    return false;
}

void
RobotSimEvents::Lrf::__write(::IceInternal::BasicStream* __os) const
{
    __os->write(angle);
    __os->write(distance);
}

void
RobotSimEvents::Lrf::__read(::IceInternal::BasicStream* __is)
{
    __is->read(angle);
    __is->read(distance);
}

bool
RobotSimEvents::Sonar::operator==(const Sonar& __rhs) const
{
    if(this == &__rhs)
    {
        return true;
    }
    if(sonarID != __rhs.sonarID)
    {
        return false;
    }
    if(distance != __rhs.distance)
    {
        return false;
    }
    return true;
}

bool
RobotSimEvents::Sonar::operator<(const Sonar& __rhs) const
{
    if(this == &__rhs)
    {
        return false;
    }
    if(sonarID < __rhs.sonarID)
    {
        return true;
    }
    else if(__rhs.sonarID < sonarID)
    {
        return false;
    }
    if(distance < __rhs.distance)
    {
        return true;
    }
    else if(__rhs.distance < distance)
    {
        return false;
    }
    return false;
}

void
RobotSimEvents::Sonar::__write(::IceInternal::BasicStream* __os) const
{
    __os->write(sonarID);
    __os->write(distance);
}

void
RobotSimEvents::Sonar::__read(::IceInternal::BasicStream* __is)
{
    __is->read(sonarID);
    __is->read(distance);
}

void
RobotSimEvents::__write(::IceInternal::BasicStream* __os, ::RobotSimEvents::BeoHawkEyeSpyType v)
{
    __os->write(static_cast< ::Ice::Byte>(v), 3);
}

void
RobotSimEvents::__read(::IceInternal::BasicStream* __is, ::RobotSimEvents::BeoHawkEyeSpyType& v)
{
    ::Ice::Byte val;
    __is->read(val, 3);
    v = static_cast< ::RobotSimEvents::BeoHawkEyeSpyType>(val);
}

void
RobotSimEvents::__writeLrfSeq(::IceInternal::BasicStream* __os, const ::RobotSimEvents::Lrf* begin, const ::RobotSimEvents::Lrf* end)
{
    ::Ice::Int size = static_cast< ::Ice::Int>(end - begin);
    __os->writeSize(size);
    for(int i = 0; i < size; ++i)
    {
        begin[i].__write(__os);
    }
}

void
RobotSimEvents::__readLrfSeq(::IceInternal::BasicStream* __is, ::RobotSimEvents::LrfSeq& v)
{
    ::Ice::Int sz;
    __is->readSize(sz);
    __is->checkFixedSeq(sz, 16);
    v.resize(sz);
    for(int i = 0; i < sz; ++i)
    {
        v[i].__read(__is);
    }
}

void
RobotSimEvents::__writeSonarSeq(::IceInternal::BasicStream* __os, const ::RobotSimEvents::Sonar* begin, const ::RobotSimEvents::Sonar* end)
{
    ::Ice::Int size = static_cast< ::Ice::Int>(end - begin);
    __os->writeSize(size);
    for(int i = 0; i < size; ++i)
    {
        begin[i].__write(__os);
    }
}

void
RobotSimEvents::__readSonarSeq(::IceInternal::BasicStream* __is, ::RobotSimEvents::SonarSeq& v)
{
    ::Ice::Int sz;
    __is->readSize(sz);
    __is->startSeq(sz, 9);
    v.resize(sz);
    for(int i = 0; i < sz; ++i)
    {
        v[i].__read(__is);
        __is->checkSeq();
        __is->endElement();
    }
    __is->endSeq(sz);
}

const ::std::string&
IceProxy::RobotSimEvents::BeoHawkEyeSpyMessage::ice_staticId()
{
    return ::RobotSimEvents::BeoHawkEyeSpyMessage::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::RobotSimEvents::BeoHawkEyeSpyMessage::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::RobotSimEvents::BeoHawkEyeSpyMessage);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::RobotSimEvents::BeoHawkEyeSpyMessage::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::RobotSimEvents::BeoHawkEyeSpyMessage);
}

::IceProxy::Ice::Object*
IceProxy::RobotSimEvents::BeoHawkEyeSpyMessage::__newInstance() const
{
    return new BeoHawkEyeSpyMessage;
}

const ::std::string&
IceProxy::RobotSimEvents::ControlCameraMessage::ice_staticId()
{
    return ::RobotSimEvents::ControlCameraMessage::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::RobotSimEvents::ControlCameraMessage::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::RobotSimEvents::ControlCameraMessage);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::RobotSimEvents::ControlCameraMessage::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::RobotSimEvents::ControlCameraMessage);
}

::IceProxy::Ice::Object*
IceProxy::RobotSimEvents::ControlCameraMessage::__newInstance() const
{
    return new ControlCameraMessage;
}

const ::std::string&
IceProxy::RobotSimEvents::CameraImageMessage::ice_staticId()
{
    return ::RobotSimEvents::CameraImageMessage::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::RobotSimEvents::CameraImageMessage::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::RobotSimEvents::CameraImageMessage);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::RobotSimEvents::CameraImageMessage::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::RobotSimEvents::CameraImageMessage);
}

::IceProxy::Ice::Object*
IceProxy::RobotSimEvents::CameraImageMessage::__newInstance() const
{
    return new CameraImageMessage;
}

const ::std::string&
IceProxy::RobotSimEvents::ControlDriveVisionMessage::ice_staticId()
{
    return ::RobotSimEvents::ControlDriveVisionMessage::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::RobotSimEvents::ControlDriveVisionMessage::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::RobotSimEvents::ControlDriveVisionMessage);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::RobotSimEvents::ControlDriveVisionMessage::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::RobotSimEvents::ControlDriveVisionMessage);
}

::IceProxy::Ice::Object*
IceProxy::RobotSimEvents::ControlDriveVisionMessage::__newInstance() const
{
    return new ControlDriveVisionMessage;
}

const ::std::string&
IceProxy::RobotSimEvents::ExecuteMissionMessage::ice_staticId()
{
    return ::RobotSimEvents::ExecuteMissionMessage::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::RobotSimEvents::ExecuteMissionMessage::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::RobotSimEvents::ExecuteMissionMessage);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::RobotSimEvents::ExecuteMissionMessage::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::RobotSimEvents::ExecuteMissionMessage);
}

::IceProxy::Ice::Object*
IceProxy::RobotSimEvents::ExecuteMissionMessage::__newInstance() const
{
    return new ExecuteMissionMessage;
}

const ::std::string&
IceProxy::RobotSimEvents::SlamDataMessage::ice_staticId()
{
    return ::RobotSimEvents::SlamDataMessage::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::RobotSimEvents::SlamDataMessage::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::RobotSimEvents::SlamDataMessage);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::RobotSimEvents::SlamDataMessage::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::RobotSimEvents::SlamDataMessage);
}

::IceProxy::Ice::Object*
IceProxy::RobotSimEvents::SlamDataMessage::__newInstance() const
{
    return new SlamDataMessage;
}

const ::std::string&
IceProxy::RobotSimEvents::ControlLandMessage::ice_staticId()
{
    return ::RobotSimEvents::ControlLandMessage::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::RobotSimEvents::ControlLandMessage::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::RobotSimEvents::ControlLandMessage);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::RobotSimEvents::ControlLandMessage::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::RobotSimEvents::ControlLandMessage);
}

::IceProxy::Ice::Object*
IceProxy::RobotSimEvents::ControlLandMessage::__newInstance() const
{
    return new ControlLandMessage;
}

const ::std::string&
IceProxy::RobotSimEvents::ControlMoveMessage::ice_staticId()
{
    return ::RobotSimEvents::ControlMoveMessage::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::RobotSimEvents::ControlMoveMessage::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::RobotSimEvents::ControlMoveMessage);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::RobotSimEvents::ControlMoveMessage::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::RobotSimEvents::ControlMoveMessage);
}

::IceProxy::Ice::Object*
IceProxy::RobotSimEvents::ControlMoveMessage::__newInstance() const
{
    return new ControlMoveMessage;
}

const ::std::string&
IceProxy::RobotSimEvents::SensorDataMessage::ice_staticId()
{
    return ::RobotSimEvents::SensorDataMessage::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::RobotSimEvents::SensorDataMessage::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::RobotSimEvents::SensorDataMessage);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::RobotSimEvents::SensorDataMessage::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::RobotSimEvents::SensorDataMessage);
}

::IceProxy::Ice::Object*
IceProxy::RobotSimEvents::SensorDataMessage::__newInstance() const
{
    return new SensorDataMessage;
}

RobotSimEvents::BeoHawkEyeSpyMessage::BeoHawkEyeSpyMessage(::RobotSimEvents::BeoHawkEyeSpyType __ice_foundType, const ::std::string& __ice_cameraID, const ::ImageIceMod::Point3DIce& __ice_topLeft, const ::ImageIceMod::Point3DIce& __ice_topRight, const ::ImageIceMod::Point3DIce& __ice_bottomLeft, const ::ImageIceMod::Point3DIce& __ice_bottomRight, ::Ice::Float __ice_certaintyLevel) :
    foundType(__ice_foundType),
    cameraID(__ice_cameraID),
    topLeft(__ice_topLeft),
    topRight(__ice_topRight),
    bottomLeft(__ice_bottomLeft),
    bottomRight(__ice_bottomRight),
    certaintyLevel(__ice_certaintyLevel)
{
}

::Ice::ObjectPtr
RobotSimEvents::BeoHawkEyeSpyMessage::ice_clone() const
{
    ::RobotSimEvents::BeoHawkEyeSpyMessagePtr __p = new ::RobotSimEvents::BeoHawkEyeSpyMessage(*this);
    return __p;
}

static const ::std::string __RobotSimEvents__BeoHawkEyeSpyMessage_ids[3] =
{
    "::Ice::Object",
    "::RobotSimEvents::BeoHawkEyeSpyMessage",
    "::RobotSimEvents::EventMessage"
};

bool
RobotSimEvents::BeoHawkEyeSpyMessage::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__RobotSimEvents__BeoHawkEyeSpyMessage_ids, __RobotSimEvents__BeoHawkEyeSpyMessage_ids + 3, _s);
}

::std::vector< ::std::string>
RobotSimEvents::BeoHawkEyeSpyMessage::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__RobotSimEvents__BeoHawkEyeSpyMessage_ids[0], &__RobotSimEvents__BeoHawkEyeSpyMessage_ids[3]);
}

const ::std::string&
RobotSimEvents::BeoHawkEyeSpyMessage::ice_id(const ::Ice::Current&) const
{
    return __RobotSimEvents__BeoHawkEyeSpyMessage_ids[1];
}

const ::std::string&
RobotSimEvents::BeoHawkEyeSpyMessage::ice_staticId()
{
    return __RobotSimEvents__BeoHawkEyeSpyMessage_ids[1];
}

void
RobotSimEvents::BeoHawkEyeSpyMessage::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    ::RobotSimEvents::__write(__os, foundType);
    __os->write(cameraID);
    topLeft.__write(__os);
    topRight.__write(__os);
    bottomLeft.__write(__os);
    bottomRight.__write(__os);
    __os->write(certaintyLevel);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    EventMessage::__write(__os);
#else
    ::RobotSimEvents::EventMessage::__write(__os);
#endif
}

void
RobotSimEvents::BeoHawkEyeSpyMessage::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    ::RobotSimEvents::__read(__is, foundType);
    __is->read(cameraID);
    topLeft.__read(__is);
    topRight.__read(__is);
    bottomLeft.__read(__is);
    bottomRight.__read(__is);
    __is->read(certaintyLevel);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    EventMessage::__read(__is, true);
#else
    ::RobotSimEvents::EventMessage::__read(__is, true);
#endif
}

void
RobotSimEvents::BeoHawkEyeSpyMessage::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type RobotSimEvents::BeoHawkEyeSpyMessage was not generated with stream support";
    throw ex;
}

void
RobotSimEvents::BeoHawkEyeSpyMessage::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type RobotSimEvents::BeoHawkEyeSpyMessage was not generated with stream support";
    throw ex;
}

class __F__RobotSimEvents__BeoHawkEyeSpyMessage : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::RobotSimEvents::BeoHawkEyeSpyMessage::ice_staticId());
        return new ::RobotSimEvents::BeoHawkEyeSpyMessage;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__RobotSimEvents__BeoHawkEyeSpyMessage_Ptr = new __F__RobotSimEvents__BeoHawkEyeSpyMessage;

const ::Ice::ObjectFactoryPtr&
RobotSimEvents::BeoHawkEyeSpyMessage::ice_factory()
{
    return __F__RobotSimEvents__BeoHawkEyeSpyMessage_Ptr;
}

class __F__RobotSimEvents__BeoHawkEyeSpyMessage__Init
{
public:

    __F__RobotSimEvents__BeoHawkEyeSpyMessage__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::RobotSimEvents::BeoHawkEyeSpyMessage::ice_staticId(), ::RobotSimEvents::BeoHawkEyeSpyMessage::ice_factory());
    }

    ~__F__RobotSimEvents__BeoHawkEyeSpyMessage__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::RobotSimEvents::BeoHawkEyeSpyMessage::ice_staticId());
    }
};

static __F__RobotSimEvents__BeoHawkEyeSpyMessage__Init __F__RobotSimEvents__BeoHawkEyeSpyMessage__i;

#ifdef __APPLE__
extern "C" { void __F__RobotSimEvents__BeoHawkEyeSpyMessage__initializer() {} }
#endif

void
RobotSimEvents::__patch__BeoHawkEyeSpyMessagePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::RobotSimEvents::BeoHawkEyeSpyMessagePtr* p = static_cast< ::RobotSimEvents::BeoHawkEyeSpyMessagePtr*>(__addr);
    assert(p);
    *p = ::RobotSimEvents::BeoHawkEyeSpyMessagePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::RobotSimEvents::BeoHawkEyeSpyMessage::ice_staticId(), v->ice_id());
    }
}

bool
RobotSimEvents::operator==(const ::RobotSimEvents::BeoHawkEyeSpyMessage& l, const ::RobotSimEvents::BeoHawkEyeSpyMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
RobotSimEvents::operator<(const ::RobotSimEvents::BeoHawkEyeSpyMessage& l, const ::RobotSimEvents::BeoHawkEyeSpyMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

RobotSimEvents::ControlCameraMessage::ControlCameraMessage(const ::std::string& __ice_cameraID, const ::std::string& __ice_compression, ::Ice::Int __ice_fps, bool __ice_cameraOn) :
    cameraID(__ice_cameraID),
    compression(__ice_compression),
    fps(__ice_fps),
    cameraOn(__ice_cameraOn)
{
}

::Ice::ObjectPtr
RobotSimEvents::ControlCameraMessage::ice_clone() const
{
    ::RobotSimEvents::ControlCameraMessagePtr __p = new ::RobotSimEvents::ControlCameraMessage(*this);
    return __p;
}

static const ::std::string __RobotSimEvents__ControlCameraMessage_ids[3] =
{
    "::Ice::Object",
    "::RobotSimEvents::ControlCameraMessage",
    "::RobotSimEvents::EventMessage"
};

bool
RobotSimEvents::ControlCameraMessage::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__RobotSimEvents__ControlCameraMessage_ids, __RobotSimEvents__ControlCameraMessage_ids + 3, _s);
}

::std::vector< ::std::string>
RobotSimEvents::ControlCameraMessage::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__RobotSimEvents__ControlCameraMessage_ids[0], &__RobotSimEvents__ControlCameraMessage_ids[3]);
}

const ::std::string&
RobotSimEvents::ControlCameraMessage::ice_id(const ::Ice::Current&) const
{
    return __RobotSimEvents__ControlCameraMessage_ids[1];
}

const ::std::string&
RobotSimEvents::ControlCameraMessage::ice_staticId()
{
    return __RobotSimEvents__ControlCameraMessage_ids[1];
}

void
RobotSimEvents::ControlCameraMessage::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(cameraID);
    __os->write(compression);
    __os->write(fps);
    __os->write(cameraOn);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    EventMessage::__write(__os);
#else
    ::RobotSimEvents::EventMessage::__write(__os);
#endif
}

void
RobotSimEvents::ControlCameraMessage::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(cameraID);
    __is->read(compression);
    __is->read(fps);
    __is->read(cameraOn);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    EventMessage::__read(__is, true);
#else
    ::RobotSimEvents::EventMessage::__read(__is, true);
#endif
}

void
RobotSimEvents::ControlCameraMessage::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type RobotSimEvents::ControlCameraMessage was not generated with stream support";
    throw ex;
}

void
RobotSimEvents::ControlCameraMessage::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type RobotSimEvents::ControlCameraMessage was not generated with stream support";
    throw ex;
}

class __F__RobotSimEvents__ControlCameraMessage : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::RobotSimEvents::ControlCameraMessage::ice_staticId());
        return new ::RobotSimEvents::ControlCameraMessage;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__RobotSimEvents__ControlCameraMessage_Ptr = new __F__RobotSimEvents__ControlCameraMessage;

const ::Ice::ObjectFactoryPtr&
RobotSimEvents::ControlCameraMessage::ice_factory()
{
    return __F__RobotSimEvents__ControlCameraMessage_Ptr;
}

class __F__RobotSimEvents__ControlCameraMessage__Init
{
public:

    __F__RobotSimEvents__ControlCameraMessage__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::RobotSimEvents::ControlCameraMessage::ice_staticId(), ::RobotSimEvents::ControlCameraMessage::ice_factory());
    }

    ~__F__RobotSimEvents__ControlCameraMessage__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::RobotSimEvents::ControlCameraMessage::ice_staticId());
    }
};

static __F__RobotSimEvents__ControlCameraMessage__Init __F__RobotSimEvents__ControlCameraMessage__i;

#ifdef __APPLE__
extern "C" { void __F__RobotSimEvents__ControlCameraMessage__initializer() {} }
#endif

void
RobotSimEvents::__patch__ControlCameraMessagePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::RobotSimEvents::ControlCameraMessagePtr* p = static_cast< ::RobotSimEvents::ControlCameraMessagePtr*>(__addr);
    assert(p);
    *p = ::RobotSimEvents::ControlCameraMessagePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::RobotSimEvents::ControlCameraMessage::ice_staticId(), v->ice_id());
    }
}

bool
RobotSimEvents::operator==(const ::RobotSimEvents::ControlCameraMessage& l, const ::RobotSimEvents::ControlCameraMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
RobotSimEvents::operator<(const ::RobotSimEvents::ControlCameraMessage& l, const ::RobotSimEvents::ControlCameraMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

RobotSimEvents::CameraImageMessage::CameraImageMessage(const ::std::string& __ice_cameraID, const ::std::string& __ice_compression, const ::ImageIceMod::ImageIce& __ice_img) :
    cameraID(__ice_cameraID),
    compression(__ice_compression),
    img(__ice_img)
{
}

::Ice::ObjectPtr
RobotSimEvents::CameraImageMessage::ice_clone() const
{
    ::RobotSimEvents::CameraImageMessagePtr __p = new ::RobotSimEvents::CameraImageMessage(*this);
    return __p;
}

static const ::std::string __RobotSimEvents__CameraImageMessage_ids[3] =
{
    "::Ice::Object",
    "::RobotSimEvents::CameraImageMessage",
    "::RobotSimEvents::EventMessage"
};

bool
RobotSimEvents::CameraImageMessage::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__RobotSimEvents__CameraImageMessage_ids, __RobotSimEvents__CameraImageMessage_ids + 3, _s);
}

::std::vector< ::std::string>
RobotSimEvents::CameraImageMessage::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__RobotSimEvents__CameraImageMessage_ids[0], &__RobotSimEvents__CameraImageMessage_ids[3]);
}

const ::std::string&
RobotSimEvents::CameraImageMessage::ice_id(const ::Ice::Current&) const
{
    return __RobotSimEvents__CameraImageMessage_ids[1];
}

const ::std::string&
RobotSimEvents::CameraImageMessage::ice_staticId()
{
    return __RobotSimEvents__CameraImageMessage_ids[1];
}

void
RobotSimEvents::CameraImageMessage::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(cameraID);
    __os->write(compression);
    img.__write(__os);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    EventMessage::__write(__os);
#else
    ::RobotSimEvents::EventMessage::__write(__os);
#endif
}

void
RobotSimEvents::CameraImageMessage::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(cameraID);
    __is->read(compression);
    img.__read(__is);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    EventMessage::__read(__is, true);
#else
    ::RobotSimEvents::EventMessage::__read(__is, true);
#endif
}

void
RobotSimEvents::CameraImageMessage::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type RobotSimEvents::CameraImageMessage was not generated with stream support";
    throw ex;
}

void
RobotSimEvents::CameraImageMessage::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type RobotSimEvents::CameraImageMessage was not generated with stream support";
    throw ex;
}

class __F__RobotSimEvents__CameraImageMessage : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::RobotSimEvents::CameraImageMessage::ice_staticId());
        return new ::RobotSimEvents::CameraImageMessage;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__RobotSimEvents__CameraImageMessage_Ptr = new __F__RobotSimEvents__CameraImageMessage;

const ::Ice::ObjectFactoryPtr&
RobotSimEvents::CameraImageMessage::ice_factory()
{
    return __F__RobotSimEvents__CameraImageMessage_Ptr;
}

class __F__RobotSimEvents__CameraImageMessage__Init
{
public:

    __F__RobotSimEvents__CameraImageMessage__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::RobotSimEvents::CameraImageMessage::ice_staticId(), ::RobotSimEvents::CameraImageMessage::ice_factory());
    }

    ~__F__RobotSimEvents__CameraImageMessage__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::RobotSimEvents::CameraImageMessage::ice_staticId());
    }
};

static __F__RobotSimEvents__CameraImageMessage__Init __F__RobotSimEvents__CameraImageMessage__i;

#ifdef __APPLE__
extern "C" { void __F__RobotSimEvents__CameraImageMessage__initializer() {} }
#endif

void
RobotSimEvents::__patch__CameraImageMessagePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::RobotSimEvents::CameraImageMessagePtr* p = static_cast< ::RobotSimEvents::CameraImageMessagePtr*>(__addr);
    assert(p);
    *p = ::RobotSimEvents::CameraImageMessagePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::RobotSimEvents::CameraImageMessage::ice_staticId(), v->ice_id());
    }
}

bool
RobotSimEvents::operator==(const ::RobotSimEvents::CameraImageMessage& l, const ::RobotSimEvents::CameraImageMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
RobotSimEvents::operator<(const ::RobotSimEvents::CameraImageMessage& l, const ::RobotSimEvents::CameraImageMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

RobotSimEvents::ControlDriveVisionMessage::ControlDriveVisionMessage(bool __ice_drivevisionOn) :
    drivevisionOn(__ice_drivevisionOn)
{
}

::Ice::ObjectPtr
RobotSimEvents::ControlDriveVisionMessage::ice_clone() const
{
    ::RobotSimEvents::ControlDriveVisionMessagePtr __p = new ::RobotSimEvents::ControlDriveVisionMessage(*this);
    return __p;
}

static const ::std::string __RobotSimEvents__ControlDriveVisionMessage_ids[3] =
{
    "::Ice::Object",
    "::RobotSimEvents::ControlDriveVisionMessage",
    "::RobotSimEvents::EventMessage"
};

bool
RobotSimEvents::ControlDriveVisionMessage::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__RobotSimEvents__ControlDriveVisionMessage_ids, __RobotSimEvents__ControlDriveVisionMessage_ids + 3, _s);
}

::std::vector< ::std::string>
RobotSimEvents::ControlDriveVisionMessage::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__RobotSimEvents__ControlDriveVisionMessage_ids[0], &__RobotSimEvents__ControlDriveVisionMessage_ids[3]);
}

const ::std::string&
RobotSimEvents::ControlDriveVisionMessage::ice_id(const ::Ice::Current&) const
{
    return __RobotSimEvents__ControlDriveVisionMessage_ids[1];
}

const ::std::string&
RobotSimEvents::ControlDriveVisionMessage::ice_staticId()
{
    return __RobotSimEvents__ControlDriveVisionMessage_ids[1];
}

void
RobotSimEvents::ControlDriveVisionMessage::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(drivevisionOn);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    EventMessage::__write(__os);
#else
    ::RobotSimEvents::EventMessage::__write(__os);
#endif
}

void
RobotSimEvents::ControlDriveVisionMessage::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(drivevisionOn);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    EventMessage::__read(__is, true);
#else
    ::RobotSimEvents::EventMessage::__read(__is, true);
#endif
}

void
RobotSimEvents::ControlDriveVisionMessage::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type RobotSimEvents::ControlDriveVisionMessage was not generated with stream support";
    throw ex;
}

void
RobotSimEvents::ControlDriveVisionMessage::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type RobotSimEvents::ControlDriveVisionMessage was not generated with stream support";
    throw ex;
}

class __F__RobotSimEvents__ControlDriveVisionMessage : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::RobotSimEvents::ControlDriveVisionMessage::ice_staticId());
        return new ::RobotSimEvents::ControlDriveVisionMessage;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__RobotSimEvents__ControlDriveVisionMessage_Ptr = new __F__RobotSimEvents__ControlDriveVisionMessage;

const ::Ice::ObjectFactoryPtr&
RobotSimEvents::ControlDriveVisionMessage::ice_factory()
{
    return __F__RobotSimEvents__ControlDriveVisionMessage_Ptr;
}

class __F__RobotSimEvents__ControlDriveVisionMessage__Init
{
public:

    __F__RobotSimEvents__ControlDriveVisionMessage__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::RobotSimEvents::ControlDriveVisionMessage::ice_staticId(), ::RobotSimEvents::ControlDriveVisionMessage::ice_factory());
    }

    ~__F__RobotSimEvents__ControlDriveVisionMessage__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::RobotSimEvents::ControlDriveVisionMessage::ice_staticId());
    }
};

static __F__RobotSimEvents__ControlDriveVisionMessage__Init __F__RobotSimEvents__ControlDriveVisionMessage__i;

#ifdef __APPLE__
extern "C" { void __F__RobotSimEvents__ControlDriveVisionMessage__initializer() {} }
#endif

void
RobotSimEvents::__patch__ControlDriveVisionMessagePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::RobotSimEvents::ControlDriveVisionMessagePtr* p = static_cast< ::RobotSimEvents::ControlDriveVisionMessagePtr*>(__addr);
    assert(p);
    *p = ::RobotSimEvents::ControlDriveVisionMessagePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::RobotSimEvents::ControlDriveVisionMessage::ice_staticId(), v->ice_id());
    }
}

bool
RobotSimEvents::operator==(const ::RobotSimEvents::ControlDriveVisionMessage& l, const ::RobotSimEvents::ControlDriveVisionMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
RobotSimEvents::operator<(const ::RobotSimEvents::ControlDriveVisionMessage& l, const ::RobotSimEvents::ControlDriveVisionMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

RobotSimEvents::ExecuteMissionMessage::ExecuteMissionMessage(const ::std::string& __ice_mission) :
    mission(__ice_mission)
{
}

::Ice::ObjectPtr
RobotSimEvents::ExecuteMissionMessage::ice_clone() const
{
    ::RobotSimEvents::ExecuteMissionMessagePtr __p = new ::RobotSimEvents::ExecuteMissionMessage(*this);
    return __p;
}

static const ::std::string __RobotSimEvents__ExecuteMissionMessage_ids[3] =
{
    "::Ice::Object",
    "::RobotSimEvents::EventMessage",
    "::RobotSimEvents::ExecuteMissionMessage"
};

bool
RobotSimEvents::ExecuteMissionMessage::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__RobotSimEvents__ExecuteMissionMessage_ids, __RobotSimEvents__ExecuteMissionMessage_ids + 3, _s);
}

::std::vector< ::std::string>
RobotSimEvents::ExecuteMissionMessage::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__RobotSimEvents__ExecuteMissionMessage_ids[0], &__RobotSimEvents__ExecuteMissionMessage_ids[3]);
}

const ::std::string&
RobotSimEvents::ExecuteMissionMessage::ice_id(const ::Ice::Current&) const
{
    return __RobotSimEvents__ExecuteMissionMessage_ids[2];
}

const ::std::string&
RobotSimEvents::ExecuteMissionMessage::ice_staticId()
{
    return __RobotSimEvents__ExecuteMissionMessage_ids[2];
}

void
RobotSimEvents::ExecuteMissionMessage::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(mission);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    EventMessage::__write(__os);
#else
    ::RobotSimEvents::EventMessage::__write(__os);
#endif
}

void
RobotSimEvents::ExecuteMissionMessage::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(mission);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    EventMessage::__read(__is, true);
#else
    ::RobotSimEvents::EventMessage::__read(__is, true);
#endif
}

void
RobotSimEvents::ExecuteMissionMessage::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type RobotSimEvents::ExecuteMissionMessage was not generated with stream support";
    throw ex;
}

void
RobotSimEvents::ExecuteMissionMessage::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type RobotSimEvents::ExecuteMissionMessage was not generated with stream support";
    throw ex;
}

class __F__RobotSimEvents__ExecuteMissionMessage : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::RobotSimEvents::ExecuteMissionMessage::ice_staticId());
        return new ::RobotSimEvents::ExecuteMissionMessage;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__RobotSimEvents__ExecuteMissionMessage_Ptr = new __F__RobotSimEvents__ExecuteMissionMessage;

const ::Ice::ObjectFactoryPtr&
RobotSimEvents::ExecuteMissionMessage::ice_factory()
{
    return __F__RobotSimEvents__ExecuteMissionMessage_Ptr;
}

class __F__RobotSimEvents__ExecuteMissionMessage__Init
{
public:

    __F__RobotSimEvents__ExecuteMissionMessage__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::RobotSimEvents::ExecuteMissionMessage::ice_staticId(), ::RobotSimEvents::ExecuteMissionMessage::ice_factory());
    }

    ~__F__RobotSimEvents__ExecuteMissionMessage__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::RobotSimEvents::ExecuteMissionMessage::ice_staticId());
    }
};

static __F__RobotSimEvents__ExecuteMissionMessage__Init __F__RobotSimEvents__ExecuteMissionMessage__i;

#ifdef __APPLE__
extern "C" { void __F__RobotSimEvents__ExecuteMissionMessage__initializer() {} }
#endif

void
RobotSimEvents::__patch__ExecuteMissionMessagePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::RobotSimEvents::ExecuteMissionMessagePtr* p = static_cast< ::RobotSimEvents::ExecuteMissionMessagePtr*>(__addr);
    assert(p);
    *p = ::RobotSimEvents::ExecuteMissionMessagePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::RobotSimEvents::ExecuteMissionMessage::ice_staticId(), v->ice_id());
    }
}

bool
RobotSimEvents::operator==(const ::RobotSimEvents::ExecuteMissionMessage& l, const ::RobotSimEvents::ExecuteMissionMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
RobotSimEvents::operator<(const ::RobotSimEvents::ExecuteMissionMessage& l, const ::RobotSimEvents::ExecuteMissionMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

RobotSimEvents::SlamDataMessage::SlamDataMessage(const ::RobotSimEvents::Location& __ice_lctn) :
    lctn(__ice_lctn)
{
}

::Ice::ObjectPtr
RobotSimEvents::SlamDataMessage::ice_clone() const
{
    ::RobotSimEvents::SlamDataMessagePtr __p = new ::RobotSimEvents::SlamDataMessage(*this);
    return __p;
}

static const ::std::string __RobotSimEvents__SlamDataMessage_ids[3] =
{
    "::Ice::Object",
    "::RobotSimEvents::EventMessage",
    "::RobotSimEvents::SlamDataMessage"
};

bool
RobotSimEvents::SlamDataMessage::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__RobotSimEvents__SlamDataMessage_ids, __RobotSimEvents__SlamDataMessage_ids + 3, _s);
}

::std::vector< ::std::string>
RobotSimEvents::SlamDataMessage::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__RobotSimEvents__SlamDataMessage_ids[0], &__RobotSimEvents__SlamDataMessage_ids[3]);
}

const ::std::string&
RobotSimEvents::SlamDataMessage::ice_id(const ::Ice::Current&) const
{
    return __RobotSimEvents__SlamDataMessage_ids[2];
}

const ::std::string&
RobotSimEvents::SlamDataMessage::ice_staticId()
{
    return __RobotSimEvents__SlamDataMessage_ids[2];
}

void
RobotSimEvents::SlamDataMessage::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    lctn.__write(__os);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    EventMessage::__write(__os);
#else
    ::RobotSimEvents::EventMessage::__write(__os);
#endif
}

void
RobotSimEvents::SlamDataMessage::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    lctn.__read(__is);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    EventMessage::__read(__is, true);
#else
    ::RobotSimEvents::EventMessage::__read(__is, true);
#endif
}

void
RobotSimEvents::SlamDataMessage::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type RobotSimEvents::SlamDataMessage was not generated with stream support";
    throw ex;
}

void
RobotSimEvents::SlamDataMessage::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type RobotSimEvents::SlamDataMessage was not generated with stream support";
    throw ex;
}

class __F__RobotSimEvents__SlamDataMessage : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::RobotSimEvents::SlamDataMessage::ice_staticId());
        return new ::RobotSimEvents::SlamDataMessage;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__RobotSimEvents__SlamDataMessage_Ptr = new __F__RobotSimEvents__SlamDataMessage;

const ::Ice::ObjectFactoryPtr&
RobotSimEvents::SlamDataMessage::ice_factory()
{
    return __F__RobotSimEvents__SlamDataMessage_Ptr;
}

class __F__RobotSimEvents__SlamDataMessage__Init
{
public:

    __F__RobotSimEvents__SlamDataMessage__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::RobotSimEvents::SlamDataMessage::ice_staticId(), ::RobotSimEvents::SlamDataMessage::ice_factory());
    }

    ~__F__RobotSimEvents__SlamDataMessage__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::RobotSimEvents::SlamDataMessage::ice_staticId());
    }
};

static __F__RobotSimEvents__SlamDataMessage__Init __F__RobotSimEvents__SlamDataMessage__i;

#ifdef __APPLE__
extern "C" { void __F__RobotSimEvents__SlamDataMessage__initializer() {} }
#endif

void
RobotSimEvents::__patch__SlamDataMessagePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::RobotSimEvents::SlamDataMessagePtr* p = static_cast< ::RobotSimEvents::SlamDataMessagePtr*>(__addr);
    assert(p);
    *p = ::RobotSimEvents::SlamDataMessagePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::RobotSimEvents::SlamDataMessage::ice_staticId(), v->ice_id());
    }
}

bool
RobotSimEvents::operator==(const ::RobotSimEvents::SlamDataMessage& l, const ::RobotSimEvents::SlamDataMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
RobotSimEvents::operator<(const ::RobotSimEvents::SlamDataMessage& l, const ::RobotSimEvents::SlamDataMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

::Ice::ObjectPtr
RobotSimEvents::ControlLandMessage::ice_clone() const
{
    ::RobotSimEvents::ControlLandMessagePtr __p = new ::RobotSimEvents::ControlLandMessage(*this);
    return __p;
}

static const ::std::string __RobotSimEvents__ControlLandMessage_ids[3] =
{
    "::Ice::Object",
    "::RobotSimEvents::ControlLandMessage",
    "::RobotSimEvents::EventMessage"
};

bool
RobotSimEvents::ControlLandMessage::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__RobotSimEvents__ControlLandMessage_ids, __RobotSimEvents__ControlLandMessage_ids + 3, _s);
}

::std::vector< ::std::string>
RobotSimEvents::ControlLandMessage::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__RobotSimEvents__ControlLandMessage_ids[0], &__RobotSimEvents__ControlLandMessage_ids[3]);
}

const ::std::string&
RobotSimEvents::ControlLandMessage::ice_id(const ::Ice::Current&) const
{
    return __RobotSimEvents__ControlLandMessage_ids[1];
}

const ::std::string&
RobotSimEvents::ControlLandMessage::ice_staticId()
{
    return __RobotSimEvents__ControlLandMessage_ids[1];
}

void
RobotSimEvents::ControlLandMessage::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    EventMessage::__write(__os);
#else
    ::RobotSimEvents::EventMessage::__write(__os);
#endif
}

void
RobotSimEvents::ControlLandMessage::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    EventMessage::__read(__is, true);
#else
    ::RobotSimEvents::EventMessage::__read(__is, true);
#endif
}

void
RobotSimEvents::ControlLandMessage::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type RobotSimEvents::ControlLandMessage was not generated with stream support";
    throw ex;
}

void
RobotSimEvents::ControlLandMessage::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type RobotSimEvents::ControlLandMessage was not generated with stream support";
    throw ex;
}

class __F__RobotSimEvents__ControlLandMessage : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::RobotSimEvents::ControlLandMessage::ice_staticId());
        return new ::RobotSimEvents::ControlLandMessage;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__RobotSimEvents__ControlLandMessage_Ptr = new __F__RobotSimEvents__ControlLandMessage;

const ::Ice::ObjectFactoryPtr&
RobotSimEvents::ControlLandMessage::ice_factory()
{
    return __F__RobotSimEvents__ControlLandMessage_Ptr;
}

class __F__RobotSimEvents__ControlLandMessage__Init
{
public:

    __F__RobotSimEvents__ControlLandMessage__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::RobotSimEvents::ControlLandMessage::ice_staticId(), ::RobotSimEvents::ControlLandMessage::ice_factory());
    }

    ~__F__RobotSimEvents__ControlLandMessage__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::RobotSimEvents::ControlLandMessage::ice_staticId());
    }
};

static __F__RobotSimEvents__ControlLandMessage__Init __F__RobotSimEvents__ControlLandMessage__i;

#ifdef __APPLE__
extern "C" { void __F__RobotSimEvents__ControlLandMessage__initializer() {} }
#endif

void
RobotSimEvents::__patch__ControlLandMessagePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::RobotSimEvents::ControlLandMessagePtr* p = static_cast< ::RobotSimEvents::ControlLandMessagePtr*>(__addr);
    assert(p);
    *p = ::RobotSimEvents::ControlLandMessagePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::RobotSimEvents::ControlLandMessage::ice_staticId(), v->ice_id());
    }
}

bool
RobotSimEvents::operator==(const ::RobotSimEvents::ControlLandMessage& l, const ::RobotSimEvents::ControlLandMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
RobotSimEvents::operator<(const ::RobotSimEvents::ControlLandMessage& l, const ::RobotSimEvents::ControlLandMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

RobotSimEvents::ControlMoveMessage::ControlMoveMessage(const ::RobotSimEvents::Location& __ice_move) :
    move(__ice_move)
{
}

::Ice::ObjectPtr
RobotSimEvents::ControlMoveMessage::ice_clone() const
{
    ::RobotSimEvents::ControlMoveMessagePtr __p = new ::RobotSimEvents::ControlMoveMessage(*this);
    return __p;
}

static const ::std::string __RobotSimEvents__ControlMoveMessage_ids[3] =
{
    "::Ice::Object",
    "::RobotSimEvents::ControlMoveMessage",
    "::RobotSimEvents::EventMessage"
};

bool
RobotSimEvents::ControlMoveMessage::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__RobotSimEvents__ControlMoveMessage_ids, __RobotSimEvents__ControlMoveMessage_ids + 3, _s);
}

::std::vector< ::std::string>
RobotSimEvents::ControlMoveMessage::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__RobotSimEvents__ControlMoveMessage_ids[0], &__RobotSimEvents__ControlMoveMessage_ids[3]);
}

const ::std::string&
RobotSimEvents::ControlMoveMessage::ice_id(const ::Ice::Current&) const
{
    return __RobotSimEvents__ControlMoveMessage_ids[1];
}

const ::std::string&
RobotSimEvents::ControlMoveMessage::ice_staticId()
{
    return __RobotSimEvents__ControlMoveMessage_ids[1];
}

void
RobotSimEvents::ControlMoveMessage::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    move.__write(__os);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    EventMessage::__write(__os);
#else
    ::RobotSimEvents::EventMessage::__write(__os);
#endif
}

void
RobotSimEvents::ControlMoveMessage::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    move.__read(__is);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    EventMessage::__read(__is, true);
#else
    ::RobotSimEvents::EventMessage::__read(__is, true);
#endif
}

void
RobotSimEvents::ControlMoveMessage::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type RobotSimEvents::ControlMoveMessage was not generated with stream support";
    throw ex;
}

void
RobotSimEvents::ControlMoveMessage::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type RobotSimEvents::ControlMoveMessage was not generated with stream support";
    throw ex;
}

class __F__RobotSimEvents__ControlMoveMessage : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::RobotSimEvents::ControlMoveMessage::ice_staticId());
        return new ::RobotSimEvents::ControlMoveMessage;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__RobotSimEvents__ControlMoveMessage_Ptr = new __F__RobotSimEvents__ControlMoveMessage;

const ::Ice::ObjectFactoryPtr&
RobotSimEvents::ControlMoveMessage::ice_factory()
{
    return __F__RobotSimEvents__ControlMoveMessage_Ptr;
}

class __F__RobotSimEvents__ControlMoveMessage__Init
{
public:

    __F__RobotSimEvents__ControlMoveMessage__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::RobotSimEvents::ControlMoveMessage::ice_staticId(), ::RobotSimEvents::ControlMoveMessage::ice_factory());
    }

    ~__F__RobotSimEvents__ControlMoveMessage__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::RobotSimEvents::ControlMoveMessage::ice_staticId());
    }
};

static __F__RobotSimEvents__ControlMoveMessage__Init __F__RobotSimEvents__ControlMoveMessage__i;

#ifdef __APPLE__
extern "C" { void __F__RobotSimEvents__ControlMoveMessage__initializer() {} }
#endif

void
RobotSimEvents::__patch__ControlMoveMessagePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::RobotSimEvents::ControlMoveMessagePtr* p = static_cast< ::RobotSimEvents::ControlMoveMessagePtr*>(__addr);
    assert(p);
    *p = ::RobotSimEvents::ControlMoveMessagePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::RobotSimEvents::ControlMoveMessage::ice_staticId(), v->ice_id());
    }
}

bool
RobotSimEvents::operator==(const ::RobotSimEvents::ControlMoveMessage& l, const ::RobotSimEvents::ControlMoveMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
RobotSimEvents::operator<(const ::RobotSimEvents::ControlMoveMessage& l, const ::RobotSimEvents::ControlMoveMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

RobotSimEvents::SensorDataMessage::SensorDataMessage(::Ice::Double __ice_motorSpeeds, bool __ice_validRollPitchYaw, ::Ice::Double __ice_pitch, ::Ice::Double __ice_yaw, ::Ice::Double __ice_absouteHeading, const ::RobotSimEvents::LrfSeq& __ice_lrf, const ::RobotSimEvents::SonarSeq& __ice_sonars) :
    motorSpeeds(__ice_motorSpeeds),
    validRollPitchYaw(__ice_validRollPitchYaw),
    pitch(__ice_pitch),
    yaw(__ice_yaw),
    absouteHeading(__ice_absouteHeading),
    lrf(__ice_lrf),
    sonars(__ice_sonars)
{
}

::Ice::ObjectPtr
RobotSimEvents::SensorDataMessage::ice_clone() const
{
    ::RobotSimEvents::SensorDataMessagePtr __p = new ::RobotSimEvents::SensorDataMessage(*this);
    return __p;
}

static const ::std::string __RobotSimEvents__SensorDataMessage_ids[3] =
{
    "::Ice::Object",
    "::RobotSimEvents::EventMessage",
    "::RobotSimEvents::SensorDataMessage"
};

bool
RobotSimEvents::SensorDataMessage::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__RobotSimEvents__SensorDataMessage_ids, __RobotSimEvents__SensorDataMessage_ids + 3, _s);
}

::std::vector< ::std::string>
RobotSimEvents::SensorDataMessage::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__RobotSimEvents__SensorDataMessage_ids[0], &__RobotSimEvents__SensorDataMessage_ids[3]);
}

const ::std::string&
RobotSimEvents::SensorDataMessage::ice_id(const ::Ice::Current&) const
{
    return __RobotSimEvents__SensorDataMessage_ids[2];
}

const ::std::string&
RobotSimEvents::SensorDataMessage::ice_staticId()
{
    return __RobotSimEvents__SensorDataMessage_ids[2];
}

void
RobotSimEvents::SensorDataMessage::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(motorSpeeds);
    __os->write(validRollPitchYaw);
    __os->write(pitch);
    __os->write(yaw);
    __os->write(absouteHeading);
    if(lrf.size() == 0)
    {
        __os->writeSize(0);
    }
    else
    {
        ::RobotSimEvents::__writeLrfSeq(__os, &lrf[0], &lrf[0] + lrf.size());
    }
    if(sonars.size() == 0)
    {
        __os->writeSize(0);
    }
    else
    {
        ::RobotSimEvents::__writeSonarSeq(__os, &sonars[0], &sonars[0] + sonars.size());
    }
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    EventMessage::__write(__os);
#else
    ::RobotSimEvents::EventMessage::__write(__os);
#endif
}

void
RobotSimEvents::SensorDataMessage::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(motorSpeeds);
    __is->read(validRollPitchYaw);
    __is->read(pitch);
    __is->read(yaw);
    __is->read(absouteHeading);
    ::RobotSimEvents::__readLrfSeq(__is, lrf);
    ::RobotSimEvents::__readSonarSeq(__is, sonars);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    EventMessage::__read(__is, true);
#else
    ::RobotSimEvents::EventMessage::__read(__is, true);
#endif
}

void
RobotSimEvents::SensorDataMessage::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type RobotSimEvents::SensorDataMessage was not generated with stream support";
    throw ex;
}

void
RobotSimEvents::SensorDataMessage::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type RobotSimEvents::SensorDataMessage was not generated with stream support";
    throw ex;
}

class __F__RobotSimEvents__SensorDataMessage : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::RobotSimEvents::SensorDataMessage::ice_staticId());
        return new ::RobotSimEvents::SensorDataMessage;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__RobotSimEvents__SensorDataMessage_Ptr = new __F__RobotSimEvents__SensorDataMessage;

const ::Ice::ObjectFactoryPtr&
RobotSimEvents::SensorDataMessage::ice_factory()
{
    return __F__RobotSimEvents__SensorDataMessage_Ptr;
}

class __F__RobotSimEvents__SensorDataMessage__Init
{
public:

    __F__RobotSimEvents__SensorDataMessage__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::RobotSimEvents::SensorDataMessage::ice_staticId(), ::RobotSimEvents::SensorDataMessage::ice_factory());
    }

    ~__F__RobotSimEvents__SensorDataMessage__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::RobotSimEvents::SensorDataMessage::ice_staticId());
    }
};

static __F__RobotSimEvents__SensorDataMessage__Init __F__RobotSimEvents__SensorDataMessage__i;

#ifdef __APPLE__
extern "C" { void __F__RobotSimEvents__SensorDataMessage__initializer() {} }
#endif

void
RobotSimEvents::__patch__SensorDataMessagePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::RobotSimEvents::SensorDataMessagePtr* p = static_cast< ::RobotSimEvents::SensorDataMessagePtr*>(__addr);
    assert(p);
    *p = ::RobotSimEvents::SensorDataMessagePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::RobotSimEvents::SensorDataMessage::ice_staticId(), v->ice_id());
    }
}

bool
RobotSimEvents::operator==(const ::RobotSimEvents::SensorDataMessage& l, const ::RobotSimEvents::SensorDataMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
RobotSimEvents::operator<(const ::RobotSimEvents::SensorDataMessage& l, const ::RobotSimEvents::SensorDataMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}
