// **********************************************************************
//
// Copyright (c) 2003-2008 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.0
// Generated from file `HawkMessages.ice'

#include <HawkMessages.ice.H>
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
#   if ICE_INT_VERSION % 100 < 0
#       error Ice patch level mismatch!
#   endif
#endif

static const ::std::string __HawkMessages__MessageAgent__catchMessage_name = "catchMessage";

::Ice::Object* IceInternal::upCast(::HawkMessages::Message* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::HawkMessages::Message* p) { return p; }

::Ice::Object* IceInternal::upCast(::HawkMessages::MessageAgent* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::HawkMessages::MessageAgent* p) { return p; }

::Ice::Object* IceInternal::upCast(::HawkMessages::CameraImageMessage* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::HawkMessages::CameraImageMessage* p) { return p; }

::Ice::Object* IceInternal::upCast(::HawkMessages::ControlCameraMessage* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::HawkMessages::ControlCameraMessage* p) { return p; }

::Ice::Object* IceInternal::upCast(::HawkMessages::ControlDriveVisionMessage* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::HawkMessages::ControlDriveVisionMessage* p) { return p; }

::Ice::Object* IceInternal::upCast(::HawkMessages::ControlLandMessage* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::HawkMessages::ControlLandMessage* p) { return p; }

::Ice::Object* IceInternal::upCast(::HawkMessages::ControlMoveMessage* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::HawkMessages::ControlMoveMessage* p) { return p; }

::Ice::Object* IceInternal::upCast(::HawkMessages::ControlRoomVisionMessage* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::HawkMessages::ControlRoomVisionMessage* p) { return p; }

::Ice::Object* IceInternal::upCast(::HawkMessages::ControlTakeOffMessage* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::HawkMessages::ControlTakeOffMessage* p) { return p; }

::Ice::Object* IceInternal::upCast(::HawkMessages::DriveFinderMessage* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::HawkMessages::DriveFinderMessage* p) { return p; }

::Ice::Object* IceInternal::upCast(::HawkMessages::ExampleMessage* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::HawkMessages::ExampleMessage* p) { return p; }

::Ice::Object* IceInternal::upCast(::HawkMessages::ExecuteMissionMessage* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::HawkMessages::ExecuteMissionMessage* p) { return p; }

::Ice::Object* IceInternal::upCast(::HawkMessages::MissionListMessage* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::HawkMessages::MissionListMessage* p) { return p; }

::Ice::Object* IceInternal::upCast(::HawkMessages::RoomFinderMessage* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::HawkMessages::RoomFinderMessage* p) { return p; }

::Ice::Object* IceInternal::upCast(::HawkMessages::SensorDataMessage* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::HawkMessages::SensorDataMessage* p) { return p; }

::Ice::Object* IceInternal::upCast(::HawkMessages::SlamDataMessage* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::HawkMessages::SlamDataMessage* p) { return p; }

void
HawkMessages::__read(::IceInternal::BasicStream* __is, ::HawkMessages::MessagePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::HawkMessages::Message;
        v->__copyFrom(proxy);
    }
}

void
HawkMessages::__read(::IceInternal::BasicStream* __is, ::HawkMessages::MessageAgentPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::HawkMessages::MessageAgent;
        v->__copyFrom(proxy);
    }
}

void
HawkMessages::__read(::IceInternal::BasicStream* __is, ::HawkMessages::CameraImageMessagePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::HawkMessages::CameraImageMessage;
        v->__copyFrom(proxy);
    }
}

void
HawkMessages::__read(::IceInternal::BasicStream* __is, ::HawkMessages::ControlCameraMessagePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::HawkMessages::ControlCameraMessage;
        v->__copyFrom(proxy);
    }
}

void
HawkMessages::__read(::IceInternal::BasicStream* __is, ::HawkMessages::ControlDriveVisionMessagePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::HawkMessages::ControlDriveVisionMessage;
        v->__copyFrom(proxy);
    }
}

void
HawkMessages::__read(::IceInternal::BasicStream* __is, ::HawkMessages::ControlLandMessagePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::HawkMessages::ControlLandMessage;
        v->__copyFrom(proxy);
    }
}

void
HawkMessages::__read(::IceInternal::BasicStream* __is, ::HawkMessages::ControlMoveMessagePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::HawkMessages::ControlMoveMessage;
        v->__copyFrom(proxy);
    }
}

void
HawkMessages::__read(::IceInternal::BasicStream* __is, ::HawkMessages::ControlRoomVisionMessagePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::HawkMessages::ControlRoomVisionMessage;
        v->__copyFrom(proxy);
    }
}

void
HawkMessages::__read(::IceInternal::BasicStream* __is, ::HawkMessages::ControlTakeOffMessagePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::HawkMessages::ControlTakeOffMessage;
        v->__copyFrom(proxy);
    }
}

void
HawkMessages::__read(::IceInternal::BasicStream* __is, ::HawkMessages::DriveFinderMessagePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::HawkMessages::DriveFinderMessage;
        v->__copyFrom(proxy);
    }
}

void
HawkMessages::__read(::IceInternal::BasicStream* __is, ::HawkMessages::ExampleMessagePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::HawkMessages::ExampleMessage;
        v->__copyFrom(proxy);
    }
}

void
HawkMessages::__read(::IceInternal::BasicStream* __is, ::HawkMessages::ExecuteMissionMessagePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::HawkMessages::ExecuteMissionMessage;
        v->__copyFrom(proxy);
    }
}

void
HawkMessages::__read(::IceInternal::BasicStream* __is, ::HawkMessages::MissionListMessagePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::HawkMessages::MissionListMessage;
        v->__copyFrom(proxy);
    }
}

void
HawkMessages::__read(::IceInternal::BasicStream* __is, ::HawkMessages::RoomFinderMessagePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::HawkMessages::RoomFinderMessage;
        v->__copyFrom(proxy);
    }
}

void
HawkMessages::__read(::IceInternal::BasicStream* __is, ::HawkMessages::SensorDataMessagePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::HawkMessages::SensorDataMessage;
        v->__copyFrom(proxy);
    }
}

void
HawkMessages::__read(::IceInternal::BasicStream* __is, ::HawkMessages::SlamDataMessagePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::HawkMessages::SlamDataMessage;
        v->__copyFrom(proxy);
    }
}

bool
HawkMessages::Pose::operator==(const Pose& __rhs) const
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
HawkMessages::Pose::operator<(const Pose& __rhs) const
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
HawkMessages::Pose::__write(::IceInternal::BasicStream* __os) const
{
    __os->write(x);
    __os->write(y);
    __os->write(z);
    __os->write(theta);
}

void
HawkMessages::Pose::__read(::IceInternal::BasicStream* __is)
{
    __is->read(x);
    __is->read(y);
    __is->read(z);
    __is->read(theta);
}

bool
HawkMessages::Sonar::operator==(const Sonar& __rhs) const
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
HawkMessages::Sonar::operator<(const Sonar& __rhs) const
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
HawkMessages::Sonar::__write(::IceInternal::BasicStream* __os) const
{
    __os->write(sonarID);
    __os->write(distance);
}

void
HawkMessages::Sonar::__read(::IceInternal::BasicStream* __is)
{
    __is->read(sonarID);
    __is->read(distance);
}

void
HawkMessages::__writeSonarSeq(::IceInternal::BasicStream* __os, const ::HawkMessages::Sonar* begin, const ::HawkMessages::Sonar* end)
{
    ::Ice::Int size = static_cast< ::Ice::Int>(end - begin);
    __os->writeSize(size);
    for(int i = 0; i < size; ++i)
    {
        begin[i].__write(__os);
    }
}

void
HawkMessages::__readSonarSeq(::IceInternal::BasicStream* __is, ::HawkMessages::SonarSeq& v)
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
IceProxy::HawkMessages::Message::ice_staticId()
{
    return ::HawkMessages::Message::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::HawkMessages::Message::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::HawkMessages::Message);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::HawkMessages::Message::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::HawkMessages::Message);
}

::IceProxy::Ice::Object*
IceProxy::HawkMessages::Message::__newInstance() const
{
    return new Message;
}

void
IceProxy::HawkMessages::MessageAgent::catchMessage(const ::HawkMessages::MessagePtr& msg, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __delBase = __getDelegate(false);
            ::IceDelegate::HawkMessages::MessageAgent* __del = dynamic_cast< ::IceDelegate::HawkMessages::MessageAgent*>(__delBase.get());
            __del->catchMessage(msg, __ctx);
            return;
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, 0);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, 0, __cnt);
        }
    }
}

const ::std::string&
IceProxy::HawkMessages::MessageAgent::ice_staticId()
{
    return ::HawkMessages::MessageAgent::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::HawkMessages::MessageAgent::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::HawkMessages::MessageAgent);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::HawkMessages::MessageAgent::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::HawkMessages::MessageAgent);
}

::IceProxy::Ice::Object*
IceProxy::HawkMessages::MessageAgent::__newInstance() const
{
    return new MessageAgent;
}

const ::std::string&
IceProxy::HawkMessages::CameraImageMessage::ice_staticId()
{
    return ::HawkMessages::CameraImageMessage::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::HawkMessages::CameraImageMessage::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::HawkMessages::CameraImageMessage);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::HawkMessages::CameraImageMessage::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::HawkMessages::CameraImageMessage);
}

::IceProxy::Ice::Object*
IceProxy::HawkMessages::CameraImageMessage::__newInstance() const
{
    return new CameraImageMessage;
}

const ::std::string&
IceProxy::HawkMessages::ControlCameraMessage::ice_staticId()
{
    return ::HawkMessages::ControlCameraMessage::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::HawkMessages::ControlCameraMessage::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::HawkMessages::ControlCameraMessage);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::HawkMessages::ControlCameraMessage::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::HawkMessages::ControlCameraMessage);
}

::IceProxy::Ice::Object*
IceProxy::HawkMessages::ControlCameraMessage::__newInstance() const
{
    return new ControlCameraMessage;
}

const ::std::string&
IceProxy::HawkMessages::ControlDriveVisionMessage::ice_staticId()
{
    return ::HawkMessages::ControlDriveVisionMessage::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::HawkMessages::ControlDriveVisionMessage::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::HawkMessages::ControlDriveVisionMessage);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::HawkMessages::ControlDriveVisionMessage::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::HawkMessages::ControlDriveVisionMessage);
}

::IceProxy::Ice::Object*
IceProxy::HawkMessages::ControlDriveVisionMessage::__newInstance() const
{
    return new ControlDriveVisionMessage;
}

const ::std::string&
IceProxy::HawkMessages::ControlLandMessage::ice_staticId()
{
    return ::HawkMessages::ControlLandMessage::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::HawkMessages::ControlLandMessage::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::HawkMessages::ControlLandMessage);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::HawkMessages::ControlLandMessage::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::HawkMessages::ControlLandMessage);
}

::IceProxy::Ice::Object*
IceProxy::HawkMessages::ControlLandMessage::__newInstance() const
{
    return new ControlLandMessage;
}

const ::std::string&
IceProxy::HawkMessages::ControlMoveMessage::ice_staticId()
{
    return ::HawkMessages::ControlMoveMessage::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::HawkMessages::ControlMoveMessage::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::HawkMessages::ControlMoveMessage);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::HawkMessages::ControlMoveMessage::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::HawkMessages::ControlMoveMessage);
}

::IceProxy::Ice::Object*
IceProxy::HawkMessages::ControlMoveMessage::__newInstance() const
{
    return new ControlMoveMessage;
}

const ::std::string&
IceProxy::HawkMessages::ControlRoomVisionMessage::ice_staticId()
{
    return ::HawkMessages::ControlRoomVisionMessage::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::HawkMessages::ControlRoomVisionMessage::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::HawkMessages::ControlRoomVisionMessage);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::HawkMessages::ControlRoomVisionMessage::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::HawkMessages::ControlRoomVisionMessage);
}

::IceProxy::Ice::Object*
IceProxy::HawkMessages::ControlRoomVisionMessage::__newInstance() const
{
    return new ControlRoomVisionMessage;
}

const ::std::string&
IceProxy::HawkMessages::ControlTakeOffMessage::ice_staticId()
{
    return ::HawkMessages::ControlTakeOffMessage::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::HawkMessages::ControlTakeOffMessage::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::HawkMessages::ControlTakeOffMessage);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::HawkMessages::ControlTakeOffMessage::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::HawkMessages::ControlTakeOffMessage);
}

::IceProxy::Ice::Object*
IceProxy::HawkMessages::ControlTakeOffMessage::__newInstance() const
{
    return new ControlTakeOffMessage;
}

const ::std::string&
IceProxy::HawkMessages::DriveFinderMessage::ice_staticId()
{
    return ::HawkMessages::DriveFinderMessage::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::HawkMessages::DriveFinderMessage::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::HawkMessages::DriveFinderMessage);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::HawkMessages::DriveFinderMessage::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::HawkMessages::DriveFinderMessage);
}

::IceProxy::Ice::Object*
IceProxy::HawkMessages::DriveFinderMessage::__newInstance() const
{
    return new DriveFinderMessage;
}

const ::std::string&
IceProxy::HawkMessages::ExampleMessage::ice_staticId()
{
    return ::HawkMessages::ExampleMessage::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::HawkMessages::ExampleMessage::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::HawkMessages::ExampleMessage);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::HawkMessages::ExampleMessage::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::HawkMessages::ExampleMessage);
}

::IceProxy::Ice::Object*
IceProxy::HawkMessages::ExampleMessage::__newInstance() const
{
    return new ExampleMessage;
}

const ::std::string&
IceProxy::HawkMessages::ExecuteMissionMessage::ice_staticId()
{
    return ::HawkMessages::ExecuteMissionMessage::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::HawkMessages::ExecuteMissionMessage::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::HawkMessages::ExecuteMissionMessage);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::HawkMessages::ExecuteMissionMessage::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::HawkMessages::ExecuteMissionMessage);
}

::IceProxy::Ice::Object*
IceProxy::HawkMessages::ExecuteMissionMessage::__newInstance() const
{
    return new ExecuteMissionMessage;
}

const ::std::string&
IceProxy::HawkMessages::MissionListMessage::ice_staticId()
{
    return ::HawkMessages::MissionListMessage::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::HawkMessages::MissionListMessage::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::HawkMessages::MissionListMessage);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::HawkMessages::MissionListMessage::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::HawkMessages::MissionListMessage);
}

::IceProxy::Ice::Object*
IceProxy::HawkMessages::MissionListMessage::__newInstance() const
{
    return new MissionListMessage;
}

const ::std::string&
IceProxy::HawkMessages::RoomFinderMessage::ice_staticId()
{
    return ::HawkMessages::RoomFinderMessage::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::HawkMessages::RoomFinderMessage::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::HawkMessages::RoomFinderMessage);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::HawkMessages::RoomFinderMessage::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::HawkMessages::RoomFinderMessage);
}

::IceProxy::Ice::Object*
IceProxy::HawkMessages::RoomFinderMessage::__newInstance() const
{
    return new RoomFinderMessage;
}

const ::std::string&
IceProxy::HawkMessages::SensorDataMessage::ice_staticId()
{
    return ::HawkMessages::SensorDataMessage::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::HawkMessages::SensorDataMessage::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::HawkMessages::SensorDataMessage);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::HawkMessages::SensorDataMessage::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::HawkMessages::SensorDataMessage);
}

::IceProxy::Ice::Object*
IceProxy::HawkMessages::SensorDataMessage::__newInstance() const
{
    return new SensorDataMessage;
}

const ::std::string&
IceProxy::HawkMessages::SlamDataMessage::ice_staticId()
{
    return ::HawkMessages::SlamDataMessage::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::HawkMessages::SlamDataMessage::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::HawkMessages::SlamDataMessage);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::HawkMessages::SlamDataMessage::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::HawkMessages::SlamDataMessage);
}

::IceProxy::Ice::Object*
IceProxy::HawkMessages::SlamDataMessage::__newInstance() const
{
    return new SlamDataMessage;
}

void
IceDelegateM::HawkMessages::MessageAgent::catchMessage(const ::HawkMessages::MessagePtr& msg, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __HawkMessages__MessageAgent__catchMessage_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        __os->write(::Ice::ObjectPtr(::IceInternal::upCast(msg.get())));
        __os->writePendingObjects();
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    if(!__og.is()->b.empty())
    {
        try
        {
            if(!__ok)
            {
                try
                {
                    __og.throwUserException();
                }
                catch(const ::Ice::UserException& __ex)
                {
                    ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                    throw __uue;
                }
            }
            __og.is()->skipEmptyEncaps();
        }
        catch(const ::Ice::LocalException& __ex)
        {
            throw ::IceInternal::LocalExceptionWrapper(__ex, false);
        }
    }
}

void
IceDelegateD::HawkMessages::MessageAgent::catchMessage(const ::HawkMessages::MessagePtr& msg, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::HawkMessages::MessagePtr& msg, const ::Ice::Current& __current) :
            ::IceInternal::Direct(__current),
            _m_msg(msg)
        {
        }

        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::HawkMessages::MessageAgent* servant = dynamic_cast< ::HawkMessages::MessageAgent*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->catchMessage(_m_msg, _current);
            return ::Ice::DispatchOK;
        }

    private:

        const ::HawkMessages::MessagePtr& _m_msg;
    };

    ::Ice::Current __current;
    __initCurrent(__current, __HawkMessages__MessageAgent__catchMessage_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(msg, __current);
        try
        {
            __direct.servant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
}

::Ice::ObjectPtr
HawkMessages::Message::ice_clone() const
{
    ::HawkMessages::MessagePtr __p = new ::HawkMessages::Message(*this);
    return __p;
}

static const ::std::string __HawkMessages__Message_ids[2] =
{
    "::HawkMessages::Message",
    "::Ice::Object"
};

bool
HawkMessages::Message::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__HawkMessages__Message_ids, __HawkMessages__Message_ids + 2, _s);
}

::std::vector< ::std::string>
HawkMessages::Message::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__HawkMessages__Message_ids[0], &__HawkMessages__Message_ids[2]);
}

const ::std::string&
HawkMessages::Message::ice_id(const ::Ice::Current&) const
{
    return __HawkMessages__Message_ids[0];
}

const ::std::string&
HawkMessages::Message::ice_staticId()
{
    return __HawkMessages__Message_ids[0];
}

void
HawkMessages::Message::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
HawkMessages::Message::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
HawkMessages::Message::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type HawkMessages::Message was not generated with stream support";
    throw ex;
}

void
HawkMessages::Message::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type HawkMessages::Message was not generated with stream support";
    throw ex;
}

class __F__HawkMessages__Message : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::HawkMessages::Message::ice_staticId());
        return new ::HawkMessages::Message;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__HawkMessages__Message_Ptr = new __F__HawkMessages__Message;

const ::Ice::ObjectFactoryPtr&
HawkMessages::Message::ice_factory()
{
    return __F__HawkMessages__Message_Ptr;
}

class __F__HawkMessages__Message__Init
{
public:

    __F__HawkMessages__Message__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::HawkMessages::Message::ice_staticId(), ::HawkMessages::Message::ice_factory());
    }

    ~__F__HawkMessages__Message__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::HawkMessages::Message::ice_staticId());
    }
};

static __F__HawkMessages__Message__Init __F__HawkMessages__Message__i;

#ifdef __APPLE__
extern "C" { void __F__HawkMessages__Message__initializer() {} }
#endif

void
HawkMessages::__patch__MessagePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::HawkMessages::MessagePtr* p = static_cast< ::HawkMessages::MessagePtr*>(__addr);
    assert(p);
    *p = ::HawkMessages::MessagePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::HawkMessages::Message::ice_staticId(), v->ice_id());
    }
}

bool
HawkMessages::operator==(const ::HawkMessages::Message& l, const ::HawkMessages::Message& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
HawkMessages::operator<(const ::HawkMessages::Message& l, const ::HawkMessages::Message& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

::Ice::ObjectPtr
HawkMessages::MessageAgent::ice_clone() const
{
    throw ::Ice::CloneNotImplementedException(__FILE__, __LINE__);
    return 0; // to avoid a warning with some compilers
}

static const ::std::string __HawkMessages__MessageAgent_ids[2] =
{
    "::HawkMessages::MessageAgent",
    "::Ice::Object"
};

bool
HawkMessages::MessageAgent::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__HawkMessages__MessageAgent_ids, __HawkMessages__MessageAgent_ids + 2, _s);
}

::std::vector< ::std::string>
HawkMessages::MessageAgent::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__HawkMessages__MessageAgent_ids[0], &__HawkMessages__MessageAgent_ids[2]);
}

const ::std::string&
HawkMessages::MessageAgent::ice_id(const ::Ice::Current&) const
{
    return __HawkMessages__MessageAgent_ids[0];
}

const ::std::string&
HawkMessages::MessageAgent::ice_staticId()
{
    return __HawkMessages__MessageAgent_ids[0];
}

::Ice::DispatchStatus
HawkMessages::MessageAgent::___catchMessage(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::HawkMessages::MessagePtr msg;
    __is->read(::HawkMessages::__patch__MessagePtr, &msg);
    __is->readPendingObjects();
    __is->endReadEncaps();
    catchMessage(msg, __current);
    return ::Ice::DispatchOK;
}

static ::std::string __HawkMessages__MessageAgent_all[] =
{
    "catchMessage",
    "ice_id",
    "ice_ids",
    "ice_isA",
    "ice_ping"
};

::Ice::DispatchStatus
HawkMessages::MessageAgent::__dispatch(::IceInternal::Incoming& in, const ::Ice::Current& current)
{
    ::std::pair< ::std::string*, ::std::string*> r = ::std::equal_range(__HawkMessages__MessageAgent_all, __HawkMessages__MessageAgent_all + 5, current.operation);
    if(r.first == r.second)
    {
        throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
    }

    switch(r.first - __HawkMessages__MessageAgent_all)
    {
        case 0:
        {
            return ___catchMessage(in, current);
        }
        case 1:
        {
            return ___ice_id(in, current);
        }
        case 2:
        {
            return ___ice_ids(in, current);
        }
        case 3:
        {
            return ___ice_isA(in, current);
        }
        case 4:
        {
            return ___ice_ping(in, current);
        }
    }

    assert(false);
    throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
}

void
HawkMessages::MessageAgent::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
HawkMessages::MessageAgent::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
HawkMessages::MessageAgent::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type HawkMessages::MessageAgent was not generated with stream support";
    throw ex;
}

void
HawkMessages::MessageAgent::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type HawkMessages::MessageAgent was not generated with stream support";
    throw ex;
}

void
HawkMessages::__patch__MessageAgentPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::HawkMessages::MessageAgentPtr* p = static_cast< ::HawkMessages::MessageAgentPtr*>(__addr);
    assert(p);
    *p = ::HawkMessages::MessageAgentPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::HawkMessages::MessageAgent::ice_staticId(), v->ice_id());
    }
}

bool
HawkMessages::operator==(const ::HawkMessages::MessageAgent& l, const ::HawkMessages::MessageAgent& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
HawkMessages::operator<(const ::HawkMessages::MessageAgent& l, const ::HawkMessages::MessageAgent& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

HawkMessages::CameraImageMessage::CameraImageMessage(const ::std::string& __ice_cameraID, const ::std::string& __ice_compression, const ::ImageIceMod::ImageIce& __ice_image) :
    cameraID(__ice_cameraID),
    compression(__ice_compression),
    image(__ice_image)
{
}

::Ice::ObjectPtr
HawkMessages::CameraImageMessage::ice_clone() const
{
    ::HawkMessages::CameraImageMessagePtr __p = new ::HawkMessages::CameraImageMessage(*this);
    return __p;
}

static const ::std::string __HawkMessages__CameraImageMessage_ids[3] =
{
    "::HawkMessages::CameraImageMessage",
    "::HawkMessages::Message",
    "::Ice::Object"
};

bool
HawkMessages::CameraImageMessage::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__HawkMessages__CameraImageMessage_ids, __HawkMessages__CameraImageMessage_ids + 3, _s);
}

::std::vector< ::std::string>
HawkMessages::CameraImageMessage::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__HawkMessages__CameraImageMessage_ids[0], &__HawkMessages__CameraImageMessage_ids[3]);
}

const ::std::string&
HawkMessages::CameraImageMessage::ice_id(const ::Ice::Current&) const
{
    return __HawkMessages__CameraImageMessage_ids[0];
}

const ::std::string&
HawkMessages::CameraImageMessage::ice_staticId()
{
    return __HawkMessages__CameraImageMessage_ids[0];
}

void
HawkMessages::CameraImageMessage::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(cameraID);
    __os->write(compression);
    image.__write(__os);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Message::__write(__os);
#else
    ::HawkMessages::Message::__write(__os);
#endif
}

void
HawkMessages::CameraImageMessage::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(cameraID);
    __is->read(compression);
    image.__read(__is);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Message::__read(__is, true);
#else
    ::HawkMessages::Message::__read(__is, true);
#endif
}

void
HawkMessages::CameraImageMessage::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type HawkMessages::CameraImageMessage was not generated with stream support";
    throw ex;
}

void
HawkMessages::CameraImageMessage::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type HawkMessages::CameraImageMessage was not generated with stream support";
    throw ex;
}

class __F__HawkMessages__CameraImageMessage : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::HawkMessages::CameraImageMessage::ice_staticId());
        return new ::HawkMessages::CameraImageMessage;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__HawkMessages__CameraImageMessage_Ptr = new __F__HawkMessages__CameraImageMessage;

const ::Ice::ObjectFactoryPtr&
HawkMessages::CameraImageMessage::ice_factory()
{
    return __F__HawkMessages__CameraImageMessage_Ptr;
}

class __F__HawkMessages__CameraImageMessage__Init
{
public:

    __F__HawkMessages__CameraImageMessage__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::HawkMessages::CameraImageMessage::ice_staticId(), ::HawkMessages::CameraImageMessage::ice_factory());
    }

    ~__F__HawkMessages__CameraImageMessage__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::HawkMessages::CameraImageMessage::ice_staticId());
    }
};

static __F__HawkMessages__CameraImageMessage__Init __F__HawkMessages__CameraImageMessage__i;

#ifdef __APPLE__
extern "C" { void __F__HawkMessages__CameraImageMessage__initializer() {} }
#endif

void
HawkMessages::__patch__CameraImageMessagePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::HawkMessages::CameraImageMessagePtr* p = static_cast< ::HawkMessages::CameraImageMessagePtr*>(__addr);
    assert(p);
    *p = ::HawkMessages::CameraImageMessagePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::HawkMessages::CameraImageMessage::ice_staticId(), v->ice_id());
    }
}

bool
HawkMessages::operator==(const ::HawkMessages::CameraImageMessage& l, const ::HawkMessages::CameraImageMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
HawkMessages::operator<(const ::HawkMessages::CameraImageMessage& l, const ::HawkMessages::CameraImageMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

HawkMessages::ControlCameraMessage::ControlCameraMessage(const ::std::string& __ice_cameraID, const ::std::string& __ice_compression, ::Ice::Int __ice_fps, bool __ice_cameraOn) :
    cameraID(__ice_cameraID),
    compression(__ice_compression),
    fps(__ice_fps),
    cameraOn(__ice_cameraOn)
{
}

::Ice::ObjectPtr
HawkMessages::ControlCameraMessage::ice_clone() const
{
    ::HawkMessages::ControlCameraMessagePtr __p = new ::HawkMessages::ControlCameraMessage(*this);
    return __p;
}

static const ::std::string __HawkMessages__ControlCameraMessage_ids[3] =
{
    "::HawkMessages::ControlCameraMessage",
    "::HawkMessages::Message",
    "::Ice::Object"
};

bool
HawkMessages::ControlCameraMessage::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__HawkMessages__ControlCameraMessage_ids, __HawkMessages__ControlCameraMessage_ids + 3, _s);
}

::std::vector< ::std::string>
HawkMessages::ControlCameraMessage::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__HawkMessages__ControlCameraMessage_ids[0], &__HawkMessages__ControlCameraMessage_ids[3]);
}

const ::std::string&
HawkMessages::ControlCameraMessage::ice_id(const ::Ice::Current&) const
{
    return __HawkMessages__ControlCameraMessage_ids[0];
}

const ::std::string&
HawkMessages::ControlCameraMessage::ice_staticId()
{
    return __HawkMessages__ControlCameraMessage_ids[0];
}

void
HawkMessages::ControlCameraMessage::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(cameraID);
    __os->write(compression);
    __os->write(fps);
    __os->write(cameraOn);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Message::__write(__os);
#else
    ::HawkMessages::Message::__write(__os);
#endif
}

void
HawkMessages::ControlCameraMessage::__read(::IceInternal::BasicStream* __is, bool __rid)
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
    Message::__read(__is, true);
#else
    ::HawkMessages::Message::__read(__is, true);
#endif
}

void
HawkMessages::ControlCameraMessage::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type HawkMessages::ControlCameraMessage was not generated with stream support";
    throw ex;
}

void
HawkMessages::ControlCameraMessage::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type HawkMessages::ControlCameraMessage was not generated with stream support";
    throw ex;
}

class __F__HawkMessages__ControlCameraMessage : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::HawkMessages::ControlCameraMessage::ice_staticId());
        return new ::HawkMessages::ControlCameraMessage;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__HawkMessages__ControlCameraMessage_Ptr = new __F__HawkMessages__ControlCameraMessage;

const ::Ice::ObjectFactoryPtr&
HawkMessages::ControlCameraMessage::ice_factory()
{
    return __F__HawkMessages__ControlCameraMessage_Ptr;
}

class __F__HawkMessages__ControlCameraMessage__Init
{
public:

    __F__HawkMessages__ControlCameraMessage__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::HawkMessages::ControlCameraMessage::ice_staticId(), ::HawkMessages::ControlCameraMessage::ice_factory());
    }

    ~__F__HawkMessages__ControlCameraMessage__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::HawkMessages::ControlCameraMessage::ice_staticId());
    }
};

static __F__HawkMessages__ControlCameraMessage__Init __F__HawkMessages__ControlCameraMessage__i;

#ifdef __APPLE__
extern "C" { void __F__HawkMessages__ControlCameraMessage__initializer() {} }
#endif

void
HawkMessages::__patch__ControlCameraMessagePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::HawkMessages::ControlCameraMessagePtr* p = static_cast< ::HawkMessages::ControlCameraMessagePtr*>(__addr);
    assert(p);
    *p = ::HawkMessages::ControlCameraMessagePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::HawkMessages::ControlCameraMessage::ice_staticId(), v->ice_id());
    }
}

bool
HawkMessages::operator==(const ::HawkMessages::ControlCameraMessage& l, const ::HawkMessages::ControlCameraMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
HawkMessages::operator<(const ::HawkMessages::ControlCameraMessage& l, const ::HawkMessages::ControlCameraMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

HawkMessages::ControlDriveVisionMessage::ControlDriveVisionMessage(bool __ice_driveVisionOn) :
    driveVisionOn(__ice_driveVisionOn)
{
}

::Ice::ObjectPtr
HawkMessages::ControlDriveVisionMessage::ice_clone() const
{
    ::HawkMessages::ControlDriveVisionMessagePtr __p = new ::HawkMessages::ControlDriveVisionMessage(*this);
    return __p;
}

static const ::std::string __HawkMessages__ControlDriveVisionMessage_ids[3] =
{
    "::HawkMessages::ControlDriveVisionMessage",
    "::HawkMessages::Message",
    "::Ice::Object"
};

bool
HawkMessages::ControlDriveVisionMessage::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__HawkMessages__ControlDriveVisionMessage_ids, __HawkMessages__ControlDriveVisionMessage_ids + 3, _s);
}

::std::vector< ::std::string>
HawkMessages::ControlDriveVisionMessage::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__HawkMessages__ControlDriveVisionMessage_ids[0], &__HawkMessages__ControlDriveVisionMessage_ids[3]);
}

const ::std::string&
HawkMessages::ControlDriveVisionMessage::ice_id(const ::Ice::Current&) const
{
    return __HawkMessages__ControlDriveVisionMessage_ids[0];
}

const ::std::string&
HawkMessages::ControlDriveVisionMessage::ice_staticId()
{
    return __HawkMessages__ControlDriveVisionMessage_ids[0];
}

void
HawkMessages::ControlDriveVisionMessage::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(driveVisionOn);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Message::__write(__os);
#else
    ::HawkMessages::Message::__write(__os);
#endif
}

void
HawkMessages::ControlDriveVisionMessage::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(driveVisionOn);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Message::__read(__is, true);
#else
    ::HawkMessages::Message::__read(__is, true);
#endif
}

void
HawkMessages::ControlDriveVisionMessage::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type HawkMessages::ControlDriveVisionMessage was not generated with stream support";
    throw ex;
}

void
HawkMessages::ControlDriveVisionMessage::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type HawkMessages::ControlDriveVisionMessage was not generated with stream support";
    throw ex;
}

class __F__HawkMessages__ControlDriveVisionMessage : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::HawkMessages::ControlDriveVisionMessage::ice_staticId());
        return new ::HawkMessages::ControlDriveVisionMessage;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__HawkMessages__ControlDriveVisionMessage_Ptr = new __F__HawkMessages__ControlDriveVisionMessage;

const ::Ice::ObjectFactoryPtr&
HawkMessages::ControlDriveVisionMessage::ice_factory()
{
    return __F__HawkMessages__ControlDriveVisionMessage_Ptr;
}

class __F__HawkMessages__ControlDriveVisionMessage__Init
{
public:

    __F__HawkMessages__ControlDriveVisionMessage__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::HawkMessages::ControlDriveVisionMessage::ice_staticId(), ::HawkMessages::ControlDriveVisionMessage::ice_factory());
    }

    ~__F__HawkMessages__ControlDriveVisionMessage__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::HawkMessages::ControlDriveVisionMessage::ice_staticId());
    }
};

static __F__HawkMessages__ControlDriveVisionMessage__Init __F__HawkMessages__ControlDriveVisionMessage__i;

#ifdef __APPLE__
extern "C" { void __F__HawkMessages__ControlDriveVisionMessage__initializer() {} }
#endif

void
HawkMessages::__patch__ControlDriveVisionMessagePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::HawkMessages::ControlDriveVisionMessagePtr* p = static_cast< ::HawkMessages::ControlDriveVisionMessagePtr*>(__addr);
    assert(p);
    *p = ::HawkMessages::ControlDriveVisionMessagePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::HawkMessages::ControlDriveVisionMessage::ice_staticId(), v->ice_id());
    }
}

bool
HawkMessages::operator==(const ::HawkMessages::ControlDriveVisionMessage& l, const ::HawkMessages::ControlDriveVisionMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
HawkMessages::operator<(const ::HawkMessages::ControlDriveVisionMessage& l, const ::HawkMessages::ControlDriveVisionMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

::Ice::ObjectPtr
HawkMessages::ControlLandMessage::ice_clone() const
{
    ::HawkMessages::ControlLandMessagePtr __p = new ::HawkMessages::ControlLandMessage(*this);
    return __p;
}

static const ::std::string __HawkMessages__ControlLandMessage_ids[3] =
{
    "::HawkMessages::ControlLandMessage",
    "::HawkMessages::Message",
    "::Ice::Object"
};

bool
HawkMessages::ControlLandMessage::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__HawkMessages__ControlLandMessage_ids, __HawkMessages__ControlLandMessage_ids + 3, _s);
}

::std::vector< ::std::string>
HawkMessages::ControlLandMessage::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__HawkMessages__ControlLandMessage_ids[0], &__HawkMessages__ControlLandMessage_ids[3]);
}

const ::std::string&
HawkMessages::ControlLandMessage::ice_id(const ::Ice::Current&) const
{
    return __HawkMessages__ControlLandMessage_ids[0];
}

const ::std::string&
HawkMessages::ControlLandMessage::ice_staticId()
{
    return __HawkMessages__ControlLandMessage_ids[0];
}

void
HawkMessages::ControlLandMessage::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Message::__write(__os);
#else
    ::HawkMessages::Message::__write(__os);
#endif
}

void
HawkMessages::ControlLandMessage::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Message::__read(__is, true);
#else
    ::HawkMessages::Message::__read(__is, true);
#endif
}

void
HawkMessages::ControlLandMessage::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type HawkMessages::ControlLandMessage was not generated with stream support";
    throw ex;
}

void
HawkMessages::ControlLandMessage::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type HawkMessages::ControlLandMessage was not generated with stream support";
    throw ex;
}

class __F__HawkMessages__ControlLandMessage : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::HawkMessages::ControlLandMessage::ice_staticId());
        return new ::HawkMessages::ControlLandMessage;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__HawkMessages__ControlLandMessage_Ptr = new __F__HawkMessages__ControlLandMessage;

const ::Ice::ObjectFactoryPtr&
HawkMessages::ControlLandMessage::ice_factory()
{
    return __F__HawkMessages__ControlLandMessage_Ptr;
}

class __F__HawkMessages__ControlLandMessage__Init
{
public:

    __F__HawkMessages__ControlLandMessage__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::HawkMessages::ControlLandMessage::ice_staticId(), ::HawkMessages::ControlLandMessage::ice_factory());
    }

    ~__F__HawkMessages__ControlLandMessage__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::HawkMessages::ControlLandMessage::ice_staticId());
    }
};

static __F__HawkMessages__ControlLandMessage__Init __F__HawkMessages__ControlLandMessage__i;

#ifdef __APPLE__
extern "C" { void __F__HawkMessages__ControlLandMessage__initializer() {} }
#endif

void
HawkMessages::__patch__ControlLandMessagePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::HawkMessages::ControlLandMessagePtr* p = static_cast< ::HawkMessages::ControlLandMessagePtr*>(__addr);
    assert(p);
    *p = ::HawkMessages::ControlLandMessagePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::HawkMessages::ControlLandMessage::ice_staticId(), v->ice_id());
    }
}

bool
HawkMessages::operator==(const ::HawkMessages::ControlLandMessage& l, const ::HawkMessages::ControlLandMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
HawkMessages::operator<(const ::HawkMessages::ControlLandMessage& l, const ::HawkMessages::ControlLandMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

HawkMessages::ControlMoveMessage::ControlMoveMessage(const ::HawkMessages::Pose& __ice_move) :
    move(__ice_move)
{
}

::Ice::ObjectPtr
HawkMessages::ControlMoveMessage::ice_clone() const
{
    ::HawkMessages::ControlMoveMessagePtr __p = new ::HawkMessages::ControlMoveMessage(*this);
    return __p;
}

static const ::std::string __HawkMessages__ControlMoveMessage_ids[3] =
{
    "::HawkMessages::ControlMoveMessage",
    "::HawkMessages::Message",
    "::Ice::Object"
};

bool
HawkMessages::ControlMoveMessage::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__HawkMessages__ControlMoveMessage_ids, __HawkMessages__ControlMoveMessage_ids + 3, _s);
}

::std::vector< ::std::string>
HawkMessages::ControlMoveMessage::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__HawkMessages__ControlMoveMessage_ids[0], &__HawkMessages__ControlMoveMessage_ids[3]);
}

const ::std::string&
HawkMessages::ControlMoveMessage::ice_id(const ::Ice::Current&) const
{
    return __HawkMessages__ControlMoveMessage_ids[0];
}

const ::std::string&
HawkMessages::ControlMoveMessage::ice_staticId()
{
    return __HawkMessages__ControlMoveMessage_ids[0];
}

void
HawkMessages::ControlMoveMessage::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    move.__write(__os);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Message::__write(__os);
#else
    ::HawkMessages::Message::__write(__os);
#endif
}

void
HawkMessages::ControlMoveMessage::__read(::IceInternal::BasicStream* __is, bool __rid)
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
    Message::__read(__is, true);
#else
    ::HawkMessages::Message::__read(__is, true);
#endif
}

void
HawkMessages::ControlMoveMessage::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type HawkMessages::ControlMoveMessage was not generated with stream support";
    throw ex;
}

void
HawkMessages::ControlMoveMessage::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type HawkMessages::ControlMoveMessage was not generated with stream support";
    throw ex;
}

class __F__HawkMessages__ControlMoveMessage : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::HawkMessages::ControlMoveMessage::ice_staticId());
        return new ::HawkMessages::ControlMoveMessage;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__HawkMessages__ControlMoveMessage_Ptr = new __F__HawkMessages__ControlMoveMessage;

const ::Ice::ObjectFactoryPtr&
HawkMessages::ControlMoveMessage::ice_factory()
{
    return __F__HawkMessages__ControlMoveMessage_Ptr;
}

class __F__HawkMessages__ControlMoveMessage__Init
{
public:

    __F__HawkMessages__ControlMoveMessage__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::HawkMessages::ControlMoveMessage::ice_staticId(), ::HawkMessages::ControlMoveMessage::ice_factory());
    }

    ~__F__HawkMessages__ControlMoveMessage__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::HawkMessages::ControlMoveMessage::ice_staticId());
    }
};

static __F__HawkMessages__ControlMoveMessage__Init __F__HawkMessages__ControlMoveMessage__i;

#ifdef __APPLE__
extern "C" { void __F__HawkMessages__ControlMoveMessage__initializer() {} }
#endif

void
HawkMessages::__patch__ControlMoveMessagePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::HawkMessages::ControlMoveMessagePtr* p = static_cast< ::HawkMessages::ControlMoveMessagePtr*>(__addr);
    assert(p);
    *p = ::HawkMessages::ControlMoveMessagePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::HawkMessages::ControlMoveMessage::ice_staticId(), v->ice_id());
    }
}

bool
HawkMessages::operator==(const ::HawkMessages::ControlMoveMessage& l, const ::HawkMessages::ControlMoveMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
HawkMessages::operator<(const ::HawkMessages::ControlMoveMessage& l, const ::HawkMessages::ControlMoveMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

HawkMessages::ControlRoomVisionMessage::ControlRoomVisionMessage(bool __ice_roomVisionOn) :
    roomVisionOn(__ice_roomVisionOn)
{
}

::Ice::ObjectPtr
HawkMessages::ControlRoomVisionMessage::ice_clone() const
{
    ::HawkMessages::ControlRoomVisionMessagePtr __p = new ::HawkMessages::ControlRoomVisionMessage(*this);
    return __p;
}

static const ::std::string __HawkMessages__ControlRoomVisionMessage_ids[3] =
{
    "::HawkMessages::ControlRoomVisionMessage",
    "::HawkMessages::Message",
    "::Ice::Object"
};

bool
HawkMessages::ControlRoomVisionMessage::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__HawkMessages__ControlRoomVisionMessage_ids, __HawkMessages__ControlRoomVisionMessage_ids + 3, _s);
}

::std::vector< ::std::string>
HawkMessages::ControlRoomVisionMessage::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__HawkMessages__ControlRoomVisionMessage_ids[0], &__HawkMessages__ControlRoomVisionMessage_ids[3]);
}

const ::std::string&
HawkMessages::ControlRoomVisionMessage::ice_id(const ::Ice::Current&) const
{
    return __HawkMessages__ControlRoomVisionMessage_ids[0];
}

const ::std::string&
HawkMessages::ControlRoomVisionMessage::ice_staticId()
{
    return __HawkMessages__ControlRoomVisionMessage_ids[0];
}

void
HawkMessages::ControlRoomVisionMessage::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(roomVisionOn);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Message::__write(__os);
#else
    ::HawkMessages::Message::__write(__os);
#endif
}

void
HawkMessages::ControlRoomVisionMessage::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(roomVisionOn);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Message::__read(__is, true);
#else
    ::HawkMessages::Message::__read(__is, true);
#endif
}

void
HawkMessages::ControlRoomVisionMessage::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type HawkMessages::ControlRoomVisionMessage was not generated with stream support";
    throw ex;
}

void
HawkMessages::ControlRoomVisionMessage::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type HawkMessages::ControlRoomVisionMessage was not generated with stream support";
    throw ex;
}

class __F__HawkMessages__ControlRoomVisionMessage : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::HawkMessages::ControlRoomVisionMessage::ice_staticId());
        return new ::HawkMessages::ControlRoomVisionMessage;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__HawkMessages__ControlRoomVisionMessage_Ptr = new __F__HawkMessages__ControlRoomVisionMessage;

const ::Ice::ObjectFactoryPtr&
HawkMessages::ControlRoomVisionMessage::ice_factory()
{
    return __F__HawkMessages__ControlRoomVisionMessage_Ptr;
}

class __F__HawkMessages__ControlRoomVisionMessage__Init
{
public:

    __F__HawkMessages__ControlRoomVisionMessage__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::HawkMessages::ControlRoomVisionMessage::ice_staticId(), ::HawkMessages::ControlRoomVisionMessage::ice_factory());
    }

    ~__F__HawkMessages__ControlRoomVisionMessage__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::HawkMessages::ControlRoomVisionMessage::ice_staticId());
    }
};

static __F__HawkMessages__ControlRoomVisionMessage__Init __F__HawkMessages__ControlRoomVisionMessage__i;

#ifdef __APPLE__
extern "C" { void __F__HawkMessages__ControlRoomVisionMessage__initializer() {} }
#endif

void
HawkMessages::__patch__ControlRoomVisionMessagePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::HawkMessages::ControlRoomVisionMessagePtr* p = static_cast< ::HawkMessages::ControlRoomVisionMessagePtr*>(__addr);
    assert(p);
    *p = ::HawkMessages::ControlRoomVisionMessagePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::HawkMessages::ControlRoomVisionMessage::ice_staticId(), v->ice_id());
    }
}

bool
HawkMessages::operator==(const ::HawkMessages::ControlRoomVisionMessage& l, const ::HawkMessages::ControlRoomVisionMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
HawkMessages::operator<(const ::HawkMessages::ControlRoomVisionMessage& l, const ::HawkMessages::ControlRoomVisionMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

HawkMessages::ControlTakeOffMessage::ControlTakeOffMessage(::Ice::Double __ice_altitude) :
    altitude(__ice_altitude)
{
}

::Ice::ObjectPtr
HawkMessages::ControlTakeOffMessage::ice_clone() const
{
    ::HawkMessages::ControlTakeOffMessagePtr __p = new ::HawkMessages::ControlTakeOffMessage(*this);
    return __p;
}

static const ::std::string __HawkMessages__ControlTakeOffMessage_ids[3] =
{
    "::HawkMessages::ControlTakeOffMessage",
    "::HawkMessages::Message",
    "::Ice::Object"
};

bool
HawkMessages::ControlTakeOffMessage::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__HawkMessages__ControlTakeOffMessage_ids, __HawkMessages__ControlTakeOffMessage_ids + 3, _s);
}

::std::vector< ::std::string>
HawkMessages::ControlTakeOffMessage::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__HawkMessages__ControlTakeOffMessage_ids[0], &__HawkMessages__ControlTakeOffMessage_ids[3]);
}

const ::std::string&
HawkMessages::ControlTakeOffMessage::ice_id(const ::Ice::Current&) const
{
    return __HawkMessages__ControlTakeOffMessage_ids[0];
}

const ::std::string&
HawkMessages::ControlTakeOffMessage::ice_staticId()
{
    return __HawkMessages__ControlTakeOffMessage_ids[0];
}

void
HawkMessages::ControlTakeOffMessage::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(altitude);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Message::__write(__os);
#else
    ::HawkMessages::Message::__write(__os);
#endif
}

void
HawkMessages::ControlTakeOffMessage::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(altitude);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Message::__read(__is, true);
#else
    ::HawkMessages::Message::__read(__is, true);
#endif
}

void
HawkMessages::ControlTakeOffMessage::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type HawkMessages::ControlTakeOffMessage was not generated with stream support";
    throw ex;
}

void
HawkMessages::ControlTakeOffMessage::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type HawkMessages::ControlTakeOffMessage was not generated with stream support";
    throw ex;
}

class __F__HawkMessages__ControlTakeOffMessage : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::HawkMessages::ControlTakeOffMessage::ice_staticId());
        return new ::HawkMessages::ControlTakeOffMessage;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__HawkMessages__ControlTakeOffMessage_Ptr = new __F__HawkMessages__ControlTakeOffMessage;

const ::Ice::ObjectFactoryPtr&
HawkMessages::ControlTakeOffMessage::ice_factory()
{
    return __F__HawkMessages__ControlTakeOffMessage_Ptr;
}

class __F__HawkMessages__ControlTakeOffMessage__Init
{
public:

    __F__HawkMessages__ControlTakeOffMessage__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::HawkMessages::ControlTakeOffMessage::ice_staticId(), ::HawkMessages::ControlTakeOffMessage::ice_factory());
    }

    ~__F__HawkMessages__ControlTakeOffMessage__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::HawkMessages::ControlTakeOffMessage::ice_staticId());
    }
};

static __F__HawkMessages__ControlTakeOffMessage__Init __F__HawkMessages__ControlTakeOffMessage__i;

#ifdef __APPLE__
extern "C" { void __F__HawkMessages__ControlTakeOffMessage__initializer() {} }
#endif

void
HawkMessages::__patch__ControlTakeOffMessagePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::HawkMessages::ControlTakeOffMessagePtr* p = static_cast< ::HawkMessages::ControlTakeOffMessagePtr*>(__addr);
    assert(p);
    *p = ::HawkMessages::ControlTakeOffMessagePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::HawkMessages::ControlTakeOffMessage::ice_staticId(), v->ice_id());
    }
}

bool
HawkMessages::operator==(const ::HawkMessages::ControlTakeOffMessage& l, const ::HawkMessages::ControlTakeOffMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
HawkMessages::operator<(const ::HawkMessages::ControlTakeOffMessage& l, const ::HawkMessages::ControlTakeOffMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

HawkMessages::DriveFinderMessage::DriveFinderMessage(bool __ice_driveFound, const ::HawkMessages::Pose& __ice_drivePose) :
    driveFound(__ice_driveFound),
    drivePose(__ice_drivePose)
{
}

::Ice::ObjectPtr
HawkMessages::DriveFinderMessage::ice_clone() const
{
    ::HawkMessages::DriveFinderMessagePtr __p = new ::HawkMessages::DriveFinderMessage(*this);
    return __p;
}

static const ::std::string __HawkMessages__DriveFinderMessage_ids[3] =
{
    "::HawkMessages::DriveFinderMessage",
    "::HawkMessages::Message",
    "::Ice::Object"
};

bool
HawkMessages::DriveFinderMessage::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__HawkMessages__DriveFinderMessage_ids, __HawkMessages__DriveFinderMessage_ids + 3, _s);
}

::std::vector< ::std::string>
HawkMessages::DriveFinderMessage::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__HawkMessages__DriveFinderMessage_ids[0], &__HawkMessages__DriveFinderMessage_ids[3]);
}

const ::std::string&
HawkMessages::DriveFinderMessage::ice_id(const ::Ice::Current&) const
{
    return __HawkMessages__DriveFinderMessage_ids[0];
}

const ::std::string&
HawkMessages::DriveFinderMessage::ice_staticId()
{
    return __HawkMessages__DriveFinderMessage_ids[0];
}

void
HawkMessages::DriveFinderMessage::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(driveFound);
    drivePose.__write(__os);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Message::__write(__os);
#else
    ::HawkMessages::Message::__write(__os);
#endif
}

void
HawkMessages::DriveFinderMessage::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(driveFound);
    drivePose.__read(__is);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Message::__read(__is, true);
#else
    ::HawkMessages::Message::__read(__is, true);
#endif
}

void
HawkMessages::DriveFinderMessage::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type HawkMessages::DriveFinderMessage was not generated with stream support";
    throw ex;
}

void
HawkMessages::DriveFinderMessage::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type HawkMessages::DriveFinderMessage was not generated with stream support";
    throw ex;
}

class __F__HawkMessages__DriveFinderMessage : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::HawkMessages::DriveFinderMessage::ice_staticId());
        return new ::HawkMessages::DriveFinderMessage;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__HawkMessages__DriveFinderMessage_Ptr = new __F__HawkMessages__DriveFinderMessage;

const ::Ice::ObjectFactoryPtr&
HawkMessages::DriveFinderMessage::ice_factory()
{
    return __F__HawkMessages__DriveFinderMessage_Ptr;
}

class __F__HawkMessages__DriveFinderMessage__Init
{
public:

    __F__HawkMessages__DriveFinderMessage__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::HawkMessages::DriveFinderMessage::ice_staticId(), ::HawkMessages::DriveFinderMessage::ice_factory());
    }

    ~__F__HawkMessages__DriveFinderMessage__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::HawkMessages::DriveFinderMessage::ice_staticId());
    }
};

static __F__HawkMessages__DriveFinderMessage__Init __F__HawkMessages__DriveFinderMessage__i;

#ifdef __APPLE__
extern "C" { void __F__HawkMessages__DriveFinderMessage__initializer() {} }
#endif

void
HawkMessages::__patch__DriveFinderMessagePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::HawkMessages::DriveFinderMessagePtr* p = static_cast< ::HawkMessages::DriveFinderMessagePtr*>(__addr);
    assert(p);
    *p = ::HawkMessages::DriveFinderMessagePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::HawkMessages::DriveFinderMessage::ice_staticId(), v->ice_id());
    }
}

bool
HawkMessages::operator==(const ::HawkMessages::DriveFinderMessage& l, const ::HawkMessages::DriveFinderMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
HawkMessages::operator<(const ::HawkMessages::DriveFinderMessage& l, const ::HawkMessages::DriveFinderMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

HawkMessages::ExampleMessage::ExampleMessage(const ::std::string& __ice_name, const ::std::string& __ice_chatter) :
    name(__ice_name),
    chatter(__ice_chatter)
{
}

::Ice::ObjectPtr
HawkMessages::ExampleMessage::ice_clone() const
{
    ::HawkMessages::ExampleMessagePtr __p = new ::HawkMessages::ExampleMessage(*this);
    return __p;
}

static const ::std::string __HawkMessages__ExampleMessage_ids[3] =
{
    "::HawkMessages::ExampleMessage",
    "::HawkMessages::Message",
    "::Ice::Object"
};

bool
HawkMessages::ExampleMessage::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__HawkMessages__ExampleMessage_ids, __HawkMessages__ExampleMessage_ids + 3, _s);
}

::std::vector< ::std::string>
HawkMessages::ExampleMessage::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__HawkMessages__ExampleMessage_ids[0], &__HawkMessages__ExampleMessage_ids[3]);
}

const ::std::string&
HawkMessages::ExampleMessage::ice_id(const ::Ice::Current&) const
{
    return __HawkMessages__ExampleMessage_ids[0];
}

const ::std::string&
HawkMessages::ExampleMessage::ice_staticId()
{
    return __HawkMessages__ExampleMessage_ids[0];
}

void
HawkMessages::ExampleMessage::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(name);
    __os->write(chatter);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Message::__write(__os);
#else
    ::HawkMessages::Message::__write(__os);
#endif
}

void
HawkMessages::ExampleMessage::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(name);
    __is->read(chatter);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Message::__read(__is, true);
#else
    ::HawkMessages::Message::__read(__is, true);
#endif
}

void
HawkMessages::ExampleMessage::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type HawkMessages::ExampleMessage was not generated with stream support";
    throw ex;
}

void
HawkMessages::ExampleMessage::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type HawkMessages::ExampleMessage was not generated with stream support";
    throw ex;
}

class __F__HawkMessages__ExampleMessage : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::HawkMessages::ExampleMessage::ice_staticId());
        return new ::HawkMessages::ExampleMessage;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__HawkMessages__ExampleMessage_Ptr = new __F__HawkMessages__ExampleMessage;

const ::Ice::ObjectFactoryPtr&
HawkMessages::ExampleMessage::ice_factory()
{
    return __F__HawkMessages__ExampleMessage_Ptr;
}

class __F__HawkMessages__ExampleMessage__Init
{
public:

    __F__HawkMessages__ExampleMessage__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::HawkMessages::ExampleMessage::ice_staticId(), ::HawkMessages::ExampleMessage::ice_factory());
    }

    ~__F__HawkMessages__ExampleMessage__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::HawkMessages::ExampleMessage::ice_staticId());
    }
};

static __F__HawkMessages__ExampleMessage__Init __F__HawkMessages__ExampleMessage__i;

#ifdef __APPLE__
extern "C" { void __F__HawkMessages__ExampleMessage__initializer() {} }
#endif

void
HawkMessages::__patch__ExampleMessagePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::HawkMessages::ExampleMessagePtr* p = static_cast< ::HawkMessages::ExampleMessagePtr*>(__addr);
    assert(p);
    *p = ::HawkMessages::ExampleMessagePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::HawkMessages::ExampleMessage::ice_staticId(), v->ice_id());
    }
}

bool
HawkMessages::operator==(const ::HawkMessages::ExampleMessage& l, const ::HawkMessages::ExampleMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
HawkMessages::operator<(const ::HawkMessages::ExampleMessage& l, const ::HawkMessages::ExampleMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

HawkMessages::ExecuteMissionMessage::ExecuteMissionMessage(const ::std::string& __ice_mission) :
    mission(__ice_mission)
{
}

::Ice::ObjectPtr
HawkMessages::ExecuteMissionMessage::ice_clone() const
{
    ::HawkMessages::ExecuteMissionMessagePtr __p = new ::HawkMessages::ExecuteMissionMessage(*this);
    return __p;
}

static const ::std::string __HawkMessages__ExecuteMissionMessage_ids[3] =
{
    "::HawkMessages::ExecuteMissionMessage",
    "::HawkMessages::Message",
    "::Ice::Object"
};

bool
HawkMessages::ExecuteMissionMessage::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__HawkMessages__ExecuteMissionMessage_ids, __HawkMessages__ExecuteMissionMessage_ids + 3, _s);
}

::std::vector< ::std::string>
HawkMessages::ExecuteMissionMessage::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__HawkMessages__ExecuteMissionMessage_ids[0], &__HawkMessages__ExecuteMissionMessage_ids[3]);
}

const ::std::string&
HawkMessages::ExecuteMissionMessage::ice_id(const ::Ice::Current&) const
{
    return __HawkMessages__ExecuteMissionMessage_ids[0];
}

const ::std::string&
HawkMessages::ExecuteMissionMessage::ice_staticId()
{
    return __HawkMessages__ExecuteMissionMessage_ids[0];
}

void
HawkMessages::ExecuteMissionMessage::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(mission);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Message::__write(__os);
#else
    ::HawkMessages::Message::__write(__os);
#endif
}

void
HawkMessages::ExecuteMissionMessage::__read(::IceInternal::BasicStream* __is, bool __rid)
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
    Message::__read(__is, true);
#else
    ::HawkMessages::Message::__read(__is, true);
#endif
}

void
HawkMessages::ExecuteMissionMessage::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type HawkMessages::ExecuteMissionMessage was not generated with stream support";
    throw ex;
}

void
HawkMessages::ExecuteMissionMessage::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type HawkMessages::ExecuteMissionMessage was not generated with stream support";
    throw ex;
}

class __F__HawkMessages__ExecuteMissionMessage : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::HawkMessages::ExecuteMissionMessage::ice_staticId());
        return new ::HawkMessages::ExecuteMissionMessage;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__HawkMessages__ExecuteMissionMessage_Ptr = new __F__HawkMessages__ExecuteMissionMessage;

const ::Ice::ObjectFactoryPtr&
HawkMessages::ExecuteMissionMessage::ice_factory()
{
    return __F__HawkMessages__ExecuteMissionMessage_Ptr;
}

class __F__HawkMessages__ExecuteMissionMessage__Init
{
public:

    __F__HawkMessages__ExecuteMissionMessage__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::HawkMessages::ExecuteMissionMessage::ice_staticId(), ::HawkMessages::ExecuteMissionMessage::ice_factory());
    }

    ~__F__HawkMessages__ExecuteMissionMessage__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::HawkMessages::ExecuteMissionMessage::ice_staticId());
    }
};

static __F__HawkMessages__ExecuteMissionMessage__Init __F__HawkMessages__ExecuteMissionMessage__i;

#ifdef __APPLE__
extern "C" { void __F__HawkMessages__ExecuteMissionMessage__initializer() {} }
#endif

void
HawkMessages::__patch__ExecuteMissionMessagePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::HawkMessages::ExecuteMissionMessagePtr* p = static_cast< ::HawkMessages::ExecuteMissionMessagePtr*>(__addr);
    assert(p);
    *p = ::HawkMessages::ExecuteMissionMessagePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::HawkMessages::ExecuteMissionMessage::ice_staticId(), v->ice_id());
    }
}

bool
HawkMessages::operator==(const ::HawkMessages::ExecuteMissionMessage& l, const ::HawkMessages::ExecuteMissionMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
HawkMessages::operator<(const ::HawkMessages::ExecuteMissionMessage& l, const ::HawkMessages::ExecuteMissionMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

HawkMessages::MissionListMessage::MissionListMessage(const ::HawkMessages::StringSeq& __ice_missions) :
    missions(__ice_missions)
{
}

::Ice::ObjectPtr
HawkMessages::MissionListMessage::ice_clone() const
{
    ::HawkMessages::MissionListMessagePtr __p = new ::HawkMessages::MissionListMessage(*this);
    return __p;
}

static const ::std::string __HawkMessages__MissionListMessage_ids[3] =
{
    "::HawkMessages::Message",
    "::HawkMessages::MissionListMessage",
    "::Ice::Object"
};

bool
HawkMessages::MissionListMessage::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__HawkMessages__MissionListMessage_ids, __HawkMessages__MissionListMessage_ids + 3, _s);
}

::std::vector< ::std::string>
HawkMessages::MissionListMessage::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__HawkMessages__MissionListMessage_ids[0], &__HawkMessages__MissionListMessage_ids[3]);
}

const ::std::string&
HawkMessages::MissionListMessage::ice_id(const ::Ice::Current&) const
{
    return __HawkMessages__MissionListMessage_ids[1];
}

const ::std::string&
HawkMessages::MissionListMessage::ice_staticId()
{
    return __HawkMessages__MissionListMessage_ids[1];
}

void
HawkMessages::MissionListMessage::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    if(missions.size() == 0)
    {
        __os->writeSize(0);
    }
    else
    {
        __os->write(&missions[0], &missions[0] + missions.size());
    }
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Message::__write(__os);
#else
    ::HawkMessages::Message::__write(__os);
#endif
}

void
HawkMessages::MissionListMessage::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(missions);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Message::__read(__is, true);
#else
    ::HawkMessages::Message::__read(__is, true);
#endif
}

void
HawkMessages::MissionListMessage::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type HawkMessages::MissionListMessage was not generated with stream support";
    throw ex;
}

void
HawkMessages::MissionListMessage::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type HawkMessages::MissionListMessage was not generated with stream support";
    throw ex;
}

class __F__HawkMessages__MissionListMessage : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::HawkMessages::MissionListMessage::ice_staticId());
        return new ::HawkMessages::MissionListMessage;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__HawkMessages__MissionListMessage_Ptr = new __F__HawkMessages__MissionListMessage;

const ::Ice::ObjectFactoryPtr&
HawkMessages::MissionListMessage::ice_factory()
{
    return __F__HawkMessages__MissionListMessage_Ptr;
}

class __F__HawkMessages__MissionListMessage__Init
{
public:

    __F__HawkMessages__MissionListMessage__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::HawkMessages::MissionListMessage::ice_staticId(), ::HawkMessages::MissionListMessage::ice_factory());
    }

    ~__F__HawkMessages__MissionListMessage__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::HawkMessages::MissionListMessage::ice_staticId());
    }
};

static __F__HawkMessages__MissionListMessage__Init __F__HawkMessages__MissionListMessage__i;

#ifdef __APPLE__
extern "C" { void __F__HawkMessages__MissionListMessage__initializer() {} }
#endif

void
HawkMessages::__patch__MissionListMessagePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::HawkMessages::MissionListMessagePtr* p = static_cast< ::HawkMessages::MissionListMessagePtr*>(__addr);
    assert(p);
    *p = ::HawkMessages::MissionListMessagePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::HawkMessages::MissionListMessage::ice_staticId(), v->ice_id());
    }
}

bool
HawkMessages::operator==(const ::HawkMessages::MissionListMessage& l, const ::HawkMessages::MissionListMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
HawkMessages::operator<(const ::HawkMessages::MissionListMessage& l, const ::HawkMessages::MissionListMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

HawkMessages::RoomFinderMessage::RoomFinderMessage(bool __ice_roomFound, const ::HawkMessages::Pose& __ice_roomPose) :
    roomFound(__ice_roomFound),
    roomPose(__ice_roomPose)
{
}

::Ice::ObjectPtr
HawkMessages::RoomFinderMessage::ice_clone() const
{
    ::HawkMessages::RoomFinderMessagePtr __p = new ::HawkMessages::RoomFinderMessage(*this);
    return __p;
}

static const ::std::string __HawkMessages__RoomFinderMessage_ids[3] =
{
    "::HawkMessages::Message",
    "::HawkMessages::RoomFinderMessage",
    "::Ice::Object"
};

bool
HawkMessages::RoomFinderMessage::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__HawkMessages__RoomFinderMessage_ids, __HawkMessages__RoomFinderMessage_ids + 3, _s);
}

::std::vector< ::std::string>
HawkMessages::RoomFinderMessage::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__HawkMessages__RoomFinderMessage_ids[0], &__HawkMessages__RoomFinderMessage_ids[3]);
}

const ::std::string&
HawkMessages::RoomFinderMessage::ice_id(const ::Ice::Current&) const
{
    return __HawkMessages__RoomFinderMessage_ids[1];
}

const ::std::string&
HawkMessages::RoomFinderMessage::ice_staticId()
{
    return __HawkMessages__RoomFinderMessage_ids[1];
}

void
HawkMessages::RoomFinderMessage::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(roomFound);
    roomPose.__write(__os);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Message::__write(__os);
#else
    ::HawkMessages::Message::__write(__os);
#endif
}

void
HawkMessages::RoomFinderMessage::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(roomFound);
    roomPose.__read(__is);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Message::__read(__is, true);
#else
    ::HawkMessages::Message::__read(__is, true);
#endif
}

void
HawkMessages::RoomFinderMessage::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type HawkMessages::RoomFinderMessage was not generated with stream support";
    throw ex;
}

void
HawkMessages::RoomFinderMessage::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type HawkMessages::RoomFinderMessage was not generated with stream support";
    throw ex;
}

class __F__HawkMessages__RoomFinderMessage : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::HawkMessages::RoomFinderMessage::ice_staticId());
        return new ::HawkMessages::RoomFinderMessage;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__HawkMessages__RoomFinderMessage_Ptr = new __F__HawkMessages__RoomFinderMessage;

const ::Ice::ObjectFactoryPtr&
HawkMessages::RoomFinderMessage::ice_factory()
{
    return __F__HawkMessages__RoomFinderMessage_Ptr;
}

class __F__HawkMessages__RoomFinderMessage__Init
{
public:

    __F__HawkMessages__RoomFinderMessage__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::HawkMessages::RoomFinderMessage::ice_staticId(), ::HawkMessages::RoomFinderMessage::ice_factory());
    }

    ~__F__HawkMessages__RoomFinderMessage__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::HawkMessages::RoomFinderMessage::ice_staticId());
    }
};

static __F__HawkMessages__RoomFinderMessage__Init __F__HawkMessages__RoomFinderMessage__i;

#ifdef __APPLE__
extern "C" { void __F__HawkMessages__RoomFinderMessage__initializer() {} }
#endif

void
HawkMessages::__patch__RoomFinderMessagePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::HawkMessages::RoomFinderMessagePtr* p = static_cast< ::HawkMessages::RoomFinderMessagePtr*>(__addr);
    assert(p);
    *p = ::HawkMessages::RoomFinderMessagePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::HawkMessages::RoomFinderMessage::ice_staticId(), v->ice_id());
    }
}

bool
HawkMessages::operator==(const ::HawkMessages::RoomFinderMessage& l, const ::HawkMessages::RoomFinderMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
HawkMessages::operator<(const ::HawkMessages::RoomFinderMessage& l, const ::HawkMessages::RoomFinderMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

HawkMessages::SensorDataMessage::SensorDataMessage(const ::HawkMessages::Pose& __ice_attemptedMove, ::Ice::Double __ice_motorSpeeds, ::Ice::Double __ice_batterVoltage, bool __ice_validAccel, ::Ice::Double __ice_accelX, ::Ice::Double __ice_accelY, ::Ice::Double __ice_accelZ, bool __ice_validGyro, ::Ice::Double __ice_gyroX, ::Ice::Double __ice_gyroY, ::Ice::Double __ice_gyroZ, bool __ice_validMag, ::Ice::Double __ice_magX, ::Ice::Double __ice_magY, ::Ice::Double __ice_magZ, bool __ice_validAhr, ::Ice::Double __ice_roll, ::Ice::Double __ice_pitch, ::Ice::Double __ice_yaw, ::Ice::Double __ice_heading, ::Ice::Double __ice_angularResolution, const ::HawkMessages::LongSeq& __ice_scannerData, const ::HawkMessages::SonarSeq& __ice_sonarData) :
    attemptedMove(__ice_attemptedMove),
    motorSpeeds(__ice_motorSpeeds),
    batterVoltage(__ice_batterVoltage),
    validAccel(__ice_validAccel),
    accelX(__ice_accelX),
    accelY(__ice_accelY),
    accelZ(__ice_accelZ),
    validGyro(__ice_validGyro),
    gyroX(__ice_gyroX),
    gyroY(__ice_gyroY),
    gyroZ(__ice_gyroZ),
    validMag(__ice_validMag),
    magX(__ice_magX),
    magY(__ice_magY),
    magZ(__ice_magZ),
    validAhr(__ice_validAhr),
    roll(__ice_roll),
    pitch(__ice_pitch),
    yaw(__ice_yaw),
    heading(__ice_heading),
    angularResolution(__ice_angularResolution),
    scannerData(__ice_scannerData),
    sonarData(__ice_sonarData)
{
}

::Ice::ObjectPtr
HawkMessages::SensorDataMessage::ice_clone() const
{
    ::HawkMessages::SensorDataMessagePtr __p = new ::HawkMessages::SensorDataMessage(*this);
    return __p;
}

static const ::std::string __HawkMessages__SensorDataMessage_ids[3] =
{
    "::HawkMessages::Message",
    "::HawkMessages::SensorDataMessage",
    "::Ice::Object"
};

bool
HawkMessages::SensorDataMessage::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__HawkMessages__SensorDataMessage_ids, __HawkMessages__SensorDataMessage_ids + 3, _s);
}

::std::vector< ::std::string>
HawkMessages::SensorDataMessage::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__HawkMessages__SensorDataMessage_ids[0], &__HawkMessages__SensorDataMessage_ids[3]);
}

const ::std::string&
HawkMessages::SensorDataMessage::ice_id(const ::Ice::Current&) const
{
    return __HawkMessages__SensorDataMessage_ids[1];
}

const ::std::string&
HawkMessages::SensorDataMessage::ice_staticId()
{
    return __HawkMessages__SensorDataMessage_ids[1];
}

void
HawkMessages::SensorDataMessage::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    attemptedMove.__write(__os);
    __os->write(motorSpeeds);
    __os->write(batterVoltage);
    __os->write(validAccel);
    __os->write(accelX);
    __os->write(accelY);
    __os->write(accelZ);
    __os->write(validGyro);
    __os->write(gyroX);
    __os->write(gyroY);
    __os->write(gyroZ);
    __os->write(validMag);
    __os->write(magX);
    __os->write(magY);
    __os->write(magZ);
    __os->write(validAhr);
    __os->write(roll);
    __os->write(pitch);
    __os->write(yaw);
    __os->write(heading);
    __os->write(angularResolution);
    if(scannerData.size() == 0)
    {
        __os->writeSize(0);
    }
    else
    {
        __os->write(&scannerData[0], &scannerData[0] + scannerData.size());
    }
    if(sonarData.size() == 0)
    {
        __os->writeSize(0);
    }
    else
    {
        ::HawkMessages::__writeSonarSeq(__os, &sonarData[0], &sonarData[0] + sonarData.size());
    }
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Message::__write(__os);
#else
    ::HawkMessages::Message::__write(__os);
#endif
}

void
HawkMessages::SensorDataMessage::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    attemptedMove.__read(__is);
    __is->read(motorSpeeds);
    __is->read(batterVoltage);
    __is->read(validAccel);
    __is->read(accelX);
    __is->read(accelY);
    __is->read(accelZ);
    __is->read(validGyro);
    __is->read(gyroX);
    __is->read(gyroY);
    __is->read(gyroZ);
    __is->read(validMag);
    __is->read(magX);
    __is->read(magY);
    __is->read(magZ);
    __is->read(validAhr);
    __is->read(roll);
    __is->read(pitch);
    __is->read(yaw);
    __is->read(heading);
    __is->read(angularResolution);
    __is->read(scannerData);
    ::HawkMessages::__readSonarSeq(__is, sonarData);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Message::__read(__is, true);
#else
    ::HawkMessages::Message::__read(__is, true);
#endif
}

void
HawkMessages::SensorDataMessage::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type HawkMessages::SensorDataMessage was not generated with stream support";
    throw ex;
}

void
HawkMessages::SensorDataMessage::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type HawkMessages::SensorDataMessage was not generated with stream support";
    throw ex;
}

class __F__HawkMessages__SensorDataMessage : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::HawkMessages::SensorDataMessage::ice_staticId());
        return new ::HawkMessages::SensorDataMessage;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__HawkMessages__SensorDataMessage_Ptr = new __F__HawkMessages__SensorDataMessage;

const ::Ice::ObjectFactoryPtr&
HawkMessages::SensorDataMessage::ice_factory()
{
    return __F__HawkMessages__SensorDataMessage_Ptr;
}

class __F__HawkMessages__SensorDataMessage__Init
{
public:

    __F__HawkMessages__SensorDataMessage__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::HawkMessages::SensorDataMessage::ice_staticId(), ::HawkMessages::SensorDataMessage::ice_factory());
    }

    ~__F__HawkMessages__SensorDataMessage__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::HawkMessages::SensorDataMessage::ice_staticId());
    }
};

static __F__HawkMessages__SensorDataMessage__Init __F__HawkMessages__SensorDataMessage__i;

#ifdef __APPLE__
extern "C" { void __F__HawkMessages__SensorDataMessage__initializer() {} }
#endif

void
HawkMessages::__patch__SensorDataMessagePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::HawkMessages::SensorDataMessagePtr* p = static_cast< ::HawkMessages::SensorDataMessagePtr*>(__addr);
    assert(p);
    *p = ::HawkMessages::SensorDataMessagePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::HawkMessages::SensorDataMessage::ice_staticId(), v->ice_id());
    }
}

bool
HawkMessages::operator==(const ::HawkMessages::SensorDataMessage& l, const ::HawkMessages::SensorDataMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
HawkMessages::operator<(const ::HawkMessages::SensorDataMessage& l, const ::HawkMessages::SensorDataMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

HawkMessages::SlamDataMessage::SlamDataMessage(const ::HawkMessages::Pose& __ice_hawkPose) :
    hawkPose(__ice_hawkPose)
{
}

::Ice::ObjectPtr
HawkMessages::SlamDataMessage::ice_clone() const
{
    ::HawkMessages::SlamDataMessagePtr __p = new ::HawkMessages::SlamDataMessage(*this);
    return __p;
}

static const ::std::string __HawkMessages__SlamDataMessage_ids[3] =
{
    "::HawkMessages::Message",
    "::HawkMessages::SlamDataMessage",
    "::Ice::Object"
};

bool
HawkMessages::SlamDataMessage::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__HawkMessages__SlamDataMessage_ids, __HawkMessages__SlamDataMessage_ids + 3, _s);
}

::std::vector< ::std::string>
HawkMessages::SlamDataMessage::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__HawkMessages__SlamDataMessage_ids[0], &__HawkMessages__SlamDataMessage_ids[3]);
}

const ::std::string&
HawkMessages::SlamDataMessage::ice_id(const ::Ice::Current&) const
{
    return __HawkMessages__SlamDataMessage_ids[1];
}

const ::std::string&
HawkMessages::SlamDataMessage::ice_staticId()
{
    return __HawkMessages__SlamDataMessage_ids[1];
}

void
HawkMessages::SlamDataMessage::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    hawkPose.__write(__os);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Message::__write(__os);
#else
    ::HawkMessages::Message::__write(__os);
#endif
}

void
HawkMessages::SlamDataMessage::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    hawkPose.__read(__is);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Message::__read(__is, true);
#else
    ::HawkMessages::Message::__read(__is, true);
#endif
}

void
HawkMessages::SlamDataMessage::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type HawkMessages::SlamDataMessage was not generated with stream support";
    throw ex;
}

void
HawkMessages::SlamDataMessage::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type HawkMessages::SlamDataMessage was not generated with stream support";
    throw ex;
}

class __F__HawkMessages__SlamDataMessage : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::HawkMessages::SlamDataMessage::ice_staticId());
        return new ::HawkMessages::SlamDataMessage;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__HawkMessages__SlamDataMessage_Ptr = new __F__HawkMessages__SlamDataMessage;

const ::Ice::ObjectFactoryPtr&
HawkMessages::SlamDataMessage::ice_factory()
{
    return __F__HawkMessages__SlamDataMessage_Ptr;
}

class __F__HawkMessages__SlamDataMessage__Init
{
public:

    __F__HawkMessages__SlamDataMessage__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::HawkMessages::SlamDataMessage::ice_staticId(), ::HawkMessages::SlamDataMessage::ice_factory());
    }

    ~__F__HawkMessages__SlamDataMessage__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::HawkMessages::SlamDataMessage::ice_staticId());
    }
};

static __F__HawkMessages__SlamDataMessage__Init __F__HawkMessages__SlamDataMessage__i;

#ifdef __APPLE__
extern "C" { void __F__HawkMessages__SlamDataMessage__initializer() {} }
#endif

void
HawkMessages::__patch__SlamDataMessagePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::HawkMessages::SlamDataMessagePtr* p = static_cast< ::HawkMessages::SlamDataMessagePtr*>(__addr);
    assert(p);
    *p = ::HawkMessages::SlamDataMessagePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::HawkMessages::SlamDataMessage::ice_staticId(), v->ice_id());
    }
}

bool
HawkMessages::operator==(const ::HawkMessages::SlamDataMessage& l, const ::HawkMessages::SlamDataMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
HawkMessages::operator<(const ::HawkMessages::SlamDataMessage& l, const ::HawkMessages::SlamDataMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}
