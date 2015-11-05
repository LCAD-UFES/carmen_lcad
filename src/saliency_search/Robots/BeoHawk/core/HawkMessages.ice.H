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

#ifndef __src_Robots_BeoHawk_core__HawkMessages_ice_H__
#define __src_Robots_BeoHawk_core__HawkMessages_ice_H__

#include <Ice/LocalObjectF.h>
#include <Ice/ProxyF.h>
#include <Ice/ObjectF.h>
#include <Ice/Exception.h>
#include <Ice/LocalObject.h>
#include <Ice/Proxy.h>
#include <Ice/Object.h>
#include <Ice/Outgoing.h>
#include <Ice/Incoming.h>
#include <Ice/Direct.h>
#include <Ice/FactoryTable.h>
#include <Ice/StreamF.h>
#include <Ice/ImageIce.ice.H>
#include <Ice/UndefSysMacros.h>

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

namespace IceProxy
{

namespace HawkMessages
{

class Message;

class MessageAgent;

class CameraImageMessage;

class ControlCameraMessage;

class ControlDriveVisionMessage;

class ControlLandMessage;

class ControlMoveMessage;

class ControlRoomVisionMessage;

class ControlTakeOffMessage;

class DriveFinderMessage;

class ExampleMessage;

class ExecuteMissionMessage;

class MissionListMessage;

class RoomFinderMessage;

class SensorDataMessage;

class SlamDataMessage;

}

}

namespace HawkMessages
{

class Message;
bool operator==(const Message&, const Message&);
bool operator<(const Message&, const Message&);

class MessageAgent;
bool operator==(const MessageAgent&, const MessageAgent&);
bool operator<(const MessageAgent&, const MessageAgent&);

class CameraImageMessage;
bool operator==(const CameraImageMessage&, const CameraImageMessage&);
bool operator<(const CameraImageMessage&, const CameraImageMessage&);

class ControlCameraMessage;
bool operator==(const ControlCameraMessage&, const ControlCameraMessage&);
bool operator<(const ControlCameraMessage&, const ControlCameraMessage&);

class ControlDriveVisionMessage;
bool operator==(const ControlDriveVisionMessage&, const ControlDriveVisionMessage&);
bool operator<(const ControlDriveVisionMessage&, const ControlDriveVisionMessage&);

class ControlLandMessage;
bool operator==(const ControlLandMessage&, const ControlLandMessage&);
bool operator<(const ControlLandMessage&, const ControlLandMessage&);

class ControlMoveMessage;
bool operator==(const ControlMoveMessage&, const ControlMoveMessage&);
bool operator<(const ControlMoveMessage&, const ControlMoveMessage&);

class ControlRoomVisionMessage;
bool operator==(const ControlRoomVisionMessage&, const ControlRoomVisionMessage&);
bool operator<(const ControlRoomVisionMessage&, const ControlRoomVisionMessage&);

class ControlTakeOffMessage;
bool operator==(const ControlTakeOffMessage&, const ControlTakeOffMessage&);
bool operator<(const ControlTakeOffMessage&, const ControlTakeOffMessage&);

class DriveFinderMessage;
bool operator==(const DriveFinderMessage&, const DriveFinderMessage&);
bool operator<(const DriveFinderMessage&, const DriveFinderMessage&);

class ExampleMessage;
bool operator==(const ExampleMessage&, const ExampleMessage&);
bool operator<(const ExampleMessage&, const ExampleMessage&);

class ExecuteMissionMessage;
bool operator==(const ExecuteMissionMessage&, const ExecuteMissionMessage&);
bool operator<(const ExecuteMissionMessage&, const ExecuteMissionMessage&);

class MissionListMessage;
bool operator==(const MissionListMessage&, const MissionListMessage&);
bool operator<(const MissionListMessage&, const MissionListMessage&);

class RoomFinderMessage;
bool operator==(const RoomFinderMessage&, const RoomFinderMessage&);
bool operator<(const RoomFinderMessage&, const RoomFinderMessage&);

class SensorDataMessage;
bool operator==(const SensorDataMessage&, const SensorDataMessage&);
bool operator<(const SensorDataMessage&, const SensorDataMessage&);

class SlamDataMessage;
bool operator==(const SlamDataMessage&, const SlamDataMessage&);
bool operator<(const SlamDataMessage&, const SlamDataMessage&);

}

namespace IceInternal
{

::Ice::Object* upCast(::HawkMessages::Message*);
::IceProxy::Ice::Object* upCast(::IceProxy::HawkMessages::Message*);

::Ice::Object* upCast(::HawkMessages::MessageAgent*);
::IceProxy::Ice::Object* upCast(::IceProxy::HawkMessages::MessageAgent*);

::Ice::Object* upCast(::HawkMessages::CameraImageMessage*);
::IceProxy::Ice::Object* upCast(::IceProxy::HawkMessages::CameraImageMessage*);

::Ice::Object* upCast(::HawkMessages::ControlCameraMessage*);
::IceProxy::Ice::Object* upCast(::IceProxy::HawkMessages::ControlCameraMessage*);

::Ice::Object* upCast(::HawkMessages::ControlDriveVisionMessage*);
::IceProxy::Ice::Object* upCast(::IceProxy::HawkMessages::ControlDriveVisionMessage*);

::Ice::Object* upCast(::HawkMessages::ControlLandMessage*);
::IceProxy::Ice::Object* upCast(::IceProxy::HawkMessages::ControlLandMessage*);

::Ice::Object* upCast(::HawkMessages::ControlMoveMessage*);
::IceProxy::Ice::Object* upCast(::IceProxy::HawkMessages::ControlMoveMessage*);

::Ice::Object* upCast(::HawkMessages::ControlRoomVisionMessage*);
::IceProxy::Ice::Object* upCast(::IceProxy::HawkMessages::ControlRoomVisionMessage*);

::Ice::Object* upCast(::HawkMessages::ControlTakeOffMessage*);
::IceProxy::Ice::Object* upCast(::IceProxy::HawkMessages::ControlTakeOffMessage*);

::Ice::Object* upCast(::HawkMessages::DriveFinderMessage*);
::IceProxy::Ice::Object* upCast(::IceProxy::HawkMessages::DriveFinderMessage*);

::Ice::Object* upCast(::HawkMessages::ExampleMessage*);
::IceProxy::Ice::Object* upCast(::IceProxy::HawkMessages::ExampleMessage*);

::Ice::Object* upCast(::HawkMessages::ExecuteMissionMessage*);
::IceProxy::Ice::Object* upCast(::IceProxy::HawkMessages::ExecuteMissionMessage*);

::Ice::Object* upCast(::HawkMessages::MissionListMessage*);
::IceProxy::Ice::Object* upCast(::IceProxy::HawkMessages::MissionListMessage*);

::Ice::Object* upCast(::HawkMessages::RoomFinderMessage*);
::IceProxy::Ice::Object* upCast(::IceProxy::HawkMessages::RoomFinderMessage*);

::Ice::Object* upCast(::HawkMessages::SensorDataMessage*);
::IceProxy::Ice::Object* upCast(::IceProxy::HawkMessages::SensorDataMessage*);

::Ice::Object* upCast(::HawkMessages::SlamDataMessage*);
::IceProxy::Ice::Object* upCast(::IceProxy::HawkMessages::SlamDataMessage*);

}

namespace HawkMessages
{

typedef ::IceInternal::Handle< ::HawkMessages::Message> MessagePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::HawkMessages::Message> MessagePrx;

void __read(::IceInternal::BasicStream*, MessagePrx&);
void __patch__MessagePtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::HawkMessages::MessageAgent> MessageAgentPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::HawkMessages::MessageAgent> MessageAgentPrx;

void __read(::IceInternal::BasicStream*, MessageAgentPrx&);
void __patch__MessageAgentPtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::HawkMessages::CameraImageMessage> CameraImageMessagePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::HawkMessages::CameraImageMessage> CameraImageMessagePrx;

void __read(::IceInternal::BasicStream*, CameraImageMessagePrx&);
void __patch__CameraImageMessagePtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::HawkMessages::ControlCameraMessage> ControlCameraMessagePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::HawkMessages::ControlCameraMessage> ControlCameraMessagePrx;

void __read(::IceInternal::BasicStream*, ControlCameraMessagePrx&);
void __patch__ControlCameraMessagePtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::HawkMessages::ControlDriveVisionMessage> ControlDriveVisionMessagePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::HawkMessages::ControlDriveVisionMessage> ControlDriveVisionMessagePrx;

void __read(::IceInternal::BasicStream*, ControlDriveVisionMessagePrx&);
void __patch__ControlDriveVisionMessagePtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::HawkMessages::ControlLandMessage> ControlLandMessagePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::HawkMessages::ControlLandMessage> ControlLandMessagePrx;

void __read(::IceInternal::BasicStream*, ControlLandMessagePrx&);
void __patch__ControlLandMessagePtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::HawkMessages::ControlMoveMessage> ControlMoveMessagePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::HawkMessages::ControlMoveMessage> ControlMoveMessagePrx;

void __read(::IceInternal::BasicStream*, ControlMoveMessagePrx&);
void __patch__ControlMoveMessagePtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::HawkMessages::ControlRoomVisionMessage> ControlRoomVisionMessagePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::HawkMessages::ControlRoomVisionMessage> ControlRoomVisionMessagePrx;

void __read(::IceInternal::BasicStream*, ControlRoomVisionMessagePrx&);
void __patch__ControlRoomVisionMessagePtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::HawkMessages::ControlTakeOffMessage> ControlTakeOffMessagePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::HawkMessages::ControlTakeOffMessage> ControlTakeOffMessagePrx;

void __read(::IceInternal::BasicStream*, ControlTakeOffMessagePrx&);
void __patch__ControlTakeOffMessagePtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::HawkMessages::DriveFinderMessage> DriveFinderMessagePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::HawkMessages::DriveFinderMessage> DriveFinderMessagePrx;

void __read(::IceInternal::BasicStream*, DriveFinderMessagePrx&);
void __patch__DriveFinderMessagePtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::HawkMessages::ExampleMessage> ExampleMessagePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::HawkMessages::ExampleMessage> ExampleMessagePrx;

void __read(::IceInternal::BasicStream*, ExampleMessagePrx&);
void __patch__ExampleMessagePtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::HawkMessages::ExecuteMissionMessage> ExecuteMissionMessagePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::HawkMessages::ExecuteMissionMessage> ExecuteMissionMessagePrx;

void __read(::IceInternal::BasicStream*, ExecuteMissionMessagePrx&);
void __patch__ExecuteMissionMessagePtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::HawkMessages::MissionListMessage> MissionListMessagePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::HawkMessages::MissionListMessage> MissionListMessagePrx;

void __read(::IceInternal::BasicStream*, MissionListMessagePrx&);
void __patch__MissionListMessagePtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::HawkMessages::RoomFinderMessage> RoomFinderMessagePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::HawkMessages::RoomFinderMessage> RoomFinderMessagePrx;

void __read(::IceInternal::BasicStream*, RoomFinderMessagePrx&);
void __patch__RoomFinderMessagePtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::HawkMessages::SensorDataMessage> SensorDataMessagePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::HawkMessages::SensorDataMessage> SensorDataMessagePrx;

void __read(::IceInternal::BasicStream*, SensorDataMessagePrx&);
void __patch__SensorDataMessagePtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::HawkMessages::SlamDataMessage> SlamDataMessagePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::HawkMessages::SlamDataMessage> SlamDataMessagePrx;

void __read(::IceInternal::BasicStream*, SlamDataMessagePrx&);
void __patch__SlamDataMessagePtr(void*, ::Ice::ObjectPtr&);

}

namespace HawkMessages
{

struct Pose
{
    ::Ice::Double x;
    ::Ice::Double y;
    ::Ice::Double z;
    ::Ice::Double theta;

    bool operator==(const Pose&) const;
    bool operator<(const Pose&) const;
    bool operator!=(const Pose& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const Pose& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const Pose& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const Pose& __rhs) const
    {
        return !operator<(__rhs);
    }

    void __write(::IceInternal::BasicStream*) const;
    void __read(::IceInternal::BasicStream*);
};

struct Sonar
{
    ::std::string sonarID;
    ::Ice::Double distance;

    bool operator==(const Sonar&) const;
    bool operator<(const Sonar&) const;
    bool operator!=(const Sonar& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const Sonar& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const Sonar& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const Sonar& __rhs) const
    {
        return !operator<(__rhs);
    }

    void __write(::IceInternal::BasicStream*) const;
    void __read(::IceInternal::BasicStream*);
};

typedef ::std::vector< ::HawkMessages::Sonar> SonarSeq;
void __writeSonarSeq(::IceInternal::BasicStream*, const ::HawkMessages::Sonar*, const ::HawkMessages::Sonar*);
void __readSonarSeq(::IceInternal::BasicStream*, SonarSeq&);

typedef ::std::vector< ::std::string> StringSeq;

typedef ::std::vector< ::Ice::Long> LongSeq;

}

namespace IceProxy
{

namespace HawkMessages
{

class Message : virtual public ::IceProxy::Ice::Object
{
public:

    ::IceInternal::ProxyHandle<Message> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Message*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<Message*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }

    ::IceInternal::ProxyHandle<Message> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Message*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<Message*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }

    ::IceInternal::ProxyHandle<Message> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Message*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<Message*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }

    ::IceInternal::ProxyHandle<Message> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Message*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<Message*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }

    ::IceInternal::ProxyHandle<Message> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Message*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<Message*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }

    ::IceInternal::ProxyHandle<Message> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Message*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<Message*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }

    ::IceInternal::ProxyHandle<Message> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Message*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<Message*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }

    ::IceInternal::ProxyHandle<Message> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Message*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<Message*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }

    ::IceInternal::ProxyHandle<Message> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Message*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<Message*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }

    ::IceInternal::ProxyHandle<Message> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Message*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<Message*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }

    ::IceInternal::ProxyHandle<Message> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Message*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<Message*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }

    ::IceInternal::ProxyHandle<Message> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Message*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<Message*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<Message> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Message*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<Message*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<Message> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Message*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<Message*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<Message> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Message*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<Message*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }

    ::IceInternal::ProxyHandle<Message> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Message*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<Message*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }

    ::IceInternal::ProxyHandle<Message> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Message*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<Message*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }

    ::IceInternal::ProxyHandle<Message> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Message*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<Message*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }

    ::IceInternal::ProxyHandle<Message> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Message*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<Message*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }

    static const ::std::string& ice_staticId();

private:

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class MessageAgent : virtual public ::IceProxy::Ice::Object
{
public:

    void catchMessage(const ::HawkMessages::MessagePtr& msg)
    {
        catchMessage(msg, 0);
    }
    void catchMessage(const ::HawkMessages::MessagePtr& msg, const ::Ice::Context& __ctx)
    {
        catchMessage(msg, &__ctx);
    }

private:

    void catchMessage(const ::HawkMessages::MessagePtr&, const ::Ice::Context*);

public:

    ::IceInternal::ProxyHandle<MessageAgent> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MessageAgent*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<MessageAgent*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }

    ::IceInternal::ProxyHandle<MessageAgent> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MessageAgent*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<MessageAgent*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }

    ::IceInternal::ProxyHandle<MessageAgent> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MessageAgent*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<MessageAgent*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }

    ::IceInternal::ProxyHandle<MessageAgent> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MessageAgent*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<MessageAgent*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }

    ::IceInternal::ProxyHandle<MessageAgent> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MessageAgent*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<MessageAgent*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }

    ::IceInternal::ProxyHandle<MessageAgent> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MessageAgent*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<MessageAgent*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }

    ::IceInternal::ProxyHandle<MessageAgent> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MessageAgent*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<MessageAgent*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }

    ::IceInternal::ProxyHandle<MessageAgent> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MessageAgent*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<MessageAgent*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }

    ::IceInternal::ProxyHandle<MessageAgent> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MessageAgent*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<MessageAgent*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }

    ::IceInternal::ProxyHandle<MessageAgent> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MessageAgent*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<MessageAgent*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }

    ::IceInternal::ProxyHandle<MessageAgent> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MessageAgent*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<MessageAgent*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }

    ::IceInternal::ProxyHandle<MessageAgent> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MessageAgent*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<MessageAgent*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<MessageAgent> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MessageAgent*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<MessageAgent*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<MessageAgent> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MessageAgent*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<MessageAgent*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<MessageAgent> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MessageAgent*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<MessageAgent*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }

    ::IceInternal::ProxyHandle<MessageAgent> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MessageAgent*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<MessageAgent*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }

    ::IceInternal::ProxyHandle<MessageAgent> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MessageAgent*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<MessageAgent*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }

    ::IceInternal::ProxyHandle<MessageAgent> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MessageAgent*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<MessageAgent*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }

    ::IceInternal::ProxyHandle<MessageAgent> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MessageAgent*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<MessageAgent*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }

    static const ::std::string& ice_staticId();

private:

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class CameraImageMessage : virtual public ::IceProxy::HawkMessages::Message
{
public:

    ::IceInternal::ProxyHandle<CameraImageMessage> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CameraImageMessage*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<CameraImageMessage*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }

    ::IceInternal::ProxyHandle<CameraImageMessage> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CameraImageMessage*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<CameraImageMessage*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }

    ::IceInternal::ProxyHandle<CameraImageMessage> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CameraImageMessage*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<CameraImageMessage*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }

    ::IceInternal::ProxyHandle<CameraImageMessage> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CameraImageMessage*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<CameraImageMessage*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }

    ::IceInternal::ProxyHandle<CameraImageMessage> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CameraImageMessage*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<CameraImageMessage*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }

    ::IceInternal::ProxyHandle<CameraImageMessage> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CameraImageMessage*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<CameraImageMessage*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }

    ::IceInternal::ProxyHandle<CameraImageMessage> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CameraImageMessage*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<CameraImageMessage*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }

    ::IceInternal::ProxyHandle<CameraImageMessage> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CameraImageMessage*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<CameraImageMessage*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }

    ::IceInternal::ProxyHandle<CameraImageMessage> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CameraImageMessage*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<CameraImageMessage*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }

    ::IceInternal::ProxyHandle<CameraImageMessage> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CameraImageMessage*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<CameraImageMessage*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }

    ::IceInternal::ProxyHandle<CameraImageMessage> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CameraImageMessage*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<CameraImageMessage*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }

    ::IceInternal::ProxyHandle<CameraImageMessage> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CameraImageMessage*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<CameraImageMessage*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<CameraImageMessage> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CameraImageMessage*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<CameraImageMessage*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<CameraImageMessage> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CameraImageMessage*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<CameraImageMessage*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<CameraImageMessage> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CameraImageMessage*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<CameraImageMessage*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }

    ::IceInternal::ProxyHandle<CameraImageMessage> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CameraImageMessage*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<CameraImageMessage*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }

    ::IceInternal::ProxyHandle<CameraImageMessage> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CameraImageMessage*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<CameraImageMessage*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }

    ::IceInternal::ProxyHandle<CameraImageMessage> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CameraImageMessage*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<CameraImageMessage*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }

    ::IceInternal::ProxyHandle<CameraImageMessage> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CameraImageMessage*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<CameraImageMessage*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }

    static const ::std::string& ice_staticId();

private:

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class ControlCameraMessage : virtual public ::IceProxy::HawkMessages::Message
{
public:

    ::IceInternal::ProxyHandle<ControlCameraMessage> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlCameraMessage*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<ControlCameraMessage*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlCameraMessage> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlCameraMessage*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<ControlCameraMessage*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlCameraMessage> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlCameraMessage*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<ControlCameraMessage*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlCameraMessage> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlCameraMessage*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<ControlCameraMessage*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlCameraMessage> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlCameraMessage*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<ControlCameraMessage*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlCameraMessage> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlCameraMessage*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<ControlCameraMessage*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlCameraMessage> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlCameraMessage*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<ControlCameraMessage*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlCameraMessage> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlCameraMessage*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<ControlCameraMessage*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlCameraMessage> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlCameraMessage*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<ControlCameraMessage*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlCameraMessage> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlCameraMessage*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<ControlCameraMessage*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlCameraMessage> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlCameraMessage*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<ControlCameraMessage*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlCameraMessage> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlCameraMessage*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<ControlCameraMessage*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlCameraMessage> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlCameraMessage*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<ControlCameraMessage*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlCameraMessage> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlCameraMessage*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<ControlCameraMessage*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlCameraMessage> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlCameraMessage*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<ControlCameraMessage*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlCameraMessage> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlCameraMessage*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<ControlCameraMessage*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlCameraMessage> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlCameraMessage*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<ControlCameraMessage*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlCameraMessage> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlCameraMessage*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<ControlCameraMessage*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlCameraMessage> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlCameraMessage*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<ControlCameraMessage*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }

    static const ::std::string& ice_staticId();

private:

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class ControlDriveVisionMessage : virtual public ::IceProxy::HawkMessages::Message
{
public:

    ::IceInternal::ProxyHandle<ControlDriveVisionMessage> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlDriveVisionMessage*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<ControlDriveVisionMessage*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlDriveVisionMessage> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlDriveVisionMessage*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<ControlDriveVisionMessage*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlDriveVisionMessage> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlDriveVisionMessage*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<ControlDriveVisionMessage*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlDriveVisionMessage> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlDriveVisionMessage*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<ControlDriveVisionMessage*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlDriveVisionMessage> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlDriveVisionMessage*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<ControlDriveVisionMessage*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlDriveVisionMessage> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlDriveVisionMessage*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<ControlDriveVisionMessage*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlDriveVisionMessage> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlDriveVisionMessage*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<ControlDriveVisionMessage*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlDriveVisionMessage> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlDriveVisionMessage*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<ControlDriveVisionMessage*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlDriveVisionMessage> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlDriveVisionMessage*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<ControlDriveVisionMessage*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlDriveVisionMessage> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlDriveVisionMessage*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<ControlDriveVisionMessage*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlDriveVisionMessage> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlDriveVisionMessage*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<ControlDriveVisionMessage*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlDriveVisionMessage> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlDriveVisionMessage*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<ControlDriveVisionMessage*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlDriveVisionMessage> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlDriveVisionMessage*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<ControlDriveVisionMessage*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlDriveVisionMessage> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlDriveVisionMessage*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<ControlDriveVisionMessage*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlDriveVisionMessage> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlDriveVisionMessage*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<ControlDriveVisionMessage*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlDriveVisionMessage> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlDriveVisionMessage*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<ControlDriveVisionMessage*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlDriveVisionMessage> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlDriveVisionMessage*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<ControlDriveVisionMessage*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlDriveVisionMessage> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlDriveVisionMessage*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<ControlDriveVisionMessage*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlDriveVisionMessage> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlDriveVisionMessage*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<ControlDriveVisionMessage*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }

    static const ::std::string& ice_staticId();

private:

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class ControlLandMessage : virtual public ::IceProxy::HawkMessages::Message
{
public:

    ::IceInternal::ProxyHandle<ControlLandMessage> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlLandMessage*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<ControlLandMessage*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlLandMessage> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlLandMessage*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<ControlLandMessage*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlLandMessage> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlLandMessage*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<ControlLandMessage*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlLandMessage> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlLandMessage*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<ControlLandMessage*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlLandMessage> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlLandMessage*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<ControlLandMessage*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlLandMessage> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlLandMessage*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<ControlLandMessage*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlLandMessage> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlLandMessage*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<ControlLandMessage*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlLandMessage> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlLandMessage*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<ControlLandMessage*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlLandMessage> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlLandMessage*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<ControlLandMessage*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlLandMessage> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlLandMessage*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<ControlLandMessage*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlLandMessage> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlLandMessage*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<ControlLandMessage*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlLandMessage> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlLandMessage*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<ControlLandMessage*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlLandMessage> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlLandMessage*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<ControlLandMessage*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlLandMessage> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlLandMessage*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<ControlLandMessage*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlLandMessage> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlLandMessage*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<ControlLandMessage*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlLandMessage> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlLandMessage*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<ControlLandMessage*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlLandMessage> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlLandMessage*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<ControlLandMessage*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlLandMessage> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlLandMessage*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<ControlLandMessage*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlLandMessage> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlLandMessage*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<ControlLandMessage*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }

    static const ::std::string& ice_staticId();

private:

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class ControlMoveMessage : virtual public ::IceProxy::HawkMessages::Message
{
public:

    ::IceInternal::ProxyHandle<ControlMoveMessage> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlMoveMessage*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<ControlMoveMessage*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlMoveMessage> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlMoveMessage*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<ControlMoveMessage*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlMoveMessage> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlMoveMessage*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<ControlMoveMessage*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlMoveMessage> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlMoveMessage*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<ControlMoveMessage*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlMoveMessage> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlMoveMessage*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<ControlMoveMessage*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlMoveMessage> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlMoveMessage*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<ControlMoveMessage*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlMoveMessage> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlMoveMessage*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<ControlMoveMessage*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlMoveMessage> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlMoveMessage*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<ControlMoveMessage*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlMoveMessage> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlMoveMessage*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<ControlMoveMessage*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlMoveMessage> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlMoveMessage*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<ControlMoveMessage*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlMoveMessage> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlMoveMessage*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<ControlMoveMessage*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlMoveMessage> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlMoveMessage*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<ControlMoveMessage*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlMoveMessage> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlMoveMessage*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<ControlMoveMessage*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlMoveMessage> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlMoveMessage*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<ControlMoveMessage*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlMoveMessage> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlMoveMessage*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<ControlMoveMessage*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlMoveMessage> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlMoveMessage*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<ControlMoveMessage*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlMoveMessage> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlMoveMessage*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<ControlMoveMessage*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlMoveMessage> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlMoveMessage*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<ControlMoveMessage*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlMoveMessage> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlMoveMessage*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<ControlMoveMessage*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }

    static const ::std::string& ice_staticId();

private:

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class ControlRoomVisionMessage : virtual public ::IceProxy::HawkMessages::Message
{
public:

    ::IceInternal::ProxyHandle<ControlRoomVisionMessage> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlRoomVisionMessage*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<ControlRoomVisionMessage*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlRoomVisionMessage> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlRoomVisionMessage*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<ControlRoomVisionMessage*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlRoomVisionMessage> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlRoomVisionMessage*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<ControlRoomVisionMessage*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlRoomVisionMessage> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlRoomVisionMessage*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<ControlRoomVisionMessage*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlRoomVisionMessage> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlRoomVisionMessage*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<ControlRoomVisionMessage*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlRoomVisionMessage> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlRoomVisionMessage*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<ControlRoomVisionMessage*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlRoomVisionMessage> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlRoomVisionMessage*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<ControlRoomVisionMessage*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlRoomVisionMessage> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlRoomVisionMessage*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<ControlRoomVisionMessage*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlRoomVisionMessage> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlRoomVisionMessage*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<ControlRoomVisionMessage*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlRoomVisionMessage> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlRoomVisionMessage*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<ControlRoomVisionMessage*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlRoomVisionMessage> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlRoomVisionMessage*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<ControlRoomVisionMessage*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlRoomVisionMessage> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlRoomVisionMessage*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<ControlRoomVisionMessage*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlRoomVisionMessage> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlRoomVisionMessage*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<ControlRoomVisionMessage*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlRoomVisionMessage> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlRoomVisionMessage*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<ControlRoomVisionMessage*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlRoomVisionMessage> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlRoomVisionMessage*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<ControlRoomVisionMessage*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlRoomVisionMessage> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlRoomVisionMessage*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<ControlRoomVisionMessage*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlRoomVisionMessage> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlRoomVisionMessage*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<ControlRoomVisionMessage*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlRoomVisionMessage> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlRoomVisionMessage*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<ControlRoomVisionMessage*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlRoomVisionMessage> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlRoomVisionMessage*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<ControlRoomVisionMessage*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }

    static const ::std::string& ice_staticId();

private:

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class ControlTakeOffMessage : virtual public ::IceProxy::HawkMessages::Message
{
public:

    ::IceInternal::ProxyHandle<ControlTakeOffMessage> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlTakeOffMessage*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<ControlTakeOffMessage*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlTakeOffMessage> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlTakeOffMessage*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<ControlTakeOffMessage*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlTakeOffMessage> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlTakeOffMessage*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<ControlTakeOffMessage*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlTakeOffMessage> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlTakeOffMessage*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<ControlTakeOffMessage*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlTakeOffMessage> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlTakeOffMessage*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<ControlTakeOffMessage*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlTakeOffMessage> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlTakeOffMessage*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<ControlTakeOffMessage*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlTakeOffMessage> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlTakeOffMessage*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<ControlTakeOffMessage*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlTakeOffMessage> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlTakeOffMessage*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<ControlTakeOffMessage*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlTakeOffMessage> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlTakeOffMessage*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<ControlTakeOffMessage*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlTakeOffMessage> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlTakeOffMessage*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<ControlTakeOffMessage*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlTakeOffMessage> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlTakeOffMessage*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<ControlTakeOffMessage*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlTakeOffMessage> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlTakeOffMessage*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<ControlTakeOffMessage*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlTakeOffMessage> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlTakeOffMessage*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<ControlTakeOffMessage*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlTakeOffMessage> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlTakeOffMessage*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<ControlTakeOffMessage*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlTakeOffMessage> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlTakeOffMessage*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<ControlTakeOffMessage*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlTakeOffMessage> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlTakeOffMessage*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<ControlTakeOffMessage*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlTakeOffMessage> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlTakeOffMessage*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<ControlTakeOffMessage*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlTakeOffMessage> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlTakeOffMessage*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<ControlTakeOffMessage*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ControlTakeOffMessage> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ControlTakeOffMessage*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<ControlTakeOffMessage*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }

    static const ::std::string& ice_staticId();

private:

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class DriveFinderMessage : virtual public ::IceProxy::HawkMessages::Message
{
public:

    ::IceInternal::ProxyHandle<DriveFinderMessage> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DriveFinderMessage*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<DriveFinderMessage*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }

    ::IceInternal::ProxyHandle<DriveFinderMessage> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DriveFinderMessage*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<DriveFinderMessage*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }

    ::IceInternal::ProxyHandle<DriveFinderMessage> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DriveFinderMessage*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<DriveFinderMessage*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }

    ::IceInternal::ProxyHandle<DriveFinderMessage> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DriveFinderMessage*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<DriveFinderMessage*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }

    ::IceInternal::ProxyHandle<DriveFinderMessage> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DriveFinderMessage*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<DriveFinderMessage*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }

    ::IceInternal::ProxyHandle<DriveFinderMessage> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DriveFinderMessage*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<DriveFinderMessage*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }

    ::IceInternal::ProxyHandle<DriveFinderMessage> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DriveFinderMessage*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<DriveFinderMessage*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }

    ::IceInternal::ProxyHandle<DriveFinderMessage> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DriveFinderMessage*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<DriveFinderMessage*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }

    ::IceInternal::ProxyHandle<DriveFinderMessage> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DriveFinderMessage*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<DriveFinderMessage*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }

    ::IceInternal::ProxyHandle<DriveFinderMessage> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DriveFinderMessage*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<DriveFinderMessage*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }

    ::IceInternal::ProxyHandle<DriveFinderMessage> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DriveFinderMessage*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<DriveFinderMessage*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }

    ::IceInternal::ProxyHandle<DriveFinderMessage> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DriveFinderMessage*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<DriveFinderMessage*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<DriveFinderMessage> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DriveFinderMessage*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<DriveFinderMessage*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<DriveFinderMessage> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DriveFinderMessage*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<DriveFinderMessage*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<DriveFinderMessage> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DriveFinderMessage*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<DriveFinderMessage*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }

    ::IceInternal::ProxyHandle<DriveFinderMessage> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DriveFinderMessage*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<DriveFinderMessage*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }

    ::IceInternal::ProxyHandle<DriveFinderMessage> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DriveFinderMessage*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<DriveFinderMessage*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }

    ::IceInternal::ProxyHandle<DriveFinderMessage> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DriveFinderMessage*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<DriveFinderMessage*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }

    ::IceInternal::ProxyHandle<DriveFinderMessage> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<DriveFinderMessage*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<DriveFinderMessage*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }

    static const ::std::string& ice_staticId();

private:

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class ExampleMessage : virtual public ::IceProxy::HawkMessages::Message
{
public:

    ::IceInternal::ProxyHandle<ExampleMessage> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExampleMessage*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<ExampleMessage*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ExampleMessage> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExampleMessage*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<ExampleMessage*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ExampleMessage> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExampleMessage*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<ExampleMessage*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ExampleMessage> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExampleMessage*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<ExampleMessage*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ExampleMessage> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExampleMessage*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<ExampleMessage*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ExampleMessage> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExampleMessage*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<ExampleMessage*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ExampleMessage> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExampleMessage*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<ExampleMessage*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ExampleMessage> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExampleMessage*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<ExampleMessage*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ExampleMessage> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExampleMessage*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<ExampleMessage*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ExampleMessage> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExampleMessage*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<ExampleMessage*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ExampleMessage> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExampleMessage*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<ExampleMessage*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ExampleMessage> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExampleMessage*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<ExampleMessage*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<ExampleMessage> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExampleMessage*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<ExampleMessage*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<ExampleMessage> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExampleMessage*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<ExampleMessage*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<ExampleMessage> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExampleMessage*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<ExampleMessage*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }

    ::IceInternal::ProxyHandle<ExampleMessage> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExampleMessage*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<ExampleMessage*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }

    ::IceInternal::ProxyHandle<ExampleMessage> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExampleMessage*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<ExampleMessage*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ExampleMessage> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExampleMessage*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<ExampleMessage*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ExampleMessage> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExampleMessage*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<ExampleMessage*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }

    static const ::std::string& ice_staticId();

private:

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class ExecuteMissionMessage : virtual public ::IceProxy::HawkMessages::Message
{
public:

    ::IceInternal::ProxyHandle<ExecuteMissionMessage> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExecuteMissionMessage*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<ExecuteMissionMessage*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ExecuteMissionMessage> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExecuteMissionMessage*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<ExecuteMissionMessage*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ExecuteMissionMessage> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExecuteMissionMessage*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<ExecuteMissionMessage*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ExecuteMissionMessage> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExecuteMissionMessage*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<ExecuteMissionMessage*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ExecuteMissionMessage> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExecuteMissionMessage*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<ExecuteMissionMessage*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ExecuteMissionMessage> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExecuteMissionMessage*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<ExecuteMissionMessage*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ExecuteMissionMessage> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExecuteMissionMessage*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<ExecuteMissionMessage*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ExecuteMissionMessage> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExecuteMissionMessage*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<ExecuteMissionMessage*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ExecuteMissionMessage> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExecuteMissionMessage*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<ExecuteMissionMessage*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ExecuteMissionMessage> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExecuteMissionMessage*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<ExecuteMissionMessage*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ExecuteMissionMessage> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExecuteMissionMessage*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<ExecuteMissionMessage*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ExecuteMissionMessage> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExecuteMissionMessage*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<ExecuteMissionMessage*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<ExecuteMissionMessage> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExecuteMissionMessage*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<ExecuteMissionMessage*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<ExecuteMissionMessage> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExecuteMissionMessage*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<ExecuteMissionMessage*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<ExecuteMissionMessage> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExecuteMissionMessage*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<ExecuteMissionMessage*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }

    ::IceInternal::ProxyHandle<ExecuteMissionMessage> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExecuteMissionMessage*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<ExecuteMissionMessage*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }

    ::IceInternal::ProxyHandle<ExecuteMissionMessage> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExecuteMissionMessage*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<ExecuteMissionMessage*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ExecuteMissionMessage> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExecuteMissionMessage*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<ExecuteMissionMessage*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }

    ::IceInternal::ProxyHandle<ExecuteMissionMessage> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<ExecuteMissionMessage*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<ExecuteMissionMessage*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }

    static const ::std::string& ice_staticId();

private:

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class MissionListMessage : virtual public ::IceProxy::HawkMessages::Message
{
public:

    ::IceInternal::ProxyHandle<MissionListMessage> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MissionListMessage*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<MissionListMessage*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }

    ::IceInternal::ProxyHandle<MissionListMessage> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MissionListMessage*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<MissionListMessage*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }

    ::IceInternal::ProxyHandle<MissionListMessage> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MissionListMessage*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<MissionListMessage*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }

    ::IceInternal::ProxyHandle<MissionListMessage> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MissionListMessage*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<MissionListMessage*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }

    ::IceInternal::ProxyHandle<MissionListMessage> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MissionListMessage*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<MissionListMessage*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }

    ::IceInternal::ProxyHandle<MissionListMessage> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MissionListMessage*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<MissionListMessage*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }

    ::IceInternal::ProxyHandle<MissionListMessage> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MissionListMessage*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<MissionListMessage*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }

    ::IceInternal::ProxyHandle<MissionListMessage> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MissionListMessage*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<MissionListMessage*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }

    ::IceInternal::ProxyHandle<MissionListMessage> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MissionListMessage*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<MissionListMessage*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }

    ::IceInternal::ProxyHandle<MissionListMessage> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MissionListMessage*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<MissionListMessage*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }

    ::IceInternal::ProxyHandle<MissionListMessage> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MissionListMessage*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<MissionListMessage*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }

    ::IceInternal::ProxyHandle<MissionListMessage> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MissionListMessage*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<MissionListMessage*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<MissionListMessage> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MissionListMessage*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<MissionListMessage*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<MissionListMessage> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MissionListMessage*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<MissionListMessage*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<MissionListMessage> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MissionListMessage*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<MissionListMessage*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }

    ::IceInternal::ProxyHandle<MissionListMessage> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MissionListMessage*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<MissionListMessage*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }

    ::IceInternal::ProxyHandle<MissionListMessage> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MissionListMessage*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<MissionListMessage*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }

    ::IceInternal::ProxyHandle<MissionListMessage> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MissionListMessage*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<MissionListMessage*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }

    ::IceInternal::ProxyHandle<MissionListMessage> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<MissionListMessage*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<MissionListMessage*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }

    static const ::std::string& ice_staticId();

private:

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class RoomFinderMessage : virtual public ::IceProxy::HawkMessages::Message
{
public:

    ::IceInternal::ProxyHandle<RoomFinderMessage> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RoomFinderMessage*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<RoomFinderMessage*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }

    ::IceInternal::ProxyHandle<RoomFinderMessage> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RoomFinderMessage*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<RoomFinderMessage*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }

    ::IceInternal::ProxyHandle<RoomFinderMessage> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RoomFinderMessage*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<RoomFinderMessage*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }

    ::IceInternal::ProxyHandle<RoomFinderMessage> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RoomFinderMessage*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<RoomFinderMessage*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }

    ::IceInternal::ProxyHandle<RoomFinderMessage> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RoomFinderMessage*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<RoomFinderMessage*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }

    ::IceInternal::ProxyHandle<RoomFinderMessage> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RoomFinderMessage*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<RoomFinderMessage*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }

    ::IceInternal::ProxyHandle<RoomFinderMessage> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RoomFinderMessage*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<RoomFinderMessage*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }

    ::IceInternal::ProxyHandle<RoomFinderMessage> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RoomFinderMessage*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<RoomFinderMessage*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }

    ::IceInternal::ProxyHandle<RoomFinderMessage> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RoomFinderMessage*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<RoomFinderMessage*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }

    ::IceInternal::ProxyHandle<RoomFinderMessage> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RoomFinderMessage*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<RoomFinderMessage*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }

    ::IceInternal::ProxyHandle<RoomFinderMessage> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RoomFinderMessage*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<RoomFinderMessage*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }

    ::IceInternal::ProxyHandle<RoomFinderMessage> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RoomFinderMessage*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<RoomFinderMessage*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<RoomFinderMessage> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RoomFinderMessage*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<RoomFinderMessage*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<RoomFinderMessage> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RoomFinderMessage*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<RoomFinderMessage*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<RoomFinderMessage> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RoomFinderMessage*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<RoomFinderMessage*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }

    ::IceInternal::ProxyHandle<RoomFinderMessage> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RoomFinderMessage*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<RoomFinderMessage*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }

    ::IceInternal::ProxyHandle<RoomFinderMessage> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RoomFinderMessage*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<RoomFinderMessage*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }

    ::IceInternal::ProxyHandle<RoomFinderMessage> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RoomFinderMessage*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<RoomFinderMessage*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }

    ::IceInternal::ProxyHandle<RoomFinderMessage> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<RoomFinderMessage*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<RoomFinderMessage*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }

    static const ::std::string& ice_staticId();

private:

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class SensorDataMessage : virtual public ::IceProxy::HawkMessages::Message
{
public:

    ::IceInternal::ProxyHandle<SensorDataMessage> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SensorDataMessage*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<SensorDataMessage*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }

    ::IceInternal::ProxyHandle<SensorDataMessage> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SensorDataMessage*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<SensorDataMessage*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }

    ::IceInternal::ProxyHandle<SensorDataMessage> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SensorDataMessage*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<SensorDataMessage*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }

    ::IceInternal::ProxyHandle<SensorDataMessage> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SensorDataMessage*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<SensorDataMessage*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }

    ::IceInternal::ProxyHandle<SensorDataMessage> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SensorDataMessage*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<SensorDataMessage*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }

    ::IceInternal::ProxyHandle<SensorDataMessage> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SensorDataMessage*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<SensorDataMessage*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }

    ::IceInternal::ProxyHandle<SensorDataMessage> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SensorDataMessage*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<SensorDataMessage*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }

    ::IceInternal::ProxyHandle<SensorDataMessage> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SensorDataMessage*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<SensorDataMessage*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }

    ::IceInternal::ProxyHandle<SensorDataMessage> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SensorDataMessage*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<SensorDataMessage*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }

    ::IceInternal::ProxyHandle<SensorDataMessage> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SensorDataMessage*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<SensorDataMessage*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }

    ::IceInternal::ProxyHandle<SensorDataMessage> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SensorDataMessage*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<SensorDataMessage*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }

    ::IceInternal::ProxyHandle<SensorDataMessage> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SensorDataMessage*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<SensorDataMessage*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<SensorDataMessage> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SensorDataMessage*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<SensorDataMessage*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<SensorDataMessage> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SensorDataMessage*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<SensorDataMessage*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<SensorDataMessage> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SensorDataMessage*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<SensorDataMessage*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }

    ::IceInternal::ProxyHandle<SensorDataMessage> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SensorDataMessage*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<SensorDataMessage*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }

    ::IceInternal::ProxyHandle<SensorDataMessage> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SensorDataMessage*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<SensorDataMessage*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }

    ::IceInternal::ProxyHandle<SensorDataMessage> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SensorDataMessage*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<SensorDataMessage*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }

    ::IceInternal::ProxyHandle<SensorDataMessage> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SensorDataMessage*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<SensorDataMessage*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }

    static const ::std::string& ice_staticId();

private:

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class SlamDataMessage : virtual public ::IceProxy::HawkMessages::Message
{
public:

    ::IceInternal::ProxyHandle<SlamDataMessage> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SlamDataMessage*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<SlamDataMessage*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }

    ::IceInternal::ProxyHandle<SlamDataMessage> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SlamDataMessage*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<SlamDataMessage*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }

    ::IceInternal::ProxyHandle<SlamDataMessage> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SlamDataMessage*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<SlamDataMessage*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }

    ::IceInternal::ProxyHandle<SlamDataMessage> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SlamDataMessage*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<SlamDataMessage*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }

    ::IceInternal::ProxyHandle<SlamDataMessage> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SlamDataMessage*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<SlamDataMessage*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }

    ::IceInternal::ProxyHandle<SlamDataMessage> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SlamDataMessage*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<SlamDataMessage*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }

    ::IceInternal::ProxyHandle<SlamDataMessage> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SlamDataMessage*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<SlamDataMessage*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }

    ::IceInternal::ProxyHandle<SlamDataMessage> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SlamDataMessage*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<SlamDataMessage*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }

    ::IceInternal::ProxyHandle<SlamDataMessage> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SlamDataMessage*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<SlamDataMessage*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }

    ::IceInternal::ProxyHandle<SlamDataMessage> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SlamDataMessage*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<SlamDataMessage*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }

    ::IceInternal::ProxyHandle<SlamDataMessage> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SlamDataMessage*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<SlamDataMessage*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }

    ::IceInternal::ProxyHandle<SlamDataMessage> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SlamDataMessage*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<SlamDataMessage*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<SlamDataMessage> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SlamDataMessage*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<SlamDataMessage*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<SlamDataMessage> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SlamDataMessage*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<SlamDataMessage*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<SlamDataMessage> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SlamDataMessage*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<SlamDataMessage*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }

    ::IceInternal::ProxyHandle<SlamDataMessage> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SlamDataMessage*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<SlamDataMessage*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }

    ::IceInternal::ProxyHandle<SlamDataMessage> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SlamDataMessage*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<SlamDataMessage*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }

    ::IceInternal::ProxyHandle<SlamDataMessage> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SlamDataMessage*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<SlamDataMessage*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }

    ::IceInternal::ProxyHandle<SlamDataMessage> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SlamDataMessage*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<SlamDataMessage*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }

    static const ::std::string& ice_staticId();

private:

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

}

}

namespace IceDelegate
{

namespace HawkMessages
{

class Message : virtual public ::IceDelegate::Ice::Object
{
public:
};

class MessageAgent : virtual public ::IceDelegate::Ice::Object
{
public:

    virtual void catchMessage(const ::HawkMessages::MessagePtr&, const ::Ice::Context*) = 0;
};

class CameraImageMessage : virtual public ::IceDelegate::HawkMessages::Message
{
public:
};

class ControlCameraMessage : virtual public ::IceDelegate::HawkMessages::Message
{
public:
};

class ControlDriveVisionMessage : virtual public ::IceDelegate::HawkMessages::Message
{
public:
};

class ControlLandMessage : virtual public ::IceDelegate::HawkMessages::Message
{
public:
};

class ControlMoveMessage : virtual public ::IceDelegate::HawkMessages::Message
{
public:
};

class ControlRoomVisionMessage : virtual public ::IceDelegate::HawkMessages::Message
{
public:
};

class ControlTakeOffMessage : virtual public ::IceDelegate::HawkMessages::Message
{
public:
};

class DriveFinderMessage : virtual public ::IceDelegate::HawkMessages::Message
{
public:
};

class ExampleMessage : virtual public ::IceDelegate::HawkMessages::Message
{
public:
};

class ExecuteMissionMessage : virtual public ::IceDelegate::HawkMessages::Message
{
public:
};

class MissionListMessage : virtual public ::IceDelegate::HawkMessages::Message
{
public:
};

class RoomFinderMessage : virtual public ::IceDelegate::HawkMessages::Message
{
public:
};

class SensorDataMessage : virtual public ::IceDelegate::HawkMessages::Message
{
public:
};

class SlamDataMessage : virtual public ::IceDelegate::HawkMessages::Message
{
public:
};

}

}

namespace IceDelegateM
{

namespace HawkMessages
{

class Message : virtual public ::IceDelegate::HawkMessages::Message,
                virtual public ::IceDelegateM::Ice::Object
{
public:
};

class MessageAgent : virtual public ::IceDelegate::HawkMessages::MessageAgent,
                     virtual public ::IceDelegateM::Ice::Object
{
public:

    virtual void catchMessage(const ::HawkMessages::MessagePtr&, const ::Ice::Context*);
};

class CameraImageMessage : virtual public ::IceDelegate::HawkMessages::CameraImageMessage,
                           virtual public ::IceDelegateM::HawkMessages::Message
{
public:
};

class ControlCameraMessage : virtual public ::IceDelegate::HawkMessages::ControlCameraMessage,
                             virtual public ::IceDelegateM::HawkMessages::Message
{
public:
};

class ControlDriveVisionMessage : virtual public ::IceDelegate::HawkMessages::ControlDriveVisionMessage,
                                  virtual public ::IceDelegateM::HawkMessages::Message
{
public:
};

class ControlLandMessage : virtual public ::IceDelegate::HawkMessages::ControlLandMessage,
                           virtual public ::IceDelegateM::HawkMessages::Message
{
public:
};

class ControlMoveMessage : virtual public ::IceDelegate::HawkMessages::ControlMoveMessage,
                           virtual public ::IceDelegateM::HawkMessages::Message
{
public:
};

class ControlRoomVisionMessage : virtual public ::IceDelegate::HawkMessages::ControlRoomVisionMessage,
                                 virtual public ::IceDelegateM::HawkMessages::Message
{
public:
};

class ControlTakeOffMessage : virtual public ::IceDelegate::HawkMessages::ControlTakeOffMessage,
                              virtual public ::IceDelegateM::HawkMessages::Message
{
public:
};

class DriveFinderMessage : virtual public ::IceDelegate::HawkMessages::DriveFinderMessage,
                           virtual public ::IceDelegateM::HawkMessages::Message
{
public:
};

class ExampleMessage : virtual public ::IceDelegate::HawkMessages::ExampleMessage,
                       virtual public ::IceDelegateM::HawkMessages::Message
{
public:
};

class ExecuteMissionMessage : virtual public ::IceDelegate::HawkMessages::ExecuteMissionMessage,
                              virtual public ::IceDelegateM::HawkMessages::Message
{
public:
};

class MissionListMessage : virtual public ::IceDelegate::HawkMessages::MissionListMessage,
                           virtual public ::IceDelegateM::HawkMessages::Message
{
public:
};

class RoomFinderMessage : virtual public ::IceDelegate::HawkMessages::RoomFinderMessage,
                          virtual public ::IceDelegateM::HawkMessages::Message
{
public:
};

class SensorDataMessage : virtual public ::IceDelegate::HawkMessages::SensorDataMessage,
                          virtual public ::IceDelegateM::HawkMessages::Message
{
public:
};

class SlamDataMessage : virtual public ::IceDelegate::HawkMessages::SlamDataMessage,
                        virtual public ::IceDelegateM::HawkMessages::Message
{
public:
};

}

}

namespace IceDelegateD
{

namespace HawkMessages
{

class Message : virtual public ::IceDelegate::HawkMessages::Message,
                virtual public ::IceDelegateD::Ice::Object
{
public:
};

class MessageAgent : virtual public ::IceDelegate::HawkMessages::MessageAgent,
                     virtual public ::IceDelegateD::Ice::Object
{
public:

    virtual void catchMessage(const ::HawkMessages::MessagePtr&, const ::Ice::Context*);
};

class CameraImageMessage : virtual public ::IceDelegate::HawkMessages::CameraImageMessage,
                           virtual public ::IceDelegateD::HawkMessages::Message
{
public:
};

class ControlCameraMessage : virtual public ::IceDelegate::HawkMessages::ControlCameraMessage,
                             virtual public ::IceDelegateD::HawkMessages::Message
{
public:
};

class ControlDriveVisionMessage : virtual public ::IceDelegate::HawkMessages::ControlDriveVisionMessage,
                                  virtual public ::IceDelegateD::HawkMessages::Message
{
public:
};

class ControlLandMessage : virtual public ::IceDelegate::HawkMessages::ControlLandMessage,
                           virtual public ::IceDelegateD::HawkMessages::Message
{
public:
};

class ControlMoveMessage : virtual public ::IceDelegate::HawkMessages::ControlMoveMessage,
                           virtual public ::IceDelegateD::HawkMessages::Message
{
public:
};

class ControlRoomVisionMessage : virtual public ::IceDelegate::HawkMessages::ControlRoomVisionMessage,
                                 virtual public ::IceDelegateD::HawkMessages::Message
{
public:
};

class ControlTakeOffMessage : virtual public ::IceDelegate::HawkMessages::ControlTakeOffMessage,
                              virtual public ::IceDelegateD::HawkMessages::Message
{
public:
};

class DriveFinderMessage : virtual public ::IceDelegate::HawkMessages::DriveFinderMessage,
                           virtual public ::IceDelegateD::HawkMessages::Message
{
public:
};

class ExampleMessage : virtual public ::IceDelegate::HawkMessages::ExampleMessage,
                       virtual public ::IceDelegateD::HawkMessages::Message
{
public:
};

class ExecuteMissionMessage : virtual public ::IceDelegate::HawkMessages::ExecuteMissionMessage,
                              virtual public ::IceDelegateD::HawkMessages::Message
{
public:
};

class MissionListMessage : virtual public ::IceDelegate::HawkMessages::MissionListMessage,
                           virtual public ::IceDelegateD::HawkMessages::Message
{
public:
};

class RoomFinderMessage : virtual public ::IceDelegate::HawkMessages::RoomFinderMessage,
                          virtual public ::IceDelegateD::HawkMessages::Message
{
public:
};

class SensorDataMessage : virtual public ::IceDelegate::HawkMessages::SensorDataMessage,
                          virtual public ::IceDelegateD::HawkMessages::Message
{
public:
};

class SlamDataMessage : virtual public ::IceDelegate::HawkMessages::SlamDataMessage,
                        virtual public ::IceDelegateD::HawkMessages::Message
{
public:
};

}

}

namespace HawkMessages
{

class Message : virtual public ::Ice::Object
{
public:

    typedef MessagePrx ProxyType;
    typedef MessagePtr PointerType;

    Message() {}
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);

    static const ::Ice::ObjectFactoryPtr& ice_factory();

protected:

    virtual ~Message() {}

    friend class Message__staticInit;
};

class Message__staticInit
{
public:

    ::HawkMessages::Message _init;
};

static ::HawkMessages::Message__staticInit _Message_init;

class MessageAgent : virtual public ::Ice::Object
{
public:

    typedef MessageAgentPrx ProxyType;
    typedef MessageAgentPtr PointerType;

    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual void catchMessage(const ::HawkMessages::MessagePtr&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___catchMessage(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::Ice::DispatchStatus __dispatch(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);
};

class CameraImageMessage : virtual public ::HawkMessages::Message
{
public:

    typedef CameraImageMessagePrx ProxyType;
    typedef CameraImageMessagePtr PointerType;

    CameraImageMessage() {}
    CameraImageMessage(const ::std::string&, const ::std::string&, const ::ImageIceMod::ImageIce&);
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();


    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);

    static const ::Ice::ObjectFactoryPtr& ice_factory();

protected:

    virtual ~CameraImageMessage() {}

public:

    ::std::string cameraID;

    ::std::string compression;

    ::ImageIceMod::ImageIce image;
};

class ControlCameraMessage : virtual public ::HawkMessages::Message
{
public:

    typedef ControlCameraMessagePrx ProxyType;
    typedef ControlCameraMessagePtr PointerType;

    ControlCameraMessage() {}
    ControlCameraMessage(const ::std::string&, const ::std::string&, ::Ice::Int, bool);
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();


    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);

    static const ::Ice::ObjectFactoryPtr& ice_factory();

protected:

    virtual ~ControlCameraMessage() {}

public:

    ::std::string cameraID;

    ::std::string compression;

    ::Ice::Int fps;

    bool cameraOn;
};

class ControlDriveVisionMessage : virtual public ::HawkMessages::Message
{
public:

    typedef ControlDriveVisionMessagePrx ProxyType;
    typedef ControlDriveVisionMessagePtr PointerType;

    ControlDriveVisionMessage() {}
    explicit ControlDriveVisionMessage(bool);
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();


    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);

    static const ::Ice::ObjectFactoryPtr& ice_factory();

protected:

    virtual ~ControlDriveVisionMessage() {}

public:

    bool driveVisionOn;
};

class ControlLandMessage : virtual public ::HawkMessages::Message
{
public:

    typedef ControlLandMessagePrx ProxyType;
    typedef ControlLandMessagePtr PointerType;

    ControlLandMessage() {}
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);

    static const ::Ice::ObjectFactoryPtr& ice_factory();

protected:

    virtual ~ControlLandMessage() {}
};

class ControlMoveMessage : virtual public ::HawkMessages::Message
{
public:

    typedef ControlMoveMessagePrx ProxyType;
    typedef ControlMoveMessagePtr PointerType;

    ControlMoveMessage() {}
    explicit ControlMoveMessage(const ::HawkMessages::Pose&);
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();


    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);

    static const ::Ice::ObjectFactoryPtr& ice_factory();

protected:

    virtual ~ControlMoveMessage() {}

public:

    ::HawkMessages::Pose move;
};

class ControlRoomVisionMessage : virtual public ::HawkMessages::Message
{
public:

    typedef ControlRoomVisionMessagePrx ProxyType;
    typedef ControlRoomVisionMessagePtr PointerType;

    ControlRoomVisionMessage() {}
    explicit ControlRoomVisionMessage(bool);
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();


    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);

    static const ::Ice::ObjectFactoryPtr& ice_factory();

protected:

    virtual ~ControlRoomVisionMessage() {}

public:

    bool roomVisionOn;
};

class ControlTakeOffMessage : virtual public ::HawkMessages::Message
{
public:

    typedef ControlTakeOffMessagePrx ProxyType;
    typedef ControlTakeOffMessagePtr PointerType;

    ControlTakeOffMessage() {}
    explicit ControlTakeOffMessage(::Ice::Double);
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();


    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);

    static const ::Ice::ObjectFactoryPtr& ice_factory();

protected:

    virtual ~ControlTakeOffMessage() {}

public:

    ::Ice::Double altitude;
};

class DriveFinderMessage : virtual public ::HawkMessages::Message
{
public:

    typedef DriveFinderMessagePrx ProxyType;
    typedef DriveFinderMessagePtr PointerType;

    DriveFinderMessage() {}
    DriveFinderMessage(bool, const ::HawkMessages::Pose&);
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();


    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);

    static const ::Ice::ObjectFactoryPtr& ice_factory();

protected:

    virtual ~DriveFinderMessage() {}

public:

    bool driveFound;

    ::HawkMessages::Pose drivePose;
};

class ExampleMessage : virtual public ::HawkMessages::Message
{
public:

    typedef ExampleMessagePrx ProxyType;
    typedef ExampleMessagePtr PointerType;

    ExampleMessage() {}
    ExampleMessage(const ::std::string&, const ::std::string&);
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();


    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);

    static const ::Ice::ObjectFactoryPtr& ice_factory();

protected:

    virtual ~ExampleMessage() {}

public:

    ::std::string name;

    ::std::string chatter;
};

class ExecuteMissionMessage : virtual public ::HawkMessages::Message
{
public:

    typedef ExecuteMissionMessagePrx ProxyType;
    typedef ExecuteMissionMessagePtr PointerType;

    ExecuteMissionMessage() {}
    explicit ExecuteMissionMessage(const ::std::string&);
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();


    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);

    static const ::Ice::ObjectFactoryPtr& ice_factory();

protected:

    virtual ~ExecuteMissionMessage() {}

public:

    ::std::string mission;
};

class MissionListMessage : virtual public ::HawkMessages::Message
{
public:

    typedef MissionListMessagePrx ProxyType;
    typedef MissionListMessagePtr PointerType;

    MissionListMessage() {}
    explicit MissionListMessage(const ::HawkMessages::StringSeq&);
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();


    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);

    static const ::Ice::ObjectFactoryPtr& ice_factory();

protected:

    virtual ~MissionListMessage() {}

public:

    ::HawkMessages::StringSeq missions;
};

class RoomFinderMessage : virtual public ::HawkMessages::Message
{
public:

    typedef RoomFinderMessagePrx ProxyType;
    typedef RoomFinderMessagePtr PointerType;

    RoomFinderMessage() {}
    RoomFinderMessage(bool, const ::HawkMessages::Pose&);
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();


    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);

    static const ::Ice::ObjectFactoryPtr& ice_factory();

protected:

    virtual ~RoomFinderMessage() {}

public:

    bool roomFound;

    ::HawkMessages::Pose roomPose;
};

class SensorDataMessage : virtual public ::HawkMessages::Message
{
public:

    typedef SensorDataMessagePrx ProxyType;
    typedef SensorDataMessagePtr PointerType;

    SensorDataMessage() {}
    SensorDataMessage(const ::HawkMessages::Pose&, ::Ice::Double, ::Ice::Double, bool, ::Ice::Double, ::Ice::Double, ::Ice::Double, bool, ::Ice::Double, ::Ice::Double, ::Ice::Double, bool, ::Ice::Double, ::Ice::Double, ::Ice::Double, bool, ::Ice::Double, ::Ice::Double, ::Ice::Double, ::Ice::Double, ::Ice::Double, const ::HawkMessages::LongSeq&, const ::HawkMessages::SonarSeq&);
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();


    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);

    static const ::Ice::ObjectFactoryPtr& ice_factory();

protected:

    virtual ~SensorDataMessage() {}

public:

    ::HawkMessages::Pose attemptedMove;

    ::Ice::Double motorSpeeds;

    ::Ice::Double batterVoltage;

    bool validAccel;

    ::Ice::Double accelX;

    ::Ice::Double accelY;

    ::Ice::Double accelZ;

    bool validGyro;

    ::Ice::Double gyroX;

    ::Ice::Double gyroY;

    ::Ice::Double gyroZ;

    bool validMag;

    ::Ice::Double magX;

    ::Ice::Double magY;

    ::Ice::Double magZ;

    bool validAhr;

    ::Ice::Double roll;

    ::Ice::Double pitch;

    ::Ice::Double yaw;

    ::Ice::Double heading;

    ::Ice::Double angularResolution;

    ::HawkMessages::LongSeq scannerData;

    ::HawkMessages::SonarSeq sonarData;
};

class SlamDataMessage : virtual public ::HawkMessages::Message
{
public:

    typedef SlamDataMessagePrx ProxyType;
    typedef SlamDataMessagePtr PointerType;

    SlamDataMessage() {}
    explicit SlamDataMessage(const ::HawkMessages::Pose&);
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();


    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);

    static const ::Ice::ObjectFactoryPtr& ice_factory();

protected:

    virtual ~SlamDataMessage() {}

public:

    ::HawkMessages::Pose hawkPose;
};

}

#endif
