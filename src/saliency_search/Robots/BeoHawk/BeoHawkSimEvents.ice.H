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

#ifndef __src_Robots_BeoHawk__BeoHawkSimEvents_ice_H__
#define __src_Robots_BeoHawk__BeoHawkSimEvents_ice_H__

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
#include <Ice/RobotSimEvents.ice.H>
#include <Ice/UndefSysMacros.h>

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

namespace IceProxy
{

namespace RobotSimEvents
{

class BeoHawkEyeSpyMessage;

class ControlCameraMessage;

class CameraImageMessage;

class ControlDriveVisionMessage;

class ExecuteMissionMessage;

class SlamDataMessage;

class ControlLandMessage;

class ControlMoveMessage;

class SensorDataMessage;

}

}

namespace RobotSimEvents
{

class BeoHawkEyeSpyMessage;
bool operator==(const BeoHawkEyeSpyMessage&, const BeoHawkEyeSpyMessage&);
bool operator<(const BeoHawkEyeSpyMessage&, const BeoHawkEyeSpyMessage&);

class ControlCameraMessage;
bool operator==(const ControlCameraMessage&, const ControlCameraMessage&);
bool operator<(const ControlCameraMessage&, const ControlCameraMessage&);

class CameraImageMessage;
bool operator==(const CameraImageMessage&, const CameraImageMessage&);
bool operator<(const CameraImageMessage&, const CameraImageMessage&);

class ControlDriveVisionMessage;
bool operator==(const ControlDriveVisionMessage&, const ControlDriveVisionMessage&);
bool operator<(const ControlDriveVisionMessage&, const ControlDriveVisionMessage&);

class ExecuteMissionMessage;
bool operator==(const ExecuteMissionMessage&, const ExecuteMissionMessage&);
bool operator<(const ExecuteMissionMessage&, const ExecuteMissionMessage&);

class SlamDataMessage;
bool operator==(const SlamDataMessage&, const SlamDataMessage&);
bool operator<(const SlamDataMessage&, const SlamDataMessage&);

class ControlLandMessage;
bool operator==(const ControlLandMessage&, const ControlLandMessage&);
bool operator<(const ControlLandMessage&, const ControlLandMessage&);

class ControlMoveMessage;
bool operator==(const ControlMoveMessage&, const ControlMoveMessage&);
bool operator<(const ControlMoveMessage&, const ControlMoveMessage&);

class SensorDataMessage;
bool operator==(const SensorDataMessage&, const SensorDataMessage&);
bool operator<(const SensorDataMessage&, const SensorDataMessage&);

}

namespace IceInternal
{

::Ice::Object* upCast(::RobotSimEvents::BeoHawkEyeSpyMessage*);
::IceProxy::Ice::Object* upCast(::IceProxy::RobotSimEvents::BeoHawkEyeSpyMessage*);

::Ice::Object* upCast(::RobotSimEvents::ControlCameraMessage*);
::IceProxy::Ice::Object* upCast(::IceProxy::RobotSimEvents::ControlCameraMessage*);

::Ice::Object* upCast(::RobotSimEvents::CameraImageMessage*);
::IceProxy::Ice::Object* upCast(::IceProxy::RobotSimEvents::CameraImageMessage*);

::Ice::Object* upCast(::RobotSimEvents::ControlDriveVisionMessage*);
::IceProxy::Ice::Object* upCast(::IceProxy::RobotSimEvents::ControlDriveVisionMessage*);

::Ice::Object* upCast(::RobotSimEvents::ExecuteMissionMessage*);
::IceProxy::Ice::Object* upCast(::IceProxy::RobotSimEvents::ExecuteMissionMessage*);

::Ice::Object* upCast(::RobotSimEvents::SlamDataMessage*);
::IceProxy::Ice::Object* upCast(::IceProxy::RobotSimEvents::SlamDataMessage*);

::Ice::Object* upCast(::RobotSimEvents::ControlLandMessage*);
::IceProxy::Ice::Object* upCast(::IceProxy::RobotSimEvents::ControlLandMessage*);

::Ice::Object* upCast(::RobotSimEvents::ControlMoveMessage*);
::IceProxy::Ice::Object* upCast(::IceProxy::RobotSimEvents::ControlMoveMessage*);

::Ice::Object* upCast(::RobotSimEvents::SensorDataMessage*);
::IceProxy::Ice::Object* upCast(::IceProxy::RobotSimEvents::SensorDataMessage*);

}

namespace RobotSimEvents
{

typedef ::IceInternal::Handle< ::RobotSimEvents::BeoHawkEyeSpyMessage> BeoHawkEyeSpyMessagePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::RobotSimEvents::BeoHawkEyeSpyMessage> BeoHawkEyeSpyMessagePrx;

void __read(::IceInternal::BasicStream*, BeoHawkEyeSpyMessagePrx&);
void __patch__BeoHawkEyeSpyMessagePtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::RobotSimEvents::ControlCameraMessage> ControlCameraMessagePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::RobotSimEvents::ControlCameraMessage> ControlCameraMessagePrx;

void __read(::IceInternal::BasicStream*, ControlCameraMessagePrx&);
void __patch__ControlCameraMessagePtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::RobotSimEvents::CameraImageMessage> CameraImageMessagePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::RobotSimEvents::CameraImageMessage> CameraImageMessagePrx;

void __read(::IceInternal::BasicStream*, CameraImageMessagePrx&);
void __patch__CameraImageMessagePtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::RobotSimEvents::ControlDriveVisionMessage> ControlDriveVisionMessagePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::RobotSimEvents::ControlDriveVisionMessage> ControlDriveVisionMessagePrx;

void __read(::IceInternal::BasicStream*, ControlDriveVisionMessagePrx&);
void __patch__ControlDriveVisionMessagePtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::RobotSimEvents::ExecuteMissionMessage> ExecuteMissionMessagePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::RobotSimEvents::ExecuteMissionMessage> ExecuteMissionMessagePrx;

void __read(::IceInternal::BasicStream*, ExecuteMissionMessagePrx&);
void __patch__ExecuteMissionMessagePtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::RobotSimEvents::SlamDataMessage> SlamDataMessagePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::RobotSimEvents::SlamDataMessage> SlamDataMessagePrx;

void __read(::IceInternal::BasicStream*, SlamDataMessagePrx&);
void __patch__SlamDataMessagePtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::RobotSimEvents::ControlLandMessage> ControlLandMessagePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::RobotSimEvents::ControlLandMessage> ControlLandMessagePrx;

void __read(::IceInternal::BasicStream*, ControlLandMessagePrx&);
void __patch__ControlLandMessagePtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::RobotSimEvents::ControlMoveMessage> ControlMoveMessagePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::RobotSimEvents::ControlMoveMessage> ControlMoveMessagePrx;

void __read(::IceInternal::BasicStream*, ControlMoveMessagePrx&);
void __patch__ControlMoveMessagePtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::RobotSimEvents::SensorDataMessage> SensorDataMessagePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::RobotSimEvents::SensorDataMessage> SensorDataMessagePrx;

void __read(::IceInternal::BasicStream*, SensorDataMessagePrx&);
void __patch__SensorDataMessagePtr(void*, ::Ice::ObjectPtr&);

}

namespace RobotSimEvents
{

struct Location
{
    ::Ice::Double x;
    ::Ice::Double y;
    ::Ice::Double z;
    ::Ice::Double theta;

    bool operator==(const Location&) const;
    bool operator<(const Location&) const;
    bool operator!=(const Location& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const Location& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const Location& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const Location& __rhs) const
    {
        return !operator<(__rhs);
    }

    void __write(::IceInternal::BasicStream*) const;
    void __read(::IceInternal::BasicStream*);
};

struct Lrf
{
    ::Ice::Double angle;
    ::Ice::Double distance;

    bool operator==(const Lrf&) const;
    bool operator<(const Lrf&) const;
    bool operator!=(const Lrf& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const Lrf& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const Lrf& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const Lrf& __rhs) const
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

enum BeoHawkEyeSpyType
{
    SECURITYSIGN,
    USBKEY,
    RECTANGLE
};

void __write(::IceInternal::BasicStream*, BeoHawkEyeSpyType);
void __read(::IceInternal::BasicStream*, BeoHawkEyeSpyType&);

typedef ::std::vector< ::RobotSimEvents::Lrf> LrfSeq;
void __writeLrfSeq(::IceInternal::BasicStream*, const ::RobotSimEvents::Lrf*, const ::RobotSimEvents::Lrf*);
void __readLrfSeq(::IceInternal::BasicStream*, LrfSeq&);

typedef ::std::vector< ::RobotSimEvents::Sonar> SonarSeq;
void __writeSonarSeq(::IceInternal::BasicStream*, const ::RobotSimEvents::Sonar*, const ::RobotSimEvents::Sonar*);
void __readSonarSeq(::IceInternal::BasicStream*, SonarSeq&);

}

namespace IceProxy
{

namespace RobotSimEvents
{

class BeoHawkEyeSpyMessage : virtual public ::IceProxy::RobotSimEvents::EventMessage
{
public:

    ::IceInternal::ProxyHandle<BeoHawkEyeSpyMessage> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<BeoHawkEyeSpyMessage*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<BeoHawkEyeSpyMessage*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }

    ::IceInternal::ProxyHandle<BeoHawkEyeSpyMessage> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<BeoHawkEyeSpyMessage*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<BeoHawkEyeSpyMessage*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }

    ::IceInternal::ProxyHandle<BeoHawkEyeSpyMessage> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<BeoHawkEyeSpyMessage*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<BeoHawkEyeSpyMessage*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }

    ::IceInternal::ProxyHandle<BeoHawkEyeSpyMessage> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<BeoHawkEyeSpyMessage*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<BeoHawkEyeSpyMessage*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }

    ::IceInternal::ProxyHandle<BeoHawkEyeSpyMessage> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<BeoHawkEyeSpyMessage*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<BeoHawkEyeSpyMessage*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }

    ::IceInternal::ProxyHandle<BeoHawkEyeSpyMessage> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<BeoHawkEyeSpyMessage*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<BeoHawkEyeSpyMessage*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }

    ::IceInternal::ProxyHandle<BeoHawkEyeSpyMessage> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<BeoHawkEyeSpyMessage*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<BeoHawkEyeSpyMessage*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }

    ::IceInternal::ProxyHandle<BeoHawkEyeSpyMessage> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<BeoHawkEyeSpyMessage*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<BeoHawkEyeSpyMessage*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }

    ::IceInternal::ProxyHandle<BeoHawkEyeSpyMessage> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<BeoHawkEyeSpyMessage*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<BeoHawkEyeSpyMessage*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }

    ::IceInternal::ProxyHandle<BeoHawkEyeSpyMessage> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<BeoHawkEyeSpyMessage*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<BeoHawkEyeSpyMessage*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }

    ::IceInternal::ProxyHandle<BeoHawkEyeSpyMessage> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<BeoHawkEyeSpyMessage*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<BeoHawkEyeSpyMessage*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }

    ::IceInternal::ProxyHandle<BeoHawkEyeSpyMessage> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<BeoHawkEyeSpyMessage*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<BeoHawkEyeSpyMessage*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<BeoHawkEyeSpyMessage> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<BeoHawkEyeSpyMessage*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<BeoHawkEyeSpyMessage*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<BeoHawkEyeSpyMessage> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<BeoHawkEyeSpyMessage*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<BeoHawkEyeSpyMessage*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<BeoHawkEyeSpyMessage> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<BeoHawkEyeSpyMessage*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<BeoHawkEyeSpyMessage*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }

    ::IceInternal::ProxyHandle<BeoHawkEyeSpyMessage> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<BeoHawkEyeSpyMessage*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<BeoHawkEyeSpyMessage*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }

    ::IceInternal::ProxyHandle<BeoHawkEyeSpyMessage> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<BeoHawkEyeSpyMessage*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<BeoHawkEyeSpyMessage*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }

    ::IceInternal::ProxyHandle<BeoHawkEyeSpyMessage> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<BeoHawkEyeSpyMessage*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<BeoHawkEyeSpyMessage*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }

    ::IceInternal::ProxyHandle<BeoHawkEyeSpyMessage> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<BeoHawkEyeSpyMessage*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<BeoHawkEyeSpyMessage*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
    #endif
    }

    static const ::std::string& ice_staticId();

private:

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class ControlCameraMessage : virtual public ::IceProxy::RobotSimEvents::EventMessage
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

class CameraImageMessage : virtual public ::IceProxy::RobotSimEvents::EventMessage
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

class ControlDriveVisionMessage : virtual public ::IceProxy::RobotSimEvents::EventMessage
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

class ExecuteMissionMessage : virtual public ::IceProxy::RobotSimEvents::EventMessage
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

class SlamDataMessage : virtual public ::IceProxy::RobotSimEvents::EventMessage
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

class ControlLandMessage : virtual public ::IceProxy::RobotSimEvents::EventMessage
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

class ControlMoveMessage : virtual public ::IceProxy::RobotSimEvents::EventMessage
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

class SensorDataMessage : virtual public ::IceProxy::RobotSimEvents::EventMessage
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

}

}

namespace IceDelegate
{

namespace RobotSimEvents
{

class BeoHawkEyeSpyMessage : virtual public ::IceDelegate::RobotSimEvents::EventMessage
{
public:
};

class ControlCameraMessage : virtual public ::IceDelegate::RobotSimEvents::EventMessage
{
public:
};

class CameraImageMessage : virtual public ::IceDelegate::RobotSimEvents::EventMessage
{
public:
};

class ControlDriveVisionMessage : virtual public ::IceDelegate::RobotSimEvents::EventMessage
{
public:
};

class ExecuteMissionMessage : virtual public ::IceDelegate::RobotSimEvents::EventMessage
{
public:
};

class SlamDataMessage : virtual public ::IceDelegate::RobotSimEvents::EventMessage
{
public:
};

class ControlLandMessage : virtual public ::IceDelegate::RobotSimEvents::EventMessage
{
public:
};

class ControlMoveMessage : virtual public ::IceDelegate::RobotSimEvents::EventMessage
{
public:
};

class SensorDataMessage : virtual public ::IceDelegate::RobotSimEvents::EventMessage
{
public:
};

}

}

namespace IceDelegateM
{

namespace RobotSimEvents
{

class BeoHawkEyeSpyMessage : virtual public ::IceDelegate::RobotSimEvents::BeoHawkEyeSpyMessage,
                             virtual public ::IceDelegateM::RobotSimEvents::EventMessage
{
public:
};

class ControlCameraMessage : virtual public ::IceDelegate::RobotSimEvents::ControlCameraMessage,
                             virtual public ::IceDelegateM::RobotSimEvents::EventMessage
{
public:
};

class CameraImageMessage : virtual public ::IceDelegate::RobotSimEvents::CameraImageMessage,
                           virtual public ::IceDelegateM::RobotSimEvents::EventMessage
{
public:
};

class ControlDriveVisionMessage : virtual public ::IceDelegate::RobotSimEvents::ControlDriveVisionMessage,
                                  virtual public ::IceDelegateM::RobotSimEvents::EventMessage
{
public:
};

class ExecuteMissionMessage : virtual public ::IceDelegate::RobotSimEvents::ExecuteMissionMessage,
                              virtual public ::IceDelegateM::RobotSimEvents::EventMessage
{
public:
};

class SlamDataMessage : virtual public ::IceDelegate::RobotSimEvents::SlamDataMessage,
                        virtual public ::IceDelegateM::RobotSimEvents::EventMessage
{
public:
};

class ControlLandMessage : virtual public ::IceDelegate::RobotSimEvents::ControlLandMessage,
                           virtual public ::IceDelegateM::RobotSimEvents::EventMessage
{
public:
};

class ControlMoveMessage : virtual public ::IceDelegate::RobotSimEvents::ControlMoveMessage,
                           virtual public ::IceDelegateM::RobotSimEvents::EventMessage
{
public:
};

class SensorDataMessage : virtual public ::IceDelegate::RobotSimEvents::SensorDataMessage,
                          virtual public ::IceDelegateM::RobotSimEvents::EventMessage
{
public:
};

}

}

namespace IceDelegateD
{

namespace RobotSimEvents
{

class BeoHawkEyeSpyMessage : virtual public ::IceDelegate::RobotSimEvents::BeoHawkEyeSpyMessage,
                             virtual public ::IceDelegateD::RobotSimEvents::EventMessage
{
public:
};

class ControlCameraMessage : virtual public ::IceDelegate::RobotSimEvents::ControlCameraMessage,
                             virtual public ::IceDelegateD::RobotSimEvents::EventMessage
{
public:
};

class CameraImageMessage : virtual public ::IceDelegate::RobotSimEvents::CameraImageMessage,
                           virtual public ::IceDelegateD::RobotSimEvents::EventMessage
{
public:
};

class ControlDriveVisionMessage : virtual public ::IceDelegate::RobotSimEvents::ControlDriveVisionMessage,
                                  virtual public ::IceDelegateD::RobotSimEvents::EventMessage
{
public:
};

class ExecuteMissionMessage : virtual public ::IceDelegate::RobotSimEvents::ExecuteMissionMessage,
                              virtual public ::IceDelegateD::RobotSimEvents::EventMessage
{
public:
};

class SlamDataMessage : virtual public ::IceDelegate::RobotSimEvents::SlamDataMessage,
                        virtual public ::IceDelegateD::RobotSimEvents::EventMessage
{
public:
};

class ControlLandMessage : virtual public ::IceDelegate::RobotSimEvents::ControlLandMessage,
                           virtual public ::IceDelegateD::RobotSimEvents::EventMessage
{
public:
};

class ControlMoveMessage : virtual public ::IceDelegate::RobotSimEvents::ControlMoveMessage,
                           virtual public ::IceDelegateD::RobotSimEvents::EventMessage
{
public:
};

class SensorDataMessage : virtual public ::IceDelegate::RobotSimEvents::SensorDataMessage,
                          virtual public ::IceDelegateD::RobotSimEvents::EventMessage
{
public:
};

}

}

namespace RobotSimEvents
{

class BeoHawkEyeSpyMessage : virtual public ::RobotSimEvents::EventMessage
{
public:

    typedef BeoHawkEyeSpyMessagePrx ProxyType;
    typedef BeoHawkEyeSpyMessagePtr PointerType;

    BeoHawkEyeSpyMessage() {}
    BeoHawkEyeSpyMessage(::RobotSimEvents::BeoHawkEyeSpyType, const ::std::string&, const ::ImageIceMod::Point3DIce&, const ::ImageIceMod::Point3DIce&, const ::ImageIceMod::Point3DIce&, const ::ImageIceMod::Point3DIce&, ::Ice::Float);
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

    virtual ~BeoHawkEyeSpyMessage() {}

    friend class BeoHawkEyeSpyMessage__staticInit;

public:

    ::RobotSimEvents::BeoHawkEyeSpyType foundType;

    ::std::string cameraID;

    ::ImageIceMod::Point3DIce topLeft;

    ::ImageIceMod::Point3DIce topRight;

    ::ImageIceMod::Point3DIce bottomLeft;

    ::ImageIceMod::Point3DIce bottomRight;

    ::Ice::Float certaintyLevel;
};

class BeoHawkEyeSpyMessage__staticInit
{
public:

    ::RobotSimEvents::BeoHawkEyeSpyMessage _init;
};

static BeoHawkEyeSpyMessage__staticInit _BeoHawkEyeSpyMessage_init;

class ControlCameraMessage : virtual public ::RobotSimEvents::EventMessage
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

class CameraImageMessage : virtual public ::RobotSimEvents::EventMessage
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

    ::ImageIceMod::ImageIce img;
};

class ControlDriveVisionMessage : virtual public ::RobotSimEvents::EventMessage
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

    bool drivevisionOn;
};

class ExecuteMissionMessage : virtual public ::RobotSimEvents::EventMessage
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

class SlamDataMessage : virtual public ::RobotSimEvents::EventMessage
{
public:

    typedef SlamDataMessagePrx ProxyType;
    typedef SlamDataMessagePtr PointerType;

    SlamDataMessage() {}
    explicit SlamDataMessage(const ::RobotSimEvents::Location&);
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

    ::RobotSimEvents::Location lctn;
};

class ControlLandMessage : virtual public ::RobotSimEvents::EventMessage
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

class ControlMoveMessage : virtual public ::RobotSimEvents::EventMessage
{
public:

    typedef ControlMoveMessagePrx ProxyType;
    typedef ControlMoveMessagePtr PointerType;

    ControlMoveMessage() {}
    explicit ControlMoveMessage(const ::RobotSimEvents::Location&);
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

    ::RobotSimEvents::Location move;
};

class SensorDataMessage : virtual public ::RobotSimEvents::EventMessage
{
public:

    typedef SensorDataMessagePrx ProxyType;
    typedef SensorDataMessagePtr PointerType;

    SensorDataMessage() {}
    SensorDataMessage(::Ice::Double, bool, ::Ice::Double, ::Ice::Double, ::Ice::Double, const ::RobotSimEvents::LrfSeq&, const ::RobotSimEvents::SonarSeq&);
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

    ::Ice::Double motorSpeeds;

    bool validRollPitchYaw;

    ::Ice::Double pitch;

    ::Ice::Double yaw;

    ::Ice::Double absouteHeading;

    ::RobotSimEvents::LrfSeq lrf;

    ::RobotSimEvents::SonarSeq sonars;
};

}

#endif
