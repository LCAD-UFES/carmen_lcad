// **********************************************************************
//
// Copyright (c) 2003-2007 ZeroC, Inc. All rights reserved.
//
// This copy of Ice-E is licensed to you under the terms described in the
// ICEE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice-E version 1.3.0
// Generated from file `SimpleRobotSimEvents.ice'

#ifndef __SimpleRobotSimEvents_h__
#define __SimpleRobotSimEvents_h__

#include <IceE/ProxyF.h>
#include <IceE/ObjectF.h>
#include <IceE/Exception.h>
#include <IceE/ScopedArray.h>
#include <IceE/Proxy.h>
#include <IceE/Object.h>
#ifndef ICEE_PURE_CLIENT
#  include <IceE/Incoming.h>
#endif
#include <IceE/Outgoing.h>
#include <IceE/FactoryTable.h>
#include <ImageIce.h>
#include <IceE/UndefSysMacros.h>

#ifndef ICEE_IGNORE_VERSION
#   if ICEE_INT_VERSION / 100 != 103
#       error IceE version mismatch!
#   endif
#   if ICEE_INT_VERSION % 100 < 0
#       error IceE patch level mismatch!
#   endif
#endif

namespace IceProxy
{

namespace RobotSimEvents
{

class EventMessage;

class Events;

class RetinaMessage;

class CameraConfigMessage;

}

}

namespace RobotSimEvents
{

class EventMessage;
bool operator==(const EventMessage&, const EventMessage&);
bool operator<(const EventMessage&, const EventMessage&);

class Events;
bool operator==(const Events&, const Events&);
bool operator<(const Events&, const Events&);

class RetinaMessage;
bool operator==(const RetinaMessage&, const RetinaMessage&);
bool operator<(const RetinaMessage&, const RetinaMessage&);

class CameraConfigMessage;
bool operator==(const CameraConfigMessage&, const CameraConfigMessage&);
bool operator<(const CameraConfigMessage&, const CameraConfigMessage&);

}

namespace IceInternal
{

::Ice::Object* upCast(::RobotSimEvents::EventMessage*);
::IceProxy::Ice::Object* upCast(::IceProxy::RobotSimEvents::EventMessage*);

::Ice::Object* upCast(::RobotSimEvents::Events*);
::IceProxy::Ice::Object* upCast(::IceProxy::RobotSimEvents::Events*);

::Ice::Object* upCast(::RobotSimEvents::RetinaMessage*);
::IceProxy::Ice::Object* upCast(::IceProxy::RobotSimEvents::RetinaMessage*);

::Ice::Object* upCast(::RobotSimEvents::CameraConfigMessage*);
::IceProxy::Ice::Object* upCast(::IceProxy::RobotSimEvents::CameraConfigMessage*);

}

namespace RobotSimEvents
{

typedef ::IceInternal::Handle< ::RobotSimEvents::EventMessage> EventMessagePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::RobotSimEvents::EventMessage> EventMessagePrx;

void __read(::IceInternal::BasicStream*, EventMessagePrx&);
void __patch__EventMessagePtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::RobotSimEvents::Events> EventsPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::RobotSimEvents::Events> EventsPrx;

void __read(::IceInternal::BasicStream*, EventsPrx&);
void __patch__EventsPtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::RobotSimEvents::RetinaMessage> RetinaMessagePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::RobotSimEvents::RetinaMessage> RetinaMessagePrx;

void __read(::IceInternal::BasicStream*, RetinaMessagePrx&);
void __patch__RetinaMessagePtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::RobotSimEvents::CameraConfigMessage> CameraConfigMessagePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::RobotSimEvents::CameraConfigMessage> CameraConfigMessagePrx;

void __read(::IceInternal::BasicStream*, CameraConfigMessagePrx&);
void __patch__CameraConfigMessagePtr(void*, ::Ice::ObjectPtr&);

}

namespace RobotSimEvents
{

}

namespace RobotSimEvents
{

class EventMessage : virtual public ::Ice::Object
{
public:

    typedef EventMessagePrx ProxyType;
    typedef EventMessagePtr PointerType;

    EventMessage() {}

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);

    static const ::Ice::ObjectFactoryPtr& ice_factory();

protected:

    virtual ~EventMessage() {}
};

class Events : virtual public ::Ice::Object
{
public:

    typedef EventsPrx ProxyType;
    typedef EventsPtr PointerType;


    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual void updateMessage(const ::RobotSimEvents::EventMessagePtr&, const ::Ice::Current& = ::Ice::Current()) = 0;
#ifndef ICEE_PURE_CLIENT
    ::Ice::DispatchStatus ___updateMessage(::IceInternal::Incoming&, const ::Ice::Current&);
#endif // ICEE_PURE_CLIENT

#ifndef ICEE_PURE_CLIENT
    virtual ::Ice::DispatchStatus __dispatch(::IceInternal::Incoming&, const ::Ice::Current&);
#endif // ICEE_PURE_CLIENT

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
};

class RetinaMessage : virtual public ::RobotSimEvents::EventMessage
{
public:

    typedef RetinaMessagePrx ProxyType;
    typedef RetinaMessagePtr PointerType;

    RetinaMessage() {}
    RetinaMessage(const ::ImageIceMod::ImageIce&, const ::std::string&);

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();


    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);

    static const ::Ice::ObjectFactoryPtr& ice_factory();

protected:

    virtual ~RetinaMessage() {}


public:

    ::ImageIceMod::ImageIce img;
    ::std::string cameraID;
};

class CameraConfigMessage : virtual public ::RobotSimEvents::EventMessage
{
public:

    typedef CameraConfigMessagePrx ProxyType;
    typedef CameraConfigMessagePtr PointerType;

    CameraConfigMessage() {}
    CameraConfigMessage(const ::std::string&, bool);

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();


    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);

    static const ::Ice::ObjectFactoryPtr& ice_factory();

protected:

    virtual ~CameraConfigMessage() {}


public:

    ::std::string cameraID;
    bool active;
};

}

namespace IceProxy
{

namespace RobotSimEvents
{

class EventMessage : virtual public ::IceProxy::Ice::Object
{
public:

    ::IceInternal::ProxyHandle<EventMessage> ice_context(const ::Ice::Context& __context) const
    {
        return dynamic_cast<EventMessage*>(::IceProxy::Ice::Object::ice_context(__context).get());
    }

    ::IceInternal::ProxyHandle<EventMessage> ice_secure(bool __secure) const
    {
        return dynamic_cast<EventMessage*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    }

#ifdef ICEE_HAS_ROUTER
    ::IceInternal::ProxyHandle<EventMessage> ice_router(const ::Ice::RouterPrx& __router) const
    {
        return dynamic_cast<EventMessage*>(::IceProxy::Ice::Object::ice_router(__router).get());
    }
#endif // ICEE_HAS_ROUTER

#ifdef ICEE_HAS_LOCATOR
    ::IceInternal::ProxyHandle<EventMessage> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
        return dynamic_cast<EventMessage*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    }

    ::IceInternal::ProxyHandle<EventMessage> ice_adapterId(const std::string& __id) const
    {
        return dynamic_cast<EventMessage*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    }
#endif // ICEE_HAS_LOCATOR

    ::IceInternal::ProxyHandle<EventMessage> ice_twoway() const
    {
        return dynamic_cast<EventMessage*>(::IceProxy::Ice::Object::ice_twoway().get());
    }

    ::IceInternal::ProxyHandle<EventMessage> ice_oneway() const
    {
        return dynamic_cast<EventMessage*>(::IceProxy::Ice::Object::ice_oneway().get());
    }

    ::IceInternal::ProxyHandle<EventMessage> ice_batchOneway() const
    {
        return dynamic_cast<EventMessage*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    }

    ::IceInternal::ProxyHandle<EventMessage> ice_datagram() const
    {
        return dynamic_cast<EventMessage*>(::IceProxy::Ice::Object::ice_datagram().get());
    }

    ::IceInternal::ProxyHandle<EventMessage> ice_batchDatagram() const
    {
        return dynamic_cast<EventMessage*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    }

    ::IceInternal::ProxyHandle<EventMessage> ice_timeout(int __timeout) const
    {
        return dynamic_cast<EventMessage*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    }

    static const ::std::string& ice_staticId();

private:

    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class Events : virtual public ::IceProxy::Ice::Object
{
public:

    void updateMessage(const ::RobotSimEvents::EventMessagePtr& eMsg)
    {
        updateMessage(eMsg, 0);
    }
    void updateMessage(const ::RobotSimEvents::EventMessagePtr& eMsg, const ::Ice::Context& __ctx)
    {
        updateMessage(eMsg, &__ctx);
    }

private:

    void updateMessage(const ::RobotSimEvents::EventMessagePtr&, const ::Ice::Context*);

public:

    ::IceInternal::ProxyHandle<Events> ice_context(const ::Ice::Context& __context) const
    {
        return dynamic_cast<Events*>(::IceProxy::Ice::Object::ice_context(__context).get());
    }

    ::IceInternal::ProxyHandle<Events> ice_secure(bool __secure) const
    {
        return dynamic_cast<Events*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    }

#ifdef ICEE_HAS_ROUTER
    ::IceInternal::ProxyHandle<Events> ice_router(const ::Ice::RouterPrx& __router) const
    {
        return dynamic_cast<Events*>(::IceProxy::Ice::Object::ice_router(__router).get());
    }
#endif // ICEE_HAS_ROUTER

#ifdef ICEE_HAS_LOCATOR
    ::IceInternal::ProxyHandle<Events> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
        return dynamic_cast<Events*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    }

    ::IceInternal::ProxyHandle<Events> ice_adapterId(const std::string& __id) const
    {
        return dynamic_cast<Events*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    }
#endif // ICEE_HAS_LOCATOR

    ::IceInternal::ProxyHandle<Events> ice_twoway() const
    {
        return dynamic_cast<Events*>(::IceProxy::Ice::Object::ice_twoway().get());
    }

    ::IceInternal::ProxyHandle<Events> ice_oneway() const
    {
        return dynamic_cast<Events*>(::IceProxy::Ice::Object::ice_oneway().get());
    }

    ::IceInternal::ProxyHandle<Events> ice_batchOneway() const
    {
        return dynamic_cast<Events*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    }

    ::IceInternal::ProxyHandle<Events> ice_datagram() const
    {
        return dynamic_cast<Events*>(::IceProxy::Ice::Object::ice_datagram().get());
    }

    ::IceInternal::ProxyHandle<Events> ice_batchDatagram() const
    {
        return dynamic_cast<Events*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    }

    ::IceInternal::ProxyHandle<Events> ice_timeout(int __timeout) const
    {
        return dynamic_cast<Events*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    }

    static const ::std::string& ice_staticId();

private:

    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class RetinaMessage : virtual public ::IceProxy::RobotSimEvents::EventMessage
{
public:

    ::IceInternal::ProxyHandle<RetinaMessage> ice_context(const ::Ice::Context& __context) const
    {
        return dynamic_cast<RetinaMessage*>(::IceProxy::Ice::Object::ice_context(__context).get());
    }

    ::IceInternal::ProxyHandle<RetinaMessage> ice_secure(bool __secure) const
    {
        return dynamic_cast<RetinaMessage*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    }

#ifdef ICEE_HAS_ROUTER
    ::IceInternal::ProxyHandle<RetinaMessage> ice_router(const ::Ice::RouterPrx& __router) const
    {
        return dynamic_cast<RetinaMessage*>(::IceProxy::Ice::Object::ice_router(__router).get());
    }
#endif // ICEE_HAS_ROUTER

#ifdef ICEE_HAS_LOCATOR
    ::IceInternal::ProxyHandle<RetinaMessage> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
        return dynamic_cast<RetinaMessage*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    }

    ::IceInternal::ProxyHandle<RetinaMessage> ice_adapterId(const std::string& __id) const
    {
        return dynamic_cast<RetinaMessage*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    }
#endif // ICEE_HAS_LOCATOR

    ::IceInternal::ProxyHandle<RetinaMessage> ice_twoway() const
    {
        return dynamic_cast<RetinaMessage*>(::IceProxy::Ice::Object::ice_twoway().get());
    }

    ::IceInternal::ProxyHandle<RetinaMessage> ice_oneway() const
    {
        return dynamic_cast<RetinaMessage*>(::IceProxy::Ice::Object::ice_oneway().get());
    }

    ::IceInternal::ProxyHandle<RetinaMessage> ice_batchOneway() const
    {
        return dynamic_cast<RetinaMessage*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    }

    ::IceInternal::ProxyHandle<RetinaMessage> ice_datagram() const
    {
        return dynamic_cast<RetinaMessage*>(::IceProxy::Ice::Object::ice_datagram().get());
    }

    ::IceInternal::ProxyHandle<RetinaMessage> ice_batchDatagram() const
    {
        return dynamic_cast<RetinaMessage*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    }

    ::IceInternal::ProxyHandle<RetinaMessage> ice_timeout(int __timeout) const
    {
        return dynamic_cast<RetinaMessage*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    }

    static const ::std::string& ice_staticId();

private:

    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class CameraConfigMessage : virtual public ::IceProxy::RobotSimEvents::EventMessage
{
public:

    ::IceInternal::ProxyHandle<CameraConfigMessage> ice_context(const ::Ice::Context& __context) const
    {
        return dynamic_cast<CameraConfigMessage*>(::IceProxy::Ice::Object::ice_context(__context).get());
    }

    ::IceInternal::ProxyHandle<CameraConfigMessage> ice_secure(bool __secure) const
    {
        return dynamic_cast<CameraConfigMessage*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    }

#ifdef ICEE_HAS_ROUTER
    ::IceInternal::ProxyHandle<CameraConfigMessage> ice_router(const ::Ice::RouterPrx& __router) const
    {
        return dynamic_cast<CameraConfigMessage*>(::IceProxy::Ice::Object::ice_router(__router).get());
    }
#endif // ICEE_HAS_ROUTER

#ifdef ICEE_HAS_LOCATOR
    ::IceInternal::ProxyHandle<CameraConfigMessage> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
        return dynamic_cast<CameraConfigMessage*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    }

    ::IceInternal::ProxyHandle<CameraConfigMessage> ice_adapterId(const std::string& __id) const
    {
        return dynamic_cast<CameraConfigMessage*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    }
#endif // ICEE_HAS_LOCATOR

    ::IceInternal::ProxyHandle<CameraConfigMessage> ice_twoway() const
    {
        return dynamic_cast<CameraConfigMessage*>(::IceProxy::Ice::Object::ice_twoway().get());
    }

    ::IceInternal::ProxyHandle<CameraConfigMessage> ice_oneway() const
    {
        return dynamic_cast<CameraConfigMessage*>(::IceProxy::Ice::Object::ice_oneway().get());
    }

    ::IceInternal::ProxyHandle<CameraConfigMessage> ice_batchOneway() const
    {
        return dynamic_cast<CameraConfigMessage*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    }

    ::IceInternal::ProxyHandle<CameraConfigMessage> ice_datagram() const
    {
        return dynamic_cast<CameraConfigMessage*>(::IceProxy::Ice::Object::ice_datagram().get());
    }

    ::IceInternal::ProxyHandle<CameraConfigMessage> ice_batchDatagram() const
    {
        return dynamic_cast<CameraConfigMessage*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    }

    ::IceInternal::ProxyHandle<CameraConfigMessage> ice_timeout(int __timeout) const
    {
        return dynamic_cast<CameraConfigMessage*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    }

    static const ::std::string& ice_staticId();

private:

    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

}

}

#endif
