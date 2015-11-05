// **********************************************************************
//
// Copyright (c) 2003-2008 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.0
// Generated from file `SeaBeeMessages.ice'

#ifndef __src_Robots_SeaBeeIII_Ice__SeaBeeMessages_ice_H__
#define __src_Robots_SeaBeeIII_Ice__SeaBeeMessages_ice_H__

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
#   if ICE_INT_VERSION % 100 < 0
#       error Ice patch level mismatch!
#   endif
#endif

namespace IceProxy
{

namespace SeaBeeSimEvents
{

class CameraConfigMessage;

}

}

namespace SeaBeeSimEvents
{

class CameraConfigMessage;
bool operator==(const CameraConfigMessage&, const CameraConfigMessage&);
bool operator<(const CameraConfigMessage&, const CameraConfigMessage&);

}

namespace IceInternal
{

::Ice::Object* upCast(::SeaBeeSimEvents::CameraConfigMessage*);
::IceProxy::Ice::Object* upCast(::IceProxy::SeaBeeSimEvents::CameraConfigMessage*);

}

namespace SeaBeeSimEvents
{

typedef ::IceInternal::Handle< ::SeaBeeSimEvents::CameraConfigMessage> CameraConfigMessagePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::SeaBeeSimEvents::CameraConfigMessage> CameraConfigMessagePrx;

void __read(::IceInternal::BasicStream*, CameraConfigMessagePrx&);
void __patch__CameraConfigMessagePtr(void*, ::Ice::ObjectPtr&);

}

namespace IceProxy
{

namespace SeaBeeSimEvents
{

class CameraConfigMessage : virtual public ::IceProxy::RobotSimEvents::EventMessage
{
public:

    ::IceInternal::ProxyHandle<CameraConfigMessage> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CameraConfigMessage*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<CameraConfigMessage*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }

    ::IceInternal::ProxyHandle<CameraConfigMessage> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CameraConfigMessage*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<CameraConfigMessage*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }

    ::IceInternal::ProxyHandle<CameraConfigMessage> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CameraConfigMessage*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<CameraConfigMessage*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }

    ::IceInternal::ProxyHandle<CameraConfigMessage> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CameraConfigMessage*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<CameraConfigMessage*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }

    ::IceInternal::ProxyHandle<CameraConfigMessage> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CameraConfigMessage*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<CameraConfigMessage*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }

    ::IceInternal::ProxyHandle<CameraConfigMessage> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CameraConfigMessage*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<CameraConfigMessage*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }

    ::IceInternal::ProxyHandle<CameraConfigMessage> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CameraConfigMessage*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<CameraConfigMessage*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }

    ::IceInternal::ProxyHandle<CameraConfigMessage> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CameraConfigMessage*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<CameraConfigMessage*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }

    ::IceInternal::ProxyHandle<CameraConfigMessage> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CameraConfigMessage*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<CameraConfigMessage*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }

    ::IceInternal::ProxyHandle<CameraConfigMessage> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CameraConfigMessage*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<CameraConfigMessage*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }

    ::IceInternal::ProxyHandle<CameraConfigMessage> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CameraConfigMessage*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<CameraConfigMessage*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }

    ::IceInternal::ProxyHandle<CameraConfigMessage> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CameraConfigMessage*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<CameraConfigMessage*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<CameraConfigMessage> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CameraConfigMessage*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<CameraConfigMessage*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<CameraConfigMessage> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CameraConfigMessage*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<CameraConfigMessage*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<CameraConfigMessage> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CameraConfigMessage*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<CameraConfigMessage*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }

    ::IceInternal::ProxyHandle<CameraConfigMessage> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CameraConfigMessage*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<CameraConfigMessage*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }

    ::IceInternal::ProxyHandle<CameraConfigMessage> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CameraConfigMessage*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<CameraConfigMessage*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }

    ::IceInternal::ProxyHandle<CameraConfigMessage> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CameraConfigMessage*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<CameraConfigMessage*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }

    ::IceInternal::ProxyHandle<CameraConfigMessage> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<CameraConfigMessage*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<CameraConfigMessage*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
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

namespace SeaBeeSimEvents
{

class CameraConfigMessage : virtual public ::IceDelegate::RobotSimEvents::EventMessage
{
public:
};

}

}

namespace IceDelegateM
{

namespace SeaBeeSimEvents
{

class CameraConfigMessage : virtual public ::IceDelegate::SeaBeeSimEvents::CameraConfigMessage,
                            virtual public ::IceDelegateM::RobotSimEvents::EventMessage
{
public:
};

}

}

namespace IceDelegateD
{

namespace SeaBeeSimEvents
{

class CameraConfigMessage : virtual public ::IceDelegate::SeaBeeSimEvents::CameraConfigMessage,
                            virtual public ::IceDelegateD::RobotSimEvents::EventMessage
{
public:
};

}

}

namespace SeaBeeSimEvents
{

class CameraConfigMessage : virtual public ::RobotSimEvents::EventMessage
{
public:

    typedef CameraConfigMessagePrx ProxyType;
    typedef CameraConfigMessagePtr PointerType;

    CameraConfigMessage() {}
    CameraConfigMessage(::Ice::Int, bool);
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

    virtual ~CameraConfigMessage() {}

    friend class CameraConfigMessage__staticInit;

public:

    ::Ice::Int cameraID;

    bool active;
};

class CameraConfigMessage__staticInit
{
public:

    ::SeaBeeSimEvents::CameraConfigMessage _init;
};

static ::SeaBeeSimEvents::CameraConfigMessage__staticInit _CameraConfigMessage_init;

}

#endif
