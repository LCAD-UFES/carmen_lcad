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

#include <SeaBeeMessages.ice.H>
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

::Ice::Object* IceInternal::upCast(::SeaBeeSimEvents::CameraConfigMessage* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::SeaBeeSimEvents::CameraConfigMessage* p) { return p; }

void
SeaBeeSimEvents::__read(::IceInternal::BasicStream* __is, ::SeaBeeSimEvents::CameraConfigMessagePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::SeaBeeSimEvents::CameraConfigMessage;
        v->__copyFrom(proxy);
    }
}

const ::std::string&
IceProxy::SeaBeeSimEvents::CameraConfigMessage::ice_staticId()
{
    return ::SeaBeeSimEvents::CameraConfigMessage::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::SeaBeeSimEvents::CameraConfigMessage::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::SeaBeeSimEvents::CameraConfigMessage);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::SeaBeeSimEvents::CameraConfigMessage::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::SeaBeeSimEvents::CameraConfigMessage);
}

::IceProxy::Ice::Object*
IceProxy::SeaBeeSimEvents::CameraConfigMessage::__newInstance() const
{
    return new CameraConfigMessage;
}

SeaBeeSimEvents::CameraConfigMessage::CameraConfigMessage(::Ice::Int __ice_cameraID, bool __ice_active) :
    cameraID(__ice_cameraID),
    active(__ice_active)
{
}

::Ice::ObjectPtr
SeaBeeSimEvents::CameraConfigMessage::ice_clone() const
{
    ::SeaBeeSimEvents::CameraConfigMessagePtr __p = new ::SeaBeeSimEvents::CameraConfigMessage(*this);
    return __p;
}

static const ::std::string __SeaBeeSimEvents__CameraConfigMessage_ids[3] =
{
    "::Ice::Object",
    "::RobotSimEvents::EventMessage",
    "::SeaBeeSimEvents::CameraConfigMessage"
};

bool
SeaBeeSimEvents::CameraConfigMessage::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__SeaBeeSimEvents__CameraConfigMessage_ids, __SeaBeeSimEvents__CameraConfigMessage_ids + 3, _s);
}

::std::vector< ::std::string>
SeaBeeSimEvents::CameraConfigMessage::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__SeaBeeSimEvents__CameraConfigMessage_ids[0], &__SeaBeeSimEvents__CameraConfigMessage_ids[3]);
}

const ::std::string&
SeaBeeSimEvents::CameraConfigMessage::ice_id(const ::Ice::Current&) const
{
    return __SeaBeeSimEvents__CameraConfigMessage_ids[2];
}

const ::std::string&
SeaBeeSimEvents::CameraConfigMessage::ice_staticId()
{
    return __SeaBeeSimEvents__CameraConfigMessage_ids[2];
}

void
SeaBeeSimEvents::CameraConfigMessage::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(cameraID);
    __os->write(active);
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    EventMessage::__write(__os);
#else
    ::RobotSimEvents::EventMessage::__write(__os);
#endif
}

void
SeaBeeSimEvents::CameraConfigMessage::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->read(cameraID);
    __is->read(active);
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    EventMessage::__read(__is, true);
#else
    ::RobotSimEvents::EventMessage::__read(__is, true);
#endif
}

void
SeaBeeSimEvents::CameraConfigMessage::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type SeaBeeSimEvents::CameraConfigMessage was not generated with stream support";
    throw ex;
}

void
SeaBeeSimEvents::CameraConfigMessage::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type SeaBeeSimEvents::CameraConfigMessage was not generated with stream support";
    throw ex;
}

class __F__SeaBeeSimEvents__CameraConfigMessage : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::SeaBeeSimEvents::CameraConfigMessage::ice_staticId());
        return new ::SeaBeeSimEvents::CameraConfigMessage;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__SeaBeeSimEvents__CameraConfigMessage_Ptr = new __F__SeaBeeSimEvents__CameraConfigMessage;

const ::Ice::ObjectFactoryPtr&
SeaBeeSimEvents::CameraConfigMessage::ice_factory()
{
    return __F__SeaBeeSimEvents__CameraConfigMessage_Ptr;
}

class __F__SeaBeeSimEvents__CameraConfigMessage__Init
{
public:

    __F__SeaBeeSimEvents__CameraConfigMessage__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::SeaBeeSimEvents::CameraConfigMessage::ice_staticId(), ::SeaBeeSimEvents::CameraConfigMessage::ice_factory());
    }

    ~__F__SeaBeeSimEvents__CameraConfigMessage__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::SeaBeeSimEvents::CameraConfigMessage::ice_staticId());
    }
};

static __F__SeaBeeSimEvents__CameraConfigMessage__Init __F__SeaBeeSimEvents__CameraConfigMessage__i;

#ifdef __APPLE__
extern "C" { void __F__SeaBeeSimEvents__CameraConfigMessage__initializer() {} }
#endif

void
SeaBeeSimEvents::__patch__CameraConfigMessagePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::SeaBeeSimEvents::CameraConfigMessagePtr* p = static_cast< ::SeaBeeSimEvents::CameraConfigMessagePtr*>(__addr);
    assert(p);
    *p = ::SeaBeeSimEvents::CameraConfigMessagePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::SeaBeeSimEvents::CameraConfigMessage::ice_staticId(), v->ice_id());
    }
}

bool
SeaBeeSimEvents::operator==(const ::SeaBeeSimEvents::CameraConfigMessage& l, const ::SeaBeeSimEvents::CameraConfigMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
SeaBeeSimEvents::operator<(const ::SeaBeeSimEvents::CameraConfigMessage& l, const ::SeaBeeSimEvents::CameraConfigMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}
