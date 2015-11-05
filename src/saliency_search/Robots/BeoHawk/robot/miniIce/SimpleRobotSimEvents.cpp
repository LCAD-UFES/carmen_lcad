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

#include <SimpleRobotSimEvents.h>
#include <IceE/LocalException.h>
#include <IceE/ObjectFactory.h>
#include <IceE/BasicStream.h>
#include <IceE/Iterator.h>

#ifndef ICEE_IGNORE_VERSION
#   if ICEE_INT_VERSION / 100 != 103
#       error IceE version mismatch!
#   endif
#   if ICEE_INT_VERSION % 100 < 0
#       error IceE patch level mismatch!
#   endif
#endif

static const ::std::string __RobotSimEvents__Events__updateMessage_name = "updateMessage";

::Ice::Object* IceInternal::upCast(::RobotSimEvents::EventMessage* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::RobotSimEvents::EventMessage* p) { return p; }

::Ice::Object* IceInternal::upCast(::RobotSimEvents::Events* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::RobotSimEvents::Events* p) { return p; }

::Ice::Object* IceInternal::upCast(::RobotSimEvents::RetinaMessage* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::RobotSimEvents::RetinaMessage* p) { return p; }

::Ice::Object* IceInternal::upCast(::RobotSimEvents::CameraConfigMessage* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::RobotSimEvents::CameraConfigMessage* p) { return p; }

void
RobotSimEvents::__read(::IceInternal::BasicStream* __is, ::RobotSimEvents::EventMessagePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::RobotSimEvents::EventMessage;
        v->__copyFrom(proxy);
    }
}

void
RobotSimEvents::__read(::IceInternal::BasicStream* __is, ::RobotSimEvents::EventsPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::RobotSimEvents::Events;
        v->__copyFrom(proxy);
    }
}

void
RobotSimEvents::__read(::IceInternal::BasicStream* __is, ::RobotSimEvents::RetinaMessagePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::RobotSimEvents::RetinaMessage;
        v->__copyFrom(proxy);
    }
}

void
RobotSimEvents::__read(::IceInternal::BasicStream* __is, ::RobotSimEvents::CameraConfigMessagePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::RobotSimEvents::CameraConfigMessage;
        v->__copyFrom(proxy);
    }
}

static const ::std::string __RobotSimEvents__EventMessage_ids[2] =
{
    "::Ice::Object",
    "::RobotSimEvents::EventMessage"
};

bool
RobotSimEvents::EventMessage::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__RobotSimEvents__EventMessage_ids, __RobotSimEvents__EventMessage_ids + 2, _s);
}

::std::vector< ::std::string>
RobotSimEvents::EventMessage::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__RobotSimEvents__EventMessage_ids[0], &__RobotSimEvents__EventMessage_ids[2]);
}

const ::std::string&
RobotSimEvents::EventMessage::ice_id(const ::Ice::Current&) const
{
    return __RobotSimEvents__EventMessage_ids[1];
}

const ::std::string&
RobotSimEvents::EventMessage::ice_staticId()
{
    return __RobotSimEvents__EventMessage_ids[1];
}

void
RobotSimEvents::EventMessage::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->endWriteSlice();
    ::Ice::Object::__write(__os);
}

void
RobotSimEvents::EventMessage::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->endReadSlice();
    ::Ice::Object::__read(__is, true);
}

class __F__RobotSimEvents__EventMessage : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::RobotSimEvents::EventMessage::ice_staticId());
        return new ::RobotSimEvents::EventMessage;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__RobotSimEvents__EventMessage_Ptr = new __F__RobotSimEvents__EventMessage;

const ::Ice::ObjectFactoryPtr&
RobotSimEvents::EventMessage::ice_factory()
{
    return __F__RobotSimEvents__EventMessage_Ptr;
}

class __F__RobotSimEvents__EventMessage__Init
{
public:

    __F__RobotSimEvents__EventMessage__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::RobotSimEvents::EventMessage::ice_staticId(), ::RobotSimEvents::EventMessage::ice_factory());
    }

    ~__F__RobotSimEvents__EventMessage__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::RobotSimEvents::EventMessage::ice_staticId());
    }
};

static __F__RobotSimEvents__EventMessage__Init __F__RobotSimEvents__EventMessage__i;

#ifdef __APPLE__
extern "C" { void __F__RobotSimEvents__EventMessage__initializer() {} }
#endif


bool
RobotSimEvents::operator==(const ::RobotSimEvents::EventMessage& l, const ::RobotSimEvents::EventMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
RobotSimEvents::operator<(const ::RobotSimEvents::EventMessage& l, const ::RobotSimEvents::EventMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

void
RobotSimEvents::__patch__EventMessagePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::RobotSimEvents::EventMessagePtr* p = static_cast< ::RobotSimEvents::EventMessagePtr*>(__addr);
    assert(p);
    *p = ::RobotSimEvents::EventMessagePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::RobotSimEvents::EventMessage::ice_staticId(), v->ice_id());
    }
}

static const ::std::string __RobotSimEvents__Events_ids[2] =
{
    "::Ice::Object",
    "::RobotSimEvents::Events"
};

bool
RobotSimEvents::Events::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__RobotSimEvents__Events_ids, __RobotSimEvents__Events_ids + 2, _s);
}

::std::vector< ::std::string>
RobotSimEvents::Events::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__RobotSimEvents__Events_ids[0], &__RobotSimEvents__Events_ids[2]);
}

const ::std::string&
RobotSimEvents::Events::ice_id(const ::Ice::Current&) const
{
    return __RobotSimEvents__Events_ids[1];
}

const ::std::string&
RobotSimEvents::Events::ice_staticId()
{
    return __RobotSimEvents__Events_ids[1];
}

#ifndef ICEE_PURE_CLIENT
::Ice::DispatchStatus
RobotSimEvents::Events::___updateMessage(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    ::RobotSimEvents::EventMessagePtr eMsg;
    __is->read(::RobotSimEvents::__patch__EventMessagePtr, &eMsg);
    __is->readPendingObjects();
    updateMessage(eMsg, __current);
    return ::Ice::DispatchOK;
}
#endif // ICEE_PURE_CLIENT

#ifndef ICEE_PURE_CLIENT
static ::std::string __RobotSimEvents__Events_all[] =
{
    "ice_id",
    "ice_ids",
    "ice_isA",
    "ice_ping",
    "updateMessage"
};

::Ice::DispatchStatus
RobotSimEvents::Events::__dispatch(::IceInternal::Incoming& in, const ::Ice::Current& current)
{
    ::std::pair< ::std::string*, ::std::string*> r = ::std::equal_range(__RobotSimEvents__Events_all, __RobotSimEvents__Events_all + 5, current.operation);
    if(r.first == r.second)
    {
        throw Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
    }

    switch(r.first - __RobotSimEvents__Events_all)
    {
        case 0:
        {
            return ___ice_id(in, current);
        }
        case 1:
        {
            return ___ice_ids(in, current);
        }
        case 2:
        {
            return ___ice_isA(in, current);
        }
        case 3:
        {
            return ___ice_ping(in, current);
        }
        case 4:
        {
            return ___updateMessage(in, current);
        }
    }

    assert(false);
    throw Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
}
#endif // ICEE_PURE_CLIENT

void
RobotSimEvents::Events::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->endWriteSlice();
    ::Ice::Object::__write(__os);
}

void
RobotSimEvents::Events::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->endReadSlice();
    ::Ice::Object::__read(__is, true);
}


bool
RobotSimEvents::operator==(const ::RobotSimEvents::Events& l, const ::RobotSimEvents::Events& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
RobotSimEvents::operator<(const ::RobotSimEvents::Events& l, const ::RobotSimEvents::Events& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

void
RobotSimEvents::__patch__EventsPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::RobotSimEvents::EventsPtr* p = static_cast< ::RobotSimEvents::EventsPtr*>(__addr);
    assert(p);
    *p = ::RobotSimEvents::EventsPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::RobotSimEvents::Events::ice_staticId(), v->ice_id());
    }
}

RobotSimEvents::RetinaMessage::RetinaMessage(const ::ImageIceMod::ImageIce& __ice_img, const ::std::string& __ice_cameraID) :
    img(__ice_img),
    cameraID(__ice_cameraID)
{
}

static const ::std::string __RobotSimEvents__RetinaMessage_ids[3] =
{
    "::Ice::Object",
    "::RobotSimEvents::EventMessage",
    "::RobotSimEvents::RetinaMessage"
};

bool
RobotSimEvents::RetinaMessage::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__RobotSimEvents__RetinaMessage_ids, __RobotSimEvents__RetinaMessage_ids + 3, _s);
}

::std::vector< ::std::string>
RobotSimEvents::RetinaMessage::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__RobotSimEvents__RetinaMessage_ids[0], &__RobotSimEvents__RetinaMessage_ids[3]);
}

const ::std::string&
RobotSimEvents::RetinaMessage::ice_id(const ::Ice::Current&) const
{
    return __RobotSimEvents__RetinaMessage_ids[2];
}

const ::std::string&
RobotSimEvents::RetinaMessage::ice_staticId()
{
    return __RobotSimEvents__RetinaMessage_ids[2];
}

void
RobotSimEvents::RetinaMessage::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    img.__write(__os);
    __os->write(cameraID);
    __os->endWriteSlice();
    ::RobotSimEvents::EventMessage::__write(__os);
}

void
RobotSimEvents::RetinaMessage::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    img.__read(__is);
    __is->read(cameraID);
    __is->endReadSlice();
    ::RobotSimEvents::EventMessage::__read(__is, true);
}

class __F__RobotSimEvents__RetinaMessage : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::RobotSimEvents::RetinaMessage::ice_staticId());
        return new ::RobotSimEvents::RetinaMessage;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__RobotSimEvents__RetinaMessage_Ptr = new __F__RobotSimEvents__RetinaMessage;

const ::Ice::ObjectFactoryPtr&
RobotSimEvents::RetinaMessage::ice_factory()
{
    return __F__RobotSimEvents__RetinaMessage_Ptr;
}

class __F__RobotSimEvents__RetinaMessage__Init
{
public:

    __F__RobotSimEvents__RetinaMessage__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::RobotSimEvents::RetinaMessage::ice_staticId(), ::RobotSimEvents::RetinaMessage::ice_factory());
    }

    ~__F__RobotSimEvents__RetinaMessage__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::RobotSimEvents::RetinaMessage::ice_staticId());
    }
};

static __F__RobotSimEvents__RetinaMessage__Init __F__RobotSimEvents__RetinaMessage__i;

#ifdef __APPLE__
extern "C" { void __F__RobotSimEvents__RetinaMessage__initializer() {} }
#endif


bool
RobotSimEvents::operator==(const ::RobotSimEvents::RetinaMessage& l, const ::RobotSimEvents::RetinaMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
RobotSimEvents::operator<(const ::RobotSimEvents::RetinaMessage& l, const ::RobotSimEvents::RetinaMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

void
RobotSimEvents::__patch__RetinaMessagePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::RobotSimEvents::RetinaMessagePtr* p = static_cast< ::RobotSimEvents::RetinaMessagePtr*>(__addr);
    assert(p);
    *p = ::RobotSimEvents::RetinaMessagePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::RobotSimEvents::RetinaMessage::ice_staticId(), v->ice_id());
    }
}

RobotSimEvents::CameraConfigMessage::CameraConfigMessage(const ::std::string& __ice_cameraID, bool __ice_active) :
    cameraID(__ice_cameraID),
    active(__ice_active)
{
}

static const ::std::string __RobotSimEvents__CameraConfigMessage_ids[3] =
{
    "::Ice::Object",
    "::RobotSimEvents::CameraConfigMessage",
    "::RobotSimEvents::EventMessage"
};

bool
RobotSimEvents::CameraConfigMessage::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__RobotSimEvents__CameraConfigMessage_ids, __RobotSimEvents__CameraConfigMessage_ids + 3, _s);
}

::std::vector< ::std::string>
RobotSimEvents::CameraConfigMessage::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__RobotSimEvents__CameraConfigMessage_ids[0], &__RobotSimEvents__CameraConfigMessage_ids[3]);
}

const ::std::string&
RobotSimEvents::CameraConfigMessage::ice_id(const ::Ice::Current&) const
{
    return __RobotSimEvents__CameraConfigMessage_ids[1];
}

const ::std::string&
RobotSimEvents::CameraConfigMessage::ice_staticId()
{
    return __RobotSimEvents__CameraConfigMessage_ids[1];
}

void
RobotSimEvents::CameraConfigMessage::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->write(cameraID);
    __os->write(active);
    __os->endWriteSlice();
    ::RobotSimEvents::EventMessage::__write(__os);
}

void
RobotSimEvents::CameraConfigMessage::__read(::IceInternal::BasicStream* __is, bool __rid)
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
    ::RobotSimEvents::EventMessage::__read(__is, true);
}

class __F__RobotSimEvents__CameraConfigMessage : public ::Ice::ObjectFactory
{
public:

    virtual ::Ice::ObjectPtr
    create(const ::std::string& type)
    {
        assert(type == ::RobotSimEvents::CameraConfigMessage::ice_staticId());
        return new ::RobotSimEvents::CameraConfigMessage;
    }

    virtual void
    destroy()
    {
    }
};

static ::Ice::ObjectFactoryPtr __F__RobotSimEvents__CameraConfigMessage_Ptr = new __F__RobotSimEvents__CameraConfigMessage;

const ::Ice::ObjectFactoryPtr&
RobotSimEvents::CameraConfigMessage::ice_factory()
{
    return __F__RobotSimEvents__CameraConfigMessage_Ptr;
}

class __F__RobotSimEvents__CameraConfigMessage__Init
{
public:

    __F__RobotSimEvents__CameraConfigMessage__Init()
    {
        ::IceInternal::factoryTable->addObjectFactory(::RobotSimEvents::CameraConfigMessage::ice_staticId(), ::RobotSimEvents::CameraConfigMessage::ice_factory());
    }

    ~__F__RobotSimEvents__CameraConfigMessage__Init()
    {
        ::IceInternal::factoryTable->removeObjectFactory(::RobotSimEvents::CameraConfigMessage::ice_staticId());
    }
};

static __F__RobotSimEvents__CameraConfigMessage__Init __F__RobotSimEvents__CameraConfigMessage__i;

#ifdef __APPLE__
extern "C" { void __F__RobotSimEvents__CameraConfigMessage__initializer() {} }
#endif


bool
RobotSimEvents::operator==(const ::RobotSimEvents::CameraConfigMessage& l, const ::RobotSimEvents::CameraConfigMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
RobotSimEvents::operator<(const ::RobotSimEvents::CameraConfigMessage& l, const ::RobotSimEvents::CameraConfigMessage& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

void
RobotSimEvents::__patch__CameraConfigMessagePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::RobotSimEvents::CameraConfigMessagePtr* p = static_cast< ::RobotSimEvents::CameraConfigMessagePtr*>(__addr);
    assert(p);
    *p = ::RobotSimEvents::CameraConfigMessagePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::RobotSimEvents::CameraConfigMessage::ice_staticId(), v->ice_id());
    }
}

const ::std::string&
IceProxy::RobotSimEvents::EventMessage::ice_staticId()
{
    return __RobotSimEvents__EventMessage_ids[1];
}

::IceProxy::Ice::Object*
IceProxy::RobotSimEvents::EventMessage::__newInstance() const
{
    return new EventMessage;
}

void
IceProxy::RobotSimEvents::Events::updateMessage(const ::RobotSimEvents::EventMessagePtr& eMsg, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::RequestHandlerPtr __handler;
        try
        {
            __handler = __getRequestHandler();
            ::IceInternal::Outgoing __outS(__handler.get(), _reference.get(), __RobotSimEvents__Events__updateMessage_name, ::Ice::Normal, __ctx);
            try
            {
                ::IceInternal::BasicStream* __os = __outS.os();
                __os->write(::Ice::ObjectPtr(::IceInternal::upCast(eMsg.get())));
                __os->writePendingObjects();
            }
            catch(const ::Ice::LocalException& __ex)
            {
                __outS.abort(__ex);
            }
            bool __ok = __outS.invoke();
            try
            {
                if(!__ok)
                {
                    __outS.is()->throwUnknownUserException();
                }
            }
            catch(const ::Ice::LocalException& __ex)
            {
                throw ::IceInternal::LocalExceptionWrapper(__ex, false);
            }
#if defined(_MSC_VER) && defined(_M_ARM) // ARM bug.
            catch(...)
            {
                throw;
            }
#endif
            return;
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__handler, __ex);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__handler, __ex, __cnt);
        }
#if defined(_MSC_VER) && defined(_M_ARM) // ARM bug.
        catch(...)
        {
            throw;
        }
#endif
    }
}

const ::std::string&
IceProxy::RobotSimEvents::Events::ice_staticId()
{
    return __RobotSimEvents__Events_ids[1];
}

::IceProxy::Ice::Object*
IceProxy::RobotSimEvents::Events::__newInstance() const
{
    return new Events;
}

const ::std::string&
IceProxy::RobotSimEvents::RetinaMessage::ice_staticId()
{
    return __RobotSimEvents__RetinaMessage_ids[2];
}

::IceProxy::Ice::Object*
IceProxy::RobotSimEvents::RetinaMessage::__newInstance() const
{
    return new RetinaMessage;
}

const ::std::string&
IceProxy::RobotSimEvents::CameraConfigMessage::ice_staticId()
{
    return __RobotSimEvents__CameraConfigMessage_ids[1];
}

::IceProxy::Ice::Object*
IceProxy::RobotSimEvents::CameraConfigMessage::__newInstance() const
{
    return new CameraConfigMessage;
}
