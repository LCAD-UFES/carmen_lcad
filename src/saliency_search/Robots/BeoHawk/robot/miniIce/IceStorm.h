// **********************************************************************
//
// Copyright (c) 2003-2007 ZeroC, Inc. All rights reserved.
//
// This copy of Ice-E is licensed to you under the terms described in the
// ICEE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice-E version 1.3.0
// Generated from file `IceStorm.ice'

#ifndef __IceStorm_h__
#define __IceStorm_h__

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
#include <IceE/UserExceptionFactory.h>
#include <IceE/FactoryTable.h>
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

namespace IceStorm
{

class Topic;

class TopicManager;

}

}

namespace IceStorm
{

class Topic;
bool operator==(const Topic&, const Topic&);
bool operator<(const Topic&, const Topic&);

class TopicManager;
bool operator==(const TopicManager&, const TopicManager&);
bool operator<(const TopicManager&, const TopicManager&);

}

namespace IceInternal
{

::Ice::Object* upCast(::IceStorm::Topic*);
::IceProxy::Ice::Object* upCast(::IceProxy::IceStorm::Topic*);

::Ice::Object* upCast(::IceStorm::TopicManager*);
::IceProxy::Ice::Object* upCast(::IceProxy::IceStorm::TopicManager*);

}

namespace IceStorm
{

typedef ::IceInternal::Handle< ::IceStorm::Topic> TopicPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::IceStorm::Topic> TopicPrx;

void __read(::IceInternal::BasicStream*, TopicPrx&);
void __patch__TopicPtr(void*, ::Ice::ObjectPtr&);

typedef ::IceInternal::Handle< ::IceStorm::TopicManager> TopicManagerPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::IceStorm::TopicManager> TopicManagerPrx;

void __read(::IceInternal::BasicStream*, TopicManagerPrx&);
void __patch__TopicManagerPtr(void*, ::Ice::ObjectPtr&);

}

namespace Ice
{

typedef ::std::map< ::std::string, ::std::string> SliceChecksumDict;
void __writeSliceChecksumDict(::IceInternal::BasicStream*, const SliceChecksumDict&);
void __readSliceChecksumDict(::IceInternal::BasicStream*, SliceChecksumDict&);

}

namespace IceStorm
{

struct LinkInfo
{
    ::IceStorm::TopicPrx theTopic;
    ::std::string name;
    ::Ice::Int cost;

    bool operator==(const LinkInfo&) const;
    bool operator<(const LinkInfo&) const;
    bool operator!=(const LinkInfo& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const LinkInfo& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const LinkInfo& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const LinkInfo& __rhs) const
    {
        return !operator<(__rhs);
    }

    void __write(::IceInternal::BasicStream*) const;
    void __read(::IceInternal::BasicStream*);
};

typedef ::std::vector< ::IceStorm::LinkInfo> LinkInfoSeq;
void __writeLinkInfoSeq(::IceInternal::BasicStream*, const ::IceStorm::LinkInfo*, const ::IceStorm::LinkInfo*);
void __readLinkInfoSeq(::IceInternal::BasicStream*, LinkInfoSeq&);

typedef ::std::map< ::std::string, ::std::string> QoS;
void __writeQoS(::IceInternal::BasicStream*, const QoS&);
void __readQoS(::IceInternal::BasicStream*, QoS&);

class LinkExists : public ::Ice::UserException
{
public:

    LinkExists() {}
    explicit LinkExists(const ::std::string&);
    virtual ~LinkExists() throw();

    virtual ::std::string ice_name() const;
    virtual ::Ice::Exception* ice_clone() const;
    virtual void ice_throw() const;

    static const ::IceInternal::UserExceptionFactoryPtr& ice_factory();

    ::std::string name;

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
};

static LinkExists __LinkExists_init;

class NoSuchLink : public ::Ice::UserException
{
public:

    NoSuchLink() {}
    explicit NoSuchLink(const ::std::string&);
    virtual ~NoSuchLink() throw();

    virtual ::std::string ice_name() const;
    virtual ::Ice::Exception* ice_clone() const;
    virtual void ice_throw() const;

    static const ::IceInternal::UserExceptionFactoryPtr& ice_factory();

    ::std::string name;

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
};

class AlreadySubscribed : public ::Ice::UserException
{
public:

    AlreadySubscribed() {}
    virtual ~AlreadySubscribed() throw();

    virtual ::std::string ice_name() const;
    virtual ::Ice::Exception* ice_clone() const;
    virtual void ice_throw() const;

    static const ::IceInternal::UserExceptionFactoryPtr& ice_factory();

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
};

class BadQoS : public ::Ice::UserException
{
public:

    BadQoS() {}
    explicit BadQoS(const ::std::string&);
    virtual ~BadQoS() throw();

    virtual ::std::string ice_name() const;
    virtual ::Ice::Exception* ice_clone() const;
    virtual void ice_throw() const;

    static const ::IceInternal::UserExceptionFactoryPtr& ice_factory();

    ::std::string reason;

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
};

typedef ::std::map< ::std::string, ::IceStorm::TopicPrx> TopicDict;
void __writeTopicDict(::IceInternal::BasicStream*, const TopicDict&);
void __readTopicDict(::IceInternal::BasicStream*, TopicDict&);

class TopicExists : public ::Ice::UserException
{
public:

    TopicExists() {}
    explicit TopicExists(const ::std::string&);
    virtual ~TopicExists() throw();

    virtual ::std::string ice_name() const;
    virtual ::Ice::Exception* ice_clone() const;
    virtual void ice_throw() const;

    static const ::IceInternal::UserExceptionFactoryPtr& ice_factory();

    ::std::string name;

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
};

class NoSuchTopic : public ::Ice::UserException
{
public:

    NoSuchTopic() {}
    explicit NoSuchTopic(const ::std::string&);
    virtual ~NoSuchTopic() throw();

    virtual ::std::string ice_name() const;
    virtual ::Ice::Exception* ice_clone() const;
    virtual void ice_throw() const;

    static const ::IceInternal::UserExceptionFactoryPtr& ice_factory();

    ::std::string name;

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
};

}

namespace IceStorm
{

class Topic : virtual public ::Ice::Object
{
public:

    typedef TopicPrx ProxyType;
    typedef TopicPtr PointerType;


    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual ::std::string getName(const ::Ice::Current& = ::Ice::Current()) const = 0;
#ifndef ICEE_PURE_CLIENT
    ::Ice::DispatchStatus ___getName(::IceInternal::Incoming&, const ::Ice::Current&) const;
#endif // ICEE_PURE_CLIENT

    virtual ::Ice::ObjectPrx getPublisher(const ::Ice::Current& = ::Ice::Current()) const = 0;
#ifndef ICEE_PURE_CLIENT
    ::Ice::DispatchStatus ___getPublisher(::IceInternal::Incoming&, const ::Ice::Current&) const;
#endif // ICEE_PURE_CLIENT

    virtual ::Ice::ObjectPrx getNonReplicatedPublisher(const ::Ice::Current& = ::Ice::Current()) const = 0;
#ifndef ICEE_PURE_CLIENT
    ::Ice::DispatchStatus ___getNonReplicatedPublisher(::IceInternal::Incoming&, const ::Ice::Current&) const;
#endif // ICEE_PURE_CLIENT

    ICE_DEPRECATED_API virtual void subscribe(const ::IceStorm::QoS&, const ::Ice::ObjectPrx&, const ::Ice::Current& = ::Ice::Current()) = 0;
#ifndef ICEE_PURE_CLIENT
    ::Ice::DispatchStatus ___subscribe(::IceInternal::Incoming&, const ::Ice::Current&);
#endif // ICEE_PURE_CLIENT

    virtual ::Ice::ObjectPrx subscribeAndGetPublisher(const ::IceStorm::QoS&, const ::Ice::ObjectPrx&, const ::Ice::Current& = ::Ice::Current()) = 0;
#ifndef ICEE_PURE_CLIENT
    ::Ice::DispatchStatus ___subscribeAndGetPublisher(::IceInternal::Incoming&, const ::Ice::Current&);
#endif // ICEE_PURE_CLIENT

    virtual void unsubscribe(const ::Ice::ObjectPrx&, const ::Ice::Current& = ::Ice::Current()) = 0;
#ifndef ICEE_PURE_CLIENT
    ::Ice::DispatchStatus ___unsubscribe(::IceInternal::Incoming&, const ::Ice::Current&);
#endif // ICEE_PURE_CLIENT

    virtual void link(const ::IceStorm::TopicPrx&, ::Ice::Int, const ::Ice::Current& = ::Ice::Current()) = 0;
#ifndef ICEE_PURE_CLIENT
    ::Ice::DispatchStatus ___link(::IceInternal::Incoming&, const ::Ice::Current&);
#endif // ICEE_PURE_CLIENT

    virtual void unlink(const ::IceStorm::TopicPrx&, const ::Ice::Current& = ::Ice::Current()) = 0;
#ifndef ICEE_PURE_CLIENT
    ::Ice::DispatchStatus ___unlink(::IceInternal::Incoming&, const ::Ice::Current&);
#endif // ICEE_PURE_CLIENT

    virtual ::IceStorm::LinkInfoSeq getLinkInfoSeq(const ::Ice::Current& = ::Ice::Current()) const = 0;
#ifndef ICEE_PURE_CLIENT
    ::Ice::DispatchStatus ___getLinkInfoSeq(::IceInternal::Incoming&, const ::Ice::Current&) const;
#endif // ICEE_PURE_CLIENT

    virtual void destroy(const ::Ice::Current& = ::Ice::Current()) = 0;
#ifndef ICEE_PURE_CLIENT
    ::Ice::DispatchStatus ___destroy(::IceInternal::Incoming&, const ::Ice::Current&);
#endif // ICEE_PURE_CLIENT

#ifndef ICEE_PURE_CLIENT
    virtual ::Ice::DispatchStatus __dispatch(::IceInternal::Incoming&, const ::Ice::Current&);
#endif // ICEE_PURE_CLIENT

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
};

class TopicManager : virtual public ::Ice::Object
{
public:

    typedef TopicManagerPrx ProxyType;
    typedef TopicManagerPtr PointerType;


    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual ::IceStorm::TopicPrx create(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) = 0;
#ifndef ICEE_PURE_CLIENT
    ::Ice::DispatchStatus ___create(::IceInternal::Incoming&, const ::Ice::Current&);
#endif // ICEE_PURE_CLIENT

    virtual ::IceStorm::TopicPrx retrieve(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const = 0;
#ifndef ICEE_PURE_CLIENT
    ::Ice::DispatchStatus ___retrieve(::IceInternal::Incoming&, const ::Ice::Current&) const;
#endif // ICEE_PURE_CLIENT

    virtual ::IceStorm::TopicDict retrieveAll(const ::Ice::Current& = ::Ice::Current()) const = 0;
#ifndef ICEE_PURE_CLIENT
    ::Ice::DispatchStatus ___retrieveAll(::IceInternal::Incoming&, const ::Ice::Current&) const;
#endif // ICEE_PURE_CLIENT

    virtual ::Ice::SliceChecksumDict getSliceChecksums(const ::Ice::Current& = ::Ice::Current()) const = 0;
#ifndef ICEE_PURE_CLIENT
    ::Ice::DispatchStatus ___getSliceChecksums(::IceInternal::Incoming&, const ::Ice::Current&) const;
#endif // ICEE_PURE_CLIENT

#ifndef ICEE_PURE_CLIENT
    virtual ::Ice::DispatchStatus __dispatch(::IceInternal::Incoming&, const ::Ice::Current&);
#endif // ICEE_PURE_CLIENT

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
};

}

namespace IceProxy
{

namespace IceStorm
{

class Topic : virtual public ::IceProxy::Ice::Object
{
public:

    ::std::string getName()
    {
        return getName(0);
    }
    ::std::string getName(const ::Ice::Context& __ctx)
    {
        return getName(&__ctx);
    }

private:

    ::std::string getName(const ::Ice::Context*);

public:

    ::Ice::ObjectPrx getPublisher()
    {
        return getPublisher(0);
    }
    ::Ice::ObjectPrx getPublisher(const ::Ice::Context& __ctx)
    {
        return getPublisher(&__ctx);
    }

private:

    ::Ice::ObjectPrx getPublisher(const ::Ice::Context*);

public:

    ::Ice::ObjectPrx getNonReplicatedPublisher()
    {
        return getNonReplicatedPublisher(0);
    }
    ::Ice::ObjectPrx getNonReplicatedPublisher(const ::Ice::Context& __ctx)
    {
        return getNonReplicatedPublisher(&__ctx);
    }

private:

    ::Ice::ObjectPrx getNonReplicatedPublisher(const ::Ice::Context*);

public:

    ICE_DEPRECATED_API void subscribe(const ::IceStorm::QoS& theQoS, const ::Ice::ObjectPrx& subscriber)
    {
        subscribe(theQoS, subscriber, 0);
    }
    ICE_DEPRECATED_API void subscribe(const ::IceStorm::QoS& theQoS, const ::Ice::ObjectPrx& subscriber, const ::Ice::Context& __ctx)
    {
        subscribe(theQoS, subscriber, &__ctx);
    }

private:

    void subscribe(const ::IceStorm::QoS&, const ::Ice::ObjectPrx&, const ::Ice::Context*);

public:

    ::Ice::ObjectPrx subscribeAndGetPublisher(const ::IceStorm::QoS& theQoS, const ::Ice::ObjectPrx& subscriber)
    {
        return subscribeAndGetPublisher(theQoS, subscriber, 0);
    }
    ::Ice::ObjectPrx subscribeAndGetPublisher(const ::IceStorm::QoS& theQoS, const ::Ice::ObjectPrx& subscriber, const ::Ice::Context& __ctx)
    {
        return subscribeAndGetPublisher(theQoS, subscriber, &__ctx);
    }

private:

    ::Ice::ObjectPrx subscribeAndGetPublisher(const ::IceStorm::QoS&, const ::Ice::ObjectPrx&, const ::Ice::Context*);

public:

    void unsubscribe(const ::Ice::ObjectPrx& subscriber)
    {
        unsubscribe(subscriber, 0);
    }
    void unsubscribe(const ::Ice::ObjectPrx& subscriber, const ::Ice::Context& __ctx)
    {
        unsubscribe(subscriber, &__ctx);
    }

private:

    void unsubscribe(const ::Ice::ObjectPrx&, const ::Ice::Context*);

public:

    void link(const ::IceStorm::TopicPrx& linkTo, ::Ice::Int cost)
    {
        link(linkTo, cost, 0);
    }
    void link(const ::IceStorm::TopicPrx& linkTo, ::Ice::Int cost, const ::Ice::Context& __ctx)
    {
        link(linkTo, cost, &__ctx);
    }

private:

    void link(const ::IceStorm::TopicPrx&, ::Ice::Int, const ::Ice::Context*);

public:

    void unlink(const ::IceStorm::TopicPrx& linkTo)
    {
        unlink(linkTo, 0);
    }
    void unlink(const ::IceStorm::TopicPrx& linkTo, const ::Ice::Context& __ctx)
    {
        unlink(linkTo, &__ctx);
    }

private:

    void unlink(const ::IceStorm::TopicPrx&, const ::Ice::Context*);

public:

    ::IceStorm::LinkInfoSeq getLinkInfoSeq()
    {
        return getLinkInfoSeq(0);
    }
    ::IceStorm::LinkInfoSeq getLinkInfoSeq(const ::Ice::Context& __ctx)
    {
        return getLinkInfoSeq(&__ctx);
    }

private:

    ::IceStorm::LinkInfoSeq getLinkInfoSeq(const ::Ice::Context*);

public:

    void destroy()
    {
        destroy(0);
    }
    void destroy(const ::Ice::Context& __ctx)
    {
        destroy(&__ctx);
    }

private:

    void destroy(const ::Ice::Context*);

public:

    ::IceInternal::ProxyHandle<Topic> ice_context(const ::Ice::Context& __context) const
    {
        return dynamic_cast<Topic*>(::IceProxy::Ice::Object::ice_context(__context).get());
    }

    ::IceInternal::ProxyHandle<Topic> ice_secure(bool __secure) const
    {
        return dynamic_cast<Topic*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    }

#ifdef ICEE_HAS_ROUTER
    ::IceInternal::ProxyHandle<Topic> ice_router(const ::Ice::RouterPrx& __router) const
    {
        return dynamic_cast<Topic*>(::IceProxy::Ice::Object::ice_router(__router).get());
    }
#endif // ICEE_HAS_ROUTER

#ifdef ICEE_HAS_LOCATOR
    ::IceInternal::ProxyHandle<Topic> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
        return dynamic_cast<Topic*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    }

    ::IceInternal::ProxyHandle<Topic> ice_adapterId(const std::string& __id) const
    {
        return dynamic_cast<Topic*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    }
#endif // ICEE_HAS_LOCATOR

    ::IceInternal::ProxyHandle<Topic> ice_twoway() const
    {
        return dynamic_cast<Topic*>(::IceProxy::Ice::Object::ice_twoway().get());
    }

    ::IceInternal::ProxyHandle<Topic> ice_oneway() const
    {
        return dynamic_cast<Topic*>(::IceProxy::Ice::Object::ice_oneway().get());
    }

    ::IceInternal::ProxyHandle<Topic> ice_batchOneway() const
    {
        return dynamic_cast<Topic*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    }

    ::IceInternal::ProxyHandle<Topic> ice_datagram() const
    {
        return dynamic_cast<Topic*>(::IceProxy::Ice::Object::ice_datagram().get());
    }

    ::IceInternal::ProxyHandle<Topic> ice_batchDatagram() const
    {
        return dynamic_cast<Topic*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    }

    ::IceInternal::ProxyHandle<Topic> ice_timeout(int __timeout) const
    {
        return dynamic_cast<Topic*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    }

    static const ::std::string& ice_staticId();

private:

    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

class TopicManager : virtual public ::IceProxy::Ice::Object
{
public:

    ::IceStorm::TopicPrx create(const ::std::string& name)
    {
        return create(name, 0);
    }
    ::IceStorm::TopicPrx create(const ::std::string& name, const ::Ice::Context& __ctx)
    {
        return create(name, &__ctx);
    }

private:

    ::IceStorm::TopicPrx create(const ::std::string&, const ::Ice::Context*);

public:

    ::IceStorm::TopicPrx retrieve(const ::std::string& name)
    {
        return retrieve(name, 0);
    }
    ::IceStorm::TopicPrx retrieve(const ::std::string& name, const ::Ice::Context& __ctx)
    {
        return retrieve(name, &__ctx);
    }

private:

    ::IceStorm::TopicPrx retrieve(const ::std::string&, const ::Ice::Context*);

public:

    ::IceStorm::TopicDict retrieveAll()
    {
        return retrieveAll(0);
    }
    ::IceStorm::TopicDict retrieveAll(const ::Ice::Context& __ctx)
    {
        return retrieveAll(&__ctx);
    }

private:

    ::IceStorm::TopicDict retrieveAll(const ::Ice::Context*);

public:

    ::Ice::SliceChecksumDict getSliceChecksums()
    {
        return getSliceChecksums(0);
    }
    ::Ice::SliceChecksumDict getSliceChecksums(const ::Ice::Context& __ctx)
    {
        return getSliceChecksums(&__ctx);
    }

private:

    ::Ice::SliceChecksumDict getSliceChecksums(const ::Ice::Context*);

public:

    ::IceInternal::ProxyHandle<TopicManager> ice_context(const ::Ice::Context& __context) const
    {
        return dynamic_cast<TopicManager*>(::IceProxy::Ice::Object::ice_context(__context).get());
    }

    ::IceInternal::ProxyHandle<TopicManager> ice_secure(bool __secure) const
    {
        return dynamic_cast<TopicManager*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    }

#ifdef ICEE_HAS_ROUTER
    ::IceInternal::ProxyHandle<TopicManager> ice_router(const ::Ice::RouterPrx& __router) const
    {
        return dynamic_cast<TopicManager*>(::IceProxy::Ice::Object::ice_router(__router).get());
    }
#endif // ICEE_HAS_ROUTER

#ifdef ICEE_HAS_LOCATOR
    ::IceInternal::ProxyHandle<TopicManager> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
        return dynamic_cast<TopicManager*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    }

    ::IceInternal::ProxyHandle<TopicManager> ice_adapterId(const std::string& __id) const
    {
        return dynamic_cast<TopicManager*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    }
#endif // ICEE_HAS_LOCATOR

    ::IceInternal::ProxyHandle<TopicManager> ice_twoway() const
    {
        return dynamic_cast<TopicManager*>(::IceProxy::Ice::Object::ice_twoway().get());
    }

    ::IceInternal::ProxyHandle<TopicManager> ice_oneway() const
    {
        return dynamic_cast<TopicManager*>(::IceProxy::Ice::Object::ice_oneway().get());
    }

    ::IceInternal::ProxyHandle<TopicManager> ice_batchOneway() const
    {
        return dynamic_cast<TopicManager*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    }

    ::IceInternal::ProxyHandle<TopicManager> ice_datagram() const
    {
        return dynamic_cast<TopicManager*>(::IceProxy::Ice::Object::ice_datagram().get());
    }

    ::IceInternal::ProxyHandle<TopicManager> ice_batchDatagram() const
    {
        return dynamic_cast<TopicManager*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    }

    ::IceInternal::ProxyHandle<TopicManager> ice_timeout(int __timeout) const
    {
        return dynamic_cast<TopicManager*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    }

    static const ::std::string& ice_staticId();

private:

    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

}

}

#endif
