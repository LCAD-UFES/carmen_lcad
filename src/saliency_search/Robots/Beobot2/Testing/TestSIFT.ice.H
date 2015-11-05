// **********************************************************************
//
// Copyright (c) 2003-2008 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.0
// Generated from file `TestSIFT.ice'

#ifndef __src_Robots_Beobot2_Testing__TestSIFT_ice_H__
#define __src_Robots_Beobot2_Testing__TestSIFT_ice_H__

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
#include <Ice/StreamF.h>
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

namespace TestSIFT
{

class SIFTMatcher;

}

}

namespace TestSIFT
{

class SIFTMatcher;
bool operator==(const SIFTMatcher&, const SIFTMatcher&);
bool operator<(const SIFTMatcher&, const SIFTMatcher&);

}

namespace IceInternal
{

::Ice::Object* upCast(::TestSIFT::SIFTMatcher*);
::IceProxy::Ice::Object* upCast(::IceProxy::TestSIFT::SIFTMatcher*);

}

namespace TestSIFT
{

typedef ::IceInternal::Handle< ::TestSIFT::SIFTMatcher> SIFTMatcherPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::TestSIFT::SIFTMatcher> SIFTMatcherPrx;

void __read(::IceInternal::BasicStream*, SIFTMatcherPrx&);
void __patch__SIFTMatcherPtr(void*, ::Ice::ObjectPtr&);

}

namespace TestSIFT
{

const ::Ice::Int DEFAULTWORKERPORT = 10000;

typedef ::std::vector< ::Ice::Byte> byteSequence;

struct keypoint
{
    ::Ice::Float x;
    ::Ice::Float y;
    ::Ice::Float s;
    ::Ice::Float o;
    ::Ice::Float m;
    ::TestSIFT::byteSequence oriFV;

    bool operator==(const keypoint&) const;
    bool operator<(const keypoint&) const;
    bool operator!=(const keypoint& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const keypoint& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const keypoint& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const keypoint& __rhs) const
    {
        return !operator<(__rhs);
    }

    void __write(::IceInternal::BasicStream*) const;
    void __read(::IceInternal::BasicStream*);
};

typedef ::std::vector< ::TestSIFT::keypoint> keypointSequence;
void __writekeypointSequence(::IceInternal::BasicStream*, const ::TestSIFT::keypoint*, const ::TestSIFT::keypoint*);
void __readkeypointSequence(::IceInternal::BasicStream*, keypointSequence&);

typedef ::std::vector< ::Ice::Double> idSequence;

}

namespace IceProxy
{

namespace TestSIFT
{

class SIFTMatcher : virtual public ::IceProxy::Ice::Object
{
public:

    ::TestSIFT::idSequence matchKeypoints(const ::TestSIFT::keypointSequence& keypoints)
    {
        return matchKeypoints(keypoints, 0);
    }
    ::TestSIFT::idSequence matchKeypoints(const ::TestSIFT::keypointSequence& keypoints, const ::Ice::Context& __ctx)
    {
        return matchKeypoints(keypoints, &__ctx);
    }

private:

    ::TestSIFT::idSequence matchKeypoints(const ::TestSIFT::keypointSequence&, const ::Ice::Context*);

public:

    ::IceInternal::ProxyHandle<SIFTMatcher> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SIFTMatcher*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<SIFTMatcher*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }

    ::IceInternal::ProxyHandle<SIFTMatcher> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SIFTMatcher*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<SIFTMatcher*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }

    ::IceInternal::ProxyHandle<SIFTMatcher> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SIFTMatcher*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<SIFTMatcher*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }

    ::IceInternal::ProxyHandle<SIFTMatcher> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SIFTMatcher*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<SIFTMatcher*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }

    ::IceInternal::ProxyHandle<SIFTMatcher> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SIFTMatcher*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<SIFTMatcher*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }

    ::IceInternal::ProxyHandle<SIFTMatcher> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SIFTMatcher*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<SIFTMatcher*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }

    ::IceInternal::ProxyHandle<SIFTMatcher> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SIFTMatcher*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<SIFTMatcher*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }

    ::IceInternal::ProxyHandle<SIFTMatcher> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SIFTMatcher*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<SIFTMatcher*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }

    ::IceInternal::ProxyHandle<SIFTMatcher> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SIFTMatcher*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<SIFTMatcher*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }

    ::IceInternal::ProxyHandle<SIFTMatcher> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SIFTMatcher*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<SIFTMatcher*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }

    ::IceInternal::ProxyHandle<SIFTMatcher> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SIFTMatcher*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<SIFTMatcher*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }

    ::IceInternal::ProxyHandle<SIFTMatcher> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SIFTMatcher*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<SIFTMatcher*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<SIFTMatcher> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SIFTMatcher*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<SIFTMatcher*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<SIFTMatcher> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SIFTMatcher*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<SIFTMatcher*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<SIFTMatcher> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SIFTMatcher*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<SIFTMatcher*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }

    ::IceInternal::ProxyHandle<SIFTMatcher> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SIFTMatcher*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<SIFTMatcher*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }

    ::IceInternal::ProxyHandle<SIFTMatcher> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SIFTMatcher*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<SIFTMatcher*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }

    ::IceInternal::ProxyHandle<SIFTMatcher> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SIFTMatcher*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<SIFTMatcher*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }

    ::IceInternal::ProxyHandle<SIFTMatcher> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<SIFTMatcher*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<SIFTMatcher*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
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

namespace TestSIFT
{

class SIFTMatcher : virtual public ::IceDelegate::Ice::Object
{
public:

    virtual ::TestSIFT::idSequence matchKeypoints(const ::TestSIFT::keypointSequence&, const ::Ice::Context*) = 0;
};

}

}

namespace IceDelegateM
{

namespace TestSIFT
{

class SIFTMatcher : virtual public ::IceDelegate::TestSIFT::SIFTMatcher,
                    virtual public ::IceDelegateM::Ice::Object
{
public:

    virtual ::TestSIFT::idSequence matchKeypoints(const ::TestSIFT::keypointSequence&, const ::Ice::Context*);
};

}

}

namespace IceDelegateD
{

namespace TestSIFT
{

class SIFTMatcher : virtual public ::IceDelegate::TestSIFT::SIFTMatcher,
                    virtual public ::IceDelegateD::Ice::Object
{
public:

    virtual ::TestSIFT::idSequence matchKeypoints(const ::TestSIFT::keypointSequence&, const ::Ice::Context*);
};

}

}

namespace TestSIFT
{

class SIFTMatcher : virtual public ::Ice::Object
{
public:

    typedef SIFTMatcherPrx ProxyType;
    typedef SIFTMatcherPtr PointerType;

    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual ::TestSIFT::idSequence matchKeypoints(const ::TestSIFT::keypointSequence&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___matchKeypoints(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::Ice::DispatchStatus __dispatch(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);
};

}

#endif
