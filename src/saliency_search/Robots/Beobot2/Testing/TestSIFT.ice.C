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

#include <TestSIFT.ice.H>
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

static const ::std::string __TestSIFT__SIFTMatcher__matchKeypoints_name = "matchKeypoints";

::Ice::Object* IceInternal::upCast(::TestSIFT::SIFTMatcher* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::TestSIFT::SIFTMatcher* p) { return p; }

void
TestSIFT::__read(::IceInternal::BasicStream* __is, ::TestSIFT::SIFTMatcherPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::TestSIFT::SIFTMatcher;
        v->__copyFrom(proxy);
    }
}

bool
TestSIFT::keypoint::operator==(const keypoint& __rhs) const
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
    if(s != __rhs.s)
    {
        return false;
    }
    if(o != __rhs.o)
    {
        return false;
    }
    if(m != __rhs.m)
    {
        return false;
    }
    if(oriFV != __rhs.oriFV)
    {
        return false;
    }
    return true;
}

bool
TestSIFT::keypoint::operator<(const keypoint& __rhs) const
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
    if(s < __rhs.s)
    {
        return true;
    }
    else if(__rhs.s < s)
    {
        return false;
    }
    if(o < __rhs.o)
    {
        return true;
    }
    else if(__rhs.o < o)
    {
        return false;
    }
    if(m < __rhs.m)
    {
        return true;
    }
    else if(__rhs.m < m)
    {
        return false;
    }
    if(oriFV < __rhs.oriFV)
    {
        return true;
    }
    else if(__rhs.oriFV < oriFV)
    {
        return false;
    }
    return false;
}

void
TestSIFT::keypoint::__write(::IceInternal::BasicStream* __os) const
{
    __os->write(x);
    __os->write(y);
    __os->write(s);
    __os->write(o);
    __os->write(m);
    if(oriFV.size() == 0)
    {
        __os->writeSize(0);
    }
    else
    {
        __os->write(&oriFV[0], &oriFV[0] + oriFV.size());
    }
}

void
TestSIFT::keypoint::__read(::IceInternal::BasicStream* __is)
{
    __is->read(x);
    __is->read(y);
    __is->read(s);
    __is->read(o);
    __is->read(m);
    ::std::pair<const ::Ice::Byte*, const ::Ice::Byte*> ___oriFV;
    __is->read(___oriFV);
    ::std::vector< ::Ice::Byte>(___oriFV.first, ___oriFV.second).swap(oriFV);
}

void
TestSIFT::__writekeypointSequence(::IceInternal::BasicStream* __os, const ::TestSIFT::keypoint* begin, const ::TestSIFT::keypoint* end)
{
    ::Ice::Int size = static_cast< ::Ice::Int>(end - begin);
    __os->writeSize(size);
    for(int i = 0; i < size; ++i)
    {
        begin[i].__write(__os);
    }
}

void
TestSIFT::__readkeypointSequence(::IceInternal::BasicStream* __is, ::TestSIFT::keypointSequence& v)
{
    ::Ice::Int sz;
    __is->readSize(sz);
    __is->startSeq(sz, 21);
    v.resize(sz);
    for(int i = 0; i < sz; ++i)
    {
        v[i].__read(__is);
        __is->checkSeq();
        __is->endElement();
    }
    __is->endSeq(sz);
}

::TestSIFT::idSequence
IceProxy::TestSIFT::SIFTMatcher::matchKeypoints(const ::TestSIFT::keypointSequence& keypoints, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __checkTwowayOnly(__TestSIFT__SIFTMatcher__matchKeypoints_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::TestSIFT::SIFTMatcher* __del = dynamic_cast< ::IceDelegate::TestSIFT::SIFTMatcher*>(__delBase.get());
            return __del->matchKeypoints(keypoints, __ctx);
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
IceProxy::TestSIFT::SIFTMatcher::ice_staticId()
{
    return ::TestSIFT::SIFTMatcher::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::TestSIFT::SIFTMatcher::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::TestSIFT::SIFTMatcher);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::TestSIFT::SIFTMatcher::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::TestSIFT::SIFTMatcher);
}

::IceProxy::Ice::Object*
IceProxy::TestSIFT::SIFTMatcher::__newInstance() const
{
    return new SIFTMatcher;
}

::TestSIFT::idSequence
IceDelegateM::TestSIFT::SIFTMatcher::matchKeypoints(const ::TestSIFT::keypointSequence& keypoints, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __TestSIFT__SIFTMatcher__matchKeypoints_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        if(keypoints.size() == 0)
        {
            __os->writeSize(0);
        }
        else
        {
            ::TestSIFT::__writekeypointSequence(__os, &keypoints[0], &keypoints[0] + keypoints.size());
        }
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
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
        ::TestSIFT::idSequence __ret;
        ::IceInternal::BasicStream* __is = __og.is();
        __is->startReadEncaps();
        __is->read(__ret);
        __is->endReadEncaps();
        return __ret;
    }
    catch(const ::Ice::LocalException& __ex)
    {
        throw ::IceInternal::LocalExceptionWrapper(__ex, false);
    }
}

::TestSIFT::idSequence
IceDelegateD::TestSIFT::SIFTMatcher::matchKeypoints(const ::TestSIFT::keypointSequence& keypoints, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(::TestSIFT::idSequence& __result, const ::TestSIFT::keypointSequence& keypoints, const ::Ice::Current& __current) :
            ::IceInternal::Direct(__current),
            _result(__result),
            _m_keypoints(keypoints)
        {
        }

        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::TestSIFT::SIFTMatcher* servant = dynamic_cast< ::TestSIFT::SIFTMatcher*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            _result = servant->matchKeypoints(_m_keypoints, _current);
            return ::Ice::DispatchOK;
        }

    private:

        ::TestSIFT::idSequence& _result;
        const ::TestSIFT::keypointSequence& _m_keypoints;
    };

    ::Ice::Current __current;
    __initCurrent(__current, __TestSIFT__SIFTMatcher__matchKeypoints_name, ::Ice::Normal, __context);
    ::TestSIFT::idSequence __result;
    try
    {
        _DirectI __direct(__result, keypoints, __current);
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
    return __result;
}

::Ice::ObjectPtr
TestSIFT::SIFTMatcher::ice_clone() const
{
    throw ::Ice::CloneNotImplementedException(__FILE__, __LINE__);
    return 0; // to avoid a warning with some compilers
}

static const ::std::string __TestSIFT__SIFTMatcher_ids[2] =
{
    "::Ice::Object",
    "::TestSIFT::SIFTMatcher"
};

bool
TestSIFT::SIFTMatcher::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__TestSIFT__SIFTMatcher_ids, __TestSIFT__SIFTMatcher_ids + 2, _s);
}

::std::vector< ::std::string>
TestSIFT::SIFTMatcher::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__TestSIFT__SIFTMatcher_ids[0], &__TestSIFT__SIFTMatcher_ids[2]);
}

const ::std::string&
TestSIFT::SIFTMatcher::ice_id(const ::Ice::Current&) const
{
    return __TestSIFT__SIFTMatcher_ids[1];
}

const ::std::string&
TestSIFT::SIFTMatcher::ice_staticId()
{
    return __TestSIFT__SIFTMatcher_ids[1];
}

::Ice::DispatchStatus
TestSIFT::SIFTMatcher::___matchKeypoints(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::TestSIFT::keypointSequence keypoints;
    ::TestSIFT::__readkeypointSequence(__is, keypoints);
    __is->endReadEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    ::TestSIFT::idSequence __ret = matchKeypoints(keypoints, __current);
    if(__ret.size() == 0)
    {
        __os->writeSize(0);
    }
    else
    {
        __os->write(&__ret[0], &__ret[0] + __ret.size());
    }
    return ::Ice::DispatchOK;
}

static ::std::string __TestSIFT__SIFTMatcher_all[] =
{
    "ice_id",
    "ice_ids",
    "ice_isA",
    "ice_ping",
    "matchKeypoints"
};

::Ice::DispatchStatus
TestSIFT::SIFTMatcher::__dispatch(::IceInternal::Incoming& in, const ::Ice::Current& current)
{
    ::std::pair< ::std::string*, ::std::string*> r = ::std::equal_range(__TestSIFT__SIFTMatcher_all, __TestSIFT__SIFTMatcher_all + 5, current.operation);
    if(r.first == r.second)
    {
        throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
    }

    switch(r.first - __TestSIFT__SIFTMatcher_all)
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
            return ___matchKeypoints(in, current);
        }
    }

    assert(false);
    throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
}

void
TestSIFT::SIFTMatcher::__write(::IceInternal::BasicStream* __os) const
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
TestSIFT::SIFTMatcher::__read(::IceInternal::BasicStream* __is, bool __rid)
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
TestSIFT::SIFTMatcher::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type TestSIFT::SIFTMatcher was not generated with stream support";
    throw ex;
}

void
TestSIFT::SIFTMatcher::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type TestSIFT::SIFTMatcher was not generated with stream support";
    throw ex;
}

void
TestSIFT::__patch__SIFTMatcherPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::TestSIFT::SIFTMatcherPtr* p = static_cast< ::TestSIFT::SIFTMatcherPtr*>(__addr);
    assert(p);
    *p = ::TestSIFT::SIFTMatcherPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::TestSIFT::SIFTMatcher::ice_staticId(), v->ice_id());
    }
}

bool
TestSIFT::operator==(const ::TestSIFT::SIFTMatcher& l, const ::TestSIFT::SIFTMatcher& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
TestSIFT::operator<(const ::TestSIFT::SIFTMatcher& l, const ::TestSIFT::SIFTMatcher& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}
