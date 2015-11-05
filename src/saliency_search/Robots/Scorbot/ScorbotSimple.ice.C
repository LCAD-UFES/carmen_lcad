// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1
// Generated from file `ScorbotSimple.ice'

#include <ScorbotSimple.ice.H>
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
#   if ICE_INT_VERSION % 100 < 1
#       error Ice patch level mismatch!
#   endif
#endif

static const ::std::string __ScorbotSimpleIce__ScorbotSimple__getState_name = "getState";

static const ::std::string __ScorbotSimpleIce__ScorbotSimple__setNext_name = "setNext";

static const ::std::string __ScorbotSimpleIce__ScorbotSimple__reset_name = "reset";

::Ice::Object* IceInternal::upCast(::ScorbotSimpleIce::ScorbotSimple* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::ScorbotSimpleIce::ScorbotSimple* p) { return p; }

void
ScorbotSimpleIce::__read(::IceInternal::BasicStream* __is, ::ScorbotSimpleIce::ScorbotSimplePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::ScorbotSimpleIce::ScorbotSimple;
        v->__copyFrom(proxy);
    }
}

bool
IceProxy::ScorbotSimpleIce::ScorbotSimple::getState(const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
#if defined(__BCPLUSPLUS__) && (__BCPLUSPLUS__ >= 0x0600) // C++Builder 2009 compiler bug
            IceUtil::DummyBCC dummy;
#endif
            __checkTwowayOnly(__ScorbotSimpleIce__ScorbotSimple__getState_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::ScorbotSimpleIce::ScorbotSimple* __del = dynamic_cast< ::IceDelegate::ScorbotSimpleIce::ScorbotSimple*>(__delBase.get());
            return __del->getState(__ctx);
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

void
IceProxy::ScorbotSimpleIce::ScorbotSimple::setNext(const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
#if defined(__BCPLUSPLUS__) && (__BCPLUSPLUS__ >= 0x0600) // C++Builder 2009 compiler bug
            IceUtil::DummyBCC dummy;
#endif
            __delBase = __getDelegate(false);
            ::IceDelegate::ScorbotSimpleIce::ScorbotSimple* __del = dynamic_cast< ::IceDelegate::ScorbotSimpleIce::ScorbotSimple*>(__delBase.get());
            __del->setNext(__ctx);
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

void
IceProxy::ScorbotSimpleIce::ScorbotSimple::reset(const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
#if defined(__BCPLUSPLUS__) && (__BCPLUSPLUS__ >= 0x0600) // C++Builder 2009 compiler bug
            IceUtil::DummyBCC dummy;
#endif
            __delBase = __getDelegate(false);
            ::IceDelegate::ScorbotSimpleIce::ScorbotSimple* __del = dynamic_cast< ::IceDelegate::ScorbotSimpleIce::ScorbotSimple*>(__delBase.get());
            __del->reset(__ctx);
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
IceProxy::ScorbotSimpleIce::ScorbotSimple::ice_staticId()
{
    return ::ScorbotSimpleIce::ScorbotSimple::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::ScorbotSimpleIce::ScorbotSimple::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::ScorbotSimpleIce::ScorbotSimple);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::ScorbotSimpleIce::ScorbotSimple::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::ScorbotSimpleIce::ScorbotSimple);
}

::IceProxy::Ice::Object*
IceProxy::ScorbotSimpleIce::ScorbotSimple::__newInstance() const
{
    return new ScorbotSimple;
}

bool
IceDelegateM::ScorbotSimpleIce::ScorbotSimple::getState(const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __ScorbotSimpleIce__ScorbotSimple__getState_name, ::Ice::Normal, __context);
    bool __ok = __og.invoke();
    bool __ret;
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

void
IceDelegateM::ScorbotSimpleIce::ScorbotSimple::setNext(const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __ScorbotSimpleIce__ScorbotSimple__setNext_name, ::Ice::Normal, __context);
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
IceDelegateM::ScorbotSimpleIce::ScorbotSimple::reset(const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __ScorbotSimpleIce__ScorbotSimple__reset_name, ::Ice::Normal, __context);
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

bool
IceDelegateD::ScorbotSimpleIce::ScorbotSimple::getState(const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(bool& __result, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _result(__result)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::ScorbotSimpleIce::ScorbotSimple* servant = dynamic_cast< ::ScorbotSimpleIce::ScorbotSimple*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            _result = servant->getState(_current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        bool& _result;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __ScorbotSimpleIce__ScorbotSimple__getState_name, ::Ice::Normal, __context);
    bool __result;
    try
    {
        _DirectI __direct(__result, __current);
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

void
IceDelegateD::ScorbotSimpleIce::ScorbotSimple::setNext(const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::ScorbotSimpleIce::ScorbotSimple* servant = dynamic_cast< ::ScorbotSimpleIce::ScorbotSimple*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->setNext(_current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __ScorbotSimpleIce__ScorbotSimple__setNext_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(__current);
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

void
IceDelegateD::ScorbotSimpleIce::ScorbotSimple::reset(const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::ScorbotSimpleIce::ScorbotSimple* servant = dynamic_cast< ::ScorbotSimpleIce::ScorbotSimple*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->reset(_current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __ScorbotSimpleIce__ScorbotSimple__reset_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(__current);
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
ScorbotSimpleIce::ScorbotSimple::ice_clone() const
{
    throw ::Ice::CloneNotImplementedException(__FILE__, __LINE__);
    return 0; // to avoid a warning with some compilers
}

static const ::std::string __ScorbotSimpleIce__ScorbotSimple_ids[2] =
{
    "::Ice::Object",
    "::ScorbotSimpleIce::ScorbotSimple"
};

bool
ScorbotSimpleIce::ScorbotSimple::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__ScorbotSimpleIce__ScorbotSimple_ids, __ScorbotSimpleIce__ScorbotSimple_ids + 2, _s);
}

::std::vector< ::std::string>
ScorbotSimpleIce::ScorbotSimple::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__ScorbotSimpleIce__ScorbotSimple_ids[0], &__ScorbotSimpleIce__ScorbotSimple_ids[2]);
}

const ::std::string&
ScorbotSimpleIce::ScorbotSimple::ice_id(const ::Ice::Current&) const
{
    return __ScorbotSimpleIce__ScorbotSimple_ids[1];
}

const ::std::string&
ScorbotSimpleIce::ScorbotSimple::ice_staticId()
{
    return __ScorbotSimpleIce__ScorbotSimple_ids[1];
}

::Ice::DispatchStatus
ScorbotSimpleIce::ScorbotSimple::___getState(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    __inS.is()->skipEmptyEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    bool __ret = getState(__current);
    __os->write(__ret);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
ScorbotSimpleIce::ScorbotSimple::___setNext(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    __inS.is()->skipEmptyEncaps();
    setNext(__current);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
ScorbotSimpleIce::ScorbotSimple::___reset(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    __inS.is()->skipEmptyEncaps();
    reset(__current);
    return ::Ice::DispatchOK;
}

static ::std::string __ScorbotSimpleIce__ScorbotSimple_all[] =
{
    "getState",
    "ice_id",
    "ice_ids",
    "ice_isA",
    "ice_ping",
    "reset",
    "setNext"
};

::Ice::DispatchStatus
ScorbotSimpleIce::ScorbotSimple::__dispatch(::IceInternal::Incoming& in, const ::Ice::Current& current)
{
    ::std::pair< ::std::string*, ::std::string*> r = ::std::equal_range(__ScorbotSimpleIce__ScorbotSimple_all, __ScorbotSimpleIce__ScorbotSimple_all + 7, current.operation);
    if(r.first == r.second)
    {
        throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
    }

    switch(r.first - __ScorbotSimpleIce__ScorbotSimple_all)
    {
        case 0:
        {
            return ___getState(in, current);
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
        case 5:
        {
            return ___reset(in, current);
        }
        case 6:
        {
            return ___setNext(in, current);
        }
    }

    assert(false);
    throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
}

void
ScorbotSimpleIce::ScorbotSimple::__write(::IceInternal::BasicStream* __os) const
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
ScorbotSimpleIce::ScorbotSimple::__read(::IceInternal::BasicStream* __is, bool __rid)
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
ScorbotSimpleIce::ScorbotSimple::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type ScorbotSimpleIce::ScorbotSimple was not generated with stream support";
    throw ex;
}

void
ScorbotSimpleIce::ScorbotSimple::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type ScorbotSimpleIce::ScorbotSimple was not generated with stream support";
    throw ex;
}

void 
ScorbotSimpleIce::__patch__ScorbotSimplePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::ScorbotSimpleIce::ScorbotSimplePtr* p = static_cast< ::ScorbotSimpleIce::ScorbotSimplePtr*>(__addr);
    assert(p);
    *p = ::ScorbotSimpleIce::ScorbotSimplePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::ScorbotSimpleIce::ScorbotSimple::ice_staticId(), v->ice_id());
    }
}

bool
ScorbotSimpleIce::operator==(const ::ScorbotSimpleIce::ScorbotSimple& l, const ::ScorbotSimpleIce::ScorbotSimple& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
ScorbotSimpleIce::operator<(const ::ScorbotSimpleIce::ScorbotSimple& l, const ::ScorbotSimpleIce::ScorbotSimple& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}
