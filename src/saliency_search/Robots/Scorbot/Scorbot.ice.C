// **********************************************************************
//
// Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.3.1
// Generated from file `Scorbot.ice'

#include <Scorbot.ice.H>
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

static const ::std::string __ScorbotIce__Scorbot__setJoint_name = "setJoint";

static const ::std::string __ScorbotIce__Scorbot__setJoints_name = "setJoints";

static const ::std::string __ScorbotIce__Scorbot__getEncoder_name = "getEncoder";

static const ::std::string __ScorbotIce__Scorbot__getEncoders_name = "getEncoders";

static const ::std::string __ScorbotIce__Scorbot__setEnabled_name = "setEnabled";

static const ::std::string __ScorbotIce__Scorbot__resetEncoders_name = "resetEncoders";

static const ::std::string __ScorbotIce__Scorbot__getPWM_name = "getPWM";

static const ::std::string __ScorbotIce__Scorbot__getPWMs_name = "getPWMs";

static const ::std::string __ScorbotIce__Scorbot__setControlParams_name = "setControlParams";

static const ::std::string __ScorbotIce__Scorbot__getPIDVals_name = "getPIDVals";

static const ::std::string __ScorbotIce__Scorbot__getTuningVals_name = "getTuningVals";

static const ::std::string __ScorbotIce__Scorbot__setGravityParameters_name = "setGravityParameters";

static const ::std::string __ScorbotIce__Scorbot__getGravityParameters_name = "getGravityParameters";

::Ice::Object* IceInternal::upCast(::ScorbotIce::Scorbot* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::ScorbotIce::Scorbot* p) { return p; }

void
ScorbotIce::__read(::IceInternal::BasicStream* __is, ::ScorbotIce::ScorbotPrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::ScorbotIce::Scorbot;
        v->__copyFrom(proxy);
    }
}

void
ScorbotIce::__write(::IceInternal::BasicStream* __os, ::ScorbotIce::JointType v)
{
    __os->write(static_cast< ::Ice::Byte>(v), 7);
}

void
ScorbotIce::__read(::IceInternal::BasicStream* __is, ::ScorbotIce::JointType& v)
{
    ::Ice::Byte val;
    __is->read(val, 7);
    v = static_cast< ::ScorbotIce::JointType>(val);
}

void
ScorbotIce::__writeencoderValsType(::IceInternal::BasicStream* __os, const ::ScorbotIce::encoderValsType& v)
{
    __os->writeSize(::Ice::Int(v.size()));
    ::ScorbotIce::encoderValsType::const_iterator p;
    for(p = v.begin(); p != v.end(); ++p)
    {
        ::ScorbotIce::__write(__os, p->first);
        __os->write(p->second);
    }
}

void
ScorbotIce::__readencoderValsType(::IceInternal::BasicStream* __is, ::ScorbotIce::encoderValsType& v)
{
    ::Ice::Int sz;
    __is->readSize(sz);
    while(sz--)
    {
        ::std::pair<const  ::ScorbotIce::JointType, ::Ice::Int> pair;
        ::ScorbotIce::__read(__is, const_cast< ::ScorbotIce::JointType&>(pair.first));
        ::ScorbotIce::encoderValsType::iterator __i = v.insert(v.end(), pair);
        __is->read(__i->second);
    }
}

void
ScorbotIce::__writepwmValsType(::IceInternal::BasicStream* __os, const ::ScorbotIce::pwmValsType& v)
{
    __os->writeSize(::Ice::Int(v.size()));
    ::ScorbotIce::pwmValsType::const_iterator p;
    for(p = v.begin(); p != v.end(); ++p)
    {
        ::ScorbotIce::__write(__os, p->first);
        __os->write(p->second);
    }
}

void
ScorbotIce::__readpwmValsType(::IceInternal::BasicStream* __is, ::ScorbotIce::pwmValsType& v)
{
    ::Ice::Int sz;
    __is->readSize(sz);
    while(sz--)
    {
        ::std::pair<const  ::ScorbotIce::JointType, ::Ice::Float> pair;
        ::ScorbotIce::__read(__is, const_cast< ::ScorbotIce::JointType&>(pair.first));
        ::ScorbotIce::pwmValsType::iterator __i = v.insert(v.end(), pair);
        __is->read(__i->second);
    }
}

void
IceProxy::ScorbotIce::Scorbot::setJoint(::ScorbotIce::JointType joint, ::Ice::Int encoderPos, ::Ice::Int timeMS, const ::Ice::Context* __ctx)
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
            ::IceDelegate::ScorbotIce::Scorbot* __del = dynamic_cast< ::IceDelegate::ScorbotIce::Scorbot*>(__delBase.get());
            __del->setJoint(joint, encoderPos, timeMS, __ctx);
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
IceProxy::ScorbotIce::Scorbot::setJoints(const ::ScorbotIce::encoderValsType& pos, ::Ice::Int timeMS, const ::Ice::Context* __ctx)
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
            ::IceDelegate::ScorbotIce::Scorbot* __del = dynamic_cast< ::IceDelegate::ScorbotIce::Scorbot*>(__delBase.get());
            __del->setJoints(pos, timeMS, __ctx);
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

::Ice::Int
IceProxy::ScorbotIce::Scorbot::getEncoder(::ScorbotIce::JointType joint, const ::Ice::Context* __ctx)
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
            __checkTwowayOnly(__ScorbotIce__Scorbot__getEncoder_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::ScorbotIce::Scorbot* __del = dynamic_cast< ::IceDelegate::ScorbotIce::Scorbot*>(__delBase.get());
            return __del->getEncoder(joint, __ctx);
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

::ScorbotIce::encoderValsType
IceProxy::ScorbotIce::Scorbot::getEncoders(const ::Ice::Context* __ctx)
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
            __checkTwowayOnly(__ScorbotIce__Scorbot__getEncoders_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::ScorbotIce::Scorbot* __del = dynamic_cast< ::IceDelegate::ScorbotIce::Scorbot*>(__delBase.get());
            return __del->getEncoders(__ctx);
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
IceProxy::ScorbotIce::Scorbot::setEnabled(bool enabled, const ::Ice::Context* __ctx)
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
            ::IceDelegate::ScorbotIce::Scorbot* __del = dynamic_cast< ::IceDelegate::ScorbotIce::Scorbot*>(__delBase.get());
            __del->setEnabled(enabled, __ctx);
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
IceProxy::ScorbotIce::Scorbot::resetEncoders(const ::Ice::Context* __ctx)
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
            ::IceDelegate::ScorbotIce::Scorbot* __del = dynamic_cast< ::IceDelegate::ScorbotIce::Scorbot*>(__delBase.get());
            __del->resetEncoders(__ctx);
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

::Ice::Float
IceProxy::ScorbotIce::Scorbot::getPWM(::ScorbotIce::JointType joint, const ::Ice::Context* __ctx)
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
            __checkTwowayOnly(__ScorbotIce__Scorbot__getPWM_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::ScorbotIce::Scorbot* __del = dynamic_cast< ::IceDelegate::ScorbotIce::Scorbot*>(__delBase.get());
            return __del->getPWM(joint, __ctx);
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

::ScorbotIce::pwmValsType
IceProxy::ScorbotIce::Scorbot::getPWMs(const ::Ice::Context* __ctx)
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
            __checkTwowayOnly(__ScorbotIce__Scorbot__getPWMs_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::ScorbotIce::Scorbot* __del = dynamic_cast< ::IceDelegate::ScorbotIce::Scorbot*>(__delBase.get());
            return __del->getPWMs(__ctx);
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
IceProxy::ScorbotIce::Scorbot::setControlParams(::ScorbotIce::JointType joint, ::Ice::Float pGain, ::Ice::Float iGain, ::Ice::Float dGain, ::Ice::Float maxI, ::Ice::Float maxPWM, ::Ice::Float pwmOffset, const ::Ice::Context* __ctx)
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
            ::IceDelegate::ScorbotIce::Scorbot* __del = dynamic_cast< ::IceDelegate::ScorbotIce::Scorbot*>(__delBase.get());
            __del->setControlParams(joint, pGain, iGain, dGain, maxI, maxPWM, pwmOffset, __ctx);
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
IceProxy::ScorbotIce::Scorbot::getPIDVals(::ScorbotIce::JointType joint, ::Ice::Float& pGain, ::Ice::Float& iGain, ::Ice::Float& dGain, ::Ice::Float& maxI, ::Ice::Float& maxPWM, ::Ice::Float& pwmOffset, const ::Ice::Context* __ctx)
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
            __checkTwowayOnly(__ScorbotIce__Scorbot__getPIDVals_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::ScorbotIce::Scorbot* __del = dynamic_cast< ::IceDelegate::ScorbotIce::Scorbot*>(__delBase.get());
            __del->getPIDVals(joint, pGain, iGain, dGain, maxI, maxPWM, pwmOffset, __ctx);
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
IceProxy::ScorbotIce::Scorbot::getTuningVals(::ScorbotIce::JointType joint, ::Ice::Int& targetPos, ::Ice::Int& targetVel, ::Ice::Float& gravityCompensation, const ::Ice::Context* __ctx)
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
            __checkTwowayOnly(__ScorbotIce__Scorbot__getTuningVals_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::ScorbotIce::Scorbot* __del = dynamic_cast< ::IceDelegate::ScorbotIce::Scorbot*>(__delBase.get());
            __del->getTuningVals(joint, targetPos, targetVel, gravityCompensation, __ctx);
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
IceProxy::ScorbotIce::Scorbot::setGravityParameters(::Ice::Int upperArmMass, ::Ice::Int foreArmMass, ::Ice::Float compensationScale, const ::Ice::Context* __ctx)
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
            ::IceDelegate::ScorbotIce::Scorbot* __del = dynamic_cast< ::IceDelegate::ScorbotIce::Scorbot*>(__delBase.get());
            __del->setGravityParameters(upperArmMass, foreArmMass, compensationScale, __ctx);
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
IceProxy::ScorbotIce::Scorbot::getGravityParameters(::Ice::Int& upperArmMass, ::Ice::Int& foreArmMass, ::Ice::Float& compensationScale, const ::Ice::Context* __ctx)
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
            __checkTwowayOnly(__ScorbotIce__Scorbot__getGravityParameters_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::ScorbotIce::Scorbot* __del = dynamic_cast< ::IceDelegate::ScorbotIce::Scorbot*>(__delBase.get());
            __del->getGravityParameters(upperArmMass, foreArmMass, compensationScale, __ctx);
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
IceProxy::ScorbotIce::Scorbot::ice_staticId()
{
    return ::ScorbotIce::Scorbot::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::ScorbotIce::Scorbot::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::ScorbotIce::Scorbot);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::ScorbotIce::Scorbot::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::ScorbotIce::Scorbot);
}

::IceProxy::Ice::Object*
IceProxy::ScorbotIce::Scorbot::__newInstance() const
{
    return new Scorbot;
}

void
IceDelegateM::ScorbotIce::Scorbot::setJoint(::ScorbotIce::JointType joint, ::Ice::Int encoderPos, ::Ice::Int timeMS, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __ScorbotIce__Scorbot__setJoint_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        ::ScorbotIce::__write(__os, joint);
        __os->write(encoderPos);
        __os->write(timeMS);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
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
IceDelegateM::ScorbotIce::Scorbot::setJoints(const ::ScorbotIce::encoderValsType& pos, ::Ice::Int timeMS, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __ScorbotIce__Scorbot__setJoints_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        ::ScorbotIce::__writeencoderValsType(__os, pos);
        __os->write(timeMS);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
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

::Ice::Int
IceDelegateM::ScorbotIce::Scorbot::getEncoder(::ScorbotIce::JointType joint, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __ScorbotIce__Scorbot__getEncoder_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        ::ScorbotIce::__write(__os, joint);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    ::Ice::Int __ret;
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

::ScorbotIce::encoderValsType
IceDelegateM::ScorbotIce::Scorbot::getEncoders(const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __ScorbotIce__Scorbot__getEncoders_name, ::Ice::Normal, __context);
    bool __ok = __og.invoke();
    ::ScorbotIce::encoderValsType __ret;
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
        ::ScorbotIce::__readencoderValsType(__is, __ret);
        __is->endReadEncaps();
        return __ret;
    }
    catch(const ::Ice::LocalException& __ex)
    {
        throw ::IceInternal::LocalExceptionWrapper(__ex, false);
    }
}

void
IceDelegateM::ScorbotIce::Scorbot::setEnabled(bool enabled, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __ScorbotIce__Scorbot__setEnabled_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        __os->write(enabled);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
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
IceDelegateM::ScorbotIce::Scorbot::resetEncoders(const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __ScorbotIce__Scorbot__resetEncoders_name, ::Ice::Normal, __context);
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

::Ice::Float
IceDelegateM::ScorbotIce::Scorbot::getPWM(::ScorbotIce::JointType joint, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __ScorbotIce__Scorbot__getPWM_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        ::ScorbotIce::__write(__os, joint);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    ::Ice::Float __ret;
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

::ScorbotIce::pwmValsType
IceDelegateM::ScorbotIce::Scorbot::getPWMs(const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __ScorbotIce__Scorbot__getPWMs_name, ::Ice::Normal, __context);
    bool __ok = __og.invoke();
    ::ScorbotIce::pwmValsType __ret;
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
        ::ScorbotIce::__readpwmValsType(__is, __ret);
        __is->endReadEncaps();
        return __ret;
    }
    catch(const ::Ice::LocalException& __ex)
    {
        throw ::IceInternal::LocalExceptionWrapper(__ex, false);
    }
}

void
IceDelegateM::ScorbotIce::Scorbot::setControlParams(::ScorbotIce::JointType joint, ::Ice::Float pGain, ::Ice::Float iGain, ::Ice::Float dGain, ::Ice::Float maxI, ::Ice::Float maxPWM, ::Ice::Float pwmOffset, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __ScorbotIce__Scorbot__setControlParams_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        ::ScorbotIce::__write(__os, joint);
        __os->write(pGain);
        __os->write(iGain);
        __os->write(dGain);
        __os->write(maxI);
        __os->write(maxPWM);
        __os->write(pwmOffset);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
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
IceDelegateM::ScorbotIce::Scorbot::getPIDVals(::ScorbotIce::JointType joint, ::Ice::Float& pGain, ::Ice::Float& iGain, ::Ice::Float& dGain, ::Ice::Float& maxI, ::Ice::Float& maxPWM, ::Ice::Float& pwmOffset, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __ScorbotIce__Scorbot__getPIDVals_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        ::ScorbotIce::__write(__os, joint);
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
        ::IceInternal::BasicStream* __is = __og.is();
        __is->startReadEncaps();
        __is->read(pGain);
        __is->read(iGain);
        __is->read(dGain);
        __is->read(maxI);
        __is->read(maxPWM);
        __is->read(pwmOffset);
        __is->endReadEncaps();
    }
    catch(const ::Ice::LocalException& __ex)
    {
        throw ::IceInternal::LocalExceptionWrapper(__ex, false);
    }
}

void
IceDelegateM::ScorbotIce::Scorbot::getTuningVals(::ScorbotIce::JointType joint, ::Ice::Int& targetPos, ::Ice::Int& targetVel, ::Ice::Float& gravityCompensation, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __ScorbotIce__Scorbot__getTuningVals_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        ::ScorbotIce::__write(__os, joint);
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
        ::IceInternal::BasicStream* __is = __og.is();
        __is->startReadEncaps();
        __is->read(targetPos);
        __is->read(targetVel);
        __is->read(gravityCompensation);
        __is->endReadEncaps();
    }
    catch(const ::Ice::LocalException& __ex)
    {
        throw ::IceInternal::LocalExceptionWrapper(__ex, false);
    }
}

void
IceDelegateM::ScorbotIce::Scorbot::setGravityParameters(::Ice::Int upperArmMass, ::Ice::Int foreArmMass, ::Ice::Float compensationScale, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __ScorbotIce__Scorbot__setGravityParameters_name, ::Ice::Normal, __context);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        __os->write(upperArmMass);
        __os->write(foreArmMass);
        __os->write(compensationScale);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
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
IceDelegateM::ScorbotIce::Scorbot::getGravityParameters(::Ice::Int& upperArmMass, ::Ice::Int& foreArmMass, ::Ice::Float& compensationScale, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__handler.get(), __ScorbotIce__Scorbot__getGravityParameters_name, ::Ice::Normal, __context);
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
        ::IceInternal::BasicStream* __is = __og.is();
        __is->startReadEncaps();
        __is->read(upperArmMass);
        __is->read(foreArmMass);
        __is->read(compensationScale);
        __is->endReadEncaps();
    }
    catch(const ::Ice::LocalException& __ex)
    {
        throw ::IceInternal::LocalExceptionWrapper(__ex, false);
    }
}

void
IceDelegateD::ScorbotIce::Scorbot::setJoint(::ScorbotIce::JointType joint, ::Ice::Int encoderPos, ::Ice::Int timeMS, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(::ScorbotIce::JointType joint, ::Ice::Int encoderPos, ::Ice::Int timeMS, const ::Ice::Current& __current) :
            ::IceInternal::Direct(__current),
            _m_joint(joint),
            _m_encoderPos(encoderPos),
            _m_timeMS(timeMS)
        {
        }

        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::ScorbotIce::Scorbot* servant = dynamic_cast< ::ScorbotIce::Scorbot*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->setJoint(_m_joint, _m_encoderPos, _m_timeMS, _current);
            return ::Ice::DispatchOK;
        }

    private:

        ::ScorbotIce::JointType _m_joint;
        ::Ice::Int _m_encoderPos;
        ::Ice::Int _m_timeMS;
    };

    ::Ice::Current __current;
    __initCurrent(__current, __ScorbotIce__Scorbot__setJoint_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(joint, encoderPos, timeMS, __current);
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
IceDelegateD::ScorbotIce::Scorbot::setJoints(const ::ScorbotIce::encoderValsType& pos, ::Ice::Int timeMS, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::ScorbotIce::encoderValsType& pos, ::Ice::Int timeMS, const ::Ice::Current& __current) :
            ::IceInternal::Direct(__current),
            _m_pos(pos),
            _m_timeMS(timeMS)
        {
        }

        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::ScorbotIce::Scorbot* servant = dynamic_cast< ::ScorbotIce::Scorbot*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->setJoints(_m_pos, _m_timeMS, _current);
            return ::Ice::DispatchOK;
        }

    private:

        const ::ScorbotIce::encoderValsType& _m_pos;
        ::Ice::Int _m_timeMS;
    };

    ::Ice::Current __current;
    __initCurrent(__current, __ScorbotIce__Scorbot__setJoints_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(pos, timeMS, __current);
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

::Ice::Int
IceDelegateD::ScorbotIce::Scorbot::getEncoder(::ScorbotIce::JointType joint, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(::Ice::Int& __result, ::ScorbotIce::JointType joint, const ::Ice::Current& __current) :
            ::IceInternal::Direct(__current),
            _result(__result),
            _m_joint(joint)
        {
        }

        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::ScorbotIce::Scorbot* servant = dynamic_cast< ::ScorbotIce::Scorbot*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            _result = servant->getEncoder(_m_joint, _current);
            return ::Ice::DispatchOK;
        }

    private:

        ::Ice::Int& _result;
        ::ScorbotIce::JointType _m_joint;
    };

    ::Ice::Current __current;
    __initCurrent(__current, __ScorbotIce__Scorbot__getEncoder_name, ::Ice::Normal, __context);
    ::Ice::Int __result;
    try
    {
        _DirectI __direct(__result, joint, __current);
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

::ScorbotIce::encoderValsType
IceDelegateD::ScorbotIce::Scorbot::getEncoders(const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(::ScorbotIce::encoderValsType& __result, const ::Ice::Current& __current) :
            ::IceInternal::Direct(__current),
            _result(__result)
        {
        }

        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::ScorbotIce::Scorbot* servant = dynamic_cast< ::ScorbotIce::Scorbot*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            _result = servant->getEncoders(_current);
            return ::Ice::DispatchOK;
        }

    private:

        ::ScorbotIce::encoderValsType& _result;
    };

    ::Ice::Current __current;
    __initCurrent(__current, __ScorbotIce__Scorbot__getEncoders_name, ::Ice::Normal, __context);
    ::ScorbotIce::encoderValsType __result;
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
IceDelegateD::ScorbotIce::Scorbot::setEnabled(bool enabled, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(bool enabled, const ::Ice::Current& __current) :
            ::IceInternal::Direct(__current),
            _m_enabled(enabled)
        {
        }

        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::ScorbotIce::Scorbot* servant = dynamic_cast< ::ScorbotIce::Scorbot*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->setEnabled(_m_enabled, _current);
            return ::Ice::DispatchOK;
        }

    private:

        bool _m_enabled;
    };

    ::Ice::Current __current;
    __initCurrent(__current, __ScorbotIce__Scorbot__setEnabled_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(enabled, __current);
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
IceDelegateD::ScorbotIce::Scorbot::resetEncoders(const ::Ice::Context* __context)
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
            ::ScorbotIce::Scorbot* servant = dynamic_cast< ::ScorbotIce::Scorbot*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->resetEncoders(_current);
            return ::Ice::DispatchOK;
        }

    private:

    };

    ::Ice::Current __current;
    __initCurrent(__current, __ScorbotIce__Scorbot__resetEncoders_name, ::Ice::Normal, __context);
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

::Ice::Float
IceDelegateD::ScorbotIce::Scorbot::getPWM(::ScorbotIce::JointType joint, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(::Ice::Float& __result, ::ScorbotIce::JointType joint, const ::Ice::Current& __current) :
            ::IceInternal::Direct(__current),
            _result(__result),
            _m_joint(joint)
        {
        }

        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::ScorbotIce::Scorbot* servant = dynamic_cast< ::ScorbotIce::Scorbot*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            _result = servant->getPWM(_m_joint, _current);
            return ::Ice::DispatchOK;
        }

    private:

        ::Ice::Float& _result;
        ::ScorbotIce::JointType _m_joint;
    };

    ::Ice::Current __current;
    __initCurrent(__current, __ScorbotIce__Scorbot__getPWM_name, ::Ice::Normal, __context);
    ::Ice::Float __result;
    try
    {
        _DirectI __direct(__result, joint, __current);
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

::ScorbotIce::pwmValsType
IceDelegateD::ScorbotIce::Scorbot::getPWMs(const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(::ScorbotIce::pwmValsType& __result, const ::Ice::Current& __current) :
            ::IceInternal::Direct(__current),
            _result(__result)
        {
        }

        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::ScorbotIce::Scorbot* servant = dynamic_cast< ::ScorbotIce::Scorbot*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            _result = servant->getPWMs(_current);
            return ::Ice::DispatchOK;
        }

    private:

        ::ScorbotIce::pwmValsType& _result;
    };

    ::Ice::Current __current;
    __initCurrent(__current, __ScorbotIce__Scorbot__getPWMs_name, ::Ice::Normal, __context);
    ::ScorbotIce::pwmValsType __result;
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
IceDelegateD::ScorbotIce::Scorbot::setControlParams(::ScorbotIce::JointType joint, ::Ice::Float pGain, ::Ice::Float iGain, ::Ice::Float dGain, ::Ice::Float maxI, ::Ice::Float maxPWM, ::Ice::Float pwmOffset, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(::ScorbotIce::JointType joint, ::Ice::Float pGain, ::Ice::Float iGain, ::Ice::Float dGain, ::Ice::Float maxI, ::Ice::Float maxPWM, ::Ice::Float pwmOffset, const ::Ice::Current& __current) :
            ::IceInternal::Direct(__current),
            _m_joint(joint),
            _m_pGain(pGain),
            _m_iGain(iGain),
            _m_dGain(dGain),
            _m_maxI(maxI),
            _m_maxPWM(maxPWM),
            _m_pwmOffset(pwmOffset)
        {
        }

        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::ScorbotIce::Scorbot* servant = dynamic_cast< ::ScorbotIce::Scorbot*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->setControlParams(_m_joint, _m_pGain, _m_iGain, _m_dGain, _m_maxI, _m_maxPWM, _m_pwmOffset, _current);
            return ::Ice::DispatchOK;
        }

    private:

        ::ScorbotIce::JointType _m_joint;
        ::Ice::Float _m_pGain;
        ::Ice::Float _m_iGain;
        ::Ice::Float _m_dGain;
        ::Ice::Float _m_maxI;
        ::Ice::Float _m_maxPWM;
        ::Ice::Float _m_pwmOffset;
    };

    ::Ice::Current __current;
    __initCurrent(__current, __ScorbotIce__Scorbot__setControlParams_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(joint, pGain, iGain, dGain, maxI, maxPWM, pwmOffset, __current);
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
IceDelegateD::ScorbotIce::Scorbot::getPIDVals(::ScorbotIce::JointType joint, ::Ice::Float& pGain, ::Ice::Float& iGain, ::Ice::Float& dGain, ::Ice::Float& maxI, ::Ice::Float& maxPWM, ::Ice::Float& pwmOffset, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(::ScorbotIce::JointType joint, ::Ice::Float& pGain, ::Ice::Float& iGain, ::Ice::Float& dGain, ::Ice::Float& maxI, ::Ice::Float& maxPWM, ::Ice::Float& pwmOffset, const ::Ice::Current& __current) :
            ::IceInternal::Direct(__current),
            _m_joint(joint),
            _m_pGain(pGain),
            _m_iGain(iGain),
            _m_dGain(dGain),
            _m_maxI(maxI),
            _m_maxPWM(maxPWM),
            _m_pwmOffset(pwmOffset)
        {
        }

        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::ScorbotIce::Scorbot* servant = dynamic_cast< ::ScorbotIce::Scorbot*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->getPIDVals(_m_joint, _m_pGain, _m_iGain, _m_dGain, _m_maxI, _m_maxPWM, _m_pwmOffset, _current);
            return ::Ice::DispatchOK;
        }

    private:

        ::ScorbotIce::JointType _m_joint;
        ::Ice::Float& _m_pGain;
        ::Ice::Float& _m_iGain;
        ::Ice::Float& _m_dGain;
        ::Ice::Float& _m_maxI;
        ::Ice::Float& _m_maxPWM;
        ::Ice::Float& _m_pwmOffset;
    };

    ::Ice::Current __current;
    __initCurrent(__current, __ScorbotIce__Scorbot__getPIDVals_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(joint, pGain, iGain, dGain, maxI, maxPWM, pwmOffset, __current);
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
IceDelegateD::ScorbotIce::Scorbot::getTuningVals(::ScorbotIce::JointType joint, ::Ice::Int& targetPos, ::Ice::Int& targetVel, ::Ice::Float& gravityCompensation, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(::ScorbotIce::JointType joint, ::Ice::Int& targetPos, ::Ice::Int& targetVel, ::Ice::Float& gravityCompensation, const ::Ice::Current& __current) :
            ::IceInternal::Direct(__current),
            _m_joint(joint),
            _m_targetPos(targetPos),
            _m_targetVel(targetVel),
            _m_gravityCompensation(gravityCompensation)
        {
        }

        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::ScorbotIce::Scorbot* servant = dynamic_cast< ::ScorbotIce::Scorbot*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->getTuningVals(_m_joint, _m_targetPos, _m_targetVel, _m_gravityCompensation, _current);
            return ::Ice::DispatchOK;
        }

    private:

        ::ScorbotIce::JointType _m_joint;
        ::Ice::Int& _m_targetPos;
        ::Ice::Int& _m_targetVel;
        ::Ice::Float& _m_gravityCompensation;
    };

    ::Ice::Current __current;
    __initCurrent(__current, __ScorbotIce__Scorbot__getTuningVals_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(joint, targetPos, targetVel, gravityCompensation, __current);
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
IceDelegateD::ScorbotIce::Scorbot::setGravityParameters(::Ice::Int upperArmMass, ::Ice::Int foreArmMass, ::Ice::Float compensationScale, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(::Ice::Int upperArmMass, ::Ice::Int foreArmMass, ::Ice::Float compensationScale, const ::Ice::Current& __current) :
            ::IceInternal::Direct(__current),
            _m_upperArmMass(upperArmMass),
            _m_foreArmMass(foreArmMass),
            _m_compensationScale(compensationScale)
        {
        }

        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::ScorbotIce::Scorbot* servant = dynamic_cast< ::ScorbotIce::Scorbot*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->setGravityParameters(_m_upperArmMass, _m_foreArmMass, _m_compensationScale, _current);
            return ::Ice::DispatchOK;
        }

    private:

        ::Ice::Int _m_upperArmMass;
        ::Ice::Int _m_foreArmMass;
        ::Ice::Float _m_compensationScale;
    };

    ::Ice::Current __current;
    __initCurrent(__current, __ScorbotIce__Scorbot__setGravityParameters_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(upperArmMass, foreArmMass, compensationScale, __current);
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
IceDelegateD::ScorbotIce::Scorbot::getGravityParameters(::Ice::Int& upperArmMass, ::Ice::Int& foreArmMass, ::Ice::Float& compensationScale, const ::Ice::Context* __context)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(::Ice::Int& upperArmMass, ::Ice::Int& foreArmMass, ::Ice::Float& compensationScale, const ::Ice::Current& __current) :
            ::IceInternal::Direct(__current),
            _m_upperArmMass(upperArmMass),
            _m_foreArmMass(foreArmMass),
            _m_compensationScale(compensationScale)
        {
        }

        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::ScorbotIce::Scorbot* servant = dynamic_cast< ::ScorbotIce::Scorbot*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->getGravityParameters(_m_upperArmMass, _m_foreArmMass, _m_compensationScale, _current);
            return ::Ice::DispatchOK;
        }

    private:

        ::Ice::Int& _m_upperArmMass;
        ::Ice::Int& _m_foreArmMass;
        ::Ice::Float& _m_compensationScale;
    };

    ::Ice::Current __current;
    __initCurrent(__current, __ScorbotIce__Scorbot__getGravityParameters_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(upperArmMass, foreArmMass, compensationScale, __current);
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
ScorbotIce::Scorbot::ice_clone() const
{
    throw ::Ice::CloneNotImplementedException(__FILE__, __LINE__);
    return 0; // to avoid a warning with some compilers
}

static const ::std::string __ScorbotIce__Scorbot_ids[2] =
{
    "::Ice::Object",
    "::ScorbotIce::Scorbot"
};

bool
ScorbotIce::Scorbot::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__ScorbotIce__Scorbot_ids, __ScorbotIce__Scorbot_ids + 2, _s);
}

::std::vector< ::std::string>
ScorbotIce::Scorbot::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__ScorbotIce__Scorbot_ids[0], &__ScorbotIce__Scorbot_ids[2]);
}

const ::std::string&
ScorbotIce::Scorbot::ice_id(const ::Ice::Current&) const
{
    return __ScorbotIce__Scorbot_ids[1];
}

const ::std::string&
ScorbotIce::Scorbot::ice_staticId()
{
    return __ScorbotIce__Scorbot_ids[1];
}

::Ice::DispatchStatus
ScorbotIce::Scorbot::___setJoint(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::ScorbotIce::JointType joint;
    ::Ice::Int encoderPos;
    ::Ice::Int timeMS;
    ::ScorbotIce::__read(__is, joint);
    __is->read(encoderPos);
    __is->read(timeMS);
    __is->endReadEncaps();
    setJoint(joint, encoderPos, timeMS, __current);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
ScorbotIce::Scorbot::___setJoints(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::ScorbotIce::encoderValsType pos;
    ::Ice::Int timeMS;
    ::ScorbotIce::__readencoderValsType(__is, pos);
    __is->read(timeMS);
    __is->endReadEncaps();
    setJoints(pos, timeMS, __current);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
ScorbotIce::Scorbot::___getEncoder(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::ScorbotIce::JointType joint;
    ::ScorbotIce::__read(__is, joint);
    __is->endReadEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    ::Ice::Int __ret = getEncoder(joint, __current);
    __os->write(__ret);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
ScorbotIce::Scorbot::___getEncoders(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    __inS.is()->skipEmptyEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    ::ScorbotIce::encoderValsType __ret = getEncoders(__current);
    ::ScorbotIce::__writeencoderValsType(__os, __ret);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
ScorbotIce::Scorbot::___setEnabled(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    bool enabled;
    __is->read(enabled);
    __is->endReadEncaps();
    setEnabled(enabled, __current);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
ScorbotIce::Scorbot::___resetEncoders(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    __inS.is()->skipEmptyEncaps();
    resetEncoders(__current);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
ScorbotIce::Scorbot::___getPWM(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::ScorbotIce::JointType joint;
    ::ScorbotIce::__read(__is, joint);
    __is->endReadEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    ::Ice::Float __ret = getPWM(joint, __current);
    __os->write(__ret);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
ScorbotIce::Scorbot::___getPWMs(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    __inS.is()->skipEmptyEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    ::ScorbotIce::pwmValsType __ret = getPWMs(__current);
    ::ScorbotIce::__writepwmValsType(__os, __ret);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
ScorbotIce::Scorbot::___setControlParams(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::ScorbotIce::JointType joint;
    ::Ice::Float pGain;
    ::Ice::Float iGain;
    ::Ice::Float dGain;
    ::Ice::Float maxI;
    ::Ice::Float maxPWM;
    ::Ice::Float pwmOffset;
    ::ScorbotIce::__read(__is, joint);
    __is->read(pGain);
    __is->read(iGain);
    __is->read(dGain);
    __is->read(maxI);
    __is->read(maxPWM);
    __is->read(pwmOffset);
    __is->endReadEncaps();
    setControlParams(joint, pGain, iGain, dGain, maxI, maxPWM, pwmOffset, __current);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
ScorbotIce::Scorbot::___getPIDVals(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::ScorbotIce::JointType joint;
    ::ScorbotIce::__read(__is, joint);
    __is->endReadEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    ::Ice::Float pGain;
    ::Ice::Float iGain;
    ::Ice::Float dGain;
    ::Ice::Float maxI;
    ::Ice::Float maxPWM;
    ::Ice::Float pwmOffset;
    getPIDVals(joint, pGain, iGain, dGain, maxI, maxPWM, pwmOffset, __current);
    __os->write(pGain);
    __os->write(iGain);
    __os->write(dGain);
    __os->write(maxI);
    __os->write(maxPWM);
    __os->write(pwmOffset);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
ScorbotIce::Scorbot::___getTuningVals(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::ScorbotIce::JointType joint;
    ::ScorbotIce::__read(__is, joint);
    __is->endReadEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    ::Ice::Int targetPos;
    ::Ice::Int targetVel;
    ::Ice::Float gravityCompensation;
    getTuningVals(joint, targetPos, targetVel, gravityCompensation, __current);
    __os->write(targetPos);
    __os->write(targetVel);
    __os->write(gravityCompensation);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
ScorbotIce::Scorbot::___setGravityParameters(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    __is->startReadEncaps();
    ::Ice::Int upperArmMass;
    ::Ice::Int foreArmMass;
    ::Ice::Float compensationScale;
    __is->read(upperArmMass);
    __is->read(foreArmMass);
    __is->read(compensationScale);
    __is->endReadEncaps();
    setGravityParameters(upperArmMass, foreArmMass, compensationScale, __current);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
ScorbotIce::Scorbot::___getGravityParameters(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    __inS.is()->skipEmptyEncaps();
    ::IceInternal::BasicStream* __os = __inS.os();
    ::Ice::Int upperArmMass;
    ::Ice::Int foreArmMass;
    ::Ice::Float compensationScale;
    getGravityParameters(upperArmMass, foreArmMass, compensationScale, __current);
    __os->write(upperArmMass);
    __os->write(foreArmMass);
    __os->write(compensationScale);
    return ::Ice::DispatchOK;
}

static ::std::string __ScorbotIce__Scorbot_all[] =
{
    "getEncoder",
    "getEncoders",
    "getGravityParameters",
    "getPIDVals",
    "getPWM",
    "getPWMs",
    "getTuningVals",
    "ice_id",
    "ice_ids",
    "ice_isA",
    "ice_ping",
    "resetEncoders",
    "setControlParams",
    "setEnabled",
    "setGravityParameters",
    "setJoint",
    "setJoints"
};

::Ice::DispatchStatus
ScorbotIce::Scorbot::__dispatch(::IceInternal::Incoming& in, const ::Ice::Current& current)
{
    ::std::pair< ::std::string*, ::std::string*> r = ::std::equal_range(__ScorbotIce__Scorbot_all, __ScorbotIce__Scorbot_all + 17, current.operation);
    if(r.first == r.second)
    {
        throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
    }

    switch(r.first - __ScorbotIce__Scorbot_all)
    {
        case 0:
        {
            return ___getEncoder(in, current);
        }
        case 1:
        {
            return ___getEncoders(in, current);
        }
        case 2:
        {
            return ___getGravityParameters(in, current);
        }
        case 3:
        {
            return ___getPIDVals(in, current);
        }
        case 4:
        {
            return ___getPWM(in, current);
        }
        case 5:
        {
            return ___getPWMs(in, current);
        }
        case 6:
        {
            return ___getTuningVals(in, current);
        }
        case 7:
        {
            return ___ice_id(in, current);
        }
        case 8:
        {
            return ___ice_ids(in, current);
        }
        case 9:
        {
            return ___ice_isA(in, current);
        }
        case 10:
        {
            return ___ice_ping(in, current);
        }
        case 11:
        {
            return ___resetEncoders(in, current);
        }
        case 12:
        {
            return ___setControlParams(in, current);
        }
        case 13:
        {
            return ___setEnabled(in, current);
        }
        case 14:
        {
            return ___setGravityParameters(in, current);
        }
        case 15:
        {
            return ___setJoint(in, current);
        }
        case 16:
        {
            return ___setJoints(in, current);
        }
    }

    assert(false);
    throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
}

void
ScorbotIce::Scorbot::__write(::IceInternal::BasicStream* __os) const
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
ScorbotIce::Scorbot::__read(::IceInternal::BasicStream* __is, bool __rid)
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
ScorbotIce::Scorbot::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type ScorbotIce::Scorbot was not generated with stream support";
    throw ex;
}

void
ScorbotIce::Scorbot::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type ScorbotIce::Scorbot was not generated with stream support";
    throw ex;
}

void
ScorbotIce::__patch__ScorbotPtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::ScorbotIce::ScorbotPtr* p = static_cast< ::ScorbotIce::ScorbotPtr*>(__addr);
    assert(p);
    *p = ::ScorbotIce::ScorbotPtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::ScorbotIce::Scorbot::ice_staticId(), v->ice_id());
    }
}

bool
ScorbotIce::operator==(const ::ScorbotIce::Scorbot& l, const ::ScorbotIce::Scorbot& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
ScorbotIce::operator<(const ::ScorbotIce::Scorbot& l, const ::ScorbotIce::Scorbot& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}
