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

#ifndef __src_Robots_Scorbot__Scorbot_ice_H__
#define __src_Robots_Scorbot__Scorbot_ice_H__

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
#   if ICE_INT_VERSION % 100 < 1
#       error Ice patch level mismatch!
#   endif
#endif

namespace IceProxy
{

namespace ScorbotIce
{

class Scorbot;

}

}

namespace ScorbotIce
{

class Scorbot;
bool operator==(const Scorbot&, const Scorbot&);
bool operator<(const Scorbot&, const Scorbot&);

}

namespace IceInternal
{

::Ice::Object* upCast(::ScorbotIce::Scorbot*);
::IceProxy::Ice::Object* upCast(::IceProxy::ScorbotIce::Scorbot*);

}

namespace ScorbotIce
{

typedef ::IceInternal::Handle< ::ScorbotIce::Scorbot> ScorbotPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::ScorbotIce::Scorbot> ScorbotPrx;

void __read(::IceInternal::BasicStream*, ScorbotPrx&);
void __patch__ScorbotPtr(void*, ::Ice::ObjectPtr&);

}

namespace ScorbotIce
{

enum JointType
{
    Base,
    Shoulder,
    Elbow,
    Wrist1,
    Wrist2,
    Gripper,
    Slider
};

void __write(::IceInternal::BasicStream*, JointType);
void __read(::IceInternal::BasicStream*, JointType&);

typedef ::std::map< ::ScorbotIce::JointType, ::Ice::Int> encoderValsType;
void __writeencoderValsType(::IceInternal::BasicStream*, const encoderValsType&);
void __readencoderValsType(::IceInternal::BasicStream*, encoderValsType&);

typedef ::std::map< ::ScorbotIce::JointType, ::Ice::Float> pwmValsType;
void __writepwmValsType(::IceInternal::BasicStream*, const pwmValsType&);
void __readpwmValsType(::IceInternal::BasicStream*, pwmValsType&);

}

namespace IceProxy
{

namespace ScorbotIce
{

class Scorbot : virtual public ::IceProxy::Ice::Object
{
public:

    void setJoint(::ScorbotIce::JointType joint, ::Ice::Int encoderPos, ::Ice::Int timeMS)
    {
        setJoint(joint, encoderPos, timeMS, 0);
    }
    void setJoint(::ScorbotIce::JointType joint, ::Ice::Int encoderPos, ::Ice::Int timeMS, const ::Ice::Context& __ctx)
    {
        setJoint(joint, encoderPos, timeMS, &__ctx);
    }

private:

    void setJoint(::ScorbotIce::JointType, ::Ice::Int, ::Ice::Int, const ::Ice::Context*);

public:

    void setJoints(const ::ScorbotIce::encoderValsType& pos, ::Ice::Int timeMS)
    {
        setJoints(pos, timeMS, 0);
    }
    void setJoints(const ::ScorbotIce::encoderValsType& pos, ::Ice::Int timeMS, const ::Ice::Context& __ctx)
    {
        setJoints(pos, timeMS, &__ctx);
    }

private:

    void setJoints(const ::ScorbotIce::encoderValsType&, ::Ice::Int, const ::Ice::Context*);

public:

    ::Ice::Int getEncoder(::ScorbotIce::JointType joint)
    {
        return getEncoder(joint, 0);
    }
    ::Ice::Int getEncoder(::ScorbotIce::JointType joint, const ::Ice::Context& __ctx)
    {
        return getEncoder(joint, &__ctx);
    }

private:

    ::Ice::Int getEncoder(::ScorbotIce::JointType, const ::Ice::Context*);

public:

    ::ScorbotIce::encoderValsType getEncoders()
    {
        return getEncoders(0);
    }
    ::ScorbotIce::encoderValsType getEncoders(const ::Ice::Context& __ctx)
    {
        return getEncoders(&__ctx);
    }

private:

    ::ScorbotIce::encoderValsType getEncoders(const ::Ice::Context*);

public:

    void setEnabled(bool enabled)
    {
        setEnabled(enabled, 0);
    }
    void setEnabled(bool enabled, const ::Ice::Context& __ctx)
    {
        setEnabled(enabled, &__ctx);
    }

private:

    void setEnabled(bool, const ::Ice::Context*);

public:

    void resetEncoders()
    {
        resetEncoders(0);
    }
    void resetEncoders(const ::Ice::Context& __ctx)
    {
        resetEncoders(&__ctx);
    }

private:

    void resetEncoders(const ::Ice::Context*);

public:

    ::Ice::Float getPWM(::ScorbotIce::JointType joint)
    {
        return getPWM(joint, 0);
    }
    ::Ice::Float getPWM(::ScorbotIce::JointType joint, const ::Ice::Context& __ctx)
    {
        return getPWM(joint, &__ctx);
    }

private:

    ::Ice::Float getPWM(::ScorbotIce::JointType, const ::Ice::Context*);

public:

    ::ScorbotIce::pwmValsType getPWMs()
    {
        return getPWMs(0);
    }
    ::ScorbotIce::pwmValsType getPWMs(const ::Ice::Context& __ctx)
    {
        return getPWMs(&__ctx);
    }

private:

    ::ScorbotIce::pwmValsType getPWMs(const ::Ice::Context*);

public:

    void setControlParams(::ScorbotIce::JointType joint, ::Ice::Float pGain, ::Ice::Float iGain, ::Ice::Float dGain, ::Ice::Float maxI, ::Ice::Float maxPWM, ::Ice::Float pwmOffset)
    {
        setControlParams(joint, pGain, iGain, dGain, maxI, maxPWM, pwmOffset, 0);
    }
    void setControlParams(::ScorbotIce::JointType joint, ::Ice::Float pGain, ::Ice::Float iGain, ::Ice::Float dGain, ::Ice::Float maxI, ::Ice::Float maxPWM, ::Ice::Float pwmOffset, const ::Ice::Context& __ctx)
    {
        setControlParams(joint, pGain, iGain, dGain, maxI, maxPWM, pwmOffset, &__ctx);
    }

private:

    void setControlParams(::ScorbotIce::JointType, ::Ice::Float, ::Ice::Float, ::Ice::Float, ::Ice::Float, ::Ice::Float, ::Ice::Float, const ::Ice::Context*);

public:

    void getPIDVals(::ScorbotIce::JointType joint, ::Ice::Float& pGain, ::Ice::Float& iGain, ::Ice::Float& dGain, ::Ice::Float& maxI, ::Ice::Float& maxPWM, ::Ice::Float& pwmOffset)
    {
        getPIDVals(joint, pGain, iGain, dGain, maxI, maxPWM, pwmOffset, 0);
    }
    void getPIDVals(::ScorbotIce::JointType joint, ::Ice::Float& pGain, ::Ice::Float& iGain, ::Ice::Float& dGain, ::Ice::Float& maxI, ::Ice::Float& maxPWM, ::Ice::Float& pwmOffset, const ::Ice::Context& __ctx)
    {
        getPIDVals(joint, pGain, iGain, dGain, maxI, maxPWM, pwmOffset, &__ctx);
    }

private:

    void getPIDVals(::ScorbotIce::JointType, ::Ice::Float&, ::Ice::Float&, ::Ice::Float&, ::Ice::Float&, ::Ice::Float&, ::Ice::Float&, const ::Ice::Context*);

public:

    void getTuningVals(::ScorbotIce::JointType joint, ::Ice::Int& targetPos, ::Ice::Int& targetVel, ::Ice::Float& gravityCompensation)
    {
        getTuningVals(joint, targetPos, targetVel, gravityCompensation, 0);
    }
    void getTuningVals(::ScorbotIce::JointType joint, ::Ice::Int& targetPos, ::Ice::Int& targetVel, ::Ice::Float& gravityCompensation, const ::Ice::Context& __ctx)
    {
        getTuningVals(joint, targetPos, targetVel, gravityCompensation, &__ctx);
    }

private:

    void getTuningVals(::ScorbotIce::JointType, ::Ice::Int&, ::Ice::Int&, ::Ice::Float&, const ::Ice::Context*);

public:

    void setGravityParameters(::Ice::Int upperArmMass, ::Ice::Int foreArmMass, ::Ice::Float compensationScale)
    {
        setGravityParameters(upperArmMass, foreArmMass, compensationScale, 0);
    }
    void setGravityParameters(::Ice::Int upperArmMass, ::Ice::Int foreArmMass, ::Ice::Float compensationScale, const ::Ice::Context& __ctx)
    {
        setGravityParameters(upperArmMass, foreArmMass, compensationScale, &__ctx);
    }

private:

    void setGravityParameters(::Ice::Int, ::Ice::Int, ::Ice::Float, const ::Ice::Context*);

public:

    void getGravityParameters(::Ice::Int& upperArmMass, ::Ice::Int& foreArmMass, ::Ice::Float& compensationScale)
    {
        getGravityParameters(upperArmMass, foreArmMass, compensationScale, 0);
    }
    void getGravityParameters(::Ice::Int& upperArmMass, ::Ice::Int& foreArmMass, ::Ice::Float& compensationScale, const ::Ice::Context& __ctx)
    {
        getGravityParameters(upperArmMass, foreArmMass, compensationScale, &__ctx);
    }

private:

    void getGravityParameters(::Ice::Int&, ::Ice::Int&, ::Ice::Float&, const ::Ice::Context*);

public:

    ::IceInternal::ProxyHandle<Scorbot> ice_context(const ::Ice::Context& __context) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Scorbot*>(_Base::ice_context(__context).get());
    #else
        return dynamic_cast<Scorbot*>(::IceProxy::Ice::Object::ice_context(__context).get());
    #endif
    }

    ::IceInternal::ProxyHandle<Scorbot> ice_adapterId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Scorbot*>(_Base::ice_adapterId(__id).get());
    #else
        return dynamic_cast<Scorbot*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    #endif
    }

    ::IceInternal::ProxyHandle<Scorbot> ice_endpoints(const ::Ice::EndpointSeq& __endpoints) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Scorbot*>(_Base::ice_endpoints(__endpoints).get());
    #else
        return dynamic_cast<Scorbot*>(::IceProxy::Ice::Object::ice_endpoints(__endpoints).get());
    #endif
    }

    ::IceInternal::ProxyHandle<Scorbot> ice_locatorCacheTimeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Scorbot*>(_Base::ice_locatorCacheTimeout(__timeout).get());
    #else
        return dynamic_cast<Scorbot*>(::IceProxy::Ice::Object::ice_locatorCacheTimeout(__timeout).get());
    #endif
    }

    ::IceInternal::ProxyHandle<Scorbot> ice_connectionCached(bool __cached) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Scorbot*>(_Base::ice_connectionCached(__cached).get());
    #else
        return dynamic_cast<Scorbot*>(::IceProxy::Ice::Object::ice_connectionCached(__cached).get());
    #endif
    }

    ::IceInternal::ProxyHandle<Scorbot> ice_endpointSelection(::Ice::EndpointSelectionType __est) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Scorbot*>(_Base::ice_endpointSelection(__est).get());
    #else
        return dynamic_cast<Scorbot*>(::IceProxy::Ice::Object::ice_endpointSelection(__est).get());
    #endif
    }

    ::IceInternal::ProxyHandle<Scorbot> ice_secure(bool __secure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Scorbot*>(_Base::ice_secure(__secure).get());
    #else
        return dynamic_cast<Scorbot*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    #endif
    }

    ::IceInternal::ProxyHandle<Scorbot> ice_preferSecure(bool __preferSecure) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Scorbot*>(_Base::ice_preferSecure(__preferSecure).get());
    #else
        return dynamic_cast<Scorbot*>(::IceProxy::Ice::Object::ice_preferSecure(__preferSecure).get());
    #endif
    }

    ::IceInternal::ProxyHandle<Scorbot> ice_router(const ::Ice::RouterPrx& __router) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Scorbot*>(_Base::ice_router(__router).get());
    #else
        return dynamic_cast<Scorbot*>(::IceProxy::Ice::Object::ice_router(__router).get());
    #endif
    }

    ::IceInternal::ProxyHandle<Scorbot> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Scorbot*>(_Base::ice_locator(__locator).get());
    #else
        return dynamic_cast<Scorbot*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    #endif
    }

    ::IceInternal::ProxyHandle<Scorbot> ice_collocationOptimized(bool __co) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Scorbot*>(_Base::ice_collocationOptimized(__co).get());
    #else
        return dynamic_cast<Scorbot*>(::IceProxy::Ice::Object::ice_collocationOptimized(__co).get());
    #endif
    }

    ::IceInternal::ProxyHandle<Scorbot> ice_twoway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Scorbot*>(_Base::ice_twoway().get());
    #else
        return dynamic_cast<Scorbot*>(::IceProxy::Ice::Object::ice_twoway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<Scorbot> ice_oneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Scorbot*>(_Base::ice_oneway().get());
    #else
        return dynamic_cast<Scorbot*>(::IceProxy::Ice::Object::ice_oneway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<Scorbot> ice_batchOneway() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Scorbot*>(_Base::ice_batchOneway().get());
    #else
        return dynamic_cast<Scorbot*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    #endif
    }

    ::IceInternal::ProxyHandle<Scorbot> ice_datagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Scorbot*>(_Base::ice_datagram().get());
    #else
        return dynamic_cast<Scorbot*>(::IceProxy::Ice::Object::ice_datagram().get());
    #endif
    }

    ::IceInternal::ProxyHandle<Scorbot> ice_batchDatagram() const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Scorbot*>(_Base::ice_batchDatagram().get());
    #else
        return dynamic_cast<Scorbot*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    #endif
    }

    ::IceInternal::ProxyHandle<Scorbot> ice_compress(bool __compress) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Scorbot*>(_Base::ice_compress(__compress).get());
    #else
        return dynamic_cast<Scorbot*>(::IceProxy::Ice::Object::ice_compress(__compress).get());
    #endif
    }

    ::IceInternal::ProxyHandle<Scorbot> ice_timeout(int __timeout) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Scorbot*>(_Base::ice_timeout(__timeout).get());
    #else
        return dynamic_cast<Scorbot*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    #endif
    }

    ::IceInternal::ProxyHandle<Scorbot> ice_connectionId(const std::string& __id) const
    {
    #if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
        typedef ::IceProxy::Ice::Object _Base;
        return dynamic_cast<Scorbot*>(_Base::ice_connectionId(__id).get());
    #else
        return dynamic_cast<Scorbot*>(::IceProxy::Ice::Object::ice_connectionId(__id).get());
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

namespace ScorbotIce
{

class Scorbot : virtual public ::IceDelegate::Ice::Object
{
public:

    virtual void setJoint(::ScorbotIce::JointType, ::Ice::Int, ::Ice::Int, const ::Ice::Context*) = 0;

    virtual void setJoints(const ::ScorbotIce::encoderValsType&, ::Ice::Int, const ::Ice::Context*) = 0;

    virtual ::Ice::Int getEncoder(::ScorbotIce::JointType, const ::Ice::Context*) = 0;

    virtual ::ScorbotIce::encoderValsType getEncoders(const ::Ice::Context*) = 0;

    virtual void setEnabled(bool, const ::Ice::Context*) = 0;

    virtual void resetEncoders(const ::Ice::Context*) = 0;

    virtual ::Ice::Float getPWM(::ScorbotIce::JointType, const ::Ice::Context*) = 0;

    virtual ::ScorbotIce::pwmValsType getPWMs(const ::Ice::Context*) = 0;

    virtual void setControlParams(::ScorbotIce::JointType, ::Ice::Float, ::Ice::Float, ::Ice::Float, ::Ice::Float, ::Ice::Float, ::Ice::Float, const ::Ice::Context*) = 0;

    virtual void getPIDVals(::ScorbotIce::JointType, ::Ice::Float&, ::Ice::Float&, ::Ice::Float&, ::Ice::Float&, ::Ice::Float&, ::Ice::Float&, const ::Ice::Context*) = 0;

    virtual void getTuningVals(::ScorbotIce::JointType, ::Ice::Int&, ::Ice::Int&, ::Ice::Float&, const ::Ice::Context*) = 0;

    virtual void setGravityParameters(::Ice::Int, ::Ice::Int, ::Ice::Float, const ::Ice::Context*) = 0;

    virtual void getGravityParameters(::Ice::Int&, ::Ice::Int&, ::Ice::Float&, const ::Ice::Context*) = 0;
};

}

}

namespace IceDelegateM
{

namespace ScorbotIce
{

class Scorbot : virtual public ::IceDelegate::ScorbotIce::Scorbot,
                virtual public ::IceDelegateM::Ice::Object
{
public:

    virtual void setJoint(::ScorbotIce::JointType, ::Ice::Int, ::Ice::Int, const ::Ice::Context*);

    virtual void setJoints(const ::ScorbotIce::encoderValsType&, ::Ice::Int, const ::Ice::Context*);

    virtual ::Ice::Int getEncoder(::ScorbotIce::JointType, const ::Ice::Context*);

    virtual ::ScorbotIce::encoderValsType getEncoders(const ::Ice::Context*);

    virtual void setEnabled(bool, const ::Ice::Context*);

    virtual void resetEncoders(const ::Ice::Context*);

    virtual ::Ice::Float getPWM(::ScorbotIce::JointType, const ::Ice::Context*);

    virtual ::ScorbotIce::pwmValsType getPWMs(const ::Ice::Context*);

    virtual void setControlParams(::ScorbotIce::JointType, ::Ice::Float, ::Ice::Float, ::Ice::Float, ::Ice::Float, ::Ice::Float, ::Ice::Float, const ::Ice::Context*);

    virtual void getPIDVals(::ScorbotIce::JointType, ::Ice::Float&, ::Ice::Float&, ::Ice::Float&, ::Ice::Float&, ::Ice::Float&, ::Ice::Float&, const ::Ice::Context*);

    virtual void getTuningVals(::ScorbotIce::JointType, ::Ice::Int&, ::Ice::Int&, ::Ice::Float&, const ::Ice::Context*);

    virtual void setGravityParameters(::Ice::Int, ::Ice::Int, ::Ice::Float, const ::Ice::Context*);

    virtual void getGravityParameters(::Ice::Int&, ::Ice::Int&, ::Ice::Float&, const ::Ice::Context*);
};

}

}

namespace IceDelegateD
{

namespace ScorbotIce
{

class Scorbot : virtual public ::IceDelegate::ScorbotIce::Scorbot,
                virtual public ::IceDelegateD::Ice::Object
{
public:

    virtual void setJoint(::ScorbotIce::JointType, ::Ice::Int, ::Ice::Int, const ::Ice::Context*);

    virtual void setJoints(const ::ScorbotIce::encoderValsType&, ::Ice::Int, const ::Ice::Context*);

    virtual ::Ice::Int getEncoder(::ScorbotIce::JointType, const ::Ice::Context*);

    virtual ::ScorbotIce::encoderValsType getEncoders(const ::Ice::Context*);

    virtual void setEnabled(bool, const ::Ice::Context*);

    virtual void resetEncoders(const ::Ice::Context*);

    virtual ::Ice::Float getPWM(::ScorbotIce::JointType, const ::Ice::Context*);

    virtual ::ScorbotIce::pwmValsType getPWMs(const ::Ice::Context*);

    virtual void setControlParams(::ScorbotIce::JointType, ::Ice::Float, ::Ice::Float, ::Ice::Float, ::Ice::Float, ::Ice::Float, ::Ice::Float, const ::Ice::Context*);

    virtual void getPIDVals(::ScorbotIce::JointType, ::Ice::Float&, ::Ice::Float&, ::Ice::Float&, ::Ice::Float&, ::Ice::Float&, ::Ice::Float&, const ::Ice::Context*);

    virtual void getTuningVals(::ScorbotIce::JointType, ::Ice::Int&, ::Ice::Int&, ::Ice::Float&, const ::Ice::Context*);

    virtual void setGravityParameters(::Ice::Int, ::Ice::Int, ::Ice::Float, const ::Ice::Context*);

    virtual void getGravityParameters(::Ice::Int&, ::Ice::Int&, ::Ice::Float&, const ::Ice::Context*);
};

}

}

namespace ScorbotIce
{

class Scorbot : virtual public ::Ice::Object
{
public:

    typedef ScorbotPrx ProxyType;
    typedef ScorbotPtr PointerType;

    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual void setJoint(::ScorbotIce::JointType, ::Ice::Int, ::Ice::Int, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___setJoint(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void setJoints(const ::ScorbotIce::encoderValsType&, ::Ice::Int, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___setJoints(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::Ice::Int getEncoder(::ScorbotIce::JointType, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___getEncoder(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::ScorbotIce::encoderValsType getEncoders(const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___getEncoders(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void setEnabled(bool, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___setEnabled(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void resetEncoders(const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___resetEncoders(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::Ice::Float getPWM(::ScorbotIce::JointType, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___getPWM(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::ScorbotIce::pwmValsType getPWMs(const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___getPWMs(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void setControlParams(::ScorbotIce::JointType, ::Ice::Float, ::Ice::Float, ::Ice::Float, ::Ice::Float, ::Ice::Float, ::Ice::Float, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___setControlParams(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void getPIDVals(::ScorbotIce::JointType, ::Ice::Float&, ::Ice::Float&, ::Ice::Float&, ::Ice::Float&, ::Ice::Float&, ::Ice::Float&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___getPIDVals(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void getTuningVals(::ScorbotIce::JointType, ::Ice::Int&, ::Ice::Int&, ::Ice::Float&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___getTuningVals(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void setGravityParameters(::Ice::Int, ::Ice::Int, ::Ice::Float, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___setGravityParameters(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void getGravityParameters(::Ice::Int&, ::Ice::Int&, ::Ice::Float&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::Ice::DispatchStatus ___getGravityParameters(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::Ice::DispatchStatus __dispatch(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);
};

}

#endif
