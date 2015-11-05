// **********************************************************************
//
// Copyright (c) 2003-2007 ZeroC, Inc. All rights reserved.
//
// This copy of Ice-E is licensed to you under the terms described in the
// ICEE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice-E version 1.3.0
// Generated from file `ImageIce.ice'

#ifndef __ImageIce_h__
#define __ImageIce_h__

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

namespace ImageIceMod
{

class ImageShuttle;

}

}

namespace ImageIceMod
{

class ImageShuttle;
bool operator==(const ImageShuttle&, const ImageShuttle&);
bool operator<(const ImageShuttle&, const ImageShuttle&);

}

namespace IceInternal
{

::Ice::Object* upCast(::ImageIceMod::ImageShuttle*);
::IceProxy::Ice::Object* upCast(::IceProxy::ImageIceMod::ImageShuttle*);

}

namespace ImageIceMod
{

typedef ::IceInternal::Handle< ::ImageIceMod::ImageShuttle> ImageShuttlePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::ImageIceMod::ImageShuttle> ImageShuttlePrx;

void __read(::IceInternal::BasicStream*, ImageShuttlePrx&);
void __patch__ImageShuttlePtr(void*, ::Ice::ObjectPtr&);

}

namespace ImageIceMod
{

typedef ::std::vector< ::Ice::Byte> ByteSeq;

struct DimsIce
{
    ::Ice::Int w;
    ::Ice::Int h;

    bool operator==(const DimsIce&) const;
    bool operator<(const DimsIce&) const;
    bool operator!=(const DimsIce& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const DimsIce& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const DimsIce& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const DimsIce& __rhs) const
    {
        return !operator<(__rhs);
    }

    void __write(::IceInternal::BasicStream*) const;
    void __read(::IceInternal::BasicStream*);
};

struct SensorPose
{
    ::Ice::Int val;
    ::Ice::Float weight;
    ::Ice::Float decay;

    bool operator==(const SensorPose&) const;
    bool operator<(const SensorPose&) const;
    bool operator!=(const SensorPose& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const SensorPose& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const SensorPose& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const SensorPose& __rhs) const
    {
        return !operator<(__rhs);
    }

    void __write(::IceInternal::BasicStream*) const;
    void __read(::IceInternal::BasicStream*);
};

enum SensorType
{
    PATH,
    SALIENCY,
    PINGER,
    BARBWIRE
};

void __write(::IceInternal::BasicStream*, SensorType);
void __read(::IceInternal::BasicStream*, SensorType&);

struct SensorVote
{
    ::ImageIceMod::SensorType type;
    ::ImageIceMod::SensorPose heading;
    ::ImageIceMod::SensorPose depth;

    bool operator==(const SensorVote&) const;
    bool operator<(const SensorVote&) const;
    bool operator!=(const SensorVote& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const SensorVote& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const SensorVote& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const SensorVote& __rhs) const
    {
        return !operator<(__rhs);
    }

    void __write(::IceInternal::BasicStream*) const;
    void __read(::IceInternal::BasicStream*);
};

struct Point2DIce
{
    ::Ice::Int i;
    ::Ice::Int j;

    bool operator==(const Point2DIce&) const;
    bool operator<(const Point2DIce&) const;
    bool operator!=(const Point2DIce& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const Point2DIce& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const Point2DIce& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const Point2DIce& __rhs) const
    {
        return !operator<(__rhs);
    }

    void __write(::IceInternal::BasicStream*) const;
    void __read(::IceInternal::BasicStream*);
};

struct Point3DIce
{
    ::Ice::Float x;
    ::Ice::Float y;
    ::Ice::Float z;

    bool operator==(const Point3DIce&) const;
    bool operator<(const Point3DIce&) const;
    bool operator!=(const Point3DIce& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const Point3DIce& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const Point3DIce& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const Point3DIce& __rhs) const
    {
        return !operator<(__rhs);
    }

    void __write(::IceInternal::BasicStream*) const;
    void __read(::IceInternal::BasicStream*);
};

struct RectangleIce
{
    ::ImageIceMod::Point2DIce tl;
    ::ImageIceMod::Point2DIce br;

    bool operator==(const RectangleIce&) const;
    bool operator<(const RectangleIce&) const;
    bool operator!=(const RectangleIce& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const RectangleIce& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const RectangleIce& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const RectangleIce& __rhs) const
    {
        return !operator<(__rhs);
    }

    void __write(::IceInternal::BasicStream*) const;
    void __read(::IceInternal::BasicStream*);
};

struct QuadrilateralIce
{
    ::ImageIceMod::Point2DIce tl;
    ::ImageIceMod::Point2DIce tr;
    ::ImageIceMod::Point2DIce bl;
    ::ImageIceMod::Point2DIce br;
    ::ImageIceMod::Point2DIce center;
    ::Ice::Float ratio;
    ::Ice::Float angle;

    bool operator==(const QuadrilateralIce&) const;
    bool operator<(const QuadrilateralIce&) const;
    bool operator!=(const QuadrilateralIce& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const QuadrilateralIce& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const QuadrilateralIce& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const QuadrilateralIce& __rhs) const
    {
        return !operator<(__rhs);
    }

    void __write(::IceInternal::BasicStream*) const;
    void __read(::IceInternal::BasicStream*);
};

struct LineIce
{
    ::ImageIceMod::Point2DIce pt1;
    ::ImageIceMod::Point2DIce pt2;
    ::Ice::Float angle;

    bool operator==(const LineIce&) const;
    bool operator<(const LineIce&) const;
    bool operator!=(const LineIce& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const LineIce& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const LineIce& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const LineIce& __rhs) const
    {
        return !operator<(__rhs);
    }

    void __write(::IceInternal::BasicStream*) const;
    void __read(::IceInternal::BasicStream*);
};

struct ImageIce
{
    ::ImageIceMod::ByteSeq data;
    ::Ice::Int width;
    ::Ice::Int height;
    ::Ice::Int pixSize;

    bool operator==(const ImageIce&) const;
    bool operator<(const ImageIce&) const;
    bool operator!=(const ImageIce& __rhs) const
    {
        return !operator==(__rhs);
    }
    bool operator<=(const ImageIce& __rhs) const
    {
        return operator<(__rhs) || operator==(__rhs);
    }
    bool operator>(const ImageIce& __rhs) const
    {
        return !operator<(__rhs) && !operator==(__rhs);
    }
    bool operator>=(const ImageIce& __rhs) const
    {
        return !operator<(__rhs);
    }

    void __write(::IceInternal::BasicStream*) const;
    void __read(::IceInternal::BasicStream*);
};

typedef ::std::vector< ::ImageIceMod::ImageIce> ImageIceSeq;
void __writeImageIceSeq(::IceInternal::BasicStream*, const ::ImageIceMod::ImageIce*, const ::ImageIceMod::ImageIce*);
void __readImageIceSeq(::IceInternal::BasicStream*, ImageIceSeq&);

}

namespace ImageIceMod
{

class ImageShuttle : virtual public ::Ice::Object
{
public:

    typedef ImageShuttlePrx ProxyType;
    typedef ImageShuttlePtr PointerType;


    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual void transferImage(const ::ImageIceMod::ImageIce&, const ::Ice::Current& = ::Ice::Current()) = 0;
#ifndef ICEE_PURE_CLIENT
    ::Ice::DispatchStatus ___transferImage(::IceInternal::Incoming&, const ::Ice::Current&);
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

namespace ImageIceMod
{

class ImageShuttle : virtual public ::IceProxy::Ice::Object
{
public:

    void transferImage(const ::ImageIceMod::ImageIce& i)
    {
        transferImage(i, 0);
    }
    void transferImage(const ::ImageIceMod::ImageIce& i, const ::Ice::Context& __ctx)
    {
        transferImage(i, &__ctx);
    }

private:

    void transferImage(const ::ImageIceMod::ImageIce&, const ::Ice::Context*);

public:

    ::IceInternal::ProxyHandle<ImageShuttle> ice_context(const ::Ice::Context& __context) const
    {
        return dynamic_cast<ImageShuttle*>(::IceProxy::Ice::Object::ice_context(__context).get());
    }

    ::IceInternal::ProxyHandle<ImageShuttle> ice_secure(bool __secure) const
    {
        return dynamic_cast<ImageShuttle*>(::IceProxy::Ice::Object::ice_secure(__secure).get());
    }

#ifdef ICEE_HAS_ROUTER
    ::IceInternal::ProxyHandle<ImageShuttle> ice_router(const ::Ice::RouterPrx& __router) const
    {
        return dynamic_cast<ImageShuttle*>(::IceProxy::Ice::Object::ice_router(__router).get());
    }
#endif // ICEE_HAS_ROUTER

#ifdef ICEE_HAS_LOCATOR
    ::IceInternal::ProxyHandle<ImageShuttle> ice_locator(const ::Ice::LocatorPrx& __locator) const
    {
        return dynamic_cast<ImageShuttle*>(::IceProxy::Ice::Object::ice_locator(__locator).get());
    }

    ::IceInternal::ProxyHandle<ImageShuttle> ice_adapterId(const std::string& __id) const
    {
        return dynamic_cast<ImageShuttle*>(::IceProxy::Ice::Object::ice_adapterId(__id).get());
    }
#endif // ICEE_HAS_LOCATOR

    ::IceInternal::ProxyHandle<ImageShuttle> ice_twoway() const
    {
        return dynamic_cast<ImageShuttle*>(::IceProxy::Ice::Object::ice_twoway().get());
    }

    ::IceInternal::ProxyHandle<ImageShuttle> ice_oneway() const
    {
        return dynamic_cast<ImageShuttle*>(::IceProxy::Ice::Object::ice_oneway().get());
    }

    ::IceInternal::ProxyHandle<ImageShuttle> ice_batchOneway() const
    {
        return dynamic_cast<ImageShuttle*>(::IceProxy::Ice::Object::ice_batchOneway().get());
    }

    ::IceInternal::ProxyHandle<ImageShuttle> ice_datagram() const
    {
        return dynamic_cast<ImageShuttle*>(::IceProxy::Ice::Object::ice_datagram().get());
    }

    ::IceInternal::ProxyHandle<ImageShuttle> ice_batchDatagram() const
    {
        return dynamic_cast<ImageShuttle*>(::IceProxy::Ice::Object::ice_batchDatagram().get());
    }

    ::IceInternal::ProxyHandle<ImageShuttle> ice_timeout(int __timeout) const
    {
        return dynamic_cast<ImageShuttle*>(::IceProxy::Ice::Object::ice_timeout(__timeout).get());
    }

    static const ::std::string& ice_staticId();

private:

    virtual ::IceProxy::Ice::Object* __newInstance() const;
};

}

}

#endif
