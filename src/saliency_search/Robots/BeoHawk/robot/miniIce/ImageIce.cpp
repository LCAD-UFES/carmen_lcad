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

#include "ImageIce.h"
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

static const ::std::string __ImageIceMod__ImageShuttle__transferImage_name = "transferImage";

::Ice::Object* IceInternal::upCast(::ImageIceMod::ImageShuttle* p) { return p; }
::IceProxy::Ice::Object* IceInternal::upCast(::IceProxy::ImageIceMod::ImageShuttle* p) { return p; }

void
ImageIceMod::__read(::IceInternal::BasicStream* __is, ::ImageIceMod::ImageShuttlePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::ImageIceMod::ImageShuttle;
        v->__copyFrom(proxy);
    }
}

bool
ImageIceMod::DimsIce::operator==(const DimsIce& __rhs) const
{
    if(this == &__rhs)
    {
        return true;
    }
    if(w != __rhs.w)
    {
        return false;
    }
    if(h != __rhs.h)
    {
        return false;
    }
    return true;
}

bool
ImageIceMod::DimsIce::operator<(const DimsIce& __rhs) const
{
    if(this == &__rhs)
    {
        return false;
    }
    if(w < __rhs.w)
    {
        return true;
    }
    else if(__rhs.w < w)
    {
        return false;
    }
    if(h < __rhs.h)
    {
        return true;
    }
    else if(__rhs.h < h)
    {
        return false;
    }
    return false;
}

void
ImageIceMod::DimsIce::__write(::IceInternal::BasicStream* __os) const
{
    __os->write(w);
    __os->write(h);
}

void
ImageIceMod::DimsIce::__read(::IceInternal::BasicStream* __is)
{
    __is->read(w);
    __is->read(h);
}

bool
ImageIceMod::SensorPose::operator==(const SensorPose& __rhs) const
{
    if(this == &__rhs)
    {
        return true;
    }
    if(val != __rhs.val)
    {
        return false;
    }
    if(weight != __rhs.weight)
    {
        return false;
    }
    if(decay != __rhs.decay)
    {
        return false;
    }
    return true;
}

bool
ImageIceMod::SensorPose::operator<(const SensorPose& __rhs) const
{
    if(this == &__rhs)
    {
        return false;
    }
    if(val < __rhs.val)
    {
        return true;
    }
    else if(__rhs.val < val)
    {
        return false;
    }
    if(weight < __rhs.weight)
    {
        return true;
    }
    else if(__rhs.weight < weight)
    {
        return false;
    }
    if(decay < __rhs.decay)
    {
        return true;
    }
    else if(__rhs.decay < decay)
    {
        return false;
    }
    return false;
}

void
ImageIceMod::SensorPose::__write(::IceInternal::BasicStream* __os) const
{
    __os->write(val);
    __os->write(weight);
    __os->write(decay);
}

void
ImageIceMod::SensorPose::__read(::IceInternal::BasicStream* __is)
{
    __is->read(val);
    __is->read(weight);
    __is->read(decay);
}

void
ImageIceMod::__write(::IceInternal::BasicStream* __os, ::ImageIceMod::SensorType v)
{
    __os->write(static_cast< ::Ice::Byte>(v), 4);
}

void
ImageIceMod::__read(::IceInternal::BasicStream* __is, ::ImageIceMod::SensorType& v)
{
    ::Ice::Byte val;
    __is->read(val, 4);
    v = static_cast< ::ImageIceMod::SensorType>(val);
}

bool
ImageIceMod::SensorVote::operator==(const SensorVote& __rhs) const
{
    if(this == &__rhs)
    {
        return true;
    }
    if(type != __rhs.type)
    {
        return false;
    }
    if(heading != __rhs.heading)
    {
        return false;
    }
    if(depth != __rhs.depth)
    {
        return false;
    }
    return true;
}

bool
ImageIceMod::SensorVote::operator<(const SensorVote& __rhs) const
{
    if(this == &__rhs)
    {
        return false;
    }
    if(type < __rhs.type)
    {
        return true;
    }
    else if(__rhs.type < type)
    {
        return false;
    }
    if(heading < __rhs.heading)
    {
        return true;
    }
    else if(__rhs.heading < heading)
    {
        return false;
    }
    if(depth < __rhs.depth)
    {
        return true;
    }
    else if(__rhs.depth < depth)
    {
        return false;
    }
    return false;
}

void
ImageIceMod::SensorVote::__write(::IceInternal::BasicStream* __os) const
{
    ::ImageIceMod::__write(__os, type);
    heading.__write(__os);
    depth.__write(__os);
}

void
ImageIceMod::SensorVote::__read(::IceInternal::BasicStream* __is)
{
    ::ImageIceMod::__read(__is, type);
    heading.__read(__is);
    depth.__read(__is);
}

bool
ImageIceMod::Point2DIce::operator==(const Point2DIce& __rhs) const
{
    if(this == &__rhs)
    {
        return true;
    }
    if(i != __rhs.i)
    {
        return false;
    }
    if(j != __rhs.j)
    {
        return false;
    }
    return true;
}

bool
ImageIceMod::Point2DIce::operator<(const Point2DIce& __rhs) const
{
    if(this == &__rhs)
    {
        return false;
    }
    if(i < __rhs.i)
    {
        return true;
    }
    else if(__rhs.i < i)
    {
        return false;
    }
    if(j < __rhs.j)
    {
        return true;
    }
    else if(__rhs.j < j)
    {
        return false;
    }
    return false;
}

void
ImageIceMod::Point2DIce::__write(::IceInternal::BasicStream* __os) const
{
    __os->write(i);
    __os->write(j);
}

void
ImageIceMod::Point2DIce::__read(::IceInternal::BasicStream* __is)
{
    __is->read(i);
    __is->read(j);
}

bool
ImageIceMod::Point3DIce::operator==(const Point3DIce& __rhs) const
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
    if(z != __rhs.z)
    {
        return false;
    }
    return true;
}

bool
ImageIceMod::Point3DIce::operator<(const Point3DIce& __rhs) const
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
    if(z < __rhs.z)
    {
        return true;
    }
    else if(__rhs.z < z)
    {
        return false;
    }
    return false;
}

void
ImageIceMod::Point3DIce::__write(::IceInternal::BasicStream* __os) const
{
    __os->write(x);
    __os->write(y);
    __os->write(z);
}

void
ImageIceMod::Point3DIce::__read(::IceInternal::BasicStream* __is)
{
    __is->read(x);
    __is->read(y);
    __is->read(z);
}

bool
ImageIceMod::RectangleIce::operator==(const RectangleIce& __rhs) const
{
    if(this == &__rhs)
    {
        return true;
    }
    if(tl != __rhs.tl)
    {
        return false;
    }
    if(br != __rhs.br)
    {
        return false;
    }
    return true;
}

bool
ImageIceMod::RectangleIce::operator<(const RectangleIce& __rhs) const
{
    if(this == &__rhs)
    {
        return false;
    }
    if(tl < __rhs.tl)
    {
        return true;
    }
    else if(__rhs.tl < tl)
    {
        return false;
    }
    if(br < __rhs.br)
    {
        return true;
    }
    else if(__rhs.br < br)
    {
        return false;
    }
    return false;
}

void
ImageIceMod::RectangleIce::__write(::IceInternal::BasicStream* __os) const
{
    tl.__write(__os);
    br.__write(__os);
}

void
ImageIceMod::RectangleIce::__read(::IceInternal::BasicStream* __is)
{
    tl.__read(__is);
    br.__read(__is);
}

bool
ImageIceMod::QuadrilateralIce::operator==(const QuadrilateralIce& __rhs) const
{
    if(this == &__rhs)
    {
        return true;
    }
    if(tl != __rhs.tl)
    {
        return false;
    }
    if(tr != __rhs.tr)
    {
        return false;
    }
    if(bl != __rhs.bl)
    {
        return false;
    }
    if(br != __rhs.br)
    {
        return false;
    }
    if(center != __rhs.center)
    {
        return false;
    }
    if(ratio != __rhs.ratio)
    {
        return false;
    }
    if(angle != __rhs.angle)
    {
        return false;
    }
    return true;
}

bool
ImageIceMod::QuadrilateralIce::operator<(const QuadrilateralIce& __rhs) const
{
    if(this == &__rhs)
    {
        return false;
    }
    if(tl < __rhs.tl)
    {
        return true;
    }
    else if(__rhs.tl < tl)
    {
        return false;
    }
    if(tr < __rhs.tr)
    {
        return true;
    }
    else if(__rhs.tr < tr)
    {
        return false;
    }
    if(bl < __rhs.bl)
    {
        return true;
    }
    else if(__rhs.bl < bl)
    {
        return false;
    }
    if(br < __rhs.br)
    {
        return true;
    }
    else if(__rhs.br < br)
    {
        return false;
    }
    if(center < __rhs.center)
    {
        return true;
    }
    else if(__rhs.center < center)
    {
        return false;
    }
    if(ratio < __rhs.ratio)
    {
        return true;
    }
    else if(__rhs.ratio < ratio)
    {
        return false;
    }
    if(angle < __rhs.angle)
    {
        return true;
    }
    else if(__rhs.angle < angle)
    {
        return false;
    }
    return false;
}

void
ImageIceMod::QuadrilateralIce::__write(::IceInternal::BasicStream* __os) const
{
    tl.__write(__os);
    tr.__write(__os);
    bl.__write(__os);
    br.__write(__os);
    center.__write(__os);
    __os->write(ratio);
    __os->write(angle);
}

void
ImageIceMod::QuadrilateralIce::__read(::IceInternal::BasicStream* __is)
{
    tl.__read(__is);
    tr.__read(__is);
    bl.__read(__is);
    br.__read(__is);
    center.__read(__is);
    __is->read(ratio);
    __is->read(angle);
}

bool
ImageIceMod::LineIce::operator==(const LineIce& __rhs) const
{
    if(this == &__rhs)
    {
        return true;
    }
    if(pt1 != __rhs.pt1)
    {
        return false;
    }
    if(pt2 != __rhs.pt2)
    {
        return false;
    }
    if(angle != __rhs.angle)
    {
        return false;
    }
    return true;
}

bool
ImageIceMod::LineIce::operator<(const LineIce& __rhs) const
{
    if(this == &__rhs)
    {
        return false;
    }
    if(pt1 < __rhs.pt1)
    {
        return true;
    }
    else if(__rhs.pt1 < pt1)
    {
        return false;
    }
    if(pt2 < __rhs.pt2)
    {
        return true;
    }
    else if(__rhs.pt2 < pt2)
    {
        return false;
    }
    if(angle < __rhs.angle)
    {
        return true;
    }
    else if(__rhs.angle < angle)
    {
        return false;
    }
    return false;
}

void
ImageIceMod::LineIce::__write(::IceInternal::BasicStream* __os) const
{
    pt1.__write(__os);
    pt2.__write(__os);
    __os->write(angle);
}

void
ImageIceMod::LineIce::__read(::IceInternal::BasicStream* __is)
{
    pt1.__read(__is);
    pt2.__read(__is);
    __is->read(angle);
}

bool
ImageIceMod::ImageIce::operator==(const ImageIce& __rhs) const
{
    if(this == &__rhs)
    {
        return true;
    }
    if(data != __rhs.data)
    {
        return false;
    }
    if(width != __rhs.width)
    {
        return false;
    }
    if(height != __rhs.height)
    {
        return false;
    }
    if(pixSize != __rhs.pixSize)
    {
        return false;
    }
    return true;
}

bool
ImageIceMod::ImageIce::operator<(const ImageIce& __rhs) const
{
    if(this == &__rhs)
    {
        return false;
    }
    if(data < __rhs.data)
    {
        return true;
    }
    else if(__rhs.data < data)
    {
        return false;
    }
    if(width < __rhs.width)
    {
        return true;
    }
    else if(__rhs.width < width)
    {
        return false;
    }
    if(height < __rhs.height)
    {
        return true;
    }
    else if(__rhs.height < height)
    {
        return false;
    }
    if(pixSize < __rhs.pixSize)
    {
        return true;
    }
    else if(__rhs.pixSize < pixSize)
    {
        return false;
    }
    return false;
}

void
ImageIceMod::ImageIce::__write(::IceInternal::BasicStream* __os) const
{
    if(data.size() == 0)
    {
        __os->writeSize(0);
    }
    else
    {
        __os->write(&data[0], &data[0] + data.size());
    }
    __os->write(width);
    __os->write(height);
    __os->write(pixSize);
}

void
ImageIceMod::ImageIce::__read(::IceInternal::BasicStream* __is)
{
    ::std::pair<const ::Ice::Byte*, const ::Ice::Byte*> ___data;
    __is->read(___data);
    ::std::vector< ::Ice::Byte>(___data.first, ___data.second).swap(data);
    __is->read(width);
    __is->read(height);
    __is->read(pixSize);
}

void
ImageIceMod::__writeImageIceSeq(::IceInternal::BasicStream* __os, const ::ImageIceMod::ImageIce* begin, const ::ImageIceMod::ImageIce* end)
{
    ::Ice::Int size = static_cast< ::Ice::Int>(end - begin);
    __os->writeSize(size);
    for(int i = 0; i < size; ++i)
    {
        begin[i].__write(__os);
    }
}

void
ImageIceMod::__readImageIceSeq(::IceInternal::BasicStream* __is, ::ImageIceMod::ImageIceSeq& v)
{
    ::Ice::Int sz;
    __is->readSize(sz);
    __is->startSeq(sz, 13);
    v.resize(sz);
    for(int i = 0; i < sz; ++i)
    {
        v[i].__read(__is);
        __is->checkSeq();
        __is->endElement();
    }
    __is->endSeq(sz);
}

static const ::std::string __ImageIceMod__ImageShuttle_ids[2] =
{
    "::Ice::Object",
    "::ImageIceMod::ImageShuttle"
};

bool
ImageIceMod::ImageShuttle::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__ImageIceMod__ImageShuttle_ids, __ImageIceMod__ImageShuttle_ids + 2, _s);
}

::std::vector< ::std::string>
ImageIceMod::ImageShuttle::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__ImageIceMod__ImageShuttle_ids[0], &__ImageIceMod__ImageShuttle_ids[2]);
}

const ::std::string&
ImageIceMod::ImageShuttle::ice_id(const ::Ice::Current&) const
{
    return __ImageIceMod__ImageShuttle_ids[1];
}

const ::std::string&
ImageIceMod::ImageShuttle::ice_staticId()
{
    return __ImageIceMod__ImageShuttle_ids[1];
}

#ifndef ICEE_PURE_CLIENT
::Ice::DispatchStatus
ImageIceMod::ImageShuttle::___transferImage(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    ::ImageIceMod::ImageIce i;
    i.__read(__is);
    transferImage(i, __current);
    return ::Ice::DispatchOK;
}
#endif // ICEE_PURE_CLIENT

#ifndef ICEE_PURE_CLIENT
static ::std::string __ImageIceMod__ImageShuttle_all[] =
{
    "ice_id",
    "ice_ids",
    "ice_isA",
    "ice_ping",
    "transferImage"
};

::Ice::DispatchStatus
ImageIceMod::ImageShuttle::__dispatch(::IceInternal::Incoming& in, const ::Ice::Current& current)
{
    ::std::pair< ::std::string*, ::std::string*> r = ::std::equal_range(__ImageIceMod__ImageShuttle_all, __ImageIceMod__ImageShuttle_all + 5, current.operation);
    if(r.first == r.second)
    {
        throw Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
    }

    switch(r.first - __ImageIceMod__ImageShuttle_all)
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
            return ___transferImage(in, current);
        }
    }

    assert(false);
    throw Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
}
#endif // ICEE_PURE_CLIENT

void
ImageIceMod::ImageShuttle::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->endWriteSlice();
    ::Ice::Object::__write(__os);
}

void
ImageIceMod::ImageShuttle::__read(::IceInternal::BasicStream* __is, bool __rid)
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
ImageIceMod::operator==(const ::ImageIceMod::ImageShuttle& l, const ::ImageIceMod::ImageShuttle& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
ImageIceMod::operator<(const ::ImageIceMod::ImageShuttle& l, const ::ImageIceMod::ImageShuttle& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

void
ImageIceMod::__patch__ImageShuttlePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::ImageIceMod::ImageShuttlePtr* p = static_cast< ::ImageIceMod::ImageShuttlePtr*>(__addr);
    assert(p);
    *p = ::ImageIceMod::ImageShuttlePtr::dynamicCast(v);
    if(v && !*p)
    {
        IceInternal::Ex::throwUOE(::ImageIceMod::ImageShuttle::ice_staticId(), v->ice_id());
    }
}

void
IceProxy::ImageIceMod::ImageShuttle::transferImage(const ::ImageIceMod::ImageIce& i, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::RequestHandlerPtr __handler;
        try
        {
            __handler = __getRequestHandler();
            ::IceInternal::Outgoing __outS(__handler.get(), _reference.get(), __ImageIceMod__ImageShuttle__transferImage_name, ::Ice::Normal, __ctx);
            try
            {
                ::IceInternal::BasicStream* __os = __outS.os();
                i.__write(__os);
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
IceProxy::ImageIceMod::ImageShuttle::ice_staticId()
{
    return __ImageIceMod__ImageShuttle_ids[1];
}

::IceProxy::Ice::Object*
IceProxy::ImageIceMod::ImageShuttle::__newInstance() const
{
    return new ImageShuttle;
}
