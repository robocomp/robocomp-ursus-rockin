// **********************************************************************
//
// Copyright (c) 2003-2013 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************
//
// Ice version 3.5.0
//
// <auto-generated>
//
// Generated from file `CropFaceImage.ice'
//
// Warning: do not edit this file.
//
// </auto-generated>
//

#include <CropFaceImage.h>
#include <Ice/LocalException.h>
#include <Ice/ObjectFactory.h>
#include <Ice/BasicStream.h>
#include <Ice/Object.h>
#include <IceUtil/Iterator.h>

#ifndef ICE_IGNORE_VERSION
#   if ICE_INT_VERSION / 100 != 305
#       error Ice version mismatch!
#   endif
#   if ICE_INT_VERSION % 100 > 50
#       error Beta header file detected
#   endif
#   if ICE_INT_VERSION % 100 < 0
#       error Ice patch level mismatch!
#   endif
#endif

namespace
{

}

namespace Ice
{
}
::IceProxy::Ice::Object* ::IceProxy::RoboCompCropFaceImage::upCast(::IceProxy::RoboCompCropFaceImage::CropFaceImage* p) { return p; }

void
::IceProxy::RoboCompCropFaceImage::__read(::IceInternal::BasicStream* __is, ::IceInternal::ProxyHandle< ::IceProxy::RoboCompCropFaceImage::CropFaceImage>& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::RoboCompCropFaceImage::CropFaceImage;
        v->__copyFrom(proxy);
    }
}

const ::std::string&
IceProxy::RoboCompCropFaceImage::CropFaceImage::ice_staticId()
{
    return ::RoboCompCropFaceImage::CropFaceImage::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::RoboCompCropFaceImage::CropFaceImage::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::RoboCompCropFaceImage::CropFaceImage);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::RoboCompCropFaceImage::CropFaceImage::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::RoboCompCropFaceImage::CropFaceImage);
}

::IceProxy::Ice::Object*
IceProxy::RoboCompCropFaceImage::CropFaceImage::__newInstance() const
{
    return new CropFaceImage;
}

::Ice::Object* RoboCompCropFaceImage::upCast(::RoboCompCropFaceImage::CropFaceImage* p) { return p; }

namespace
{
const ::std::string __RoboCompCropFaceImage__CropFaceImage_ids[2] =
{
    "::Ice::Object",
    "::RoboCompCropFaceImage::CropFaceImage"
};

}

bool
RoboCompCropFaceImage::CropFaceImage::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__RoboCompCropFaceImage__CropFaceImage_ids, __RoboCompCropFaceImage__CropFaceImage_ids + 2, _s);
}

::std::vector< ::std::string>
RoboCompCropFaceImage::CropFaceImage::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__RoboCompCropFaceImage__CropFaceImage_ids[0], &__RoboCompCropFaceImage__CropFaceImage_ids[2]);
}

const ::std::string&
RoboCompCropFaceImage::CropFaceImage::ice_id(const ::Ice::Current&) const
{
    return __RoboCompCropFaceImage__CropFaceImage_ids[1];
}

const ::std::string&
RoboCompCropFaceImage::CropFaceImage::ice_staticId()
{
    return __RoboCompCropFaceImage__CropFaceImage_ids[1];
}

void
RoboCompCropFaceImage::CropFaceImage::__writeImpl(::IceInternal::BasicStream* __os) const
{
    __os->startWriteSlice(ice_staticId(), -1, true);
    __os->endWriteSlice();
}

void
RoboCompCropFaceImage::CropFaceImage::__readImpl(::IceInternal::BasicStream* __is)
{
    __is->startReadSlice();
    __is->endReadSlice();
}

void 
RoboCompCropFaceImage::__patch(CropFaceImagePtr& handle, const ::Ice::ObjectPtr& v)
{
    handle = ::RoboCompCropFaceImage::CropFaceImagePtr::dynamicCast(v);
    if(v && !handle)
    {
        IceInternal::Ex::throwUOE(::RoboCompCropFaceImage::CropFaceImage::ice_staticId(), v);
    }
}