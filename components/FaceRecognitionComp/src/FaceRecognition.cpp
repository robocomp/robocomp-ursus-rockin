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
// Generated from file `FaceRecognition.ice'
//
// Warning: do not edit this file.
//
// </auto-generated>
//

#include <FaceRecognition.h>
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

const ::std::string __RoboCompFaceRecognition__FaceRecognition__getClass_name = "getClass";

}

namespace Ice
{
}
::IceProxy::Ice::Object* ::IceProxy::RoboCompFaceRecognition::upCast(::IceProxy::RoboCompFaceRecognition::FaceRecognition* p) { return p; }

void
::IceProxy::RoboCompFaceRecognition::__read(::IceInternal::BasicStream* __is, ::IceInternal::ProxyHandle< ::IceProxy::RoboCompFaceRecognition::FaceRecognition>& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::RoboCompFaceRecognition::FaceRecognition;
        v->__copyFrom(proxy);
    }
}

::RoboCompFaceRecognition::vectorClasses
IceProxy::RoboCompFaceRecognition::FaceRecognition::getClass(const ::Ice::Context* __ctx)
{
    ::IceInternal::InvocationObserver __observer(this, __RoboCompFaceRecognition__FaceRecognition__getClass_name, __ctx);
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __checkTwowayOnly(__RoboCompFaceRecognition__FaceRecognition__getClass_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::RoboCompFaceRecognition::FaceRecognition* __del = dynamic_cast< ::IceDelegate::RoboCompFaceRecognition::FaceRecognition*>(__delBase.get());
            return __del->getClass(__ctx, __observer);
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, __observer);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, true, __cnt, __observer);
        }
    }
}

::Ice::AsyncResultPtr
IceProxy::RoboCompFaceRecognition::FaceRecognition::begin_getClass(const ::Ice::Context* __ctx, const ::IceInternal::CallbackBasePtr& __del, const ::Ice::LocalObjectPtr& __cookie)
{
    __checkAsyncTwowayOnly(__RoboCompFaceRecognition__FaceRecognition__getClass_name);
    ::IceInternal::OutgoingAsyncPtr __result = new ::IceInternal::OutgoingAsync(this, __RoboCompFaceRecognition__FaceRecognition__getClass_name, __del, __cookie);
    try
    {
        __result->__prepare(__RoboCompFaceRecognition__FaceRecognition__getClass_name, ::Ice::Normal, __ctx);
        __result->__writeEmptyParams();
        __result->__send(true);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __result->__exceptionAsync(__ex);
    }
    return __result;
}

::RoboCompFaceRecognition::vectorClasses
IceProxy::RoboCompFaceRecognition::FaceRecognition::end_getClass(const ::Ice::AsyncResultPtr& __result)
{
    ::Ice::AsyncResult::__check(__result, this, __RoboCompFaceRecognition__FaceRecognition__getClass_name);
    ::RoboCompFaceRecognition::vectorClasses __ret;
    bool __ok = __result->__wait();
    try
    {
        if(!__ok)
        {
            try
            {
                __result->__throwUserException();
            }
            catch(const ::Ice::UserException& __ex)
            {
                throw ::Ice::UnknownUserException(__FILE__, __LINE__, __ex.ice_name());
            }
        }
        ::IceInternal::BasicStream* __is = __result->__startReadParams();
        __is->read(__ret);
        __result->__endReadParams();
        return __ret;
    }
    catch(const ::Ice::LocalException& ex)
    {
        __result->__getObserver().failed(ex.ice_name());
        throw;
    }
}

const ::std::string&
IceProxy::RoboCompFaceRecognition::FaceRecognition::ice_staticId()
{
    return ::RoboCompFaceRecognition::FaceRecognition::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::RoboCompFaceRecognition::FaceRecognition::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::RoboCompFaceRecognition::FaceRecognition);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::RoboCompFaceRecognition::FaceRecognition::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::RoboCompFaceRecognition::FaceRecognition);
}

::IceProxy::Ice::Object*
IceProxy::RoboCompFaceRecognition::FaceRecognition::__newInstance() const
{
    return new FaceRecognition;
}

::RoboCompFaceRecognition::vectorClasses
IceDelegateM::RoboCompFaceRecognition::FaceRecognition::getClass(const ::Ice::Context* __context, ::IceInternal::InvocationObserver& __observer)
{
    ::IceInternal::Outgoing __og(__handler.get(), __RoboCompFaceRecognition__FaceRecognition__getClass_name, ::Ice::Normal, __context, __observer);
    __og.writeEmptyParams();
    bool __ok = __og.invoke();
    ::RoboCompFaceRecognition::vectorClasses __ret;
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
        ::IceInternal::BasicStream* __is = __og.startReadParams();
        __is->read(__ret);
        __og.endReadParams();
        return __ret;
    }
    catch(const ::Ice::LocalException& __ex)
    {
        throw ::IceInternal::LocalExceptionWrapper(__ex, false);
    }
}

::RoboCompFaceRecognition::vectorClasses
IceDelegateD::RoboCompFaceRecognition::FaceRecognition::getClass(const ::Ice::Context* __context, ::IceInternal::InvocationObserver&)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(::RoboCompFaceRecognition::vectorClasses& __result, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _result(__result)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::RoboCompFaceRecognition::FaceRecognition* servant = dynamic_cast< ::RoboCompFaceRecognition::FaceRecognition*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            _result = servant->getClass(_current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        ::RoboCompFaceRecognition::vectorClasses& _result;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __RoboCompFaceRecognition__FaceRecognition__getClass_name, ::Ice::Normal, __context);
    ::RoboCompFaceRecognition::vectorClasses __result;
    try
    {
        _DirectI __direct(__result, __current);
        try
        {
            __direct.getServant()->__collocDispatch(__direct);
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

::Ice::Object* RoboCompFaceRecognition::upCast(::RoboCompFaceRecognition::FaceRecognition* p) { return p; }

namespace
{
const ::std::string __RoboCompFaceRecognition__FaceRecognition_ids[2] =
{
    "::Ice::Object",
    "::RoboCompFaceRecognition::FaceRecognition"
};

}

bool
RoboCompFaceRecognition::FaceRecognition::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__RoboCompFaceRecognition__FaceRecognition_ids, __RoboCompFaceRecognition__FaceRecognition_ids + 2, _s);
}

::std::vector< ::std::string>
RoboCompFaceRecognition::FaceRecognition::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__RoboCompFaceRecognition__FaceRecognition_ids[0], &__RoboCompFaceRecognition__FaceRecognition_ids[2]);
}

const ::std::string&
RoboCompFaceRecognition::FaceRecognition::ice_id(const ::Ice::Current&) const
{
    return __RoboCompFaceRecognition__FaceRecognition_ids[1];
}

const ::std::string&
RoboCompFaceRecognition::FaceRecognition::ice_staticId()
{
    return __RoboCompFaceRecognition__FaceRecognition_ids[1];
}

::Ice::DispatchStatus
RoboCompFaceRecognition::FaceRecognition::___getClass(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    __inS.readEmptyParams();
    ::RoboCompFaceRecognition::vectorClasses __ret = getClass(__current);
    ::IceInternal::BasicStream* __os = __inS.__startWriteParams(::Ice::DefaultFormat);
    __os->write(__ret);
    __inS.__endWriteParams(true);
    return ::Ice::DispatchOK;
}

namespace
{
const ::std::string __RoboCompFaceRecognition__FaceRecognition_all[] =
{
    "getClass",
    "ice_id",
    "ice_ids",
    "ice_isA",
    "ice_ping"
};

}

::Ice::DispatchStatus
RoboCompFaceRecognition::FaceRecognition::__dispatch(::IceInternal::Incoming& in, const ::Ice::Current& current)
{
    ::std::pair< const ::std::string*, const ::std::string*> r = ::std::equal_range(__RoboCompFaceRecognition__FaceRecognition_all, __RoboCompFaceRecognition__FaceRecognition_all + 5, current.operation);
    if(r.first == r.second)
    {
        throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
    }

    switch(r.first - __RoboCompFaceRecognition__FaceRecognition_all)
    {
        case 0:
        {
            return ___getClass(in, current);
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
    }

    assert(false);
    throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
}

void
RoboCompFaceRecognition::FaceRecognition::__writeImpl(::IceInternal::BasicStream* __os) const
{
    __os->startWriteSlice(ice_staticId(), -1, true);
    __os->endWriteSlice();
}

void
RoboCompFaceRecognition::FaceRecognition::__readImpl(::IceInternal::BasicStream* __is)
{
    __is->startReadSlice();
    __is->endReadSlice();
}

void 
RoboCompFaceRecognition::__patch(FaceRecognitionPtr& handle, const ::Ice::ObjectPtr& v)
{
    handle = ::RoboCompFaceRecognition::FaceRecognitionPtr::dynamicCast(v);
    if(v && !handle)
    {
        IceInternal::Ex::throwUOE(::RoboCompFaceRecognition::FaceRecognition::ice_staticId(), v);
    }
}