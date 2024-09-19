/*
Copyright 2024 Mojtaba (Moji) Fathi

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”),
to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __POOYA_SIGNAL_SCALAR_SIGNAL_HPP__
#define __POOYA_SIGNAL_SCALAR_SIGNAL_HPP__

#include <functional>
#include <memory>

#include "src/helper/trace.hpp"
#include "src/helper/verify.hpp"
#include "float_signal.hpp"

#define pooya_verify_scalar_signal(sig) \
    pooya_verify_valid_signal(sig); \
    pooya_verify((sig)->is_scalar(), (sig)->_full_name + ": scalar signal expected!");

namespace pooya
{

// #if defined(POOYA_USE_SMART_PTRS)

// inline ScalarSignalId SignalInfo::as_scalar() {return _scalar ? std::static_pointer_cast<ScalarSignalInfo>(shared_from_this()) : ScalarSignalId();}
// inline ReadOnlyScalarSignalId SignalInfo::as_scalar() const {return _scalar ? std::static_pointer_cast<const ScalarSignalInfo>(shared_from_this()) : ReadOnlyScalarSignalId();}

// #else // defined(POOYA_USE_SMART_PTS)

// inline ScalarSignalId SignalInfo::as_scalar() {return _scalar ? static_cast<ScalarSignalId>(this) : nullptr;}
// inline ReadOnlyScalarSignalId SignalInfo::as_scalar() const {return _scalar ? static_cast<ReadOnlyScalarSignalId>(this) : nullptr;}

// #endif // defined(POOYA_USE_SMART_PTS)

class ScalarSignal : public FloatSignal
{
    friend class Model;
protected:
    struct ScalarSignalInfo : public FloatSignal::FloatSignalInfo
    {
        friend class Model;

    // protected:
        std::optional<std::reference_wrapper<double>> _scalar_value;
        std::optional<std::reference_wrapper<double>> _deriv_scalar_value;    // only valid if _is_deriv is true, nullptr otherwise

    // public:
    //     ScalarSignalInfo(Protected, const std::string& full_name)
    //         : FloatSignalInfo(full_name)
    //     {
    //         // _scalar = true;
    //     }

        static std::shared_ptr<ScalarSignalInfo> create_new(const std::string& full_name)
        {
            auto ret = std::make_shared<ScalarSignalInfo>();
            if (!ret) {helper::pooya_throw_exception(__FILE__, __LINE__, "Failed to create a new ScalarSignalInfo!");}
            ret->_full_name = full_name;
            return ret;
        } 

        // double get() const
        // {
        //     pooya_trace0;
        //     pooya_verify(_scalar_value, _full_name + ": attempting to retrieve the value of an uninitialized scalar signal!");
        //     pooya_verify(_assigned, _full_name + ": attempting to access an unassigned value!");
        //     return _scalar_value.value().get();
        // }

        // void set(double value)
        // {
        //     pooya_trace("value: " + std::to_string(value));
        //     pooya_verify(_scalar_value, _full_name + ": attempting to assign the value of an uninitialized scalar signal!");
        //     pooya_verify(!_assigned, _full_name + ": re-assignment is prohibited!");
        //     _scalar_value.value().get() = value;
        //     if (_deriv_scalar_value)
        //     {
        //         _deriv_scalar_value.value().get() = value;
        //     }
        //     _assigned = true;
        // }

        // operator double() const {return get();}
    };

    std::shared_ptr<ScalarSignalInfo> impl_;

public:
    ScalarSignal(const std::string& given_name="") :
        FloatSignal(*ScalarSignalInfo::create_new(given_name)),
        impl_(std::static_pointer_cast<ScalarSignalInfo>(float_signal_impl_.get().shared_from_this()))
    {
        _scalar = *this;
    }
    ScalarSignal(const ScalarSignal& scalar_signal) :
        FloatSignal(scalar_signal.as_float()),
        impl_(std::static_pointer_cast<ScalarSignalInfo>(float_signal_impl_.get().shared_from_this()))
    {
        _scalar = *this;
    }

    // ScalarSignalInfo& operator->() {return *impl_;}
    // double get() const {return impl_->get();}
    // void set(double v) {impl_->set(v);}

    ScalarSignal& operator=(const ScalarSignal& scalar_signal) = default;

    double get() const
    {
        pooya_trace0;
        // pooya_verify_valid_signal(impl_);
        pooya_verify(impl_->_scalar_value.has_value(), impl_->_full_name + ": attempting to retrieve the value of an uninitialized scalar signal!");
        pooya_verify(impl_->_assigned, impl_->_full_name + ": attempting to access an unassigned value!");
        return impl_->_scalar_value->get();
    }

    void set(double value)
    {
        pooya_trace("value: " + std::to_string(value));
        pooya_verify(impl_->_scalar_value.has_value(), impl_->_full_name + ": attempting to assign the value of an uninitialized scalar signal!");
        pooya_verify(!impl_->_assigned, impl_->_full_name + ": re-assignment is prohibited!");
        impl_->_scalar_value->get() = value;
        if (impl_->_deriv_scalar_value.has_value())
        {
            impl_->_deriv_scalar_value->get() = value;
        }
        impl_->_assigned = true;
    }

    // std::shared_ptr<ScalarSignalInfo> operator->() // TEMPORARY
    // {
    //     return std::static_pointer_cast<ScalarSignalInfo>(impl_->shared_from_this());
    // }
    // std::shared_ptr<const ScalarSignalInfo> operator->() const // TEMPORARY
    // {
    //     return std::static_pointer_cast<const ScalarSignalInfo>(impl_->shared_from_this());
    // }
    // std::shared_ptr<ScalarSignalInfo> id() // TEMPORARY
    // {
    //     return std::static_pointer_cast<ScalarSignalInfo>(impl_->shared_from_this());
    // }
    // operator SignalId()
    // {
    //     return impl_->shared_from_this();
    // }
    // const std::string& name() const override {return impl_->_full_name;}
    void set_value_ref(double& ref)
    {
        impl_->_scalar_value = ref;
    }
    void set_deriv_value_ref(double& value)
    {
        static_cast<ScalarSignalInfo&>(impl_->_deriv_sig->get())._deriv_scalar_value = value;
    }
};

// using ScalarSignalRef = std::reference_wrapper<ScalarSignal>;
// using ScalarSignalConstRef = std::reference_wrapper<const ScalarSignal>;

}

#endif // __POOYA_SIGNAL_SCALAR_SIGNAL_HPP__
