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

#ifndef __POOYA_SIGNAL_FLOAT_SIGNAL_HPP__
#define __POOYA_SIGNAL_FLOAT_SIGNAL_HPP__

#include "signal_id.hpp"
#include "value_signal.hpp"
#include <functional>
#include <optional>

namespace pooya
{

// class FloatSignal;

// #if defined(POOYA_USE_SMART_PTRS)

// inline FloatSignalId SignalInfo::as_float() {return _float ? std::static_pointer_cast<FloatSignalInfo>(shared_from_this()) : FloatSignalId();}
// inline ReadOnlyFloatSignalId SignalInfo::as_float() const {return _float ? std::static_pointer_cast<const FloatSignalInfo>(shared_from_this()) : ReadOnlyFloatSignalId();}

// #else // defined(POOYA_USE_SMART_PTS)

// inline FloatSignalId SignalInfo::as_float() {return _float ? static_cast<FloatSignalId>(this) : nullptr;}
// inline ReadOnlyFloatSignalId SignalInfo::as_float() const {return _float ? static_cast<ReadOnlyFloatSignalId>(this) : nullptr;}

// #endif // defined(POOYA_USE_SMART_PTS)

class FloatSignal : public ValueSignal
{
    friend class Model;
protected:
    struct FloatSignalInfo : public ValueSignal::ValueSignalInfo
    {
        friend class Model;

    // protected:
        std::optional<std::reference_wrapper<FloatSignalInfo>> _deriv_sig; // the derivative signal if this is a state variable, nullptr otherwise
        bool _is_deriv{false};             // is this the derivative of another signal?

        // FloatSignalInfo(const std::string& full_name) :
        //     ValueSignalInfo(full_name)
        // {
        //     // _float = true;
        // }

    // public:
    //     bool is_state_variable() const {return static_cast<bool>(_deriv_sig);}
    };

    std::reference_wrapper<FloatSignalInfo> float_signal_impl_;

    FloatSignal(FloatSignalInfo& float_signal_impl) : ValueSignal(float_signal_impl), float_signal_impl_(float_signal_impl)
    {
        _float = *this;
    }

public:
    // ScalarSignal(const std::string& given_name="")
    // {
    //     impl_ = ScalarSignalInfo::create_new(given_name, 0);
    // }
    // ScalarSignal(const ScalarSignal& scalar_signal)
    // {
    //     impl_ = std::static_pointer_cast<ScalarSignalInfo>(scalar_signal.impl_->shared_from_this());
    // }
    // FloatSignal()
    // {
    //     _float = *this;
    // }

    // ScalarSignalInfo& operator->() {return *impl_;}
    // double get() const {return impl_->get();}
    // void set(double v) {impl_->set(v);}

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

    void register_state_variable(FloatSignal& deriv_sig)
    {
        // pooya_trace("block: " + full_name());
        // pooya_verify_signals_not_locked();
        // pooya_verify_valid_signal(sig);
        // pooya_verify_valid_signal(deriv_sig);
        pooya_verify(!is_state_variable(), name() + ": signal is already registered as a state variable!");
        pooya_verify(!deriv_sig.is_deriv(), deriv_sig.name() + ": signal is already registered as a state variable derivative!");
        pooya_verify((is_scalar() && deriv_sig.is_scalar()) /*|| (sig._array && deriv_sig._array && sig.as_array()->_size == deriv_sig.as_array()->_size)*/,
            name() + ", " + deriv_sig.name() + ": type or size mismatch!");

        float_signal_impl_.get()._deriv_sig = deriv_sig.float_signal_impl_;
        deriv_sig.float_signal_impl_.get()._is_deriv = true;
    }

    bool is_state_variable() const {return float_signal_impl_.get()._deriv_sig.has_value();}
    bool is_deriv() const {return float_signal_impl_.get()._is_deriv;}
    bool deriv_signal() const {return float_signal_impl_.get()._is_deriv;}
};

// using FloatSignalRef = std::reference_wrapper<FloatSignal>;
// using FloatSignalConstRef = std::reference_wrapper<const FloatSignal>;

}

#endif // __POOYA_SIGNAL_FLOAT_SIGNAL_HPP__
