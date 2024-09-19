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

#ifndef __POOYA_SIGNAL_SIGNAL_HPP__
#define __POOYA_SIGNAL_SIGNAL_HPP__

#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "signal_id.hpp"

namespace pooya
{

class ValueSignal;
class FloatSignal;
class ScalarSignal;
class Bus;

class Signal
{
protected:
    // std::shared_ptr<ScalarSignalInfo> impl_;

    struct SignalInfo : public std::enable_shared_from_this<SignalInfo>
    {
    public:
        std::string _full_name; // full name of the signal
        // const std::size_t _index{0};  // the signal index (REMOVE)

        // SignalInfo(const std::string& full_name) : _full_name(full_name) {}
    };

    std::reference_wrapper<SignalInfo> signal_impl_;

    std::optional<std::reference_wrapper<ValueSignal>> _value;
    std::optional<std::reference_wrapper<FloatSignal>> _float;
    std::optional<std::reference_wrapper<ScalarSignal>> _scalar;
    std::optional<std::reference_wrapper<Bus>> _bus;

    explicit Signal(SignalInfo& signal_impl) : signal_impl_(signal_impl) {}

public:
    // ScalarSignal(const std::string& given_name="")
    // {
    //     impl_ = ScalarSignalInfo::create_new(given_name, 0);
    // }
    // ScalarSignal(const ScalarSignal& scalar_signal)
    // {
    //     impl_ = std::static_pointer_cast<ScalarSignalInfo>(scalar_signal.impl_->shared_from_this());
    // }
    // Signal() = default;

    bool is_value() const
    {
        return _value.has_value();
    }
    bool is_float() const
    {
        return _float.has_value();
    }
    bool is_scalar() const
    {
        return _scalar.has_value();
    }
    // IntSignalId       is_int();
    // BoolSignalId     is_bool();
    // ArraySignalId   is_array();
    bool is_bus() const
    {
        return _bus.has_value();
    }

    ValueSignal& as_value() {return _value.value();}
    FloatSignal& as_float() {return _float.value();}
    ScalarSignal& as_scalar() {return _scalar.value();}
    // IntSignalId       as_int();
    // BoolSignalId     as_bool();
    // ArraySignalId   as_array();
    Bus& as_bus() {return _bus.value();}

    const ValueSignal& as_value() const {return _value.value();}
    const FloatSignal& as_float() const {return _float.value();}
    const ScalarSignal& as_scalar() const {return _scalar.value();}
    // IntSignalId       as_int();
    // BoolSignalId     as_bool();
    // ArraySignalId   as_array();
    const Bus& as_bus() const {return _bus.value();}

    const std::string& name() const {return signal_impl_.get()._full_name;}
};

}

#endif // __POOYA_SIGNAL_SIGNAL_HPP__
