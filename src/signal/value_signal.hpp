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

#ifndef __POOYA_SIGNAL_VALUE_SIGNAL_HPP__
#define __POOYA_SIGNAL_VALUE_SIGNAL_HPP__

#include "src/helper/trace.hpp"
#include "src/helper/util.hpp"
#include "src/helper/verify.hpp"
#include "signal.hpp"
#include <cstdint>

#define  pooya_verify_value_signal(sig) \
    pooya_verify_valid_signal(sig); \
    pooya_verify((sig)->is_value(), (sig)->name().str() + ": value signal expected!");

namespace pooya
{

class ValueSignalImpl : public SignalImpl
{
protected:
    bool _assigned{false};             // has the value been assigned?

    ValueSignalImpl(const ValidName& name, uint32_t type) :
        SignalImpl(name, type | ValueType) {}

public:
    double get_as_scalar() const;
    void set_as_scalar(double value);
    void clear() {_assigned = false;}

    bool is_assigned() const {return _assigned;}
};

inline ValueSignalImpl& SignalImpl::as_value()
{
    pooya_verify(_type & ValueType, "Illegal attempt to dereference a non-value as a value.");
    return *static_cast<ValueSignalImpl*>(this);
}

inline const ValueSignalImpl& SignalImpl::as_value() const
{
    pooya_verify(_type & ValueType, "Illegal attempt to dereference a non-value as a value.");
    return *static_cast<const ValueSignalImpl*>(this);
}

template<typename Derived, typename T>
class ValueSignal : public Signal<Derived, T>
{
    using Base = Signal<Derived, T>;

public:
    explicit ValueSignal(const typename Types<T>::SignalImplPtr& sid) : Signal<Derived, T>(sid) {}

    operator typename Types<T>::GetValue() const {return Base::_sid->get();}
    typename Types<T>::GetValue operator*() const {return Base::_sid->get();}

    typename Types<T>::GetValue operator=(typename Types<T>::SetValue value) const
    {
        Base::_sid->set(value);
        return value;
    }

    ValueSignal& operator=(const ValueSignal&) = delete;

    operator ValueSignalImplPtr() const {return std::static_pointer_cast<ValueSignalImpl>(static_cast<const Derived*>(this)->id()->shared_from_this());}
};

}

#endif // __POOYA_SIGNAL_VALUE_SIGNAL_HPP__
