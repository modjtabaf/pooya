/*
Copyright 2024 Mojtaba (Moji) Fathi

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
documentation files (the “Software”), to deal in the Software without restriction, including without limitation the
rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit
persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
Software.

 THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __POOYA_SIGNAL_SIGNAL_HPP__
#define __POOYA_SIGNAL_SIGNAL_HPP__

#include <cstdint>

#include "src/helper/util.hpp"
#include "src/shared/named_object.hpp"
#include "trait.hpp"

namespace pooya
{

class SignalImpl : public std::enable_shared_from_this<SignalImpl>, public NamedObject
{
protected:
    struct Protected
    {
    };

public:
    const uint32_t _type;

    enum SignalTypes
    {
        BusType    = 0x0001,
        ValueType  = 0x0002,
        FloatType  = 0x0004,
        ScalarType = 0x0008,
        IntType    = 0x0010,
        BoolType   = 0x0020,
        ArrayType  = 0x0040,
        AllTypes   = -1,
    };

protected:
    SignalImpl(const ValidName& name, uint32_t type) : NamedObject(name), _type(type) {}

public:
    bool is_value() const { return _type & ValueType; }
    bool is_float() const { return _type & FloatType; }
    bool is_scalar() const { return _type & ScalarType; }
    bool is_int() const { return _type & IntType; }
    bool is_bool() const { return _type & BoolType; }
    bool is_array() const { return _type & ArrayType; }
    bool is_bus() const { return _type & BusType; }

    ValueSignalImpl& as_value();
    FloatSignalImpl& as_float();
    ScalarSignalImpl& as_scalar();
    IntSignalImpl& as_int();
    BoolSignalImpl& as_bool();
    ArraySignalImpl& as_array();
    BusImpl& as_bus();

    const ValueSignalImpl& as_value() const;
    const FloatSignalImpl& as_float() const;
    const ScalarSignalImpl& as_scalar() const;
    const IntSignalImpl& as_int() const;
    const BoolSignalImpl& as_bool() const;
    const ArraySignalImpl& as_array() const;
    const BusImpl& as_bus() const;
};

template<typename T>
class Signal
{
protected:
    typename Types<T>::SignalImplPtr _sid;

    static void fail_if_invalid_signal(const SignalImplPtr& sid)
    {
        if (!sid) helper::pooya_throw_exception(__FILE__, __LINE__, "invalid signal id!");
    }

public:
    explicit Signal(const typename Types<T>::SignalImplPtr& sid) { reset(sid); }

    Signal<T>& operator=(const Signal<T>&) = delete;

    void reset(const typename Types<T>::SignalImplPtr& sid)
    {
        fail_if_invalid_signal(sid);
        _sid = sid;
    }

    const typename Types<T>::SignalImplPtr operator->() const { return _sid; }
    typename Types<T>::SignalImplPtr operator->() { return _sid; }

    operator const typename Types<T>::SignalImplPtr &() const { return _sid; }
    operator SignalImplPtr() const { return _sid->shared_from_this(); }
};

} // namespace pooya

#endif // __POOYA_SIGNAL_SIGNAL_HPP__
