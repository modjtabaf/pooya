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

#include <cstdint>

#include "src/shared/named_object.hpp"
#include "src/helper/util.hpp"
#include "trait.hpp"

namespace pooya
{

class SignalInfo : public std::enable_shared_from_this<SignalInfo>, public NamedObject
{
protected:
    struct Protected {};

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
    SignalInfo(const ValidName& name, uint32_t type) : NamedObject(name), _type(type) {}

public:
    bool is_value() const {return _type & ValueType;}
    bool is_float() const {return _type & FloatType;}
    bool is_scalar() const {return _type & ScalarType;}
    bool is_int() const {return _type & IntType;}
    bool is_bool() const {return _type & BoolType;}
    bool is_array() const {return _type & ArrayType;}
    bool is_bus() const {return _type & BusType;}

    ValueSignalInfo&   as_value();
    FloatSignalInfo&   as_float();
    ScalarSignalInfo& as_scalar();
    IntSignalInfo&       as_int();
    BoolSignalInfo&     as_bool();
    ArraySignalInfo&   as_array();
    BusInfo&             as_bus();

    const ValueSignalInfo&   as_value() const;
    const FloatSignalInfo&   as_float() const;
    const ScalarSignalInfo& as_scalar() const;
    const IntSignalInfo&       as_int() const;
    const BoolSignalInfo&     as_bool() const;
    const ArraySignalInfo&   as_array() const;
    const BusInfo&             as_bus() const;
};

template<typename Derived, typename T>
class Signal
{
protected:
    typename Types<T>::SignalId _sid;

    static void fail_if_invalid_signal(const SignalId& sid)
    {
        if (!sid)
            helper::pooya_throw_exception(__FILE__, __LINE__, "invalid signal id!");
    }

public:
    explicit Signal(const typename Types<T>::SignalId& sid)
    {
        reset(sid);
    }

    Signal<Derived, T>& operator=(const Signal<Derived, T>&) = delete;

    void reset(const typename Types<T>::SignalId& sid)
    {
        fail_if_invalid_signal(sid);
        _sid = sid;
    }

    const typename Types<T>::SignalId operator->() const {return _sid;}
    typename Types<T>::SignalId operator->() {return _sid;}

    const typename Types<T>::SignalId& id() const {return _sid;}
    typename Types<T>::SignalId& id() {return _sid;}

    operator const typename Types<T>::SignalId&() const {return _sid;}
    operator SignalId() const {return _sid->shared_from_this();}
};

}

#endif // __POOYA_SIGNAL_SIGNAL_HPP__
