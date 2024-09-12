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

#include <string>

#include "signal_id.hpp"

namespace pooya
{

class SignalInfo
#if defined(POOYA_USE_SMART_PTRS)
    : public std::enable_shared_from_this<SignalInfo>
#endif // defined(POOYA_USE_SMART_PTRS)
{
    friend class Model;

#if defined(POOYA_USE_SMART_PTRS)
protected:
    struct Protected {};
#endif // defined(POOYA_USE_SMART_PTRS)

public:
    const std::string _full_name; // full name of the signal
    const std::size_t _index{0};  // the signal index

protected:
    bool  _value{false};
    bool  _float{false};
    bool _scalar{false};
    bool  _array{false};
    bool   _int {false};
    bool   _bool{false};
    bool    _bus{false};

    SignalInfo(const std::string& full_name, std::size_t index) : _full_name(full_name), _index(index) {}

public:
    bool is_value() const {return _value;}
    bool is_float() const {return _float;}
    bool is_scalar() const {return _scalar;}
    bool is_int() const {return _int;}
    bool is_bool() const {return _bool;}
    bool is_array() const {return _array;}
    bool is_bus() const {return _bus;}

    ValueSignalId   as_value();
    FloatSignalId   as_float();
    ScalarSignalId as_scalar();
    IntSignalId       as_int();
    BoolSignalId     as_bool();
    ArraySignalId   as_array();
    BusId             as_bus();

    ReadOnlyValueSignalId   as_value() const;
    ReadOnlyFloatSignalId   as_float() const;
    ReadOnlyScalarSignalId as_scalar() const;
    ReadOnlyIntSignalId       as_int() const;
    ReadOnlyBoolSignalId     as_bool() const;
    ReadOnlyArraySignalId   as_array() const;
    ReadOnlyBusId             as_bus() const;
};

}

#endif // __POOYA_SIGNAL_SIGNAL_HPP__
