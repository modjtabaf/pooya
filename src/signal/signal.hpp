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
#include <string>

#include "signal_id.hpp"

namespace pooya
{

class SignalInfo : public std::enable_shared_from_this<SignalInfo>
{
    friend class Model;

protected:
    struct Protected {};

public:
    const std::string _full_name; // full name of the signal
    const std::size_t _index{0};  // the signal index
    const uint16_t _type;

    enum SignalTypes
    {
        BusType    = 0x0001,
        ValueType  = 0x0002,
        FloatType  = 0x0004,
        ScalarType = 0x0008,
        IntType    = 0x0010,
        BoolType   = 0x0020,
        ArrayType  = 0x0040,
    };

protected:
    SignalInfo(const std::string& full_name, uint16_t type, std::size_t index) : _full_name(full_name), _index(index), _type(type) {}

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

}

#endif // __POOYA_SIGNAL_SIGNAL_HPP__
