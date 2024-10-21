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

#ifndef __POOYA_SIGNAL_INT_SIGNAL_HPP__
#define __POOYA_SIGNAL_INT_SIGNAL_HPP__

#include <cmath>
#include <optional>

#include "signal.hpp"
#include "src/helper/trace.hpp"
#include "src/helper/util.hpp"
#include "src/helper/verify.hpp"
#include "value_signal.hpp"

#define pooya_verify_int_signal(sig) \
    pooya_verify_valid_signal(sig); \
    pooya_verify((sig)->is_int(), (sig)->_full_name + ": int signal expected!");

namespace pooya
{

class IntSignalInfo : public ValueSignalInfo
{
    friend class Model;

protected:
    int _int_value;

    static IntSignalId create_new(const std::string& full_name, std::size_t index)
    {
        return std::make_shared<IntSignalInfo>(Protected(), full_name, index);
    } 

public:
    IntSignalInfo(Protected, const std::string& full_name, std::size_t index)
        : ValueSignalInfo(full_name, IntType, index) {}

    int get() const
    {
        pooya_trace0;
        pooya_verify(is_assigned(), _full_name + ": attempting to access an unassigned value!");
        return _int_value;
    }

    void set(int value)
    {
        pooya_trace("value: " + std::to_string(value));
        pooya_verify(!is_assigned(), _full_name + ": re-assignment is prohibited!");
        _int_value = value;
        _assigned = true;
    }

    operator int() const {return get();}
};

inline IntSignalInfo& SignalInfo::as_int()
{
    pooya_verify(_type & IntType, "Illegal attempt to dereference a non-int as an int.");
    return *static_cast<IntSignalInfo*>(this);
}

inline const IntSignalInfo& SignalInfo::as_int() const
{
    pooya_verify(_type & IntType, "Illegal attempt to dereference a non-int as an int.");
    return *static_cast<const IntSignalInfo*>(this);
}

}

#endif // __POOYA_SIGNAL_INT_SIGNAL_HPP__