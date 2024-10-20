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

#ifndef __POOYA_SIGNAL_BOOL_SIGNAL_HPP__
#define __POOYA_SIGNAL_BOOL_SIGNAL_HPP__

#include "src/helper/trace.hpp"
#include "src/helper/verify.hpp"
#include "value_signal.hpp"

#define pooya_verify_bool_signal(sig) \
    pooya_verify_valid_signal(sig); \
    pooya_verify((sig)->is_bool(), (sig)->_full_name + ": bool signal expected!");

namespace pooya
{

class BoolSignalInfo : public ValueSignalInfo
{
    friend class Model;

protected:
    std::optional<std::reference_wrapper<double>> _bool_value;

    static BoolSignalId create_new(const std::string& full_name, std::size_t index)
    {
        return std::make_shared<BoolSignalInfo>(Protected(), full_name, index);
    } 

public:
    BoolSignalInfo(Protected, const std::string& full_name, std::size_t index)
        : ValueSignalInfo(full_name, index)
    {
        _bool = true;
    }

    bool get() const
    {
        pooya_trace0;
        pooya_verify(_bool_value, _full_name + ": attempting to retrieve the value of an uninitialized bool signal!");
        pooya_verify(is_assigned(), _full_name + ": attempting to access an unassigned value!");
        return _bool_value.value().get() != 0;
    }

    void set(bool value)
    {
        pooya_trace("value: " + std::to_string(value));
        pooya_verify(_bool_value, _full_name + ": attempting to assign the value of an uninitialized bool signal!");
        pooya_verify(!is_assigned(), _full_name + ": re-assignment is prohibited!");
        _bool_value.value().get() = value;
        _assigned = true;
    }

    operator bool() const {return get();}
};

inline BoolSignalId SignalInfo::as_bool() {return _bool ? std::static_pointer_cast<BoolSignalInfo>(shared_from_this()) : BoolSignalId();}
inline ReadOnlyBoolSignalId SignalInfo::as_bool() const {return _bool ? std::static_pointer_cast<const BoolSignalInfo>(shared_from_this()) : ReadOnlyBoolSignalId();}

}

#endif // __POOYA_SIGNAL_BOOL_SIGNAL_HPP__
