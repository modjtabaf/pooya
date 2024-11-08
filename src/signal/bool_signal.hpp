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

#include <optional>

#include "src/helper/trace.hpp"
#include "src/helper/util.hpp"
#include "src/helper/verify.hpp"
#include "value_signal.hpp"

#define pooya_verify_bool_signal(sig) \
    pooya_verify_valid_signal(sig); \
    pooya_verify((sig)->is_bool(), (sig)->_full_name + ": bool signal expected!");

namespace pooya
{

class BoolSignalInfo : public ValueSignalInfo
{
protected:
    bool _bool_value;

public:
    BoolSignalInfo(Protected, const std::string& full_name)
        : ValueSignalInfo(full_name, BoolType) {}

    static BoolSignalId create_new(const std::string& full_name="")
    {
        return std::make_shared<BoolSignalInfo>(Protected(), full_name);
    } 

    bool get() const
    {
        pooya_trace0;
        pooya_verify(is_assigned(), _full_name + ": attempting to access an unassigned value!");
        return _bool_value;
    }

    void set(bool value)
    {
        pooya_trace("value: " + std::to_string(value));
        pooya_verify(!is_assigned(), _full_name + ": re-assignment is prohibited!");
        _bool_value = value;
        _assigned = true;
    }

    operator bool() const {return get();}
};

inline BoolSignalInfo& SignalInfo::as_bool()
{
    pooya_verify(_type & BoolType, "Illegal attempt to dereference a non-bool as a bool.");
    return *static_cast<BoolSignalInfo*>(this);
}

inline const BoolSignalInfo& SignalInfo::as_bool() const
{
    pooya_verify(_type & BoolType, "Illegal attempt to dereference a non-bool as a bool.");
    return *static_cast<const BoolSignalInfo*>(this);
}

class BoolSignal : public ValueSignal<BoolSignal, bool>
{
    using Base = ValueSignal<BoolSignal, bool>;

public:
    explicit BoolSignal(const std::string& full_name="") : Base(BoolSignalInfo::create_new(full_name)) {}
    BoolSignal(const BoolSignalId& sid) : Base(sid) {}

    BoolSignal& operator=(const BoolSignal&) = delete;

    void reset(const std::string& full_name="")
    {
        _sid = BoolSignalInfo::create_new(full_name);
    }

    using Signal<BoolSignal, bool>::operator=;
    using Signal<BoolSignal, bool>::reset;
};

}

#endif // __POOYA_SIGNAL_BOOL_SIGNAL_HPP__
