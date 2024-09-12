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
#if defined(POOYA_USE_SMART_PTRS)
    std::optional<std::reference_wrapper<double>> _bool_value;
#else // defined(POOYA_USE_SMART_PTRS)
    double* _bool_value{nullptr};
#endif // defined(POOYA_USE_SMART_PTRS)

    static BoolSignalId create_new(const std::string& full_name, std::size_t index)
    {
#if defined(POOYA_USE_SMART_PTRS)
        return std::make_shared<BoolSignalInfo>(Protected(), full_name, index);
#else // defined(POOYA_USE_SMART_PTRS)
        return new BoolSignalInfo(full_name, index);
#endif // defined(POOYA_USE_SMART_PTRS)
    } 

public:
#if defined(POOYA_USE_SMART_PTRS)
    BoolSignalInfo(Protected, const std::string& full_name, std::size_t index)
#else // defined(POOYA_USE_SMART_PTRS)
    BoolSignalInfo(const std::string& full_name, std::size_t index)
#endif // defined(POOYA_USE_SMART_PTRS)
        : ValueSignalInfo(full_name, index)
    {
        _bool = true;
    }

    bool get() const
    {
        pooya_trace0;
        pooya_verify(_bool_value, _full_name + ": attempting to retrieve the value of an uninitialized bool signal!");
        pooya_verify(is_assigned(), _full_name + ": attempting to access an unassigned value!");
#if defined(POOYA_USE_SMART_PTRS)
        return _bool_value.value().get() != 0;
#else // defined(POOYA_USE_SMART_PTRS)
        return *_bool_value != 0;
#endif // defined(POOYA_USE_SMART_PTRS)
    }

    void set(bool value)
    {
        pooya_trace("value: " + std::to_string(value));
        pooya_verify(_bool_value, _full_name + ": attempting to assign the value of an uninitialized bool signal!");
        pooya_verify(!is_assigned(), _full_name + ": re-assignment is prohibited!");
#if defined(POOYA_USE_SMART_PTRS)
        _bool_value.value().get() = value;
#else // defined(POOYA_USE_SMART_PTRS)
        *_bool_value = value;
#endif // defined(POOYA_USE_SMART_PTRS)
        _assigned = true;
    }

    operator bool() const {return get();}
};

#if defined(POOYA_USE_SMART_PTRS)

inline BoolSignalId SignalInfo::as_bool() {return _bool ? std::static_pointer_cast<BoolSignalInfo>(shared_from_this()) : BoolSignalId();}
inline ReadOnlyBoolSignalId SignalInfo::as_bool() const {return _bool ? std::static_pointer_cast<const BoolSignalInfo>(shared_from_this()) : ReadOnlyBoolSignalId();}

#else // defined(POOYA_USE_SMART_PTS)

inline BoolSignalId SignalInfo::as_bool() {return _bool ? static_cast<BoolSignalId>(this) : nullptr;}
inline ReadOnlyBoolSignalId SignalInfo::as_bool() const {return _bool ? static_cast<ReadOnlyBoolSignalId>(this) : nullptr;}

#endif // defined(POOYA_USE_SMART_PTS)

}

#endif // __POOYA_SIGNAL_BOOL_SIGNAL_HPP__
