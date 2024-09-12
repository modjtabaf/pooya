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
#include "src/helper/verify.hpp"
#include "signal.hpp"

#define  pooya_verify_value_signal(sig) \
    pooya_verify_valid_signal(sig); \
    pooya_verify((sig)->is_value(), (sig)->_full_name + ": value signal expected!");

namespace pooya
{

class ValueSignalInfo : public SignalInfo
{
    friend class Model;

protected:
    bool _assigned{false};             // has the value been assigned?

    ValueSignalInfo(const std::string& full_name, std::size_t index) :
        SignalInfo(full_name, index)
    {
        _value = true;
    }

public:
    double get_as_scalar() const;
    void set_as_scalar(double value);

    bool is_assigned() const {return _assigned;}
};

#if defined(POOYA_USE_SMART_PTRS)

inline ValueSignalId SignalInfo::as_value() {return _value ? std::static_pointer_cast<ValueSignalInfo>(shared_from_this()) : ValueSignalId();}
inline ReadOnlyValueSignalId SignalInfo::as_value() const {return _value ? std::static_pointer_cast<const ValueSignalInfo>(shared_from_this()) : ReadOnlyValueSignalId();}

#else // defined(POOYA_USE_SMART_PTS)

inline ValueSignalId SignalInfo::as_value() {return _value ? static_cast<ValueSignalId>(this) : nullptr;}
inline ReadOnlyValueSignalId SignalInfo::as_value() const {return _value ? static_cast<ReadOnlyValueSignalId>(this) : nullptr;}

#endif // defined(POOYA_USE_SMART_PTS)

}

#endif // __POOYA_SIGNAL_VALUE_SIGNAL_HPP__
