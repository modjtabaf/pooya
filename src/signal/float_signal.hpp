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

#ifndef __POOYA_SIGNAL_FLOAT_SIGNAL_HPP__
#define __POOYA_SIGNAL_FLOAT_SIGNAL_HPP__

#include "signal_id.hpp"
#include "value_signal.hpp"

namespace pooya
{

class FloatSignalInfo : public ValueSignalInfo
{
    friend class Model;

protected:
    FloatSignalId _deriv_sig{nullptr}; // the derivative signal if this is a state variable, nullptr otherwise
    bool _is_deriv{false};             // is this the derivative of another signal?

    FloatSignalInfo(const std::string& full_name, std::size_t index) :
        ValueSignalInfo(full_name, index)
    {
        _float = true;
    }

public:
    bool is_state_variable() const {return static_cast<bool>(_deriv_sig);}
};


#if defined(POOYA_USE_SMART_PTRS)

inline FloatSignalId SignalInfo::as_float() {return _float ? std::static_pointer_cast<FloatSignalInfo>(shared_from_this()) : FloatSignalId();}
inline ReadOnlyFloatSignalId SignalInfo::as_float() const {return _float ? std::static_pointer_cast<const FloatSignalInfo>(shared_from_this()) : ReadOnlyFloatSignalId();}

#else // defined(POOYA_USE_SMART_PTS)

inline FloatSignalId SignalInfo::as_float() {return _float ? static_cast<FloatSignalId>(this) : nullptr;}
inline ReadOnlyFloatSignalId SignalInfo::as_float() const {return _float ? static_cast<ReadOnlyFloatSignalId>(this) : nullptr;}

#endif // defined(POOYA_USE_SMART_PTS)
}

#endif // __POOYA_SIGNAL_FLOAT_SIGNAL_HPP__
