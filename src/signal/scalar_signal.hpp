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

#ifndef __POOYA_SIGNAL_SCALAR_SIGNAL_HPP__
#define __POOYA_SIGNAL_SCALAR_SIGNAL_HPP__

#include "src/helper/trace.hpp"
#include "src/helper/verify.hpp"
#include "float_signal.hpp"

#if defined(POOYA_USE_SMART_PTRS)
#include <optional>
#endif // defined(POOYA_USE_SMART_PTRS)

#define pooya_verify_scalar_signal(sig) \
    pooya_verify_valid_signal(sig); \
    pooya_verify((sig)->is_scalar(), (sig)->_full_name + ": scalar signal expected!");

namespace pooya
{

class ScalarSignalInfo : public FloatSignalInfo
{
    friend class Model;

protected:
#if defined(POOYA_USE_SMART_PTRS)
    std::optional<std::reference_wrapper<double>> _scalar_value;
    std::optional<std::reference_wrapper<double>> _deriv_scalar_value;    // only valid if _is_deriv is true, nullptr otherwise
#else // defined(POOYA_USE_SMART_PTRS)
    double* _scalar_value{nullptr};
    double* _deriv_scalar_value{nullptr};    // only valid if _is_deriv is true, nullptr otherwise
#endif // defined(POOYA_USE_SMART_PTRS)

    static ScalarSignalId create_new(const std::string& full_name, std::size_t index)
    {
#if defined(POOYA_USE_SMART_PTRS)
        return std::make_shared<ScalarSignalInfo>(Protected(), full_name, index);
#else // defined(POOYA_USE_SMART_PTRS)
        return new ScalarSignalInfo(full_name, index);
#endif // defined(POOYA_USE_SMART_PTRS)
    } 

public:
#if defined(POOYA_USE_SMART_PTRS)
    ScalarSignalInfo(Protected, const std::string& full_name, std::size_t index)
#else // defined(POOYA_USE_SMART_PTRS)
    ScalarSignalInfo(const std::string& full_name, std::size_t index)
#endif // defined(POOYA_USE_SMART_PTRS)
        : FloatSignalInfo(full_name, index)
    {
        _scalar = true;
    }

    double get() const
    {
        pooya_trace0;
        pooya_verify(_scalar_value, _full_name + ": attempting to retrieve the value of an uninitialized scalar signal!");
        pooya_verify(is_assigned(), _full_name + ": attempting to access an unassigned value!");
#if defined(POOYA_USE_SMART_PTRS)
        return _scalar_value.value().get();
#else // defined(POOYA_USE_SMART_PTRS)
        return *_scalar_value;
#endif // defined(POOYA_USE_SMART_PTRS)
    }

    void set(double value)
    {
        pooya_trace("value: " + std::to_string(value));
        pooya_verify(_scalar_value, _full_name + ": attempting to assign the value of an uninitialized scalar signal!");
        pooya_verify(!is_assigned(), _full_name + ": re-assignment is prohibited!");
#if defined(POOYA_USE_SMART_PTRS)
        _scalar_value.value().get() = value;
        if (_deriv_scalar_value)
        {
            _deriv_scalar_value.value().get() = value;
        }
#else // defined(POOYA_USE_SMART_PTRS)
        *_scalar_value = value;
        if (_deriv_scalar_value)
        {
            *_deriv_scalar_value = value;
        }
#endif // defined(POOYA_USE_SMART_PTRS)
        _assigned = true;
    }

    operator double() const {return get();}
};

#if defined(POOYA_USE_SMART_PTRS)

inline ScalarSignalId SignalInfo::as_scalar() {return _scalar ? std::static_pointer_cast<ScalarSignalInfo>(shared_from_this()) : ScalarSignalId();}
inline ReadOnlyScalarSignalId SignalInfo::as_scalar() const {return _scalar ? std::static_pointer_cast<const ScalarSignalInfo>(shared_from_this()) : ReadOnlyScalarSignalId();}

#else // defined(POOYA_USE_SMART_PTS)

inline ScalarSignalId SignalInfo::as_scalar() {return _scalar ? static_cast<ScalarSignalId>(this) : nullptr;}
inline ReadOnlyScalarSignalId SignalInfo::as_scalar() const {return _scalar ? static_cast<ReadOnlyScalarSignalId>(this) : nullptr;}

#endif // defined(POOYA_USE_SMART_PTS)

}

#endif // __POOYA_SIGNAL_SCALAR_SIGNAL_HPP__
