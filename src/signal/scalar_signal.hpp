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

#include <optional>

#include "signal.hpp"
#include "src/helper/trace.hpp"
#include "src/helper/util.hpp"
#include "src/helper/verify.hpp"
#include "float_signal.hpp"

#define pooya_verify_scalar_signal(sig) \
    pooya_verify_valid_signal(sig); \
    pooya_verify((sig)->is_scalar(), (sig)->_full_name + ": scalar signal expected!");

namespace pooya
{

class ScalarSignalInfo : public FloatSignalInfo
{
    friend class Model;

protected:
    std::optional<std::reference_wrapper<double>> _scalar_value;
    std::optional<std::reference_wrapper<double>> _deriv_scalar_value;    // only valid if _is_deriv is true, nullptr otherwise

    static ScalarSignalId create_new(const std::string& full_name, std::size_t index)
    {
        return std::make_shared<ScalarSignalInfo>(Protected(), full_name, index);
    } 

public:
    ScalarSignalInfo(Protected, const std::string& full_name, std::size_t index)
        : FloatSignalInfo(full_name, ScalarType, index) {}

    double get() const
    {
        pooya_trace0;
        pooya_verify(_scalar_value, _full_name + ": attempting to retrieve the value of an uninitialized scalar signal!");
        pooya_verify(is_assigned(), _full_name + ": attempting to access an unassigned value!");
        return _scalar_value.value().get();
    }

    void set(double value)
    {
        pooya_trace("value: " + std::to_string(value));
        pooya_verify(_scalar_value, _full_name + ": attempting to assign the value of an uninitialized scalar signal!");
        pooya_verify(!is_assigned(), _full_name + ": re-assignment is prohibited!");
        _scalar_value.value().get() = value;
        if (_deriv_scalar_value)
        {
            _deriv_scalar_value.value().get() = value;
        }
        _assigned = true;
    }

    operator double() const {return get();}
};

inline ScalarSignalInfo& SignalInfo::as_scalar()
{
    pooya_verify(_type & ScalarType, "Illegal attempt to dereference a non-scalar as a scalar.");
    return *static_cast<ScalarSignalInfo*>(this);
}

inline const ScalarSignalInfo& SignalInfo::as_scalar() const
{
    pooya_verify(_type & ScalarType, "Illegal attempt to dereference a non-scalar as a scalar.");
    return *static_cast<const ScalarSignalInfo*>(this);
}

}

#endif // __POOYA_SIGNAL_SCALAR_SIGNAL_HPP__
