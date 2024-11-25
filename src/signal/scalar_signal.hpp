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
#include "src/helper/util.hpp"
#include "src/helper/verify.hpp"
#include "float_signal.hpp"

#define pooya_verify_scalar_signal(sig) \
    pooya_verify_valid_signal(sig); \
    pooya_verify((sig)->is_scalar(), (sig)->name().str() + ": scalar signal expected!");

namespace pooya
{

class ScalarSignalImpl : public FloatSignalImpl
{
protected:
    double _scalar_value;

public:
    ScalarSignalImpl(Protected, const ValidName& name="")
        : FloatSignalImpl(name, ScalarType) {}

    static ScalarSignalImplPtr create_new(const ValidName& name="")
    {
        return std::make_shared<ScalarSignalImpl>(Protected(), name);
    } 

    double get() const
    {
        pooya_trace0;
        pooya_verify(is_assigned(), name().str() + ": attempting to access an unassigned value!");
        return _scalar_value;
    }

    void set(double value)
    {
        pooya_trace("value: " + std::to_string(value));
        pooya_verify(!is_assigned(), name().str() + ": re-assignment is prohibited!");
        _scalar_value = value;
        _assigned = true;
    }

    operator double() const {return get();}
};

inline ScalarSignalImpl& SignalImpl::as_scalar()
{
    pooya_verify(_type & ScalarType, "Illegal attempt to dereference a non-scalar as a scalar.");
    return *static_cast<ScalarSignalImpl*>(this);
}

inline const ScalarSignalImpl& SignalImpl::as_scalar() const
{
    pooya_verify(_type & ScalarType, "Illegal attempt to dereference a non-scalar as a scalar.");
    return *static_cast<const ScalarSignalImpl*>(this);
}

class ScalarSignal : public FloatSignal<ScalarSignal, double>
{
    using Base = FloatSignal<ScalarSignal, double>;

public:
    explicit ScalarSignal(const ValidName& name="") : Base(ScalarSignalImpl::create_new(name)) {}
    ScalarSignal(const ScalarSignalImplPtr& sid) : Base(sid) {}

    ScalarSignal& operator=(const ScalarSignal&) = delete;

    void reset(const ValidName& name="")
    {
        _sid = ScalarSignalImpl::create_new(name);
    }

    using ValueSignal<ScalarSignal, double>::operator=;
    using Signal<ScalarSignal, double>::reset;
};

}

#endif // __POOYA_SIGNAL_SCALAR_SIGNAL_HPP__
