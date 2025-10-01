/*
Copyright 2024 Mojtaba (Moji) Fathi

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
documentation files (the “Software”), to deal in the Software without restriction, including without limitation the
rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit
persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
Software.

 THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __POOYA_SIGNAL_SCALAR_SIGNAL_HPP__
#define __POOYA_SIGNAL_SCALAR_SIGNAL_HPP__

#include <memory>

#include "float_signal.hpp"
#include "src/helper/trace.hpp"
#include "src/helper/util.hpp"
#include "src/helper/verify.hpp"

namespace pooya
{

class ScalarSignalImpl : public FloatSignalImplT<double>
{
public:
    using Base = FloatSignalImplT<double>;
    using Ptr  = std::shared_ptr<ScalarSignalImpl>;

    ScalarSignalImpl(Protected, const ValidName& name = "") : Base(1, name) {}

    static std::shared_ptr<ScalarSignalImpl> create_new(const ValidName& name = "")
    {
        return std::make_shared<ScalarSignalImpl>(Protected(), name);
    }

    double get_value() const
    {
        pooya_trace0;
        pooya_debug_verify(is_assigned(), name().str() + ": attempting to access an unassigned value!");
        return _scalar_value;
    }

    void set_value(double value)
    {
        pooya_trace("value: " + std::to_string(value));
        pooya_debug_verify(!is_assigned(), name().str() + ": re-assignment is prohibited!");
        _scalar_value = value;
        _assigned     = true;
    }

protected:
    double _scalar_value;
};

class ScalarSignal : public SignalT<double>
{
public:
    using Base = SignalT<double>;

    ScalarSignal() : Base(*ScalarSignalImpl::create_new("").get()) {}
    ScalarSignal(const ScalarSignal& sig) : Base(sig) {}

    explicit ScalarSignal(const ValidName& name) : Base(*ScalarSignalImpl::create_new(name).get()) {}
    explicit ScalarSignal(SignalImpl& sig) : Base(sig) {}
    explicit ScalarSignal(const Signal& sig) : Base(sig) {}

    void operator=(const Signal&) = delete;
    void operator=(const ScalarSignal& sig) { _typed_ptr->set_value(sig); }
    void operator=(double value) { _typed_ptr->set_value(value); }

    operator double() const { return _typed_ptr->get_value(); }
};

} // namespace pooya

#endif // __POOYA_SIGNAL_SCALAR_SIGNAL_HPP__
