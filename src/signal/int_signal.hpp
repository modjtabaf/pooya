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

#ifndef __POOYA_SIGNAL_INT_SIGNAL_HPP__
#define __POOYA_SIGNAL_INT_SIGNAL_HPP__

#ifdef POOYA_INT_SIGNAL

#include <cmath>
#include <memory>

#include "src/helper/trace.hpp"
#include "src/helper/util.hpp"
#include "src/helper/verify.hpp"
#include "value_signal.hpp"

namespace pooya
{

class IntSignalImpl : public ValueSignalImpl
{
public:
    using Base = ValueSignalImpl;
    using Ptr  = std::shared_ptr<IntSignalImpl>;

    IntSignalImpl(Protected, std::string_view name) : Base(name) {}

    static Ptr create_new(std::string_view name) { return std::make_shared<IntSignalImpl>(Protected(), name); }

    int get_value() const
    {
        pooya_trace0;
        pooya_debug_verify(is_assigned(), name().str() + ": attempting to access an unassigned value!");
        return _int_value;
    }

    void set_value(int value)
    {
        pooya_trace("value: " + std::to_string(value));
        pooya_debug_verify(!is_assigned(), name().str() + ": re-assignment is prohibited!");
        _int_value = value;
        _assigned  = true;
    }

protected:
    int _int_value;
};

class IntSignal : public SignalT<int>
{
public:
    using Base = SignalT<int>;

    IntSignal() : IntSignal("") {}
    IntSignal(const IntSignal& sig) : Base(sig) {}

    explicit IntSignal(std::string_view name) : Base(*IntSignalImpl::create_new(name).get()) {}
    explicit IntSignal(SignalImpl& sig) : Base(sig) {}
    explicit IntSignal(const Signal& sig) : Base(sig) {}

    void operator=(const Signal&) = delete;
    void operator=(const IntSignal& sig) { _typed_ptr->set_value(sig); }
    void operator=(int value) { _typed_ptr->set_value(value); }

    operator int() const { return _typed_ptr->get_value(); }

    template<typename T>
    explicit operator T() const
    {
        return static_cast<T>(_typed_ptr->get_value());
    }
};

} // namespace pooya

#endif // POOYA_INT_SIGNAL

#endif // __POOYA_SIGNAL_INT_SIGNAL_HPP__
