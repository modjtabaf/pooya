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

#ifndef __POOYA_SIGNAL_ARRAY_SIGNAL_HPP__
#define __POOYA_SIGNAL_ARRAY_SIGNAL_HPP__

#include "array.hpp"
#include "float_signal.hpp"
#include "src/helper/trace.hpp"
#include "src/helper/util.hpp"
#include "src/helper/verify.hpp"

#define pooya_verify_array_signal(sig)                                                                                 \
    pooya_verify_valid_signal(sig);                                                                                    \
    pooya_verify((sig)->is_array(), (sig)->name().str() + ": array signal expected!");

#define pooya_verify_array_signal_size(sig, size_)                                                                     \
    pooya_verify_array_signal(sig);                                                                                    \
    pooya_verify((sig)->as_array().size() == size_, (sig)->name().str() + ": array size mismatch!");

namespace pooya
{

class ArraySignalImpl : public FloatSignalImpl
{
protected:
    Array _array_value;

public:
    ArraySignalImpl(Protected, const ValidName& name, std::size_t size)
        : FloatSignalImpl(name, ArrayType, size), _array_value(size)
    {
    }

    static ArraySignalImplPtr create_new(std::size_t size)
    {
        return std::make_shared<ArraySignalImpl>(Protected(), "", size);
    }
    static ArraySignalImplPtr create_new(const ValidName& name, std::size_t size)
    {
        return std::make_shared<ArraySignalImpl>(Protected(), name, size);
    }

    const Array& get() const
    {
        pooya_trace0;
        pooya_verify(_array_value.rows() == int(_size),
                     name().str() + ": attempting to retrieve the value of an uninitialized array signal!");
        pooya_verify(is_assigned(), name().str() + ": attempting to access an unassigned value!");
        return _array_value;
    }

    void set(const Array& value)
    {
        pooya_verify(_array_value.rows() == int(_size),
                     name().str() + ": attempting to assign the value of an uninitialized array signal!");
        pooya_verify(!is_assigned(), name().str() + ": re-assignment is prohibited!");
        pooya_verify(value.rows() == int(_size), std::string("size mismatch (id=") + name().str() + ")(" +
                                                     std::to_string(_size) + " vs " + std::to_string(value.rows()) +
                                                     ")!");
        _array_value = value;
        _assigned    = true;
    }
};

inline ArraySignalImpl& SignalImpl::as_array()
{
    pooya_verify(_type & ArrayType, "Illegal attempt to dereference a non-array as an array.");
    return *static_cast<ArraySignalImpl*>(this);
}

inline const ArraySignalImpl& SignalImpl::as_array() const
{
    pooya_verify(_type & ArrayType, "Illegal attempt to dereference a non-array as an array.");
    return *static_cast<const ArraySignalImpl*>(this);
}

class ArraySignal : public FloatSignal<Array>
{
    using Base = FloatSignal<Array>;

public:
    explicit ArraySignal(std::size_t size = 1) : Base(ArraySignalImpl::create_new(size)) {}
    ArraySignal(const ValidName& name, std::size_t size = 1) : Base(ArraySignalImpl::create_new(name, size)) {}
    explicit ArraySignal(const SignalImplPtr& sid)
        : Base(sid && sid->is_array() ? std::static_pointer_cast<ArraySignalImpl>(sid) : nullptr)
    {
    }
    ArraySignal(const ArraySignalImplPtr& sid) : Base(sid) {}

    ArraySignal& operator=(const ArraySignal&) = delete;

    void reset(std::size_t size) { _sid = ArraySignalImpl::create_new(size); }
    void reset(const ValidName& name, std::size_t size) { _sid = ArraySignalImpl::create_new(name, size); }

    using ValueSignal<Array>::operator=;
    using Signal<Array>::reset;

    double operator[](std::size_t index) const { return _sid->get()[index]; }
};

} // namespace pooya

#endif // __POOYA_SIGNAL_ARRAY_SIGNAL_HPP__
