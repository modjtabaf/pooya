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

#ifdef POOYA_ARRAY_SIGNAL

#include "array.hpp"
#include "float_signal.hpp"
#include "src/helper/trace.hpp"
#include "src/helper/util.hpp"
#include "src/helper/verify.hpp"

namespace pooya
{

class ArraySignalImpl : public FloatSignalImplT<Array>
{
protected:
    Array _array_value;

public:
    ArraySignalImpl(Protected, std::size_t size, const ValidName& name)
        : FloatSignalImplT(size, name), _array_value(size)
    {
    }

    static std::shared_ptr<ArraySignalImpl> create_new(std::size_t size, const ValidName& name)
    {
        return std::make_shared<ArraySignalImpl>(Protected(), size, name);
    }

    const Array& get() const
    {
        pooya_trace0;
        pooya_verify(_array_value.rows() == int(_size),
                     name().str() + ": attempting to retrieve the value of an uninitialized array signal!");
        pooya_verify(is_assigned(), name().str() + ": attempting to access an unassigned value!");
        return _array_value;
    }

    double get(std::size_t index) const { return get()[index]; }

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

    const Array& operator=(const Array& value)
    {
        set(value);
        return value;
    }

    operator const Array&() const { return get(); }
    double operator[](std::size_t index) const { return get(index); }
};

class ArraySignal : public SignalT<Array>
{
    using Base = SignalT<Array>;

public:
    explicit ArraySignal(std::size_t size = 1, const ValidName& name = "")
        : Base(ArraySignalImpl::create_new(size, name).get())
    {
    }

    explicit ArraySignal(SignalImpl* sig) : Base(sig) {}

    explicit ArraySignal(const Signal& sig) : Base(sig) {}

    using Base::operator=;
};

} // namespace pooya

#endif // POOYA_ARRAY_SIGNAL

#endif // __POOYA_SIGNAL_ARRAY_SIGNAL_HPP__
