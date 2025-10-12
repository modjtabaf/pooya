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

#ifndef __POOYA_SIGNAL_FLOAT_SIGNAL_HPP__
#define __POOYA_SIGNAL_FLOAT_SIGNAL_HPP__

#include "src/helper/trace.hpp"
#include "src/helper/util.hpp"
#include "src/helper/verify.hpp"
#include "value_signal.hpp"

namespace pooya
{

template<typename T>
class FloatSignalImplT : public ValueSignalImpl
{
public:
    using Base = ValueSignalImpl;
    using Ptr  = std::shared_ptr<FloatSignalImplT<T>>;

    typename Types<T>::SignalImpl* state_variable() const { return _deriv_sig.get(); }
    std::size_t size() const { return _size; }

    void set_deriv_signal(const Signal& deriv_sig)
    {
        Signal::fail_if_invalid_signal_type<T>(&deriv_sig.impl());
        _deriv_sig = std::static_pointer_cast<typename Types<T>::SignalImpl>(deriv_sig->shared_from_this());
        pooya_verify(_size == _deriv_sig->size(), name().str() + ", " + deriv_sig->name().str() + ": size mismatch!");
    }

    typename Types<T>::SignalImpl* deriv_signal() const { return _deriv_sig.get(); }

protected:
    std::shared_ptr<typename Types<T>::SignalImpl> _deriv_sig{
        nullptr}; // the derivative signal if this is a state variable, nullptr otherwise
    const std::size_t _size;

    FloatSignalImplT(std::size_t size, std::string_view name) : Base(name), _size(size) {}
};

} // namespace pooya

#endif // __POOYA_SIGNAL_FLOAT_SIGNAL_HPP__
