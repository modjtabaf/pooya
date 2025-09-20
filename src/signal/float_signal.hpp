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

#include "signal.hpp"
#include "src/helper/trace.hpp"
#include "src/helper/util.hpp"
#include "src/helper/verify.hpp"
#include "value_signal.hpp"
#include <memory>

namespace pooya
{

template<typename T>
class FloatSignalImplT : public ValueSignalImpl
{
protected:
    std::shared_ptr<typename Types<T>::SignalImpl> _deriv_sig{
        nullptr}; // the derivative signal if this is a state variable, nullptr otherwise
    const std::size_t _size;

    FloatSignalImplT(std::size_t size, const ValidName& name) : ValueSignalImpl(name), _size(size) {}

public:
    bool is_state_variable() const { return static_cast<bool>(_deriv_sig); }
    std::size_t size() const { return _size; }
    void set_deriv_signal(typename Types<T>::SignalImpl* deriv_sig)
    {
        if (!deriv_sig) return;
        pooya_verify(_size == deriv_sig->_size, name().str() + ", " + deriv_sig->name().str() + ": size mismatch!");

        _deriv_sig = std::static_pointer_cast<typename Types<T>::SignalImpl>(deriv_sig->shared_from_this());
    }
    typename Types<T>::SignalImpl* deriv_signal() { return _deriv_sig.get(); }
    const typename Types<T>::SignalImpl* deriv_signal() const { return _deriv_sig.get(); }
};

} // namespace pooya

#endif // __POOYA_SIGNAL_FLOAT_SIGNAL_HPP__
