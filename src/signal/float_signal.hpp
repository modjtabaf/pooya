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

#include "signal.hpp"
#include "src/helper/trace.hpp"
#include "src/helper/util.hpp"
#include "src/helper/verify.hpp"
#include "value_signal.hpp"

namespace pooya
{

class FloatSignalInfo : public ValueSignalInfo
{
    // friend class Model;

protected:
    FloatSignalId _deriv_sig{nullptr}; // the derivative signal if this is a state variable, nullptr otherwise
    const std::size_t _size;

    FloatSignalInfo(const std::string& full_name, uint32_t type, std::size_t size=1) :
        ValueSignalInfo(full_name, type | FloatType), _size(size) {}

public:
    bool is_state_variable() const {return static_cast<bool>(_deriv_sig);}
    std::size_t size() const {return _size;}
    void set_deriv_signal(FloatSignalId deriv_sig)
    {
        pooya_verify_valid_signal(deriv_sig);
        // pooya_verify(!sig->is_state_variable(), sig->_full_name + ": signal is already registered as a state variable!");
        // pooya_verify((sig->is_scalar() && deriv_sig->is_scalar()) || (sig->is_array() && deriv_sig->is_array() && sig->as_array()._size == deriv_sig->as_array()._size),
        //     sig->_full_name + ", " + deriv_sig->_full_name + ": type or size mismatch!");
        pooya_verify(_type == deriv_sig->_type, _full_name + ", " + deriv_sig->_full_name + ": type mismatch!");
        pooya_verify(_size == deriv_sig->_size, _full_name + ", " + deriv_sig->_full_name + ": size mismatch!");

        _deriv_sig = deriv_sig;
    }
    FloatSignalId& deriv_signal() {return _deriv_sig;}
    const FloatSignalId& deriv_signal() const {return _deriv_sig;}
};

inline FloatSignalInfo& SignalInfo::as_float()
{
    pooya_verify(_type & FloatType, "Illegal attempt to dereference a non-float as a float.");
    return *static_cast<FloatSignalInfo*>(this);
}

inline const FloatSignalInfo& SignalInfo::as_float() const
{
    pooya_verify(_type & FloatType, "Illegal attempt to dereference a non-float as a float.");
    return *static_cast<const FloatSignalInfo*>(this);
}

}

#endif // __POOYA_SIGNAL_FLOAT_SIGNAL_HPP__
