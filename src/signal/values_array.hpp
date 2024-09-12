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

#ifndef __POOYA_SIGNAL_VALUES_ARRAY_HPP__
#define __POOYA_SIGNAL_VALUES_ARRAY_HPP__

#include "src/helper/trace.hpp"
#include "src/helper/verify.hpp"
#include "src/signal/array.hpp"

namespace pooya
{

class ValuesArray
{
    friend class Model;

public:
    using Values = Array;
    using StateVariables = MappedArray;
    using StateVariableDerivs = Array;

protected:
    Values _values;
    StateVariables _state_variables{nullptr, 0};
    StateVariableDerivs _state_variable_derivs;

public:
    ValuesArray() {}

    void init(std::size_t num_values, std::size_t num_state_variables);

    std::size_t num_state_variables() const {return _state_variables.size();}
    Values& values() {return _values;}
    const Values& values() const {return _values;}
    const StateVariables& state_variables() const {return _state_variables;}
    StateVariableDerivs& state_variable_derivs() {return _state_variable_derivs;}
    const StateVariableDerivs& state_variable_derivs() const {return _state_variable_derivs;}
};

}

#endif // __POOYA_SIGNAL_VALUES_ARRAY_HPP__
