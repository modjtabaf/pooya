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

#include "values_array.hpp"

namespace pooya
{

void ValuesArray::init(std::size_t num_values, std::size_t num_state_variables)
{
    pooya_trace("ValuesArray: " + std::to_string(num_values) + " " + std::to_string(num_state_variables) + " states");
    pooya_verify(num_state_variables <= num_values, "Number of state variables (" + std::to_string(num_state_variables)
        + " cannot be more than the number of values (" + std::to_string(num_values) + ").");

    _values.resize(num_values);
    new (&_state_variables) StateVariables(_values.data(), num_state_variables);
    _state_variable_derivs.resize(num_state_variables);
}

}
