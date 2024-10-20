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

#include "value_signal.hpp"
#include "scalar_signal.hpp"
#include "int_signal.hpp"
#include "bool_signal.hpp"

namespace pooya
{

double ValueSignalInfo::get_as_scalar() const
{
    if (is_scalar())
    {
        return as_scalar().get();
    }
    else if (is_int())
    {
        return static_cast<double>(as_int().get());
    }
    else
    {
        pooya_verify(is_bool(), "boolean signal expected!");
        return as_bool().get() ? 1 : 0;
    }
}

void ValueSignalInfo::set_as_scalar(double value)
{
    if (is_scalar())
    {
        as_scalar().set(value);
    }
    else if (is_int())
    {
        as_int().set(std::round(value));
    }
    else
    {
        pooya_verify(is_bool(), "boolean signal expected!");
        as_bool().set(value != 0);
    }
}

}
