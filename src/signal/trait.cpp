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

#include "src/helper/ndebug.hpp"

#if defined(POOYA_DEBUG)

#include "trait.hpp"
#include "array_signal.hpp"
#include "scalar_signal.hpp"
#include "int_signal.hpp"
#include "bool_signal.hpp"
#include "bus.hpp"

namespace pooya
{

void Types<Array>::verify_signal_type(pooya::SignalImplPtr sig)
{
    pooya_verify_array_signal(sig);
}

void Types<Array>::verify_signal_type(pooya::SignalImplPtr sig, std::size_t size)
{
    pooya_verify_array_signal_size(sig, size);
}

void Types<double>::verify_signal_type(pooya::SignalImplPtr sig)
{
    pooya_verify_scalar_signal(sig);
}

void Types<int>::verify_signal_type(pooya::SignalImplPtr sig)
{
    pooya_verify_int_signal(sig);
}

void Types<bool>::verify_signal_type(pooya::SignalImplPtr sig)
{
    pooya_verify_bool_signal(sig);
}

void Types<BusSpec>::verify_signal_type(pooya::SignalImplPtr sig, const BusSpec& spec)
{
    pooya_verify_bus_spec(sig, spec);
}

}

#endif // defined(POOYA_DEBUG)
