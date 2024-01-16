/*
Copyright 2023 Mojtaba (Moji) Fathi

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”),
to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __POOYA_UTIL_HPP__
#define __POOYA_UTIL_HPP__

#include <string>

// helper macros

#define POOYA_HERE(s) std::cout << "Pooya: " << __FILE__ << ":" << __LINE__ << ": " << s << "\n" << std::flush;
// #undef POOYA_HERE
// #define POOYA_HERE(s)

#define POOYA_HERE0 POOYA_HERE("")

#if defined(NDEBUG)
#define verify(cond, msg)
#else
#define verify(cond, msg) \
    if (!(cond)) \
        throw std::runtime_error( \
            "\nPooya Exception:\n" + \
            std::string("  " __FILE__ ":") + std::to_string(__LINE__) + "\n" \
            "  " + (msg) + "\n")
#endif // defined(NDEBUG)

#define  verify_valid_signal(sig) verify(sig, "invalid signal!")

#define  verify_value_signal(sig) \
    verify_valid_signal(sig); \
    verify((sig)->as_value(), (sig)->_full_name + ": value signal expected!");

#define verify_scalar_signal(sig) \
    verify_valid_signal(sig); \
    verify((sig)->as_scalar(), (sig)->_full_name + ": scalar signal expected!");

#define verify_array_signal(sig) \
    verify_valid_signal(sig); \
    verify((sig)->as_array(), (sig)->_full_name + ": array signal expected!");

#define verify_float_signal(sig) \
    verify_valid_signal(sig); \
    verify(!(sig)->as_integer(), (sig)->_full_name + ": float signal expected!");

#define verify_integer_signal(sig) \
    verify_valid_signal(sig); \
    verify((sig)->as_integer(), (sig)->_full_name + ": integer signal expected!");

#define verify_array_signal_size(sig, size_) \
    verify_array_signal(sig); \
    verify((sig)->as_array()->_size == size_, (sig)->_full_name + ": array size mismatch!");

#define verify_bus_signal(sig) \
    verify_valid_signal(sig); \
    verify((sig)->as_bus(), (sig)->_full_name + ": bus signal expected!");

#define verify_bus_signal_spec(sig, spec_) \
    verify_bus_signal(sig); \
    verify((sig)->as_bus()->spec() == spec_, (sig)->_full_name + ": bus spec mismatch!");

// utility functions

#endif // __POOYA_UTIL_HPP__
