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

#define POOYA_HERE(s) std::cout << "Pooya:" << __FILE__ << ":" << __LINE__ << s << "\n" << std::flush;
// #undef POOYA_HERE
// #define POOYA_HERE

#define POOYA_HERE0 POOYA_HERE("")
// #undef POOYA_HERE0
// #define POOYA_HERE0

#define  get_input(T, index) values.get<T>(_iports[index])
#define  scalar_input(index) get_input(double, index)
#define   array_input(index) get_input(pooya::Array, index)

#define set_output(T, index, value) values.set<T>(_oports[index], value)
#define scalar_output(index, value) set_output(double, index, value)
#define  array_output(index, value) set_output(pooya::Array, index, value)

#define verify(cond, msg) \
    if (!(cond)) \
        throw std::runtime_error( \
            "\nPooya Exception:\n" + \
            std::string("  @" __FILE__ ":") + std::to_string(__LINE__) + "\n" \
            "  " + (msg) + "\n")

// utility functions

namespace pooya
{

std::string generate_random_name(int len=10);

}

#endif // __POOYA_UTIL_HPP__
