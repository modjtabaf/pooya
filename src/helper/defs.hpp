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

#ifndef __POOYA_HELPER_DEFS_HPP__
#define __POOYA_HELPER_DEFS_HPP__

#include <utility>

#undef POOYA_DEBUG

#if !defined(NDEBUG)
#define POOYA_DEBUG
#endif

#define POOYA_INT_SIGNAL
#define POOYA_BOOL_SIGNAL
#define POOYA_ARRAY_SIGNAL

template<typename T>
struct is_pair : std::false_type
{
};

template<typename T1, typename T2>
struct is_pair<std::pair<T1, T2>> : std::true_type
{
};

template<typename T>
constexpr bool is_pair_v = is_pair<T>::value;

#endif // __POOYA_HELPER_DEFS_HPP__
