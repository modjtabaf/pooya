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

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace pooya::util
{

// helper macros

#ifdef NDEBUG
#define POOYA_NDEBUG
#else
#undef POOYA_NDEBUG
#endif

#define pooya_here(s) std::cout << "Pooya: " << __FILE__ << ":" << __LINE__ << ": " << s << "\n" << std::flush;
// #undef pooya_here
// #define pooya_here(s)

#define pooya_here0 pooya_here("")

#if defined(POOYA_NDEBUG)
#define verify(cond, msg)
#define pooya_trace(msg)
#define pooya_trace_update(msg)
#else

std::string pooya_trace_info_string();
void pooya_throw_exception(const std::string& file, int line, const std::string& msg);

#define verify(cond, msg) if (!(cond)) {pooya::util::pooya_throw_exception(__FILE__, __LINE__, msg);}

struct PooyaTraceInfo
{
    std::string _file;
    int _line{-1};
    std::string _msg;
};

extern std::vector<PooyaTraceInfo> pooya_trace_info;

class PooyaTracer
{
public:
    PooyaTracer()
    {
        pooya_trace_info.emplace_back();
    }

    static void update(const std::string& file, int line, const std::string& msg)
    {
        auto& pt = pooya_trace_info.back();
        pt._file = file;
        pt._line = line;
        if (!msg.empty())
            pt._msg = msg;
    }

    ~PooyaTracer()
    {
        pooya_trace_info.pop_back();
    }
};

#define pooya_trace(msg) const pooya::util::PooyaTracer __pooya_tracer__; pooya_trace_update(msg);
#define pooya_trace_update(msg) __pooya_tracer__.update(__FILE__, __LINE__, msg);

#endif // defined(POOYA_NDEBUG)

#define pooya_trace0 pooya_trace("")
#define pooya_trace_update0 pooya_trace_update("")

#define  verify_valid_signal(sig) \
    verify(sig, "invalid signal!")

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
    verify(!(sig)->as_int() && !(sig)->as_bool(), (sig)->_full_name + ": float signal expected!");

#define verify_int_signal(sig) \
    verify_valid_signal(sig); \
    verify((sig)->as_int(), (sig)->_full_name + ": int signal expected!");

#define verify_bool_signal(sig) \
    verify_valid_signal(sig); \
    verify((sig)->as_bool(), (sig)->_full_name + ": bool signal expected!");

#define verify_array_signal_size(sig, size_) \
    verify_array_signal(sig); \
    verify((sig)->as_array()->_size == size_, (sig)->_full_name + ": array size mismatch!");

#define verify_bus(sig) \
    verify_valid_signal(sig); \
    verify((sig)->as_bus(), (sig)->_full_name + ": bus signal expected!");

#define verify_bus_spec(sig, spec_) \
    verify_bus(sig); \
    verify((sig)->as_bus()->spec() == spec_, (sig)->_full_name + ": bus spec mismatch!");

// utility functions

// utility classes

}

#endif // __POOYA_UTIL_HPP__
