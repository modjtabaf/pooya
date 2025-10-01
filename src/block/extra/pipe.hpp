/*
Copyright 2024 Mojtaba (Moji) Fathi

 Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the “Software”), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __POOYA_BLOCK_PIPE_HPP__
#define __POOYA_BLOCK_PIPE_HPP__

#include "src/block/singleio.hpp"
#include "src/signal/array.hpp"

namespace pooya
{

template<typename T>
class PipeT : public SingleInputOutputT<T>
{
public:
    explicit PipeT() : SingleInputOutputT<T>(1) {}
    explicit PipeT(Submodel* parent) : SingleInputOutputT<T>(parent, 1) {}

    void activation_function(double /*t*/) override
    {
        pooya_trace("block: " + SingleInputOutputT<T>::full_name().str());
        SingleInputOutputT<T>::_s_out = SingleInputOutputT<T>::_s_in;
    }
};

using Pipe = PipeT<double>;

#ifdef POOYA_INT_SIGNAL
using PipeI = PipeT<int>;
#endif // POOYA_INT_SIGNAL

#ifdef POOYA_BOOL_SIGNAL
using PipeB = PipeT<bool>;
#endif // POOYA_BOOL_SIGNAL

#ifdef POOYA_ARRAY_SIGNAL
using PipeA = PipeT<Array>;
#endif // POOYA_ARRAY_SIGNAL

} // namespace pooya

#endif // __POOYA_BLOCK_PIPE_HPP__
