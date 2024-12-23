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

#include "bus_pipe.hpp"
#include "pipe.hpp"
#include "src/signal/array_signal.hpp"
#include "src/signal/bool_signal.hpp"
#include "src/signal/int_signal.hpp"
#include "src/signal/scalar_signal.hpp"
#include "submodel.hpp"

namespace pooya
{

void BusPipe::block_builder(const std::string& /*full_label*/, const SignalImplPtr& sig_in,
                            const SignalImplPtr& sig_out)
{
    pooya_trace("block: " + full_name().str());
    std::shared_ptr<Block> block;
    if (sig_in->is_scalar())
    {
        block = std::make_shared<Pipe>();
    }
    else if (sig_in->is_int())
    {
        block = std::make_shared<PipeI>();
    }
    else if (sig_in->is_bool())
    {
        block = std::make_shared<PipeB>();
    }
    else if (sig_in->is_array())
    {
        block = std::make_shared<PipeA>();
    }
    else
    {
        pooya_verify(false, "cannot create a pipe block for a non-value signal.");
    }

    block->connect(sig_in, sig_out);
    _blocks.emplace_back(std::move(block));
}

} // namespace pooya
