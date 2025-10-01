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
#include "src/block/submodel.hpp"
#include "src/signal/array_signal.hpp"
#include "src/signal/bool_signal.hpp"
#include "src/signal/int_signal.hpp"
#include "src/signal/scalar_signal.hpp"

namespace pooya
{

void BusPipe::block_builder(const std::string& full_label, const Signal& sig_in, const Signal& sig_out)
{
    pooya_trace("block: " + full_name().str());
    std::shared_ptr<Block> block;
    if (dynamic_cast<ScalarSignalImpl*>(&sig_in.impl()))
    {
        block = std::make_shared<Pipe>();
    }
#ifdef POOYA_INT_SIGNAL
    else if (dynamic_cast<IntSignalImpl*>(&sig_in.impl()))
    {
        block = std::make_shared<PipeI>();
    }
#endif // POOYA_INT_SIGNAL
#ifdef POOYA_BOOL_SIGNAL
    else if (dynamic_cast<BoolSignalImpl*>(&sig_in.impl()))
    {
        block = std::make_shared<PipeB>();
    }
#endif // POOYA_BOOL_SIGNAL
#ifdef POOYA_ARRAY_SIGNAL
    else if (dynamic_cast<ArraySignalImpl*>(&sig_in.impl()))
    {
        block = std::make_shared<PipeA>();
    }
#endif // POOYA_ARRAY_SIGNAL
    else
    {
        pooya_verify(false, "cannot create a pipe block for a non-value signal.");
    }

    block->rename(full_label);
    _parent->add_block(*block, {sig_in}, {sig_out});
    _blocks.emplace_back(std::move(block));
}

} // namespace pooya
