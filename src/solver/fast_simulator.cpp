/*
Copyright 2025 Mojtaba (Moji) Fathi

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

#include "fast_simulator.hpp"
#include "src/block/leaf.hpp"
#include "src/helper/util.hpp"

namespace pooya
{

void FastSimulator::process_model(double t, bool call_pre_step, bool call_post_step)
{
    pooya_trace("t: " + std::to_string(t));

    if (_inputs_cb)
    {
        _inputs_cb(_model, t);
    }
    _model.input_cb(t);

    if (call_pre_step) _model.pre_step(t);

    for (auto& list : _processing_order)
        for (auto* leaf : list) leaf->activation_function(t);

    if (call_post_step) _model.post_step(t);
}

void FastSimulator::init(double t0)
{
    pooya_trace("t0: " + std::to_string(t0));

    SimulatorBase::init(t0);

    pooya_debug_verify0(_processing_order.empty());

    uint num_blocks     = 0;
    auto enum_blocks_cb = [&](Block& c, uint32_t /*level*/) -> bool
    {
        if (dynamic_cast<Leaf*>(&c)) num_blocks++;
        return true;
    };
    _model.visit(enum_blocks_cb, 0);

    std::vector<Leaf*> po;
    po.reserve(num_blocks);
    _processing_order.reserve(num_blocks);

    auto add_blocks_cb = [&po](Block& c, uint32_t /*level*/) -> bool
    {
        if (auto* p = dynamic_cast<Leaf*>(&c)) po.push_back(p);
        return true;
    };
    _model.visit(add_blocks_cb, 0);

    if (_inputs_cb)
    {
        _inputs_cb(_model, t0);
    }
    _model.input_cb(t0);
    _model.pre_step(t0);

    uint num_blocks_added = 0;
    for (;;)
    {
        uint num_blocks_to_add = 0;
        for (auto* leaf : po)
        {
            if (!leaf->processed() && leaf->ready_to_process()) ++num_blocks_to_add;
        }

        if (num_blocks_to_add == 0) break;

        auto& list = _processing_order.emplace_back();
        list.reserve(num_blocks_to_add);

        for (auto* leaf : po)
        {
            if (list.size() == num_blocks_to_add) break;

            if (leaf->processed() || !leaf->ready_to_process()) continue;

            leaf->process(t0, false);

            if (leaf->processed())
                list.push_back(leaf);
            else
                break;
        }

        if (list.size() != num_blocks_to_add) break;

        list.shrink_to_fit();

        num_blocks_added += num_blocks_to_add;
    }

    pooya_verify(num_blocks_added == num_blocks,
                 "The FastSimulator does not seem the right choice for the given model. Try Simulator instead.");

    _processing_order.shrink_to_fit();

    for (auto& sig : value_signals_) sig->clear();

    process_model(t0, true, true);
}

} // namespace pooya
