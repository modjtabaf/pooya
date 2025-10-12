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

#include "src/helper/defs.hpp"

#if defined(POOYA_DEBUG)
#include <iostream>
#endif // defined(POOYA_DEBUG)

#include "simulator.hpp"
#include "src/block/block.hpp"
#include "src/helper/util.hpp"

namespace pooya
{

void Simulator::process_model(double t, bool call_pre_step, bool call_post_step)
{
    pooya_trace("t: " + std::to_string(t));

    if (_inputs_cb)
    {
        _inputs_cb(_model, t);
    }
    _model.input_cb(t);

    if (call_pre_step) _model.pre_step(t);

    _model._mark_unprocessed();

    if (_reuse_order)
    {
        _new_po->clear();

        bool any_processed;
        do
        {
            any_processed = false;
            for (auto* base : *_current_po)
            {
                if (base->processed())
                {
                    continue;
                }
                base->process(t, false);
                if (base->processed())
                {
                    _new_po->emplace_back(base);
                    any_processed = true;
                }
            }
        } while (any_processed);

        for (auto* base : *_current_po)
        {
            if (!base->processed())
            {
                _new_po->emplace_back(base);
            }
        }

        pooya_debug_verify0(_current_po->size() == _new_po->size());
        pooya_debug_verify0(_current_po->size() == _current_po->capacity());
        pooya_debug_verify0(_new_po->size() == _new_po->capacity());

        std::swap(_current_po, _new_po);
    }
    else
    {
        while (_model.process(t))
        {
        }
    }

#if defined(POOYA_DEBUG)
    std::vector<const Block*> unprocessed;

    auto find_unprocessed_cb = [&](const Block& c, uint32_t /*level*/) -> bool
    {
        if (!c.processed())
        {
            unprocessed.push_back(&c);
        }
        return true;
    };

    _model.visit(find_unprocessed_cb, 0);
    if (unprocessed.size())
    {
        std::cout << "\n-- unprocessed blocks detected:\n";
        for (const auto* c : unprocessed)
        {
            std::cout << "- " << c->full_name().str() << "\n";
            for (auto& sig_type : c->linked_signals())
            {
                if (((sig_type.second & Block::SignalLinkType::Required) == 0) || sig_type.first->assigned())
                {
                    continue;
                }
                std::cout << "  - " << sig_type.first->name().str() << "\n";
            }
        }
    }
#endif // defined(POOYA_DEBUG)

    if (call_post_step) _model.post_step(t);
}

void Simulator::init(double t0)
{
    pooya_trace("t0: " + std::to_string(t0));

    SimulatorBase::init(t0);

    if (_state_variables.size() > 0 && _reuse_order)
    {
        pooya_debug_verify0(_processing_order1.empty());
        pooya_debug_verify0(_processing_order2.empty());

        uint num_blocks     = 0;
        auto enum_blocks_cb = [&](Block& /*c*/, uint32_t /*level*/) -> bool
        {
            num_blocks++;
            return true;
        };
        _model.visit(enum_blocks_cb, 0);

        _processing_order1.reserve(num_blocks);
        _processing_order2.reserve(num_blocks);

        auto add_blocks_cb = [&](Block& c, uint32_t /*level*/) -> bool
        {
            _processing_order1.emplace_back(&c);
            return true;
        };
        _model.visit(add_blocks_cb, 0);

        _current_po = &_processing_order1;
        _new_po     = &_processing_order2;
    }

    process_model(t0, true, true);
}

} // namespace pooya
