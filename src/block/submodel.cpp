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

#include "submodel.hpp"
#include "src/signal/array_signal.hpp"

namespace pooya
{

void Submodel::_mark_unprocessed()
{
    pooya_trace("block: " + full_name().str());
    Block::_mark_unprocessed();

#if defined(POOYA_USE_SMART_PTRS)
    for (auto& block : _blocks)
    {
        block.get()._mark_unprocessed();
    }
#else  // defined(POOYA_USE_SMART_PTRS)
    for (auto* block : _blocks)
    {
        block->_mark_unprocessed();
    }
#endif // defined(POOYA_USE_SMART_PTRS)
}

uint Submodel::_process(double t, bool go_deep)
{
    pooya_trace("block: " + full_name().str());
    uint n_processed = 0;
    if (!_processed)
    {
        _processed = true;
        if (go_deep)
#if defined(POOYA_USE_SMART_PTRS)
            for (auto& block : _blocks)
            {
                n_processed += block.get()._process(t);
                if (not block.get().processed())
                {
                    _processed = false;
                }
            }
#else  // defined(POOYA_USE_SMART_PTRS)
            for (auto* block : _blocks)
            {
                n_processed += block->_process(t);
                if (not block->processed())
                {
                    _processed = false;
                }
            }
#endif // defined(POOYA_USE_SMART_PTRS)
    }

    return n_processed;
}

bool Submodel::visit(VisitorCallback cb, uint32_t level, decltype(level) max_level)
{
    pooya_trace("block: " + full_name().str());
    if (level > max_level)
    {
        return true;
    }

    if (!Block::visit(cb, level, max_level))
    {
        return false;
    }

    if (level < max_level)
    {
#if defined(POOYA_USE_SMART_PTRS)
        for (auto& c : _blocks)
        {
            if (!c.get().visit(cb, level + 1, max_level))
            {
                return false;
            }
        }
#else  // defined(POOYA_USE_SMART_PTRS)
        for (auto* c : _blocks)
        {
            if (!c->visit(cb, level + 1, max_level))
            {
                return false;
            }
        }
#endif // defined(POOYA_USE_SMART_PTRS)
    }

    return true;
}

bool Submodel::const_visit(ConstVisitorCallback cb, uint32_t level, decltype(level) max_level) const
{
    pooya_trace("block: " + full_name().str());
    if (level > max_level)
    {
        return true;
    }

    if (!Block::const_visit(cb, level, max_level))
    {
        return false;
    }

    if (level < max_level)
    {
#if defined(POOYA_USE_SMART_PTRS)
        for (auto& c : _blocks)
        {
            if (!c.get().const_visit(cb, level + 1, max_level))
            {
                return false;
            }
        }
#else  // defined(POOYA_USE_SMART_PTRS)
        for (auto* c : _blocks)
        {
            if (!c->const_visit(cb, level + 1, max_level))
            {
                return false;
            }
        }
#endif // defined(POOYA_USE_SMART_PTRS)
    }

    return true;
}

bool Submodel::add_block(Block& block)
{
    pooya_trace("block: " + full_name().str());

    if (block.parent() != this)
    {
        helper::pooya_show_warning(__FILE__, __LINE__, block.full_name().str() + " is not my child!");
        return false;
    }

#if defined(POOYA_USE_SMART_PTRS)
    _blocks.push_back(block);
#else  // defined(POOYA_USE_SMART_PTRS)
    _blocks.push_back(&block);
#endif // defined(POOYA_USE_SMART_PTRS)

    return true;
}

} // namespace pooya
