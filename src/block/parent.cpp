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

#include "parent.hpp"
#include "src/signal/array_signal.hpp"

namespace pooya
{

void Parent::_mark_unprocessed()
{
    pooya_trace("block: " + full_name());
    Block::_mark_unprocessed();

#if defined(POOYA_USE_SMART_PTRS)
    for (auto& component : _components)
    {
        component.get()._mark_unprocessed();
    }
#else  // defined(POOYA_USE_SMART_PTRS)
    for (auto* component : _components)
    {
        component->_mark_unprocessed();
    }
#endif // defined(POOYA_USE_SMART_PTRS)
}

uint Parent::_process(double t, bool go_deep)
{
    pooya_trace("block: " + full_name());
    uint n_processed = 0;
    if (!_processed)
    {
        _processed = true;
        if (go_deep)
#if defined(POOYA_USE_SMART_PTRS)
            for (auto& component : _components)
            {
                n_processed += component.get()._process(t);
                if (not component.get().processed())
                {
                    _processed = false;
                }
            }
#else  // defined(POOYA_USE_SMART_PTRS)
            for (auto* component : _components)
            {
                n_processed += component->_process(t);
                if (not component->processed())
                {
                    _processed = false;
                }
            }
#endif // defined(POOYA_USE_SMART_PTRS)
    }

    return n_processed;
}

bool Parent::traverse(TraverseCallback cb, uint32_t level, decltype(level) max_level)
{
    pooya_trace("block: " + full_name());
    if (level > max_level)
    {
        return true;
    }

    if (!Block::traverse(cb, level, max_level))
    {
        return false;
    }

    if (level < max_level)
    {
#if defined(POOYA_USE_SMART_PTRS)
        for (auto& c : _components)
        {
            if (!c.get().traverse(cb, level + 1, max_level))
            {
                return false;
            }
        }
#else  // defined(POOYA_USE_SMART_PTRS)
        for (auto* c : _components)
        {
            if (!c->traverse(cb, level + 1, max_level))
            {
                return false;
            }
        }
#endif // defined(POOYA_USE_SMART_PTRS)
    }

    return true;
}

bool Parent::const_traverse(ConstTraverseCallback cb, uint32_t level, decltype(level) max_level) const
{
    pooya_trace("block: " + full_name());
    if (level > max_level)
    {
        return true;
    }

    if (!Block::const_traverse(cb, level, max_level))
    {
        return false;
    }

    if (level < max_level)
    {
#if defined(POOYA_USE_SMART_PTRS)
        for (auto& c : _components)
        {
            if (!c.get().const_traverse(cb, level + 1, max_level))
            {
                return false;
            }
        }
#else  // defined(POOYA_USE_SMART_PTRS)
        for (auto* c : _components)
        {
            if (!c->const_traverse(cb, level + 1, max_level))
            {
                return false;
            }
        }
#endif // defined(POOYA_USE_SMART_PTRS)
    }

    return true;
}

bool Parent::add_block(Block& component, const LabelSignals& iports, const LabelSignals& oports)
{
    pooya_trace("block: " + full_name());

    if (!_initialized && !init())
    {
        return false;
    }

    auto make_bus = [&](const LabelSignals& ports) -> BusId
    {
        std::vector<BusSpec::WireInfo> wire_infos;
        LabelSignalIdList wires;
        for (const auto& ls : ports)
        {
            pooya_verify_valid_signal(ls.second);
            if (ls.second->is_scalar())
            {
                wire_infos.emplace_back(ls.first);
            }
            else if (ls.second->is_int())
            {
                wire_infos.emplace_back("i:" + ls.first);
            }
            else if (ls.second->is_bool())
            {
                wire_infos.emplace_back("b:" + ls.first);
            }
            else if (ls.second->is_array())
            {
                wire_infos.emplace_back(ls.first, ls.second->as_array().size());
            }
            else if (ls.second->is_bus())
            {
                wire_infos.emplace_back(ls.first, ls.second->as_bus()._spec);
            }
            else
            {
                pooya_verify(false, "unknown signal type!");
            }

            wires.emplace_back(ls.first, ls.second);
        }
        _interface_bus_specs.emplace_back(std::make_unique<BusSpec>(wire_infos.begin(), wire_infos.end()));
        return BusInfo::create_new("", *_interface_bus_specs.back(), wires.begin(), wires.end());
    };

    if (!component.init(this, make_bus(iports), make_bus(oports)))
    {
        return false;
    }

#if defined(POOYA_USE_SMART_PTRS)
    _components.push_back(component);
#else  // defined(POOYA_USE_SMART_PTRS)
    _components.push_back(&component);
#endif // defined(POOYA_USE_SMART_PTRS)
    component.post_init();
    return true;
}

} // namespace pooya
