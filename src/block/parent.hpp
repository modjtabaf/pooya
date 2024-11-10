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

#ifndef __POOYA_BLOCK_PARENT_HPP__
#define __POOYA_BLOCK_PARENT_HPP__

#include <memory>

#include "block.hpp"

namespace pooya
{

class Parent : public Block
{
protected:
#if defined(POOYA_USE_SMART_PTRS)
    std::vector<std::reference_wrapper<Block>> _components;
#else  // defined(POOYA_USE_SMART_PTRS)
    std::vector<Block*> _components;
#endif // defined(POOYA_USE_SMART_PTRS)
    std::vector<std::unique_ptr<BusSpec>> _interface_bus_specs;

    Parent(const std::string& given_name, uint16_t num_iports = NoIOLimit, uint16_t num_oports = NoIOLimit)
        : Block(given_name, false, num_iports, num_oports)
    {
    }

public:
    bool add_block(Block& component, const LabelSignals& iports = {}, const LabelSignals& oports = {});

    void pre_step(double t) override
    {
        pooya_trace("block: " + full_name());
#if defined(POOYA_USE_SMART_PTRS)
        for (auto& component : _components)
        {
            component.get().pre_step(t);
        }
#else  // defined(POOYA_USE_SMART_PTRS)
        for (auto* component : _components)
        {
            component->pre_step(t);
        }
#endif // defined(POOYA_USE_SMART_PTRS)
    }

    void post_step(double t) override
    {
        pooya_trace("block: " + full_name());
#if defined(POOYA_USE_SMART_PTRS)
        for (auto& component : _components)
        {
            component.get().post_step(t);
        }
#else  // defined(POOYA_USE_SMART_PTRS)
        for (auto* component : _components)
        {
            component->post_step(t);
        }
#endif // defined(POOYA_USE_SMART_PTRS)
    }

    void _mark_unprocessed() override;
    uint _process(double t, bool go_deep = true) override;
    bool traverse(TraverseCallback cb, uint32_t level,
                  uint32_t max_level = std::numeric_limits<uint32_t>::max()) override;
    bool const_traverse(ConstTraverseCallback cb, uint32_t level,
                        uint32_t max_level = std::numeric_limits<uint32_t>::max()) const override;
}; // class Parent

} // namespace pooya

#endif // __POOYA_BLOCK_PARENT_HPP__
