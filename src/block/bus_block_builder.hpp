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

#ifndef __POOYA_BLOCK_BUS_BLOCK_BUILDER_HPP__
#define __POOYA_BLOCK_BUS_BLOCK_BUILDER_HPP__

#include <memory>
#include <vector>

#include "leaf.hpp"
#include "src/signal/array.hpp"
#include "src/signal/bus.hpp"

namespace pooya
{

class BusBlockBuilder : public Leaf
{
protected:
    std::vector<std::shared_ptr<Block>> _blocks;
    std::vector<std::string> _excluded_labels;

    void visit_bus(const std::string& path_name, const Bus& bus);

    virtual void block_builder(std::string_view path_name, SignalImpl& sig_in, SignalImpl& sig_out) = 0;

public:
    explicit BusBlockBuilder(Submodel& parent, std::initializer_list<std::string> excluded_labels = {})
        : Leaf(&parent), _excluded_labels(excluded_labels)
    {
    }

    bool connect(const Bus& ibus, const Bus& obus) override;
    void _mark_unprocessed() override;
};

} // namespace pooya

#endif // __POOYA_BLOCK_BUS_BLOCK_BUILDER_HPP__
