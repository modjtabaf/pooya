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

#include <vector>
#include <memory>

#include "src/signal/array.hpp"
#include "src/signal/bus.hpp"
#include "singleio.hpp"

namespace pooya
{

class BusBlockBuilder : public SingleInputOutputT<BusSpec>
{
protected:
    std::vector<std::shared_ptr<Block>> _blocks;
    std::vector<std::string> _excluded_labels;

    void traverse_bus(const std::string& path_name, const BusSpec &bus_spec);

    virtual void block_builder(const std::string& path_name, const BusSpec::WireInfo &wi,
        SignalId sig_in, SignalId sig_out) = 0;

public:
    BusBlockBuilder(const std::string& given_name, const std::initializer_list<std::string>& excluded_labels={})
        : SingleInputOutputT<BusSpec>(given_name, false, 1, 1), _excluded_labels(excluded_labels) {}

    bool init(Parent* parent, BusId ibus, BusId obus) override;
    void post_init() override;
};

} // namespace pooya

#endif // __POOYA_BLOCK_BUS_BLOCK_BUILDER_HPP__
