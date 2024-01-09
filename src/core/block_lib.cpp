/*
Copyright 2023 Mojtaba (Moji) Fathi

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”),
to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <iostream>
#include "block_lib.hpp"

namespace pooya
{

template<>
void SinT<double>::activation_function(double /*t*/, Values& values)
{
    values.set<double>(_oports[0], std::sin(values.get<double>(_iports[0])));
}

BusBlockBuilder::~BusBlockBuilder()
{
    for (auto* p: _blocks)
        if (p)
            delete p;
}

bool BusBlockBuilder::init(Parent& parent, const Signals& iports, const Signals& oports)
{
    if (!Block::init(parent, iports, oports))
        return false;

    iports.bind(0, _x);
    oports.bind(0, _y);

    verify(_x->spec() == _y->spec(), "Bus specs don't match!");

    return true;
}

void BusBlockBuilder::post_init()
{
    const auto& bus_spec = _x->spec();
    _blocks.reserve(bus_spec.total_size());
    traverse_bus("", bus_spec);
}

void BusBlockBuilder::traverse_bus(const std::string& path_name, const BusSpec& bus_spec)
{
    for (const auto& wi: bus_spec._wires)
    {
        if (wi._bus)
            traverse_bus(path_name + wi._name + ".", *wi._bus);
        else
        {
            auto name = path_name + wi._name;
            block_builder(name, wi, _x->at(name), _y->at(name));
        }
    }
}

}
