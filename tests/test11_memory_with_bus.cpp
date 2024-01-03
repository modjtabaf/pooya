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
#include <math.h>
#include <vector>
#include <chrono>

#include "src/core/pooya.hpp"
#include "src/core/helper.hpp"
#include "src/core/solver.hpp"
#include "src/misc/gp-ios.hpp"

class BusMemory : public pooya::BusBlockBuilder
{
public:
    BusMemory(std::string given_name) : pooya::BusBlockBuilder(given_name) {}

protected:
    void block_builder(const std::string& /*path*/, const pooya::BusSpec::WireInfo& wi, pooya::Signal sig_in, pooya::Signal sig_out) override
    {
        if (wi._name == "x2")
        {
            // delay x2 by 2 steps
            auto z = _parent->signal();

            _blocks.push_back(new pooya::Memory("memory", 0.5));
            _parent->add_block(*_blocks.back(), sig_in, z);

            _blocks.push_back(new pooya::Memory("memory", 1.0));
            _parent->add_block(*_blocks.back(), z, sig_out);
        }
        else
        {
            _blocks.push_back(new pooya::Memory("memory"));
            _parent->add_block(*_blocks.back(), sig_in, sig_out);
        }
    }
};

int main()
{
    using milli = std::chrono::milliseconds;
    auto  start = std::chrono::high_resolution_clock::now();

    // create raw blocks
    pooya::Model   model("test11");
    BusMemory bus_memory("memory");

    pooya::BusSpec bus_spec({
        {"x1"}, // scalar
        {"x2"}, // scalar
        {"x3"}, // scalar
        });

    // create buses (signals)

    // create bus by specifying wire names (neither order nor count matters)
    auto x = model.bus("x", bus_spec,{
        {"x1", model.signal("x1")},
        {"x3", model.signal("x3")},
        {"x2", model.signal("x2")},
        });

    // create bus without specifying wire names (both order and count matter)
    auto y = model.bus("y", bus_spec, {
        model.signal("y1"),
        model.signal("y2"),
        model.signal("y3"),
        });

    // setup the model
    model.add_block(bus_memory, x, y);

    pooya::Simulator sim(model,
        [&](pooya::Model&, double t, pooya::Values& values) -> void
        {
            values.set(x->at("x1"), std::sin(M_PI * t / 3));
            values.set(x->at("x2"), std::sin(M_PI * t / 5));
            values.set(x->at("x3"), std::sin(M_PI * t / 7));
        });

    pooya::History history(model);

    uint k = 0;
    double t;
    while (pooya::arange(k, t, 0, 10, 0.5))
    {
        sim.run(t);
        history.update(k, t, sim.values());
        k++;
    }

    auto finish = std::chrono::high_resolution_clock::now();
    std::cout << "It took "
              << std::chrono::duration_cast<milli>(finish - start).count()
              << " milliseconds\n";

    history.shrink_to_fit();

    Gnuplot gp;
	gp << "set xrange [0:" << history.nrows() - 1 << "]\n";
    gp << "set yrange [-1:1]\n";
	gp << "plot"
        << gp.file1d(history[x->at("x1")]) << "with lines title 'x1',"
		<< gp.file1d(history[y->at("x1")]) << "with lines title 'y1',"
        << gp.file1d(history[x->at("x2")]) << "with lines title 'x2',"
		<< gp.file1d(history[y->at("x2")]) << "with lines title 'y2',"
        << gp.file1d(history[x->at("x3")]) << "with lines title 'x3',"
		<< gp.file1d(history[y->at("x3")]) << "with lines title 'y3',"
        "\n";

    return 0;
}
