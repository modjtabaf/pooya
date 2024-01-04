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
        {"x0"}, // wire 0 called "x1" carries a scalar signal
        {"x1"}, // wire 1 called "x2" carries a scalar signal
        {"x2"}, // wire 2 called "x3" carries a scalar signal
        });

    // create buses (signals)

    auto x = model.bus("x", bus_spec); // assign bus wires implicitely
    /*
    // alternative 1
    auto x = model.bus("x", bus_spec, { // assign bus wires explicitely without specifying wire labels (order matters)
        model.signal("x0"),
        model.signal("x1"),
        model.signal("x2"),
    });
    */
    /*
    // alternative 2
    auto x = model.bus("x", bus_spec, { // assign bus wires explicitely while specifying wire labels (order does not matter)
        {"x2", model.signal("x2"),},
        {"x0", model.signal("x0"),},
        {"x1", model.signal("x1"),},
    });
    */

    auto y = model.bus("y", bus_spec);

    // setup the model
    model.add_block(bus_memory, x, y);

    auto x_x0 = x->at("x0");
    auto x_x1 = x->at("x1");
    auto x_x2 = x->at("x2");

    pooya::Simulator sim(model,
        [&](pooya::Model&, double t, pooya::Values& values) -> void
        {
            values.set(x_x0, std::sin(M_PI * t / 3));
            values.set(x_x1, std::sin(M_PI * t / 5));
            values.set(x_x2, std::sin(M_PI * t / 7));
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

    auto y_x0 = y->at("x0");
    auto y_x1 = y->at("x1");
    auto y_x2 = y->at("x2");

    Gnuplot gp;
	gp << "set xrange [0:" << history.nrows() - 1 << "]\n";
    gp << "set yrange [-1:1]\n";
	gp << "plot"
        << gp.file1d(history[x_x0]) << "with lines title 'x0',"
		<< gp.file1d(history[y_x0]) << "with lines title 'y0',"
        << gp.file1d(history[x_x1]) << "with lines title 'x1',"
		<< gp.file1d(history[y_x1]) << "with lines title 'y1',"
        << gp.file1d(history[x_x2]) << "with lines title 'x2',"
		<< gp.file1d(history[y_x2]) << "with lines title 'y2',"
        "\n";

    return 0;
}
