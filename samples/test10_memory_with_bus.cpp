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

#include <chrono>
#include <iostream>
#include <math.h>
#include <string>

#include "src/block/bus_memory.hpp"
#include "src/block/submodel.hpp"
#include "src/helper/trace.hpp"
#include "src/misc/gp-ios.hpp"
#include "src/solver/history.hpp"
#include "src/solver/simulator.hpp"

class MyBus : public pooya::Bus
{
public:
    MyBus(const std::string& name)
        : pooya::Bus( //
              name,   //
              {
                  {"x0", pooya::ScalarSignal("x0")},
                  {"x1", pooya::ScalarSignal("x1")},
                  {"Z", pooya::Bus( //
                            "Z",    //
                            {
                                {"z3", pooya::ScalarSignal("z3")},
                            })},
                  {"x2", pooya::ScalarSignal("x2")},
              })
    {
    }
};

int main()
{
    pooya_trace0;

    using milli = std::chrono::milliseconds;
    auto start  = std::chrono::high_resolution_clock::now();

    // create pooya blocks
    pooya::Submodel model(nullptr, "test10");
    pooya::BusMemory bus_memory(model, {{"Z.z3", 1.0}}, {"x1"});

    // create buses (signals)
    MyBus x("x");
    MyBus y("y");

    // setup the model
    bus_memory.connect(x, y);

    pooya::ScalarSignal x_x0 = x.scalar_at("x0");
    pooya::ScalarSignal x_x1 = x.scalar_at("x1");
    pooya::ScalarSignal x_x2 = x.scalar_at("x2");
    pooya::ScalarSignal x_z3 = x.scalar_at("Z.z3");

    pooya::ScalarSignal y_x0 = y.scalar_at("x0");
    pooya::ScalarSignal y_x1 = y.scalar_at("x1");
    pooya::ScalarSignal y_x2 = y.scalar_at("x2");
    pooya::ScalarSignal y_z3 = y.scalar_at("Z.z3");

    pooya::Simulator sim(model,
                         [&](pooya::Block&, double t) -> void
                         {
                             pooya_trace0;
                             x_x0 = std::sin(M_PI * t / 3);
                             x_x2 = std::sin(M_PI * t / 7);
                             x_z3 = std::sin(M_PI * t / 9);
                         });

    pooya::History history;
    history.track(x_x0);
    history.track(x_x1);
    history.track(x_x2);
    history.track(x_z3);
    history.track(y_x0);
    history.track(y_x1);
    history.track(y_x2);
    history.track(y_z3);

    uint k = 0;
    double t;
    while (pooya::arange(k, t, 0, 10, 0.5))
    {
        sim.run(t);
        history.update(k, t);
        k++;
    }

    auto finish = std::chrono::high_resolution_clock::now();
    std::cout << "It took " << std::chrono::duration_cast<milli>(finish - start).count() << " milliseconds\n";

    history.shrink_to_fit();

    Gnuplot gp;
    gp << "set xrange [0:" << history.nrows() - 1 << "]\n";
    gp << "set yrange [-1:1]\n";
    gp << "plot" << gp.file1d(history[x_x0]) << "with lines title 'x0'," << gp.file1d(history[y_x0])
       << "with lines title 'y0'," << gp.file1d(history[x_x1]) << "with lines title 'x1'," << gp.file1d(history[y_x1])
       << "with lines title 'y1'," << gp.file1d(history[x_x2]) << "with lines title 'x2'," << gp.file1d(history[y_x2])
       << "with lines title 'y2'," << gp.file1d(history[x_z3]) << "with lines title 'x3'," << gp.file1d(history[y_z3])
       << "with lines title 'y3'," << "\n";

    assert(pooya::helper::pooya_trace_info.size() == 1);

    return 0;
}
