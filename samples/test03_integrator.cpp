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

#include <chrono>
#include <cmath>
#include <iostream>

#include "src/block/submodel.hpp"
#include "src/block/triggered_integrator.hpp"
#include "src/helper/trace.hpp"
#include "src/misc/gp-ios.hpp"
#include "src/solver/euler.hpp"
#include "src/solver/history.hpp"
#include "src/solver/simulator.hpp"

int main()
{
    pooya_trace0;

    using milli = std::chrono::milliseconds;
    auto start  = std::chrono::high_resolution_clock::now();

    // create pooya blocks
    pooya::Submodel model("test03");
    pooya::TriggeredIntegrator integ("integ", &model, 1.0);

    // create pooya signals
    pooya::ScalarSignal x("x");
    pooya::ScalarSignal xd("xd");
    pooya::BoolSignal trigger("trigger");

    // setup the model
    model.add_block(integ, {{"in", xd}, {"trigger", trigger}}, x);

    pooya::Euler stepper;

    pooya::Simulator sim(
        model,
        [&](pooya::Block&, double t) -> void
        {
            pooya_trace0;
            xd      = t < 3 or t > 7 ? 1.0 : 0.0;
            trigger = t >= 4.9 && t <= 5.1;
        },
        &stepper);

    pooya::History history;
    history.track(x);
    history.track(xd);

    uint k = 0;
    double t;
    while (pooya::arange(k, t, 0, 10, 0.1))
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
    gp << "set yrange [-1:7]\n";
    gp << "plot" << gp.file1d(history[x]) << "with lines title 'x'," << gp.file1d(history[xd])
       << "with lines title 'xd'\n";

    assert(pooya::helper::pooya_trace_info.size() == 1);

    return 0;
}
