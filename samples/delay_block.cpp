/*
Copyright 2024 Mojtaba (Moji) Fathi

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”),
to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <iostream>
#include <chrono>

#include "src/helper/trace.hpp"
#include "src/block/submodel.hpp"
#include "src/block/const.hpp"
#include "src/block/delay.hpp"
#include "src/block/source.hpp"
#include "src/solver/simulator.hpp"
#include "src/solver/history.hpp"
#include "src/misc/gp-ios.hpp"

class MyModel : public pooya::Submodel
{
protected:
    pooya::Source _source{"Source",
        [](double t) -> double
        {
            pooya_trace0;
            return std::sin(M_PI * t / 5);
        }};
    pooya::Const _const1{"TimeDelay", 2.7435};
    pooya::Const _const2{"Initial", 0.0};
    pooya::Delay _delay{"Delay"};

public:
    pooya::ScalarSignal _s_x{"x"};
    pooya::ScalarSignal _s_y{"y"};

public:
    MyModel() : pooya::Submodel("delay_block")
    {
        pooya_trace0;

        // create pooya signals
        pooya::ScalarSignal time_delay("time_delay");
        pooya::ScalarSignal initial("initial");

        // _s_x = pooya::ScalarSignalInfo::create_new("x");
        // _s_y = pooya::ScalarSignalInfo::create_new("y");

        // setup the submodel
        add_block(_source, {}, _s_x);
        add_block(_const1, {}, time_delay);
        add_block(_const2, {}, initial);
        add_block(_delay, {
            {"delay", time_delay},
            {"in", _s_x},
            {"initial", initial}}, _s_y);
    }
};

int main()
{
    pooya_trace0;

    using milli = std::chrono::milliseconds;
    auto start  = std::chrono::high_resolution_clock::now();

    // create the model
    MyModel model;

    pooya::Simulator sim(model);
    pooya::History history(model);
    history.track(model._s_x);
    history.track(model._s_y);

    uint k = 0;
    double t;
    while (pooya::arange(k, t, 0, 10, 0.1))
    {
        sim.run(t);
        history.update(k, t);
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
	gp << "plot" << gp.file1d(history[model._s_x]) << "with lines title 'x',"
		<< gp.file1d(history[model._s_y]) << "with lines title 'xd'\n";

    assert(pooya::helper::pooya_trace_info.size() == 1);

    return 0;
}
