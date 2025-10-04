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
#include <iostream>

#include "src/block/extra/const.hpp"
#include "src/block/extra/delay.hpp"
#include "src/block/extra/source.hpp"
#include "src/block/submodel.hpp"
#include "src/helper/trace.hpp"
#include "src/misc/gp-ios.hpp"
#include "src/solver/history.hpp"
#include "src/solver/simulator.hpp"

class MyModel : public pooya::Submodel
{
protected:
    pooya::Source _source{[](double t) -> double
                          {
                              pooya_trace0;
                              return std::sin(M_PI * t / 5);
                          },
                          this};
    pooya::Const _const1{2.7435, this};
    pooya::Const _const2{0.0, this};
    pooya::Delay _delay{10.0, this};

public:
    pooya::ScalarSignal _s_x;
    pooya::ScalarSignal _s_y;

public:
    MyModel()
    {
        pooya_trace0;

        // create pooya signals
        pooya::ScalarSignal time_delay;
        pooya::ScalarSignal initial;

        // setup the submodel
        _source.connect({}, {_s_x});
        _const1.connect({}, {time_delay});
        _const2.connect({}, {initial});
        _delay.connect({{"delay", time_delay}, {"in", _s_x}, {"initial", initial}}, {_s_y});
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
    pooya::History history;
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
    std::cout << "It took " << std::chrono::duration_cast<milli>(finish - start).count() << " milliseconds\n";

    history.shrink_to_fit();

    Gnuplot gp;
    gp << "set xrange [0:" << history.nrows() - 1 << "]\n";
    gp << "set yrange [-1:1]\n";
    gp << "plot" << gp.file1d(history[model._s_x]) << "with lines title 'x'," << gp.file1d(history[model._s_y])
       << "with lines title 'xd'\n";

    pooya_debug_verify0(pooya::helper::pooya_trace_info.size() == 1);

    return 0;
}
