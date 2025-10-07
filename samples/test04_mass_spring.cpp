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
#include <math.h>

#include "src/block/extra/gain.hpp"
#include "src/block/integrator.hpp"
#include "src/block/submodel.hpp"
#include "src/helper/trace.hpp"
#include "src/misc/gp-ios.hpp"
#include "src/solver/history.hpp"
#include "src/solver/rk4.hpp"
#include "src/solver/simulator.hpp"

class MyModel : public pooya::Submodel
{
protected:
    pooya::Integrator _integ1{0.1, this, "xd"};
    pooya::Integrator _integ2{0.0, this, "x"};
    pooya::Gain _gain{-1.0 / 1.0, this, "-k\\m"};

public:
    pooya::ScalarSignal _x{"x"};
    pooya::ScalarSignal _xd{"xd"};
    pooya::ScalarSignal _xdd{"xdd"};

    MyModel() : pooya::Submodel(nullptr, "MyModel")
    {
        _integ1.connect({_xdd}, {_xd});
        _integ2.connect({_xd}, {_x});
        _gain.connect({_x}, {_xdd});
    }
};

int main()
{
    pooya_trace0;

    using milli = std::chrono::milliseconds;
    auto start  = std::chrono::high_resolution_clock::now();

    // create pooya blocks
    MyModel model;

    pooya::Rk4 stepper;
    pooya::Simulator sim(model, nullptr, &stepper);

    pooya::History history;
    history.track(model._x);
    history.track(model._xd);

    uint ind{0};
    for (double t = 0; t <= 5; t += 0.01)
    {
        sim.run(t);
        history.update(ind++, t);
    }

    auto finish = std::chrono::high_resolution_clock::now();
    std::cout << "It took " << std::chrono::duration_cast<milli>(finish - start).count() << " milliseconds\n";

    history.shrink_to_fit();
    history.export_csv("mass_spring.csv");

    Gnuplot gp;
    gp << "set xrange [0:" << history.nrows() - 1 << "]\n";
    gp << "set yrange [-0.15:0.15]\n";
    gp << "plot" << gp.file1d(history[model._x]) << "with lines title 'x'," << gp.file1d(history[model._xd])
       << "with lines title 'xd'\n";

    pooya_debug_verify0(pooya::helper::pooya_trace_info.size() == 1);

    return 0;
}
