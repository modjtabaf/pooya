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

#include "src/block/divide.hpp"
#include "src/block/integrator.hpp"
#include "src/block/multiply.hpp"
#include "src/block/sin.hpp"
// #include "src/block/siso_function.hpp"
#include "src/block/submodel.hpp"
#include "src/helper/trace.hpp"
#include "src/misc/gp-ios.hpp"
#include "src/solver/history.hpp"
#include "src/solver/rk4.hpp"
#include "src/solver/simulator.hpp"

class Pendulum : public pooya::Submodel
{
protected:
    pooya::Integrator _integ1{this};
    pooya::Integrator _integ2{this, M_PI_4};
    // pooya::SISOFunction _sin(this, [](double /*t*/, double x) -> double { return std::sin(x); });
    pooya::Sin _sin{this};
    pooya::Multiply _mul{this, -1};
    pooya::Divide _div{this};

public:
    pooya::ScalarSignal _phi{"phi"};
    pooya::ScalarSignal _dphi{"dphi"};
    pooya::ScalarSignal _d2phi{"d2phi"};
    pooya::ScalarSignal _g{"g"};
    pooya::ScalarSignal _l{"l"};

    Pendulum()
    {
        pooya_trace0;

        _integ1.rename("dphi");
        _integ2.rename("phi");
        _sin.rename("sin(phi)");
        _mul.rename("-g");
        _div.rename("_l");

        pooya::ScalarSignal s10;
        pooya::ScalarSignal s20;

        // setup the submodel
        _integ1.connect({_d2phi}, {_dphi});
        _integ2.connect({_dphi}, {_phi});
        _sin.connect({_phi}, {s10});
        _mul.connect({s10, _g}, {s20});
        _div.connect({s20, _l}, {_d2phi});
    }
};

int main()
{
    pooya_trace0;

    using milli = std::chrono::milliseconds;
    auto start  = std::chrono::high_resolution_clock::now();

    // create pooya blocks
    Pendulum pendulum;

    pooya::Rk4 stepper;
    pooya::Simulator sim(
        pendulum,
        [&](pooya::Block&, double /*t*/) -> void
        {
            pooya_trace0;
            *pendulum._l = 0.1;
            *pendulum._g = 9.81;
        },
        &stepper);

    pooya::History history;
    history.track(pendulum._phi);

    uint k = 0;
    double t;
    while (pooya::arange(k, t, 0, 5, 0.01))
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
    gp << "set yrange [-0.8:0.8]\n";
    gp << "plot" << gp.file1d(history[pendulum._phi]) << "with lines title 'x'\n";

    assert(pooya::helper::pooya_trace_info.size() == 1);

    return 0;
}
