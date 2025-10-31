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

#include "src/block/extra/divide.hpp"
#include "src/block/extra/function.hpp"
#include "src/block/extra/multiply.hpp"
#include "src/block/integrator.hpp"
// #include "src/block/extra/siso_function.hpp"
// #include "src/block/extra/so_function.hpp"
#include "src/block/extra/subtract.hpp"
#include "src/block/submodel.hpp"
#include "src/helper/trace.hpp"
#include "src/misc/gp-ios.hpp"
#include "src/solver/history.hpp"
#include "src/solver/rkf45.hpp"
#include "src/solver/simulator.hpp"

class Pendulum : public pooya::Submodel
{
protected:
    pooya::Integrator _integ1{M_PI_4, this, "dphi"};
    pooya::Integrator _integ2{0.0, this, "phi"};
    pooya::Function _sin{[](double /*t*/, const pooya::Bus& ibus, const pooya::Bus& obus) -> void
                         { pooya::ScalarSignal(obus.at(0)) = std::sin(pooya::ScalarSignal(ibus.at(0))); }, this,
                         "sin(phi)"};
    // pooya::SISOFunction _sin{[](double /*t*/, double x) -> double { return std::sin(x); }, this, "sin(phi)"};
    // pooya::SOFunction _sin{[](double /*t*/, const pooya::Bus& ibus) -> double
    //                        { return std::sin(pooya::ScalarSignal(ibus.at(0))); }, this, "sin(phi)"};
    pooya::Multiply _mul1{1.0, this, "g"};
    pooya::Divide _div1{this, "_l"};
    pooya::Multiply _mul2{1.0, this, "ml2"};
    pooya::Divide _div2{this, "_ml2"};
    pooya::Subtract _sub{this, "d2phi"};

    pooya::ScalarSignal _m{"m", 0.2};
    pooya::ScalarSignal _g{"g", 9.81};
    pooya::ScalarSignal _l{"l", 0.1};

public:
    pooya::ScalarSignal _tau{"tau"};
    pooya::ScalarSignal _phi{"phi"};
    pooya::ScalarSignal _dphi{"dphi"};
    pooya::ScalarSignal _d2phi{"d2phi"};

    Pendulum()
    {
        pooya_trace0;

        pooya::ScalarSignal s10("s10");
        pooya::ScalarSignal s15("s15");
        pooya::ScalarSignal s20("s20");
        pooya::ScalarSignal s25("s25");
        pooya::ScalarSignal s30("s30");

        // setup the submodel
        _integ1.connect({_d2phi}, {_dphi});
        _integ2.connect({_dphi}, {_phi});
        _sin.connect({_phi}, {s10});
        _mul1.connect({s10, _g}, {s15});
        _div1.connect({s15, _l}, {s20});
        _mul2.connect({_m, _l, _l}, {s25});
        _div2.connect({_tau, s25}, {s30});
        _sub.connect({s30, s20}, {_d2phi});
    }
};

int main()
{
    pooya_trace0;

    using milli = std::chrono::milliseconds;
    auto start  = std::chrono::high_resolution_clock::now();

    // create pooya blocks
    Pendulum pendulum;

    pooya::Rkf45 stepper;
    pooya::Simulator sim(
        pendulum,
        [&](pooya::Block&, double /*t*/) -> void
        {
            pooya_trace0;
            pendulum._tau = 0.13;
        },
        &stepper, true);

    pooya::History history;
    history.track(pendulum._phi);
    history.track(pendulum._dphi);

    uint ind{0};
    for (double t = 0; t <= 5; t += 0.1)
    {
        sim.run(t);
        history.update(ind++, t);
    }

    auto finish = std::chrono::high_resolution_clock::now();
    std::cout << "It took " << std::chrono::duration_cast<milli>(finish - start).count() << " milliseconds\n";

    history.shrink_to_fit();

    Gnuplot gp;
    gp << "set xrange [0:" << history.nrows() - 1 << "]\n";
    gp << "set yrange [-8:8]\n";
    gp << "plot" << gp.file1d(history[pendulum._phi]) << "with lines title 'phi'," << gp.file1d(history[pendulum._dphi])
       << "with lines title 'dphi'\n";

    pooya_debug_verify0(pooya::helper::pooya_trace_info.size() == 1);

    return 0;
}
