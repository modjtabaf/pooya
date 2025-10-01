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
// #include "src/block/extra/function.hpp"
#include "src/block/extra/multiply.hpp"
#include "src/block/extra/sin.hpp"
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
    pooya::Integrator _integ1{this, M_PI_4};
    pooya::Integrator _integ2{this};
    // pooya::SISOFunction _sin{this, [](double /*t*/, double x) -> double { return std::sin(x); }};
    // pooya::SOFunction _sin{this, [](double /*t*/, const pooya::Bus& ibus) -> double
    //                        { return std::sin(pooya::ScalarSignal(ibus.at(0))); }};
    // pooya::Function _sin{this, [](double /*t*/, const pooya::Bus& ibus, const pooya::Bus& obus) -> void
    //                      { pooya::ScalarSignal(obus.at(0)) = std::sin(pooya::ScalarSignal(ibus.at(0))); }};
    pooya::Sin _sin{this};
    pooya::Multiply _mul1{this};
    pooya::Divide _div1{this};
    pooya::Multiply _mul2{this};
    pooya::Divide _div2{this};
    pooya::Subtract _sub{this};

public:
    pooya::ScalarSignal _tau{"tau"};
    pooya::ScalarSignal _m{"m"};
    pooya::ScalarSignal _g{"g"};
    pooya::ScalarSignal _l{"l"};
    pooya::ScalarSignal _phi{"phi"};
    pooya::ScalarSignal _dphi{"dphi"};
    pooya::ScalarSignal _d2phi{"d2phi"};

    Pendulum()
    {
        pooya_trace0;

        _integ1.rename("dphi");
        _integ2.rename("phi");
        _sin.rename("sin(phi)");
        _mul1.rename("g");
        _div1.rename("_l");
        _mul2.rename("ml2");
        _div2.rename("_ml2");
        _sub.rename("d2phi");

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
            pendulum._m   = 0.2;
            pendulum._l   = 0.1;
            pendulum._g   = 9.81;
            pendulum._tau = 0.13;
        },
        &stepper, true);

    pooya::History history;
    history.track(pendulum._phi);
    history.track(pendulum._dphi);

    uint k = 0;
    double t;
    while (pooya::arange(k, t, 0, 5, 0.1))
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
    gp << "set yrange [-8:8]\n";
    gp << "plot" << gp.file1d(history[pendulum._phi]) << "with lines title 'phi'," << gp.file1d(history[pendulum._dphi])
       << "with lines title 'dphi'\n";

    pooya_debug_verify0(pooya::helper::pooya_trace_info.size() == 1);

    return 0;
}
