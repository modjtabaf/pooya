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

#include <iostream>
// #include <math.h>
#include <chrono>

#include "src/block/submodel.hpp"
#include "src/helper/trace.hpp"
// #include "src/block/siso_function.hpp"
// #include "src/block/so_function.hpp"
// #include "src/block/function.hpp"
#include "src/block/integrator.hpp"
#include "src/block/muldiv.hpp"
#include "src/block/sin.hpp"
#include "src/block/subtract.hpp"
#include "src/misc/gp-ios.hpp"
#include "src/solver/history.hpp"
#include "src/solver/rkf45.hpp"
#include "src/solver/simulator.hpp"

class Pendulum : public pooya::Submodel
{
protected:
    pooya::Integrator _integ1{"dphi", M_PI_4};
    pooya::Integrator _integ2{"phi"};
    // pooya::SISOFunction _sin{"sin_phi",
    //     [](double /*t*/, double x) -> double
    //     {
    //         return std::sin(x);
    //     }};
    // pooya::SOFunction _sin{"sin_phi",
    //     [](double /*t*/, const pooya::Bus& ibus) -> double
    //     {
    //         return std::sin(ibus.scalar_at(0)->get());
    //     }};
    // pooya::Function _sin{"sin_phi",
    //     [](double /*t*/, const pooya::Bus& ibus, const pooya::Bus& obus) -> void
    //     {
    //         obus.scalar_at(0)->set(std::sin(ibus.scalar_at(0)->get()));
    //     }};
    pooya::Sin _sin{"sin(phi)"};
    pooya::MulDiv _muldiv1{"g_l", "**/"};
    pooya::MulDiv _muldiv2{"tau_ml2", "*///"};
    pooya::Subtract _sub{"d2phi"};

public:
    Pendulum() : pooya::Submodel("pendulum") {}

    pooya::ScalarSignal _tau{"tau"};
    pooya::ScalarSignal _m{"m"};
    pooya::ScalarSignal _g{"g"};
    pooya::ScalarSignal _l{"l"};
    pooya::ScalarSignal _phi{"phi"};
    pooya::ScalarSignal _dphi{"dphi"};
    pooya::ScalarSignal _d2phi{"d2phi"};

    bool init(pooya::Submodel* parent, const pooya::Bus&, const pooya::Bus&) override
    {
        pooya_trace0;

        if (!pooya::Submodel::init(parent)) return false;

        pooya::ScalarSignal s10;
        pooya::ScalarSignal s20;
        pooya::ScalarSignal s30;

        // setup the submodel
        add_block(_integ1, _d2phi, _dphi);
        add_block(_integ2, _dphi, _phi);
        add_block(_sin, _phi, s10);
        add_block(_muldiv1, {s10, _g, _l}, s20);
        add_block(_muldiv2, {_tau, _m, _l, _l}, s30);
        add_block(_sub, {s30, s20}, _d2phi);

        return true;
    }
};

int main()
{
    pooya_trace0;

    using milli = std::chrono::milliseconds;
    auto start  = std::chrono::high_resolution_clock::now();

    // create pooya blocks
    pooya::Submodel model("test06");
    Pendulum pendulum;

    // setup the model
    model.add_block(pendulum);

    pooya::Rkf45 stepper;
    pooya::Simulator sim(
        model,
        [&](pooya::Block&, double /*t*/) -> void
        {
            pooya_trace0;
            pendulum._m   = 0.2;
            pendulum._l   = 0.1;
            pendulum._g   = 9.81;
            pendulum._tau = 0.13;
        },
        &stepper, true);

    pooya::History history(model);
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

    assert(pooya::helper::pooya_trace_info.size() == 1);

    return 0;
}
