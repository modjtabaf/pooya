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
#include "src/block/model.hpp"
#include "src/block/sin.hpp"
// #include "src/block/siso_function.hpp"
#include "src/block/muldiv.hpp"
#include "src/block/integrator.hpp"
#include "src/solver/rk4.hpp"
#include "src/solver/simulator.hpp"
#include "src/solver/history.hpp"
#include "src/misc/gp-ios.hpp"

class Pendulum : public pooya::Submodel
{
protected:
    pooya::Integrator _integ1{"dphi"};
    pooya::Integrator _integ2{"phi", M_PI_4};
    // pooya::SISOFunction _sin{"sin(phi)",
    //     [](double /*t*/, double x) -> double
    //     {
    //         return std::sin(x);
    //     }};
    pooya::Sin _sin{"sin(phi)"};
    pooya::MulDiv _muldiv{"-g\\l", "**/", -1};

public:
    Pendulum() : pooya::Submodel("pendulum") {}

    pooya::ScalarSignalId _phi{nullptr};
    pooya::ScalarSignalId _dphi{nullptr};
    pooya::ScalarSignalId _d2phi{nullptr};
    pooya::ScalarSignalId _g{nullptr};
    pooya::ScalarSignalId _l{nullptr};

    bool init(pooya::Parent& parent, pooya::BusId, pooya::BusId) override
    {
        pooya_trace0;

        if (!pooya::Submodel::init(parent))
            return false;

        // create pooya signals
        _phi   = pooya::ScalarSignalInfo::create_new("phi");
        _dphi  = pooya::ScalarSignalInfo::create_new("dphi");
        _d2phi = pooya::ScalarSignalInfo::create_new("d2phi");

        auto s10 = pooya::ScalarSignalInfo::create_new("");

        _g = pooya::ScalarSignalInfo::create_new("g");
        _l = pooya::ScalarSignalInfo::create_new("l");

        // setup the submodel
        add_block(_integ1, _d2phi, _dphi);
        add_block(_integ2, _dphi, _phi);
        add_block(_sin, _phi, s10);
        add_block(_muldiv, {s10, _g, _l}, _d2phi);

        return true;
    }
};

int main()
{
    pooya_trace0;

    using milli = std::chrono::milliseconds;
    auto start = std::chrono::high_resolution_clock::now();

    // create pooya blocks
    pooya::Model model("test05");
    Pendulum pendulum;

    // setup the model
    model.add_block(pendulum);

    pooya::Rk4 stepper;
    pooya::Simulator sim(model,
        [&](pooya::Model&, double /*t*/) -> void
        {
            pooya_trace0;
            pendulum._l->set(0.1);
            pendulum._g->set(9.81);
        },
        &stepper);

    pooya::History history(model);
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
    std::cout << "It took "
              << std::chrono::duration_cast<milli>(finish - start).count()
              << " milliseconds\n";

    // auto phi = model.lookup_signal("/pendulum~phi");

    history.shrink_to_fit();

    Gnuplot gp;
	gp << "set xrange [0:" << history.nrows() - 1 << "]\n";
    gp << "set yrange [-0.8:0.8]\n";
	gp << "plot" << gp.file1d(history[pendulum._phi]) << "with lines title 'x'\n";

    assert(pooya::helper::pooya_trace_info.size() == 1);

    return 0;
}
