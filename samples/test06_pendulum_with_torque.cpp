/*
Copyright 2023 Mojtaba (Moji) Fathi

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

#include "src/core/pooya.hpp"
#include "src/core/helper.hpp"
#include "src/core/solver.hpp"
#include "src/misc/gp-ios.hpp"

class Pendulum : public pooya::Submodel
{
protected:
    pooya::Integrator _integ1{"dphi", M_PI_4};
    pooya::Integrator _integ2{"phi"};
    pooya::SISOFunction _sin{"sin_phi",
        [](double /*t*/, double x) -> double
        {
            return std::sin(x);
        }};
    // pooya::SOFunction _sin{"sin_phi",
    //     [](double /*t*/, pooya::BusId ibus) -> double
    //     {
    //         return std::sin(ibus->scalar_at(0)->get());
    //     }};
    // pooya::Function _sin{"sin_phi",
    //     [](double /*t*/, pooya::BusId ibus, pooya::BusId obus) -> void
    //     {
    //         obus->scalar_at(0)->set(std::sin(ibus->scalar_at(0)->get()));
    //     }};
    // pooya::Sin _sin{"sin(phi)"};
    pooya::MulDiv _muldiv1{"g_l", "**/"};
    pooya::MulDiv _muldiv2{"tau_ml2", "*///"};
    pooya::Subtract _sub{"d2phi"};

public:
    Pendulum() : pooya::Submodel("pendulum") {}

    pooya::ScalarSignalId _tau{nullptr};
    pooya::ScalarSignalId _m{nullptr};
    pooya::ScalarSignalId _g{nullptr};
    pooya::ScalarSignalId _l{nullptr};

    bool init(pooya::Parent& parent, pooya::BusId, pooya::BusId) override
    {
        pooya_trace0;

        if (!pooya::Submodel::init(parent))
            return false;

        // create pooya signals
        auto phi   = create_scalar_signal("phi");
        auto dphi  = create_scalar_signal("dphi");
        auto d2phi = create_scalar_signal("d2phi");

        // choose random names for these internal signals
        auto s10 = create_scalar_signal();
        auto s20 = create_scalar_signal();
        auto s30 = create_scalar_signal();

        auto& model_ = model_ref();
        _tau = model_.create_scalar_signal("tau");
        _m = model_.create_scalar_signal("m");
        _g = model_.create_scalar_signal("g");
        _l = model_.create_scalar_signal("l");

        // setup the submodel
        add_block(_integ1, d2phi, dphi);
        add_block(_integ2, dphi, phi);
        add_block(_sin, phi, s10);
        add_block(_muldiv1, {s10, _g, _l}, s20);
        add_block(_muldiv2, {_tau, _m, _l, _l}, s30);
        add_block(_sub, {s30, s20}, d2phi);

        return true;
    }
};

int main()
{
    pooya_trace0;

    using milli = std::chrono::milliseconds;
    auto  start = std::chrono::high_resolution_clock::now();

    // create pooya blocks
    pooya::Model model("test06");
    Pendulum pendulum;

    // setup the model
    model.add_block(pendulum);

    pooya::Rkf45 stepper;
    pooya::Simulator sim(model,
        [&](pooya::Model&, double /*t*/) -> void
        {
            pooya_trace0;
            pendulum._m->set(0.2);
            pendulum._l->set(0.1);
            pendulum._g->set(9.81);
            pendulum._tau->set(0.13);
        },
        &stepper, true);

    pooya::History history(model);

    uint k = 0;
    double t;
    while (pooya::arange(k, t, 0, 5, 0.1))
    {
        sim.run(t);
        history.update(k, t);
        k++;
    }

    auto finish = std::chrono::high_resolution_clock::now();
    std::cout << "It took "
              << std::chrono::duration_cast<milli>(finish - start).count()
              << " milliseconds\n";

    auto  phi = model.lookup_signal("/pendulum~phi");
    auto dphi = model.lookup_signal("~dphi");

    history.shrink_to_fit();

    Gnuplot gp;
	gp << "set xrange [0:" << history.nrows() - 1 << "]\n";
    gp << "set yrange [-8:8]\n";
	gp << "plot" << gp.file1d(history[phi]) << "with lines title 'phi',"
	    << gp.file1d(history[dphi]) << "with lines title 'dphi'\n";

    assert(pooya::util::pooya_trace_info.size() == 1);

    return 0;
}
