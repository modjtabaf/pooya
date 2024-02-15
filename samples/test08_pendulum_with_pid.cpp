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
#include <math.h>
#include <vector>
#include <chrono>

#include "src/core/pooya.hpp"
#include "src/core/helper.hpp"
#include "src/core/solver.hpp"
#include "src/misc/gp-ios.hpp"

class Pendulum : public pooya::Submodel
{
protected:
    pooya::MulDiv _muldiv1{ "tau_ml2", "*///"};
    pooya::Subtract _sub{"err"};
    pooya::Integrator _integ1{"dphi"};
    pooya::Integrator _integ2{"phi"};
    pooya::Sin _sin{"sin(phi)"};
    pooya::MulDiv _muldiv2{"g_l", "**/"};

public:
    Pendulum() : pooya::Submodel("pendulum") {}

    bool init(pooya::Parent& parent, pooya::BusId ibus, pooya::BusId obus) override
    {
        pooya_trace0;

        if (!pooya::Submodel::init(parent, ibus, obus))
            return false;

        // create signals
        auto dphi = scalar_signal("dphi");

        // choose random names for these internal signals
        auto s10 = scalar_signal();
        auto s20 = scalar_signal();
        auto s30 = scalar_signal();
        auto s40 = scalar_signal();

        auto& model_ = model_ref();
        auto m = model_.scalar_signal("m");
        auto g = model_.scalar_signal("g");
        auto l = model_.scalar_signal("l");

        auto tau = scalar_input_at(0);
        auto phi = scalar_output_at(0);

        // setup the submodel
        add_block(_muldiv1, {tau, m, l, l}, s10);
        add_block(_sub,  {s10, s20}, s30);
        add_block(_integ1, s30, dphi);
        add_block(_integ2, dphi, phi);
        add_block(_sin, phi,  s40);
        add_block(_muldiv2, {s40, g, l}, s20);

        return true;
    }
};

class PID : public pooya::Submodel
{
protected:
    pooya::Gain _gain_p;
    pooya::Integrator _integ;
    pooya::Gain _gain_i;
    pooya::Add _add{"Add"};
    pooya::Derivative _deriv{"dx"};
    pooya::Gain _gain_d;

public:
    PID(double Kp, double Ki, double Kd, double x0=0.0) :
        pooya::Submodel("PI"),
        _gain_p("Kp", Kp),
        _integ("ix", x0),
        _gain_i("Ki", Ki),
        _gain_d("Kd", Kd) {}

    bool init(pooya::Parent& parent, pooya::BusId ibus, pooya::BusId obus) override
    {
        pooya_trace0;

        if (!pooya::Submodel::init(parent, ibus, obus))
            return false;

        // choose random names for these internal signals
        auto s10 = create_scalar_signal();
        auto s20 = create_scalar_signal();
        auto s30 = create_scalar_signal();
        auto s40 = create_scalar_signal();
        auto s50 = create_scalar_signal();

        auto x = scalar_input_at(0);
        auto y = scalar_output_at(0);

        // blocks
        add_block(_gain_p, x, s10);
        add_block(_integ, x, s20);
        add_block(_gain_i, s20, s30);
        add_block(_deriv, x, s40);
        add_block(_gain_d, s40, s50);
        add_block(_add, {s10, s30, s50}, y);

        return true;
    }
};

class PendulumWithPID : public pooya::Submodel
{
protected:
    pooya::Subtract _sub{"Sub"};
    PID _pid{40.0, 20.0, 0.05};
    Pendulum _pend;

public:
    PendulumWithPID() : pooya::Submodel("pendulum_with_PID") {}

    bool init(pooya::Parent& parent, pooya::BusId, pooya::BusId) override
    {
        pooya_trace0;

        if (!pooya::Submodel::init(parent))
            return false;

        // signals
        auto phi = scalar_signal("phi");
        auto tau = scalar_signal("tau");
        auto err = scalar_signal("err");

        auto des_phi = model_ref().scalar_signal("des_phi");

        // blocks
        add_block(_sub, {des_phi, phi}, err);
        add_block(_pid, err, tau);
        add_block(_pend, tau, phi);

        return true;
    }
};

int main()
{
    pooya_trace0;

    using milli = std::chrono::milliseconds;
    auto  start = std::chrono::high_resolution_clock::now();

    // create raw blocks
    pooya::Model model("test08");
    PendulumWithPID pendulum_with_pid;

    // setup the model
    model.add_block(pendulum_with_pid);

    auto m = model.scalar_signal("m");
    auto l = model.scalar_signal("l");
    auto g = model.scalar_signal("g");
    auto des_phi = model.scalar_signal("des_phi");

    pooya::Rkf45 stepper(model);
    pooya::Simulator sim(model,
        [&](pooya::Model&, double /*t*/, pooya::Values& values) -> void
        {
            values.set(m, 0.2);
            values.set(l, 0.1);
            values.set(g, 9.81);
            values.set(des_phi, M_PI_4);
        },
        &stepper);

    pooya::History history(model);

    uint k = 0;
    double t;
    while (pooya::arange(k, t, 0, 5, 0.02))
    {
        sim.run(t);
        history.update(k, t, sim.values());
        k++;
    }

    auto finish = std::chrono::high_resolution_clock::now();
    std::cout << "It took "
              << std::chrono::duration_cast<milli>(finish - start).count()
              << " milliseconds\n";

    auto  phi = model.lookup_signal("/test08/pendulum_with_PID~phi", true); // find using the exact name
    auto dphi = model.lookup_signal("~dphi"); // find using the partial name
    auto  tau = model.lookup_signal("tau");   // find using the partial name

    history.shrink_to_fit();

    Gnuplot gp;
	gp << "set xrange [0:" << history.nrows() - 1 << "]\n";
    gp << "set yrange [-80:80]\n";
	gp << "plot" << gp.file1d((history[phi] * (180/M_PI)).eval()) << "with lines title 'phi',"
	    << gp.file1d(history[dphi]) << "with lines title 'dphi',"
	    << gp.file1d(history[tau]) << "with lines title 'tau'\n";

    assert(pooya::util::pooya_trace_info.size() == 1);

    return 0;
}
