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
#include "src/block/gain.hpp"
#include "src/block/add.hpp"
#include "src/block/sin.hpp"
#include "src/block/muldiv.hpp"
#include "src/block/subtract.hpp"
#include "src/block/integrator.hpp"
#include "src/block/derivative.hpp"
#include "src/solver/rkf45.hpp"
#include "src/solver/simulator.hpp"
#include "src/solver/history.hpp"
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

    pooya::ScalarSignalId _m{nullptr};
    pooya::ScalarSignalId _g{nullptr};
    pooya::ScalarSignalId _l{nullptr};
    pooya::ScalarSignalId _dphi{nullptr};

    bool init(pooya::Parent& parent, pooya::BusId ibus, pooya::BusId obus) override
    {
        pooya_trace0;

        if (!pooya::Submodel::init(parent, ibus, obus))
            return false;

        // create pooya signals
        _dphi = pooya::ScalarSignalInfo::create_new("dphi");

        // choose random names for these internal signals
        auto s10 = pooya::ScalarSignalInfo::create_new("");
        auto s20 = pooya::ScalarSignalInfo::create_new("");
        auto s30 = pooya::ScalarSignalInfo::create_new("");
        auto s40 = pooya::ScalarSignalInfo::create_new("");

        _m = pooya::ScalarSignalInfo::create_new("m");
        _g = pooya::ScalarSignalInfo::create_new("g");
        _l = pooya::ScalarSignalInfo::create_new("l");

        auto tau = scalar_input_at(0);
        auto phi = scalar_output_at(0);

        // setup the submodel
        add_block(_muldiv1, {tau, _m, _l, _l}, s10);
        add_block(_sub,  {s10, s20}, s30);
        add_block(_integ1, s30, _dphi);
        add_block(_integ2, _dphi, phi);
        add_block(_sin, phi,  s40);
        add_block(_muldiv2, {s40, _g, _l}, s20);

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
        auto s10 = pooya::ScalarSignalInfo::create_new("");
        auto s20 = pooya::ScalarSignalInfo::create_new("");
        auto s30 = pooya::ScalarSignalInfo::create_new("");
        auto s40 = pooya::ScalarSignalInfo::create_new("");
        auto s50 = pooya::ScalarSignalInfo::create_new("");

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

public:
    PendulumWithPID() : pooya::Submodel("pendulum_with_PID") {}

    Pendulum _pend;
    pooya::ScalarSignalId _des_phi{nullptr};
    pooya::ScalarSignalId _phi{nullptr};
    pooya::ScalarSignalId _tau{nullptr};
    pooya::ScalarSignalId _err{nullptr};

    bool init(pooya::Parent& parent, pooya::BusId, pooya::BusId) override
    {
        pooya_trace0;

        if (!pooya::Submodel::init(parent))
            return false;

        // signals
        _phi = pooya::ScalarSignalInfo::create_new("phi");
        _tau = pooya::ScalarSignalInfo::create_new("tau");
        _err = pooya::ScalarSignalInfo::create_new("err");
        _des_phi = pooya::ScalarSignalInfo::create_new("des_phi");

        // blocks
        add_block(_sub, {_des_phi, _phi}, _err);
        add_block(_pid, _err, _tau);
        add_block(_pend, _tau, _phi);

        return true;
    }
};

int main()
{
    pooya_trace0;

    using milli = std::chrono::milliseconds;
    auto  start = std::chrono::high_resolution_clock::now();

    // create pooya blocks
    pooya::Model model("test08");
    PendulumWithPID pendulum_with_pid;

    // setup the model
    model.add_block(pendulum_with_pid);

    pooya::Rkf45 stepper;
    pooya::Simulator sim(model,
        [&](pooya::Model&, double /*t*/) -> void
        {
            pendulum_with_pid._pend._m->set(0.2);
            pendulum_with_pid._pend._l->set(0.1);
            pendulum_with_pid._pend._g->set(9.81);
            pendulum_with_pid._des_phi->set(M_PI_4);
        },
        &stepper);

    pooya::History history(model);
    history.track(pendulum_with_pid._phi);
    history.track(pendulum_with_pid._pend._dphi);
    history.track(pendulum_with_pid._tau);

    uint k = 0;
    double t;
    while (pooya::arange(k, t, 0, 5, 0.02))
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
    gp << "set yrange [-80:80]\n";
	gp << "plot" << gp.file1d((history[pendulum_with_pid._phi] * (180/M_PI)).eval()) << "with lines title 'phi',"
	    << gp.file1d(history[pendulum_with_pid._pend._dphi]) << "with lines title 'dphi',"
	    << gp.file1d(history[pendulum_with_pid._tau]) << "with lines title 'tau'\n";

    assert(pooya::helper::pooya_trace_info.size() == 1);

    return 0;
}
