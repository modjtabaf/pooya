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
#include <cstddef>
#include <iostream>

#include "src/block/add.hpp"
#include "src/block/gain.hpp"
#include "src/block/integrator.hpp"
#include "src/block/muldiv.hpp"
#include "src/block/sin.hpp"
#include "src/block/submodel.hpp"
#include "src/block/subtract.hpp"
#include "src/helper/trace.hpp"
#include "src/misc/gp-ios.hpp"
#include "src/solver/history.hpp"
#include "src/solver/rkf45.hpp"
#include "src/solver/simulator.hpp"

class Pendulum : public pooya::Submodel
{
protected:
#if __cplusplus >= 202002L // C++20
    pooya::MulDiv _muldiv1{"*///", {.parent = this }};
    pooya::Subtract _sub{{.parent = this }};
    pooya::Integrator _integ1{{.parent = this }};
    pooya::Integrator _integ2{{.parent = this }};
    pooya::Sin _sin{{.parent = this }};
    pooya::MulDiv _muldiv2{"**/", {.parent = this }};
#else  // __cplusplus >= 202002L // C++20
    pooya::MulDiv _muldiv1{this, "", "*///"};
    pooya::Subtract _sub{this};
    pooya::Integrator _integ1{this};
    pooya::Integrator _integ2{this};
    pooya::Sin _sin{this};
    pooya::MulDiv _muldiv2{this, "", "**/"};
#endif // __cplusplus >= 202002L // C++20

public:
    pooya::ScalarSignal _m;
    pooya::ScalarSignal _g;
    pooya::ScalarSignal _l;

    explicit Pendulum() : pooya::Submodel(nullptr, "pendulum") {}

    bool connect(const pooya::Bus& ibus, const pooya::Bus& obus) override
    {
        pooya_trace0;

        if (!pooya::Submodel::connect(ibus, obus)) return false;

        // create pooya signals
        pooya::ScalarSignal dphi;
        pooya::ScalarSignal s10;
        pooya::ScalarSignal s20;
        pooya::ScalarSignal s30;
        pooya::ScalarSignal s40;

        auto tau = scalar_input_at(0);
        auto phi = scalar_output_at(0);

        // setup the submodel
        _muldiv1.connect({tau, _m, _l, _l}, s10);
        _sub.connect({s10, s20}, s30);
        _integ1.connect(s30, dphi);
        _integ2.connect(dphi, phi);
        _sin.connect(phi, s40);
        _muldiv2.connect({s40, _g, _l}, s20);

        return true;
    }
};

class PI : public pooya::Submodel
{
protected:
    pooya::Gain _gain_p;
    pooya::Integrator _integ;
    pooya::Gain _gain_i;
#if __cplusplus >= 202002L // C++20
    pooya::Add _add{{.parent = this }};
#else  // __cplusplus >= 202002L // C++20
    pooya::Add _add{this};
#endif // __cplusplus >= 202002L // C++20

public:
    PI(double Kp, double Ki, double x0 = 0.0)
#if __cplusplus >= 202002L // C++20
        : pooya::Submodel({.name = "PI"}), _gain_p(Kp, {.parent = this, .name = "Kp"}),
          _integ({.parent = this, .name = "ix", .ic = x0}), _gain_i(Ki, {.parent = this, .name = "Ki"})
#else  // __cplusplus >= 202002L // C++20
        : pooya::Submodel(nullptr, "PI"), _gain_p(this, "Kp", Kp), _integ(this, "ix", x0), _gain_i(this, "Ki", Ki)
#endif // __cplusplus >= 202002L // C++20
    {
    }

    bool connect(const pooya::Bus& ibus, const pooya::Bus& obus) override
    {
        pooya_trace0;

        if (!pooya::Submodel::connect(ibus, obus)) return false;

        pooya::ScalarSignal s10;
        pooya::ScalarSignal s20;
        pooya::ScalarSignal s30;

        auto x = scalar_input_at(0);
        auto y = scalar_output_at(0);

        // blocks
        _gain_p.connect(x, s10);
        _integ.connect(x, s20);
        _gain_i.connect(s20, s30);
        _add.connect({s10, s30}, y);

        return true;
    }
};

class PendulumWithPI : public pooya::Submodel
{
protected:
    pooya::Subtract _sub;
    PI _pi{40.0, 20.0};

public:
    Pendulum _pend;
    pooya::ScalarSignal _des_phi;
    pooya::ScalarSignal _phi;
    pooya::ScalarSignal _tau;
    pooya::ScalarSignal _err;

    PendulumWithPI()
    {
        add_block(_sub, {_des_phi, _phi}, _err);
        add_block(_pi, _err, _tau);
        add_block(_pend, _tau, _phi);
    }
};

int main()
{
    pooya_trace0;

    using milli = std::chrono::milliseconds;
    auto start  = std::chrono::high_resolution_clock::now();

    // create pooya blocks
    PendulumWithPI pendulum_with_pi;

    pooya::Rkf45 stepper;
    pooya::Simulator sim(
        pendulum_with_pi,
        [&](pooya::Block&, double /*t*/) -> void
        {
            pooya_trace0;
            pendulum_with_pi._pend._m = 0.2;
            pendulum_with_pi._pend._l = 0.1;
            pendulum_with_pi._pend._g = 9.81;
            pendulum_with_pi._des_phi = M_PI_4;
        },
        &stepper); // try Rk4 with h = 0.01 to see the difference

    pooya::History history;
    history.track(pendulum_with_pi._phi);

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
    gp << "set yrange [-50:150]\n";
    gp << "plot" << gp.file1d((history[pendulum_with_pi._phi] * (180 / M_PI)).eval())
       << "with lines title 'phi'"
          ","
          // << gp.file1d(history[sig_reg.lookup_signal(".dphi")]) << "with lines title 'dphi',"
          // << gp.file1d(history[sig_reg.lookup_signal(".tau")]) << "with lines title 'tau'"
          "\n";

    assert(pooya::helper::pooya_trace_info.size() == 1);

    return 0;
}
