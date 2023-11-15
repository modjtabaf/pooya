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

using namespace pooya;

class MassSpringDamper : public Base
{
protected:
    double _m;
    double _k;
    double _c;
    double _x0;
    double _xd0;
    Signal _tau;
    Signal _x;
    Signal _xd;
    Signal _xdd;

public:
    MassSpringDamper(Parent& parent, std::string given_name, const Signal& tau,
        double m, double k, double c, double x0, double xd0) :
        Base(&parent, given_name, tau), _m(m), _k(k), _c(c), _x0(x0), _xd0(xd0), _tau(tau), _x("x", parent), _xd("xd", parent), _xdd("xdd", parent)
    {
        // it is not necessary to add these dependencies since both _x and _xd are states and so, are known always
        _add_dependecny(_x);
        _add_dependecny(_xd);
    }

    void get_states(StatesInfo& states) override
    {
        states.add(_x, _x0, _xd);
        states.add(_xd, _xd0, _xdd);
    }

    void activation_function(double /*t*/, Values& values) override
    {
        auto& x = *values.get(_x);
        auto& xd = *values.get(_xd);
        auto& tau = *values.get(_tau);
        values.set(_xdd, tau/_m - _c/_m * xd - _k/_m * x);
    }
};

int main()
{
    using milli = std::chrono::milliseconds;
    auto start = std::chrono::high_resolution_clock::now();

    auto model = Model();
    auto tau = model.signal("tau");
    new MassSpringDamper(model, "msd", tau, 1, 1, 0.1, 0.1, -0.2);

    History history(model);

    Simulator sim(model,
    [&](double t, Values& values) -> void
    {
        values.set(tau, 0.01 * std::sin(t));
    }, rkf45);

    uint k = 0;
    double t;
    while (arange(k, t, 0, 50, 0.01))
    {
        sim.run(t);
        history.update(k, t, sim.values());
        k++;
    }

    auto finish = std::chrono::high_resolution_clock::now();
    std::cout << "It took "
              << std::chrono::duration_cast<milli>(finish - start).count()
              << " milliseconds\n";

    history.shrink_to_fit();

    auto x = model.find_signal(".x");

    Gnuplot gp;
	gp << "set xrange [0:" << history.nrows() - 1 << "]\n";
    gp << "set yrange [-0.4:0.5]\n";
	gp << "plot" << gp.file1d(history[x]) << "with lines title 'x'\n";

    return 0;
}
