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

class MyModel : public Model
{
public:
    MyModel() : Model("pendulum")
    {
        auto   phi = signal(  "phi");
        auto  dphi = signal( "dphi");
        auto d2phi = signal("d2phi");

        auto tau = parameter("tau");
        auto m = parameter("m");
        auto g = parameter("g");
        auto l = parameter("l");

        new Integrator(*this, "dphi", d2phi, dphi, M_PI_4);
        new Integrator(*this, "phi", dphi, phi);
        new Function(*this, "sin(phi)",
            [](double /*t*/, const Value& x) -> Value
            {
                return x.sin();
            }, phi, signal(10));
        // new Sin(*this, "sin(phi)", phi, signal(10));
        new MulDiv(*this, "g\\l", "**/", {signal(10), g, l}, signal(20));
        new MulDiv(*this, "tau\\ml2", "*///", {tau, m, l, l}, signal(30));
        new AddSub(*this, "d2phi", "+-", {signal(30), signal(20)}, d2phi);
    }
};

int main()
{
    using milli = std::chrono::milliseconds;
    auto start = std::chrono::high_resolution_clock::now();

    auto model = MyModel();

    auto m = model.signal("m");
    auto l = model.signal("l");
    auto g = model.signal("g");
    auto tau = model.signal("tau");

    History history(model);

    Simulator sim(model,
        [&](double /*t*/, Values& values) -> void
        {
            values.set(m, 0.2);
            values.set(l, 0.1);
            values.set(g, 9.81);
            values.set(tau, 0.13);
        },
        rkf45, true);

    uint k = 0;
    double t;
    while (arange(k, t, 0, 5, 0.1))
    {
        sim.run(t);
        history.update(k, t, sim.values());
        k++;
    }

    auto finish = std::chrono::high_resolution_clock::now();
    std::cout << "It took "
              << std::chrono::duration_cast<milli>(finish - start).count()
              << " milliseconds\n";

    auto  phi = model.find_signal("/pendulum.phi");
    auto dphi = model.find_signal(".dphi");

    Gnuplot gp;
	gp << "set xrange [0:" << history[phi].size() - 1 << "]\n";
    gp << "set yrange [-8:8]\n";
	gp << "plot" << gp.file1d(history[phi]) << "with lines title 'phi',"
	    << gp.file1d(history[dphi]) << "with lines title 'dphi'\n";

    return 0;
}
