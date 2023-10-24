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

        // choose a random name for this internal signal
        auto s10 = signal();

        auto g = parameter("g");
        auto l = parameter("l");

        new Integrator(*this, "dphi", d2phi, dphi);
        new Integrator(*this, "phi", dphi, phi, M_PI_4);
        // new Function(this, "sin(phi)",
        //     [](double /*t*/, const Value& x) -> Value
        //     {
        //         return x.sin();
        //     }, phi, s10);
        new Sin(*this, "sin(phi)", phi, s10);
        new Divide(*this, "-g\\l", {s10, g, l}, d2phi, -1);
    }
};

int main()
{
    using milli = std::chrono::milliseconds;
    auto start = std::chrono::high_resolution_clock::now();

    auto model = MyModel();

    History history(model);

    auto l = model.signal("l");
    auto g = model.signal("g");

    Simulator sim(model,
        [&](double /*t*/, Values& values) -> void
        {
            values.set(l, 0.1);
            values.set(g, 9.81);
        },
        rk4);

    uint k = 0;
    double t;
    while (arange(k, t, 0, 5, 0.01))
    {
        sim.run(t);
        history.update(k, t, sim.values());
        k++;
    }

    auto finish = std::chrono::high_resolution_clock::now();
    std::cout << "It took "
              << std::chrono::duration_cast<milli>(finish - start).count()
              << " milliseconds\n";

    auto phi = model.find_signal("/pendulum.phi", true);

    Gnuplot gp;
	gp << "set xrange [0:" << history[phi].size() - 1 << "]\n";
    gp << "set yrange [-0.8:0.8]\n";
	gp << "plot" << gp.file1d(history[phi]) << "with lines title 'x'\n";

    return 0;
}
