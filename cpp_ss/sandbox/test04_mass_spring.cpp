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

#include "pooya.hpp"
#include "helper.hpp"
#include "solver.hpp"
#include "gp-ios.hpp"

using namespace pooya;

class MyModel : public Model
{
public:
    MyModel() : Model("MyModel")
    {
        auto   x = signal(  "x");
        auto  xd = signal( "xd");
        auto xdd = signal("xdd");

        new Integrator(this, "xd", xdd, xd, 0.1);
        new Integrator(this, "x", xd, x);
        new Gain(this, "-k\\m", -1.0/1.0, x, xdd);
    }
};

int main()
{
    using milli = std::chrono::milliseconds;
    auto start = std::chrono::high_resolution_clock::now();

    auto model = MyModel();

    History history(model);

    Simulator sim(model, nullptr, rk4);

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

    history.export_csv("mass_spring.csv");

    auto x  = model.find_signal("x");
    auto xd = model.find_signal("xd");

    Gnuplot gp;
	gp << "set xrange [0:500]\n";
    gp << "set yrange [-0.15:0.15]\n";
	gp << "plot" << gp.file1d(history[x]) << "with lines title 'x',"
		<< gp.file1d(history[xd]) << "with lines title 'xd'\n";

    return 0;
}
