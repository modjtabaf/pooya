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

class MyModel : public pooya::Submodel
{
protected:
    pooya::Integrator _integ1{   "xd",      0.1};
    pooya::Integrator _integ2{    "x"          };
    pooya::Gain         _gain{"-k\\m", -1.0/1.0};

public:
    MyModel() : pooya::Submodel("MyModel") {}

    bool init(pooya::Parent& parent, const pooya::LabelSignals& iports, const pooya::LabelSignals& oports) override
    {
        pooya_trace0;
        if (!pooya::Submodel::init(parent, iports, oports))
            return false;

        // create signals
        auto   x = signal(  "x");
        auto  xd = signal( "xd");
        auto xdd = signal("xdd");

        // setup the submodel
        add_block(_integ1, xdd,  xd);
        add_block(_integ2,  xd,   x);
        add_block(  _gain,   x, xdd);

        return true;
    }
};

int main()
{
    pooya_trace0;
    using milli = std::chrono::milliseconds;
    auto  start = std::chrono::high_resolution_clock::now();

    // create raw blocks
    pooya::Model model("test04");
    MyModel mymodel;

    // setup the model
    model.add_block(mymodel);

    pooya::Rk4 stepper(model);
    pooya::Simulator sim(model, nullptr, &stepper);

    pooya::History history(model);

    uint k = 0;
    double t;
    while (pooya::arange(k, t, 0, 5, 0.01))
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
    history.export_csv("mass_spring.csv");

    auto  x = model.lookup_signal( "x");
    auto xd = model.lookup_signal("xd");

    Gnuplot gp;
	gp << "set xrange [0:" << history.nrows() - 1 << "]\n";
    gp << "set yrange [-0.15:0.15]\n";
	gp << "plot" << gp.file1d(history[x]) << "with lines title 'x',"
		<< gp.file1d(history[xd]) << "with lines title 'xd'\n";

    assert(pooya::util::pooya_trace_info.size() == 1);

    return 0;
}
