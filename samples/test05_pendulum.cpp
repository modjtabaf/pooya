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
    pooya::Integrator _integ1{"dphi"};
    pooya::Integrator _integ2{"phi", M_PI_4};
    // pooya::Function     _func{"sin(phi)",
    //     [](double /*t*/, const pooya::Array& x) -> Array
    //     {
    //         return x.sin();
    //     }},
    pooya::Sin _sin{"sin(phi)"};
    pooya::MulDiv _muldiv{"-g\\l", "**/", -1};

public:
    Pendulum() : pooya::Submodel("pendulum") {}

    bool init(pooya::Parent& parent, pooya::BusId, pooya::BusId) override
    {
        pooya_trace0;

        if (!pooya::Submodel::init(parent))
            return false;

        // create pooya signals
        auto phi   = create_scalar_signal("phi");
        auto dphi  = create_scalar_signal("dphi");
        auto d2phi = create_scalar_signal("d2phi");

        auto s10 = create_scalar_signal(); // choose a random name for this internal signal

        auto& model_ = model_ref();
        auto g = model_.scalar_signal("g");
        auto l = model_.scalar_signal("l");

        // setup the submodel
        add_block(_integ1, d2phi, dphi);
        add_block(_integ2, dphi, phi);
        // add_block(_func, phi, s10);
        add_block(_sin, phi, s10);
        add_block(_muldiv, {s10, g, l}, d2phi);

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

    auto l = model.scalar_signal("l");
    auto g = model.scalar_signal("g");

    pooya::Rk4 stepper(model);
    pooya::Simulator sim(model,
        [&](pooya::Model&, double /*t*/, pooya::Values& values) -> void
        {
            pooya_trace0;
            values[l] = 0.1;
            values[g] = 9.81;
        },
        &stepper);

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

    auto phi = model.lookup_signal("/pendulum~phi");

    history.shrink_to_fit();

    Gnuplot gp;
	gp << "set xrange [0:" << history.nrows() - 1 << "]\n";
    gp << "set yrange [-0.8:0.8]\n";
	gp << "plot" << gp.file1d(history[phi]) << "with lines title 'x'\n";

    assert(pooya::util::pooya_trace_info.size() == 1);

    return 0;
}
