/*
Copyright 2024 Mojtaba (Moji) Fathi

 Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the “Software”), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "src/block/addsub.hpp"
#include "src/block/gain.hpp"
#include "src/block/integrator.hpp"
#include "src/block/source.hpp"
#include "src/block/submodel.hpp"
#include "src/misc/gp-ios.hpp"
#include "src/solver/history.hpp"
#include "src/solver/rk4.hpp"
#include "src/solver/simulator.hpp"

int main()
{
    // define model parameters
    constexpr double m{0.25}, c{0.3}, k{0.7};

    // create pooya blocks
    pooya::Submodel model("model");
    pooya::Source src_F("F(t)", [](double t) -> double { return t < 1 ? 0 : 1; });
    pooya::Gain gain_1_m("1_m", 1 / m);
    pooya::AddSub addsub_rhs("rhs", "+--");
    pooya::Integrator int_xdd("xdd");
    pooya::Integrator int_xd("xd");
    pooya::Gain gain_c_m("c_m", c / m);
    pooya::Gain gain_k_m("k_m", k / m);

    // create pooya signals
    pooya::ScalarSignal s_F("F");
    pooya::ScalarSignal s_F_m("F_m");
    pooya::ScalarSignal s_xdd("xdd");
    pooya::ScalarSignal s_xd("xd");
    pooya::ScalarSignal s_x("x");
    pooya::ScalarSignal s_kx_m("kx_m");
    pooya::ScalarSignal s_cxd_m("cxd_m");

    // set up the model
    model.add_block(src_F, {}, s_F);
    model.add_block(gain_1_m, s_F, s_F_m);
    model.add_block(addsub_rhs, {s_F_m, s_cxd_m, s_kx_m}, s_xdd);
    model.add_block(int_xdd, s_xdd, s_xd);
    model.add_block(int_xd, s_xd, s_x);
    model.add_block(gain_c_m, s_xd, s_cxd_m);
    model.add_block(gain_k_m, s_x, s_kx_m);

    // set up the simulator
    pooya::Rk4 solver;
    pooya::Simulator sim(model, nullptr, &solver);

    pooya::History history(model);
    history.track(s_x);
    history.track(s_xd);

    // run the simulation
    double t;
    for (uint k = 0; pooya::arange(k, t, 0, 10, 0.1); k++)
    {
        sim.run(t);
        history.update(k, t);
    }

    // post-processing
    history.shrink_to_fit();
    history.export_csv("pooya_tutorial_01_mass_spring_damper.csv");

    Gnuplot gp;
    gp << "set xrange [0:" << history.nrows() - 1 << "]\n";
    gp << "plot" << gp.file1d(history[s_x]) << "with lines title 'x'," << gp.file1d(history[s_xd])
       << "with lines title 'xd'\n";

    return 0;
}
