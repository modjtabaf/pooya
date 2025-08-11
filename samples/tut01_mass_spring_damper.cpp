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

#include "src/block/gain.hpp"
#include "src/block/integrator.hpp"
#include "src/block/source.hpp"
#include "src/block/submodel.hpp"
#include "src/block/subtract.hpp"
#include "src/misc/gp-ios.hpp"
#include "src/solver/history.hpp"
#include "src/solver/rk4.hpp"
#include "src/solver/simulator.hpp"

int main()
{
    // define model parameters
    constexpr double m{0.25}, c{0.3}, k{0.7};

    // create pooya blocks
    pooya::Submodel model;
    pooya::Source src_F(&model, [](double t) -> double { return t < 1 ? 0 : 1; });
    pooya::Gain gain_1_m(&model, 1 / m);
    pooya::Subtract sub1(&model);
    pooya::Subtract sub2(&model);
    pooya::Integrator int_xdd(&model);
    pooya::Integrator int_xd(&model);
    pooya::Gain gain_c_m(&model, c / m);
    pooya::Gain gain_k_m(&model, k / m);

    // create pooya signals
    pooya::ScalarSignal s_F;
    pooya::ScalarSignal s_F_m;
    pooya::ScalarSignal s1;
    pooya::ScalarSignal s_xdd;
    pooya::ScalarSignal s_xd;
    pooya::ScalarSignal s_x;
    pooya::ScalarSignal s_kx_m;
    pooya::ScalarSignal s_cxd_m;

    // set up the model
    src_F.connect({}, s_F);
    gain_1_m.connect(s_F, s_F_m);
    sub1.connect({s_F_m, s_cxd_m}, s1);
    sub2.connect({s1, s_kx_m}, s_xdd);
    int_xdd.connect(s_xdd, s_xd);
    int_xd.connect(s_xd, s_x);
    gain_c_m.connect(s_xd, s_cxd_m);
    gain_k_m.connect(s_x, s_kx_m);

    // set up the simulator
    pooya::Rk4 solver;
    pooya::Simulator sim(model, nullptr, &solver);

    pooya::History history;
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
