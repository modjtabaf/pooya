
#include <iostream>
#include <math.h>
#include <vector>

#include "blocks.hpp"
#include "helper.hpp"
#include "solver.hpp"
#include "gp-ios.hpp"

using namespace blocks;

class SSModel : public Submodel
{
public:
    SSModel() : Submodel("")
    {
        auto x  = N("x" );
        auto xd = N("xd");

        enter();
        {
            new Const("", 2.7435, Ports({N("-time_delay")}));
            new Const("", 0.0, Ports({N("-initial")}));
            new Delay("", Ports({x, N("-time_delay"), N("-initial")}), Ports({xd}));
        }
        exit();
    }
};

int main()
{
    auto model = SSModel();
    auto history = run(model,
        [](uint k, double& t) -> bool
        {
            return arange(k, t, 0, 10, 0.1);
        },
        [](double t, const Signal& x, NamedSignals& inputs) -> void
        {
            inputs.insert_or_assign("x", S(std::sin(M_PI * t / 5)));
        });

    Gnuplot gp;
	gp << "set xrange [0:100]\n";
    gp << "set yrange [-1:1]\n";
	gp << "plot" << gp.file1d(history["x"]) << "with lines title 'x',"
		<< gp.file1d(history["xd"]) << "with lines title 'xd'\n";

    return 0;
}
