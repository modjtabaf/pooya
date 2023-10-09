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

#include <algorithm>
#include <iostream>
#include <iterator>
#include <math.h>
#include <vector>
#include <chrono>

#include "src/core/pooya.hpp"
#include "src/core/helper.hpp"
#include "src/core/solver.hpp"
#include "src/misc/gp-ios.hpp"

#include "tests/data/front_wheel_angle_Rq.hpp"

using namespace pooya;

class PT : public Submodel
{
public:
    PT(Parent& parent, const Signal& y_in, const Signal& tau, const Signal& y0, const Signal& y_out) : Submodel(parent, "PT", {y_in, tau, y0}, y_out)
    {
        // blocks
        new AddSub(*this, "+-1", "+-", {y_in, y_out}, signal(10));
        new MulDiv(*this, "*\\", "*/", {signal(10), tau}, signal(20));
        new Integrator(*this, "Int", signal(20), signal(30));
        new InitialValue(*this, "IV", y0, signal(40));
        new AddSub(*this, "+-2", "++", {signal(30), signal(40)}, y_out);
    }
};

class ComputeFrontWheelAngleRightLeftPinpoint : public Submodel
{
public:
    ComputeFrontWheelAngleRightLeftPinpoint(Parent& parent, const Signal& front_wheel_angle, const Signal& front_wheel_angle_right, const Signal& front_wheel_angle_left) :
        Submodel(parent, "ComputeFrontWheelAngleRightLeftPinpoint", front_wheel_angle, {front_wheel_angle_right, front_wheel_angle_left})
    {
        auto tractor_wheelbase = parameter("tractor_wheelbase");
        auto tractor_Width = parameter("tractor_Width");

        // blocks
        new MulDiv(*this, "*\\1", "*/", {tractor_wheelbase, front_wheel_angle}, signal(10));
        new AddSub(*this, "+-1", "++", {signal(10), tractor_Width}, signal(20));
        new MulDiv(*this, "*\\2", "*/", {tractor_wheelbase, signal(20)}, front_wheel_angle_right);
        new Gain(*this, "K", 0.5, tractor_Width, signal(30));
        new AddSub(*this, "+-2", "+-", {signal(10), signal(30)}, signal(40));
        new MulDiv(*this, "*\\3", "*/", {tractor_wheelbase, signal(40)}, front_wheel_angle_left);
    }
};

class SteeringSystem : public Submodel
{
public:
    SteeringSystem(Parent& parent, const Signal& ad_DsrdFtWhlAngl_Rq_VD, const Signals& steering_info) :
        Submodel(parent, "Steering_System", ad_DsrdFtWhlAngl_Rq_VD, steering_info)
    {
        // signals
        const auto& front_wheel_angle = _oports[0];
        const auto& front_wheel_angle_rate = _oports[1];
        const auto& front_wheel_angle_neg = _oports[2];
        const auto& front_wheel_angle_rate_neg = _oports[3];
        const auto& AxFr_front_right = _oports[4];
        const auto& AxFr_front_left = _oports[5];

        auto front_wheel_ang_gain = parameter("front_wheel_ang_gain");
        auto front_wheel_ang_delay = parameter("front_wheel_ang_delay");
        auto front_wheel_ang_init_value = parameter("front_wheel_ang_init_value");
        auto front_wheel_ang_t_const = parameter("front_wheel_ang_t_const");

        // blocks
        new MulDiv(*this, "*\\", "**", {ad_DsrdFtWhlAngl_Rq_VD, front_wheel_ang_gain}, signal(10));
        new Delay(*this, "Delay", {signal(10), front_wheel_ang_delay, front_wheel_ang_init_value}, signal(20));
        new Function(*this, "Clamp",
            [](double /*t*/, const Value& x) -> Value
            {
                return x.max(0.001).min(10);
            }, front_wheel_ang_t_const, signal(30));
        new PT(*this, signal(20), signal(30), signal(20), front_wheel_angle);
        new Derivative(*this, "Derivative", front_wheel_angle, front_wheel_angle_rate);
        new Gain(*this, "K1", -1, front_wheel_angle, front_wheel_angle_neg);
        new Gain(*this, "K2", -1, front_wheel_angle_rate, front_wheel_angle_rate_neg);
        new ComputeFrontWheelAngleRightLeftPinpoint(*this, front_wheel_angle, AxFr_front_right, AxFr_front_left);
        // new Bus(*this, "Bus", {
        //     front_wheel_angle,
        //     front_wheel_angle_rate,
        //     front_wheel_angle_neg,
        //     front_wheel_angle_rate_neg,
        //     AxFr_front_right,
        //     AxFr_front_left
        //     }, steering_info);
    }
};

double interp1d(double x, const std::vector<double>& X, const std::vector<double>& Y)
{
    assert((X.size() == Y.size()) and (not X.empty()));

    if (x <= X.front())
        return Y.front();
    if (x >= X.back())
        return Y.back();
    auto xlb_it = std::lower_bound(X.begin(), X.end(), x);
    double x0 = *xlb_it;
    double x1 = *(xlb_it + 1);
    auto ylb_it = Y.begin() + std::distance(X.begin(), xlb_it);
    double y0 = *ylb_it;
    double y1 = *(ylb_it + 1);
    return (y1 - y0)*(x - x0)/(x1 - x0) + y0;
}

int main()
{
    using milli = std::chrono::milliseconds;
    auto start = std::chrono::high_resolution_clock::now();

    auto model = Model();

    auto front_wheel_angle_Rq = model.signal("front_wheel_angle_Rq");

    auto front_wheel_angle = model.signal("front_wheel_angle");
    auto front_wheel_angle_rate = model.signal("front_wheel_angle_rate");
    auto front_wheel_angle_neg = model.signal("front_wheel_angle_neg");
    auto front_wheel_angle_rate_neg = model.signal("front_wheel_angle_rate_neg");
    auto AxFr_front_right = model.signal("AxFr_front_right");
    auto AxFr_front_left = model.signal("AxFr_front_left");

    new SteeringSystem(model,
        front_wheel_angle_Rq,
        {
            front_wheel_angle,
            front_wheel_angle_rate,
            front_wheel_angle_neg,
            front_wheel_angle_rate_neg,
            AxFr_front_right,
            AxFr_front_left
        });

    auto tractor_wheelbase = model.parameter("tractor_wheelbase");
    auto tractor_Width = model.parameter("tractor_Width");
    auto front_wheel_ang_t_const = model.parameter("front_wheel_ang_t_const");
    auto front_wheel_ang_delay = model.parameter("front_wheel_ang_delay");
    auto front_wheel_ang_gain = model.parameter("front_wheel_ang_gain");
    auto front_wheel_ang_init_value = model.parameter("front_wheel_ang_init_value");

    History history(model);

    Simulator sim(model,
        [&](double t, Values& values) -> void
        {
            values.set(front_wheel_angle_Rq,
                interp1d(t, FRONT_WHEEL_ANGLE_RQ_X, FRONT_WHEEL_ANGLE_RQ_Y));
            values.set(tractor_wheelbase, 5.8325);
            values.set(tractor_Width, 2.5);
            values.set(front_wheel_ang_t_const, 0.1);
            values.set(front_wheel_ang_delay, 0.02);
            values.set(front_wheel_ang_gain, 1.0);
            values.set(front_wheel_ang_init_value, 0.0);
        },
        rkf45, true);

    uint k = 0;
    double t;
    while (arange(k, t, FRONT_WHEEL_ANGLE_RQ_X.front(), FRONT_WHEEL_ANGLE_RQ_X.back(), 0.1))
    {
        sim.run(t);
        history.update(k, t, sim.values());
        k++;
    }

    auto finish = std::chrono::high_resolution_clock::now();
    std::cout << "It took "
              << std::chrono::duration_cast<milli>(finish - start).count()
              << " milliseconds\n";

    Gnuplot gp;
	gp << "set xrange [0:2000]\n";
    gp << "set yrange [-0.6:0.6]\n";
	gp << "plot" << gp.file1d((history[front_wheel_angle_Rq] * (180/M_PI)).eval()) << "with lines title 'front\\_wheel\\_angle\\_Rq',"
	    << gp.file1d(((history[AxFr_front_right] + history[AxFr_front_left])/2 * (180/M_PI)).eval()) << "with lines title 'dphi',"
        "\n";
}
