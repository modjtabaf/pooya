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
protected:
    Subtract     _sub{"sub"};
    Divide       _div{"div"};
    Integrator _integ{"int"};
    InitialValue  _iv{"iv"};
    Add          _add{"add"};

public:
    PT() : Submodel("PT") {}

    bool init(Parent& parent, const Signals& iports, const Signals& oports) override
    {
        if (!Submodel::init(parent, iports, oports))
            return false;

        // choose random names for these internal signals
        auto s10 = signal();
        auto s20 = signal();
        auto s30 = signal();
        auto s40 = signal();

        // blocks
        add_block(  _sub, {y_in, y_out},   s10);
        add_block(  _div, {s10, tau}   ,   s20);
        add_block(_integ,  s20         ,   s30);
        add_block(   _iv,  y0          ,   s40);
        add_block(  _add, {s30, s40}   , y_out);

        return true;
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

        // choose random names for these internal signals
        auto s10 = signal();
        auto s20 = signal();
        auto s30 = signal();
        auto s40 = signal();

        // blocks
        new Divide(*this, "*\\1", {tractor_wheelbase, front_wheel_angle}, s10);
        new Add(*this, "+1", {s10, tractor_Width}, s20);
        new Divide(*this, "*\\2", {tractor_wheelbase, s20}, front_wheel_angle_right);
        new Gain(*this, "K", 0.5, tractor_Width, s30);
        new Subtract(*this, "+-2", {s10, s30}, s40);
        new Divide(*this, "*\\3", {tractor_wheelbase, s40}, front_wheel_angle_left);
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

        // choose random names for these internal signals
        auto s10 = signal();
        auto s20 = signal();
        auto s30 = signal();

        // blocks
        new Multiply(*this, "Mul", {ad_DsrdFtWhlAngl_Rq_VD, front_wheel_ang_gain}, s10);
        new Delay(*this, "Delay", {s10, front_wheel_ang_delay, front_wheel_ang_init_value}, s20);
        new Function(*this, "Clamp",
            [](double /*t*/, const Value& x) -> Value
            {
                return x.max(0.001).min(10);
            }, front_wheel_ang_t_const, s30);
        new PT(*this, s20, s30, s20, front_wheel_angle);
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

    History history(model, 1000);

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

    history.shrink_to_fit();

    Gnuplot gp;
	gp << "set xrange [0:2000]\n";
    gp << "set yrange [-0.6:0.6]\n";
	gp << "plot" << gp.file1d((history[front_wheel_angle_Rq] * (180/M_PI)).eval()) << "with lines title 'front\\_wheel\\_angle\\_Rq',"
	    << gp.file1d(((history[AxFr_front_right] + history[AxFr_front_left])/2 * (180/M_PI)).eval()) << "with lines title 'dphi',"
        "\n";
}
