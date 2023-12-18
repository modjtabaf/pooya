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

class PT : public pooya::Submodel
{
protected:
    pooya::Subtract     _sub{"sub"};
    pooya::Const      _const;
    pooya::Divide       _div{"div"};
    pooya::Integrator _integ{"int"};
    pooya::InitialValue  _iv{"iv"};
    pooya::Add          _add{"add"};

public:
    PT(double tau) : pooya::Submodel("PT", 2, 1), _const("tau", tau) {}

    bool init(pooya::Parent& parent, const pooya::Signals& iports, const pooya::Signals& oports) override
    {
        if (!pooya::Submodel::init(parent, iports, oports))
            return false;

        // choose random names for these internal signals
        auto s10 = signal();
        auto s15 = signal();
        auto s20 = signal();
        auto s30 = signal();
        auto s40 = signal();

        auto&  y_in = iports[0];
        auto&    y0 = iports[1];
        auto& y_out = oports[0];

        // blocks
        add_block(  _sub, {y_in, y_out},   s10);
        add_block(_const,            {},   s15);
        add_block(  _div,    {s10, s15},   s20);
        add_block(_integ,          s20 ,   s30);
        add_block(   _iv,           y0 ,   s40);
        add_block(  _add,    {s30, s40}, y_out);

        return true;
    }
};

class ComputeFrontWheelAngleRightLeftPinpoint : public pooya::Submodel
{
protected:
    pooya::Divide  _div1{"div1"};
    pooya::Add      _add{"add"};
    pooya::Divide  _div2{"div2"};
    pooya::Gain    _gain{"K", 0.5};
    pooya::Subtract _sub{"sub"};
    pooya::Divide  _div3{"div3"};

    pooya::Signal       _s_front_wheel_angle;
    pooya::Signal _s_front_wheel_angle_right;
    pooya::Signal  _s_front_wheel_angle_left;

public:
    ComputeFrontWheelAngleRightLeftPinpoint() : pooya::Submodel("ComputeFrontWheelAngleRightLeftPinpoint", 1, 2) {}

    bool init(pooya::Parent& parent, const pooya::Signals& iports, const pooya::Signals& oports) override
    {
        if (!pooya::Submodel::init(parent, iports, oports))
            return false;

        auto tractor_wheelbase = parameter("tractor_wheelbase");
        auto     tractor_Width = parameter("tractor_Width");

        // choose random names for these internal signals
        auto s10 = signal();
        auto s20 = signal();
        auto s30 = signal();
        auto s40 = signal();

        _s_front_wheel_angle       = iports[0];
        _s_front_wheel_angle_right = oports[0];
        _s_front_wheel_angle_left  = oports[1];

        // blocks
        add_block(_div1, {tractor_wheelbase, _s_front_wheel_angle}, s10);
        add_block( _add, {s10, tractor_Width}, s20);
        add_block(_div2, {tractor_wheelbase, s20}, _s_front_wheel_angle_right);
        add_block(_gain, tractor_Width, s30);
        add_block( _sub, {s10, s30}, s40);
        add_block(_div3, {tractor_wheelbase, s40}, _s_front_wheel_angle_left);

        return true;
    }
};

class SteeringSystem : public pooya::Submodel
{
protected:
    pooya::Multiply    _mul{"mul"};
    pooya::Delay     _delay{"delay"};
    pooya::Function _clamp{"clamp",
        [](double /*t*/, const double& x) -> double
        {
            return std::max(0.001, std::min(x, 10.0));
        }};
    PT _pt{0.1};
    pooya::Derivative _deriv{"deriv"};
    pooya::Gain          _k1{"K1", -1};
    pooya::Gain          _k2{"K2", -1};
    ComputeFrontWheelAngleRightLeftPinpoint _cfwarlp;

    pooya::Signal     _s_ad_DsrdFtWhlAngl_Rq_VD;
    pooya::Signal          _s_front_wheel_angle;
    pooya::Signal     _s_front_wheel_angle_rate;
    pooya::Signal      _s_front_wheel_angle_neg;
    pooya::Signal _s_front_wheel_angle_rate_neg;
    pooya::Signal           _s_AxFr_front_right;
    pooya::Signal            _s_AxFr_front_left;

public:
    SteeringSystem() : pooya::Submodel("Steering_System", 1, 6) {}

    bool init(pooya::Parent& parent, const pooya::Signals& iports, const pooya::Signals& oports) override
    {
        if (!pooya::Submodel::init(parent, iports, oports))
            return false;

        // signals
        _s_ad_DsrdFtWhlAngl_Rq_VD     = iports[0];

        _s_front_wheel_angle          = oports[0];
        _s_front_wheel_angle_rate     = oports[1];
        _s_front_wheel_angle_neg      = oports[2];
        _s_front_wheel_angle_rate_neg = oports[3];
        _s_AxFr_front_right           = oports[4];
        _s_AxFr_front_left            = oports[5];

        auto       front_wheel_ang_gain = parameter("front_wheel_ang_gain");
        auto      front_wheel_ang_delay = parameter("front_wheel_ang_delay");
        auto front_wheel_ang_init_value = parameter("front_wheel_ang_init_value");
        auto    front_wheel_ang_t_const = parameter("front_wheel_ang_t_const");

        // choose random names for these internal signals
        auto s10 = signal();
        auto s20 = signal();
        auto s30 = signal();

        // blocks
        add_block(    _mul, {_s_ad_DsrdFtWhlAngl_Rq_VD, front_wheel_ang_gain}, s10);
        add_block(  _delay, {s10, front_wheel_ang_delay, front_wheel_ang_init_value}, s20);
        add_block(  _clamp, front_wheel_ang_t_const, s30);
        add_block(     _pt, {s20, s30}, _s_front_wheel_angle);
        add_block(  _deriv, _s_front_wheel_angle, _s_front_wheel_angle_rate);
        add_block(     _k1, _s_front_wheel_angle, _s_front_wheel_angle_neg);
        add_block(     _k2, _s_front_wheel_angle_rate, _s_front_wheel_angle_rate_neg);
        add_block(_cfwarlp, _s_front_wheel_angle, {_s_AxFr_front_right, _s_AxFr_front_left});
        // new Bus(*this, "Bus", {
        //     front_wheel_angle,
        //     front_wheel_angle_rate,
        //     front_wheel_angle_neg,
        //     front_wheel_angle_rate_neg,
        //     AxFr_front_right,
        //     AxFr_front_left
        //     }, steering_info);

        return true;
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

    auto model = pooya::Model();
    auto str_sys = SteeringSystem();

    auto       front_wheel_angle_Rq = model.signal("front_wheel_angle_Rq");
    auto          front_wheel_angle = model.signal("front_wheel_angle");
    auto     front_wheel_angle_rate = model.signal("front_wheel_angle_rate");
    auto      front_wheel_angle_neg = model.signal("front_wheel_angle_neg");
    auto front_wheel_angle_rate_neg = model.signal("front_wheel_angle_rate_neg");
    auto           AxFr_front_right = model.signal("AxFr_front_right");
    auto            AxFr_front_left = model.signal("AxFr_front_left");

    model.add_block(str_sys, front_wheel_angle_Rq,
        {
            front_wheel_angle,
            front_wheel_angle_rate,
            front_wheel_angle_neg,
            front_wheel_angle_rate_neg,
            AxFr_front_right,
            AxFr_front_left
        });

    auto          tractor_wheelbase = model.parameter("tractor_wheelbase");
    auto              tractor_Width = model.parameter("tractor_Width");
    auto    front_wheel_ang_t_const = model.parameter("front_wheel_ang_t_const");
    auto      front_wheel_ang_delay = model.parameter("front_wheel_ang_delay");
    auto       front_wheel_ang_gain = model.parameter("front_wheel_ang_gain");
    auto front_wheel_ang_init_value = model.parameter("front_wheel_ang_init_value");

    pooya::History history(model, 1000);

    pooya::Simulator sim(model,
        [&](pooya::Model& /*model*/, double t, pooya::Values& values) -> void
        {
            values.set_scalar(front_wheel_angle_Rq,
                interp1d(t, FRONT_WHEEL_ANGLE_RQ_X, FRONT_WHEEL_ANGLE_RQ_Y));
            values.set_scalar(tractor_wheelbase, 5.8325);
            values.set_scalar(tractor_Width, 2.5);
            values.set_scalar(front_wheel_ang_t_const, 0.1);
            values.set_scalar(front_wheel_ang_delay, 0.02);
            values.set_scalar(front_wheel_ang_gain, 1.0);
            values.set_scalar(front_wheel_ang_init_value, 0.0);
        },
        pooya::rkf45, true);

    uint k = 0;
    double t;
    while (pooya::arange(k, t, FRONT_WHEEL_ANGLE_RQ_X.front(), FRONT_WHEEL_ANGLE_RQ_X.back(), 0.1))
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
