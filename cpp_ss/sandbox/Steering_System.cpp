
#include <algorithm>
#include <iostream>
#include <iterator>
#include <math.h>
#include <vector>
#include <chrono>

#include "blocks.hpp"
#include "helper.hpp"
#include "solver.hpp"
#include "gp-ios.hpp"

#include "front_wheel_angle_Rq.hpp"

using namespace blocks;

class PT : public Submodel
{
public:
    PT(Submodel* parent, const Signals& iports, const Signal& y_out) : Submodel(parent, "PT", iports, {y_out})
    {
        // signals
        const auto& y_in = _iports[0];
        const auto& tau  = _iports[1];
        const auto& y0   = _iports[2];

        // blocks
        new AddSub(this, "+-1", "+-", {y_in, y_out}, 10);
        new MulDiv(this, "*\\", "*/", {10, tau}, 20);
        new Integrator(this, "Int", 20, 30);
        new InitialValue(this, "IV", y0, 40);
        new AddSub(this, "+-2", "++", {30, 40}, y_out);
    }
};

class ComputeFrontWheelAngleRightLeftPinpoint : public Submodel
{
public:
    ComputeFrontWheelAngleRightLeftPinpoint(Submodel* parent, const Signal& front_wheel_angle, const Signals& oports) :
        Submodel(parent, "ComputeFrontWheelAngleRightLeftPinpoint", front_wheel_angle, oports)
    {
        // signals
        const auto& front_wheel_angle_right = oports[0];
        const auto& front_wheel_angle_left  = oports[1];
        auto tractor_wheelbase = get_model()->create_signal("tractor_wheelbase");
        auto tractor_Width = get_model()->create_signal("tractor_Width");

        // blocks
        new MulDiv(this, "*\\1", "*/", {tractor_wheelbase, front_wheel_angle}, 10);
        new AddSub(this, "+-1", "++", {10, tractor_Width}, 20);
        new MulDiv(this, "*\\2", "*/", {tractor_wheelbase, 20}, front_wheel_angle_right);
        new Gain(this, "K", 0.5, tractor_Width, 30);
        new AddSub(this, "+-2", "+-", {10, 30}, 40);
        new MulDiv(this, "*\\3", "*/", {tractor_wheelbase, 40}, front_wheel_angle_left);
    }
};

class SteeringSystem : public Submodel
{
public:
    SteeringSystem(Submodel* parent, const Signal& ad_DsrdFtWhlAngl_Rq_VD, const Signals& steering_info) :
        Submodel(parent, "Steering_System", ad_DsrdFtWhlAngl_Rq_VD, steering_info)
    {
        // signals
        const auto& front_wheel_angle = _oports[0];
        const auto& front_wheel_angle_rate = _oports[1];
        const auto& front_wheel_angle_neg = _oports[2];
        const auto& front_wheel_angle_rate_neg = _oports[3];
        const auto& AxFr_front_right = _oports[4];
        const auto& AxFr_front_left = _oports[5];

        auto front_wheel_ang_gain = get_model()->create_signal("front_wheel_ang_gain");
        auto front_wheel_ang_delay = get_model()->create_signal("front_wheel_ang_delay");
        auto front_wheel_ang_init_value = get_model()->create_signal("front_wheel_ang_init_value");
        auto front_wheel_ang_t_const = get_model()->create_signal("front_wheel_ang_t_const");

        // blocks
        new MulDiv(this, "*\\", "**", {ad_DsrdFtWhlAngl_Rq_VD, front_wheel_ang_gain}, 10);
        new Delay(this, "Delay", {10, front_wheel_ang_delay, front_wheel_ang_init_value}, 20);
        new Function(this, "Clamp",
            [](double /*t*/, const Value& x) -> Value
            {
                return x.max(0.001).min(10);
            }, front_wheel_ang_t_const, 30);
        new PT(this, {20, 30, 20}, front_wheel_angle);
        new Derivative(this, "Derivative", front_wheel_angle, front_wheel_angle_rate);
        new Gain(this, "K1", -1, front_wheel_angle, front_wheel_angle_neg);
        new Gain(this, "K2", -1, front_wheel_angle_rate, front_wheel_angle_rate_neg);
        new ComputeFrontWheelAngleRightLeftPinpoint(this, front_wheel_angle, {AxFr_front_right, AxFr_front_left});
        // new Bus(this, "Bus", {
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

    // front_wheel_angle_Rq = helper.load_mat_files_as_bus(
    //     "/home/fathi/torc/git/playground/py_ss/data/processed_mat",
    //     "front_wheel_angle_Rq")

    auto model = Model();

    SignalValues parameters({
        {"tractor_wheelbase", 5.8325},
        {"tractor_Width", 2.5},
        {"front_wheel_ang_t_const", 0.1},
        {"front_wheel_ang_delay", 0.02},
        {"front_wheel_ang_gain", 1.0},
        {"front_wheel_ang_init_value", 0.0},
        }, model);

    auto front_wheel_angle_Rq = model.create_signal("front_wheel_angle_Rq");

    auto front_wheel_angle = model.create_signal("front_wheel_angle");
    auto front_wheel_angle_rate = model.create_signal("front_wheel_angle_rate");
    auto front_wheel_angle_neg = model.create_signal("front_wheel_angle_neg");
    auto front_wheel_angle_rate_neg = model.create_signal("front_wheel_angle_rate_neg");
    auto AxFr_front_right = model.create_signal("AxFr_front_right");
    auto AxFr_front_left = model.create_signal("AxFr_front_left");

    auto steering_system = SteeringSystem(&model,
        front_wheel_angle_Rq,
        {
            front_wheel_angle,
            front_wheel_angle_rate,
            front_wheel_angle_neg,
            front_wheel_angle_rate_neg,
            AxFr_front_right,
            AxFr_front_left
        });
    auto history = run(model,
        [](uint k, double& t) -> bool
        {
            return arange(k, t, FRONT_WHEEL_ANGLE_RQ_X.front(),
                FRONT_WHEEL_ANGLE_RQ_X.back(), 0.1);
        },
        [&](double t, Values& values) -> void
        {
            values.set(front_wheel_angle_Rq,
                interp1d(t, FRONT_WHEEL_ANGLE_RQ_X, FRONT_WHEEL_ANGLE_RQ_Y));
        },
        parameters, rk4);

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

    // steering_info = helper.load_mat_files_as_bus(
    //     "/home/fathi/torc/git/playground/py_ss/data/processed_mat",
    //     "steering_info")

    // print(history.keys())

    // T = history["t"]

    // for f, mat in steering_info.items():
    //     print(f)
    //     if callable(mat):
    //         mat = mat.y
    //     mat = mat[slc]
        
    //     blk = history["Steering_System." + f]
        
    //     plt.figure()
        
    //     if blk.ndim == 2:
    //         n = blk.shape[1]
    //         for k in range(n):
    //             plt.subplot(n, 1, k+1)
    //             if k == 0:
    //                 plt.title(f)
    //             blk_k = blk[:, k]
    //             if mat.ndim == 2:
    //                 mat_k = mat[:, k]
    //                 plt.plot(T, mat_k, "rx")
    //             else:
    //                 mat_k = mat[k]
    //                 plt.plot(T[0], mat_k, "rx")
    //             plt.plot(T, blk_k, "b-")
    //             plt.plot(T, blk_k - mat_k, "g-")
    //             plt.grid()
    //     else:
    //         plt.title(f)
    //         plt.plot(T, mat, "rx")
    //         plt.plot(T, blk, "b-")
    //         plt.plot(T, blk - mat, "g-")
    //         plt.grid()

    // plt.show()
}
