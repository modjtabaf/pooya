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

#ifndef __POOYA_HELPER_HPP__
#define __POOYA_HELPER_HPP__

#include <vector>
#include <fstream>
#include "pooya.hpp"
#include "solver.hpp"

namespace pooya
{

class History : public std::unordered_map<SignalId, Eigen::MatrixXd>
{
protected:
    const Model& _model;
    uint _nrows_grow;
    uint _bottom_row{static_cast<uint>(-1)};
    Eigen::ArrayXd _time;
    std::vector<SignalId> _signals;

public:
    History(const Model& model, uint nrows_grow = 1000) :
        _model(model), _nrows_grow(nrows_grow), _time(nrows_grow) {}

    void track_all(const Values& values);
    void track(SignalId sig);
    void untrack(SignalId sig);
    void update(uint k, double t, const Values& values);
    void export_csv(std::string filename);
    void shrink_to_fit();
    uint nrows() const {return _bottom_row + 1;}
    const Eigen::ArrayXd& time() const {return _time;}
};

using InputCallback = std::function<void(Model&, double, Values&)>;

class Simulator
{
protected:
    Model& _model;
    double _t_prev{0};
    InputCallback _inputs_cb;
    Values _values;
    Array _state_variables;
    Array _state_variables_orig;
    StepperBase* _stepper{nullptr};
    bool _initialized{false};

    const bool _reuse_order;
    using ProcessingOrder = std::vector<Block*>;
    ProcessingOrder _processing_order1;
    ProcessingOrder _processing_order2;
    ProcessingOrder* _current_po{nullptr};
    ProcessingOrder* _new_po{nullptr};

    uint _process(double t, Values& values);

public:
    Simulator(Model& model, InputCallback inputs_cb = nullptr, StepperBase* stepper = nullptr, bool reuse_order = false);

    void init(double t0=0.0);
    void run(double t, double min_time_step = 1e-3, double max_time_step = 1);

    const Values& values() const {return _values;}
};

bool arange(uint k, double& t, double t_init, double t_end, double dt);

}

#endif // __POOYA_HELPER_HPP__
