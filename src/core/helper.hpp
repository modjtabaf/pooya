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

#include "pooya.hpp"
#include "solver.hpp"

namespace pooya
{

class History : public std::unordered_map<SignalId, Eigen::MatrixXd>
{
protected:
#if defined(POOYA_USE_SMART_PTRS)
    std::reference_wrapper<const Model> _model;
#else // defined(POOYA_USE_SMART_PTRS)
    const Model& _model;
#endif // defined(POOYA_USE_SMART_PTRS)
    uint _nrows_grow;
    uint _bottom_row{static_cast<uint>(-1)};
    Array _time;
    std::vector<ValueSignalId> _signals;

public:
    History(const Model& model, uint nrows_grow = 1000) :
        _model(model), _nrows_grow(nrows_grow), _time(nrows_grow) {}

    void track_all();
    void track(SignalId sig);
    void untrack(SignalId sig);
    void update(uint k, double t);
    void export_csv(const std::string& filename);
    void shrink_to_fit();
    uint nrows() const {return _bottom_row + 1;}
    const Array& time() const {return _time;}
};

using InputCallback = std::function<void(Model&, double)>;

class Simulator
{
protected:
#if defined(POOYA_USE_SMART_PTRS)
    std::reference_wrapper<Model> _model;
#else // defined(POOYA_USE_SMART_PTRS)
    Model& _model;
#endif // defined(POOYA_USE_SMART_PTRS)
    double _t_prev{0};
    InputCallback _inputs_cb;
    Array _state_variables;
    Array _state_variables_orig;
#if defined(POOYA_USE_SMART_PTRS)
    std::optional<std::reference_wrapper<StepperBase>> _stepper;
#else // defined(POOYA_USE_SMART_PTRS)
    StepperBase* _stepper{nullptr};
#endif // defined(POOYA_USE_SMART_PTRS)
    bool _initialized{false};

    const bool _reuse_order;
#if defined(POOYA_USE_SMART_PTRS)
    using ProcessingOrder = std::vector<std::reference_wrapper<Block>>;
#else // defined(POOYA_USE_SMART_PTRS)
    using ProcessingOrder = std::vector<Block*>;
#endif // defined(POOYA_USE_SMART_PTRS)
    ProcessingOrder _processing_order1;
    ProcessingOrder _processing_order2;
#if defined(POOYA_USE_SMART_PTRS)
    std::reference_wrapper<ProcessingOrder> _current_po{_processing_order1};
    std::reference_wrapper<ProcessingOrder> _new_po{_processing_order2};
#else // defined(POOYA_USE_SMART_PTRS)
    ProcessingOrder* _current_po{nullptr};
    ProcessingOrder* _new_po{nullptr};
#endif // defined(POOYA_USE_SMART_PTRS)

    uint _process(double t);

public:
    Simulator(Model& model, InputCallback inputs_cb = nullptr, StepperBase* stepper = nullptr, bool reuse_order = false);

    void init(double t0=0.0);
    void run(double t, double min_time_step = 1e-3, double max_time_step = 1);
};

bool arange(uint k, double& t, double t_init, double t_end, double dt);

}

#endif // __POOYA_HELPER_HPP__
