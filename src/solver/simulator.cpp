/*
Copyright 2024 Mojtaba (Moji) Fathi

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”),
to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <iostream>
#include <memory>
#include <unordered_set>

#include "src/helper/util.hpp"
#include "simulator.hpp"

namespace pooya
{

bool arange(uint k, double& t, double t_init, double t_end, double dt)
{
    t_init += k * dt;
    return (t_init <= t_end) && (t = t_init, true);
}

Simulator::Simulator(Block& model, InputCallback inputs_cb, StepperBase* stepper, bool reuse_order) :
    _model(model), _inputs_cb(inputs_cb), _reuse_order(reuse_order)
{
    pooya_trace("model: " + model.full_name());
    if (stepper)
#if defined(POOYA_USE_SMART_PTRS)
        {_stepper = *stepper;}
#else // defined(POOYA_USE_SMART_PTRS)
        {_stepper = stepper;}
#endif // defined(POOYA_USE_SMART_PTRS)
}

uint Simulator::_process(double t)
{
    pooya_trace("t: " + std::to_string(t));
    _model._mark_unprocessed();

    uint n_processed = 0;

    if (_reuse_order)
    {
#if defined(POOYA_USE_SMART_PTRS)
        _new_po.get().clear();
#else // defined(POOYA_USE_SMART_PTRS)
        _new_po->clear();
#endif // defined(POOYA_USE_SMART_PTRS)

        bool any_processed;
        do
        {
            any_processed = false;
#if defined(POOYA_USE_SMART_PTRS)
            for (auto& base: _current_po.get())
            {
                if (base.get().processed()) {continue;}
                n_processed += base.get()._process(t, false);
                if (base.get().processed())
                {
                    _new_po.get().emplace_back(base);
                    any_processed = true;
                }
            }
#else // defined(POOYA_USE_SMART_PTRS)
            for (auto* base: *_current_po)
            {
                if (base->processed()) {continue;}
                n_processed += base->_process(t, false);
                if (base->processed())
                {
                    _new_po->emplace_back(base);
                    any_processed = true;
                }
            }
#endif // defined(POOYA_USE_SMART_PTRS)
        } while(any_processed);

#if defined(POOYA_USE_SMART_PTRS)
        for (auto& base: _current_po.get())
        {
            if (!base.get().processed())
                {_new_po.get().emplace_back(base);}
        }

        assert(_current_po.get().size() == _new_po.get().size());
        assert(_current_po.get().size() == _current_po.get().capacity());
        assert(_new_po.get().size() == _new_po.get().capacity());
#else // defined(POOYA_USE_SMART_PTRS)
        for (auto* base: *_current_po)
        {
            if (!base->processed())
                {_new_po->emplace_back(base);}
        }

        assert(_current_po->size() == _new_po->size());
        assert(_current_po->size() == _current_po->capacity());
        assert(_new_po->size() == _new_po->capacity());
#endif // defined(POOYA_USE_SMART_PTRS)

        std::swap(_current_po, _new_po);
    }
    else
    {
        uint n;
        do
        {
            n = _model._process(t);
            n_processed += n;
        } while(n);
    }

#if defined(POOYA_DEBUG)
    std::vector<const Block*> unprocessed;

    auto find_unprocessed_cb = [&] (const Block& c, uint32_t /*level*/) -> bool
    {
        if (!c.processed()) {unprocessed.push_back(&c);}
        return true;
    };

    _model.traverse(find_unprocessed_cb, 0);
    if (unprocessed.size())
    {
        std::cout << "\n-- unprocessed blocks detected:\n";
        for (const auto& c: unprocessed)
        {
            std::cout << "- " << c->full_name() << "\n";
            for (auto& sig_type: c->associated_signals())
            {
                if ((sig_type.second != Block::SignalAssociationType::Input) || sig_type.first->is_assigned()) {continue;}
                std::cout << "  - " << sig_type.first->_full_name << "\n";
            }
        }
    }
#endif // defined(POOYA_DEBUG)

    return n_processed;
}

void Simulator::init(double t0)
{
    pooya_trace("t0: " + std::to_string(t0));

    if (!_model.is_initialized())
    {
        _model.init();
    }

    std::size_t state_variables_size{0};

    std::unordered_set<ValueSignalId> value_signals;
    std::unordered_set<ScalarSignalId> scalar_state_signals;
    std::unordered_set<ArraySignalId> array_state_signals;

    _model.traverse(
        [&](Block& block, uint32_t /*level*/) -> bool
        {
            const auto& signals = block.associated_signals();
            for(auto& sig: signals)
            {
                value_signals.insert(sig.first);

                if (!sig.first->is_float() || !sig.first->as_float().is_state_variable()) {continue;}
                if (sig.first->is_scalar())
                {
                    if (scalar_state_signals.insert(std::static_pointer_cast<ScalarSignalInfo>(sig.first->shared_from_this())).second)
                    {
                        state_variables_size++;
                    }
                }
                else if (sig.first->is_array())
                {
                    if (array_state_signals.insert(std::static_pointer_cast<ArraySignalInfo>(sig.first->shared_from_this())).second)
                    {
                        state_variables_size += sig.first->as_array().size();
                    }
                }
                else
                {
                    helper::pooya_throw_exception(__FILE__, __LINE__, "Unknown state signal type!");
                }
            }

            return true;
        },
        0);

    value_signals_.reserve(value_signals.size());
    for (auto& sig: value_signals) {value_signals_.emplace_back(sig);}
    scalar_state_signals_.reserve(scalar_state_signals.size());
    for (auto& sig: scalar_state_signals) {scalar_state_signals_.emplace_back(sig);}
    array_state_signals_.reserve(array_state_signals.size());
    for (auto& sig: array_state_signals) {array_state_signals_.emplace_back(sig);}

    _state_variables.resize(state_variables_size);
    _state_variables_orig.resize(state_variables_size);
    _state_variable_derivs.resize(state_variables_size);

    _t_prev = t0;

    if (state_variables_size > 0)
    {
        if (_reuse_order)
        {
            assert(_processing_order1.empty());
            assert(_processing_order2.empty());

            uint num_blocks = 0;
            auto enum_blocks_cb = [&] (Block& /*c*/, uint32_t /*level*/) -> bool
            {
                num_blocks++;
                return true;
            };
            _model.traverse(enum_blocks_cb, 0);

            _processing_order1.reserve(num_blocks);
            _processing_order2.reserve(num_blocks);

            auto add_blocks_cb = [&] (Block& c, uint32_t /*level*/) -> bool
            {
#if defined(POOYA_USE_SMART_PTRS)
                _processing_order1.emplace_back(c);
#else // defined(POOYA_USE_SMART_PTRS)
                _processing_order1.emplace_back(&c);
#endif // defined(POOYA_USE_SMART_PTRS)
                return true;
            };
            _model.traverse(add_blocks_cb, 0);

#if defined(POOYA_USE_SMART_PTRS)
            _current_po = _processing_order1;
            _new_po = _processing_order2;
#else // defined(POOYA_USE_SMART_PTRS)
            _current_po = &_processing_order1;
            _new_po = &_processing_order2;
#endif // defined(POOYA_USE_SMART_PTRS)
        }
    }
    else
    {
        if (_stepper)
        {
            helper::pooya_show_warning(__FILE__, __LINE__,
                "A stepper is provided but no state variable is defined! The stepper will be ignored.");
        }
    }

    for (auto& sig: value_signals_) {sig->clear();}
    if (_inputs_cb) {_inputs_cb(_model, t0);}
    _model.input_cb(t0);
    _model.pre_step(t0);
    _process(t0);
    _model.post_step(t0);

    _initialized = true;
}

void Simulator::run(double t, double min_time_step, double max_time_step)
{
    pooya_trace("t: " + std::to_string(t));
    auto stepper_callback = [&](double t, const Array& state_variables) -> const Array&
    {
        pooya_trace("t: " + std::to_string(t));
        reset_with_state_variables(state_variables);
        if (_inputs_cb) {_inputs_cb(_model, t);}
        _model.input_cb(t);
        _process(t);

        double* data = _state_variable_derivs.data();
        for (auto& sig: scalar_state_signals_)
        {
            *data = sig->deriv_signal()->as_scalar().get();
            data++;
        }
        for (auto& sig: array_state_signals_)
        {
            auto& deriv_sig = sig->deriv_signal();
            Eigen::Map<Array>(data, deriv_sig->size()) = deriv_sig->as_array().get();
            data += deriv_sig->size();
        }

        return _state_variable_derivs;
    };

    if (!_initialized)
    {
        // treat the first call as the initialization call if init(t0) was not called explicitely
        init(t);
        return;
    }

    if (_state_variables.size() > 0)
    {
        if (t <_t_prev)
        {
            helper::pooya_throw_exception(__FILE__, __LINE__,
                "Simulation cannot go back in time. Aborting!\n  "
                "- current time  = " + std::to_string(t) + "\n  "
                "- previous time = " + std::to_string(_t_prev) + "\n");
        }

        if (t == _t_prev)
        {
            helper::pooya_show_warning(__FILE__, __LINE__, 
                "Repeated simulation step:\n  "
                "- time = " + std::to_string(t) + "\n");

            for (auto& sig: value_signals_) {sig->clear();}
            if (_inputs_cb) {_inputs_cb(_model, t);}
            _model.input_cb(t);
            _model.pre_step(t);
            get_state_variables(_state_variables);
        }
        else
        {
            assert(_stepper);

            assert(min_time_step > 0);
            assert(max_time_step > min_time_step);

            double new_h;
            double t1 = _t_prev;
            double t2 = t;
            bool force_accept = false;
            while (t1 < t)
            {
                for (auto& sig: value_signals_) {sig->clear();}
                if (_inputs_cb) {_inputs_cb(_model, t1);}
                _model.input_cb(t1);
                _model.pre_step(t1);
                get_state_variables(_state_variables_orig);


#if defined(POOYA_USE_SMART_PTRS)
                _stepper.value().get().step
#else // defined(POOYA_USE_SMART_PTRS)step
                _stepper->step
#endif // defined(POOYA_USE_SMART_PTRS)
                    (stepper_callback, t1, _state_variables_orig, t2, _state_variables, new_h);

                double h = t2 - t1;
                if (force_accept || (new_h >= h) || (h <= min_time_step))
                {
                    // accept this step
                    force_accept = false;
                    new_h = std::max(min_time_step, std::min(new_h, max_time_step));
                    t1 = t2;
                    t2 = std::min(t1 + new_h, t);

                    if (t1 < t)
                    {
                        reset_with_state_variables(_state_variables);
                        if (_inputs_cb) {_inputs_cb(_model, t1);}
                        _model.input_cb(t1);
                        _process(t1);
                        _model.post_step(t1);
                    }
                }
                else
                {
                    // redo this step
                    force_accept = new_h <= min_time_step;
                    new_h = std::max(min_time_step, std::min(new_h, max_time_step));
                    t2 = t1 + new_h;
                }
            }
        }
    }

    reset_with_state_variables(_state_variables);
    if (_inputs_cb) {_inputs_cb(_model, t);}
    _model.input_cb(t);
    if (_state_variables.size() == 0)
        {_model.pre_step(t);}
    _process(t);
    _model.post_step(t);

    _t_prev = t;
}

void Simulator::reset_with_state_variables(const Array& state_variables)
{
    pooya_trace0;
    for (auto& sig: value_signals_) {sig->clear();}
    const double* data = state_variables.data();
    for (auto& sig: scalar_state_signals_)
    {
        sig->set(*data);
        data++;
    }
    for (auto& sig: array_state_signals_)
    {
        sig->set(Eigen::Map<const Array>(data, sig->size()));
        data += sig->size();
    }
}

void Simulator::get_state_variables(Array& state_variables)
{
    pooya_trace0;
    pooya_verify(state_variables.size() == _state_variables.size(), "Incorrect output array size!");
    double* data = state_variables.data();
    for (auto& sig: scalar_state_signals_)
    {
        *data = sig->get();
        data++;
    }
    for (auto& sig: array_state_signals_)
    {
        Eigen::Map<Array>(data, sig->size()) = sig->get();
        data += sig->size();
    }
}

}
