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

#include <iostream>
#include <fstream>
#include <string>

#include "signal.hpp"
#include "util.hpp"
#include "helper.hpp"

namespace pooya
{

void History::track_all()
{
    pooya_verify(empty(), "track_all should be called before the history is updated!");
    _signals.clear();
    std::size_t n = 0;
#if defined(POOYA_USE_SMART_PTRS)
    for (auto& sig: _model.get().signals())
#else // defined(POOYA_USE_SMART_PTRS)
    for (auto sig: _model.signals())
#endif // defined(POOYA_USE_SMART_PTRS)
        {if (sig->is_value()) {n++;}}
    _signals.reserve(n);
#if defined(POOYA_USE_SMART_PTRS)
    for (auto& sig: _model.get().signals())
#else // defined(POOYA_USE_SMART_PTRS)
    for (auto sig: _model.signals())
#endif // defined(POOYA_USE_SMART_PTRS)
        {if (sig->is_value()) {_signals.push_back(sig->as_value());}}
}

void History::track(SignalId sig)
{
    pooya_verify(empty(), "track should be called before the history is updated!");
    pooya_verify_value_signal(sig);
    if (std::find(_signals.begin(), _signals.end(), sig) == _signals.end())
        {_signals.push_back(sig->as_value());}
}

void History::untrack(SignalId sig)
{
    pooya_verify(empty(), "untrack should be called before the history is updated!");
    pooya_verify_value_signal(sig);
    _signals.erase(std::find(_signals.begin(), _signals.end(), sig));
}

void History::update(uint k, double t)
{
    pooya_trace("k = " + std::to_string(k));
    if (empty())
    {
        if (_signals.empty()) {track_all();}
        for (auto sig: _signals)
        {
            if (sig->is_scalar() || sig->is_int() || sig->is_bool())
                {insert_or_assign(sig, Array(_nrows_grow));}
            else
                {insert_or_assign(sig, Eigen::MatrixXd(_nrows_grow, sig->as_array()->size()));}
        }
    }

    if (k >= _time.rows())
        {_time.conservativeResize(k + _nrows_grow, Eigen::NoChange);}
    _time(k, 0) = t;
    for (auto sig: _signals)
    {
        auto& h = at(sig);
        if (k >= h.rows())
            {h.conservativeResize(k + _nrows_grow, Eigen::NoChange);}
        bool valid = sig->is_assigned();
        if (sig->is_int())
            {h(k, 0) = valid ? sig->as_int()->get() : 0;}
        else if (sig->is_bool())
            {h(k, 0) = valid ? sig->as_bool()->get() : 0;}
        else if (sig->is_scalar())
            {h(k, 0) = valid ? sig->as_scalar()->get() : 0;}
        else
        {
            if (valid)
                {h.row(k) = sig->as_array()->get();}
            else
                {h.row(k).setZero();}
        }
    }

    if ((_bottom_row == uint(-1)) || (k > _bottom_row)) {_bottom_row = k;}
}

void History::shrink_to_fit()
{
    pooya_trace0;
    const uint nrows = _bottom_row + 1;

    if (nrows >= _time.rows()) {return;} // practically, nrows can't be the greater

    _time.conservativeResize(nrows, Eigen::NoChange);
    for (auto& p: *this)
        {p.second.conservativeResize(nrows, Eigen::NoChange);}
}

void History::export_csv(const std::string& filename)
{
    pooya_trace("filename = " + filename);
    if (size() == 0) {return;}

    std::ofstream ofs(filename);

    // header
    ofs << "time";
    for (const auto& h: *this)
    {
        if (h.first)
        {
            if (h.first->is_array())
            {
                auto sig = h.first->as_array();
                for (std::size_t k=0; k < sig->size(); k++)
                    {ofs << "," << h.first->_full_name << "[" << k << "]";}
            }
            else
            {
                ofs << "," << h.first->_full_name;
            }
        }
    }
    ofs << "\n";

    // values
    auto n = time().size();
    for (int k = 0; k < n; k++)
    {
        ofs << time()(k);
        for (const auto& h: *this)
        {
            if (h.first)
            {
                if (h.first->is_array())
                {
                    auto sig = h.first->as_array();
                    for (std::size_t j=0; j < sig->size(); j++)
                        {ofs << "," << h.second(k, j);}
                }
                else
                {
                    ofs << "," << h.second(k);
                }
            }
        }
        ofs << "\n";
    }
}

bool arange(uint k, double& t, double t_init, double t_end, double dt)
{
    t_init += k * dt;
    return (t_init <= t_end) && (t = t_init, true);
}

Simulator::Simulator(Model& model, InputCallback inputs_cb, StepperBase* stepper, bool reuse_order) :
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
#if defined(POOYA_USE_SMART_PTRS)
    _model.get()._mark_unprocessed();
#else // defined(POOYA_USE_SMART_PTRS)
    _model._mark_unprocessed();
#endif // defined(POOYA_USE_SMART_PTRS)

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
#if defined(POOYA_USE_SMART_PTRS)
            n = _model.get()._process(t);
#else // defined(POOYA_USE_SMART_PTRS)
            n = _model._process(t);
#endif // defined(POOYA_USE_SMART_PTRS)
            n_processed += n;
        } while(n);
    }

#if !defined(POOYA_NDEBUG)
    std::vector<const Block*> unprocessed;

    auto find_unprocessed_cb = [&] (const Block& c, uint32_t /*level*/) -> bool
    {
        if (!c.processed()) {unprocessed.push_back(&c);}
        return true;
    };

#if defined(POOYA_USE_SMART_PTRS)
    _model.get().traverse(find_unprocessed_cb, 0);
#else // defined(POOYA_USE_SMART_PTRS)
    _model.traverse(find_unprocessed_cb, 0);
#endif // defined(POOYA_USE_SMART_PTRS)
    if (unprocessed.size())
    {
        std::cout << "\n-- unprocessed blocks detected:\n";
        for (const auto& c: unprocessed)
        {
            std::cout << "- " << c->full_name() << "\n";
            for (auto sig: c->dependencies())
            {
                if (sig->is_assigned()) {continue;}
                std::cout << "  - " << sig->_full_name << "\n";
            }
        }
    }
#endif // !defined(POOYA_NDEBUG)

    return n_processed;
}

void Simulator::init(double t0)
{
    pooya_trace("t0: " + std::to_string(t0));

    static_cast<Model&>(_model).lock_signals();

    _t_prev = t0;

    if (static_cast<Model&>(_model).values().num_state_variables() > 0)
    {
        if (_reuse_order)
        {
            assert(_processing_order1.empty());
            assert(_processing_order2.empty());

            uint num_blocks = 0;
            auto enum_blocks_cb = [&] (Block& c, uint32_t /*level*/) -> bool
            {
                num_blocks++;
                return true;
            };
            static_cast<Model&>(_model).traverse(enum_blocks_cb, 0);

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
            static_cast<Model&>(_model).traverse(add_blocks_cb, 0);

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
            util::pooya_show_warning(__FILE__, __LINE__,
                "A stepper is provided but no state variable is defined! The stepper will be ignored.");
        }
    }

    static_cast<Model&>(_model).invalidate();
    if (_inputs_cb) {_inputs_cb(_model, t0);}
    static_cast<Model&>(_model).input_cb(t0);
    static_cast<Model&>(_model).pre_step(t0);
    _process(t0);
    static_cast<Model&>(_model).post_step(t0);

    _initialized = true;
}

void Simulator::run(double t, double min_time_step, double max_time_step)
{
    pooya_trace("t: " + std::to_string(t));
    auto stepper_callback = [&](double t, const Array& state_variables) -> const ValuesArray::StateVariableDerivs&
    {
        pooya_trace("t: " + std::to_string(t));
        static_cast<Model&>(_model).reset_with_state_variables(state_variables);
        if (_inputs_cb) {_inputs_cb(_model, t);}
        static_cast<Model&>(_model).input_cb(t);
        _process(t);
        return static_cast<Model&>(_model).values().state_variable_derivs();
    };

    if (!_initialized)
    {
        // treat the first call as the initialization call if init(t0) was not called explicitely
        init(t);
        return;
    }

    if (static_cast<Model&>(_model).values().num_state_variables() > 0)
    {
        if (t <_t_prev)
        {
            util::pooya_throw_exception(__FILE__, __LINE__,
                "Simulation cannot go back in time. Aborting!\n  "
                "- current time  = " + std::to_string(t) + "\n  "
                "- previous time = " + std::to_string(_t_prev) + "\n");
        }

        if (t == _t_prev)
        {
            util::pooya_show_warning(__FILE__, __LINE__, 
                "Repeated simulation step:\n  "
                "- time = " + std::to_string(t) + "\n");

            static_cast<Model&>(_model).invalidate();
            if (_inputs_cb) {_inputs_cb(_model, t);}
            static_cast<Model&>(_model).input_cb(t);
            static_cast<Model&>(_model).pre_step(t);
            _state_variables = static_cast<Model&>(_model).values().state_variables();
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
                static_cast<Model&>(_model).invalidate();
                if (_inputs_cb) {_inputs_cb(_model, t1);}
                static_cast<Model&>(_model).input_cb(t1);
                static_cast<Model&>(_model).pre_step(t1);
                _state_variables_orig = static_cast<Model&>(_model).values().state_variables();

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
                        static_cast<Model&>(_model).reset_with_state_variables(_state_variables);
                        if (_inputs_cb) {_inputs_cb(_model, t1);}
                        static_cast<Model&>(_model).input_cb(t1);
                        _process(t1);
                        static_cast<Model&>(_model).post_step(t1);
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

    static_cast<Model&>(_model).reset_with_state_variables(_state_variables);
    if (_inputs_cb) {_inputs_cb(_model, t);}
    static_cast<Model&>(_model).input_cb(t);
    if (static_cast<Model&>(_model).values().num_state_variables() == 0)
        {static_cast<Model&>(_model).pre_step(t);}
    _process(t);
    static_cast<Model&>(_model).post_step(t);

    _t_prev = t;
}

}
