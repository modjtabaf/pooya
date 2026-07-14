/*
Copyright 2025 Mojtaba (Moji) Fathi

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

#ifndef __POOYA_SOLVER_FAST_SIMULATOR_HPP__
#define __POOYA_SOLVER_FAST_SIMULATOR_HPP__

#include <cstddef>
#include <optional>

#include "misc/BS_thread_pool.hpp"
#include "simulator_base.hpp"

namespace pooya
{

class Leaf;

class FastSimulator : public SimulatorBase
{
public:
    enum NumThreads : std::size_t
    {
        Auto   = 0,
        Single = 1,
    };

    explicit FastSimulator(Block& model, SimulatorBase::InputCallback inputs_cb = nullptr,
                           StepperBase* stepper = nullptr, std::size_t num_threads = Single)
        : SimulatorBase(model, inputs_cb, stepper)
    {
        if (num_threads != Single) _thread_pool.emplace(num_threads);
    }
    FastSimulator(const FastSimulator&) = delete; // no copy constructor
    virtual ~FastSimulator()            = default;

    void init(double t0 = 0.0) override;

protected:
    std::vector<std::vector<Leaf*>> _processing_order;
    std::optional<BS::thread_pool<>> _thread_pool;

    void process_model(double t, bool call_pre_step, bool call_post_step) override;
};

} // namespace pooya

#endif // __POOYA_SOLVER_FAST_SIMULATOR_HPP__
