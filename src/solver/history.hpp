/*
Copyright 2024 Mojtaba (Moji) Fathi

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

#ifndef __POOYA_HELPER_HISTORY_HPP__
#define __POOYA_HELPER_HISTORY_HPP__

#include <sys/types.h>
#include <unordered_map>
#include <vector>

#include "src/signal/array.hpp"
#include "src/signal/signal.hpp"
#include "src/signal/value_signal.hpp"

namespace pooya
{

class Block;

class History : public std::unordered_map<std::shared_ptr<ValueSignalImpl>, Eigen::MatrixXd>
{
    using Base = std::unordered_map<std::shared_ptr<ValueSignalImpl>, Eigen::MatrixXd>;

protected:
    uint _nrows_grow;
    uint _bottom_row{static_cast<uint>(-1)};
    Array _time;
    std::vector<std::shared_ptr<ValueSignalImpl>> _signals;

public:
    History(uint nrows_grow = 1000) : _nrows_grow(nrows_grow), _time(nrows_grow) {}

    bool track(const Signal& sig);
    void untrack(const Signal& sig);
    void update(uint k, double t);
    void export_csv(const std::string& filename);
    void shrink_to_fit();
    uint nrows() const { return _bottom_row + 1; }
    const Array& time() const { return _time; }

    const Eigen::MatrixXd& operator[](const Signal& sig) const
    {
        auto ptr = std::dynamic_pointer_cast<ValueSignalImpl>(sig->shared_from_this());
        pooya_verify(ptr, "value signal expected!") return at(ptr);
    }
};

} // namespace pooya

#endif // __POOYA_HELPER_HISTORY_HPP__
