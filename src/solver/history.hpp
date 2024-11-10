/*
Copyright 2024 Mojtaba (Moji) Fathi

 Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the “Software”), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
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

class History : public std::unordered_map<SignalId, Eigen::MatrixXd>
{
protected:
    const Block& _model;
    uint _nrows_grow;
    uint _bottom_row{static_cast<uint>(-1)};
    Array _time;
    std::vector<ValueSignalId> _signals;

public:
    History(const Block& model, uint nrows_grow = 1000) : _model(model), _nrows_grow(nrows_grow), _time(nrows_grow) {}

    void track(SignalId sig);
    void untrack(SignalId sig);
    void update(uint k, double t);
    void export_csv(const std::string& filename);
    void shrink_to_fit();
    uint nrows() const { return _bottom_row + 1; }
    const Array& time() const { return _time; }
};

} // namespace pooya

#endif // __POOYA_HELPER_HISTORY_HPP__
