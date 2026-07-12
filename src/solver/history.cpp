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

#include <fstream>
#include <memory>
#include <string>

#include "history.hpp"
#include "src/block/block.hpp"

namespace pooya
{

bool History::track(const Signal& sig)
{
    pooya_verify(empty(), "track should be called before the history is updated!");

    auto vsig = std::dynamic_pointer_cast<ValueSignalImpl>(sig->shared_from_this());
    if (!vsig || std::find(_signals.begin(), _signals.end(), vsig) != _signals.end()) return false;

    _signals.emplace_back(std::move(vsig));
    return true;
}

void History::untrack(const Signal& sig)
{
    pooya_verify(empty(), "untrack should be called before the history is updated!");

    auto vsig = std::dynamic_pointer_cast<ValueSignalImpl>(sig->shared_from_this());
    if (vsig) _signals.erase(std::find(_signals.begin(), _signals.end(), vsig));
}

void History::update(uint k, double t)
{
    pooya_trace("k = " + std::to_string(k));
    if (empty())
    {
        if (_signals.empty())
        {
            return;
        }
        for (auto& sig : _signals)
        {
            if (dynamic_cast<ScalarSignalImpl*>(sig.get())
#if DEFINE_INT_SIGNAL != 0
                || dynamic_cast<IntSignalImpl*>(sig.get())
#endif // DEFINE_INT_SIGNAL != 0
#if DEFINE_BOOL_SIGNAL != 0
                || dynamic_cast<BoolSignalImpl*>(sig.get())
#endif // DEFINE_BOOL_SIGNAL != 0
            )
            {
                insert_or_assign(sig, Array(_nrows_grow));
            }
#if DEFINE_ARRAY_SIGNAL != 0
            else if (auto* pa = dynamic_cast<ArraySignalImpl*>(sig.get()); pa)
            {
                insert_or_assign(sig, Eigen::MatrixXd(_nrows_grow, pa->size()));
            }
#endif // DEFINE_ARRAY_SIGNAL != 0
        }
    }

    if (k >= _time.rows())
    {
        _time.conservativeResize(k + _nrows_grow, Eigen::NoChange);
    }
    _time(k, 0) = t;
    for (auto& sig : _signals)
    {
        auto& h = at(sig);
        if (k >= h.rows())
        {
            h.conservativeResize(k + _nrows_grow, Eigen::NoChange);
        }
        bool valid = sig->assigned();
        if (auto* ps = dynamic_cast<ScalarSignalImpl*>(sig.get()); ps)
        {
            h(k, 0) = valid ? ps->get_value() : 0;
        }
#if DEFINE_INT_SIGNAL != 0
        else if (auto* pi = dynamic_cast<IntSignalImpl*>(sig.get()); pi)
        {
            h(k, 0) = valid ? pi->get_value() : 0;
        }
#endif // DEFINE_INT_SIGNAL != 0
#if DEFINE_BOOL_SIGNAL != 0
        else if (auto* pb = dynamic_cast<BoolSignalImpl*>(sig.get()); pb)
        {
            h(k, 0) = valid ? pb->get_value() : 0;
        }
#endif // DEFINE_BOOL_SIGNAL != 0
#if DEFINE_ARRAY_SIGNAL != 0
        else if (auto* pa = dynamic_cast<ArraySignalImpl*>(sig.get()); pa)
        {
            if (valid)
            {
                h.row(k) = pa->get_value();
            }
            else
            {
                h.row(k).setZero();
            }
        }
#endif // DEFINE_ARRAY_SIGNAL != 0
    }

    if ((_bottom_row == uint(-1)) || (k > _bottom_row))
    {
        _bottom_row = k;
    }
}

void History::shrink_to_fit()
{
    pooya_trace0;
    const uint nrows = _bottom_row + 1;

    if (nrows >= _time.rows())
    {
        return;
    } // practically, nrows can't be the greater

    _time.conservativeResize(nrows, Eigen::NoChange);
    for (auto& p : *this)
    {
        p.second.conservativeResize(nrows, Eigen::NoChange);
    }
}

void History::export_csv(const std::string& filename)
{
    pooya_trace("filename = " + filename);
    if (size() == 0)
    {
        return;
    }

    std::ofstream ofs(filename);

    // header
    ofs << "time";
    for (const auto& h : *this)
    {
        if (h.first)
        {
#if DEFINE_ARRAY_SIGNAL != 0
            if (auto* pa = dynamic_cast<ArraySignalImpl*>(h.first.get()); pa)
            {
                for (std::size_t k = 0; k < pa->size(); k++)
                {
                    ofs << "," << h.first->name().str() << "[" << k << "]";
                }
            }
            else
#endif // DEFINE_ARRAY_SIGNAL != 0
            {
                ofs << "," << h.first->name().str();
            }
        }
    }
    ofs << "\n";

    // values
    auto n = time().size();
    for (int k = 0; k < n; k++)
    {
        ofs << time()(k);
        for (const auto& h : *this)
        {
            if (h.first)
            {
#if DEFINE_ARRAY_SIGNAL != 0
                if (auto* pa = dynamic_cast<ArraySignalImpl*>(h.first.get()); pa)
                {
                    for (std::size_t j = 0; j < pa->size(); j++)
                    {
                        ofs << "," << h.second(k, j);
                    }
                }
                else
#endif // DEFINE_ARRAY_SIGNAL != 0
                {
                    ofs << "," << h.second(k);
                }
            }
        }
        ofs << "\n";
    }
}

} // namespace pooya
