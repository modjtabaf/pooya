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

#include <cassert>
#include <memory>

#include "block.hpp"
#include "submodel.hpp"
#include "src/signal/value_signal.hpp"
#include "src/helper/util.hpp"

namespace pooya
{

bool Block::init(Submodel* parent, BusId ibus, BusId obus)
{
    pooya_trace(_given_name);

    assert(!_parent);
    if (_parent) {return false;}
    _parent = parent;

    pooya_verify(!parent || parent->is_initialized(), _given_name + ": parent block is not initialized yet!");

    _given_name = make_valid_given_name(_given_name);
    if (_full_name.empty())
    {
        _full_name = (parent ? parent->full_name() : "") + "/" + _given_name;
    }

    pooya_verify((_num_iports == NoIOLimit) || (!ibus && _num_iports == 0) || (ibus && ibus->size() == _num_iports),
        _num_iports == 0 ?
            _full_name + " cannot take any input." :
            _full_name + " requires " + std::to_string(_num_iports) + std::string(" input signal") + (_num_iports == 1 ? "." : "s."));

    pooya_verify((_num_oports == NoIOLimit) || (!obus && _num_oports == 0) || (obus && obus->size() == _num_oports),
        _num_oports == 0 ?
            _full_name + " cannot generate any output." :
            _full_name + " requires " + std::to_string(_num_oports) + std::string(" output signal") + (_num_oports == 1 ? "." : "s."));

    _associated_signals.reserve((ibus ? ibus->size() : 0) + (obus ? obus->size() : 0));

    if (ibus)
    {
        for (auto sig: *ibus)
        {
            if (sig.second->is_value())
            {
                register_associated_signal(std::static_pointer_cast<ValueSignalInfo>(sig.second->shared_from_this()), SignalAssociationType::Input);
            }
        }
    }

    if (obus)
    {
        for (auto sig: *obus)
        {
            if (sig.second->is_value())
            {
                register_associated_signal(std::static_pointer_cast<ValueSignalInfo>(sig.second->shared_from_this()), SignalAssociationType::Output);
            }
        }
    }

    _associated_signals.shrink_to_fit();

    _ibus = ibus;
    _obus = obus;

    _initialized = true;
    return true;
}

bool Block::register_associated_signal(ValueSignalId signal, SignalAssociationType type)
{
    pooya_trace("block: " + full_name());
    pooya_verify_valid_signal(signal);
    auto it = std::find_if(_associated_signals.begin(), _associated_signals.end(),
        [&](const SignalAssociationPair& sig_type) -> bool
        {
            return sig_type.first == signal;
        });
    if (it == _associated_signals.end())
    {
        _associated_signals.emplace_back(signal, type);
        return true;
    }

    return false;
}

void Block::_mark_unprocessed()
{
    _processed = false;
}

uint Block::_process(double t, bool /*go_deep*/)
{
    pooya_trace("block: " + full_name());
    if (_processed) {return 0;}
    for (auto& sig: _associated_signals)
    {
        if ((sig.second == SignalAssociationType::Input) && !sig.first->is_assigned()) {return 0;}
    }

    activation_function(t);

    _processed = true;
    return 1;
}

std::string Block::make_valid_given_name(const std::string& given_name)
{
    pooya_trace(given_name);

    std::string ret;
    if (given_name.empty())
    {
        ret = "unnamed_block";
    }
    else
    {
        ret = given_name;
        std::replace(ret.begin(), ret.end(), ' ', '_');
        std::replace(ret.begin(), ret.end(), '~', '_');
        std::replace(ret.begin(), ret.end(), '/', '_');
    }

    if (_parent)
    {
        uint n = 0;

        std::string foo{ret};
        auto pooya_verify_unique_name_cb = [&] (const pooya::Block& c, uint32_t level) -> bool
        {
            return (level == 0) || (c.given_name() != foo);
        };

        while (!_parent->traverse(pooya_verify_unique_name_cb, 0, 1))
            {foo = ret + "_" + std::to_string(n++);}

        ret = foo;
    }

    return ret;
}

}
