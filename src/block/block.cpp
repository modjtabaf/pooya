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

#include <cassert>
#include <memory>

#include "block.hpp"
#include "src/helper/util.hpp"
#include "src/signal/value_signal.hpp"
#include "submodel.hpp"

namespace pooya
{

Block::Block(uint16_t num_iports, uint16_t num_oports) : _num_iports(num_iports), _num_oports(num_oports)
{
    if (_parent) _parent->link_block(*this);
}

Block::Block(Submodel* parent, std::string_view name, uint16_t num_iports, uint16_t num_oports)
    : NamedObject(name), _parent(parent), _num_iports(num_iports), _num_oports(num_oports)
{
    if (_parent) _parent->link_block(*this);
}

bool Block::set_parent(Submodel& parent)
{
    if (_parent != nullptr && _parent != &parent)
    {
        helper::pooya_show_warning(__FILE__, __LINE__, full_name().str() + ": parent is set already!");
        return false;
    }

    _parent = &parent;
    parent.link_block(*this);

    return true;
}

bool Block::connect(const Bus& ibus, const Bus& obus)
{
    pooya_trace(_name.str());

    if (_connected)
    {
        helper::pooya_throw_exception(__FILE__, __LINE__, _name.str() + ": illegal attempt to reconnecting a block!");
    }

    pooya_verify((_num_iports == NoIOLimit) || (ibus.size() == _num_iports),
                 _num_iports == 0 ? full_name().str() + " cannot take any input."
                                  : full_name().str() + " requires " + std::to_string(_num_iports) +
                                        std::string(" input signal") + (_num_iports == 1 ? "." : "s."));

    pooya_verify((_num_oports == NoIOLimit) || (obus.size() == _num_oports),
                 _num_oports == 0 ? full_name().str() + " cannot generate any output."
                                  : full_name().str() + " requires " + std::to_string(_num_oports) +
                                        std::string(" output signal") + (_num_oports == 1 ? "." : "s."));

    _linked_signals.reserve(ibus.size() + obus.size());

    for (const auto& sig_key : ibus)
    {
        const auto& sig = ibus[sig_key];
        if (sig->is_value())
        {
            link_signal(std::static_pointer_cast<ValueSignalImpl>(sig->shared_from_this()), SignalLinkType::Input);
        }
    }

    for (const auto& sig_key : obus)
    {
        const auto& sig = obus[sig_key];
        if (sig->is_value())
        {
            link_signal(std::static_pointer_cast<ValueSignalImpl>(sig->shared_from_this()), SignalLinkType::Output);
        }
    }

    _linked_signals.shrink_to_fit();

    _ibus.reset(ibus);
    _obus.reset(obus);

    _connected = true;
    return true;
}

ValidName Block::full_name() const
{
    return (_parent ? _parent->full_name() : ValidName()) / _name;
}

bool Block::link_signal(const ValueSignalImplPtr& signal, SignalLinkType type)
{
    pooya_trace("block: " + full_name().str());
    pooya_verify_valid_signal(signal);
    auto it = std::find_if(_linked_signals.begin(), _linked_signals.end(),
                           [&](const SignalLinkPair& sig_type) -> bool { return sig_type.first == signal; });
    if (it == _linked_signals.end())
    {
        _linked_signals.emplace_back(signal, type);
        return true;
    }

    return false;
}

void Block::_mark_unprocessed()
{
    _processed = false;
}

} // namespace pooya
