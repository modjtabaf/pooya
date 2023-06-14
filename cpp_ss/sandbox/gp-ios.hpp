
#ifndef __GP_IOS_HPP__
#define __GP_IOS_HPP__

#include "blocks.hpp"

#include "../3rdparty/eigen/Eigen/Core"
#include "gnuplot-iostream.h"

using namespace blocks;
using namespace Eigen;

namespace gnuplotio
{

template<>
class ArrayTraits<Signal>
{
public:
    static constexpr int depth = 1;

	typedef IteratorRange<typename Signal::const_iterator, typename Signal::value_type> range_type;

	static range_type get_range(const Signal& arg)
    {
		return range_type(arg.begin(), arg.end());
	}
};

template<> std::string Gnuplot::file1d(const MatrixXd &arg, const std::string &filename)
{
    return file1d(Signal{arg.reshaped()}, filename);
}

}

#endif // __GP_IOS_HPP__
