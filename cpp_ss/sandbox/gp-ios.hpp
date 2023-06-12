
#ifndef __GP_IOS_HPP__
#define __GP_IOS_HPP__

#include "../3rdparty/eigen/Eigen/Core"
#include "gnuplot-iostream.h"

using namespace Eigen;

namespace gnuplotio
{

template<>
class ArrayTraits<VectorXd>
{
public:
    static constexpr int depth = 1;

	typedef IteratorRange<typename VectorXd::const_iterator, typename VectorXd::value_type> range_type;

	static range_type get_range(const VectorXd& arg)
    {
		return range_type(arg.begin(), arg.end());
	}
};

template<> std::string Gnuplot::file1d(const MatrixXd &arg, const std::string &filename)
{
    return file1d(VectorXd{arg.reshaped()}, filename);
}

}

#endif // __GP_IOS_HPP__
