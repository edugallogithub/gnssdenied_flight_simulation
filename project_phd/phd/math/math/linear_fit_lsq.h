#ifndef MATH_LINEAR_FIT_LSQ
#define MATH_LINEAR_FIT_LSQ

#include "../math.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace math {

/*
 * Implements a linear least squares fitting with a straight line
 */

// CLASS LINEAR_FIT_LSQ
// ====================
// ====================

class MATH_API linear_fit_lsq {
private:

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< default constructor */
    linear_fit_lsq() = delete;
    /**< copy constructor */
    linear_fit_lsq(const linear_fit_lsq&) = delete;
    /**< move constructor */
    linear_fit_lsq(linear_fit_lsq&&) = delete;
    /**< destructor */
    ~linear_fit_lsq() = default;
    /**< copy assignment */
    linear_fit_lsq& operator=(const linear_fit_lsq&) = delete;
    /**< move assignment */
    linear_fit_lsq& operator=(linear_fit_lsq&&) = delete;

    /**< XXXXXXXXXXXXXXXXXXXXXXXXXXx */
    static void compute(Eigen::Vector2d& x, double& err_mean, double& err_std, Eigen::VectorXd& err, const Eigen::MatrixXd& in, const Eigen::MatrixXd& out);
}; // closes class linear_fit_lsq


} // closes namespace math

#endif

