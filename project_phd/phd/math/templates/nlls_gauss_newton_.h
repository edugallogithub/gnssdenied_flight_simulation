
#ifndef MATH_NLLS_GAUSS_NEWTON
#define MATH_NLLS_GAUSS_NEWTON

#include "../math.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
//#include <cmath>
//#include <numeric>
//#include <vector>

namespace math {

// TEMPLATE NON LINEAR LEAST SQUARES GAUSS-NEWTON SOLVER
// =====================================================
// =====================================================

/* State vector x of size siz_x.
 * Vector of function u(x) of siz_u.
 * Cost function f(x) of size 1 is sum of squares of u(x).
 * Jacobian of u(x) is matrix of siz_x rows and siz_u columns.
 */

template <unsigned short siz_x, unsigned short siz_u> class nlls_gauss_newton_ {
private:
    /**< current value of state vector */
    Eigen::Matrix<double,siz_x,1> _x;
    /**< current value of component functions */
    Eigen::Matrix<double,siz_u,1> _u;
    /**< current value of cost function */
    double _ssq;
    /**< current value of jacobian */
    Eigen::Matrix<double,siz_x,siz_u> _J;

    /**< provide initial conditions */
//////    virtual Eigen::Matrix<double,siz_x,1> init() = 0;
    /**< evaluates component functions based on state vector x */
//////    virtual Eigen::Matrix<double,siz_u,1> eval(const Eigen::Matrix<double,siz_x,1>& x) = 0;
    /**< computes the cost function, this is, the sum of squares of the component functions */
//////    virtual double ssq() const = 0;
    /**< computes the jacobian of the component functions with respect to the state vector */
//////    virtual Eigen::Matrix<double,siz_x,siz_u> jacobian(const Eigen::Matrix<double,siz_x,1>& x) const = 0;

    ////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////

    /**< Hessian approximation == information matrix (equal to the inverse covariance matrix) */
//    Eigen::Matrix<double, D, D> _H;
    /**< Jacobian x Residual */
//    Eigen::Matrix<double, D, 1> _Jres;
    /**< optimization result (update step) */
//    Eigen::Matrix<double, D, 1> _x;
    /* squared error */
    double _chi2;

    /**< maximum number of iterations */
    unsigned int _n_iter;
    /**< current iteration */
    unsigned int _iter;
    /**< number of measurements in current iteration */
    unsigned int _n_meas;
    /**< stop iterating when update metric absolute value smaller than _eps */
    double _eps;
    /**< internal flag showing when iteration has stopped for any reason (true) or not (false) */
    bool _flag_stop;
    /**< if true show solver statistics on console */
    bool _flag_verbose;



    /**< computes the jacobian and sets the member variables _H and _Jres */
    //virtual double compute_residuals(const T& model) = 0;
    /**< solve the linear system H*x = Jres, setting the update step in the internal variable _x.
    Returns true if system solved, false if singular. */
        //virtual bool solve () = 0;
    /**< updates the new model based on the old model and the optimization result */
        //virtual void update (const T& old_model, T& new_model) = 0;
    /**< action to perform at the start of each iteration (overloaded in derivate classes) */
        //virtual void start_iteration() {}
    /**< action to perform at the end of each iteration (overloaded in derivate classes) */
        //virtual void finish_iteration() {}
public:
    /**< default constructor */
    nlls_gauss_newton_();
    /**< destructor */
    virtual ~nlls_gauss_newton_() = default;

    /**< ################### */
    virtual void solve();

        /**< reset all parameters to their default values to restart the optimization */
    void reset();
    /**< implements the selected optimization strategy for the input model */
        //void optimize(T& Omodel);
}; // closes template nlls_gauss_newton_

#include "nlls_gauss_newton_.hpp"

} // closes namespace math

#endif


