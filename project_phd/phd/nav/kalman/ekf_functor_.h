#ifndef NAV_EKF_FUNCTOR_TEMPLATE
#define NAV_EKF_FUNCTOR_TEMPLATE

#include "../nav.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

namespace nav {

// CLASS EKF_STATE
// ===============
// ===============

template <unsigned short siz_x>
class NAV_API ekf_state_ {
/**< Represents EKF state system where input is the state vector x and output is its differential xdot.
* Represents f(x,u,w,t), where u is deterministic and siz_x, while w is noise and siz_w. However,
* the noise is provided already discretized and linearized, and hence siz_x */
public:
    /**< evaluates functor based on state vector. */
    virtual Eigen::Matrix<double,siz_x, 1> eval(const Eigen::Matrix<double,siz_x,1>& xhat) const = 0;
    /**< evaluates jacobian of functor with respect to state vector x based on state vector. */
    virtual void jacobian_state(Eigen::Matrix<double,siz_x,siz_x>& A, const Eigen::Matrix<double,siz_x,1>& xhat) const = 0;
    /**< returns covariance matrix */
    virtual Eigen::Matrix<double,siz_x,siz_x> covariance_matrix() const = 0;
}; // closes class ekf_state

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// CLASS EKF_OBSER
// ===============
// ===============

template <unsigned short siz_x, unsigned short siz_y, unsigned short siz_v>
class NAV_API ekf_obser_ {
/**< Represents EKF observation system where input is the state vector x and output is the observations y.
 * Represents h(x,v,t), where v is noise and siz_v. */
public:
    /**< evaluates functor based on state vector. */
    virtual Eigen::Matrix<double,siz_y,1> eval(const Eigen::Matrix<double,siz_x,1>& xhat) const = 0;
    /**< evaluates jacobian of functor with respect to state vector x based on state vector. */
    virtual void jacobian_state(Eigen::Matrix<double,siz_y,siz_x>& output, const Eigen::Matrix<double,siz_x,1>& xhat) const = 0;
    /**< evaluates jacobian of functor with respect to noise vector (w or v) based on state vector. */
    virtual void jacobian_noise(Eigen::Matrix<double,siz_y,siz_v>& output, const Eigen::Matrix<double,siz_x,1>& xhat) const = 0;
    /**< returns covariance matrix */
    virtual Eigen::Matrix<double,siz_v,siz_v> covariance_matrix() const = 0;
}; // closes class ekf_obser_

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// CLASS EKFD_OBSER
// ================
// ================

template <unsigned short siz_x, unsigned short siz_y1, unsigned short siz_y2, unsigned short siz_v1, unsigned short siz_v2>
class NAV_API ekfd_obser_ {
/**< Represents EKFD observation system where input is the state vector x and output is the observations y.
* Represents h(x,v,t), where v is noise and siz_v. */
public:
    /**< evaluates functor (fast) based on state vector. */
    virtual Eigen::Matrix<double,siz_y1,1> eval_fast(const Eigen::Matrix<double,siz_x,1>& xhat) const = 0;
    /**< evaluates functor (slow) based on state vector. */
    virtual Eigen::Matrix<double,siz_y2,1> eval_slow(const Eigen::Matrix<double,siz_x,1>& xhat) const = 0;

    /**< evaluates jacobian of functor (fast) with respect to state vector x based on state vector. */
    virtual void jacobian_state_fast(Eigen::Matrix<double,siz_y1,siz_x>& output, const Eigen::Matrix<double,siz_x,1>& xhat) const = 0;
    /**< evaluates jacobian of functor (slow) with respect to state vector x based on state vector. */
    virtual void jacobian_state_slow(Eigen::Matrix<double,siz_y2,siz_x>& output, const Eigen::Matrix<double,siz_x,1>& xhat) const = 0;

    /**< evaluates jacobian of functor (fast) with respect to noise vector (w or v) based on state vector. */
    virtual void jacobian_noise_fast(Eigen::Matrix<double,siz_y1,siz_v1>& output, const Eigen::Matrix<double,siz_x,1>& xhat) const = 0;
    /**< evaluates jacobian of functor (slow) with respect to noise vector (w or v) based on state vector. */
    virtual void jacobian_noise_slow(Eigen::Matrix<double,siz_y2,siz_v2>& output, const Eigen::Matrix<double,siz_x,1>& xhat) const = 0;

    /**< returns covariance matrix (fast) */
    virtual Eigen::Matrix<double,siz_v1,siz_v1> covariance_matrix_fast() const = 0;
    /**< returns covariance matrix (slow) */
    virtual Eigen::Matrix<double,siz_v2,siz_v2> covariance_matrix_slow() const = 0;
}; // closes class ekfd_obser_

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// CLASS EKFT_OBSER
// ================
// ================

template <unsigned short siz_x, unsigned short siz_y1, unsigned short siz_y2, unsigned short siz_y3, unsigned short siz_v1, unsigned short siz_v2, unsigned short siz_v3>
class NAV_API ekft_obser_ {
/**< Represents EKFT observation system where input is the state vector x and output is the observations y.
* Represents h(x,v,t), where v is noise and siz_v. */
public:
/**< evaluates functor (fast) based on state vector. */
    virtual Eigen::Matrix<double,siz_y1,1> eval_fast(const Eigen::Matrix<double,siz_x,1>& xhat) const = 0;
    /**< evaluates functor (slow) based on state vector. */
    virtual Eigen::Matrix<double,siz_y2,1> eval_slow(const Eigen::Matrix<double,siz_x,1>& xhat) const = 0;
    /**< evaluates functor (loss) based on state vector. */
    virtual Eigen::Matrix<double,siz_y3,1> eval_loss(const Eigen::Matrix<double,siz_x,1>& xhat) const = 0;

    /**< evaluates jacobian of functor (fast) with respect to state vector x based on state vector. */
    virtual void jacobian_state_fast(Eigen::Matrix<double,siz_y1,siz_x>& output, const Eigen::Matrix<double,siz_x,1>& xhat) const = 0;
    /**< evaluates jacobian of functor (slow) with respect to state vector x based on state vector. */
    virtual void jacobian_state_slow(Eigen::Matrix<double,siz_y2,siz_x>& output, const Eigen::Matrix<double,siz_x,1>& xhat) const = 0;
    /**< evaluates jacobian of functor (loss) with respect to state vector x based on state vector. */
    virtual void jacobian_state_loss(Eigen::Matrix<double,siz_y3,siz_x>& output, const Eigen::Matrix<double,siz_x,1>& xhat) const = 0;

    /**< evaluates jacobian of functor (fast) with respect to noise vector (w or v) based on state vector. */
    virtual void jacobian_noise_fast(Eigen::Matrix<double,siz_y1,siz_v1>& output, const Eigen::Matrix<double,siz_x,1>& xhat) const = 0;
    /**< evaluates jacobian of functor (slow) with respect to noise vector (w or v) based on state vector. */
    virtual void jacobian_noise_slow(Eigen::Matrix<double,siz_y2,siz_v2>& output, const Eigen::Matrix<double,siz_x,1>& xhat) const = 0;
    /**< evaluates jacobian of functor (loss) with respect to noise vector (w or v) based on state vector. */
    virtual void jacobian_noise_loss(Eigen::Matrix<double,siz_y3,siz_v3>& output, const Eigen::Matrix<double,siz_x,1>& xhat) const = 0;

    /**< returns covariance matrix (fast) */
    virtual Eigen::Matrix<double,siz_v1,siz_v1> covariance_matrix_fast() const = 0;
    /**< returns covariance matrix (slow) */
    virtual Eigen::Matrix<double,siz_v2,siz_v2> covariance_matrix_slow() const = 0;
    /**< returns covariance matrix (loss) */
    virtual Eigen::Matrix<double,siz_v3,siz_v3> covariance_matrix_loss() const = 0;
}; // closes class ekft_obser_

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// CLASS EKF_STATE_CONT
// ====================
// ====================

// Here noise is provided in continuous non linear form. Most likely not used and shall be deleted at some point.

template <unsigned short siz_x, unsigned short siz_w>
class NAV_API ekf_state_cont_ {
/**< Represents EKF state system where input is the state vector x and output is its differential xdot.
* Represents f(x,u,w,t), where u is deterministic and siz_x, while w is noise and siz_w. */
public:
    /**< evaluates functor based on state vector. */
    virtual Eigen::Matrix<double,siz_x, 1> eval(const Eigen::Matrix<double,siz_x,1>& xhat) const = 0;
    /**< evaluates jacobian of functor with respect to state vector x based on state vector. */
    virtual void jacobian_state(Eigen::Matrix<double,siz_x,siz_x>& A, const Eigen::Matrix<double,siz_x,1>& xhat) const = 0;
    /**< evaluates jacobian of functor with respect to noise vector (w or v) based on state vector. */
    virtual void jacobian_noise(Eigen::Matrix<double,siz_x,siz_w>& L, const Eigen::Matrix<double,siz_x,1>& xhat) const = 0;
    /**< returns covariance matrix */
    virtual Eigen::Matrix<double,siz_w,siz_w> covariance_matrix() const = 0;
}; // closes class ekf_state_cont

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}; // closes namespace nav

#endif
