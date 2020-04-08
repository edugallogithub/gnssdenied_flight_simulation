#ifndef NAV_EKF_TEMPLATE
#define NAV_EKF_TEMPLATE

#include "../nav.h"
#include "ekf_functor_.h"
#include <iostream>
#include <iomanip>

namespace nav {

// CLASS EXTENDED KALMAN FILTER EKF
// ================================
// ================================

template <unsigned short siz_x, unsigned short siz_y, unsigned short siz_w, unsigned short siz_v>
class NAV_API ekf_ {
private:
    /**< state vector size */
    unsigned short _siz_x;
    /**< measurement vector size */
    unsigned short _siz_y;
    /**< state noise size */
    unsigned short _siz_w;
    /**< measurement noise size */
    unsigned short _siz_v;
    /**< filter time step */
    double _Deltat_sec;

    /**< weak pointer to state system function f */
    nav::ekf_state_<siz_x>* _Pstate;
    /**< continuous time linear system matrix (state jacobian of f) */
    Eigen::Matrix<double,siz_x,siz_x> _A;
    /**< discrete time linear system matrix */
    Eigen::Matrix<double,siz_x,siz_x> _F;
    /**< discrete time linear state covariance matrix */
    Eigen::Matrix<double,siz_x,siz_x> _Qtilde;

    /**< weak pointer to observation function */
    nav::ekf_obser_<siz_x, siz_y, siz_v>* _Pobser;
    /**< discrete time linear observation matrix (state jacobian of h) */
    Eigen::Matrix<double,siz_y,siz_x> _H;
    /**< transpose of discrete time linear observation matrix */
    Eigen::Matrix<double,siz_x,siz_y> _Htranspose;
    /**< noise jacobian of h */
    Eigen::Matrix<double,siz_y,siz_v> _M;
    /**< discrete time linear observation covariance matrix */
    Eigen::Matrix<double,siz_y,siz_y> _Rtilde;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**< default constructor */
    ekf_() = delete;
    /**< constructor based on filter time step */
    ekf_(const double& Deltat_sec, nav::ekf_state_<siz_x>& Ostate, nav::ekf_obser_<siz_x, siz_y, siz_v>& Oobser)
            : _siz_x(siz_x), _siz_y(siz_y), _siz_w(siz_w), _siz_v(siz_v), _Deltat_sec(Deltat_sec),
              _Pstate(&Ostate), _A(Eigen::Matrix<double,siz_x,siz_x>::Zero()), //_L(Eigen::Matrix<double,siz_x,siz_w>::Zero()),
              _Pobser(&Oobser), _H(Eigen::Matrix<double,siz_y,siz_x>::Zero()), _M(Eigen::Matrix<double,siz_y,siz_v>::Zero()) {
    }
    /**< copy constructor */
    ekf_(const ekf_&) = delete;
    /**< move constructor */
    ekf_(ekf_&&) = delete;
    /**< destructor (DO NOT replace by default as there are weak pointers) */
    ~ekf_() {}
    /**< copy assignment */
    ekf_& operator=(const ekf_&) = delete;
    /**< move assignment */
    ekf_& operator=(ekf_&&) = delete;

    /**< ===== ===== ===== Obtain ===== ===== ===== */
    /**< ========================================== */

    /**< execute prediction phase of EKF based on previous a posteriori state estimate and previous a posteriori error covariance:
     * - obtains the discrete time linear system matrix F.
     * - obtains the discrete time linear system covariance matrix Qtilde.
     * - fills up the a priori state vector estimate xhat_bef.
     * - fills up the a priori state vector estimate covariance matrix P_bef. */
    void process_state(Eigen::Matrix<double,siz_x,1>& xhat_bef, Eigen::Matrix<double,siz_x,siz_x>& P_bef,
                       const Eigen::Matrix<double,siz_x,1>& xhat_aft_prev, const Eigen::Matrix<double,siz_x,siz_x>& P_aft_prev) {
        _Pstate->jacobian_state(_A, xhat_aft_prev);
        _F = (_A * _Deltat_sec).exp();
        _Qtilde = _Pstate->covariance_matrix();
        xhat_bef = xhat_aft_prev + _Pstate->eval(xhat_aft_prev) * _Deltat_sec;
        P_bef = _F * P_aft_prev * _F.transpose() + _Qtilde;
    }

    /**< execute update phase of EKF based on a priori state estimate, a priori error covariance, and observations:
     * - obtains the discrete time linear observation matrix H.
     * - obtains the discrete time linear observation matrix Rtilde.
     * - fills up the optimum Kalman gain K.
     * - fills up the measurements y.
     * - fills up the innovations r.
     * - fills up the innovations covariance S.
     * - fills up the a posteriori state vector estimate xhat_aft.
     * - fills up the a posteriori state vector estimate covariance matrix P_aft. */
    void process_observation(Eigen::Matrix<double,siz_x,siz_y>& K, Eigen::Matrix<double,siz_y,1>& r, Eigen::Matrix<double,siz_y,siz_y>& S,
                             Eigen::Matrix<double,siz_x,1>& xhat_aft, Eigen::Matrix<double,siz_x,siz_x>& P_aft,
                             const Eigen::Matrix<double,siz_x,1>& xhat_bef, const Eigen::Matrix<double,siz_x,siz_x>& P_bef, const Eigen::Matrix<double,siz_y,1>& y) {
        _Pobser->jacobian_state(_H, xhat_bef);
        _Htranspose = _H.transpose();
        _Pobser->jacobian_noise(_M , xhat_bef);
        _Rtilde = _M * _Pobser->covariance_matrix() * _M.transpose();
        S = _H * P_bef * _Htranspose + _Rtilde;
        K = P_bef * _Htranspose * S.inverse(); /////////////////////////////////////////////////////////////////////////////// OJO NO CALCULAR INVERSA SINO SOLUCION CUANDO SE NECESITE
        r = y - _Pobser->eval(xhat_bef);
        xhat_aft = xhat_bef + K * r;
        Eigen::Matrix<double,siz_x,siz_x> Temp = Eigen::Matrix<double,siz_x,siz_x>::Identity() - K * _H;
        P_aft = Temp * P_bef * Temp.transpose() + K * _Rtilde * K.transpose();
    }

    /**< ===== ===== ===== Getters ===== ===== ===== */
    /**< =========================================== */
    /**< get state vector size */
    const unsigned short& get_siz_x() const {return _siz_x;}
    /**< get measurement vector size */
    const unsigned short& get_siz_y() const {return _siz_y;}
    /**< get state noise size */
    const unsigned short& get_siz_w() const {return _siz_w;}
    /**< get measurement noise size */
    const unsigned short& get_siz_v() const {return _siz_v;}
    /**< get filter time step */
    const double& get_Deltat_sec() const {return _Deltat_sec;}
}; // closes class ekf_

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}; // closes namespace nav

#endif
