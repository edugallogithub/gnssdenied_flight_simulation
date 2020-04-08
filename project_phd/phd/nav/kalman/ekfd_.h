#ifndef NAV_EKFD_TEMPLATE
#define NAV_EKFD_TEMPLATE

#include "../nav.h"
#include "ekf_functor_.h"
#include <iostream>
#include <iomanip>

namespace nav {

/*
 * Same as extended Kalman Filter but observations come at two different frequencies.
 */

// CLASS EXTENDED KALMAN FILTER DUAL TIMES EKFD
// ============================================
// ============================================

template <unsigned short siz_x, unsigned short siz_y1, unsigned short siz_y2, unsigned short siz_w, unsigned short siz_v1, unsigned short siz_v2>
class NAV_API ekfd_ {
private:
    /**< state vector size */
    unsigned short _siz_x;
    /**< fast measurement vector size */
    unsigned short _siz_y1;
    /**< slow measurement vector size */
    unsigned short _siz_y2;
    /**< state noise size */
    unsigned short _siz_w;
    /**< fast measurement noise size */
    unsigned short _siz_v1;
    /**< slow measurement noise size */
    unsigned short _siz_v2;
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
    nav::ekfd_obser_<siz_x, siz_y1, siz_y2, siz_v1, siz_v2>* _Pobser;
    /**< fast discrete time linear observation matrix (state jacobian of h) */
    Eigen::Matrix<double,siz_y1,siz_x> _H_fast;
    /**< slow discrete time linear observation matrix (state jacobian of h) */
    Eigen::Matrix<double,siz_y2,siz_x> _H_slow;
    /**< transpose of fast discrete time linear observation matrix */
    Eigen::Matrix<double,siz_x,siz_y1> _H_fast_transpose;
    /**< transpose of slow discrete time linear observation matrix */
    Eigen::Matrix<double,siz_x,siz_y2> _H_slow_transpose;
    /**< fast noise jacobian of h */
    Eigen::Matrix<double,siz_y1,siz_v1> _M_fast;
    /**< slow noise jacobian of h */
    Eigen::Matrix<double,siz_y2,siz_v2> _M_slow;
    /**< fast discrete time linear observation covariance matrix */
    Eigen::Matrix<double,siz_y1,siz_y1> _Rtilde_fast;
    /**< slow discrete time linear observation covariance matrix */
    Eigen::Matrix<double,siz_y2,siz_y2> _Rtilde_slow;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**< default constructor */
    ekfd_() = delete;
    /**< constructor based on filter time step */
    ekfd_(const double& Deltat_sec, nav::ekf_state_<siz_x>& Ostate, nav::ekfd_obser_<siz_x, siz_y1, siz_y2, siz_v1, siz_v2>& Oobser)
            : _siz_x(siz_x), _siz_y1(siz_y1), _siz_y2(siz_y2), _siz_w(siz_w), _siz_v1(siz_v1), _siz_v2(siz_v2), _Deltat_sec(Deltat_sec),
              _Pstate(&Ostate), _A(Eigen::Matrix<double,siz_x,siz_x>::Zero()),
              _Pobser(&Oobser), _H_fast(Eigen::Matrix<double,siz_y1,siz_x>::Zero()), _H_slow(Eigen::Matrix<double,siz_y2,siz_x>::Zero()),
              _M_fast(Eigen::Matrix<double,siz_y1,siz_v1>::Zero()), _M_slow(Eigen::Matrix<double,siz_y2,siz_v2>::Zero()) {
    }
    /**< copy constructor */
    ekfd_(const ekfd_&) = delete;
    /**< move constructor */
    ekfd_(ekfd_&&) = delete;
    /**< destructor (DO NOT replace by default as there are weak pointers) */
    ~ekfd_() {}
    /**< copy assignment */
    ekfd_& operator=(const ekfd_&) = delete;
    /**< move assignment */
    ekfd_& operator=(ekfd_&&) = delete;

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

    /**< execute fast update phase of EKF based on a priori state estimate, a priori error covariance, and observations:
     * - obtains the discrete time linear observation matrix H.
     * - obtains the discrete time linear observation matrix Rtilde.
     * - fills up the optimum Kalman gain K.
     * - fills up the measurements y.
     * - fills up the innovations r.
     * - fills up the innovations covariance S.
     * - fills up the a posteriori state vector estimate xhat_aft.
     * - fills up the a posteriori state vector estimate covariance matrix P_aft. */
    void process_observation_fast(Eigen::Matrix<double,siz_x,siz_y1>& K_fast, Eigen::Matrix<double,siz_y1,1>& r_fast, Eigen::Matrix<double,siz_y1,siz_y1>& S_fast,
                                  Eigen::Matrix<double,siz_x,1>& xhat_aft, Eigen::Matrix<double,siz_x,siz_x>& P_aft,
                                  const Eigen::Matrix<double,siz_x,1>& xhat_bef, const Eigen::Matrix<double,siz_x,siz_x>& P_bef, const Eigen::Matrix<double,siz_y1,1>& y_fast) {
        _Pobser->jacobian_state_fast(_H_fast, xhat_bef);
        _H_fast_transpose = _H_fast.transpose();
        _Pobser->jacobian_noise_fast(_M_fast , xhat_bef);
        _Rtilde_fast = _M_fast * _Pobser->covariance_matrix_fast() * _M_fast.transpose();
        S_fast = _H_fast * P_bef * _H_fast_transpose + _Rtilde_fast;
        K_fast = P_bef * _H_fast_transpose * S_fast.inverse(); /////////////////////////////////////////////////////////////////////////////// OJO NO CALCULAR INVERSA SINO SOLUCION CUANDO SE NECESITE
        r_fast = y_fast - _Pobser->eval_fast(xhat_bef);
        xhat_aft = xhat_bef + K_fast * r_fast;
        Eigen::Matrix<double,siz_x,siz_x> Temp = Eigen::Matrix<double,siz_x,siz_x>::Identity() - K_fast * _H_fast;
        P_aft = Temp * P_bef * Temp.transpose() + K_fast * _Rtilde_fast * K_fast.transpose();
    }

    /**< execute slow update phase of EKF based on a priori state estimate, a priori error covariance, and observations:
     * - obtains the discrete time linear observation matrix H.
     * - obtains the discrete time linear observation matrix Rtilde.
     * - fills up the optimum Kalman gain K.
     * - fills up the measurements y.
     * - fills up the innovations r.
     * - fills up the innovations covariance S.
     * - fills up the a posteriori state vector estimate xhat_aft.
     * - fills up the a posteriori state vector estimate covariance matrix P_aft. */
    void process_observation_slow(Eigen::Matrix<double,siz_x,siz_y2>& K_slow, Eigen::Matrix<double,siz_y2,1>& r_slow, Eigen::Matrix<double,siz_y2,siz_y2>& S_slow,
                                  Eigen::Matrix<double,siz_x,1>& xhat_aft, Eigen::Matrix<double,siz_x,siz_x>& P_aft,
                                  const Eigen::Matrix<double,siz_x,1>& xhat_bef, const Eigen::Matrix<double,siz_x,siz_x>& P_bef, const Eigen::Matrix<double,siz_y2,1>& y_slow) {
        _Pobser->jacobian_state_slow(_H_slow, xhat_bef);
        _H_slow_transpose = _H_slow.transpose();
        _Pobser->jacobian_noise_slow(_M_slow , xhat_bef);
        _Rtilde_slow = _M_slow * _Pobser->covariance_matrix_slow() * _M_slow.transpose();
        S_slow = _H_slow * P_bef * _H_slow_transpose + _Rtilde_slow;
        K_slow = P_bef * _H_slow_transpose * S_slow.inverse(); /////////////////////////////////////////////////////////////////////////////// OJO NO CALCULAR INVERSA SINO SOLUCION CUANDO SE NECESITE
        r_slow = y_slow - _Pobser->eval_slow(xhat_bef);
        xhat_aft = xhat_bef + K_slow * r_slow;
        Eigen::Matrix<double,siz_x,siz_x> Temp = Eigen::Matrix<double,siz_x,siz_x>::Identity() - K_slow * _H_slow;
        P_aft = Temp * P_bef * Temp.transpose() + K_slow * _Rtilde_slow * K_slow.transpose();
    }

    /**< ===== ===== ===== Getters ===== ===== ===== */
    /**< =========================================== */
    /**< get state vector size */
    const unsigned short& get_siz_x() const {return _siz_x;}
    /**< get fast measurement vector size */
    const unsigned short& get_siz_y1() const {return _siz_y1;}
    /**< get slow measurement vector size */
    const unsigned short& get_siz_y2() const {return _siz_y2;}
    /**< get state noise size */
    const unsigned short& get_siz_w() const {return _siz_w;}
    /**< get fast measurement noise size */
    const unsigned short& get_siz_v1() const {return _siz_v1;}
    /**< get slow measurement noise size */
    const unsigned short& get_siz_v2() const {return _siz_v2;}
    /**< get filter time step */
    const double& get_Deltat_sec() const {return _Deltat_sec;}
}; // closes class ekf_

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}; // closes namespace nav

#endif
