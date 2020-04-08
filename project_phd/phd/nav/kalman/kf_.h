#ifndef NAV_KF_TEMPLATE
#define NAV_KF_TEMPLATE

#include "../nav.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

namespace nav {

// CLASS KALMAN FILTER KF
// ======================
// ======================

template <unsigned short siz_x, unsigned short siz_y>
class NAV_API kf_ {
private:
    /**< state vector size */
    unsigned short _siz_x;
    /**< measurement vector size */
    unsigned short _siz_y;
    /**< filter time step */
    double _Deltat_sec;

    /**< system matrix */
    Eigen::Matrix<double,siz_x,siz_x> _F;
    /**< measurement matrix */
    Eigen::Matrix<double,siz_y,siz_x> _H;
    /**< state covariance matrix */
    Eigen::Matrix<double,siz_x,siz_x> _Q;
    /**< measurement covariance matrix */
    Eigen::Matrix<double,siz_y,siz_y> _R;
    /**< transpose of measurement matrix */
    Eigen::Matrix<double,siz_x,siz_y> _Htranspose;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**< default constructor */
    kf_() = delete;
    /**< constructor based on filter time step */
    explicit kf_(const double& Deltat_sec) : _siz_x(siz_x), _siz_y(siz_y), _Deltat_sec(Deltat_sec) {}
    /**< copy constructor */
    kf_(const kf_&) = delete;
    /**< move constructor */
    kf_(kf_&&) = delete;
    /**< destructor */
    ~kf_() = default;
    /**< copy assignment */
    kf_& operator=(const kf_&) = delete;
    /**< move assignment */
    kf_& operator=(kf_&&) = delete;

    /**< ===== ===== ===== Ordered Steps ===== ===== ===== */
    /**< ================================================= */

    /**< pass continuous system matrix A to filter, which gets internally discretized into F based on the filter time step */
    void step01_add_A(const Eigen::Matrix<double,siz_x,siz_x>& A) {
        _F = (A * _Deltat_sec).exp();
    }
    /**< pass continuous state covariance matrix Qcont to filter, which gets internally discretized into Q based on the filter time step */
    void step02_add_Qcont(const Eigen::Matrix<double,siz_x,siz_x>& Qcont) {
        _Q = Qcont * _Deltat_sec;
    }
    /**< obtain the a priori state vector estimate xhat_bef_k. USE NOALIAS() */
    Eigen::Matrix<double,siz_x,1> step03_obtain_estimate_bef(const Eigen::Matrix<double,siz_x,1>& xhat_aft_prev) const {
        return _F * xhat_aft_prev;
    }
    /**< obtain the a priori covariance matrix of the state vector estimate error P_bef_k. USE NOALIAS() */
    Eigen::Matrix<double,siz_x,siz_x> step04_obtain_covariance_bef(const Eigen::Matrix<double,siz_x,siz_x>& P_aft_prev) const {
        return _F * P_aft_prev * _F.transpose() + _Q;
    }
    /**< pass measurement matrix H to filter */
    void step05_add_H(const Eigen::Matrix<double,siz_y,siz_x>& H) {
        _H = H;
        _Htranspose = _H.transpose();
    }
    /**< pass measurement covariance matrix to filter */
    void step06_add_R(const Eigen::Matrix<double,siz_y,siz_y>& R) {
        _R = R;
    }
    /**< obtain the optimum Kalman gain K_k. USE NOALIAS() */
    Eigen::Matrix<double,siz_x,siz_y> step07_obtain_gain(const Eigen::Matrix<double,siz_x,siz_x>& P_bef_k) const {
        return P_bef_k * _Htranspose * (_H * P_bef_k * _Htranspose + _R).inverse(); /////////////////////////////////////////////////////////////////////////////// OJO NO CALCULAR INVERSA SINO SOLUCION CUANDO SE NECESITE
    }
    /**< obtain the innovations r_k. USE NOALIAS() */
    Eigen::Matrix<double,siz_y,1> step08_obtain_innovations(const Eigen::Matrix<double,siz_x,1>& xhat_bef_k, const Eigen::Matrix<double,siz_y,1>& y_k) const {
        return y_k - _H * xhat_bef_k;
    }
    /**< obtain the a posteriori state vector estimate xhat_aft_k. USE NOALIAS() */
    Eigen::Matrix<double,siz_x,1> step09_obtain_estimate_aft(const Eigen::Matrix<double,siz_x,1>& xhat_bef_k, const Eigen::Matrix<double,siz_y,1>& r_k, const Eigen::Matrix<double,siz_x,siz_y>& K_k) const {
        return xhat_bef_k + K_k * r_k;
    }
    /**< obtain the a posteriori covariance matrix of the state vector estimate error P_aft_k. USE NOALIAS() */
    Eigen::Matrix<double,siz_x,siz_x> step10_obtain_covariance_aft(const Eigen::Matrix<double,siz_x,siz_x>& P_bef_k, const Eigen::Matrix<double,siz_x,siz_y>& K_k) const {
        Eigen::Matrix<double,siz_x,siz_x> Temp = Eigen::Matrix<double,siz_x,siz_x>::Identity() - K_k * _H;
        return Temp * P_bef_k * Temp.transpose() + K_k * _R * K_k.transpose();
    }
    /**< obtain the innovation covariance S_k. USE NOALIAS() */
    Eigen::Matrix<double,siz_y,siz_y> obtain_innov_covariance(const Eigen::Matrix<double,siz_x,siz_x>& P_bef_k) const {
        return _H * P_bef_k * _Htranspose + _R;
    }

    /**< ===== ===== ===== Getters ===== ===== ===== */
    /**< =========================================== */

    /**< get state vector size */
    const unsigned short& get_siz_x() const {return _siz_x;}
    /**< get measurement vector size */
    const unsigned short& get_siz_y() const {return _siz_y;}

    /**< get system matrix F */
    const Eigen::Matrix<double,siz_x,siz_x>& get_F() const {return _F;}
    /**< get measurement matrix */
    const Eigen::Matrix<double,siz_y,siz_x>& get_H() const {return _H;}
    /**< get state covariance matrix */
    const Eigen::Matrix<double,siz_x,siz_x>& get_Q() const {return _Q;}
    /**< get measurement covariance matrix */
    const Eigen::Matrix<double,siz_y,siz_y>& get_R() const {return _R;}

}; // closes class kf_

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}; // closes namespace nav

#endif
