#ifndef NAV_KF_HANDLER_TEMPLATE
#define NAV_KF_HANDLER_TEMPLATE

#include "../nav.h"
#include "kf_.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <iostream>
#include <unsupported/Eigen/MatrixFunctions>

namespace nav {

// CLASS KALMAN FILTER KF HANDLER
// ==============================
// ==============================

template <unsigned short siz_x, unsigned short siz_y>
class NAV_API kf_handler_ {
private:
    /**< weak pointer to Kalman filter */
    nav::kf_<siz_x, siz_y> _Okf;

    /**< vector of a priori state estimations */
    std::vector<Eigen::Matrix<double,siz_x,1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_x,1>>> _Vxhat_bef;
    /**< vector of a posteriori state estimations */
    std::vector<Eigen::Matrix<double,siz_x,1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_x,1>>> _Vxhat_aft;
    /**< matrix of a priori state estimation error covariances */
    std::vector<Eigen::Matrix<double,siz_x,siz_x>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_x,siz_x>>> _VP_bef;
    /**< matrix of a postetiori state estimation error covariances */
    std::vector<Eigen::Matrix<double,siz_x,siz_x>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_x,siz_x>>> _VP_aft;
    /**< matrix of optimum Kalman gains */
    std::vector<Eigen::Matrix<double,siz_x,siz_y>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_x,siz_y>>> _VK;
    /**< vector of measurements */
    std::vector<Eigen::Matrix<double,siz_y,1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_y,1>>> _Vy;
    /**< vector of innovations */
    std::vector<Eigen::Matrix<double,siz_y,1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_y,1>>> _Vr;
    /**< vector of innovation covariances */
    std::vector<Eigen::Matrix<double,siz_y,siz_y>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_y,siz_y>>> _VS;

    /**< vector of a priori state estimation errors (truth minus estimation) running mean */
    std::vector<Eigen::Matrix<double,siz_x,1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_x,1>>> _Veps_xhat_bef_mean;
    /**< vector of a posteriori state estimation errors (truth minus estimation) running mean */
    std::vector<Eigen::Matrix<double,siz_x,1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_x,1>>> _Veps_xhat_aft_mean;
    /**< vector of innovations state estimations running mean */
    std::vector<Eigen::Matrix<double,siz_y,1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_y,1>>> _Vr_mean;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**< default constructor */
    kf_handler_() = delete;
    /**< constructor based on size of navigation vector and filter time step */
    explicit kf_handler_(const unsigned int& nel, const double Deltat_sec_nav)
            : _Okf(Deltat_sec_nav), _Vxhat_bef(nel), _Vxhat_aft(nel), _VP_bef(nel), _VP_aft(nel), _VK(nel), _Vy(nel), _Vr(nel), _VS(nel),
            _Veps_xhat_bef_mean(nel), _Veps_xhat_aft_mean(nel), _Vr_mean(nel) {}
    /**< copy constructor */
    kf_handler_(const kf_handler_&) = delete;
    /**< move constructor */
    kf_handler_(kf_handler_&&) = delete;
    /**< destructor */
    ~kf_handler_() = default;
    /**< copy assignment */
    kf_handler_& operator=(const kf_handler_&) = delete;
    /**< move assignment */
    kf_handler_& operator=(kf_handler_&&) = delete;

    /**< initialize all internal vector, although only required ones are xhat_aft and P_aft */
    void initialize(const Eigen::Matrix<double,siz_x,1>& x0_truth, const Eigen::Matrix<double,siz_x,siz_x>& P0) {
        _Vxhat_bef[0] << x0_truth;
        _VP_bef[0]    << P0;
        _VK[0]        << Eigen::Matrix<double,siz_x,siz_y>::Zero();
        _Vy[0]        << Eigen::Matrix<double,siz_y,1>::Zero();
        _Vr[0]        << Eigen::Matrix<double,siz_y,1>::Zero();
        _VS[0]        << Eigen::Matrix<double,siz_y,siz_y>::Zero();
        _Vxhat_aft[0] << x0_truth;
        _VP_aft[0]    << P0;

        this->obtain_estimate_error_running_mean_bef(0, x0_truth - _Vxhat_bef[0]);
        this->obtain_estimate_error_running_mean_aft(0, x0_truth - _Vxhat_aft[0]);
        this->obtain_innov_running_mean(0);
    }

    /**< ===== ===== ===== Obtain and Add (every time step) ===== ===== ===== */
    /**< ==================================================================== */

    /**< execute the required functions of a filter step. Inputs are continuous system matrix A, continuous state covariance
     * matrix Qcont, measurement y, and input position. */
    void execute_step(const Eigen::Matrix<double,siz_x,siz_x>& A, const Eigen::Matrix<double,siz_x,siz_x>& Qcont,
                      const Eigen::Matrix<double,siz_y,siz_x>& H, const Eigen::Matrix<double,siz_y,siz_y>& R,
                      const Eigen::Matrix<double,siz_y,1>& y, const unsigned int& i) {
        _Vy[i] = y;
        this->add_A(A);
        this->add_Qcont(Qcont);
        this->obtain_estimate_bef(i);
        this->obtain_covariance_bef(i);
        this->add_H(H);
        this->add_R(R);
        this->obtain_gain(i);
        this->obtain_innovations(i);
        this->obtain_innov_covariance(i);
        this->obtain_estimate_aft(i);
        this->obtain_covariance_aft(i);
    }
    /**< execute the auxiliary functions of a filter step. Inputs are real state vector x_truth and input position. */
    void execute_step_aux(const Eigen::Matrix<double,siz_x,1> x_truth, const unsigned int& i) {
        this->obtain_estimate_error_running_mean_bef(i, x_truth - _Vxhat_bef[i]);
        this->obtain_estimate_error_running_mean_aft(i, x_truth - _Vxhat_aft[i]);
        this->obtain_innov_running_mean(i);
    }

    /**< pass continuous system matrix A to filter, which gets internally discretized into F based on the filter time step */
    void add_A(const Eigen::Matrix<double,siz_x,siz_x>& A) {
        _Okf.step01_add_A(A);
    }
    /**< pass continuous state covariance matrix Qcont to filter, which gets internally discretized into Q based on the filter time step */
    void add_Qcont(const Eigen::Matrix<double,siz_x,siz_x>& Qcont) {
        _Okf.step02_add_Qcont(Qcont);
    }
    /**< fill up the a priori state vector estimate for the input position */
    void obtain_estimate_bef(const unsigned int& i) {
        _Vxhat_bef[i].noalias() = _Okf.step03_obtain_estimate_bef(_Vxhat_aft[i-1]);
    }
    /**< fill up the a priori state vector estimate covariance matrix for the input position */
    void obtain_covariance_bef(const unsigned int& i) {
        _VP_bef[i].noalias() = _Okf.step04_obtain_covariance_bef(_VP_aft[i-1]);
    }
    /**< pass measurement matrix H to filter */
    void add_H(const Eigen::Matrix<double,siz_y,siz_x>& H) {
        _Okf.step05_add_H(H);
    }
    /**< pass measurement covariance matrix to filter */
    void add_R(const Eigen::Matrix<double,siz_y,siz_y>& R) {
        _Okf.step06_add_R(R);
    }
    /**< fill up the optimum Kalman gain for the input position */
    void obtain_gain(const unsigned int& i) {
        _VK[i].noalias() = _Okf.step07_obtain_gain(_VP_bef[i]);
    }
    /**< fill up the innovations for the input position */
    void obtain_innovations(const unsigned int& i) {
        _Vr[i].noalias() = _Okf.step08_obtain_innovations(_Vxhat_bef[i], _Vy[i]);
    }
    /**< fill up the innovation covariance for the input position */
    void obtain_innov_covariance(const unsigned int& i) {
        _VS[i].noalias() = _Okf.obtain_innov_covariance(_VP_bef[i]);
    }
    /**< fill up the a posteriori state vector estimate for the input position */
    void obtain_estimate_aft(const unsigned int& i) {
        _Vxhat_aft[i].noalias() = _Okf.step09_obtain_estimate_aft(_Vxhat_bef[i], _Vr[i], _VK[i]);
    }
    /**< fill up the a posteriori state vector estimate covariance matrix for the input position */
    void obtain_covariance_aft(const unsigned int& i) {
        _VP_aft[i].noalias() = _Okf.step10_obtain_covariance_aft(_VP_bef[i], _VK[i]);
    }

    /**< fill up the a priori state vector estimate error (truth minus estimation) running mean for the input position and estimation error */
    void obtain_estimate_error_running_mean_bef(const unsigned int& i, const Eigen::Matrix<double,siz_x,1>& eps_xhat_bef) {
        if (i==0) {
            _Veps_xhat_bef_mean[i] = eps_xhat_bef;
            return;
        }
        _Veps_xhat_bef_mean[i] = (_Veps_xhat_bef_mean[i-1] * (i-1) + eps_xhat_bef) / i;
    }
    /**< fill up the a posteriori state vector estimate errors (truth minus estimation) running mean for the input position and estimation error */
    void obtain_estimate_error_running_mean_aft(const unsigned int& i, const Eigen::Matrix<double,siz_x,1>& eps_xhat_aft) {
        if (i==0) {
            _Veps_xhat_aft_mean[i] = eps_xhat_aft;
            return;
        }
        _Veps_xhat_aft_mean[i] = (_Veps_xhat_aft_mean[i-1] * (i-1) + eps_xhat_aft) / i;
    }
    /**< fill up the innovations running mean for the input position */
    void obtain_innov_running_mean(const unsigned int& i) {
        if (i==0) {
            _Vr_mean[i] = _Vr[i];
            return;
        }
        _Vr_mean[i] = (_Vr_mean[i-1] * (i-1) + _Vr[i]) / i;
    }

    /**< ===== ===== ===== Getters ===== ===== ===== */
    /**< =========================================== */

    /**< get Kalman filter */
    const nav::kf_<siz_x, siz_y>& get_kf() const {return _Okf;}

    /**< get vector of a priori state estimations */
    const std::vector<Eigen::Matrix<double,siz_x,1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_x,1>>>& get_xhat_bef() const {return _Vxhat_bef;}
    std::vector<Eigen::Matrix<double,siz_x,1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_x,1>>>& get_xhat_bef() {return _Vxhat_bef;}
    /**< get vector of a posteriori state estimations */
    const std::vector<Eigen::Matrix<double,siz_x,1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_x,1>>>& get_xhat_aft() const {return _Vxhat_aft;}
    std::vector<Eigen::Matrix<double,siz_x,1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_x,1>>>& get_xhat_aft() {return _Vxhat_aft;}
    /**< get matrix of a priori state estimation error covariances */
    const std::vector<Eigen::Matrix<double,siz_x,siz_x>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_x,siz_x>>>& get_P_bef() const {return _VP_bef;}
    std::vector<Eigen::Matrix<double,siz_x,siz_x>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_x,siz_x>>>& get_P_bef() {return _VP_bef;}
    /**< get matrix of a postetiori state estimation error covariances */
    const std::vector<Eigen::Matrix<double,siz_x,siz_x>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_x,siz_x>>>& get_P_aft() const {return _VP_aft;}
    std::vector<Eigen::Matrix<double,siz_x,siz_x>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_x,siz_x>>>& get_P_aft() {return _VP_aft;}
    /**< get matrix of optimum Kalman gains */
    const std::vector<Eigen::Matrix<double,siz_x,siz_y>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_x,siz_y>>>& get_K() const {return _VK;}
    std::vector<Eigen::Matrix<double,siz_x,siz_y>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_x,siz_y>>>& get_K() {return _VK;}
    /**< get vector of measurements */
    const std::vector<Eigen::Matrix<double,siz_y,1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_y,1>>>& get_y() const {return _Vy;}
    std::vector<Eigen::Matrix<double,siz_y,1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_y,1>>>& get_y() {return _Vy;}
    /**< get vector of innovations */
    const std::vector<Eigen::Matrix<double,siz_y,1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_y,1>>>& get_r() const {return _Vr;}
    std::vector<Eigen::Matrix<double,siz_y,1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_y,1>>>& get_r() {return _Vr;}
    /**< get vector of innovation covariances */
    const std::vector<Eigen::Matrix<double,siz_y,siz_y>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_y,siz_y>>>& get_S() const {return _VS;}
    std::vector<Eigen::Matrix<double,siz_y,siz_y>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_y,siz_y>>>& get_S() {return _VS;}

    /**< get vector of a priori state estimation errors (truth minus estimation) running mean */
    const std::vector<Eigen::Matrix<double,siz_x,1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_x,1>>>& get_eps_xhat_bef_mean() const {return _Veps_xhat_bef_mean;}
    std::vector<Eigen::Matrix<double,siz_x,1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_x,1>>>& get_eps_xhat_bef_mean() {return _Veps_xhat_bef_mean;}
    /**< get vector of a posteriori state estimation errors (truth minus estimation) running mean */
    const std::vector<Eigen::Matrix<double,siz_x,1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_x,1>>>& get_eps_xhat_aft_mean() const {return _Veps_xhat_aft_mean;}
    std::vector<Eigen::Matrix<double,siz_x,1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_x,1>>>& get_eps_xhat_aft_mean() {return _Veps_xhat_aft_mean;}
    /**< get vector of innovations state estimations running mean */
    const std::vector<Eigen::Matrix<double,siz_y,1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_y,1>>>& get_r_mean() const {return _Vr_mean;}
    std::vector<Eigen::Matrix<double,siz_y,1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_y,1>>>& get_r_mean() {return _Vr_mean;}

}; // closes class kf_handler_

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}; // closes namespace nav

#endif
