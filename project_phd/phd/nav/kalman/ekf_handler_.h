#ifndef NAV_EKF_HANDLER_TEMPLATE
#define NAV_EKF_HANDLER_TEMPLATE

#include "../nav.h"
#include "ekf_.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <iostream>
#include <unsupported/Eigen/MatrixFunctions>
#include <iomanip>

namespace nav {

// CLASS EXTENDED KALMAN FILTER EKF HANDLER
// ========================================
// ========================================

template <unsigned short siz_x, unsigned short siz_y, unsigned short siz_w, unsigned short siz_v>
class NAV_API ekf_handler_ {
private:
    /**< weak pointer to extended Kalman filter */
    nav::ekf_<siz_x, siz_y, siz_w, siz_v> _Oekf;

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
    ekf_handler_() = delete;
    /**< constructor based on size of navigation vector and filter time step */
    ekf_handler_(const unsigned int& nel, const double Deltat_sec_nav, nav::ekf_state_<siz_x>& Ostate, nav::ekf_obser_<siz_x, siz_y, siz_v>& Oobser)
    : _Oekf(Deltat_sec_nav, Ostate, Oobser), _Vxhat_bef(nel), _Vxhat_aft(nel), _VP_bef(nel), _VP_aft(nel), _VK(nel), _Vy(nel), _Vr(nel), _VS(nel),
      _Veps_xhat_bef_mean(nel), _Veps_xhat_aft_mean(nel), _Vr_mean(nel) {}
    /**< copy constructor */
    ekf_handler_(const ekf_handler_&) = delete;
    /**< move constructor */
    ekf_handler_(ekf_handler_&&) = delete;
    /**< destructor */
    ~ekf_handler_() = default;
    /**< copy assignment */
    ekf_handler_& operator=(const ekf_handler_&) = delete;
    /**< move assignment */
    ekf_handler_& operator=(ekf_handler_&&) = delete;

    /**< initialize all internal vectors, although only required ones are xhat_aft and P_aft */
    void initialize(const Eigen::Matrix<double,siz_x,1>& x0, const Eigen::Matrix<double,siz_x,siz_x>& P0, const Eigen::Matrix<double,siz_x,1>& x0_truth) {
        _Vxhat_bef[0] << x0;
        _VP_bef[0]    << P0;
        _VK[0]        << Eigen::Matrix<double,siz_x,siz_y>::Zero();
        _Vy[0]        << Eigen::Matrix<double,siz_y,1>::Zero();
        _Vr[0]        << Eigen::Matrix<double,siz_y,1>::Zero();
        _VS[0]        << Eigen::Matrix<double,siz_y,siz_y>::Zero();
        _Vxhat_aft[0] << x0;
        _VP_aft[0]    << P0;
        _Veps_xhat_bef_mean[0] = (x0_truth - _Vxhat_bef[0]);
        _Veps_xhat_aft_mean[0] = (x0_truth - _Vxhat_aft[0]);
        _Vr_mean[0]   << _Vr[0];
    }
    /**< initialize all internal vectors, although only required ones are xhat_aft and P_aft. FUNCTION INTENDED FOR TRANSITION FROM
     * ANOTHER FILTER IN WHICH ALL OR MOST VALUES CAN BE DETERMINED. */
    void initialize(const Eigen::Matrix<double,siz_x,1>& xhat_bef, const Eigen::Matrix<double,siz_x,siz_x>& P_bef, const Eigen::Matrix<double,siz_x,siz_y>& K,
                    const Eigen::Matrix<double,siz_y,1>& y, const Eigen::Matrix<double,siz_y,1>& r, const Eigen::Matrix<double,siz_y,siz_y>& S,
                    const Eigen::Matrix<double,siz_x,1>& xhat_aft, const Eigen::Matrix<double,siz_x,siz_x>& P_aft,
                    const Eigen::Matrix<double,siz_x,1>& eps_xhat_bef_mean, const Eigen::Matrix<double,siz_x,1> & eps_xhat_aft_mean,
                    const Eigen::Matrix<double,siz_y,1>& r_mean) {
        _Vxhat_bef[0] << xhat_bef;
        _VP_bef[0]    << P_bef;
        _VK[0]        << K;
        _Vy[0]        << y;
        _Vr[0]        << r;
        _VS[0]        << S;
        _Vxhat_aft[0] << xhat_aft;
        _VP_aft[0]    << P_aft;
        _Veps_xhat_bef_mean[0] = eps_xhat_bef_mean;
        _Veps_xhat_aft_mean[0] = eps_xhat_aft_mean;
        _Vr_mean[0]   << r_mean;
    }

    /**< execute prediction phase of EKF for the input position:
     * - updates the discrete time linear system matrix F.
     * - updates the discrete time linear system covariance matrix Qtilde.
     * - fills up the a priori state vector estimate xhat_bef.
     * - fills up the a priori state vector estimate covariance matrix Pbef. */
    void process_state(const unsigned int& i) {
        _Oekf.process_state(_Vxhat_bef[i], _VP_bef[i], _Vxhat_aft[i-1], _VP_aft[i-1]);
    }
    /**< execute update phase of EKF for the input position:
     * - updates the discrete time linear observation matrix H.
     * - updates the discrete time linear observation matrix Rtilde.
     * - fills up the optimum Kalman gain K.
     * - fills up the measurements y.
     * - fills up the innovations r.
     * - fills up the innovations covariance S.
     * - fills up the a posteriori state vector estimate xhat_aft.
     * - fills up the a posteriori state vector estimate covariance matrix Paft. */
    void process_observation(const unsigned int& i, const Eigen::Matrix<double,siz_y,1>& y) {
        _Vy[i] = y;
        _Oekf.process_observation(_VK[i], _Vr[i], _VS[i], _Vxhat_aft[i], _VP_aft[i], _Vxhat_bef[i], _VP_bef[i], _Vy[i]);
    }
    /**< execute the auxiliary phase of EKF for the input position based on the real state vector (to compare with estimate)
     * - fills up the a priori state vector estimate error (truth minus estimation) running mean.
     * - fills up the a posteriori state vector estimate error (truth minus estimation) running mean.
     * - fills up the innovations running mean for the input position.
     * */
    void process_auxiliary(const unsigned int& i, const Eigen::Matrix<double,siz_x,1> x_truth) {
        if (i==0) {
            _Veps_xhat_bef_mean[i] = _Vxhat_bef[i] - x_truth;
            _Veps_xhat_aft_mean[i] = _Vxhat_aft[i] - x_truth;
            _Vr_mean[i]            = _Vr[i];
            return;
        }
        _Veps_xhat_bef_mean[i] = (_Veps_xhat_bef_mean[i-1] * (i-1) + _Vxhat_bef[i] - x_truth) / i;
        _Veps_xhat_aft_mean[i] = (_Veps_xhat_aft_mean[i-1] * (i-1) + _Vxhat_aft[i] - x_truth) / i;
        _Vr_mean[i]            = (_Vr_mean[i-1]            * (i-1) + _Vr[i]) / i;


        /////////////////////////////////////////////////////////////////////////////////////////////////////
       // std::cout << i << "  " << std::fixed << std::setw(15) << std::setprecision(8) << _Vxhat_aft[i](0) - x_truth(0) << "  " << _Veps_xhat_aft_mean[i](0) << std::endl;
        /////////////////////////////////////////////////////////////////////////////////////////////////////
    }

    /**< ===== ===== ===== Getters ===== ===== ===== */
    /**< =========================================== */

    /**< get Kalman filter */
    const nav::ekf_<siz_x, siz_y, siz_w, siz_v>& get_ekf() const {return _Oekf;}

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

    /**< ===== ===== ===== Size ===== ===== ===== */
    /**< ======================================== */
    /**< returns size of vectors within filter */
    unsigned long get_size() const {return _Vxhat_aft.size();}
    /**< resize size of vectors within filter to new size, leaving only first nel members */
    void resize(const unsigned long& nel) {
        _Vxhat_bef.resize(nel);
        _Vxhat_aft.resize(nel);
        _VP_bef.resize(nel);
        _VP_aft.resize(nel);
        _VK.resize(nel);
        _Vy.resize(nel);
        _Vr.resize(nel);
        _VS.resize(nel);
        _Veps_xhat_bef_mean.resize(nel);
        _Veps_xhat_aft_mean.resize(nel);
        _Vr_mean.resize(nel);
    }
}; // closes class ekf_handler_

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}; // closes namespace nav

#endif
