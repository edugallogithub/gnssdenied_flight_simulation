#ifndef NAV_EKFD_HANDLER_TEMPLATE
#define NAV_EKFD_HANDLER_TEMPLATE

#include "../nav.h"
#include "ekfd_.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <iostream>
#include <unsupported/Eigen/MatrixFunctions>
#include <iomanip>

namespace nav {

// CLASS EXTENDED KALMAN FILTER DUAL EKFD HANDLER
// ==============================================
// ==============================================

template <unsigned short siz_x, unsigned short siz_y1, unsigned short siz_y2, unsigned short siz_w, unsigned short siz_v1, unsigned short siz_v2>
class NAV_API ekfd_handler_ {
private:
    /**< weak pointer to extended Kalman filter */
    nav::ekfd_<siz_x, siz_y1, siz_y2, siz_w, siz_v1, siz_v2> _Oekfd;

    /**< vector of a priori state estimations */
    std::vector<Eigen::Matrix<double,siz_x,1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_x,1>>> _Vxhat_bef;
    /**< vector of a posteriori state estimations */
    std::vector<Eigen::Matrix<double,siz_x,1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_x,1>>> _Vxhat_aft;
    /**< matrix of a priori state estimation error covariances */
    std::vector<Eigen::Matrix<double,siz_x,siz_x>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_x,siz_x>>> _VP_bef;
    /**< matrix of a postetiori state estimation error covariances */
    std::vector<Eigen::Matrix<double,siz_x,siz_x>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_x,siz_x>>> _VP_aft;
    /**< matrix of fast optimum Kalman gains */
    std::vector<Eigen::Matrix<double,siz_x,siz_y1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_x,siz_y1>>> _VK_fast;
    /**< matrix of slow optimum Kalman gains */
    std::vector<Eigen::Matrix<double,siz_x,siz_y2>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_x,siz_y2>>> _VK_slow;
    /**< vector of fast measurements */
    std::vector<Eigen::Matrix<double,siz_y1,1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_y1,1>>> _Vy_fast;
    /**< vector of slow measurements */
    std::vector<Eigen::Matrix<double,siz_y2,1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_y2,1>>> _Vy_slow;
    /**< vector of fast innovations */
    std::vector<Eigen::Matrix<double,siz_y1,1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_y1,1>>> _Vr_fast;
    /**< vector of slow innovations */
    std::vector<Eigen::Matrix<double,siz_y2,1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_y2,1>>> _Vr_slow;
    /**< vector of fast innovation covariances */
    std::vector<Eigen::Matrix<double,siz_y1,siz_y1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_y1,siz_y1>>> _VS_fast;
    /**< vector of fast innovation covariances */
    std::vector<Eigen::Matrix<double,siz_y2,siz_y2>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_y2,siz_y2>>> _VS_slow;

    /**< vector of a priori state estimation errors (truth minus estimation) running mean */
    std::vector<Eigen::Matrix<double,siz_x,1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_x,1>>> _Veps_xhat_bef_mean;
    /**< vector of a posteriori state estimation errors (truth minus estimation) running mean */
    std::vector<Eigen::Matrix<double,siz_x,1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_x,1>>> _Veps_xhat_aft_mean;
    /**< vector of fast innovations state estimations running mean */
    std::vector<Eigen::Matrix<double,siz_y1,1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_y1,1>>> _Vr_mean_fast;
    /**< vector of slow innovations state estimations running mean */
    std::vector<Eigen::Matrix<double,siz_y2,1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_y2,1>>> _Vr_mean_slow;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**< default constructor */
    ekfd_handler_() = delete;
    /**< constructor based on size of navigation vector and filter time step */
    ekfd_handler_(const unsigned int& nel_nav, const unsigned int& nel_gps, const double Deltat_sec_nav, nav::ekf_state_<siz_x>& Ostate, nav::ekfd_obser_<siz_x, siz_y1, siz_y2, siz_v1, siz_v2>& Oobser)
    : _Oekfd(Deltat_sec_nav, Ostate, Oobser), _Vxhat_bef(nel_nav), _Vxhat_aft(nel_nav), _VP_bef(nel_nav), _VP_aft(nel_nav), _VK_fast(nel_nav), _VK_slow(nel_gps),
      _Vy_fast(nel_nav), _Vy_slow(nel_gps), _Vr_fast(nel_nav), _Vr_slow(nel_gps), _VS_fast(nel_nav), _VS_slow(nel_gps),
      _Veps_xhat_bef_mean(nel_nav), _Veps_xhat_aft_mean(nel_nav), _Vr_mean_fast(nel_nav), _Vr_mean_slow(nel_gps) {}
    /**< copy constructor */
    ekfd_handler_(const ekfd_handler_&) = delete;
    /**< move constructor */
    ekfd_handler_(ekfd_handler_&&) = delete;
    /**< destructor */
    ~ekfd_handler_() = default;
    /**< copy assignment */
    ekfd_handler_& operator=(const ekfd_handler_&) = delete;
    /**< move assignment */
    ekfd_handler_& operator=(ekfd_handler_&&) = delete;

    /**< initialize all internal vectors, although only required ones are xhat_aft and P_aft */
    void initialize(const Eigen::Matrix<double,siz_x,1>& x0, const Eigen::Matrix<double,siz_x,siz_x>& P0, const Eigen::Matrix<double,siz_x,1>& x0_truth) {
        _Vxhat_bef[0] << x0;
        _VP_bef[0]    << P0;
        _VK_fast[0]   << Eigen::Matrix<double,siz_x,siz_y1>::Zero();
        _VK_slow[0]   << Eigen::Matrix<double,siz_x,siz_y2>::Zero();
        _Vy_fast[0]   << Eigen::Matrix<double,siz_y1,1>::Zero();
        _Vy_slow[0]   << Eigen::Matrix<double,siz_y2,1>::Zero();
        _Vr_fast[0]   << Eigen::Matrix<double,siz_y1,1>::Zero();
        _Vr_slow[0]   << Eigen::Matrix<double,siz_y2,1>::Zero();
        _VS_fast[0]   << Eigen::Matrix<double,siz_y1,siz_y1>::Zero();
        _VS_slow[0]   << Eigen::Matrix<double,siz_y2,siz_y2>::Zero();
        _Vxhat_aft[0] << x0;
        _VP_aft[0]    << P0;
        _Veps_xhat_bef_mean[0] << (x0_truth - _Vxhat_bef[0]);
        _Veps_xhat_aft_mean[0] << (x0_truth - _Vxhat_aft[0]);
        _Vr_mean_fast[0]       << _Vr_fast[0];
        _Vr_mean_slow[0]       << _Vr_slow[0];
    }

    /**< execute prediction phase of EKF for the input position:
     * - updates the discrete time linear system matrix F.
     * - updates the discrete time linear system covariance matrix Qtilde.
     * - fills up the a priori state vector estimate xhat_bef.
     * - fills up the a priori state vector estimate covariance matrix Pbef. */
    void process_state(const unsigned int& s) {
        _Oekfd.process_state(_Vxhat_bef[s], _VP_bef[s], _Vxhat_aft[s-1], _VP_aft[s-1]);
    }
    /**< execute fast update phase of EKF for the input position:
     * - updates the discrete time linear observation matrix H.
     * - updates the discrete time linear observation matrix Rtilde.
     * - fills up the optimum Kalman gain K.
     * - fills up the measurements y.
     * - fills up the innovations r.
     * - fills up the innovations covariance S.
     * - fills up the a posteriori state vector estimate xhat_aft.
     * - fills up the a posteriori state vector estimate covariance matrix Paft. */
    void process_observation_fast(const unsigned int& s, const Eigen::Matrix<double,siz_y1,1>& y) {
        _Vy_fast[s] = y;
        _Oekfd.process_observation_fast(_VK_fast[s], _Vr_fast[s], _VS_fast[s], _Vxhat_aft[s], _VP_aft[s], _Vxhat_bef[s], _VP_bef[s], _Vy_fast[s]);
    }
    /**< execute slow update phase of EKF for the input position:
     * - updates the discrete time linear observation matrix H.
     * - updates the discrete time linear observation matrix Rtilde.
     * - fills up the optimum Kalman gain K.
     * - fills up the measurements y.
     * - fills up the innovations r.
     * - fills up the innovations covariance S.
     * - fills up the a posteriori state vector estimate xhat_aft.
     * - fills up the a posteriori state vector estimate covariance matrix Paft. */
    void process_observation_slow(const unsigned int& s, const unsigned int& g, const Eigen::Matrix<double,siz_y2,1>& y) {
        _Vy_slow[g] = y;
        _Oekfd.process_observation_slow(_VK_slow[g], _Vr_slow[g], _VS_slow[g], _Vxhat_aft[s], _VP_aft[s], _Vxhat_bef[s], _VP_bef[s], _Vy_slow[g]);
    }
    /**< execute the auxiliary phase of EKF for the input position based on the real state vector (to compare with estimate)
     * - fills up the a priori state vector estimate error (truth minus estimation) running mean.
     * - fills up the a posteriori state vector estimate error (truth minus estimation) running mean.
     * - fills up the innovations running mean for the input position.
     * */
    void process_auxiliary_fast(const unsigned int& s, const Eigen::Matrix<double,siz_x,1> x_truth) {
        if (s==0) {
            _Veps_xhat_bef_mean[s] = _Vxhat_bef[s] - x_truth;
            _Veps_xhat_aft_mean[s] = _Vxhat_aft[s] - x_truth;
            _Vr_mean_fast[s]       = _Vr_fast[s];
            return;
        }
        _Veps_xhat_bef_mean[s] = (_Veps_xhat_bef_mean[s-1] * (s-1) + _Vxhat_bef[s] - x_truth) / s;
        _Veps_xhat_aft_mean[s] = (_Veps_xhat_aft_mean[s-1] * (s-1) + _Vxhat_aft[s] - x_truth) / s;
        _Vr_mean_fast[s]       = (_Vr_mean_fast[s-1]       * (s-1) + _Vr_fast[s]) / s;
    }
    /**< execute the auxiliary phase of EKF for the input position based on the real state vector (to compare with estimate)
     * - fills up the a priori state vector estimate error (truth minus estimation) running mean.
     * - fills up the a posteriori state vector estimate error (truth minus estimation) running mean.
     * - fills up the innovations running mean for the input position.
     * */
    void process_auxiliary_slow(const unsigned int& s, const unsigned int& g, const Eigen::Matrix<double,siz_x,1> x_truth) {
        if (s==0) {
            _Veps_xhat_bef_mean[s] = _Vxhat_bef[s] - x_truth;
            _Veps_xhat_aft_mean[s] = _Vxhat_aft[s] - x_truth;
            _Vr_mean_fast[s]       = _Vr_fast[s];
            _Vr_mean_slow[g]       = _Vr_slow[g];
            return;
        }
        _Veps_xhat_bef_mean[s] = (_Veps_xhat_bef_mean[s-1] * (s-1) + _Vxhat_bef[s] - x_truth) / s;
        _Veps_xhat_aft_mean[s] = (_Veps_xhat_aft_mean[s-1] * (s-1) + _Vxhat_aft[s] - x_truth) / s;
        _Vr_mean_fast[s]       = (_Vr_mean_fast[s-1]       * (s-1) + _Vr_fast[s]) / s;
        _Vr_mean_slow[g]       = (_Vr_mean_slow[g-1]       * (g-1) + _Vr_slow[g]) / g;
    }

    /**< ===== ===== ===== Getters ===== ===== ===== */
    /**< =========================================== */

    /**< get Kalman filter */
    const nav::ekfd_<siz_x, siz_y1, siz_y2, siz_w, siz_v1, siz_v2>& get_ekfd() const {return _Oekfd;}

    /**< get vector of a priori state estimations */
    const std::vector<Eigen::Matrix<double,siz_x,1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_x,1>>>& get_xhat_bef() const {return _Vxhat_bef;}
    /**< get vector of a posteriori state estimations */
    const std::vector<Eigen::Matrix<double,siz_x,1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_x,1>>>& get_xhat_aft() const {return _Vxhat_aft;}
    /**< get matrix of a priori state estimation error covariances */
    const std::vector<Eigen::Matrix<double,siz_x,siz_x>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_x,siz_x>>>& get_P_bef() const {return _VP_bef;}
    /**< get matrix of a postetiori state estimation error covariances */
    const std::vector<Eigen::Matrix<double,siz_x,siz_x>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_x,siz_x>>>& get_P_aft() const {return _VP_aft;}
    /**< get matrix of fast optimum Kalman gains */
    const std::vector<Eigen::Matrix<double,siz_x,siz_y1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_x,siz_y1>>>& get_K_fast() const {return _VK_fast;}
    std::vector<Eigen::Matrix<double,siz_x,siz_y1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_x,siz_y1>>>& get_K_fast() {return _VK_fast;}
    const std::vector<Eigen::Matrix<double,siz_x,siz_y2>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_x,siz_y2>>>& get_K_slow() const {return _VK_slow;}
    /**< get vector of fast measurements */
    const std::vector<Eigen::Matrix<double,siz_y1,1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_y1,1>>>& get_y_fast() const {return _Vy_fast;}
    std::vector<Eigen::Matrix<double,siz_y1,1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_y1,1>>>& get_y_fast() {return _Vy_fast;}
    const std::vector<Eigen::Matrix<double,siz_y2,1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_y2,1>>>& get_y_slow() const {return _Vy_slow;}
    /**< get vector of fast innovations */
    const std::vector<Eigen::Matrix<double,siz_y1,1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_y1,1>>>& get_r_fast() const {return _Vr_fast;}
    std::vector<Eigen::Matrix<double,siz_y1,1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_y1,1>>>& get_r_fast() {return _Vr_fast;}
    const std::vector<Eigen::Matrix<double,siz_y2,1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_y2,1>>>& get_r_slow() const {return _Vr_slow;}
    /**< get vector of fast innovation covariances */
    const std::vector<Eigen::Matrix<double,siz_y1,siz_y1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_y1,siz_y1>>>& get_S_fast() const {return _VS_fast;}
    std::vector<Eigen::Matrix<double,siz_y1,siz_y1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_y1,siz_y1>>>& get_S_fast() {return _VS_fast;}
    const std::vector<Eigen::Matrix<double,siz_y2,siz_y2>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_y2,siz_y2>>>& get_S_slow() const {return _VS_slow;}

    /**< get vector of a priori state estimation errors (truth minus estimation) running mean */
    const std::vector<Eigen::Matrix<double,siz_x,1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_x,1>>>& get_eps_xhat_bef_mean() const {return _Veps_xhat_bef_mean;}
    /**< get vector of a posteriori state estimation errors (truth minus estimation) running mean */
    const std::vector<Eigen::Matrix<double,siz_x,1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_x,1>>>& get_eps_xhat_aft_mean() const {return _Veps_xhat_aft_mean;}
    /**< get vector of fast innovations state estimations running mean */
    const std::vector<Eigen::Matrix<double,siz_y1,1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_y1,1>>>& get_r_mean_fast() const {return _Vr_mean_fast;}
    /**< get vector of slow innovations state estimations running mean */
    const std::vector<Eigen::Matrix<double,siz_y2,1>,Eigen::aligned_allocator<Eigen::Matrix<double,siz_y2,1>>>& get_r_mean_slow() const {return _Vr_mean_slow;}


    /**< ===== ===== ===== Size ===== ===== ===== */
    /**< ======================================== */
    /**< returns size of vectors within filter */
    unsigned long get_size_fast() const {return _Vxhat_aft.size();}
    /**< returns size of vectors within filter */
    unsigned long get_size_slow() const {return _Vy_slow.size();}

    /**< resize size of vectors within filter to new size, leaving only first nel members */
    void resize(const unsigned long& nel_fast, const unsigned long& nel_slow) {
        _Vxhat_bef.resize(nel_fast);
        _Vxhat_aft.resize(nel_fast);
        _VP_bef.resize(nel_fast);
        _VP_aft.resize(nel_fast);
        _VK_fast.resize(nel_fast);
        _VK_slow.resize(nel_slow);
        _Vy_fast.resize(nel_fast);
        _Vy_slow.resize(nel_slow);
        _Vr_fast.resize(nel_fast);
        _Vr_slow.resize(nel_slow);
        _VS_fast.resize(nel_fast);
        _VS_slow.resize(nel_slow);
        _Veps_xhat_bef_mean.resize(nel_fast);
        _Veps_xhat_aft_mean.resize(nel_fast);
        _Vr_mean_fast.resize(nel_fast);
        _Vr_mean_slow.resize(nel_slow);
    }
}; // closes class ekfd_handler_

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}; // closes namespace nav

#endif
