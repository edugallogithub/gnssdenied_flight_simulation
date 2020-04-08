#ifndef NAV_FILTER_GPS01
#define NAV_FILTER_GPS01

#include "../nav.h"
#include "filter_gps.h"

namespace env {
    class geo;
}
namespace st {
    class sti;
}

/*
 * Considers complete sensors.
 * Complete equations.
 * Does not use any external variables.
 * Initializes with sensors, not truth.
 */

namespace nav {

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER GPS FUNCTOR STATE01
// ================================
// ================================

class NAV_API filter_gps_state01 : public nav::ekf_state_<12> {
private:
    /**< discrete time linear state covariance matrix */
    Eigen::Matrix<double,12,12> _Qtilde;
    /**< continuously updated position in NED */
    env::geodetic_coord _x_gdt_rad_m;
    /**< continuously updated velocity in NED */
    Eigen::Vector3d _v_ned_mps;
    /**< continuously updated specific force of BFS with respect to IRS expressed in BFS */
    Eigen::Vector3d _f_ibb_mps2;
    /**< continuously updated accelerometer error (everything except white noise) */
    Eigen::Vector3d _E_acc_mps2;

    /**< weak pointer to Earth model */
    const env::geo* const _Pgeo;
    /**< quaternion between NED and BFS */
    ang::rodrigues _q_nb;
    /**< angular velocity of ECEF with respect to IRS in NED */
    Eigen::Vector3d _w_ien_rps;
    /**< radius of curvature of prime vertical N */
    double _N_m;
    /**< radius of curvature of meridian M */
    double _M_m;
    /**< angular velocity of NED with respect to ECEF in NED */
    Eigen::Vector3d _w_enn_rps;
    /**< Coriolis acceleration in NED */
    Eigen::Vector3d _a_cor_n_mps2;
    /**< gravity vector in NED */
    Eigen::Vector3d _gc_n_mps2;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< constructor based on suite of sensors and Earth model */
    explicit filter_gps_state01(const sens::suite& Osuite, const env::geo& Ogeo);
    /**< default constructor */
    filter_gps_state01() = delete;
    /**< copy constructor */
    filter_gps_state01(const filter_gps_state01&) = delete;
    /**< move constructor */
    filter_gps_state01(filter_gps_state01&&) = delete;
    /**< destructor */
    ~filter_gps_state01() = default;
    /**< copy assignment */
    filter_gps_state01& operator=(const filter_gps_state01&) = delete;
    /**< move assignment */
    filter_gps_state01& operator=(filter_gps_state01&&) = delete;

    /**< updates the functor based on previous EKF state estimation and previous sensors outputs */
    void update(const Eigen::Matrix<double,12,1>& xhat_aft_prev, const st::st_nav_out& Ost_nav_out);
    /**< evaluates functor based on previous EKF state estimation, returning vector of size z. */
    Eigen::Matrix<double,12,1> eval(const Eigen::Matrix<double,12,1>& xhat_aft_prev) const override;
    /**< evaluates jacobian of functor with respect to state vector x based on state vector. */
    void jacobian_state(Eigen::Matrix<double,12,12>& A, const Eigen::Matrix<double,12,1>& xhat_aft_prev) const override;
    /**< returns covariance matrix */
    Eigen::Matrix<double,12,12> covariance_matrix() const override;
}; // closes class filter_gps_state01

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER GPS FUNCTOR OBSER01
// ================================
// ================================

class NAV_API filter_gps_obser01 : public nav::ekfd_obser_<12,3,9,3,9> {
private:
    /**< fast discrete time non linear covariance matrix */
    Eigen::Matrix<double,3,3> _R_fast;
    /**< slow discrete time non linear covariance matrix */
    Eigen::Matrix<double,9,9> _R_slow;
    /**< continuously updated position in NED */
    env::geodetic_coord _x_gdt_rad_m;
    /**< continuously updated velocity in NED */
    Eigen::Vector3d _v_ned_mps;
    /**< continuously updated specific force of BFS with respect to IRS expressed in BFS */
    Eigen::Vector3d _f_ibb_mps2;
    /**< continuously updated accelerometer error (everything except white noise) */
    Eigen::Vector3d _E_acc_mps2;
    /**< weak pointer to suite of sensors */
    const sens::suite* const _Psuite;
    /**< weak pointer to Earth model */
    const env::geo* const _Pgeo;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< constructor based on suite of sensors and Earth model */
    filter_gps_obser01(const sens::suite& Osuite, const env::geo& Ogeo);
    /**< default constructor */
    filter_gps_obser01() = delete;
    /**< copy constructor */
    filter_gps_obser01(const filter_gps_obser01&) = delete;
    /**< move constructor */
    filter_gps_obser01(filter_gps_obser01&&) = delete;
    /**< destructor */
    ~filter_gps_obser01() = default;
    /**< copy assignment */
    filter_gps_obser01& operator=(const filter_gps_obser01&) = delete;
    /**< move assignment */
    filter_gps_obser01& operator=(filter_gps_obser01&&) = delete;

    /**< initialization */
    void initialize(const env::geodetic_coord& x_gdt_rad_m_init);
    /**< updates the functor based on current EKF state estimation, current sensors outputs, and XXXXXXXXX */
    void update(const Eigen::Matrix<double,12,1>& xhat_bef);
    /**< evaluates functor (fast) based on current EKF state estimation, returning vector of size z. */
    Eigen::Matrix<double,3,1> eval_fast(const Eigen::Matrix<double,12,1>& xhat_bef) const override;
    /**< evaluates functor (slow) based on current EKF state estimation, returning vector of size z. */
    Eigen::Matrix<double,9,1> eval_slow(const Eigen::Matrix<double,12,1>& xhat_bef) const override;

    /**< evaluates jacobian of functor (fast) with respect to state vector x based on state vector. */
    void jacobian_state_fast(Eigen::Matrix<double,3,12>& output, const Eigen::Matrix<double,12,1>& xhat) const override;
    /**< evaluates jacobian of functor (slow) with respect to state vector x based on state vector. */
    void jacobian_state_slow(Eigen::Matrix<double,9,12>& output, const Eigen::Matrix<double,12,1>& xhat) const override;

    /**< evaluates jacobian of functor (fast) with respect to noise vector (w or v) based on state vector. */
    void jacobian_noise_fast(Eigen::Matrix<double,3,3>& output, const Eigen::Matrix<double,12,1>& xhat) const override;
    /**< evaluates jacobian of functor (slow) with respect to noise vector (w or v) based on state vector. */
    void jacobian_noise_slow(Eigen::Matrix<double,9,9>& output, const Eigen::Matrix<double,12,1>& xhat) const override;

    /**< returns covariance matrix (fast) */
    Eigen::Matrix<double,3,3> covariance_matrix_fast() const override;
    /**< returns covariance matrix (slow) */
    Eigen::Matrix<double,9,9> covariance_matrix_slow() const override;
}; // closes class filter_gps_obser01

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER GPS01
// ==================
// ==================

class NAV_API filter_gps01 : public filter_gps {
private:
    /**< pointer to extended Kalman filter */
    nav::ekfd_handler_<12,3,9,12,3,9>* _Pekfd_handler;
    /**< pointer to state functor */
    nav::filter_gps_state01* _Pstate;
    /**< pointer to observation functor */
    nav::filter_gps_obser01* _Pobser;
    /**< weak pointer to Earth model */
    const env::geo* const _Pgeo;

    /**< vector of observations */
    Eigen::Matrix<double,3,1> _y_fast;
    /**< vector of observations */
    Eigen::Matrix<double,9,1> _y_slow;
    /**< vector of truth states for filter evaluation */
    Eigen::Matrix<double,12,1> _Vx_truth;

    /**< creates text file describing the evolution of geodetic position */
    void textplot_xgdt(const std::string& txt_file_xhor, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out, const st::trj_gps_out& Otrj_gps_out) const;
    /**< creates text file describing the evolution of v_ned */
    void textplot_vned(const std::string& txt_file_vned, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out, const st::trj_gps_out& Otrj_gps_out) const;
    /**< creates text file describing the evolution of f_ibb */
    void textplot_fibb(const std::string& txt_file_fibb, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out, const st::trj_gps_out& Otrj_gps_out) const;
    /**< creates text file describing the evolution of the E_acc */
    void textplot_Eacc(const std::string& txt_file_E_acc, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out, const st::trj_gps_out& Otrj_gps_out) const;
    /**< creates text file describing the evolution of the accelerometer observations */
    void textplot_obs_acc(const std::string& txt_file_obs_acc, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out, const st::trj_gps_out& Otrj_gps_out) const;
    /**< creates text file describing the evolution of the ground speed observations */
    void textplot_obs_vned(const std::string& txt_file_obs_vned, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out, const st::trj_gps_out& Otrj_gps_out) const;
    /**< creates text file describing the evolution of the position observations */
    void textplot_obs_xgdt(const std::string& txt_file_obs_xgdt, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out, const st::trj_gps_out& Otrj_gps_out) const;
    /**< creates text file describing the evolution of the low frequency wind, DeltaT, and Deltap */
    void textplot_wind_DeltaTp(const std::string& txt_file_wind_DeltaTp, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out, const st::trj_gps_out& Otrj_gps_out) const;
public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< constructor based on suite of sensors and Earth model */
    filter_gps01(const sens::suite& Osuite, const env::geo& Ogeo);
    /**< default constructor */
    filter_gps01() = delete;
    /**< copy constructor */
    filter_gps01(const filter_gps01&) = delete;
    /**< move constructor */
    filter_gps01(filter_gps01&&) = delete;
    /**< destructor */
    ~filter_gps01();
    /**< copy assignment */
    filter_gps01& operator=(const filter_gps01&) = delete;
    /**< move assignment */
    filter_gps01& operator=(filter_gps01&&) = delete;

    /**< complete constructor with navigation time and sizes of navigation vector and gps vectors, which are not available at construction time */
    void finalize_constructor(const double& Deltat_sec_nav, const unsigned int& nel_nav, const unsigned int& nel_gps) override;
    /**< initialize filter filling up the initial navigation output state vector Ost_nav_out_init, based on the initial sensors output state vector Ost_sens_out_init,
     * the initial GPS output state vector Ost_gps_out_init, the initial navigation input state vector Ost_nav_in_init (for filter evaluation purposes only), and other
     * randomly computed variables required for filter initialization. */
    void initialize(st::st_nav_out& Ost_nav_out_init, const st::st_sens_out& Ost_sens_out_init, const st::st_gps_out& Ost_gps_out_init, const st::st_nav_in& Ost_nav_in_init,
                    const Eigen::Vector3d& E_acc_init, const Eigen::Vector3d& E_acc_std_init) override;
    /**< execute filter step filling up the navigation output state vector Ost_nav_out, based on the sensors output state vector Ost_sens_out, the GPS output
     * state vector Ost_gps_out, the navigation input state vector Ost_nav_in (for filter evaluation purposes only), the current positions for navigation and
     * GPS in their respective trajectory vectors, and the flag indicating whether there is a new GPS measurement or not. */
    void execute_step(st::st_nav_out& Ost_nav_out, const st::st_sens_out& Ost_sens_out, const st::st_gps_out& Ost_gps_out, const st::st_nav_in& Ost_nav_in, const unsigned int& s, const unsigned int& g, bool flag_gps) override;
    /**< resize of all filter components to input size */
    void resize(const unsigned long& nel_fast, const unsigned long& nel_slow) override;
    /**< returns size of all filter components */
    unsigned long get_size_fast() const override;
    unsigned long get_size_slow() const override;

    /**< creates text files describing the filter performance */
    void textplot(const boost::filesystem::path& path_folder, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out, const st::trj_gps_out& Otrj_gps_out) const override;
    /**< get extended Kalman filter to read */
    const nav::ekfd_handler_<12,3,9,12,3,9>& get_ekfd_handler() const {return *_Pekfd_handler;}

}; // closes class filter_gps01

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}; // closes namespace nav

#endif
