#ifndef NAV_FILTER_ATT03
#define NAV_FILTER_ATT03

#include "../nav.h"
#include "filter_att.h"
#include "math/math/low_pass.h"

namespace env {
    class geo;
}
namespace st {
    class sti;
}

/*
 * Neglects airspeed, turbulence, and wind differentials in body.
 * Much worse than option #2.
 */

/*
 * Considers noise, bias drift, and bias offset in gyroscopes (no scale or cross factor).
 * Considers noise, bias drift, and bias offset in accelerometers (no scale or cross factor).
 * Considers noise and bias offset in magnetometers (no scale or cross factor).
 * Complete equations.
 * Uses external variables: initial NED gravity, initial NED magnetic field.
 * Initializes with sensors, not truth.
 */

namespace nav {

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER ATTITUDE FUNCTOR STATE03
// =====================================
// =====================================

class NAV_API filter_att_state03 : public nav::ekf_state_<16> {
private:
    /**< discrete time linear state covariance matrix */
    Eigen::Matrix<double,16,16> _Qtilde;
    /**< continuously updated quaternion between NED and BFS */
    ang::rodrigues _q_nb;
    /**< continuously updated angular speed of BFS with respect to NED in BFS */
    Eigen::Vector3d _w_nbb_rps;
    /**< continuously updated gyroscope error (everything except white noise) */
    Eigen::Vector3d _E_gyr_rps;
    /**< continuously updated magnetometer error (everything except white noise) */
    Eigen::Vector3d _E_mag_nT;
    /**< continuously updated initial magnetic field error */
    Eigen::Vector3d _B_n_nT_dev;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< constructor based on suite of sensors */
    explicit filter_att_state03(const sens::suite& Osuite);
    /**< default constructor */
    filter_att_state03() = delete;
    /**< copy constructor */
    filter_att_state03(const filter_att_state03&) = delete;
    /**< move constructor */
    filter_att_state03(filter_att_state03&&) = delete;
    /**< destructor */
    ~filter_att_state03() = default;
    /**< copy assignment */
    filter_att_state03& operator=(const filter_att_state03&) = delete;
    /**< move assignment */
    filter_att_state03& operator=(filter_att_state03&&) = delete;

    /**< initialization */
    void initialize();
    /**< updates the functor based on previous EKF state estimation and previous sensors outputs */
    void update(const Eigen::Matrix<double,16,1>& xhat_aft_prev);
    /**< evaluates functor based on previous EKF state estimation, returning vector of size z. */
    Eigen::Matrix<double,16,1> eval(const Eigen::Matrix<double,16,1>& xhat_aft_prev) const override;
    /**< evaluates jacobian of functor with respect to state vector x based on state vector. */
    void jacobian_state(Eigen::Matrix<double,16,16>& A, const Eigen::Matrix<double,16,1>& xhat_aft_prev) const override;
    /**< returns covariance matrix */
    Eigen::Matrix<double,16,16> covariance_matrix() const override;
}; // closes class filter_att_state03

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER ATTITUDE FUNCTOR OBSER03
// =====================================
// =====================================

class NAV_API filter_att_obser03 : public nav::ekf_obser_<16,9,9> {
private:
    /**< discrete time non linear covariance matrix */
    Eigen::Matrix<double,9,9> _R;
    /**< continuously updated quaternion between NED and BFS */
    ang::rodrigues _q_nb;
    /**< continuously updated angular speed of BFS with respect to NED in BFS */
    Eigen::Vector3d _w_nbb_rps;
    /**< continuously updated gyroscope error (everything except white noise) */
    Eigen::Vector3d _E_gyr_rps;
    /**< continuously updated magnetometer error (everything except white noise) */
    Eigen::Vector3d _E_mag_nT;
    /**< continuously updated initial magnetic field error */
    Eigen::Vector3d _B_n_nT_dev;

    /**< weak pointer to Earth model */
    const env::geo* const _Pgeo;
    /**< flag indicating if there is gps signal (true) or not (false) */
    bool _flag_gps;

    /**< estimated aircraft geodetic coordinates */
    env::geodetic_coord _x_gdt_rad_m;
    /**< estimated aircraft velocity in NED */
    Eigen::Vector3d _v_n_mps;
    /**< estimated low frequency wind speed in NED */
    Eigen::Vector3d _vlf_n_mps;
    /**< estimated airspeed plus high frequency wind speed in NED */
    Eigen::Vector3d _vtashf_n_mps;
    /**< radius of curvature of prime vertical N */
    double _N_m;
    /**< radius of curvature of meridian M */
    double _M_m;
    /**< angular velocity of ECEF with respect to IRS in NED */
    Eigen::Vector3d _w_ien_rps;
    /**< angular velocity of NED with respect to ECEF in NED */
    Eigen::Vector3d _w_enn_rps;
    /**< Coriolis acceleration in NED */
    Eigen::Vector3d _a_cor_n_mps2;
    /**< gravity vector in NED */
    Eigen::Vector3d _gc_n_mps2_model;
    /**< NED magnetic field */
    Eigen::Vector3d _B_n_nT_model;
    /**< accelerometer error (everything except white noise) */
    Eigen::Vector3d _E_acc_mps2;

    /**< low pass filter to smooth and freeze the accelerometer error when no GPS */
    math::low_pass_triple* _Plpf_Eacc_mps2;
    /**< smoothed and frozen accelerometer error (everything except white noise) */
    Eigen::Vector3d _E_acc_mps2_smooth;

    /**< low pass filter to smooth (but not freeze) the ground speed when no GPS */
    math::low_pass_triple* _Plpf_v_n_mps;
    /**< smoothed (but not frozen) ground velocity in NED */
    Eigen::Vector3d _v_n_mps_smooth;

    /**< low pass filter to smooth (but not freeze) the low frequency wind speed when no GPS */
    math::low_pass_triple* _Plpf_vlf_n_mps;
    /**< smoothed (but not frozen) low frequency wind speed in NED */
    Eigen::Vector3d _vlf_n_mps_smooth;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< constructor based on suite of sensors and Earth model */
    filter_att_obser03(const sens::suite& Osuite, const env::geo& Ogeo);
    /**< default constructor */
    filter_att_obser03() = delete;
    /**< copy constructor */
    filter_att_obser03(const filter_att_obser03&) = delete;
    /**< move constructor */
    filter_att_obser03(filter_att_obser03&&) = delete;
    /**< destructor */
    ~filter_att_obser03();
    /**< copy assignment */
    filter_att_obser03& operator=(const filter_att_obser03&) = delete;
    /**< move assignment */
    filter_att_obser03& operator=(filter_att_obser03&&) = delete;

    /**< complete constructor with navigation time, which is not available at construction time */
    void finalize_constructor(const double& Deltat_sec_nav);
    /**< initialization */
    void initialize();
    /**< switch to no gps mode */
    void switch_to_gps_lost_mode();
    /**< updates the functor based on current EKF state estimation, current sensors outputs, and XXXXXXXXX */
    void update(const Eigen::Matrix<double,16,1>& xhat_bef, const st::st_nav_out& Ost_nav_out_prev);
    /**< evaluates functor based on current EKF state estimation, returning vector of size z. */
    Eigen::Matrix<double,9,1> eval(const Eigen::Matrix<double,16,1>& xhat_bef) const override;
    /**< evaluates jacobian of functor with respect to state vector x based on state vector. */
    void jacobian_state(Eigen::Matrix<double,9,16>& output, const Eigen::Matrix<double,16,1>& xhat) const override;
    /**< evaluates jacobian of functor with respect to noise vector (w or v) based on state vector. */
    void jacobian_noise(Eigen::Matrix<double,9,9>& output, const Eigen::Matrix<double,16,1>& xhat) const override;
    /**< returns covariance matrix */
    Eigen::Matrix<double,9,9> covariance_matrix() const override;
}; // closes class filter_att_obser03

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER ATTITUDE03
// =======================
// =======================

class NAV_API filter_att03 : public filter_att {
private:
    /**< pointer to extended Kalman filter */
    nav::ekf_handler_<16,9,16,9>* _Pekf_handler;
    /**< pointer to state functor */
    nav::filter_att_state03* _Pstate;
    /**< pointer to observation functor */
    nav::filter_att_obser03* _Pobser;

    /**< vector of observations */
    Eigen::Matrix<double,9,1> _y;
    /**< vector of truth states for filter evaluation */
    Eigen::Matrix<double,16,1> _Vx_truth;

    /**< creates text file describing the evolution of q_nb */
    void textplot_qnb(const std::string& txt_file_qnb, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out) const;
    /**< creates text file describing the evolution of the euler angles */
    void textplot_euler(const std::string& txt_file_euler, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out) const;
    /**< creates text file describing the evolution of w_nbb */
    void textplot_wnbb(const std::string& txt_file_wnbb, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out) const;
    /**< creates text file describing the evolution of bias_gyr */
    void textplot_E_gyr(const std::string& txt_file_E_gyr, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out) const;
    /**< creates text file describing the evolution of bias_mag and error_mag*/
    void textplot_E_mag(const std::string& txt_file_E_mag, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out) const;
    /**< creates text file describing the evolution of gyroscope observations */
    void textplot_obs_gyr(const std::string& txt_file_obs_gyr, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out) const;
    /**< creates text file describing the evolution of accelerometer observations */
    void textplot_obs_acc(const std::string& txt_file_obs_acc, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out) const;
    /**< creates text file describing the evolution of magnetometer observations */
    void textplot_obs_mag(const std::string& txt_file_obs_mag, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out) const;
public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< constructor based on suite of sensors and Earth model */
    filter_att03(const sens::suite& Osuite, const env::geo& Ogeo);
    /**< default constructor */
    filter_att03() = delete;
    /**< copy constructor */
    filter_att03(const filter_att03&) = delete;
    /**< move constructor */
    filter_att03(filter_att03&&) = delete;
    /**< destructor */
    ~filter_att03();
    /**< copy assignment */
    filter_att03& operator=(const filter_att03&) = delete;
    /**< move assignment */
    filter_att03& operator=(filter_att03&&) = delete;

    /**< complete constructor with navigation time and size of navigation vector, which are not available at construction time */
    void finalize_constructor(const double& Deltat_sec_nav, const unsigned int& nel_nav) override;
    /**< initialize filter filling up the initial navigation output state vector Ost_nav_out_init, based on the initial sensors output state vector Ost_sens_out_init,
     * the initial navigation input state vector Ost_nav_in_init (for filter evaluation purposes only), and other randomly computed variables required
     * for filter initialization, such as initial attitude, initial gyroscope error, initial magnetometer error, initial gravity vector, and
     * initial magnetic field). */
    void initialize(st::st_nav_out& Ost_nav_out_init, const st::st_sens_out& Ost_sens_out_init, const st::st_nav_in& Ost_nav_in_init,
                    const ang::rodrigues& q_nb_init,
                    const Eigen::Vector3d& E_gyr_init, const Eigen::Vector3d& E_gyr_std_init,
                    const Eigen::Vector3d& E_mag_init, const Eigen::Vector3d& E_mag_std_init,
                    const Eigen::Vector3d& E_B_n_init, const Eigen::Vector3d& E_B_n_std_init) override;
    /**< execute filter step filling up the navigation output state vector Ost_nav_out, based on the previous navigation output state vector Ost_nav_out_prev,
     * the sensors output state vector Ost_sens_out, the navigation input state vector Ost_nav_in (for filter evaluation purposes only), and the current
     * navigation trajectory vector position. */
    void execute_step(st::st_nav_out& Ost_nav_out, const st::st_nav_out& Ost_nav_out_prev, const st::st_sens_out& Ost_sens_out, const st::st_nav_in& Ost_nav_in, const unsigned int& s) override;
    /**< resize of all filter components to input size */
    void resize(const unsigned long& nel) override;
    /**< returns size of all filter components */
    unsigned long get_size() const override;
    /**< switch to no gps mode */
    void switch_to_gps_lost_mode() override;
    /**< creates text files describing the filter performance */
    void textplot(const boost::filesystem::path& path_folder, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out) const override;
}; // closes class filter_att03

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}; // closes namespace nav

#endif
