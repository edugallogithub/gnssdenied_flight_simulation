#include "filter_att01.h"
#include "ang/tools.h"
#include "env/geo.h"
#include "ang/rotate/dcm.h"
#include "ang/rotate/rotv.h"
#include "acft/st/trj_sens_in.h"
#include "acft/st/trj_nav_out.h"
#include "acft/st/trj_truth.h"
#include "acft/st/sti.h"
#include <fstream>
#include <iostream>

/*
 * State vector x components (siz_x == 13):
 * [0-3]   --> q_nb
 * [4-6]   --> w_nbb_rps
 * [7-9]   --> E_gyr_rps
 * [10-12] --> E_mag_nT
 *
 * Observation vector y components (siz_y == 9):
 * [0-2] --> w_ibb_rps
 * [3-5] --> f_ibb_mps2
 * [6-9] --> B_bfs_nT
 *
 * State noise vector w components (siz_w == 10):
 * - It is a dummy as all equations are perfect but I need some noise
 *
 * Observation noise vector v components (siz_v == 9):
 * [0-2] --> noise_w_ibb_rps
 * [3-5] --> noise_f_ibb_mps2
 * [6-9] --> noise_B_b_nT
 */

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER ATTITUDE FUNCTOR STATE01
// =====================================
// =====================================

nav::filter_att_state01::filter_att_state01(const sens::suite& Osuite)
: _Qtilde(Eigen::Matrix<double,13,13>::Zero()) {
    double a = 5e-15; //5e-15;
    double b = 1e-6; //1e-6;
    double c = std::pow(Osuite.get_sens_gyr().get_std_bias_drift(),2); // ~6e-14
    double d = 1e-14; // 1e-14
    _Qtilde.diagonal() << a, a, a, a, b, b, b, c, c, c, d, d, d;
}
/* constructor based on suite of sensors */

void nav::filter_att_state01::initialize() {
}
/* initialization */

void nav::filter_att_state01::update(const Eigen::Matrix<double,13,1>& xhat_aft_prev) {
    _q_nb      << xhat_aft_prev.segment<4>(0);
    _w_nbb_rps << xhat_aft_prev.segment<3>(4);
    _E_gyr_rps << xhat_aft_prev.segment<3>(7);
    _E_mag_nT  << xhat_aft_prev.segment<3>(10);
}
/* updates the functor based on previous EKF state estimation and previous sensors outputs */

Eigen::Matrix<double,13,1> nav::filter_att_state01::eval(const Eigen::Matrix<double,13,1>& xhat_aft_prev) const {
    Eigen::Matrix<double,13, 1> res;
    res.segment<4>(0)  = _q_nb.omegabody2dot(_w_nbb_rps);
    res.segment<3>(4)  = Eigen::Vector3d::Zero();
    res.segment<3>(7)  = Eigen::Vector3d::Zero();
    res.segment<3>(10) = Eigen::Vector3d::Zero();
    return res;
}
/* evaluates functor based on previous EKF state estimation, returning vector of size z. */

void nav::filter_att_state01::jacobian_state(Eigen::Matrix<double,13,13>& A, const Eigen::Matrix<double,13,1>& xhat_aft_prev) const {
    A.block<4,4>(0,0) = ang::tools::right_skew43(_w_nbb_rps) * 0.5;
    A.block<4,3>(0,4) = ang::tools::skew43(_q_nb) * 0.5;
}
/* evaluates jacobian of functor with respect to state vector x based on state vector. */

Eigen::Matrix<double,13,13> nav::filter_att_state01::covariance_matrix() const {
    return _Qtilde;
}
/* returns covariance matrix */

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER ATTITUDE FUNCTOR OBSER01
// =====================================
// =====================================

nav::filter_att_obser01::filter_att_obser01(const sens::suite& Osuite, const env::geo& Ogeo)
: _Pgeo(&Ogeo), _R(Eigen::Matrix<double,9,9>::Zero()) {
    // The following numbers are obtained from test_turb_obtain_differential() in earth_test::Tenvironment
    // and correspond to the approximate covariance (with zero mean) introduced by the turbulence. It does
    // not include the acceleration of the low frequency wind (very small) nor the tas acceleration (bigger).
    // The standard deviation is approximately constant at ~0.61 [mps2] for vdot_turb_bfs_1 and ~0.75 [mps2] for
    // vdot_turb_bfs_2 and vdot_turb_bfs_3. The variance caused by not including the wind body differential
    // with time in the accelerometer equation is hence 0.372 [m2ps4] for the x axis and 0.563 [m2ps4] for
    // the 2nd and 3rd axes. Note that this is for the wind, the tas acceleration not included.
    double var_turb_bfs_diff_m2ps4_x  = 0.372;
    double var_turb_bfs_diff_m2ps4_yz = 0.563;

    double a = std::pow(Osuite.get_sens_gyr().get_std_white_noise(),2.0);
    double b = std::pow(Osuite.get_sens_acc().get_std_white_noise(),2.0);
    double c = 10 * std::pow(Osuite.get_sens_mag().get_std_white_noise(),2.0);

    _R.block<3,3>(0,0).diagonal() << a, a, a;
    _R.block<3,3>(3,3).diagonal() << b + var_turb_bfs_diff_m2ps4_x, b + var_turb_bfs_diff_m2ps4_yz, b + var_turb_bfs_diff_m2ps4_yz;
    _R.block<3,3>(6,6).diagonal() << c, c, c;
}
/* constructor based on suite of sensors and Earth model */

void nav::filter_att_obser01::update(const Eigen::Matrix<double,13,1>& xhat_bef, const st::st_nav_out& Ost_nav_out_prev) {
    _q_nb      << xhat_bef.segment<4>(0);
    _w_nbb_rps << xhat_bef.segment<3>(4);
    _E_gyr_rps << xhat_bef.segment<3>(7);
    _E_mag_nT  << xhat_bef.segment<3>(10);

    _v_n_mps      = Ost_nav_out_prev.get_v_n_mps();
    _w_ien_rps    = _Pgeo->compute_wien_rps(Ost_nav_out_prev.get_x_gdt_rad_m().get_phi_rad());
    _N_m          = _Pgeo->radius_vert(Ost_nav_out_prev.get_x_gdt_rad_m().get_phi_rad());
    _M_m          = _Pgeo->radius_mer(Ost_nav_out_prev.get_x_gdt_rad_m().get_phi_rad(), _N_m);
    _w_enn_rps    = _Pgeo->compute_wenn_rps(_v_n_mps, _N_m, _M_m, Ost_nav_out_prev.get_x_gdt_rad_m().get_phi_rad(), Ost_nav_out_prev.get_x_gdt_rad_m().get_h_m());
    _a_cor_n_mps2 = _Pgeo->compute_coriolis_n(_v_n_mps, _w_ien_rps);
    _gc_n_mps2    = _Pgeo->compute_gravity_n_model(Ost_nav_out_prev.get_x_gdt_rad_m());
    _B_n_nT       = _Pgeo->get_mag().compute_B_n_nT_model(Ost_nav_out_prev.get_t_sec(), Ost_nav_out_prev.get_x_gdt_rad_m());
    _E_acc_mps2   = Ost_nav_out_prev.get_E_acc_mps2();
}
/* updates the functor based on current EKF state estimation, current sensors outputs, and XXXXXXXXX */

Eigen::Matrix<double,9,1> nav::filter_att_obser01::eval(const Eigen::Matrix<double,13,1>& xhat_bef) const {
    Eigen::Matrix<double,9,1> res = Eigen::Matrix<double,9,1>::Zero();
    res.segment<3>(0) = _w_nbb_rps + _q_nb / (_w_enn_rps + _w_ien_rps) + _E_gyr_rps;
    res.segment<3>(3) = _w_nbb_rps.cross(_q_nb / _v_n_mps) + _q_nb / (_w_enn_rps.cross(_v_n_mps) - _gc_n_mps2 + _a_cor_n_mps2) + _E_acc_mps2;
    res.segment<3>(6) = _q_nb / _B_n_nT + _E_mag_nT;
    return res;
}
/* evaluates functor based on current EKF state estimation, returning vector of size z. */

void nav::filter_att_obser01::jacobian_state(Eigen::Matrix<double,9,13>& H, const Eigen::Matrix<double,13,1>& xhat) const {
    H.block<3,4>(0,0)  = _q_nb.jacobian_quat_backward_rotation(_w_enn_rps + _w_ien_rps);
    H.block<3,3>(0,4)  = Eigen::Matrix3d::Identity();
    H.block<3,3>(0,7)  = Eigen::Matrix3d::Identity();
    H.block<3,4>(3,0)  = _q_nb.jacobian_quat_backward_rotation(_w_enn_rps.cross(_v_n_mps) - _gc_n_mps2 + _a_cor_n_mps2) + ang::tools::skew3(_w_nbb_rps) * _q_nb.jacobian_quat_backward_rotation(_v_n_mps);
    H.block<3,3>(3,4)  = ang::tools::right_skew3(_q_nb / _v_n_mps);
    H.block<3,4>(6,0)  = _q_nb.jacobian_quat_backward_rotation(_B_n_nT);
    H.block<3,3>(6,10) = Eigen::Matrix3d::Identity();
}
/* evaluates jacobian of functor with respect to state vector x based on state vector. */

void nav::filter_att_obser01::jacobian_noise(Eigen::Matrix<double,9,9>& M, const Eigen::Matrix<double,13,1>& xhat) const {
    M.diagonal() = Eigen::Matrix<double,9,1>::Ones();;
}
/* evaluates jacobian of functor with respect to noise vector (w or v) based on state vector. */

Eigen::Matrix<double,9,9> nav::filter_att_obser01::covariance_matrix() const {
    return _R;
}
/* returns covariance matrix */

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER ATTITUDE01
// =======================
// =======================

nav::filter_att01::filter_att01(const sens::suite& Osuite, const env::geo& Ogeo)
: filter_att(Osuite), _Pekf_handler(nullptr), _Pstate(new nav::filter_att_state01(Osuite)), _Pobser(new nav::filter_att_obser01(Osuite, Ogeo)) {
}
/* constructor based on suite of sensors and Earth model */

nav::filter_att01::~filter_att01() {
    delete _Pekf_handler;
    delete _Pstate;
    delete _Pobser;
}
/* destructor */

void nav::filter_att01::finalize_constructor(const double& Deltat_sec_nav, const unsigned int& nel_nav) {
    _Pekf_handler = new nav::ekf_handler_<13,9,13,9>(nel_nav, Deltat_sec_nav, *_Pstate, *_Pobser);
}
/* complete constructor with navigation time and size of navigation vector, which are not available at construction time */

void nav::filter_att01::initialize(st::st_nav_out& Ost_nav_out_init, const st::st_sens_out& Ost_sens_out_init, const st::st_nav_in& Ost_nav_in_init,
                                   const ang::rodrigues& q_nb_init,
                                   const Eigen::Vector3d& E_gyr_init, const Eigen::Vector3d& E_gyr_std_init,
                                   const Eigen::Vector3d& E_mag_init, const Eigen::Vector3d& E_mag_std_init,
                                   const Eigen::Vector3d& E_B_n_init, const Eigen::Vector3d& E_B_n_std_init) {
    // functor initialization
    _Pstate->initialize();

    // Initial state estimate shall be the mean, but there is no way to compute that.
    // I use the initial sensed values when possible, zero for the biases, and zero for the unknown variables.
    Eigen::Matrix<double,13,1> Vx_init;
    Vx_init.segment<4>(0)  = q_nb_init;
    Vx_init.segment<3>(4)  = Ost_sens_out_init.get_w_ibb_rps() - E_gyr_init; // note ibb, not nbb ///////////////// MOST LIKELY I CAN DO BETTER ESTIMATING IEN Y ENB
    Vx_init.segment<3>(7)  = E_gyr_init;
    Vx_init.segment<3>(10) = E_mag_init;

    // Initial covariance of the state estimate
    Eigen::Matrix<double,13,13> P0 = Eigen::Matrix<double,13,13>::Zero();
    ///////////////////////////////////////////////////////////////////////////////////
    ///////////////// DO NOT DELETE DO NOT DELETE DO NOT DELETE ///////////////////////
    // I need to take into consideration the uncertainty of the initial quaternion when employing euler_id01 instead of euler_id00.
    // Trying to do things properly (commented) produced very bad results, with crashes after a few seconds.
    // The best approach was to use all four a=1e-10. Smaller does not matter but bigger numbers are much worse.
    //double a = _Psuite->get_sensor_euler().get_sigma();
    //ang::euler euler_sigma4(a, a, a);
    //ang::rodrigues rodrigues_sigma4(euler_sigma4);
    //double a0 = std::pow(1 - rodrigues_sigma4(0),2);
    //double a1 = std::pow(rodrigues_sigma4(1),2);
    //double a2 = std::pow(rodrigues_sigma4(2),2);
    //double a3 = std::pow(rodrigues_sigma4(3),2);
    // In these conditions the gyroscope bias takes around 180 seconds to converge (polluting the metrics
    // of the results), but after that it works very well. The magnetometer bias seems to be converging
    // much slower, but also going in the right direction. I need to do two things:
    // 1. Dramatically increase the -10 [sec] that I know assign for initialization.
    // 2. Modify the magnetometer biases so it moves quicker toward the solution.
    ///////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////
    double a = 0.;
    Eigen::Array3d b = E_gyr_std_init.array().pow(2) + std::pow(_Psuite->get_sens_gyr().get_std_white_noise(),2);
    Eigen::Array3d c = E_gyr_std_init.array().pow(2);
    Eigen::Array3d d = E_mag_std_init.array().pow(2);

    P0.diagonal() << a, a, a, a, b, c, d;

    // These are just the real initial conditions passed to evaluate the errors
    _Vx_truth.segment<4>(0)  = Ost_nav_in_init.get_q_nb();
    _Vx_truth.segment<3>(4)  = Ost_nav_in_init.get_w_nbb_rps();
    _Vx_truth.segment<3>(7)  = Ost_sens_out_init.get_E_gyr_rps();
    _Vx_truth.segment<3>(10) = Ost_sens_out_init.get_E_mag_nT();

    // initialize filter
    _Pekf_handler->initialize(Vx_init, P0, _Vx_truth);

    // fill up Ost_nav_init
    Ost_nav_out_init.get_t_sec()         = Ost_sens_out_init.get_t_sec();
    Ost_nav_out_init.get_q_nb()          = Vx_init.segment<4>(0);
    Ost_nav_out_init.get_w_nbb_rps()     = Vx_init.segment<3>(4);
    Ost_nav_out_init.get_E_gyr_rps()     = Vx_init.segment<3>(7);
    Ost_nav_out_init.get_E_mag_nT()      = Vx_init.segment<3>(10);
    Ost_nav_out_init.get_B_n_nT_dev()    = Eigen::Vector3d::Zero(); // not used in this filter
}
/* initialize filter filling up the initial navigation output state vector Ost_nav_out_init, based on the initial sensors output state vector Ost_sens_out_init,
 * the initial navigation input state vector Ost_nav_in_init (for filter evaluation purposes only), and other randomly computed variables required
 * for filter initialization, such as initial attitude, initial gyroscope error, initial magnetometer error, initial gravity vector, and
 * initial magnetic field). */

void nav::filter_att01::execute_step(st::st_nav_out& Ost_nav_out, const st::st_nav_out& Ost_nav_out_prev, const st::st_sens_out& Ost_sens_out, const st::st_nav_in& Ost_nav_in, const unsigned int& s) {
    // filter prediction phase
    _Pstate->update(_Pekf_handler->get_xhat_aft()[s-1]);
    _Pekf_handler->process_state(s);
    _Pekf_handler->get_xhat_bef()[s].segment<4>(0).normalize();

    // filter update phase
    _y.segment<3>(0) = Ost_sens_out.get_w_ibb_rps();
    _y.segment<3>(3) = Ost_sens_out.get_f_ibb_mps2();
    _y.segment<3>(6) = Ost_sens_out.get_B_b_nT();

    _Pobser->update(_Pekf_handler->get_xhat_bef()[s], Ost_nav_out_prev);
    _Pekf_handler->process_observation(s, _y);
    _Pekf_handler->get_xhat_aft()[s].segment<4>(0).normalize();

    // filter auxiliary phase
    _Vx_truth.segment<4>(0)  = Ost_nav_in.get_q_nb();
    _Vx_truth.segment<3>(4)  = Ost_nav_in.get_w_nbb_rps();
    _Vx_truth.segment<3>(7)  = Ost_sens_out.get_E_gyr_rps();
    _Vx_truth.segment<3>(10) = Ost_sens_out.get_E_mag_nT();
    _Pekf_handler->process_auxiliary(s, _Vx_truth);

    // pass filter results to Ost_nav
    Ost_nav_out.get_q_nb()          = _Pekf_handler->get_xhat_aft()[s].segment<4>(0);
    Ost_nav_out.get_w_nbb_rps()     = _Pekf_handler->get_xhat_aft()[s].segment<3>(4);
    Ost_nav_out.get_E_gyr_rps()     = _Pekf_handler->get_xhat_aft()[s].segment<3>(7);
    Ost_nav_out.get_E_mag_nT()      = _Pekf_handler->get_xhat_aft()[s].segment<3>(10);
    Ost_nav_out.get_B_n_nT_dev()    = Eigen::Vector3d::Zero(); // not used in this filter
}
/* execute filter step filling up the navigation output state vector Ost_nav_out, based on the previous navigation output state vector Ost_nav_out_prev,
 * the sensors output state vector Ost_sens_out, the navigation input state vector Ost_nav_in (for filter evaluation purposes only), and the current
 * navigation trajectory vector position. */

void nav::filter_att01::resize(const unsigned long& nel) {
    _Pekf_handler->resize(nel);
}
/* resize of all filter components to input size */

unsigned long nav::filter_att01::get_size() const {
    return _Pekf_handler->get_size();
}
/* returns size of all filter components */

void nav::filter_att01::switch_to_gps_lost_mode() {
}
/* switch to no gps mode */

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////







