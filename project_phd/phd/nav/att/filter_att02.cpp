#include "filter_att02.h"
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

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER ATTITUDE FUNCTOR STATE02
// =====================================
// =====================================

nav::filter_att_state02::filter_att_state02(const sens::suite& Osuite)
: _Qtilde(Eigen::Matrix<double,16,16>::Zero()) {
    // 1st equation (q_nb)) is fully exact, so covariance as low as possible
    // 2nd equation (w_nbb) is fully unknown. Covariance high.
    // 3rd equation (E_gyr) is fully unknown. Covariance is bias drift plus effect of scale & cross coupling.
    // 4th equation (E_mag) is fully unknown. Covariance is effect of scale & cross coupling.
    // 5th equation (B_n_dev) is fully unknown, and covariance is as low as possible (in theory zero)
    // if deviation is constant (as is the case right now) or higher if at some point I do change that.
    double a = 5e-15;
    double b = 1e-6;
    double d = 1e-14;
    double e = 1e-12;

    // CONFIGURATION #1:
    //double c = std::pow(Osuite.get_sens_gyr().get_std_bias_drift(),2); // 6.14e-14
    // CONFIGURATION #2:
    //double c = 3e-14;
    // CONFIGURATION #3:
    //double c = std::pow(Osuite.get_sens_gyr().get_std_bias_drift(),2); // 6.14e-14
    // CONFIGURATION #4:
    double c = 3e-14;

    _Qtilde.diagonal() << a, a, a, a, b, b, b, c, c, c, d, d, d, e, e, e;
}
/* constructor based on suite of sensors */

void nav::filter_att_state02::initialize() {
}
/* initialization */

void nav::filter_att_state02::update(const Eigen::Matrix<double,16,1>& xhat_aft_prev) {
    _q_nb       << xhat_aft_prev.segment<4>(0);
    _w_nbb_rps  << xhat_aft_prev.segment<3>(4);
    _E_gyr_rps  << xhat_aft_prev.segment<3>(7);
    _E_mag_nT   << xhat_aft_prev.segment<3>(10);
    _B_n_nT_dev << xhat_aft_prev.segment<3>(13);
}
/* updates the functor based on previous EKF state estimation and previous sensors outputs */

Eigen::Matrix<double,16,1> nav::filter_att_state02::eval(const Eigen::Matrix<double,16,1>& xhat_aft_prev) const {
    Eigen::Matrix<double,16, 1> res;
    res.segment<4>(0)  = _q_nb.omegabody2dot(_w_nbb_rps);
    res.segment<3>(4)  = Eigen::Vector3d::Zero();
    res.segment<3>(7)  = Eigen::Vector3d::Zero();
    res.segment<3>(10) = Eigen::Vector3d::Zero();
    res.segment<3>(13) = Eigen::Vector3d::Zero();
    return res;
}
/* evaluates functor based on previous EKF state estimation, returning vector of size z. */

void nav::filter_att_state02::jacobian_state(Eigen::Matrix<double,16,16>& A, const Eigen::Matrix<double,16,1>& xhat_aft_prev) const {
    A.block<4,4>(0,0) = ang::tools::right_skew43(_w_nbb_rps) * 0.5;
    A.block<4,3>(0,4) = ang::tools::skew43(_q_nb) * 0.5;
}
/* evaluates jacobian of functor with respect to state vector x based on state vector. */

Eigen::Matrix<double,16,16> nav::filter_att_state02::covariance_matrix() const {
    return _Qtilde;
}
/* returns covariance matrix */

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER ATTITUDE FUNCTOR OBSER02
// =====================================
// =====================================

nav::filter_att_obser02::filter_att_obser02(const sens::suite& Osuite, const env::geo& Ogeo)
: _Pgeo(&Ogeo), _flag_gps(true), _R(Eigen::Matrix<double,9,9>::Zero()),
  _Plpf_Eacc_mps2(nullptr), _Plpf_v_n_mps(nullptr), _Plpf_vlf_n_mps(nullptr) {
    // 1st equation (w_ibb) is exact with conditions:
    //     - as long as w_ien and w_enn are exact, but they depend on position and velocity provided by position filter,
    //       so they are not. And without GPS far less. This adds to covariance.
    //     - covariance should be at least gyroscope noise
    // 2nd equation (f_ibb) is not very precise, as it neglects differentials of air speed, wind field, and turbulence
    //     evaluated and viewed in body frame. Covariance should be accelerometer noise plus term for turbulence
    //     plus term for airspeed body acceleration. Another source of error is Eacc, which comes from position filter
    //     but in general may be way off, specially without GPS. Another source of error is also position and velocity
    //     from previous step in position filter.
    // 3rd equation (B_n) is exact, and covariance is magnetometer white noise.

    // CONFIGURATION #1:
    //double a = std::pow(Osuite.get_sens_gyr().get_std_white_noise(),2.0); // 5.62e-7
    //double b = std::pow(Osuite.get_sens_acc().get_std_white_noise(),2.0); // 2.33e-5
    //double c = 10 * std::pow(Osuite.get_sens_mag().get_std_white_noise(),2.0); // 2500 (without multiplier), 25000 (with multiplier)

    //double var_turb_bfs_diff_m2ps4_x  = 0.372;
    //double var_turb_bfs_diff_m2ps4_yz = 0.563;

    // CONFIGURATION #2:
    //double a = 5.6e-6;
    //double b = std::pow(Osuite.get_sens_acc().get_std_white_noise(),2.0); // 2.33e-5
    //double c = 100000;

    //double var_turb_bfs_diff_m2ps4_x  = 0.372 + 0.3;
    //double var_turb_bfs_diff_m2ps4_yz = 0.563 + 0.3;

    // CONFIGURATION #1:
    //double a = std::pow(Osuite.get_sens_gyr().get_std_white_noise(),2.0); // 5.62e-7
    //double b = std::pow(Osuite.get_sens_acc().get_std_white_noise(),2.0); // 2.33e-5
    //double c = 100000;

    //double var_turb_bfs_diff_m2ps4_x  = 0.372;
    //double var_turb_bfs_diff_m2ps4_yz = 0.563;

    // CONFIGURATION #4:
    double a = 5.6e-6;
    double b = std::pow(Osuite.get_sens_acc().get_std_white_noise(),2.0); // 2.33e-5
    double c = 200000;

    double var_turb_bfs_diff_m2ps4_x  = 0.372 + 0.3;
    double var_turb_bfs_diff_m2ps4_yz = 0.563 + 0.3;
    //double b1 = 0.672;
    //double b2 = 0.863;

    _R.block<3,3>(0,0).diagonal() << a, a, a;
    _R.block<3,3>(3,3).diagonal() << b + var_turb_bfs_diff_m2ps4_x, b + var_turb_bfs_diff_m2ps4_yz, b + var_turb_bfs_diff_m2ps4_yz;
    _R.block<3,3>(6,6).diagonal() << c, c, c;

    // The following numbers are obtained from test_turb_obtain_differential() in earth_test::Tenvironment
    // and correspond to the approximate covariance (with zero mean) introduced by the turbulence. It does
    // not include the acceleration of the low frequency wind (very small) nor the tas acceleration (bigger).
    // The standard deviation is approximately constant at ~0.61 [mps2] for vdot_turb_bfs_1 and ~0.75 [mps2] for
    // vdot_turb_bfs_2 and vdot_turb_bfs_3. The variance caused by not including the wind body differential
    // with time in the accelerometer equation is hence 0.372 [m2ps4] for the x axis and 0.563 [m2ps4] for
    // the 2nd and 3rd axes. Note that this is for the wind, the tas acceleration not included.
}
/* constructor based on suite of sensors and Earth model */

nav::filter_att_obser02::~filter_att_obser02() {
    delete _Plpf_Eacc_mps2;
    delete _Plpf_v_n_mps;
    delete _Plpf_vlf_n_mps;
}
/* destructor */

void nav::filter_att_obser02::finalize_constructor(const double& Deltat_sec_nav) {
    _Plpf_Eacc_mps2 = new math::low_pass_triple(300.0 * Deltat_sec_nav, Deltat_sec_nav);
    _Plpf_v_n_mps   = new math::low_pass_triple(1.0 * Deltat_sec_nav, Deltat_sec_nav);
    _Plpf_vlf_n_mps = new math::low_pass_triple(1.0 * Deltat_sec_nav, Deltat_sec_nav);
}
/* complete constructor with navigation time, which is not available at construction time */

void nav::filter_att_obser02::initialize() {
    // initialize with zero but does not matter as it will get close and close to real value with more evaluations
    _Plpf_Eacc_mps2->init(Eigen::Vector3d::Zero());
}
/* initialization */

void nav::filter_att_obser02::switch_to_gps_lost_mode() {
    _flag_gps = false;
    _Plpf_v_n_mps->init(_v_n_mps);
    _Plpf_vlf_n_mps->init(_vlf_n_mps);
}
/* switch to no gps mode */

void nav::filter_att_obser02::update(const Eigen::Matrix<double,16,1>& xhat_bef, const st::st_nav_out& Ost_nav_out_prev) {
    _q_nb         << xhat_bef.segment<4>(0);
    _w_nbb_rps    << xhat_bef.segment<3>(4);
    _E_gyr_rps    << xhat_bef.segment<3>(7);
    _E_mag_nT     << xhat_bef.segment<3>(10);
    _B_n_nT_dev << xhat_bef.segment<3>(13);

    // trust ground speed from GPS filter and update (but do not use) smoothed variable
    // trust accelerometer error from GPS filter and update (but do not use) smoothed variable
    // smooth ground speed from POS filter
    // do not trust accelerometer error from POS filter, so slowly freeze it to smoothed variable

    _x_gdt_rad_m       = Ost_nav_out_prev.get_x_gdt_rad_m();
    _v_n_mps           = Ost_nav_out_prev.get_v_n_mps();
    _vlf_n_mps         = Ost_nav_out_prev.get_vlf_n_mps();
    _vtashf_n_mps      = _v_n_mps - _vlf_n_mps;
    _E_acc_mps2        = Ost_nav_out_prev.get_E_acc_mps2();
    _v_n_mps_smooth    = _v_n_mps;
    _E_acc_mps2_smooth = _Plpf_Eacc_mps2->eval(_E_acc_mps2);

    _w_ien_rps         = _Pgeo->compute_wien_rps(_x_gdt_rad_m.get_phi_rad());
    _N_m               = _Pgeo->radius_vert(_x_gdt_rad_m.get_phi_rad());
    _M_m               = _Pgeo->radius_mer(_x_gdt_rad_m.get_phi_rad(), _N_m);
    _w_enn_rps         = _Pgeo->compute_wenn_rps(_v_n_mps, _N_m, _M_m, _x_gdt_rad_m.get_phi_rad(), _x_gdt_rad_m.get_h_m());
    _a_cor_n_mps2      = _Pgeo->compute_coriolis_n(_v_n_mps, _w_ien_rps);
    _gc_n_mps2_model   = _Pgeo->compute_gravity_n_model(_x_gdt_rad_m);
    _B_n_nT_model      = _Pgeo->get_mag().compute_B_n_nT_model(Ost_nav_out_prev.get_t_sec(), _x_gdt_rad_m);
}
/* updates the functor based on current EKF state estimation, current sensors outputs, and XXXXXXXXX */

Eigen::Matrix<double,9,1> nav::filter_att_obser02::eval(const Eigen::Matrix<double,16,1>& xhat_bef) const {
    Eigen::Matrix<double,9,1> res = Eigen::Matrix<double,9,1>::Zero();

    res.segment<3>(0) = _w_nbb_rps + _q_nb / (_w_enn_rps + _w_ien_rps) + _E_gyr_rps;
    res.segment<3>(3) = _w_nbb_rps.cross(_q_nb / _vtashf_n_mps) + _q_nb / (_w_enn_rps.cross(_v_n_mps) - _gc_n_mps2_model + _a_cor_n_mps2) + _E_acc_mps2;
    res.segment<3>(6) = _q_nb / (_B_n_nT_model - _B_n_nT_dev) + _E_mag_nT;

    return res;
}
/* evaluates functor based on current EKF state estimation, returning vector of size z. */

void nav::filter_att_obser02::jacobian_state(Eigen::Matrix<double,9,16>& H, const Eigen::Matrix<double,16,1>& xhat) const {
    H.block<3,4>(0,0)  = _q_nb.jacobian_quat_backward_rotation(_w_enn_rps + _w_ien_rps);
    H.block<3,3>(0,4)  = Eigen::Matrix3d::Identity();
    H.block<3,3>(0,7)  = Eigen::Matrix3d::Identity();
    H.block<3,4>(3,0)  = _q_nb.jacobian_quat_backward_rotation(_w_enn_rps.cross(_v_n_mps) - _gc_n_mps2_model + _a_cor_n_mps2)
                         + ang::tools::skew3(_w_nbb_rps) * _q_nb.jacobian_quat_backward_rotation(_vtashf_n_mps);
    H.block<3,3>(3,4)  = ang::tools::right_skew3(_q_nb / _vtashf_n_mps);
    H.block<3,4>(6,0)  = _q_nb.jacobian_quat_backward_rotation(_B_n_nT_model - _B_n_nT_dev);
    H.block<3,3>(6,10) = Eigen::Matrix3d::Identity();
    H.block<3,3>(6,13) = - _q_nb.jacobian_vector_backward_rotation();
}
/* evaluates jacobian of functor with respect to state vector x based on state vector. */

void nav::filter_att_obser02::jacobian_noise(Eigen::Matrix<double,9,9>& M, const Eigen::Matrix<double,16,1>& xhat) const {
    M.diagonal() = Eigen::Matrix<double,9,1>::Ones();;
}
/* evaluates jacobian of functor with respect to noise vector (w or v) based on state vector. */

Eigen::Matrix<double,9,9> nav::filter_att_obser02::covariance_matrix() const {
    return _R;
}
/* returns covariance matrix */

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER ATTITUDE02
// =======================
// =======================

nav::filter_att02::filter_att02(const sens::suite& Osuite, const env::geo& Ogeo)
: filter_att(Osuite), _Pekf_handler(nullptr), _Pstate(new nav::filter_att_state02(Osuite)), _Pobser(new nav::filter_att_obser02(Osuite, Ogeo)) {
}
/* constructor based on suite of sensors and Earth model */

nav::filter_att02::~filter_att02() {
    delete _Pekf_handler;
    delete _Pstate;
    delete _Pobser;
}
/* destructor */

void nav::filter_att02::finalize_constructor(const double& Deltat_sec_nav, const unsigned int& nel_nav) {
    _Pekf_handler = new nav::ekf_handler_<16,9,16,9>(nel_nav, Deltat_sec_nav, *_Pstate, *_Pobser);
    _Pobser->finalize_constructor(Deltat_sec_nav);
}
/* complete constructor with navigation time and size of navigation vector, which are not available at construction time */

void nav::filter_att02::initialize(st::st_nav_out& Ost_nav_out_init, const st::st_sens_out& Ost_sens_out_init, const st::st_nav_in& Ost_nav_in_init,
                                   const ang::rodrigues& q_nb_init,
                                   const Eigen::Vector3d& E_gyr_init, const Eigen::Vector3d& E_gyr_std_init,
                                   const Eigen::Vector3d& E_mag_init, const Eigen::Vector3d& E_mag_std_init,
                                   const Eigen::Vector3d& E_B_n_init, const Eigen::Vector3d& E_B_n_std_init) {
    // functor initialization
    _Pstate->initialize();
    _Pobser->initialize();

    // Initial state estimate shall be the mean, but there is no way to compute that.
    // I use the initial sensed values when possible, zero for the biases, and zero for the unknown variables.
    Eigen::Matrix<double,16,1> Vx_init;
    Vx_init.segment<4>(0)  = q_nb_init;
    Vx_init.segment<3>(4)  = Ost_sens_out_init.get_w_ibb_rps() - E_gyr_init; // note ibb, not nbb ///////////////// MOST LIKELY I CAN DO BETTER ESTIMATING IEN Y ENB
    Vx_init.segment<3>(7)  = E_gyr_init;
    Vx_init.segment<3>(10) = E_mag_init;
    Vx_init.segment<3>(13) = E_B_n_init;

    // Initial covariance of the state estimate
    Eigen::Matrix<double,16,16> P0 = Eigen::Matrix<double,16,16>::Zero();
    ///////////////////////////////////////////////////////////////////////////////////
    ///////////////// DO NOT DELETE DO NOT DELETE DO NOT DELETE ///////////////////////
    // I need to take into consideration the uncertainty of the initial quaternion when employing euler_id02 instead of euler_id00.
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
    double a = 1e-8;
    Eigen::Array3d b = E_gyr_std_init.array().pow(2) + std::pow(_Psuite->get_sens_gyr().get_std_white_noise(),2);
    Eigen::Array3d c = E_gyr_std_init.array().pow(2);
    Eigen::Array3d d = E_mag_std_init.array().pow(2);
    Eigen::Array3d e = E_B_n_std_init.array().pow(2);

    P0.diagonal() << a, a, a, a, b, c, d, e;

    // These are just the real initial conditions passed to evaluate the errors
    _Vx_truth.segment<4>(0)  = Ost_nav_in_init.get_q_nb();
    _Vx_truth.segment<3>(4)  = Ost_nav_in_init.get_w_nbb_rps();
    _Vx_truth.segment<3>(7)  = Ost_sens_out_init.get_E_gyr_rps();
    _Vx_truth.segment<3>(10) = Ost_sens_out_init.get_E_mag_nT();
    _Vx_truth.segment<3>(13) = Ost_nav_in_init.get_B_n_nT_dev();

    // initialize filter
    _Pekf_handler->initialize(Vx_init, P0, _Vx_truth);

    // fill up Ost_nav_init
    Ost_nav_out_init.get_t_sec()         = Ost_sens_out_init.get_t_sec();
    Ost_nav_out_init.get_q_nb()          = Vx_init.segment<4>(0);
    Ost_nav_out_init.get_w_nbb_rps()     = Vx_init.segment<3>(4);
    Ost_nav_out_init.get_E_gyr_rps()     = Vx_init.segment<3>(7);
    Ost_nav_out_init.get_E_mag_nT()      = Vx_init.segment<3>(10);
    Ost_nav_out_init.get_B_n_nT_dev()  = Vx_init.segment<3>(13);
}
/* initialize filter filling up the initial navigation output state vector Ost_nav_out_init, based on the initial sensors output state vector Ost_sens_out_init,
 * the initial navigation input state vector Ost_nav_in_init (for filter evaluation purposes only), and other randomly computed variables required
 * for filter initialization, such as initial attitude, initial gyroscope error, initial magnetometer error, initial gravity vector, and
 * initial magnetic field). */

void nav::filter_att02::execute_step(st::st_nav_out& Ost_nav_out, const st::st_nav_out& Ost_nav_out_prev, const st::st_sens_out& Ost_sens_out, const st::st_nav_in& Ost_nav_in, const unsigned int& s) {
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
    _Vx_truth.segment<3>(13) = Ost_nav_in.get_B_n_nT_dev();
    _Pekf_handler->process_auxiliary(s, _Vx_truth);

    // pass filter results to Ost_nav
    Ost_nav_out.get_q_nb()        = _Pekf_handler->get_xhat_aft()[s].segment<4>(0);
    Ost_nav_out.get_w_nbb_rps()   = _Pekf_handler->get_xhat_aft()[s].segment<3>(4);
    Ost_nav_out.get_E_gyr_rps()   = _Pekf_handler->get_xhat_aft()[s].segment<3>(7);
    Ost_nav_out.get_E_mag_nT()    = _Pekf_handler->get_xhat_aft()[s].segment<3>(10);
    Ost_nav_out.get_B_n_nT_dev()  = _Pekf_handler->get_xhat_aft()[s].segment<3>(13);
}
/* execute filter step filling up the navigation output state vector Ost_nav_out, based on the previous navigation output state vector Ost_nav_out_prev,
 * the sensors output state vector Ost_sens_out, the navigation input state vector Ost_nav_in (for filter evaluation purposes only), and the current
 * navigation trajectory vector position. */

void nav::filter_att02::resize(const unsigned long& nel) {
    _Pekf_handler->resize(nel);
}
/* resize of all filter components to input size */

unsigned long nav::filter_att02::get_size() const {
    return _Pekf_handler->get_size();
}
/* returns size of all filter components */

void nav::filter_att02::switch_to_gps_lost_mode() {
    _Pobser->switch_to_gps_lost_mode();
}
/* switch to no gps mode */

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////







