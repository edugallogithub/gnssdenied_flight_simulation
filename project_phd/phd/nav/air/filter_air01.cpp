#include "filter_air01.h"
#include "ang/tools.h"
#include "env/geo.h"
#include "ang/rotate/rotv.h"
#include <fstream>
#include <iostream>

/*
 * State vector x components (siz_x == 6):
 * [0] --> tas_mps
 * [1] --> alpha_rad
 * [2] --> beta_rad
 * [3] --> T_degK
 * [4] --> Hp_m
 * [5] --> ROC_mps
 *
 * Observation vector y components (siz_y == 5):
 * [0] --> tas_mps
 * [1] --> alpha_rad
 * [2] --> beta_rad
 * [3] --> T_degK
 * [4] --> p_pa
 *
 * Observation noise vector v components (siz_v == 5):
 * [0] --> noise_tas_mps
 * [1] --> noise_alpha_rad
 * [2] --> noise_beta_rad
 * [3] --> noise_T_degK
 * [4] --> noise_p_pa
 */

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER AIR FUNCTOR STATE01
// ================================
// ================================

nav::filter_air_state01::filter_air_state01(const sens::suite& Osuite)
: _Qtilde(Eigen::Matrix<double,6,6>::Zero()) {
    // Decrease numbers for a behavior closer to low pass filter, placing less
    // weight of measurements and more on state equations (zero differential)
    double a = 1e-4  * 1.;
    double b = 1e-6  * 1.;
    double c = 1e-6  * 1.;
    double d = 1e-4  * 1.;
    double e = 1e-15 * 1.;
    double f = 1e-3  * 1.;
    _Qtilde.diagonal() << a, b, c, d, e, f;
}
/* constructor based on suite of sensors */

void nav::filter_air_state01::initialize() {
}
/* initialization */

void nav::filter_air_state01::update(const Eigen::Matrix<double,6,1>& xhat_aft_prev) {
    _vtas_mps  = xhat_aft_prev(0);
    _alpha_rad = xhat_aft_prev(1);
    _beta_rad  = xhat_aft_prev(2);
    _T_degK    = xhat_aft_prev(3);
    _Hp_m      = xhat_aft_prev(4);
    _ROC_mps   = xhat_aft_prev(5);
}
/* updates the functor based on previous EKF state estimation */

Eigen::Matrix<double,6,1> nav::filter_air_state01::eval(const Eigen::Matrix<double,6,1>& xhat_aft_prev) const {
    Eigen::Matrix<double,6, 1> res;
    res << 0., 0., 0., 0., _ROC_mps, 0.;
    return res;
}
/* evaluates functor based on previous EKF state estimation, returning vector of size z. */

void nav::filter_air_state01::jacobian_state(Eigen::Matrix<double,6,6>& A, const Eigen::Matrix<double,6,1>& xhat_aft_prev) const {
    A(4,5) = 1.;
}
/* evaluates jacobian of functor with respect to state vector x based on state vector. */

Eigen::Matrix<double,6,6> nav::filter_air_state01::covariance_matrix() const {
    return _Qtilde;
}
/* returns covariance matrix */

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER AIR FUNCTOR OBSER01
// ================================
// ================================

nav::filter_air_obser01::filter_air_obser01(const sens::suite& Osuite)
: _R(Eigen::Matrix<double,5,5>::Zero()) {
    double a = std::pow(Osuite.get_sens_tas().get_sigma(),2.0);
    double b = std::pow(Osuite.get_sens_aoa().get_sigma(),2.0);
    double c = std::pow(Osuite.get_sens_aos().get_sigma(),2.0);
    double d = std::pow(Osuite.get_sens_oat().get_sigma(),2.0);
    double e = std::pow(Osuite.get_sens_osp().get_sigma(),2.0);

    _R.diagonal() << a, b, c, d, e;
}
/* constructor based on suite of sensors */

void nav::filter_air_obser01::initialize() {
}
/* initialization */

void nav::filter_air_obser01::update(const Eigen::Matrix<double,6,1>& xhat_bef) {
    _vtas_mps  = xhat_bef(0);
    _alpha_rad = xhat_bef(1);
    _beta_rad  = xhat_bef(2);
    _T_degK    = xhat_bef(3);
    _Hp_m      = xhat_bef(4);
    _ROC_mps   = xhat_bef(5);

    _p_pa = env::atm::Hp2p(_Hp_m);
}
/* updates the functor based on current EKF state estimation */

Eigen::Matrix<double,5,1> nav::filter_air_obser01::eval(const Eigen::Matrix<double,6,1>& xhat_bef) const {
    Eigen::Matrix<double,5,1> res;
    res << _vtas_mps, _alpha_rad, _beta_rad, _T_degK, _p_pa;
    return res;
}
/* evaluates functor based on current EKF state estimation, returning vector of size z. */

void nav::filter_air_obser01::jacobian_state(Eigen::Matrix<double,5,6>& H, const Eigen::Matrix<double,6,1>& xhat_bef) const {
    H(0,0) = 1.;
    H(1,1) = 1.;
    H(2,2) = 1.;
    H(3,3) = 1.;
    H(4,4) = env::atm::obtain_dp_dHp(_p_pa, _Hp_m);
}
/* evaluates jacobian of functor with respect to state vector x based on state vector. */

void nav::filter_air_obser01::jacobian_noise(Eigen::Matrix<double,5,5>& M, const Eigen::Matrix<double,6,1>& xhat_bef) const {
    M.diagonal() = Eigen::Matrix<double,5,1>::Ones();;
}
/* evaluates jacobian of functor with respect to noise vector (w or v) based on state vector. */

Eigen::Matrix<double,5,5> nav::filter_air_obser01::covariance_matrix() const {
    return _R;
}
/* returns covariance matrix */

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER AIR01
// ==================
// ==================

nav::filter_air01::filter_air01(const sens::suite& Osuite)
: filter_air(Osuite),  _Pekf_handler(nullptr), _Pstate(new nav::filter_air_state01(Osuite)), _Pobser(new nav::filter_air_obser01(Osuite)) {
}
/* constructor based on suite of sensors and Earth model */

nav::filter_air01::~filter_air01() {
    delete _Pekf_handler;
    delete _Pstate;
    delete _Pobser;
}
/* destructor */

void nav::filter_air01::finalize_constructor(const double& Deltat_sec_nav, const unsigned int& nel_nav) {
    _Pekf_handler = new nav::ekf_handler_<6,5,6,5>(nel_nav, Deltat_sec_nav, *_Pstate, *_Pobser);
}
/* complete constructor with navigation time and size of navigation vector, which are not available at construction time */

void nav::filter_air01::initialize(st::st_nav_out& Ost_nav_out_init, const st::st_sens_out& Ost_sens_out_init, const st::st_nav_in& Ost_nav_in_init) {
    // functor initialization
    _Pstate->initialize();
    _Pobser->initialize();

    // Initial state estimate shall be the mean, but there is no way to compute that.
    // I use the initial sensed values when possible, zero for the biases, and zero for the unknown variables.
    Eigen::Matrix<double,6,1> Vx_init;
    Vx_init(0) = Ost_sens_out_init.get_vtas_mps();
    Vx_init(1) = Ost_sens_out_init.get_euler_wb().get_pitch_rad();
    Vx_init(2) = - Ost_sens_out_init.get_euler_wb().get_yaw_rad();
    Vx_init(3) = Ost_sens_out_init.get_T_degK();
    Vx_init(4) = env::atm::p2Hp(Ost_sens_out_init.get_p_pa());
    Vx_init(5) = 0.; // unknown

    // Initial covariance of the state estimate (need to transform sensor pressure to state vector pressure altitude)
    Eigen::Matrix<double,6,6> P0 = Eigen::Matrix<double,6,6>::Zero();
    P0(0,0) = std::pow(_Psuite->get_sens_tas().get_sigma(),2.0);
    P0(1,1) = std::pow(_Psuite->get_sens_aoa().get_sigma(),2.0) + std::pow(_Psuite->get_sens_aoa().get_B0(),2.0);
    P0(2,2) = std::pow(_Psuite->get_sens_aos().get_sigma(),2.0) + std::pow(_Psuite->get_sens_aos().get_B0(),2.0);
    P0(3,3) = std::pow(_Psuite->get_sens_oat().get_sigma(),2.0);
    double sigma_Hp_m = env::atm::p2Hp(Ost_sens_out_init.get_p_pa() + _Psuite->get_sens_osp().get_sigma()) - env::atm::p2Hp(Ost_sens_out_init.get_p_pa());
    P0(4,4) = std::pow(sigma_Hp_m, 2.0);
    P0(5,5) = 0.0; // it is zero in the initial conditions but in theory it does not have to

    // These are just the real initial conditions passed to evaluate the errors
    _Vx_truth(0) = Ost_nav_in_init.get_vtas_mps();
    _Vx_truth(1) = Ost_nav_in_init.get_euler_wb().get_pitch_rad();
    _Vx_truth(2) = - Ost_nav_in_init.get_euler_wb().get_yaw_rad();
    _Vx_truth(3) = Ost_nav_in_init.get_T_degK();
    _Vx_truth(4) = Ost_nav_in_init.get_Hp_m();
    _Vx_truth(5) = 0.;

    // initialize filter
    _Pekf_handler->initialize(Vx_init, P0, _Vx_truth);

    // fill up Ost_nav_init
    Ost_nav_out_init.get_vtas_mps()     = Vx_init(0);
    Ost_nav_out_init.get_euler_wb()     = ang::euler(- Vx_init(2), Vx_init(1), 0.);
    Ost_nav_out_init.get_T_degK()       = Vx_init(3);
    Ost_nav_out_init.get_Hp_m()         = Vx_init(4);
    Ost_nav_out_init.get_roc_mps()      = Vx_init(5);
    Ost_nav_out_init.get_DeltaT_degK()  = env::atm::obtain_DeltaT_degK(Ost_nav_out_init.get_Hp_m(), Ost_nav_out_init.get_T_degK());
}
/* initialize filter filling up the initial navigation output state vector Ost_nav_out_init, based on the initial sensors output state vector Ost_sens_out_init,
* and the initial navigation input state vector Ost_nav_in_init (for filter evaluation purposes only). */

void nav::filter_air01::execute_step(st::st_nav_out& Ost_nav_out, const st::st_sens_out& Ost_sens_out, const st::st_nav_in& Ost_nav_in, const unsigned int& s) {
    // filter prediction phase
    _Pstate->update(_Pekf_handler->get_xhat_aft()[s-1]);
    _Pekf_handler->process_state(s);

    // filter update phase
    _y(0) = Ost_sens_out.get_vtas_mps();
    _y(1) = Ost_sens_out.get_euler_wb().get_pitch_rad();
    _y(2) = - Ost_sens_out.get_euler_wb().get_yaw_rad();
    _y(3) = Ost_sens_out.get_T_degK();
    _y(4) = Ost_sens_out.get_p_pa();
    _Pobser->update(_Pekf_handler->get_xhat_bef()[s]);
    _Pekf_handler->process_observation(s, _y);

    // filter auxiliary phase
    _Vx_truth(0) = Ost_nav_in.get_vtas_mps();
    _Vx_truth(1) = Ost_nav_in.get_euler_wb().get_pitch_rad();
    _Vx_truth(2) = - Ost_nav_in.get_euler_wb().get_yaw_rad();
    _Vx_truth(3) = Ost_nav_in.get_T_degK();
    _Vx_truth(4) = Ost_nav_in.get_Hp_m();
    _Vx_truth(5) = - Ost_nav_in.get_v_n_mps()(2);
    _Pekf_handler->process_auxiliary(s, _Vx_truth);

    // pass filter results to Ost_nav
    Ost_nav_out.get_vtas_mps()     = _Pekf_handler->get_xhat_aft()[s](0);
    Ost_nav_out.get_euler_wb()     = ang::euler(- _Pekf_handler->get_xhat_aft()[s](2), _Pekf_handler->get_xhat_aft()[s](1), 0.);
    Ost_nav_out.get_T_degK()       = _Pekf_handler->get_xhat_aft()[s](3);
    Ost_nav_out.get_Hp_m()         = _Pekf_handler->get_xhat_aft()[s](4);
    Ost_nav_out.get_roc_mps()      = _Pekf_handler->get_xhat_aft()[s](5);
    Ost_nav_out.get_DeltaT_degK()  = env::atm::obtain_DeltaT_degK(Ost_nav_out.get_Hp_m(), Ost_nav_out.get_T_degK());
}
/* execute filter step filling up the navigation output state vector Ost_nav_out, based on the sensors output state vector Ost_sens_out, the navigation input
 * state vector Ost_nav_in (for filter evaluation purposes only), and the current navigation trajectory vector position. */

void nav::filter_air01::resize(const unsigned long& nel) {
    _Pekf_handler->resize(nel);
}
/* resize of all filter components to input size */

unsigned long nav::filter_air01::get_size() const {
    return _Pekf_handler->get_size();
}
/* returns size of all filter components */

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////







