#include "filter_gps01.h"
#include "ang/tools.h"
#include "env/geo.h"
#include "ang/rotate/dcm.h"
#include "ang/rotate/rotv.h"
#include "env/speed.h"
#include "acft/st/trj_sens_in.h"
#include "acft/st/trj_nav_out.h"
#include "acft/st/trj_truth.h"
#include "acft/st/sti.h"
#include <fstream>
#include <iostream>
#include <cmath>

/*
 * State vector x components (siz_x == 12):
 * [0-2]  --> x_gdt_rad_m
 * [3-5]  --> v_ned_mps
 * [6-8]  --> f_ibb_mps2
 * [9-11] --> E_acc_mps2
  *
 * Observation vector y components (siz_y == 9):
 * [0-2] --> ACC_f_ibb_mps2
 * [3-5] --> GPS_v_ned_mps
 * [6-8] --> GPS_x_ned_m
 *
 * State noise vector w components (siz_w == 12):
 * - It is a dummy as all equations are perfect but I need some noise
 *
 * Observation noise vector v components (siz_v == 9):
 * [0-2] --> noise_acc_mps2
 * [3-5] --> noise_gps_vel_mps
 * [6-8] --> noise_gps_pos_m
 */

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER GPS FUNCTOR STATE01
// ================================
// ================================

nav::filter_gps_state01::filter_gps_state01(const sens::suite& Osuite, const env::geo& Ogeo)
: _Qtilde(Eigen::Matrix<double,12,12>::Zero()), _Pgeo(&Ogeo) {
    double axy = 1e-6;
    double ah  = 1e-6;
    double b = 1e-5;
    double c = 1e-4;
    double d = 1e-3 * std::pow(Osuite.get_sens_acc().get_std_bias_drift(),2);
    //////////////////////////////////////////////////////////////////////////////////
    // IMPORTANT NOTE: The 1e-3 factor in d is very important. Otherwise, Eacc slowly
    // drifts with time, and it can go anywhere. Most likely (refer to tests 33 and 34) doing
    // turns stabilizes it. The issue is that the error in Eacc is the opposite of the
    // error in f_ibb. This way it seems to work, but otherwise it looks like only
    // possibility is to do a lateral turn every 300 sec or so
    //////////////////////////////////////////////////////////////////////////////////

    _Qtilde.diagonal() << axy, axy, ah, b, b, b, c, c, c, d, d, d;
}
/* constructor based on suite of sensors */

void nav::filter_gps_state01::update(const Eigen::Matrix<double,12,1>& xhat_aft_prev, const st::st_nav_out& Ost_nav_out) {
    _x_gdt_rad_m = xhat_aft_prev.segment<3>(0);
    _v_ned_mps   << xhat_aft_prev.segment<3>(3);
    _f_ibb_mps2  << xhat_aft_prev.segment<3>(6);
    _E_acc_mps2  << xhat_aft_prev.segment<3>(9);

    _q_nb         = Ost_nav_out.get_q_nb();
    _w_ien_rps    = _Pgeo->compute_wien_rps(_x_gdt_rad_m.get_phi_rad());
    _N_m          = _Pgeo->radius_vert(_x_gdt_rad_m.get_phi_rad());
    _M_m          = _Pgeo->radius_mer(_x_gdt_rad_m.get_phi_rad(), _N_m);
    _w_enn_rps    = _Pgeo->compute_wenn_rps(_v_ned_mps, _N_m, _M_m, _x_gdt_rad_m.get_phi_rad(), _x_gdt_rad_m.get_h_m());
    _a_cor_n_mps2 = _Pgeo->compute_coriolis_n(_v_ned_mps, _w_ien_rps);
    _gc_n_mps2    = _Pgeo->compute_gravity_n_model(_x_gdt_rad_m);
}
/* updates the functor based on previous EKF state estimation and previous sensors outputs */

Eigen::Matrix<double,12,1> nav::filter_gps_state01::eval(const Eigen::Matrix<double,12,1>& xhat_aft_prev) const {
    Eigen::Matrix<double,12, 1> res;
    res.segment<3>(0) = env::geo::vned_to_xgdtdot(_v_ned_mps, _x_gdt_rad_m, _N_m, _M_m);
    res.segment<3>(3) = _q_nb * _f_ibb_mps2 + _gc_n_mps2 - _w_enn_rps.cross(_v_ned_mps) - _a_cor_n_mps2;
    res.segment<3>(6) = Eigen::Vector3d::Zero();
    res.segment<3>(9) = Eigen::Vector3d::Zero();
    return res;
}
/* evaluates functor based on previous EKF state estimation, returning vector of size z. */

void nav::filter_gps_state01::jacobian_state(Eigen::Matrix<double,12,12>& A, const Eigen::Matrix<double,12,1>& xhat_aft_prev) const {
    A(0,4) = 1. / ((_N_m + _x_gdt_rad_m.get_h_m()) * cos(_x_gdt_rad_m.get_phi_rad()));
    A(1,3) = 1. / (_M_m + _x_gdt_rad_m.get_h_m());
    A(2,5) = -1.;
    A.block<3,3>(3,3) = - ang::tools::skew3(_w_enn_rps);
    A.block<3,3>(3,6) = _q_nb.jacobian_vector_forward_rotation();
}
/* evaluates jacobian of functor with respect to state vector x based on state vector. */

Eigen::Matrix<double,12,12> nav::filter_gps_state01::covariance_matrix() const {
    return _Qtilde;
}
/* returns covariance matrix */

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER GPS FUNCTOR OBSER01
// ================================
// ================================

nav::filter_gps_obser01::filter_gps_obser01(const sens::suite& Osuite, const env::geo& Ogeo)
: _R_fast(Eigen::Matrix<double,3,3>::Zero()), _R_slow(Eigen::Matrix<double,9,9>::Zero()), _Psuite(&Osuite), _Pgeo(&Ogeo) {
}
/* constructor based on suite of sensors and Earth model */

void nav::filter_gps_obser01::initialize(const env::geodetic_coord& x_gdt_rad_m_init) {
    double a   = std::pow(_Psuite->get_sens_acc().get_std_white_noise(),2.0);
    double b   = std::pow(_Psuite->get_sens_gps().get_sigma_vel_mps(),2.0);

    double sigma_hor_m = _Psuite->get_sens_gps().get_sigma_pos_hor_m(); // OJO OJOiono effect missing //////////////////////////////////////////////////
    double sigma_ver_m = _Psuite->get_sens_gps().get_sigma_pos_ver_m(); //OJOJ OJO  iono effect missing //////////////////////////////////////////////////

    double N_m = _Pgeo->radius_vert(x_gdt_rad_m_init.get_phi_rad());
    double M_m = _Pgeo->radius_mer(x_gdt_rad_m_init.get_phi_rad(), N_m);
    double sigma_lambda_rad = sigma_hor_m / ((N_m + x_gdt_rad_m_init.get_h_m()) * cos(x_gdt_rad_m_init.get_phi_rad()));
    double sigma_phi_rad    = sigma_hor_m / (M_m + x_gdt_rad_m_init.get_h_m());

    _R_fast.diagonal() << a, a, a;
    _R_slow.diagonal() << a, a, a, b, b, b, std::pow(sigma_lambda_rad,2.0), std::pow(sigma_phi_rad,2.0), std::pow(sigma_ver_m,2.0);
}
/* initialization */

void nav::filter_gps_obser01::update(const Eigen::Matrix<double,12,1>& xhat_bef) {
    _x_gdt_rad_m =  xhat_bef.segment<3>(0);
    _v_ned_mps   << xhat_bef.segment<3>(3);
    _f_ibb_mps2  << xhat_bef.segment<3>(6);
    _E_acc_mps2  << xhat_bef.segment<3>(9);
}
/* updates the functor based on current EKF state estimation, current sensors outputs, and XXXXXXXXX */

Eigen::Matrix<double,3,1> nav::filter_gps_obser01::eval_fast(const Eigen::Matrix<double,12,1>& xhat_bef) const {
    Eigen::Matrix<double,3,1> res = _f_ibb_mps2 + _E_acc_mps2;
    return res;
}
/* evaluates functor (fast) based on current EKF state estimation, returning vector of size z. */

Eigen::Matrix<double,9,1> nav::filter_gps_obser01::eval_slow(const Eigen::Matrix<double,12,1>& xhat_bef) const {
    Eigen::Matrix<double,9,1> res;
    res.segment<3>(0) = _f_ibb_mps2 + _E_acc_mps2;
    res.segment<3>(3) = _v_ned_mps;
    res.segment<3>(6) = _x_gdt_rad_m();
    return res;
}
/* evaluates functor (slow) based on current EKF state estimation, returning vector of size z. */

void nav::filter_gps_obser01::jacobian_state_fast(Eigen::Matrix<double,3,12>& H_fast, const Eigen::Matrix<double,12,1>& xhat) const {
    H_fast.block<3,3>(0,6) = Eigen::Matrix3d::Identity();
    H_fast.block<3,3>(0,9) = Eigen::Matrix3d::Identity();
}
/* evaluates jacobian of functor (fast) with respect to state vector x based on state vector. */

void nav::filter_gps_obser01::jacobian_state_slow(Eigen::Matrix<double,9,12>& H_slow, const Eigen::Matrix<double,12,1>& xhat) const {
    H_slow.block<3,3>(0,6) = Eigen::Matrix3d::Identity();
    H_slow.block<3,3>(0,9) = Eigen::Matrix3d::Identity();
    H_slow.block<3,3>(3,3) = Eigen::Matrix3d::Identity();
    H_slow.block<3,3>(6,0) = Eigen::Matrix3d::Identity();
}
/* evaluates jacobian of functor (slow) with respect to state vector x based on state vector. */

void nav::filter_gps_obser01::jacobian_noise_fast(Eigen::Matrix<double,3,3>& M_fast, const Eigen::Matrix<double,12,1>& xhat) const {
    M_fast.diagonal() = Eigen::Matrix<double,3,1>::Ones();;
}
/* evaluates jacobian of functor (fast) with respect to noise vector (w or v) based on state vector. */

void nav::filter_gps_obser01::jacobian_noise_slow(Eigen::Matrix<double,9,9>& M_slow, const Eigen::Matrix<double,12,1>& xhat) const {
    M_slow.diagonal() = Eigen::Matrix<double,9,1>::Ones();;
}
/* evaluates jacobian of functor (slow) with respect to noise vector (w or v) based on state vector. */

Eigen::Matrix<double,3,3> nav::filter_gps_obser01::covariance_matrix_fast() const {
    return _R_fast;
}
/* returns covariance matrix (fast) */

Eigen::Matrix<double,9,9> nav::filter_gps_obser01::covariance_matrix_slow() const {
    return _R_slow;
}
/* returns covariance matrix (slow) */

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER GPS01
// ==================
// ==================

nav::filter_gps01::filter_gps01(const sens::suite& Osuite, const env::geo& Ogeo)
: filter_gps(Osuite),  _Pekfd_handler(nullptr), _Pstate(new nav::filter_gps_state01(Osuite, Ogeo)), _Pobser(new nav::filter_gps_obser01(Osuite, Ogeo)), _Pgeo(&Ogeo) {
}
/* constructor based on suite of sensors and Earth model */

nav::filter_gps01::~filter_gps01() {
    delete _Pekfd_handler;
    delete _Pstate;
    delete _Pobser;
}
/* destructor */

void nav::filter_gps01::finalize_constructor(const double& Deltat_sec_nav, const unsigned int& nel_nav, const unsigned int& nel_gps) {
    _Pekfd_handler       = new nav::ekfd_handler_<12,3,9,12,3,9>(nel_nav, nel_gps, Deltat_sec_nav, *_Pstate, *_Pobser);
    _Plpf_Deltap_pa      = new math::low_pass_single(300.0 * Deltat_sec_nav, Deltat_sec_nav);
    _Plpf_windlf_ned_mps = new math::low_pass_triple(300.0 * Deltat_sec_nav, Deltat_sec_nav);
}
/* complete constructor with navigation time and sizes of navigation vector and gps vectors, which are not available at construction time */

void nav::filter_gps01::initialize(st::st_nav_out& Ost_nav_out_init, const st::st_sens_out& Ost_sens_out_init, const st::st_gps_out& Ost_gps_out_init, const st::st_nav_in& Ost_nav_in_init,
                                   const Eigen::Vector3d& E_acc_init, const Eigen::Vector3d& E_acc_std_init) {
    // functor initialization
    _Pobser->initialize(Ost_gps_out_init.get_x_gdt_rad_m());

    // Initial state estimate shall be the mean, but there is no way to compute that.
    // I use the initial sensed values when possible, zero for the biases, and zero for the unknown variables.
    Eigen::Matrix<double,12,1> Vx_init;
    Vx_init.segment<3>(0)  = Ost_gps_out_init.get_x_gdt_rad_m()();
    Vx_init.segment<3>(3)  = Ost_gps_out_init.get_v_n_mps();
    Vx_init.segment<3>(6)  = Ost_sens_out_init.get_f_ibb_mps2() - E_acc_init;
    Vx_init.segment<3>(9)  = E_acc_init;

    // Initial covariance of the state estimate
    Eigen::Matrix<double,12,12> P0 = Eigen::Matrix<double,12,12>::Zero();

    double sigma_hor_m = _Psuite->get_sens_gps().get_sigma_pos_hor_m(); // OJO OJO iono effect missing //////////////////////////////////////////////////
    double sigma_ver_m = _Psuite->get_sens_gps().get_sigma_pos_ver_m(); // OJO OJO iono effect missing //////////////////////////////////////////////////
    double phi_rad = Ost_gps_out_init.get_x_gdt_rad_m().get_phi_rad();
    double h_m     = Ost_gps_out_init.get_x_gdt_rad_m().get_h_m();
    double N_m = _Pgeo->radius_vert(phi_rad);
    double M_m = _Pgeo->radius_mer(phi_rad, N_m);
    double sigma_lambda_rad = sigma_hor_m / ((N_m + h_m) * cos(phi_rad));
    double sigma_phi_rad    = sigma_hor_m / (M_m + h_m);

    double b = std::pow(_Psuite->get_sens_gps().get_sigma_vel_mps(),2.0);
    Eigen::Array3d c = E_acc_std_init.array().pow(2) + std::pow(_Psuite->get_sens_acc().get_std_white_noise(),2);
    Eigen::Array3d d = E_acc_std_init.array().pow(2);

    P0.block<3,3>(0,0).diagonal() << std::pow(sigma_lambda_rad,2.0), std::pow(sigma_phi_rad,2.0), std::pow(sigma_ver_m,2.0);
    P0.block<3,3>(3,3).diagonal() << b, b, b;
    P0.block<3,3>(6,6).diagonal() << c;
    P0.block<3,3>(9,9).diagonal() << d;

    // These are just the real initial conditions passed to evaluate the errors
    _Vx_truth.segment<3>(0) = Ost_nav_in_init.get_x_gdt_rad_m()();
    _Vx_truth.segment<3>(3) = Ost_nav_in_init.get_v_n_mps();
    _Vx_truth.segment<3>(6) = Ost_nav_in_init.get_f_ibb_mps2();
    _Vx_truth.segment<3>(9) = Ost_sens_out_init.get_E_acc_mps2();

    // initialize filter
    _Pekfd_handler->initialize(Vx_init, P0, _Vx_truth);

    // fill up Ost_nav_init
    Ost_nav_out_init.get_x_gdt_rad_m() = Vx_init.segment<3>(0);
    Ost_nav_out_init.get_v_n_mps()     = Vx_init.segment<3>(3);
    Ost_nav_out_init.get_f_ibb_mps2()  = Vx_init.segment<3>(6);
    Ost_nav_out_init.get_E_acc_mps2()  = Vx_init.segment<3>(9);

    // fill up Ost_nav_init.Deltap_pa
    double Deltap_pa_init = env::atm::obtain_Deltap_pa(Ost_nav_out_init.get_DeltaT_degK(), Ost_nav_out_init.get_Hp_m(),
                                              _Pgeo->htoH(Ost_nav_out_init.get_x_gdt_rad_m().get_h_m(), Ost_nav_out_init.get_x_gdt_rad_m().get_phi_rad()), false);
    _Plpf_Deltap_pa->init(Deltap_pa_init);
    Ost_nav_out_init.get_Deltap_pa() = _Plpf_Deltap_pa->eval(Deltap_pa_init);

    // fill up Ost_nav_out.windlf_ned_mps
    Eigen::Vector3d windlf_ned_mps_init = Ost_nav_out_init.get_v_n_mps() - Ost_nav_out_init.get_q_nb() * env::speed::vtas2vtasbfs(Ost_nav_out_init.get_vtas_mps(), Ost_nav_out_init.get_euler_wb());
    _Plpf_windlf_ned_mps->init(windlf_ned_mps_init);
    Ost_nav_out_init.get_vlf_n_mps() = _Plpf_windlf_ned_mps->eval(windlf_ned_mps_init);
}
/* initialize filter filling up the initial navigation output state vector Ost_nav_out_init, based on the initial sensors output state vector Ost_sens_out_init,
 * the initial GPS output state vector Ost_gps_out_init, the initial navigation input state vector Ost_nav_in_init (for filter evaluation purposes only), and other
 * randomly computed variables required for filter initialization. */

void nav::filter_gps01::execute_step(st::st_nav_out& Ost_nav_out, const st::st_sens_out& Ost_sens_out, const st::st_gps_out& Ost_gps_out, const st::st_nav_in& Ost_nav_in, const unsigned int& s, const unsigned int& gg, bool flag_gps) {
    // filter prediction phase
    _Pstate->update(_Pekfd_handler->get_xhat_aft()[s - 1], Ost_nav_out);
    _Pekfd_handler->process_state(s);

    if (flag_gps == false) { // no gps measurement available
        // filter update phase
        _y_fast = Ost_sens_out.get_f_ibb_mps2();
        _Pobser->update(_Pekfd_handler->get_xhat_bef()[s]);
        _Pekfd_handler->process_observation_fast(s, _y_fast);

        // filter auxiliary phase
        _Vx_truth.segment<3>(0) = Ost_nav_in.get_x_gdt_rad_m()();
        _Vx_truth.segment<3>(3) = Ost_nav_in.get_v_n_mps();
        _Vx_truth.segment<3>(6) = Ost_nav_in.get_f_ibb_mps2();
        _Vx_truth.segment<3>(9) = Ost_sens_out.get_E_acc_mps2();
        _Pekfd_handler->process_auxiliary_fast(s, _Vx_truth);
    }
    else { // gps measurement available
        // filter update phase
        _y_slow.segment<3>(0) = Ost_sens_out.get_f_ibb_mps2();
        _y_slow.segment<3>(3) = Ost_gps_out.get_v_n_mps();
        _y_slow.segment<3>(6) = Ost_gps_out.get_x_gdt_rad_m()();
        _Pobser->update(_Pekfd_handler->get_xhat_bef()[s]);
        _Pekfd_handler->process_observation_slow(s, gg, _y_slow);

        // fill up gaps in fast structures
        _Pekfd_handler->get_K_fast()[s] = _Pekfd_handler->get_K_slow()[gg].block<12, 3>(0, 0);
        _Pekfd_handler->get_y_fast()[s] = _Pekfd_handler->get_y_slow()[gg].segment<3>(0);
        _Pekfd_handler->get_r_fast()[s] = _Pekfd_handler->get_r_slow()[gg].segment<3>(0);
        _Pekfd_handler->get_S_fast()[s] = _Pekfd_handler->get_S_slow()[gg].block<3, 3>(0, 0);

        // filter auxiliary phase
        _Vx_truth.segment<3>(0) = Ost_nav_in.get_x_gdt_rad_m()();
        _Vx_truth.segment<3>(3) = Ost_nav_in.get_v_n_mps();
        _Vx_truth.segment<3>(6) = Ost_nav_in.get_f_ibb_mps2();
        _Vx_truth.segment<3>(9) = Ost_sens_out.get_E_acc_mps2();
        _Pekfd_handler->process_auxiliary_slow(s, gg, _Vx_truth);
    }

    // pass filter results to Ost_nav
    Ost_nav_out.get_x_gdt_rad_m()       = _Pekfd_handler->get_xhat_aft()[s].segment<3>(0);
    Ost_nav_out.get_v_n_mps()           = _Pekfd_handler->get_xhat_aft()[s].segment<3>(3);
    Ost_nav_out.get_f_ibb_mps2()        = _Pekfd_handler->get_xhat_aft()[s].segment<3>(6);
    Ost_nav_out.get_E_acc_mps2()        = _Pekfd_handler->get_xhat_aft()[s].segment<3>(9);
    ang::tools::correct_longitude_rad(Ost_nav_out.get_x_gdt_rad_m().get_lambda_rad());
    Ost_nav_out.get_Deltap_pa()         = _Plpf_Deltap_pa->eval(env::atm::obtain_Deltap_pa(Ost_nav_out.get_DeltaT_degK(), Ost_nav_out.get_Hp_m(),
                                          _Pgeo->htoH(Ost_nav_out.get_x_gdt_rad_m().get_h_m(), Ost_nav_out.get_x_gdt_rad_m().get_phi_rad()), true));
    Ost_nav_out.get_vlf_n_mps()      = _Plpf_windlf_ned_mps->eval(Ost_nav_out.get_v_n_mps() - Ost_nav_out.get_q_nb() * env::speed::vtas2vtasbfs(Ost_nav_out.get_vtas_mps(), Ost_nav_out.get_euler_wb()));
}
/* execute filter step filling up the navigation output state vector Ost_nav_out, based on the sensors output state vector Ost_sens_out, the GPS output
 * state vector Ost_gps_out, the navigation input state vector Ost_nav_in (for filter evaluation purposes only), the current positions for navigation and
 * GPS in their respective trajectory vectors, and the flag indicating whether there is a new GPS measurement or not. */

void nav::filter_gps01::resize(const unsigned long& nel_fast, const unsigned long& nel_slow) {
    _Pekfd_handler->resize(nel_fast, nel_slow);
}
/* resize of all filter components to input size */

unsigned long nav::filter_gps01::get_size_fast() const {
    return _Pekfd_handler->get_size_fast();
}
unsigned long nav::filter_gps01::get_size_slow() const {
    return _Pekfd_handler->get_size_slow();
}
/* returns size of all filter components */

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////







