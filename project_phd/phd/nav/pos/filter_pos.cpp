#include "filter_pos.h"
#include "../gps/filter_gps01.h"
#include "env/geo.h"
#include "acft/st/sti.h"

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER POS FUNCTOR STATE
// ==============================
// ==============================

nav::filter_pos_state::filter_pos_state(const sens::suite& Osuite)
: _Qtilde(Eigen::Matrix<double,6,6>::Zero()) {
    double c = 1e-4;
    double d = 1e-3 * std::pow(Osuite.get_sens_acc().get_std_bias_drift(),2);
    //////////////////////////////////////////////////////////////////////////////////
    // IMPORTANT NOTE: The 1e-3 factor in d is very important. Otherwise, Eacc slowly
    // drifts with time, and it can go anywhere. Most likely (refer to tests 33 and 34) doing
    // turns stabilizes it. The issue is that the error in Eacc is the opposite of the
    // error in f_ibb. This way it seems to work, but otherwise it looks like only
    // possibility is to do a lateral turn every 300 sec or so
    //////////////////////////////////////////////////////////////////////////////////
    _Qtilde.diagonal() << c, c, c, d, d, d;
}
/* constructor based on suite of sensors */

void nav::filter_pos_state::initialize() {
}
/* initialization */

void nav::filter_pos_state::update(const Eigen::Matrix<double,6,1>& xhat_aft_prev, const st::st_nav_out& Ost_nav_out) {
    _f_ibb_mps2 << xhat_aft_prev.segment<3>(0);
    _E_acc_mps2 << xhat_aft_prev.segment<3>(3);
}
/* updates the functor based on previous EKF state estimation and previous sensors outputs */

Eigen::Matrix<double,6,1> nav::filter_pos_state::eval(const Eigen::Matrix<double,6,1>& xhat_aft_prev) const {
    Eigen::Matrix<double,6, 1> res;
    res.segment<3>(0) = Eigen::Vector3d::Zero();
    res.segment<3>(3) = Eigen::Vector3d::Zero();
    return res;
}
/* evaluates functor based on previous EKF state estimation, returning vector of size z. */

void nav::filter_pos_state::jacobian_state(Eigen::Matrix<double,6,6>& A, const Eigen::Matrix<double,6,1>& xhat_aft_prev) const {
}
/* evaluates jacobian of functor with respect to state vector x based on state vector. */

Eigen::Matrix<double,6,6> nav::filter_pos_state::covariance_matrix() const {
    return _Qtilde;
}
/* returns covariance matrix */

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER POS FUNCTOR OBSER
// ==============================
// ==============================

nav::filter_pos_obser::filter_pos_obser(const sens::suite& Osuite)
: _R(Eigen::Matrix<double,3,3>::Zero()) {
    double a = std::pow(Osuite.get_sens_acc().get_std_white_noise(),2.0);
    _R.diagonal() << a, a, a;
}
/* constructor based on suite of sensors */

void nav::filter_pos_obser::initialize() {
}
/* initialization */

void nav::filter_pos_obser::update(const Eigen::Matrix<double,6,1>& xhat_bef) {
    _f_ibb_mps2 << xhat_bef.segment<3>(0);
    _E_acc_mps2 << xhat_bef.segment<3>(3);
}
/* updates the functor based on current EKF state estimation, current sensors outputs, and XXXXXXXXX */

Eigen::Matrix<double,3,1> nav::filter_pos_obser::eval(const Eigen::Matrix<double,6,1>& xhat_bef) const {
    Eigen::Matrix<double,3,1> res = Eigen::Matrix<double,3,1>::Zero();
    res.segment<3>(0) = _f_ibb_mps2 + _E_acc_mps2;
    return res;
}
/* evaluates functor based on current EKF state estimation, returning vector of size z. */

void nav::filter_pos_obser::jacobian_state(Eigen::Matrix<double,3,6>& H, const Eigen::Matrix<double,6,1>& xhat) const {
    H.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
    H.block<3,3>(0,3) = Eigen::Matrix3d::Identity();
}
/* evaluates jacobian of functor with respect to state vector x based on state vector. */

void nav::filter_pos_obser::jacobian_noise(Eigen::Matrix<double,3,3>& M_loss, const Eigen::Matrix<double,6,1>& xhat) const {
    M_loss.diagonal() = Eigen::Matrix<double,3,1>::Ones();;
}
/* evaluates jacobian of functor (gps loss) with respect to noise vector (w or v) based on state vector. */

Eigen::Matrix<double,3,3> nav::filter_pos_obser::covariance_matrix() const {
    return _R;
}
/* returns covariance matrix (gps loss) */

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER POS
// ================
// ================

nav::filter_pos::filter_pos(const sens::suite& Osuite, const env::geo& Ogeo)
: _Psuite(&Osuite), _Pgeo(&Ogeo), _Pstate(new nav::filter_pos_state(Osuite)), _Pobser(new nav::filter_pos_obser(Osuite)),  _Pekf_handler(nullptr) {
}
/* constructor based on suite of sensors and Earth model */

nav::filter_pos::~filter_pos() {
    delete _Pstate;
    delete _Pobser;
    delete _Pekf_handler;
}
/* destructor */

void nav::filter_pos::finalize_constructor(const double& Deltat_sec_nav, const unsigned int& nel_nav) {
    _Deltat_sec_nav = Deltat_sec_nav;
    _Pekf_handler = new nav::ekf_handler_<6,3,6,3>(nel_nav, Deltat_sec_nav, *_Pstate, *_Pobser);
}
/* complete constructor with navigation time and size of navigation vector, which are not available at construction time */

void nav::filter_pos::initialize(const nav::filter_gps01& Ogps, const st::st_nav_out& Ost_nav_out_init, const unsigned int& s_init) {
    _s_init  = s_init;

    // functor initialization
    _Pstate->initialize();
    _Pobser->initialize();

    // filter holder initialization
    Eigen::Matrix<double,6,3> K_init;
    K_init.block<6,3>(0,0) = Ogps.get_ekfd_handler().get_K_slow().back().block<6,3>(6,0);
    _y = Ogps.get_ekfd_handler().get_y_slow().back().segment<3>(0);
    Eigen::Matrix<double,3,1> r_init = Ogps.get_ekfd_handler().get_r_slow().back().segment<3>(0);
    Eigen::Matrix<double,3,3> S_init = Ogps.get_ekfd_handler().get_S_slow().back().block<3,3>(0,0);
    Eigen::Matrix<double,3,1> r_mean_init = Ogps.get_ekfd_handler().get_r_mean_slow().back().segment<3>(0);
    _Pekf_handler->initialize(Ogps.get_ekfd_handler().get_xhat_bef().back().segment<6>(6), Ogps.get_ekfd_handler().get_P_bef().back().block<6,6>(6,6),
                              K_init, _y, r_init, S_init,
                              Ogps.get_ekfd_handler().get_xhat_aft().back().segment<6>(6), Ogps.get_ekfd_handler().get_P_aft().back().block<6,6>(6,6),
                              Ogps.get_ekfd_handler().get_eps_xhat_bef_mean().back().segment<6>(6), Ogps.get_ekfd_handler().get_eps_xhat_aft_mean().back().segment<6>(6),
                              r_mean_init);
}
/* execute filter step filling up the navigation output state vector Ost_nav_out, based on the previous navigation output state vector Ost_nav_out_prev,
 * the sensors output state vector Ost_sens_out, the navigation input state vector Ost_nav_in (for filter evaluation purposes only), and the current
 * navigation trajectory vector position. */

void nav::filter_pos::execute_step_filter(st::st_nav_out& Ost_nav_out, const st::st_sens_out& Ost_sens_out, const st::st_nav_in& Ost_nav_in, const unsigned int& ss) {
    // position within this filter
    unsigned int s = ss - _s_init;

    // filter prediction phase
    _Pstate->update(_Pekf_handler->get_xhat_aft()[s - 1], Ost_nav_out);
    _Pekf_handler->process_state(s);

    // filter update phase
    _y = Ost_sens_out.get_f_ibb_mps2();
    _Pobser->update(_Pekf_handler->get_xhat_bef()[s]);
    _Pekf_handler->process_observation(s, _y);

    // filter auxiliary phase
    _Vx_truth.segment<3>(0) = Ost_nav_in.get_f_ibb_mps2();
    _Vx_truth.segment<3>(3) = Ost_sens_out.get_E_acc_mps2();
    _Pekf_handler->process_auxiliary(s, _Vx_truth);

    // pass filter results to Ost_nav
    Ost_nav_out.get_f_ibb_mps2() = _Pekf_handler->get_xhat_aft()[s].segment<3>(0);
    Ost_nav_out.get_E_acc_mps2() = _Pekf_handler->get_xhat_aft()[s].segment<3>(3);
}
/* execute filter step filling up the navigation output state vector Ost_nav_out, based on the sensors output state vector Ost_sens_out,
 * the navigation input state vector Ost_nav_in (for filter evaluation purposes only), and the current navigation trajectory vector position. */

void nav::filter_pos::resize(const unsigned long& nel) {
    _Pekf_handler->resize(nel);
}
/* resize of all filter components to input size */

unsigned long nav::filter_pos::get_size() const {
    return _Pekf_handler->get_size();
}
/* returns size of all filter components */

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////




