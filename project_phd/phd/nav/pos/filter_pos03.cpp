#include "filter_pos03.h"
#include "../gps/filter_gps01.h"
#include "ang/tools.h"
#include "env/geo.h"
#include "env/speed.h"
#include "ang/rotate/rotv.h"
#include "acft/st/sti.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER POS03A
// ===================
// ===================

void nav::filter_pos03A::execute_step(st::st_nav_out& Ost_nav_out, const st::st_nav_out& Ost_nav_out_prev, const st::st_sens_out& Ost_sens_out, const st::st_nav_in& Ost_nav_in, const unsigned int& ss) {
    // execute filter to fill up specific force and accelerometer error
    this->execute_step_filter(Ost_nav_out, Ost_sens_out, Ost_nav_in, ss);

    // maintain pressure offset constant as it can not be estimated without gps
    Ost_nav_out.get_Deltap_pa() = Ost_nav_out_prev.get_Deltap_pa();

    // obtain true airspeed and its time differential both in body and NED
    const ang::rodrigues& q_nb       = Ost_nav_out.get_q_nb();
    const Eigen::Vector3d& w_nbb_rps = Ost_nav_out.get_w_nbb_rps();
    _vtas_b_mps                      = env::speed::vtas2vtasbfs(Ost_nav_out.get_vtas_mps(), Ost_nav_out.get_euler_wb());
    _vtas_b_dot_mps2                 = env::speed::vtasbfsdot(Ost_nav_out.get_vtas_mps(), Ost_nav_out.get_euler_wb(), Ost_nav_out.get_vtas_dot_mps2(), Ost_nav_out.get_euler_wb_dot_rps());
    _vtas_n_mps                      = q_nb * _vtas_b_mps;
    _vtas_n_dot_mps2                 = q_nb * (_vtas_b_dot_mps2 + w_nbb_rps.cross(_vtas_b_mps));

    // obtain ground speed time differential in NED
    const env::geodetic_coord& x_gdt_rad_m_prev = Ost_nav_out_prev.get_x_gdt_rad_m();
    const Eigen::Vector3d& v_n_mps_prev         = Ost_nav_out_prev.get_v_n_mps();

    _N_m_prev             = _Pgeo->radius_vert(x_gdt_rad_m_prev.get_phi_rad());
    _M_m_prev             = _Pgeo->radius_mer(x_gdt_rad_m_prev.get_phi_rad(), _N_m_prev);
    _w_enn_rps_prev       = _Pgeo->compute_wenn_rps(v_n_mps_prev, _N_m_prev, _M_m_prev, x_gdt_rad_m_prev.get_phi_rad(), x_gdt_rad_m_prev.get_h_m());
    _w_ien_rps_prev       = _Pgeo->compute_wien_rps(x_gdt_rad_m_prev.get_phi_rad());
    _gc_n_mps2_model_prev = _Pgeo->compute_gravity_n_model(x_gdt_rad_m_prev);
    _a_cor_n_mps2_prev    = _Pgeo->compute_coriolis_n(v_n_mps_prev, _w_ien_rps_prev);
    _v_n_dot_mps2         = q_nb * Ost_nav_out.get_f_ibb_mps2() + _gc_n_mps2_model_prev - _w_enn_rps_prev.cross(v_n_mps_prev) - _a_cor_n_mps2_prev;

    // obtain wind speed time differential in NED and integrate it
    Ost_nav_out.get_vlf_n_mps() = Ost_nav_out_prev.get_vlf_n_mps() + _Deltat_sec_nav * (_v_n_dot_mps2 - _vtas_n_dot_mps2);

    // obtain ground speed in NED adding up the airspeed and wind contributions, but replace vertical component by rate of climb
    Ost_nav_out.get_v_n_mps()    = _vtas_n_mps + Ost_nav_out.get_vlf_n_mps();
    Ost_nav_out.get_v_n_mps()(2) = - Ost_nav_out.get_roc_mps();

    // altitude computed from pressure altitude, temperature offset, and frozen pressure offset (latitude only influences in fully ellipsoidal model)
    Ost_nav_out.get_x_gdt_rad_m().get_h_m() = _Pgeo->H2h(env::atm::Hp2H(Ost_nav_out.get_Hp_m(), Ost_nav_out.get_DeltaT_degK(), Ost_nav_out.get_Deltap_pa()), x_gdt_rad_m_prev.get_phi_rad());

    // longitude and latitude computed integrating ground speed with assistance from previous step latitude and altitude computed immediately above
    Ost_nav_out.get_x_gdt_rad_m().get_lambda_rad() = x_gdt_rad_m_prev.get_lambda_rad() +
                                                     _Deltat_sec_nav * Ost_nav_out.get_v_n_mps()(1) / ((_N_m_prev + Ost_nav_out.get_x_gdt_rad_m().get_h_m()) * cos(x_gdt_rad_m_prev.get_phi_rad()));
    ang::tools::correct_longitude_rad(Ost_nav_out.get_x_gdt_rad_m().get_lambda_rad());
    Ost_nav_out.get_x_gdt_rad_m().get_phi_rad()    = x_gdt_rad_m_prev.get_phi_rad() +
                                                     _Deltat_sec_nav * Ost_nav_out.get_v_n_mps()(0) / (_M_m_prev + Ost_nav_out.get_x_gdt_rad_m().get_h_m());
}
/* execute filter step filling up the navigation output state vector Ost_nav_out, based on the previous navigation output state vector Ost_nav_out_prev,
 * the sensors output state vector Ost_sens_out, the navigation input state vector Ost_nav_in (for filter evaluation purposes only), and the current
 * navigation trajectory vector position. */

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER POS03B
// ===================
// ===================

void nav::filter_pos03B::execute_step(st::st_nav_out& Ost_nav_out, const st::st_nav_out& Ost_nav_out_prev, const st::st_sens_out& Ost_sens_out, const st::st_nav_in& Ost_nav_in, const unsigned int& ss) {
    // execute filter to fill up specific force and accelerometer error
    this->execute_step_filter(Ost_nav_out, Ost_sens_out, Ost_nav_in, ss);

    // maintain pressure offset constant as it can not be estimated without gps
    Ost_nav_out.get_Deltap_pa() = Ost_nav_out_prev.get_Deltap_pa();

    // obtain true airspeed and its time differential both in body and NED
    const ang::rodrigues& q_nb       = Ost_nav_out.get_q_nb();
    const Eigen::Vector3d& w_nbb_rps = Ost_nav_out.get_w_nbb_rps();
    _vtas_b_mps                      = env::speed::vtas2vtasbfs(Ost_nav_out.get_vtas_mps(), Ost_nav_out.get_euler_wb());
    _vtas_b_dot_mps2                 = env::speed::vtasbfsdot(Ost_nav_out.get_vtas_mps(), Ost_nav_out.get_euler_wb(), Ost_nav_out.get_vtas_dot_mps2(), Ost_nav_out.get_euler_wb_dot_rps());
    _vtas_n_mps                      = q_nb * _vtas_b_mps;
    _vtas_n_dot_mps2                 = q_nb * (_vtas_b_dot_mps2 + w_nbb_rps.cross(_vtas_b_mps));

    // obtain ground speed time differential in NED
    const env::geodetic_coord& x_gdt_rad_m_prev = Ost_nav_out_prev.get_x_gdt_rad_m();
    const Eigen::Vector3d& v_n_mps_prev         = Ost_nav_out_prev.get_v_n_mps();

    _N_m_prev             = _Pgeo->radius_vert(x_gdt_rad_m_prev.get_phi_rad());
    _M_m_prev             = _Pgeo->radius_mer(x_gdt_rad_m_prev.get_phi_rad(), _N_m_prev);
    _w_enn_rps_prev       = _Pgeo->compute_wenn_rps(v_n_mps_prev, _N_m_prev, _M_m_prev, x_gdt_rad_m_prev.get_phi_rad(), x_gdt_rad_m_prev.get_h_m());
    _w_ien_rps_prev       = _Pgeo->compute_wien_rps(x_gdt_rad_m_prev.get_phi_rad());
    _gc_n_mps2_model_prev = _Pgeo->compute_gravity_n_model(x_gdt_rad_m_prev);
    _a_cor_n_mps2_prev    = _Pgeo->compute_coriolis_n(v_n_mps_prev, _w_ien_rps_prev);
    _v_n_dot_mps2         = q_nb * Ost_nav_out.get_f_ibb_mps2() + _gc_n_mps2_model_prev - _w_enn_rps_prev.cross(v_n_mps_prev) - _a_cor_n_mps2_prev;

    // obtain wind speed time differential in NED and integrate it
    Ost_nav_out.get_vlf_n_mps() = Ost_nav_out_prev.get_vlf_n_mps() + _Deltat_sec_nav * (_v_n_dot_mps2 - _vtas_n_dot_mps2);

    // obtain ground speed in NED adding up the airspeed and wind contributions
    Ost_nav_out.get_v_n_mps()    = _vtas_n_mps + Ost_nav_out.get_vlf_n_mps();

    // altitude computed from pressure altitude, temperature offset, and frozen pressure offset (latitude only influences in fully ellipsoidal model)
    Ost_nav_out.get_x_gdt_rad_m().get_h_m() = _Pgeo->H2h(env::atm::Hp2H(Ost_nav_out.get_Hp_m(), Ost_nav_out.get_DeltaT_degK(), Ost_nav_out.get_Deltap_pa()), x_gdt_rad_m_prev.get_phi_rad());

    // longitude and latitude computed integrating ground speed with assistance from previous step latitude and altitude computed immediately above
    Ost_nav_out.get_x_gdt_rad_m().get_lambda_rad() = x_gdt_rad_m_prev.get_lambda_rad() +
                                                     _Deltat_sec_nav * Ost_nav_out.get_v_n_mps()(1) / ((_N_m_prev + Ost_nav_out.get_x_gdt_rad_m().get_h_m()) * cos(x_gdt_rad_m_prev.get_phi_rad()));
    ang::tools::correct_longitude_rad(Ost_nav_out.get_x_gdt_rad_m().get_lambda_rad());
    Ost_nav_out.get_x_gdt_rad_m().get_phi_rad()    = x_gdt_rad_m_prev.get_phi_rad() +
                                                     _Deltat_sec_nav * Ost_nav_out.get_v_n_mps()(0) / (_M_m_prev + Ost_nav_out.get_x_gdt_rad_m().get_h_m());
}
/* execute filter step filling up the navigation output state vector Ost_nav_out, based on the previous navigation output state vector Ost_nav_out_prev,
 * the sensors output state vector Ost_sens_out, the navigation input state vector Ost_nav_in (for filter evaluation purposes only), and the current
 * navigation trajectory vector position. */

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER POS03C
// ===================
// ===================

void nav::filter_pos03C::execute_step(st::st_nav_out& Ost_nav_out, const st::st_nav_out& Ost_nav_out_prev, const st::st_sens_out& Ost_sens_out, const st::st_nav_in& Ost_nav_in, const unsigned int& ss) {
    // execute filter to fill up specific force and accelerometer error
    this->execute_step_filter(Ost_nav_out, Ost_sens_out, Ost_nav_in, ss);

    // maintain pressure offset constant as it can not be estimated without gps
    Ost_nav_out.get_Deltap_pa() = Ost_nav_out_prev.get_Deltap_pa();

    // obtain true airspeed and its time differential both in body and NED
    const ang::rodrigues& q_nb       = Ost_nav_out.get_q_nb();
    const Eigen::Vector3d& w_nbb_rps = Ost_nav_out.get_w_nbb_rps();
    _vtas_b_mps                      = env::speed::vtas2vtasbfs(Ost_nav_out.get_vtas_mps(), Ost_nav_out.get_euler_wb());
    _vtas_b_dot_mps2                 = env::speed::vtasbfsdot(Ost_nav_out.get_vtas_mps(), Ost_nav_out.get_euler_wb(), Ost_nav_out.get_vtas_dot_mps2(), Ost_nav_out.get_euler_wb_dot_rps());
    _vtas_n_mps                      = q_nb * _vtas_b_mps;
    _vtas_n_dot_mps2                 = q_nb * (_vtas_b_dot_mps2 + w_nbb_rps.cross(_vtas_b_mps));

    // obtain ground speed time differential in NED
    const env::geodetic_coord& x_gdt_rad_m_prev = Ost_nav_out_prev.get_x_gdt_rad_m();
    const Eigen::Vector3d& v_n_mps_prev         = Ost_nav_out_prev.get_v_n_mps();

    _N_m_prev             = _Pgeo->radius_vert(x_gdt_rad_m_prev.get_phi_rad());
    _M_m_prev             = _Pgeo->radius_mer(x_gdt_rad_m_prev.get_phi_rad(), _N_m_prev);
    _w_enn_rps_prev       = _Pgeo->compute_wenn_rps(v_n_mps_prev, _N_m_prev, _M_m_prev, x_gdt_rad_m_prev.get_phi_rad(), x_gdt_rad_m_prev.get_h_m());
    _w_ien_rps_prev       = _Pgeo->compute_wien_rps(x_gdt_rad_m_prev.get_phi_rad());
    _gc_n_mps2_model_prev = _Pgeo->compute_gravity_n_model(x_gdt_rad_m_prev);
    _a_cor_n_mps2_prev    = _Pgeo->compute_coriolis_n(v_n_mps_prev, _w_ien_rps_prev);
    _v_n_dot_mps2         = q_nb * Ost_nav_out.get_f_ibb_mps2() + _gc_n_mps2_model_prev - _w_enn_rps_prev.cross(v_n_mps_prev) - _a_cor_n_mps2_prev;

    // obtain wind speed time differential in NED and integrate it
    Ost_nav_out.get_vlf_n_mps() = Ost_nav_out_prev.get_vlf_n_mps() + _Deltat_sec_nav * (_v_n_dot_mps2 - _vtas_n_dot_mps2);

    // obtain ground speed in NED adding up the airspeed and wind contributions
    Ost_nav_out.get_v_n_mps()    = _vtas_n_mps + Ost_nav_out.get_vlf_n_mps();

    // integrate ground speed to obtain position
    _x_gdt_rad_m_dot = env::geo::vned_to_xgdtdot(Ost_nav_out.get_v_n_mps(), x_gdt_rad_m_prev, _N_m_prev, _M_m_prev);
    Ost_nav_out.get_x_gdt_rad_m()   = x_gdt_rad_m_prev + _Deltat_sec_nav * _x_gdt_rad_m_dot;
    ang::tools::correct_longitude_rad(Ost_nav_out.get_x_gdt_rad_m().get_lambda_rad());
}
/* execute filter step filling up the navigation output state vector Ost_nav_out, based on the previous navigation output state vector Ost_nav_out_prev,
 * the sensors output state vector Ost_sens_out, the navigation input state vector Ost_nav_in (for filter evaluation purposes only), and the current
 * navigation trajectory vector position. */

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////






