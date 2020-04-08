#include "filter_pos02.h"
#include "../gps/filter_gps01.h"
#include "ang/tools.h"
#include "env/geo.h"
#include "env/speed.h"
#include "ang/rotate/rotv.h"
#include "acft/st/sti.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER POS02A
// ===================
// ===================

void nav::filter_pos02A::execute_step(st::st_nav_out& Ost_nav_out, const st::st_nav_out& Ost_nav_out_prev, const st::st_sens_out& Ost_sens_out, const st::st_nav_in& Ost_nav_in, const unsigned int& ss) {
    // execute filter to fill up specific force and accelerometer error
    this->execute_step_filter(Ost_nav_out, Ost_sens_out, Ost_nav_in, ss);

    // maintain pressure offset constant as it can not be estimated without gps
    Ost_nav_out.get_Deltap_pa() = Ost_nav_out_prev.get_Deltap_pa();

    // maintain wind speed constant as they can not be estimated without gps
    Ost_nav_out.get_vlf_n_mps() = Ost_nav_out_prev.get_vlf_n_mps();

    // ground speed computed from air speed, quaternion, and frozen wind speed
    Ost_nav_out.get_v_n_mps()  = Ost_nav_out.get_vlf_n_mps() +
                                 Ost_nav_out.get_q_nb() * env::speed::vtas2vtasbfs(Ost_nav_out.get_vtas_mps(), Ost_nav_out.get_euler_wb());
    Ost_nav_out.get_v_n_mps()(2) = - Ost_nav_out.get_roc_mps();

    // altitude computed from pressure altitude, temperature offset, and frozen pressure offset (latitude only influences in fully ellipsoidal model)
    const env::geodetic_coord& x_gdt_rad_m_prev = Ost_nav_out_prev.get_x_gdt_rad_m();

    _N_m_prev = _Pgeo->radius_vert(x_gdt_rad_m_prev.get_phi_rad());
    _M_m_prev = _Pgeo->radius_mer(x_gdt_rad_m_prev.get_phi_rad(), _N_m_prev);

    Ost_nav_out.get_x_gdt_rad_m().get_h_m() = _Pgeo->H2h(env::atm::Hp2H(Ost_nav_out.get_Hp_m(), Ost_nav_out.get_DeltaT_degK(), Ost_nav_out.get_Deltap_pa()), x_gdt_rad_m_prev.get_phi_rad());

    // longitude and latitude computed integrating ground speed with assistance from previous step latitude and altitude computed inmediately above
    Ost_nav_out.get_x_gdt_rad_m().get_lambda_rad() = x_gdt_rad_m_prev.get_lambda_rad() +
                                                     _Deltat_sec_nav * Ost_nav_out.get_v_n_mps()(1) / ((_N_m_prev + Ost_nav_out.get_x_gdt_rad_m().get_h_m()) * cos(x_gdt_rad_m_prev.get_phi_rad()));
    ang::tools::correct_longitude_rad(Ost_nav_out.get_x_gdt_rad_m().get_lambda_rad());
    Ost_nav_out.get_x_gdt_rad_m().get_phi_rad() = x_gdt_rad_m_prev.get_phi_rad() + _Deltat_sec_nav * Ost_nav_out.get_v_n_mps()(0) / (_M_m_prev + Ost_nav_out.get_x_gdt_rad_m().get_h_m());
}
/* execute filter step filling up the navigation output state vector Ost_nav_out, based on the previous navigation output state vector Ost_nav_out_prev,
 * the sensors output state vector Ost_sens_out, the navigation input state vector Ost_nav_in (for filter evaluation purposes only), and the current
 * navigation trajectory vector position. */

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER POS02B
// ===================
// ===================

void nav::filter_pos02B::execute_step(st::st_nav_out& Ost_nav_out, const st::st_nav_out& Ost_nav_out_prev, const st::st_sens_out& Ost_sens_out, const st::st_nav_in& Ost_nav_in, const unsigned int& ss) {
    // execute filter to fill up specific force and accelerometer error
    this->execute_step_filter(Ost_nav_out, Ost_sens_out, Ost_nav_in, ss);

    // maintain pressure offset constant as it can not be estimated without gps
    Ost_nav_out.get_Deltap_pa() = Ost_nav_out_prev.get_Deltap_pa();

    // maintain wind speed constant as they can not be estimated without gps
    Ost_nav_out.get_vlf_n_mps() = Ost_nav_out_prev.get_vlf_n_mps();

    // ground speed computed from air speed, quaternion, and frozen wind speed
    Ost_nav_out.get_v_n_mps()  = Ost_nav_out.get_vlf_n_mps() +
                                 Ost_nav_out.get_q_nb() * env::speed::vtas2vtasbfs(Ost_nav_out.get_vtas_mps(), Ost_nav_out.get_euler_wb());

    // altitude computed from pressure altitude, temperature offset, and frozen pressure offset (latitude only influences in fully ellipsoidal model)
    const env::geodetic_coord& x_gdt_rad_m_prev = Ost_nav_out_prev.get_x_gdt_rad_m();

    _N_m_prev = _Pgeo->radius_vert(x_gdt_rad_m_prev.get_phi_rad());
    _M_m_prev = _Pgeo->radius_mer(x_gdt_rad_m_prev.get_phi_rad(), _N_m_prev);

    Ost_nav_out.get_x_gdt_rad_m().get_h_m() = _Pgeo->H2h(env::atm::Hp2H(Ost_nav_out.get_Hp_m(), Ost_nav_out.get_DeltaT_degK(), Ost_nav_out.get_Deltap_pa()), x_gdt_rad_m_prev.get_phi_rad());

    // longitude and latitude computed integrating ground speed with assistance from previous step latitude and altitude computed inmediately above
    Ost_nav_out.get_x_gdt_rad_m().get_lambda_rad() = x_gdt_rad_m_prev.get_lambda_rad() +
            _Deltat_sec_nav * Ost_nav_out.get_v_n_mps()(1) / ((_N_m_prev + Ost_nav_out.get_x_gdt_rad_m().get_h_m()) * cos(x_gdt_rad_m_prev.get_phi_rad()));
    ang::tools::correct_longitude_rad(Ost_nav_out.get_x_gdt_rad_m().get_lambda_rad());
    Ost_nav_out.get_x_gdt_rad_m().get_phi_rad() = x_gdt_rad_m_prev.get_phi_rad() + _Deltat_sec_nav * Ost_nav_out.get_v_n_mps()(0) / (_M_m_prev + Ost_nav_out.get_x_gdt_rad_m().get_h_m());
}
/* execute filter step filling up the navigation output state vector Ost_nav_out, based on the previous navigation output state vector Ost_nav_out_prev,
 * the sensors output state vector Ost_sens_out, the navigation input state vector Ost_nav_in (for filter evaluation purposes only), and the current
 * navigation trajectory vector position. */

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER POS02C
// ===================
// ===================

void nav::filter_pos02C::execute_step(st::st_nav_out& Ost_nav_out, const st::st_nav_out& Ost_nav_out_prev, const st::st_sens_out& Ost_sens_out, const st::st_nav_in& Ost_nav_in, const unsigned int& ss) {
    // execute filter to fill up specific force and accelerometer error
    this->execute_step_filter(Ost_nav_out, Ost_sens_out, Ost_nav_in, ss);

    // maintain pressure offset constant as it can not be estimated without gps
    Ost_nav_out.get_Deltap_pa() = Ost_nav_out_prev.get_Deltap_pa();

    // maintain wind speed constant as they can not be estimated without gps
    Ost_nav_out.get_vlf_n_mps() = Ost_nav_out_prev.get_vlf_n_mps();

    // ground speed computed from air speed, quaternion, and frozen wind speed
    Ost_nav_out.get_v_n_mps()  = Ost_nav_out.get_vlf_n_mps() +
                                 Ost_nav_out.get_q_nb() * env::speed::vtas2vtasbfs(Ost_nav_out.get_vtas_mps(), Ost_nav_out.get_euler_wb());

    // integrate ground speed to obtain position
    const env::geodetic_coord& x_gdt_rad_m_prev = Ost_nav_out_prev.get_x_gdt_rad_m();

    _N_m_prev = _Pgeo->radius_vert(x_gdt_rad_m_prev.get_phi_rad());
    _M_m_prev = _Pgeo->radius_mer(x_gdt_rad_m_prev.get_phi_rad(), _N_m_prev);

    _x_gdt_rad_m_dot = env::geo::vned_to_xgdtdot(Ost_nav_out.get_v_n_mps(), x_gdt_rad_m_prev, _N_m_prev, _M_m_prev);
    Ost_nav_out.get_x_gdt_rad_m()   = x_gdt_rad_m_prev + _Deltat_sec_nav * _x_gdt_rad_m_dot;
    ang::tools::correct_longitude_rad(Ost_nav_out.get_x_gdt_rad_m().get_lambda_rad());
}
/* execute filter step filling up the navigation output state vector Ost_nav_out, based on the previous navigation output state vector Ost_nav_out_prev,
 * the sensors output state vector Ost_sens_out, the navigation input state vector Ost_nav_in (for filter evaluation purposes only), and the current
 * navigation trajectory vector position. */

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER POS02D
// ===================
// ===================

void nav::filter_pos02D::execute_step(st::st_nav_out& Ost_nav_out, const st::st_nav_out& Ost_nav_out_prev, const st::st_sens_out& Ost_sens_out, const st::st_nav_in& Ost_nav_in, const unsigned int& ss) {
    // execute filter to fill up specific force and accelerometer error
    this->execute_step_filter(Ost_nav_out, Ost_sens_out, Ost_nav_in, ss);

    // maintain pressure offset constant as it can not be estimated without gps
    Ost_nav_out.get_Deltap_pa() = Ost_nav_out_prev.get_Deltap_pa();

    // maintain horizontal wind speed constant as it can not be estimated without gps
    // impose zero vertical wind speed
    Ost_nav_out.get_vlf_n_mps() = Ost_nav_out_prev.get_vlf_n_mps();
    Ost_nav_out.get_vlf_n_mps()(2) = 0.;

    // ground speed computed from air speed, quaternion, and frozen wind speed
    Ost_nav_out.get_v_n_mps()  = Ost_nav_out.get_vlf_n_mps() +
                                 Ost_nav_out.get_q_nb() * env::speed::vtas2vtasbfs(Ost_nav_out.get_vtas_mps(), Ost_nav_out.get_euler_wb());

    // integrate ground speed to obtain position
    const env::geodetic_coord& x_gdt_rad_m_prev = Ost_nav_out_prev.get_x_gdt_rad_m();

    _N_m_prev = _Pgeo->radius_vert(x_gdt_rad_m_prev.get_phi_rad());
    _M_m_prev = _Pgeo->radius_mer(x_gdt_rad_m_prev.get_phi_rad(), _N_m_prev);

    _x_gdt_rad_m_dot = env::geo::vned_to_xgdtdot(Ost_nav_out.get_v_n_mps(), x_gdt_rad_m_prev, _N_m_prev, _M_m_prev);
    Ost_nav_out.get_x_gdt_rad_m()   = x_gdt_rad_m_prev + _Deltat_sec_nav * _x_gdt_rad_m_dot;
    ang::tools::correct_longitude_rad(Ost_nav_out.get_x_gdt_rad_m().get_lambda_rad());
}
/* execute filter step filling up the navigation output state vector Ost_nav_out, based on the previous navigation output state vector Ost_nav_out_prev,
 * the sensors output state vector Ost_sens_out, the navigation input state vector Ost_nav_in (for filter evaluation purposes only), and the current
 * navigation trajectory vector position. */

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////










