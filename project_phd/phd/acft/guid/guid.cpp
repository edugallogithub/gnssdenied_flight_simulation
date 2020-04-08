#include "guid.h"
#include "trg.h"
#include "ang/tools.h"

// CLASS GUIDANCE_OPERATION
// ========================
// ========================

control::guid_op::guid_op(control::logic::THR_ID guid_thr_id,
                         const double& guid_thr_val,    control::logic::ELV_ID guid_elv_id,
                         const double& guid_elv_val,    control::logic::AIL_ID guid_ail_id,
                         const double& guid_ail_val,    control::logic::RUD_ID guid_rud_id,
                         const double& guid_rud_val,    control::logic::TRG_ID guid_trg_id,
                         const double& guid_trg_val)
: _guid_thr_id(guid_thr_id), _guid_elv_id(guid_elv_id), _guid_ail_id(guid_ail_id), _guid_rud_id(guid_rud_id),
  _guid_val(guid_thr_val, guid_elv_val, guid_ail_val, guid_rud_val),
  _guid_trg_id(guid_trg_id), _guid_trg_val(guid_trg_val),
  _Pguid_trg_functor(nullptr) {
    switch (_guid_trg_id) {
        case control::logic::trgg_t_sec:
            _Pguid_trg_functor = new control::trg_t_sec(*this);
            break;
        case control::logic::trgg_Deltat_sec:
            _Pguid_trg_functor = new control::trg_Deltat_sec(*this);
            break;
        case control::logic::trgg_h_m:
            _Pguid_trg_functor = new control::trg_h_m(*this);
            break;
        case control::logic::trgg_Hp_m:
            _Pguid_trg_functor = new control::trg_Hp_m(*this);
            break;
        case control::logic::trgg_gamma_deg:
            _Pguid_trg_functor = new control::trg_gamma_deg(*this);
            break;
        case control::logic::trgg_chi_deg:
            _Pguid_trg_functor = new control::trg_chi_deg(*this);
            break;
        case control::logic::trgg_psi_deg:
            _Pguid_trg_functor = new control::trg_psi_deg(*this);
            break;
        case control::logic::trgg_id_size:
        default:
            throw "Trigger case not available";
    }
}
/* constructor based on guidance identificators */

control::guid_op::~guid_op() {
    delete _Pguid_trg_functor;
}
/* destructor */

void control::guid_op::init_trg(const st::st_nav_out& Ost_nav_out) {
    _Pguid_trg_functor->initialize(Ost_nav_out);
}
/* initialize trigger so it later jumps when it switches sign */

bool control::guid_op::eval_trg(const st::st_nav_out& Ost_nav_out) const {
    return _Pguid_trg_functor->eval(Ost_nav_out);
}
/* evaluate trigger, returning true when it jumps */

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS GUIDANCE
// ==============
// ==============

control::guid::guid(const unsigned short& nel_op, const double& t_sec_end, const int& seed, const unsigned short& case_guid)
: _sti_id(st::logic::sti_size), _nel_op(nel_op), _Vguid_op(nel_op, nullptr), _t_sec_end(t_sec_end), _case_guid(case_guid),
  _gen(seed), _dist_uniform_bearing(-179, 180), _dist_normal(0.,1.), _dist_uniform_time(10,50) {
}
/* constructor based on number of operations (user responsibility to fill it up afterwards) and final time.
 * Seed not employed*/

control::guid::~guid() {
    for (unsigned short i = 0; i != _nel_op; ++i) {
        delete _Vguid_op[i];
    }
}
/* destructor */

void control::guid::add_op(const unsigned short& index, control::guid_op* Pop) {
    delete _Vguid_op[index];
    _Vguid_op[index] = Pop;
}
/* adds operation at position indicated by index */

using namespace control::logic;

control::guid* control::guid::create_guid(math::seeder& Oseeder, const unsigned short& case_guid, const double& t_sec_gpsloss, bool flag_console) {
    switch (case_guid) {

        // ==================================================================
        // ===== ===== ===== Random Navigation Trajectories ===== ===== =====
        // ==================================================================

        case 1: { // change of bearing, airspeed, and pressure altitude
            // ** With filter_air02 and new tas,oat,osp sensors with biases
            // 01 -> rotv [deg] +1.248e-01 +8.247e-02 w_nbb [dps] +1.110e-01 +5.312e-02 E_gyr [dps] +1.309e-03 +5.240e-04 E_mag [nT] +5.720e+00 +1.162e+00 error_mag [nT] +3.405e+01 +9.119e+00 x_n hor [m] +1.256e+03 +5.674e+02 x_n ver [m] -3.090e+00 +2.006e+00 v_n [mps] +1.313e+00 +6.402e-01 f_ibb [mps2] +1.690e-02 +7.617e-03 E_acc [mps2] +1.927e-02 +9.261e-03
            // 02 -> rotv [deg] +1.585e-01 +1.169e-01 w_nbb [dps] +1.077e-01 +5.224e-02 E_gyr [dps] +1.274e-03 +5.366e-04 E_mag [nT] +3.210e+01 +7.213e+00 error_mag [nT] +8.123e+00 +3.808e+00 x_n hor [m] +2.500e+03 +2.094e+03 x_n ver [m] -1.395e+01 +9.240e+00 v_n [mps] +1.892e+00 +7.396e-01 f_ibb [mps2] +1.626e-02 +7.615e-03 E_acc [mps2] +1.864e-02 +9.453e-03
            // 03 -> rotv [deg] +4.210e-01 +1.789e-01 w_nbb [dps] +1.217e-01 +5.386e-02 E_gyr [dps] +1.352e-03 +6.310e-04 E_mag [nT] +6.215e+01 +1.381e+01 error_mag [nT] +3.730e+01 +1.429e+01 x_n hor [m] +2.471e+03 +1.497e+03 x_n ver [m] -6.818e+00 +6.994e+00 v_n [mps] +1.496e+00 +4.552e-01 f_ibb [mps2] +1.639e-02 +7.626e-03 E_acc [mps2] +1.883e-02 +9.415e-03

            double t_sec_end = 3800.0;
            control::guid* Pguid = new control::guid(6, t_sec_end, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid);
            Pguid->_sti_id = st::logic::sti_HpXX_tasXX_chiXX;

            double beta_deg = 0.0;

            double chi_deg_ini = Pguid->_dist_uniform_bearing(Pguid->_gen);
            double chi_deg_end_eff, chi_deg_end_trg, angle_diff_deg, xi_deg;
            do { // ensure that there is a direction change of at least 10 [deg]
                chi_deg_end_eff = Pguid->_dist_uniform_bearing(Pguid->_gen);
                angle_diff_deg = ang::tools::angle_diff_deg(chi_deg_end_eff, chi_deg_ini);
            } while (std::fabs(angle_diff_deg) < 10.0);
            // turn direction depends on initial and final bearings
            // turning end 5 [deg] earlier for better control
            if (angle_diff_deg > 0) {
                xi_deg          = +10.0;
                chi_deg_end_trg = chi_deg_end_eff - 5.;
            }
            else {
                xi_deg          = -10.0;
                chi_deg_end_trg = chi_deg_end_eff + 5.;
            }
            ang::tools::correct_yaw_deg(chi_deg_end_trg);

            double Deltat_sec_tas;
            do {
                Deltat_sec_tas = 500.0 + 100.0 * Pguid->_dist_normal(Pguid->_gen);
            } while (Deltat_sec_tas < 150.0);

            double vtas_mps_ini;
            do {
                vtas_mps_ini = 29.0 + 1.5 * Pguid->_dist_normal(Pguid->_gen);
            } while ((vtas_mps_ini > 34.0) || (vtas_mps_ini < 24.0));
            double vtas_mps_end, vtas_mps_diff;
            do {
                vtas_mps_diff = 1.5 * Pguid->_dist_normal(Pguid->_gen);
                vtas_mps_end  = vtas_mps_ini + vtas_mps_diff;

            } while ((std::fabs(vtas_mps_diff) < 0.5) || (vtas_mps_end > 34.0) || (vtas_mps_end < 24.0));

            double Deltat_sec_Hp;
            do {
                Deltat_sec_Hp = 500.0 + 100.0 * Pguid->_dist_normal(Pguid->_gen);
            } while ((Deltat_sec_Hp < 150.0) || ((Deltat_sec_tas + Deltat_sec_Hp) > 2500.0));
            double Hp_m_ini = 2700.0 + 200.0 * Pguid->_dist_normal(Pguid->_gen);
            double Hp_m_end, Hp_m_diff, gammaTAS_deg;
            do { // ensure that there is a pressure altitude change of at least 100 [m]
                Hp_m_diff = 300.0 * Pguid->_dist_normal(Pguid->_gen);
            } while (std::fabs(Hp_m_diff) < 100.0);
            Hp_m_end = Hp_m_ini + Hp_m_diff;
            // path angle depends on initial and final pressure altitudes
            if (Hp_m_diff > 0) {gammaTAS_deg = +2.0;}
            else               {gammaTAS_deg = -2.0;}

            double t_sec_turn;
            do {
                t_sec_turn = t_sec_gpsloss + 30.0 + 50.0 * Pguid->_dist_normal(Pguid->_gen);
            } while (t_sec_turn < (t_sec_gpsloss + 15.0));

            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, vtas_mps_ini, elv_Hp_m,         Hp_m_ini,     ail_chi_deg, chi_deg_ini,     rud_beta_deg, beta_deg, trgg_t_sec,      t_sec_turn));
            Pguid->add_op(1, new control::guid_op(thr_vtas_mps, vtas_mps_ini, elv_Hp_m,         Hp_m_ini,     ail_xi_deg,  xi_deg,          rud_beta_deg, beta_deg, trgg_chi_deg,    chi_deg_end_trg));
            Pguid->add_op(2, new control::guid_op(thr_vtas_mps, vtas_mps_ini, elv_Hp_m,         Hp_m_ini,     ail_chi_deg, chi_deg_end_eff, rud_beta_deg, beta_deg, trgg_Deltat_sec, Deltat_sec_tas));
            Pguid->add_op(3, new control::guid_op(thr_vtas_mps, vtas_mps_end, elv_Hp_m,         Hp_m_ini,     ail_chi_deg, chi_deg_end_eff, rud_beta_deg, beta_deg, trgg_Deltat_sec, Deltat_sec_Hp));
            Pguid->add_op(4, new control::guid_op(thr_vtas_mps, vtas_mps_end, elv_gammaTAS_deg, gammaTAS_deg, ail_chi_deg, chi_deg_end_eff, rud_beta_deg, beta_deg, trgg_Hp_m,       Hp_m_end));
            Pguid->add_op(5, new control::guid_op(thr_vtas_mps, vtas_mps_end, elv_Hp_m,         Hp_m_end,     ail_chi_deg, chi_deg_end_eff, rud_beta_deg, beta_deg, trgg_t_sec,      t_sec_end));
            if (flag_console == true) {
                Pguid->create_text(std::cout);
            }

            return Pguid;
        }

        case 2: { // change of bearing only
            // ** With filter_air02 and new tas,oat,osp sensors with biases
            // 01 -> rotv [deg] +1.248e-01 +8.247e-02 w_nbb [dps] +1.110e-01 +5.312e-02 E_gyr [dps] +1.309e-03 +5.240e-04 E_mag [nT] +5.720e+00 +1.162e+00 error_mag [nT] +3.405e+01 +9.119e+00 x_n hor [m] +1.256e+03 +5.674e+02 x_n ver [m] -3.090e+00 +2.006e+00 v_n [mps] +1.313e+00 +6.402e-01 f_ibb [mps2] +1.690e-02 +7.617e-03 E_acc [mps2] +1.927e-02 +9.261e-03
            // 02 -> rotv [deg] +1.585e-01 +1.169e-01 w_nbb [dps] +1.077e-01 +5.224e-02 E_gyr [dps] +1.274e-03 +5.366e-04 E_mag [nT] +3.210e+01 +7.213e+00 error_mag [nT] +8.123e+00 +3.808e+00 x_n hor [m] +2.500e+03 +2.094e+03 x_n ver [m] -1.395e+01 +9.240e+00 v_n [mps] +1.892e+00 +7.396e-01 f_ibb [mps2] +1.626e-02 +7.615e-03 E_acc [mps2] +1.864e-02 +9.453e-03
            // 03 -> rotv [deg] +4.210e-01 +1.789e-01 w_nbb [dps] +1.217e-01 +5.386e-02 E_gyr [dps] +1.352e-03 +6.310e-04 E_mag [nT] +6.215e+01 +1.381e+01 error_mag [nT] +3.730e+01 +1.429e+01 x_n hor [m] +2.471e+03 +1.497e+03 x_n ver [m] -6.818e+00 +6.994e+00 v_n [mps] +1.496e+00 +4.552e-01 f_ibb [mps2] +1.639e-02 +7.626e-03 E_acc [mps2] +1.883e-02 +9.415e-03

            double t_sec_end = 1000.0;
            control::guid* Pguid = new control::guid(3, t_sec_end, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid);
            Pguid->_sti_id = st::logic::sti_HpXX_tasXX_chiXX;

            double beta_deg = 0.0;

            double chi_deg_ini = Pguid->_dist_uniform_bearing(Pguid->_gen);
            double chi_deg_end_eff, chi_deg_end_trg, angle_diff_deg, xi_deg;
            do { // ensure that there is a direction change of at least 10 [deg]
                chi_deg_end_eff = Pguid->_dist_uniform_bearing(Pguid->_gen);
                angle_diff_deg = ang::tools::angle_diff_deg(chi_deg_end_eff, chi_deg_ini);
            } while (std::fabs(angle_diff_deg) < 10.0);
            // turn direction depends on initial and final bearings
            // turning end 5 [deg] earlier for better control
            if (angle_diff_deg > 0) {
                xi_deg          = +10.0;
                chi_deg_end_trg = chi_deg_end_eff - 5.;
            }
            else {
                xi_deg          = -10.0;
                chi_deg_end_trg = chi_deg_end_eff + 5.;
            }
            ang::tools::correct_yaw_deg(chi_deg_end_trg);

            double Deltat_sec_tas;
            do {
                Deltat_sec_tas = 500.0 + 100.0 * Pguid->_dist_normal(Pguid->_gen);
            } while (Deltat_sec_tas < 150.0);

            double vtas_mps_ini;
            do {
                vtas_mps_ini = 29.0 + 1.5 * Pguid->_dist_normal(Pguid->_gen);
            } while ((vtas_mps_ini > 34.0) || (vtas_mps_ini < 24.0));
            double vtas_mps_end, vtas_mps_diff;
            do {
                vtas_mps_diff = 1.5 * Pguid->_dist_normal(Pguid->_gen);
                vtas_mps_end  = vtas_mps_ini + vtas_mps_diff;

            } while ((std::fabs(vtas_mps_diff) < 0.5) || (vtas_mps_end > 34.0) || (vtas_mps_end < 24.0));

            double Deltat_sec_Hp;
            do {
                Deltat_sec_Hp = 500.0 + 100.0 * Pguid->_dist_normal(Pguid->_gen);
            } while ((Deltat_sec_Hp < 150.0) || ((Deltat_sec_tas + Deltat_sec_Hp) > 2500.0));
            double Hp_m_ini = 2700.0 + 200.0 * Pguid->_dist_normal(Pguid->_gen);
            double Hp_m_end, Hp_m_diff, gammaTAS_deg;
            do { // ensure that there is a pressure altitude change of at least 100 [m]
                Hp_m_diff = 300.0 * Pguid->_dist_normal(Pguid->_gen);
            } while (std::fabs(Hp_m_diff) < 100.0);
            Hp_m_end = Hp_m_ini + Hp_m_diff;
            // path angle depends on initial and final pressure altitudes
            if (Hp_m_diff > 0) {gammaTAS_deg = +2.0;}
            else               {gammaTAS_deg = -2.0;}

            double t_sec_turn;
            do {
                t_sec_turn = t_sec_gpsloss + 30.0 + 50.0 * Pguid->_dist_normal(Pguid->_gen);
            } while (t_sec_turn < (t_sec_gpsloss + 15.0));

            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, vtas_mps_ini, elv_Hp_m,         Hp_m_ini,     ail_chi_deg, chi_deg_ini,     rud_beta_deg, beta_deg, trgg_t_sec,      t_sec_turn));
            Pguid->add_op(1, new control::guid_op(thr_vtas_mps, vtas_mps_ini, elv_Hp_m,         Hp_m_ini,     ail_xi_deg,  xi_deg,          rud_beta_deg, beta_deg, trgg_chi_deg,    chi_deg_end_trg));
            Pguid->add_op(2, new control::guid_op(thr_vtas_mps, vtas_mps_ini, elv_Hp_m,         Hp_m_ini,     ail_chi_deg, chi_deg_end_eff, rud_beta_deg, beta_deg, trgg_t_sec,      t_sec_end));
            if (flag_console == true) {
                Pguid->create_text(std::cout);
            }

            return Pguid;
        }

        case 3: { // six bearing changes at constant airspeed and pressure altitude
            // dummy variables are necessary to maintain commonality with intent #1

            double t_sec_end = 500.0;
            control::guid* Pguid = new control::guid(17, t_sec_end, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid);
            Pguid->_sti_id = st::logic::sti_HpXX_tasXX_chiXX;

            double beta_deg = 0.0;
            double angle_diff_deg;

            double chi_deg_0 = Pguid->_dist_uniform_bearing(Pguid->_gen);

            double chi_deg_1_eff, chi_deg_1_trg, xi_deg_1;
            do { // ensure that there is a direction change of at least 10 [deg]
                chi_deg_1_eff = Pguid->_dist_uniform_bearing(Pguid->_gen);
                angle_diff_deg = ang::tools::angle_diff_deg(chi_deg_1_eff, chi_deg_0);
            } while (std::fabs(angle_diff_deg) < 10.0);
            // turn direction depends on initial and final bearings
            // turning end 5 [deg] earlier for better control
            if (angle_diff_deg > 0) {
                xi_deg_1      = +10.0;
                chi_deg_1_trg = chi_deg_1_eff - 5.;
            }
            else {
                xi_deg_1      = -10.0;
                chi_deg_1_trg = chi_deg_1_eff + 5.;
            }
            ang::tools::correct_yaw_deg(chi_deg_1_trg);

            double Deltat_sec_tas_dummy;
            do {
                Deltat_sec_tas_dummy = 500.0 + 100.0 * Pguid->_dist_normal(Pguid->_gen);
            } while (Deltat_sec_tas_dummy < 150.0);

            double vtas_mps_ini;
            do {
                vtas_mps_ini = 29.0 + 1.5 * Pguid->_dist_normal(Pguid->_gen);
            } while ((vtas_mps_ini > 34.0) || (vtas_mps_ini < 24.0));
            double vtas_mps_end_dummy, vtas_mps_diff_dummy;
            do {
                vtas_mps_diff_dummy = 1.5 * Pguid->_dist_normal(Pguid->_gen);
                vtas_mps_end_dummy  = vtas_mps_ini + vtas_mps_diff_dummy;

            } while ((std::fabs(vtas_mps_diff_dummy) < 0.5) || (vtas_mps_end_dummy > 34.0) || (vtas_mps_end_dummy < 24.0));

            double Deltat_sec_Hp_dummy;
            do {
                Deltat_sec_Hp_dummy = 500.0 + 100.0 * Pguid->_dist_normal(Pguid->_gen);
            } while ((Deltat_sec_Hp_dummy < 150.0) || ((Deltat_sec_tas_dummy + Deltat_sec_Hp_dummy) > 2500.0));

            double Hp_m_ini = 2700.0 + 200.0 * Pguid->_dist_normal(Pguid->_gen);
            double Hp_m_end_dummy, Hp_m_diff_dummy;
            do { // ensure that there is a pressure altitude change of at least 100 [m]
                Hp_m_diff_dummy = 300.0 * Pguid->_dist_normal(Pguid->_gen);
            } while (std::fabs(Hp_m_diff_dummy) < 100.0);
            Hp_m_end_dummy = Hp_m_ini + Hp_m_diff_dummy;

            double t_sec_turn_1;
            do {
                t_sec_turn_1 = t_sec_gpsloss + 30.0 + 50.0 * Pguid->_dist_normal(Pguid->_gen);
            } while (t_sec_turn_1 < (t_sec_gpsloss + 15.0));

            // up this point I have the same calls to distributions as guidance intent #1
            double Deltat_sec_turn_2 = Pguid->_dist_uniform_time(Pguid->_gen);
            double Deltat_sec_turn_3 = Pguid->_dist_uniform_time(Pguid->_gen);
            double Deltat_sec_turn_4 = Pguid->_dist_uniform_time(Pguid->_gen);
            double Deltat_sec_turn_5 = Pguid->_dist_uniform_time(Pguid->_gen);
            double Deltat_sec_turn_6 = Pguid->_dist_uniform_time(Pguid->_gen);
            double Deltat_sec_turn_7 = Pguid->_dist_uniform_time(Pguid->_gen);
            double Deltat_sec_turn_8 = Pguid->_dist_uniform_time(Pguid->_gen);

            double chi_deg_2_eff, chi_deg_2_trg, xi_deg_2;
            do { // ensure that there is a direction change of at least 10 [deg]
                chi_deg_2_eff = Pguid->_dist_uniform_bearing(Pguid->_gen);
                angle_diff_deg = ang::tools::angle_diff_deg(chi_deg_2_eff, chi_deg_1_eff);
            } while (std::fabs(angle_diff_deg) < 10.0);
            if (angle_diff_deg > 0) {
                xi_deg_2      = +10.0;
                chi_deg_2_trg = chi_deg_2_eff - 5.;
            }
            else {
                xi_deg_2      = -10.0;
                chi_deg_2_trg = chi_deg_2_eff + 5.;
            }
            ang::tools::correct_yaw_deg(chi_deg_2_trg);

            double chi_deg_3_eff, chi_deg_3_trg, xi_deg_3;
            do { // ensure that there is a direction change of at least 10 [deg]
                chi_deg_3_eff = Pguid->_dist_uniform_bearing(Pguid->_gen);
                angle_diff_deg = ang::tools::angle_diff_deg(chi_deg_3_eff, chi_deg_2_eff);
            } while (std::fabs(angle_diff_deg) < 10.0);
            if (angle_diff_deg > 0) {
                xi_deg_3      = +10.0;
                chi_deg_3_trg = chi_deg_3_eff - 5.;
            }
            else {
                xi_deg_3      = -10.0;
                chi_deg_3_trg = chi_deg_3_eff + 5.;
            }
            ang::tools::correct_yaw_deg(chi_deg_3_trg);

            double chi_deg_4_eff, chi_deg_4_trg, xi_deg_4;
            do { // ensure that there is a direction change of at least 10 [deg]
                chi_deg_4_eff = Pguid->_dist_uniform_bearing(Pguid->_gen);
                angle_diff_deg = ang::tools::angle_diff_deg(chi_deg_4_eff, chi_deg_3_eff);
            } while (std::fabs(angle_diff_deg) < 10.0);
            if (angle_diff_deg > 0) {
                xi_deg_4      = +10.0;
                chi_deg_4_trg = chi_deg_4_eff - 5.;
            }
            else {
                xi_deg_4      = -10.0;
                chi_deg_4_trg = chi_deg_4_eff + 5.;
            }
            ang::tools::correct_yaw_deg(chi_deg_4_trg);

            double chi_deg_5_eff, chi_deg_5_trg, xi_deg_5;
            do { // ensure that there is a direction change of at least 10 [deg]
                chi_deg_5_eff = Pguid->_dist_uniform_bearing(Pguid->_gen);
                angle_diff_deg = ang::tools::angle_diff_deg(chi_deg_5_eff, chi_deg_4_eff);
            } while (std::fabs(angle_diff_deg) < 10.0);
            if (angle_diff_deg > 0) {
                xi_deg_5      = +10.0;
                chi_deg_5_trg = chi_deg_5_eff - 5.;
            }
            else {
                xi_deg_5      = -10.0;
                chi_deg_5_trg = chi_deg_5_eff + 5.;
            }
            ang::tools::correct_yaw_deg(chi_deg_5_trg);

            double chi_deg_6_eff, chi_deg_6_trg, xi_deg_6;
            do { // ensure that there is a direction change of at least 10 [deg]
                chi_deg_6_eff = Pguid->_dist_uniform_bearing(Pguid->_gen);
                angle_diff_deg = ang::tools::angle_diff_deg(chi_deg_6_eff, chi_deg_5_eff);
            } while (std::fabs(angle_diff_deg) < 10.0);
            if (angle_diff_deg > 0) {
                xi_deg_6      = +10.0;
                chi_deg_6_trg = chi_deg_6_eff - 5.;
            }
            else {
                xi_deg_6      = -10.0;
                chi_deg_6_trg = chi_deg_6_eff + 5.;
            }
            ang::tools::correct_yaw_deg(chi_deg_6_trg);

            double chi_deg_7_eff, chi_deg_7_trg, xi_deg_7;
            do { // ensure that there is a direction change of at least 10 [deg]
                chi_deg_7_eff = Pguid->_dist_uniform_bearing(Pguid->_gen);
                angle_diff_deg = ang::tools::angle_diff_deg(chi_deg_7_eff, chi_deg_6_eff);
            } while (std::fabs(angle_diff_deg) < 10.0);
            if (angle_diff_deg > 0) {
                xi_deg_7      = +10.0;
                chi_deg_7_trg = chi_deg_7_eff - 5.;
            }
            else {
                xi_deg_7      = -10.0;
                chi_deg_7_trg = chi_deg_7_eff + 5.;
            }
            ang::tools::correct_yaw_deg(chi_deg_7_trg);

            double chi_deg_8_eff, chi_deg_8_trg, xi_deg_8;
            do { // ensure that there is a direction change of at least 10 [deg]
                chi_deg_8_eff = Pguid->_dist_uniform_bearing(Pguid->_gen);
                angle_diff_deg = ang::tools::angle_diff_deg(chi_deg_8_eff, chi_deg_7_eff);
            } while (std::fabs(angle_diff_deg) < 10.0);
            if (angle_diff_deg > 0) {
                xi_deg_8      = +10.0;
                chi_deg_8_trg = chi_deg_8_eff - 5.;
            }
            else {
                xi_deg_8      = -10.0;
                chi_deg_8_trg = chi_deg_8_eff + 5.;
            }
            ang::tools::correct_yaw_deg(chi_deg_8_trg);

            Pguid->add_op(0,  new control::guid_op(thr_vtas_mps, vtas_mps_ini, elv_Hp_m, Hp_m_ini, ail_chi_deg, chi_deg_0,     rud_beta_deg, beta_deg, trgg_t_sec,      t_sec_turn_1));
            Pguid->add_op(1,  new control::guid_op(thr_vtas_mps, vtas_mps_ini, elv_Hp_m, Hp_m_ini, ail_xi_deg,  xi_deg_1,      rud_beta_deg, beta_deg, trgg_chi_deg,    chi_deg_1_trg));
            Pguid->add_op(2,  new control::guid_op(thr_vtas_mps, vtas_mps_ini, elv_Hp_m, Hp_m_ini, ail_chi_deg, chi_deg_1_eff, rud_beta_deg, beta_deg, trgg_Deltat_sec, Deltat_sec_turn_2));
            Pguid->add_op(3,  new control::guid_op(thr_vtas_mps, vtas_mps_ini, elv_Hp_m, Hp_m_ini, ail_xi_deg,  xi_deg_2,      rud_beta_deg, beta_deg, trgg_chi_deg,    chi_deg_2_trg));
            Pguid->add_op(4,  new control::guid_op(thr_vtas_mps, vtas_mps_ini, elv_Hp_m, Hp_m_ini, ail_chi_deg, chi_deg_2_eff, rud_beta_deg, beta_deg, trgg_Deltat_sec, Deltat_sec_turn_3));
            Pguid->add_op(5,  new control::guid_op(thr_vtas_mps, vtas_mps_ini, elv_Hp_m, Hp_m_ini, ail_xi_deg,  xi_deg_3,      rud_beta_deg, beta_deg, trgg_chi_deg,    chi_deg_3_trg));
            Pguid->add_op(6,  new control::guid_op(thr_vtas_mps, vtas_mps_ini, elv_Hp_m, Hp_m_ini, ail_chi_deg, chi_deg_3_eff, rud_beta_deg, beta_deg, trgg_Deltat_sec, Deltat_sec_turn_4));
            Pguid->add_op(7,  new control::guid_op(thr_vtas_mps, vtas_mps_ini, elv_Hp_m, Hp_m_ini, ail_xi_deg,  xi_deg_4,      rud_beta_deg, beta_deg, trgg_chi_deg,    chi_deg_4_trg));
            Pguid->add_op(8,  new control::guid_op(thr_vtas_mps, vtas_mps_ini, elv_Hp_m, Hp_m_ini, ail_chi_deg, chi_deg_4_eff, rud_beta_deg, beta_deg, trgg_Deltat_sec, Deltat_sec_turn_5));
            Pguid->add_op(9,  new control::guid_op(thr_vtas_mps, vtas_mps_ini, elv_Hp_m, Hp_m_ini, ail_xi_deg,  xi_deg_5,      rud_beta_deg, beta_deg, trgg_chi_deg,    chi_deg_5_trg));
            Pguid->add_op(10, new control::guid_op(thr_vtas_mps, vtas_mps_ini, elv_Hp_m, Hp_m_ini, ail_chi_deg, chi_deg_5_eff, rud_beta_deg, beta_deg, trgg_Deltat_sec, Deltat_sec_turn_6));
            Pguid->add_op(11, new control::guid_op(thr_vtas_mps, vtas_mps_ini, elv_Hp_m, Hp_m_ini, ail_xi_deg,  xi_deg_6,      rud_beta_deg, beta_deg, trgg_chi_deg,    chi_deg_6_trg));
            Pguid->add_op(12, new control::guid_op(thr_vtas_mps, vtas_mps_ini, elv_Hp_m, Hp_m_ini, ail_chi_deg, chi_deg_6_eff, rud_beta_deg, beta_deg, trgg_Deltat_sec, Deltat_sec_turn_7));
            Pguid->add_op(13, new control::guid_op(thr_vtas_mps, vtas_mps_ini, elv_Hp_m, Hp_m_ini, ail_xi_deg,  xi_deg_7,      rud_beta_deg, beta_deg, trgg_chi_deg,    chi_deg_7_trg));
            Pguid->add_op(14, new control::guid_op(thr_vtas_mps, vtas_mps_ini, elv_Hp_m, Hp_m_ini, ail_chi_deg, chi_deg_7_eff, rud_beta_deg, beta_deg, trgg_Deltat_sec, Deltat_sec_turn_8));
            Pguid->add_op(15, new control::guid_op(thr_vtas_mps, vtas_mps_ini, elv_Hp_m, Hp_m_ini, ail_xi_deg,  xi_deg_8,      rud_beta_deg, beta_deg, trgg_chi_deg,    chi_deg_8_trg));
            Pguid->add_op(16, new control::guid_op(thr_vtas_mps, vtas_mps_ini, elv_Hp_m, Hp_m_ini, ail_chi_deg, chi_deg_8_eff, rud_beta_deg, beta_deg, trgg_t_sec,      t_sec_end));

            if (flag_console == true) {
                Pguid->create_text(std::cout);
            }

            return Pguid;
        }

        case 9: { // change of body heading, airspeed, and pressure altitude
            // 01 -> rotv [deg] +1.064e-01 +7.201e-02 w_nbb [dps] +1.049e-01 +5.337e-02 E_gyr [dps] +1.311e-03 +5.287e-04 E_mag [nT] +6.256e+00 +1.110e+00 error_mag [nT] +2.808e+01 +5.453e+00 x_n hor [m] +1.177e+03 +5.424e+02 x_n ver [m] -3.089e+00 +2.006e+00 v_n [mps] +1.299e+00 +6.411e-01 f_ibb [mps2] +1.690e-02 +7.617e-03 E_acc [mps2] +1.927e-02 +9.261e-03
            // 02 -> rotv [deg] +1.223e-01 +1.010e-01 w_nbb [dps] +1.025e-01 +5.176e-02 E_gyr [dps] +1.270e-03 +5.296e-04 E_mag [nT] +2.894e+01 +5.628e+00 error_mag [nT] +6.317e+00 +2.781e+00 x_n hor [m] +2.535e+03 +2.117e+03 x_n ver [m] -1.395e+01 +9.240e+00 v_n [mps] +1.910e+00 +7.441e-01 f_ibb [mps2] +1.627e-02 +7.615e-03 E_acc [mps2] +1.864e-02 +9.452e-03
            // 03 -> rotv [deg] +2.789e-01 +1.505e-01 w_nbb [dps] +1.008e-01 +4.961e-02 E_gyr [dps] +1.326e-03 +6.084e-04 E_mag [nT] +6.424e+01 +1.382e+01 error_mag [nT] +2.416e+01 +6.345e+00 x_n hor [m] +2.576e+03 +1.567e+03 x_n ver [m] -6.817e+00 +6.994e+00 v_n [mps] +1.546e+00 +4.595e-01 f_ibb [mps2] +1.639e-02 +7.626e-03 E_acc [mps2] +1.883e-02 +9.417e-03


            double t_sec_end = 3800.0;
            auto Pguid = new control::guid(6, t_sec_end, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid);
            Pguid->_sti_id = st::logic::sti_HpXX_tasXX_chiXX;

            double beta_deg = 0.0;

            double psi_deg_ini = Pguid->_dist_uniform_bearing(Pguid->_gen);
            double psi_deg_end_eff, psi_deg_end_trg, angle_diff_deg, xi_deg;
            do { // ensure that there is a direction change of at least 10 [deg]
                psi_deg_end_eff = Pguid->_dist_uniform_bearing(Pguid->_gen);
                angle_diff_deg = ang::tools::angle_diff_deg(psi_deg_end_eff, psi_deg_ini);
            } while (std::fabs(angle_diff_deg) < 10.0);
            // turn direction depends on initial and final bearings
            // turning end 5 [deg] earlier for better control
            if (angle_diff_deg > 0) {
                xi_deg          = +10.0;
                psi_deg_end_trg = psi_deg_end_eff - 5.;
            }
            else {
                xi_deg          = -10.0;
                psi_deg_end_trg = psi_deg_end_eff + 5.;
            }
            ang::tools::correct_yaw_deg(psi_deg_end_trg);

            double Deltat_sec_tas;
            do {
                Deltat_sec_tas = 500.0 + 100.0 * Pguid->_dist_normal(Pguid->_gen);
            } while (Deltat_sec_tas < 150.0);

            double vtas_mps_ini;
            do {
                vtas_mps_ini = 29.0 + 1.5 * Pguid->_dist_normal(Pguid->_gen);
            } while ((vtas_mps_ini > 34.0) || (vtas_mps_ini < 24.0));
            double vtas_mps_end, vtas_mps_diff;
            do {
                vtas_mps_diff = 1.5 * Pguid->_dist_normal(Pguid->_gen);
                vtas_mps_end  = vtas_mps_ini + vtas_mps_diff;

            } while ((std::fabs(vtas_mps_diff) < 0.5) || (vtas_mps_end > 34.0) || (vtas_mps_end < 24.0));

            double Deltat_sec_Hp;
            do {
                Deltat_sec_Hp = 500.0 + 100.0 * Pguid->_dist_normal(Pguid->_gen);
            } while ((Deltat_sec_Hp < 150.0) || ((Deltat_sec_tas + Deltat_sec_Hp) > 2500.0));
            double Hp_m_ini = 2700.0 + 200.0 * Pguid->_dist_normal(Pguid->_gen);
            double Hp_m_end, Hp_m_diff, gammaTAS_deg;
            do { // ensure that there is a pressure altitude change of at least 100 [m]
                Hp_m_diff = 300.0 * Pguid->_dist_normal(Pguid->_gen);
            } while (std::fabs(Hp_m_diff) < 100.0);
            Hp_m_end = Hp_m_ini + Hp_m_diff;
            // path angle depends on initial and final pressure altitudes
            if (Hp_m_diff > 0) {gammaTAS_deg = +2.0;}
            else               {gammaTAS_deg = -2.0;}

            double t_sec_turn;
            do {
                t_sec_turn = t_sec_gpsloss + 30.0 + 50.0 * Pguid->_dist_normal(Pguid->_gen);
            } while (t_sec_turn < (t_sec_gpsloss + 15.0));

            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, vtas_mps_ini, elv_Hp_m,         Hp_m_ini,     ail_psi_deg, psi_deg_ini,     rud_beta_deg, beta_deg, trgg_t_sec,      t_sec_turn));
            Pguid->add_op(1, new control::guid_op(thr_vtas_mps, vtas_mps_ini, elv_Hp_m,         Hp_m_ini,     ail_xi_deg,  xi_deg,          rud_beta_deg, beta_deg, trgg_chi_deg,    psi_deg_end_trg));
            Pguid->add_op(2, new control::guid_op(thr_vtas_mps, vtas_mps_ini, elv_Hp_m,         Hp_m_ini,     ail_psi_deg, psi_deg_end_eff, rud_beta_deg, beta_deg, trgg_Deltat_sec, Deltat_sec_tas));
            Pguid->add_op(3, new control::guid_op(thr_vtas_mps, vtas_mps_end, elv_Hp_m,         Hp_m_ini,     ail_psi_deg, psi_deg_end_eff, rud_beta_deg, beta_deg, trgg_Deltat_sec, Deltat_sec_Hp));
            Pguid->add_op(4, new control::guid_op(thr_vtas_mps, vtas_mps_end, elv_gammaTAS_deg, gammaTAS_deg, ail_psi_deg, psi_deg_end_eff, rud_beta_deg, beta_deg, trgg_Hp_m,       Hp_m_end));
            Pguid->add_op(5, new control::guid_op(thr_vtas_mps, vtas_mps_end, elv_Hp_m,         Hp_m_end,     ail_psi_deg, psi_deg_end_eff, rud_beta_deg, beta_deg, trgg_t_sec,      t_sec_end));
            Pguid->create_text(std::cout);

            return Pguid;
        }
        case 11: {
            // No wind
            // rotv [deg] +1.404e-01 +8.915e-02 w_nbb [dps] +2.688e-01 +1.832e-01 E_gyr [dps] +2.141e-03 +6.704e-04 E_mag [nT] +4.030e+00 +5.105e-01 error_mag [nT] +2.473e+01 +1.347e+00
            // f_ibb [mps2] +1.680e-02 +7.899e-03 E_acc [mps2] +1.859e-02 +9.673e-03
            // Wind 0-5
            // rotv [deg] +4.985e-01 +2.458e-01 w_nbb [dps] +2.692e-01 +1.831e-01 E_gyr [dps] +3.454e-03 +9.176e-04 E_mag [nT] +4.128e+00 +5.111e-01 error_mag [nT] +2.518e+01 +2.232e+00
            // f_ibb [mps2] +1.705e-02 +8.005e-03 E_acc [mps2] +1.881e-02 +9.771e-03

            // rotv [deg] +1.402e-01 +8.908e-02 w_nbb [dps] +2.688e-01 +1.832e-01 E_gyr [dps] +2.145e-03 +6.720e-04 E_mag [nT] +4.031e+00 +5.104e-01 error_mag [nT] +2.472e+01 +1.345e+00
            // f_ibb [mps2] +1.679e-02 +7.898e-03 E_acc [mps2] +1.859e-02 +9.673e-03

            auto Pguid = new control::guid(1, 200, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid); // 200
            Pguid->_sti_id = st::logic::sti_Hp3000_tas30_chi00;
            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 0.0, rud_beta_deg, 0., trgg_t_sec, 200.0));
            return Pguid;
        }

        case 12: {
            // A --> No wind
            // rotv [deg] +1.368e-01 +8.222e-02 w_nbb [dps] +2.472e-01 +1.843e-01 E_gyr [dps] +2.127e-03 +5.200e-04 E_mag [nT] +4.336e+00 +2.713e-01
            // error_mag [nT] +2.399e+01 +9.630e-01 f_ibb [mps2] +1.669e-02 +7.867e-03 E_acc [mps2] +1.860e-02 +9.656e-03
            // vtas [mps] +30.000 vlf [mps] +0.000 vhf [mps] +0.000 v [mps] +30.000
            // psi [deg] +0.000 theta [deg] +0.000 xi [deg] +0.000 chi [deg] +0.000 gamma [deg] +0.200 alpha [deg] -0.200 beta [deg] +0.000
            // vtas b i [mps] +30.000 vtas b ii [mps] +0.000 vtas b iii [mps] -0.105
            // v b i [mps]    +30.000 v b ii [mps]    +0.000 v b iii [mps]    -0.105
            // v n i [mps]    +30.000 v n ii [mps]    +0.000 v n iii [mps]    -0.105
            //
            // B --> wind 0-5
            // rotv [deg] +5.318e-01 +2.767e-01 w_nbb [dps] +2.472e-01 +1.842e-01 E_gyr [dps] +3.603e-03 +8.697e-04 E_mag [nT] +4.414e+00 +2.238e-01
            // error_mag [nT] +2.577e+01 +2.202e+00 f_ibb [mps2] +1.695e-02 +7.982e-03 E_acc [mps2] +1.882e-02 +9.766e-03
            // vtas [mps] +30.000 vlf [mps] +5.000 vhf [mps] +0.000 v [mps] +30.414
            // psi [deg] +0.000 theta [deg] +0.000 xi [deg] +0.000 chi [deg] +9.462 gamma [deg] +0.197 alpha [deg] -0.200 beta [deg] +0.000
            // vtas b i [mps] +30.000 vtas b ii [mps] +0.000 vtas b iii [mps] -0.105
            // v b i [mps]    +30.000 v b ii [mps]    +5.000 v b iii [mps]    -0.105
            // v n i [mps]    +30.000 v n ii [mps]    +5.000 v n iii [mps]    -0.105
            //
            // NEW FILTER wind 0-5
            // rotv [deg] +1.266e-01 +7.524e-02 w_nbb [dps] +2.472e-01 +1.842e-01 E_gyr [dps] +2.095e-03 +5.125e-04 E_mag [nT] +4.336e+00 +2.732e-01
            // error_mag [nT] +2.396e+01 +9.308e-01 f_ibb [mps2] +1.669e-02 +7.865e-03 E_acc [mps2] +1.859e-02 +9.655e-03

            auto Pguid = new control::guid(1, 200, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid); // 200
            Pguid->_sti_id = st::logic::sti_Hp3000_tas30_psi00;
            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_psi_deg, 0.0, rud_beta_deg, 0., trgg_t_sec, 200.0));
            return Pguid;
        }


        case 21: { // short straight flight (similar to #31 but now random) sti now includes turbulence, new att filter
            // ** 12: No wind, no offsets, turb 0.5
            // 31 -> rotv [deg] +1.506e-01 +9.661e-02 w_nbb [dps] +2.687e-01 +1.832e-01 E_gyr [dps] +2.194e-03 +6.556e-04 E_mag [nT] +4.073e+00 +4.474e-01 error_mag [nT] +2.738e+01 +1.220e+00
            // 01 -> rotv [deg] +7.579e-02 +5.211e-02 w_nbb [dps] +2.688e-01 +1.831e-01 E_gyr [dps] +1.721e-03 +5.640e-04 E_mag [nT] +4.558e+00 +5.478e-01 error_mag [nT] +1.977e+01 +1.650e+00
            // 02 -> rotv [deg] +1.566e-01 +1.233e-01 w_nbb [dps] +2.557e-01 +1.762e-01 E_gyr [dps] +1.550e-03 +4.741e-04 E_mag [nT] +1.065e+01 +7.092e-01 error_mag [nT] +2.266e+00 +2.119e-01
            // 03 -> rotv [deg] +2.265e-01 +9.127e-02 w_nbb [dps] +2.539e-01 +1.697e-01 E_gyr [dps] +1.258e-03 +5.679e-04 E_mag [nT] +7.759e+00 +8.577e-01 error_mag [nT] +2.042e+01 +7.451e-01
            // 04 ->
            // 05 ->
            // ** 13: No wind, offsets constant but random, turb 0.5
            // 01 -> rotv [deg] +7.580e-02 +5.212e-02 w_nbb [dps] +2.686e-01 +1.830e-01 E_gyr [dps] +1.721e-03 +5.640e-04 E_mag [nT] +4.561e+00 +5.488e-01 error_mag [nT] +1.978e+01 +1.651e+00
            // 02 -> rotv [deg] +1.560e-01 +1.231e-01 w_nbb [dps] +2.564e-01 +1.767e-01 E_gyr [dps] +1.549e-03 +4.742e-04 E_mag [nT] +1.062e+01 +7.065e-01 error_mag [nT] +2.267e+00 +2.115e-01
            // 03 -> rotv [deg] +2.283e-01 +9.165e-02 w_nbb [dps] +2.523e-01 +1.684e-01 E_gyr [dps] +1.264e-03 +5.699e-04 E_mag [nT] +7.876e+00 +8.780e-01 error_mag [nT] +2.042e+01 +7.446e-01
            // ** 14: Wind constant but random, offsets constant but random, turb 0.5
            // 01 -> rotv [deg] +7.485e-02 +5.561e-02 w_nbb [dps] +2.686e-01 +1.822e-01 E_gyr [dps] +1.691e-03 +5.801e-04 E_mag [nT] +4.740e+00 +5.638e-01 error_mag [nT] +1.856e+01 +1.788e+00
            // 02 -> rotv [deg] +1.533e-01 +1.227e-01 w_nbb [dps] +2.564e-01 +1.767e-01 E_gyr [dps] +1.539e-03 +4.745e-04 E_mag [nT] +1.060e+01 +7.035e-01 error_mag [nT] +2.293e+00 +2.046e-01
            // 03 -> rotv [deg] +2.184e-01 +9.135e-02 w_nbb [dps] +2.522e-01 +1.682e-01 E_gyr [dps] +1.225e-03 +5.554e-04 E_mag [nT] +7.981e+00 +8.421e-01 error_mag [nT] +2.074e+01 +7.395e-01
            // 04 -> rotv [deg] +6.941e-02 +4.399e-02 w_nbb [dps] +2.664e-01 +1.818e-01 E_gyr [dps] +8.104e-04 +2.883e-04 E_mag [nT] +9.114e+00 +1.192e+00 error_mag [nT] +7.458e+00 +1.777e+00
            // 05 -> rotv [deg] +1.227e-01 +6.189e-02 w_nbb [dps] +2.593e-01 +1.785e-01 E_gyr [dps] +9.916e-04 +5.051e-04 E_mag [nT] +7.029e+00 +3.834e-01 error_mag [nT] +2.328e+01 +9.958e-01

            auto Pguid = new control::guid(1, 200, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid);
            Pguid->_sti_id = st::logic::sti_Hp3000_tas30_chiXX;
            double chi_deg_init = Pguid->_dist_uniform_bearing(Pguid->_gen);
            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, chi_deg_init, rud_beta_deg, 0., trgg_t_sec, 200.0));
            return Pguid;
        }

        case 22: { // long straight flight (similar to #32 but now random) sti now includes turbulence, new att filter
            // ** 12: No wind, no offsets, turb 0.5
            // 32 -> rotv [deg] +2.549e-01 +1.104e-01 w_nbb [dps] +2.645e-01 +1.805e-01 E_gyr [dps] +1.538e-03 +6.985e-04 E_mag [nT] +3.281e+00 +6.385e-01 error_mag [nT] +3.627e+01 +7.045e+00
            // 01 -> rotv [deg] +9.988e-02 +6.736e-02 w_nbb [dps] +2.644e-01 +1.804e-01 E_gyr [dps] +1.315e-03 +5.357e-04 E_mag [nT] +2.887e+00 +1.246e+00 error_mag [nT] +2.919e+01 +7.288e+00
            // 02 -> rotv [deg] +1.238e-01 +9.539e-02 w_nbb [dps] +2.585e-01 +1.764e-01 E_gyr [dps] +1.326e-03 +5.363e-04 E_mag [nT] +1.639e+01 +4.915e+00 error_mag [nT] +4.144e+00 +1.596e+00
            // 03 -> rotv [deg] +1.589e-01 +1.110e-01 w_nbb [dps] +2.639e-01 +1.788e-01 E_gyr [dps] +1.189e-03 +5.479e-04 E_mag [nT] +7.948e+00 +8.791e-01 error_mag [nT] +2.561e+01 +5.026e+00
            // 04 ->
            // 05 ->
            // ** 14: Wind constant but random, offsets constant but random, turb 0.5
            // 01 -> rotv [deg] +1.037e-01 +6.944e-02 w_nbb [dps] +2.642e-01 +1.795e-01 E_gyr [dps] +1.312e-03 +5.327e-04 E_mag [nT] +3.049e+00 +1.363e+00 error_mag [nT] +2.861e+01 +7.784e+00
            // 02 -> rotv [deg] +1.217e-01 +9.433e-02 w_nbb [dps] +2.592e-01 +1.769e-01 E_gyr [dps] +1.324e-03 +5.359e-04 E_mag [nT] +1.634e+01 +4.917e+00 error_mag [nT] +4.153e+00 +1.585e+00
            // 03 -> rotv [deg] +1.544e-01 +1.097e-01 w_nbb [dps] +2.624e-01 +1.774e-01 E_gyr [dps] +1.183e-03 +5.455e-04 E_mag [nT] +8.184e+00 +8.741e-01 error_mag [nT] +2.595e+01 +5.028e+00
            // 04 -> rotv [deg] +8.996e-02 +6.178e-02 w_nbb [dps] +2.698e-01 +1.818e-01 E_gyr [dps] +1.213e-03 +4.622e-04 E_mag [nT] +1.027e+01 +1.252e+00 error_mag [nT] +4.545e+00 +2.309e+00
            // 05 -> rotv [deg] +1.029e-01 +5.973e-02 w_nbb [dps] +2.653e-01 +1.800e-01 E_gyr [dps] +1.212e-03 +5.214e-04 E_mag [nT] +5.644e+00 +9.909e-01 error_mag [nT] +2.888e+01 +4.801e+00

            auto Pguid = new control::guid(1, 800.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid);
            Pguid->_sti_id = st::logic::sti_Hp3000_tas30_chiXX;
            double chi_deg_init = Pguid->_dist_uniform_bearing(Pguid->_gen);
            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, chi_deg_init, rud_beta_deg, 0., trgg_t_sec, 800.0));
            return Pguid;
        }
        case 23: { // level & constant tas flight with three straight segments and two 180 [deg] turns at bank angle 10 [deg] (similar to #33 but now random)
            // sti now includes turbulence, new att filter
            // ** 12: No wind, no offsets, turb 0.5
            // 33 -> rotv [deg] +1.795e-01 +9.924e-02 w_nbb [dps] +2.794e-01 +2.179e-01 E_gyr [dps] +1.822e-03 +6.142e-04 E_mag [nT] +1.746e+01 +1.445e+01 error_mag [nT] +3.305e+01 +5.287e+00
            // 01 -> rotv [deg] +3.220e-01 +1.915e-01 w_nbb [dps] +2.792e-01 +2.176e-01 E_gyr [dps] +1.883e-03 +8.819e-04 E_mag [nT] +2.714e+01 +2.576e+01 error_mag [nT] +2.223e+01 +3.083e+00
            // 02 -> rotv [deg] +1.769e-01 +1.277e-01 w_nbb [dps] +2.646e-01 +2.110e-01 E_gyr [dps] +1.648e-03 +5.326e-04 E_mag [nT] +3.836e+01 +1.977e+01 error_mag [nT] +4.706e+00 +2.139e+00
            // 03 -> rotv [deg] +2.059e-01 +9.494e-02 w_nbb [dps] +2.717e-01 +2.145e-01 E_gyr [dps] +1.437e-03 +5.843e-04 E_mag [nT] +2.948e+01 +2.603e+01 error_mag [nT] +2.252e+01 +2.323e+00
            // 04 ->
            // 05 ->
            // ** 14: Wind constant but random, offsets constant but random, turb 0.5
            // 01 -> rotv [deg] +2.446e-01 +1.655e-01 w_nbb [dps] +2.793e-01 +2.165e-01 E_gyr [dps] +1.755e-03 +8.082e-04 E_mag [nT] +2.554e+01 +2.388e+01 error_mag [nT] +2.055e+01 +2.810e+00
            // 02 -> rotv [deg] +1.797e-01 +1.278e-01 w_nbb [dps] +2.656e-01 +2.123e-01 E_gyr [dps] +1.667e-03 +5.402e-04 E_mag [nT] +3.808e+01 +1.969e+01 error_mag [nT] +4.683e+00 +2.108e+00
            // 03 -> rotv [deg] +1.883e-01 +1.015e-01 w_nbb [dps] +2.704e-01 +2.141e-01 E_gyr [dps] +1.327e-03 +5.028e-04 E_mag [nT] +2.968e+01 +2.610e+01 error_mag [nT] +2.269e+01 +2.266e+00
            // 04 -> rotv [deg] +2.669e-01 +2.204e-01 w_nbb [dps] +2.797e-01 +2.219e-01 E_gyr [dps] +1.904e-03 +8.814e-04 E_mag [nT] +4.425e+01 +2.951e+01 error_mag [nT] +5.422e+00 +2.679e+00
            // 05 -> rotv [deg] +1.699e-01 +1.179e-01 w_nbb [dps] +2.745e-01 +2.167e-01 E_gyr [dps] +1.124e-03 +5.350e-04 E_mag [nT] +1.967e+01 +1.519e+01 error_mag [nT] +2.074e+01 +2.145e+00

            auto Pguid = new control::guid(5, 500.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid);
            Pguid->_sti_id = st::logic::sti_Hp3000_tas30_chiXX;
            double chi_deg_init = Pguid->_dist_uniform_bearing(Pguid->_gen);
            double chi_deg_mid  = chi_deg_init + 180.0;
            ang::tools::correct_yaw_deg(chi_deg_mid);
            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, chi_deg_init, rud_beta_deg, 0., trgg_t_sec,             100.0));
            Pguid->add_op(1, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,          10.0, rud_beta_deg, 0., trgg_chi_deg,     chi_deg_mid));
            Pguid->add_op(2, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg,  chi_deg_mid, rud_beta_deg, 0., trgg_Deltat_sec,        100.0));
            Pguid->add_op(3, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,          10.0, rud_beta_deg, 0., trgg_chi_deg,    chi_deg_init));
            Pguid->add_op(4, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, chi_deg_init, rud_beta_deg, 0., trgg_Deltat_sec,        100.0));
            return Pguid;
        }
        case 25: { // straight flight with two changes of altitude and one change of tas (similar to #35 but now random). sti now includes turbulence, new att filter
            // ** 12: No wind, no offsets, turb 0.5
            // 01 -> rotv [deg] +2.755e-01 +3.370e-01 w_nbb [dps] +1.960e-01 +1.592e-01 E_gyr [dps] +1.841e-03 +1.221e-03 E_mag [nT] +4.515e+00 +8.405e-01 error_mag [nT] +2.361e+01 +3.640e+00
            // 02 -> rotv [deg] +3.072e-01 +3.111e-01 w_nbb [dps] +1.954e-01 +1.525e-01 E_gyr [dps] +1.516e-03 +8.902e-04 E_mag [nT] +1.364e+01 +2.934e+00 error_mag [nT] +2.584e+00 +4.255e-01
            // 03 -> rotv [deg] +4.169e-01 +3.647e-01 w_nbb [dps] +2.001e-01 +1.592e-01 E_gyr [dps] +1.620e-03 +1.185e-03 E_mag [nT] +1.027e+01 +2.212e+00 error_mag [nT] +2.125e+01 +1.214e+00
            // 04 ->
            // 05 ->
            // ** 14: Wind constant but random, offsets constant but random, turb 0.5
            // 01 -> rotv [deg] +2.403e-01 +3.036e-01 w_nbb [dps] +1.972e-01 +1.584e-01 E_gyr [dps] +1.766e-03 +1.079e-03 E_mag [nT] +5.493e+00 +1.253e+00 error_mag [nT] +2.215e+01 +3.457e+00
            // 02 -> rotv [deg] +2.926e-01 +3.055e-01 w_nbb [dps] +1.952e-01 +1.501e-01 E_gyr [dps] +1.521e-03 +8.752e-04 E_mag [nT] +1.347e+01 +2.785e+00 error_mag [nT] +2.562e+00 +3.779e-01
            // 03 -> rotv [deg] +4.242e-01 +3.750e-01 w_nbb [dps] +1.986e-01 +1.575e-01 E_gyr [dps] +1.624e-03 +1.230e-03 E_mag [nT] +1.029e+01 +2.029e+00 error_mag [nT] +2.199e+01 +1.504e+00
            // 04 -> rotv [deg] +1.969e-01 +1.895e-01 w_nbb [dps] +2.030e-01 +1.569e-01 E_gyr [dps] +1.385e-03 +7.767e-04 E_mag [nT] +8.085e+00 +1.476e+00 error_mag [nT] +5.384e+00 +2.116e+00
            // 05 -> rotv [deg] +2.910e-01 +2.960e-01 w_nbb [dps] +1.977e-01 +1.546e-01 E_gyr [dps] +1.302e-03 +8.164e-04 E_mag [nT] +6.186e+00 +7.830e-01 error_mag [nT] +2.541e+01 +2.134e+00

            auto Pguid = new control::guid(6, 800.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid);
            Pguid->_sti_id = st::logic::sti_Hp3000_tas30_chiXX;
            double chi_deg_init = Pguid->_dist_uniform_bearing(Pguid->_gen);
            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         3000.0, ail_chi_deg, chi_deg_init, rud_beta_deg, 0., trgg_t_sec,       100.0));
            Pguid->add_op(1, new control::guid_op(thr_vtas_mps, 30.0, elv_gammaTAS_deg,    2.0, ail_chi_deg, chi_deg_init, rud_beta_deg, 0., trgg_Hp_m,       3100.0));
            Pguid->add_op(2, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         3100.0, ail_chi_deg, chi_deg_init, rud_beta_deg, 0., trgg_Deltat_sec,  100.0));
            Pguid->add_op(3, new control::guid_op(thr_vtas_mps, 25.0, elv_Hp_m,         3100.0, ail_chi_deg, chi_deg_init, rud_beta_deg, 0., trgg_Deltat_sec,  100.0));
            Pguid->add_op(4, new control::guid_op(thr_vtas_mps, 25.0, elv_gammaTAS_deg,   -2.0, ail_chi_deg, chi_deg_init, rud_beta_deg, 0., trgg_Hp_m,       3000.0));
            Pguid->add_op(5, new control::guid_op(thr_vtas_mps, 25.0, elv_Hp_m,         3000.0, ail_chi_deg, chi_deg_init, rud_beta_deg, 0., trgg_Deltat_sec,  100.0));
            return Pguid;
        }
        case 26: { // long flight with one turn close to the beginning (similar to GPS out) (similar to #36 but now random)- !!!!!! REQUIRES LONGER TURBULENCE TIME !!!!!!
            // sti now includes turbulence, new att filter
            // ** 12: No wind, no offsets, turb 0.5
            // 36 -> rotv [deg] +1.488e-01 +9.180e-02 w_nbb [dps] +2.655e-01 +1.822e-01 E_gyr [dps] +1.334e-03 +5.749e-04 E_mag [nT] +2.300e+01 +6.701e+00 error_mag [nT] +8.033e+01 +3.033e+01
            // 01 -> rotv [deg] +1.439e-01 +1.018e-01 w_nbb [dps] +2.656e-01 +1.822e-01 E_gyr [dps] +1.357e-03 +5.896e-04 E_mag [nT] +8.872e+00 +3.600e+00 error_mag [nT] +7.626e+01 +3.276e+01
            // 02 -> rotv [deg] +1.215e-01 +7.668e-02 w_nbb [dps] +2.627e-01 +1.812e-01 E_gyr [dps] +1.275e-03 +5.159e-04 E_mag [nT] +6.364e+01 +2.366e+01 error_mag [nT] +1.651e+01 +8.161e+00
            // 03 -> rotv [deg] +2.689e-01 +1.195e-01 w_nbb [dps] +2.649e-01 +1.810e-01 E_gyr [dps] +1.323e-03 +5.809e-04 E_mag [nT] +6.290e+01 +1.150e+01 error_mag [nT] +7.462e+01 +3.351e+01
            // 04 ->
            // 05 ->
            // ** 14: Wind constant but random, offsets constant but random, turb 0.5
            // 01 -> rotv [deg] +1.749e-01 +1.147e-01 w_nbb [dps] +2.653e-01 +1.814e-01 E_gyr [dps] +1.339e-03 +5.715e-04 E_mag [nT] +7.413e+00 +3.672e+00 error_mag [nT] +7.810e+01 +3.449e+01
            // 02 -> rotv [deg] +1.127e-01 +6.964e-02 w_nbb [dps] +2.635e-01 +1.819e-01 E_gyr [dps] +1.264e-03 +5.132e-04 E_mag [nT] +6.417e+01 +2.356e+01 error_mag [nT] +1.638e+01 +8.107e+00
            // 03 -> rotv [deg] +2.402e-01 +1.137e-01 w_nbb [dps] +2.634e-01 +1.798e-01 E_gyr [dps] +1.326e-03 +5.817e-04 E_mag [nT] +6.168e+01 +1.119e+01 error_mag [nT] +7.420e+01 +3.314e+01
            // 04 -> rotv [deg] +1.728e-01 +9.369e-02 w_nbb [dps] +2.688e-01 +1.845e-01 E_gyr [dps] +1.236e-03 +5.219e-04 E_mag [nT] +4.101e+01 +6.999e+00 error_mag [nT] +2.381e+01 +1.345e+01
            // 05 -> rotv [deg] +1.207e-01 +8.317e-02 w_nbb [dps] +2.682e-01 +1.844e-01 E_gyr [dps] +1.252e-03 +4.905e-04 E_mag [nT] +2.840e+01 +4.442e+00 error_mag [nT] +6.761e+01 +2.713e+01
            // ** 15: Wind constant but random, offsets random ramp, turb 0.5
            // 01 -> rotv [deg] +1.753e-01 +1.150e-01 w_nbb [dps] +2.644e-01 +1.808e-01 E_gyr [dps] +1.340e-03 +5.719e-04 E_mag [nT] +7.540e+00 +3.651e+00 error_mag [nT] +7.826e+01 +3.455e+01
            // ** 16: Wind random ramp, offsets random ramp, turb 0.5
            // 01 -> rotv [deg] +1.390e-01 +9.904e-02 w_nbb [dps] +2.650e-01 +1.821e-01 E_gyr [dps] +1.358e-03 +5.881e-04 E_mag [nT] +9.416e+00 +3.179e+00 error_mag [nT] +7.550e+01 +3.258e+01
            // 02 -> rotv [deg] +1.268e-01 +7.861e-02 w_nbb [dps] +2.641e-01 +1.821e-01 E_gyr [dps] +1.269e-03 +5.152e-04 E_mag [nT] +6.142e+01 +2.380e+01 error_mag [nT] +1.691e+01 +8.314e+00
            // 03 -> rotv [deg] +2.929e-01 +1.224e-01 w_nbb [dps] +2.654e-01 +1.814e-01 E_gyr [dps] +1.318e-03 +5.753e-04 E_mag [nT] +7.069e+01 +1.424e+01 error_mag [nT] +7.428e+01 +3.381e+01
            // 04 -> rotv [deg] +1.602e-01 +9.855e-02 w_nbb [dps] +2.700e-01 +1.848e-01 E_gyr [dps] +1.245e-03 +5.222e-04 E_mag [nT] +3.832e+01 +6.015e+00 error_mag [nT] +2.141e+01 +1.221e+01
            // 05 -> rotv [deg] +1.290e-01 +8.759e-02 w_nbb [dps] +2.589e-01 +1.775e-01 E_gyr [dps] +1.263e-03 +5.057e-04 E_mag [nT] +3.350e+01 +6.835e+00 error_mag [nT] +6.835e+01 +2.784e+01
            // ** GPS01: NO wind, no offsets, turb 0.5
            // 01 -> rotv [deg] +2.580e-01 +1.377e-01 w_nbb [dps] +2.976e-01 +1.737e-01 E_gyr [dps] +1.379e-03 +6.078e-04 E_mag [nT] +1.297e+01 +4.618e+00 error_mag [nT] +9.993e+01 +4.508e+01
            // 02 -> rotv [deg] +2.712e-01 +1.214e-01 w_nbb [dps] +2.965e-01 +1.724e-01 E_gyr [dps] +1.312e-03 +5.457e-04 E_mag [nT] +7.406e+01 +2.997e+01 error_mag [nT] +2.292e+01 +1.171e+01
            // 03 -> rotv [deg] +5.342e-01 +1.616e-01 w_nbb [dps] +2.993e-01 +1.718e-01 E_gyr [dps] +1.368e-03 +6.237e-04 E_mag [nT] +6.283e+01 +1.161e+01 error_mag [nT] +9.634e+01 +4.626e+01
            // 04 ->
            // 05 ->
            // ** GPS02: Wind constant but random, offsets constant but random, turb 0.5
            // 01 -> rotv [deg] +3.540e-01 +1.421e-01 w_nbb [dps] +3.015e-01 +1.725e-01 E_gyr [dps] +1.367e-03 +5.911e-04 E_mag [nT] +1.110e+01 +5.416e+00 error_mag [nT] +1.024e+02 +4.779e+01
            // 02 -> rotv [deg] +2.615e-01 +1.200e-01 w_nbb [dps] +2.960e-01 +1.732e-01 E_gyr [dps] +1.313e-03 +5.457e-04 E_mag [nT] +7.408e+01 +2.947e+01 error_mag [nT] +2.252e+01 +1.151e+01
            // 03 -> rotv [deg] +4.971e-01 +1.603e-01 w_nbb [dps] +2.952e-01 +1.710e-01 E_gyr [dps] +1.360e-03 +6.200e-04 E_mag [nT] +6.145e+01 +1.125e+01 error_mag [nT] +9.479e+01 +4.502e+01
            // 04 -> rotv [deg] +3.229e-01 +1.307e-01 w_nbb [dps] +2.952e-01 +1.762e-01 E_gyr [dps] +1.282e-03 +5.563e-04 E_mag [nT] +4.266e+01 +8.417e+00 error_mag [nT] +2.819e+01 +1.550e+01
            // 05 -> rotv [deg] +2.363e-01 +1.203e-01 w_nbb [dps] +3.151e-01 +1.747e-01 E_gyr [dps] +1.299e-03 +5.278e-04 E_mag [nT] +3.296e+01 +6.012e+00 error_mag [nT] +8.430e+01 +3.836e+01
            // ** GPS03: Wind random ramp, offsets random ramp, turb 0.5
            // 01 -> rotv [deg] +2.098e-01 +1.299e-01 w_nbb [dps] +2.892e-01 +1.745e-01 E_gyr [dps] +1.370e-03 +5.851e-04 E_mag [nT] +1.275e+01 +4.033e+00 error_mag [nT] +9.339e+01 +4.202e+01
            // 02 -> rotv [deg] +3.047e-01 +1.270e-01 w_nbb [dps] +3.037e-01 +1.726e-01 E_gyr [dps] +1.317e-03 +5.491e-04 E_mag [nT] +7.431e+01 +3.182e+01 error_mag [nT] +2.429e+01 +1.241e+01
            // 03 -> rotv [deg] +5.908e-01 +1.772e-01 w_nbb [dps] +3.038e-01 +1.718e-01 E_gyr [dps] +1.372e-03 +6.291e-04 E_mag [nT] +6.329e+01 +1.173e+01 error_mag [nT] +1.008e+02 +4.999e+01
            // 04 -> rotv [deg] +4.195e-01 +1.485e-01 w_nbb [dps] +3.156e-01 +1.749e-01 E_gyr [dps] +1.303e-03 +5.715e-04 E_mag [nT] +4.103e+01 +8.610e+00 error_mag [nT] +2.097e+01 +1.208e+01
            // 05 -> rotv [deg] +2.286e-01 +1.199e-01 w_nbb [dps] +3.026e-01 +1.682e-01 E_gyr [dps] +1.302e-03 +5.330e-04 E_mag [nT] +3.275e+01 +5.951e+00 error_mag [nT] +8.406e+01 +3.810e+01

            // ** With filter_air02 and new tas,oat,osp sensors with same values (but different results because zero bias evaluated once)
            //    Basically we obtain the same results
            // 12-01 -> rotv [deg] +1.401e-01 +9.950e-02 w_nbb [dps] +2.654e-01 +1.822e-01 E_gyr [dps] +1.355e-03 +5.878e-04 E_mag [nT] +9.134e+00 +3.221e+00 error_mag [nT] +7.596e+01 +3.264e+01
            // 14-01 -> rotv [deg] +1.692e-01 +1.131e-01 w_nbb [dps] +2.651e-01 +1.814e-01 E_gyr [dps] +1.338e-03 +5.705e-04 E_mag [nT] +7.612e+00 +3.402e+00 error_mag [nT] +7.755e+01 +3.434e+01
            // 16-01 -> rotv [deg] +1.358e-01 +9.709e-02 w_nbb [dps] +2.648e-01 +1.820e-01 E_gyr [dps] +1.357e-03 +5.863e-04 E_mag [nT] +9.784e+00 +2.953e+00 error_mag [nT] +7.513e+01 +3.246e+01
            // 01-01 -> rotv [deg] +2.508e-01 +1.362e-01 w_nbb [dps] +2.975e-01 +1.736e-01 E_gyr [dps] +1.375e-03 +6.062e-04 E_mag [nT] +1.329e+01 +4.311e+00 error_mag [nT] +9.934e+01 +4.474e+01
            // 02-01 -> rotv [deg] +3.464e-01 +1.409e-01 w_nbb [dps] +3.014e-01 +1.724e-01 E_gyr [dps] +1.363e-03 +5.896e-04 E_mag [nT] +1.127e+01 +5.112e+00 error_mag [nT] +1.016e+02 +4.742e+01
            // 03-01 -> rotv [deg] +2.032e-01 +1.275e-01 w_nbb [dps] +2.891e-01 +1.744e-01 E_gyr [dps] +1.367e-03 +5.828e-04 E_mag [nT] +1.308e+01 +3.780e+00 error_mag [nT] +9.275e+01 +4.168e+01

            // ** With filter_air02 and new tas,oat,osp sensors with biases
            // 03-01 -> rotv [deg] +2.105e-01 +1.272e-01 w_nbb [dps] +2.916e-01 +1.764e-01 E_gyr [dps] +1.368e-03 +5.743e-04 E_mag [nT] +1.228e+01 +4.339e+00 error_mag [nT] +9.337e+01 +4.215e+01
            // 03-02 -> rotv [deg] +3.037e-01 +1.263e-01 w_nbb [dps] +3.059e-01 +1.743e-01 E_gyr [dps] +1.316e-03 +5.494e-04 E_mag [nT] +7.437e+01 +3.166e+01 error_mag [nT] +2.434e+01 +1.243e+01



            auto Pguid = new control::guid(3, 3800.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid); // 3800
            Pguid->_sti_id = st::logic::sti_Hp3000_tas30_chiXX;

            double vtas_mps = 30.0;
            double Hp_m     = 3000.0;
            double beta_deg = 0.0;

            double chi_deg_init = Pguid->_dist_uniform_bearing(Pguid->_gen); // random initial ground velocity bearing
            double chi_deg_end; // random ground velocity bearing
            double angle_diff_deg; // angular difference
            double xi_deg; // bank angle during turn
            do { // ensure that there is a direction change of at least 10 [deg]
                chi_deg_end = Pguid->_dist_uniform_bearing(Pguid->_gen);
                angle_diff_deg = ang::tools::angle_diff_deg(chi_deg_end, chi_deg_init);
            } while (std::fabs(angle_diff_deg) < 10.0);

            // turn direction depends on initial and final bearings
            if (angle_diff_deg > 0) {xi_deg = +10.0;}
            else                    {xi_deg = -10.0;}

            std::cout << std::endl << "INTENT:" << std::endl << std::endl;
            std::cout << "Initial chi [deg]: " << std::fixed << std::setw(6)  << std::setprecision(1) << std::showpos << chi_deg_init << std::endl;
            std::cout << "Turn     xi [deg]: " << std::fixed << std::setw(6)  << std::setprecision(1) << std::showpos <<       xi_deg << std::endl;
            std::cout << "Final   chi [deg]: " << std::fixed << std::setw(6)  << std::setprecision(1) << std::showpos <<  chi_deg_end << std::endl;

            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, vtas_mps, elv_Hp_m, Hp_m, ail_chi_deg,  chi_deg_init, rud_beta_deg, beta_deg, trgg_t_sec,         130.0));
            Pguid->add_op(1, new control::guid_op(thr_vtas_mps, vtas_mps, elv_Hp_m, Hp_m, ail_xi_deg,         xi_deg, rud_beta_deg, beta_deg, trgg_chi_deg, chi_deg_end));
            Pguid->add_op(2, new control::guid_op(thr_vtas_mps, vtas_mps, elv_Hp_m, Hp_m, ail_chi_deg,   chi_deg_end, rud_beta_deg, beta_deg, trgg_t_sec,        3800.0));
            return Pguid;
        }
        case 27: { // long flight with one turn close to the beginning, ends up tracking psi instead of chi - !!!!!! REQUIRES LONGER TURBULENCE TIME !!!!!!
            // ** 12: No wind, no offsets, turb 0.5
            // 01 -> rotv [deg] +2.479e-01 +1.129e-01 w_nbb [dps] +2.641e-01 +1.830e-01 E_gyr [dps] +1.370e-03 +5.834e-04 E_mag [nT] +5.336e+01 +1.051e+01 error_mag [nT] +7.635e+01 +3.350e+01
            // 02 -> rotv [deg] +1.339e-01 +8.275e-02 w_nbb [dps] +2.613e-01 +1.818e-01 E_gyr [dps] +1.277e-03 +5.158e-04 E_mag [nT] +6.422e+01 +2.402e+01 error_mag [nT] +1.684e+01 +8.347e+00
            // 03 -> rotv [deg] +2.926e-01 +1.223e-01 w_nbb [dps] +2.634e-01 +1.815e-01 E_gyr [dps] +1.316e-03 +5.766e-04 E_mag [nT] +6.235e+01 +1.137e+01 error_mag [nT] +7.588e+01 +3.426e+01
            // 04 ->
            // 05 ->
            // ** 14: Wind constant but random, offsets constant but random, turb 0.5
            // 01 -> rotv [deg] +3.014e-01 +1.292e-01 w_nbb [dps] +2.653e-01 +1.837e-01 E_gyr [dps] +1.371e-03 +5.887e-04 E_mag [nT] +6.048e+01 +1.273e+01 error_mag [nT] +7.535e+01 +3.367e+01
            // 02 -> rotv [deg] +1.241e-01 +7.567e-02 w_nbb [dps] +2.621e-01 +1.824e-01 E_gyr [dps] +1.267e-03 +5.123e-04 E_mag [nT] +6.456e+01 +2.390e+01 error_mag [nT] +1.675e+01 +8.311e+00
            // 03 -> rotv [deg] +2.860e-01 +1.195e-01 w_nbb [dps] +2.621e-01 +1.803e-01 E_gyr [dps] +1.317e-03 +5.758e-04 E_mag [nT] +6.503e+01 +1.195e+01 error_mag [nT] +7.503e+01 +3.388e+01
            // 04 -> rotv [deg] +2.127e-01 +1.080e-01 w_nbb [dps] +2.677e-01 +1.849e-01 E_gyr [dps] +1.245e-03 +5.228e-04 E_mag [nT] +4.219e+01 +7.131e+00 error_mag [nT] +2.406e+01 +1.361e+01
            // 05 -> rotv [deg] +1.249e-01 +8.510e-02 w_nbb [dps] +2.665e-01 +1.851e-01 E_gyr [dps] +1.248e-03 +4.894e-04 E_mag [nT] +2.818e+01 +4.377e+00 error_mag [nT] +6.806e+01 +2.776e+01
            // ** 16: Wind random ramp, offsets random ramp, turb 0.5
            // 01 -> rotv [deg] +2.405e-01 +1.074e-01 w_nbb [dps] +2.631e-01 +1.822e-01 E_gyr [dps] +1.373e-03 +5.908e-04 E_mag [nT] +5.497e+01 +1.099e+01 error_mag [nT] +7.657e+01 +3.360e+01
            // 02 -> rotv [deg] +1.453e-01 +8.811e-02 w_nbb [dps] +2.625e-01 +1.827e-01 E_gyr [dps] +1.278e-03 +5.168e-04 E_mag [nT] +6.227e+01 +2.428e+01 error_mag [nT] +1.689e+01 +8.407e+00
            // 03 -> rotv [deg] +2.775e-01 +1.204e-01 w_nbb [dps] +2.640e-01 +1.820e-01 E_gyr [dps] +1.315e-03 +5.749e-04 E_mag [nT] +6.253e+01 +1.142e+01 error_mag [nT] +7.565e+01 +3.462e+01
            // 04 -> rotv [deg] +1.474e-01 +8.652e-02 w_nbb [dps] +2.684e-01 +1.854e-01 E_gyr [dps] +1.230e-03 +5.234e-04 E_mag [nT] +3.253e+01 +6.166e+00 error_mag [nT] +2.344e+01 +1.297e+01
            // 05 -> rotv [deg] +1.424e-01 +9.304e-02 w_nbb [dps] +2.573e-01 +1.782e-01 E_gyr [dps] +1.253e-03 +4.956e-04 E_mag [nT] +2.648e+01 +4.170e+00 error_mag [nT] +6.924e+01 +2.845e+01
            // ** GPS01: NO wind, no offsets, turb 0.5
            // 01 -> rotv [deg] +1.992e-01 +9.357e-02 w_nbb [dps] +2.643e-01 +1.830e-01 E_gyr [dps] +1.377e-03 +5.887e-04 E_mag [nT] +5.015e+01 +9.886e+00 error_mag [nT] +7.376e+01 +3.202e+01
            // 02 -> rotv [deg] +1.249e-01 +7.913e-02 w_nbb [dps] +2.616e-01 +1.817e-01 E_gyr [dps] +1.278e-03 +5.202e-04 E_mag [nT] +6.400e+01 +2.364e+01 error_mag [nT] +1.708e+01 +8.536e+00
            // 03 -> rotv [deg] +2.837e-01 +1.223e-01 w_nbb [dps] +2.637e-01 +1.812e-01 E_gyr [dps] +1.325e-03 +5.769e-04 E_mag [nT] +6.226e+01 +1.135e+01 error_mag [nT] +7.549e+01 +3.403e+01
            // 04 ->
            // 05 ->
            // ** GPS02: Wind constant but random, offsets constant but random, turb 0.5
            // 01 -> rotv [deg] +2.300e-01 +1.061e-01 w_nbb [dps] +2.656e-01 +1.837e-01 E_gyr [dps] +1.371e-03 +5.864e-04 E_mag [nT] +5.772e+01 +1.181e+01 error_mag [nT] +7.397e+01 +3.307e+01
            // 02 -> rotv [deg] +1.206e-01 +7.552e-02 w_nbb [dps] +2.624e-01 +1.823e-01 E_gyr [dps] +1.269e-03 +5.176e-04 E_mag [nT] +6.434e+01 +2.347e+01 error_mag [nT] +1.701e+01 +8.518e+00
            // 03 -> rotv [deg] +2.903e-01 +1.211e-01 w_nbb [dps] +2.623e-01 +1.801e-01 E_gyr [dps] +1.331e-03 +5.804e-04 E_mag [nT] +6.481e+01 +1.190e+01 error_mag [nT] +7.471e+01 +3.365e+01
            // 04 -> rotv [deg] +1.628e-01 +8.929e-02 w_nbb [dps] +2.679e-01 +1.847e-01 E_gyr [dps] +1.237e-03 +5.229e-04 E_mag [nT] +4.169e+01 +7.115e+00 error_mag [nT] +2.430e+01 +1.334e+01
            // 05 -> rotv [deg] +1.251e-01 +8.649e-02 w_nbb [dps] +2.667e-01 +1.851e-01 E_gyr [dps] +1.252e-03 +4.891e-04 E_mag [nT] +2.817e+01 +4.396e+00 error_mag [nT] +6.824e+01 +2.773e+01
            // ** GPS03: Wind random ramp, offsets random ramp, turb 0.5
            // 01 -> rotv [deg] +1.838e-01 +8.824e-02 w_nbb [dps] +2.633e-01 +1.822e-01 E_gyr [dps] +1.371e-03 +5.839e-04 E_mag [nT] +5.140e+01 +1.030e+01 error_mag [nT] +7.359e+01 +3.199e+01
            // 02 -> rotv [deg] +1.301e-01 +8.588e-02 w_nbb [dps] +2.627e-01 +1.826e-01 E_gyr [dps] +1.288e-03 +5.250e-04 E_mag [nT] +6.231e+01 +2.414e+01 error_mag [nT] +1.703e+01 +8.510e+00
            // 03 -> rotv [deg] +3.107e-01 +1.287e-01 w_nbb [dps] +2.641e-01 +1.816e-01 E_gyr [dps] +1.318e-03 +5.745e-04 E_mag [nT] +6.249e+01 +1.141e+01 error_mag [nT] +7.754e+01 +3.638e+01
            // 04 -> rotv [deg] +1.397e-01 +8.294e-02 w_nbb [dps] +2.686e-01 +1.852e-01 E_gyr [dps] +1.234e-03 +5.198e-04 E_mag [nT] +3.284e+01 +6.176e+00 error_mag [nT] +2.354e+01 +1.284e+01
            // 05 -> rotv [deg] +1.311e-01 +8.456e-02 w_nbb [dps] +2.574e-01 +1.781e-01 E_gyr [dps] +1.260e-03 +4.908e-04 E_mag [nT] +2.655e+01 +4.210e+00 error_mag [nT] +6.742e+01 +2.717e+01

            auto Pguid = new control::guid(3, 3800.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid);
            Pguid->_sti_id = st::logic::sti_Hp3000_tas30_chiXX;

            double vtas_mps = 30.0;
            double Hp_m     = 3000.0;
            double beta_deg = 0.0;

            double chi_deg_init = Pguid->_dist_uniform_bearing(Pguid->_gen); // random initial ground velocity bearing
            double psi_deg_end; // random final body heading
            double angle_diff_deg; // angular difference
            double xi_deg; // bank angle during turn
            do { // ensure that there is a direction change of at least 30 [deg] (chi versus  psi, but it is OK)
                psi_deg_end = Pguid->_dist_uniform_bearing(Pguid->_gen);
                angle_diff_deg = ang::tools::angle_diff_deg(psi_deg_end, chi_deg_init);
            } while (std::fabs(angle_diff_deg) < 30.0);

            // turn direction depends on initial and final bearings
            if (angle_diff_deg > 0) {xi_deg = +10.0;}
            else                    {xi_deg = -10.0;}

            std::cout << std::endl << "INTENT:" << std::endl << std::endl;
            std::cout << "Initial chi [deg]: " << std::fixed << std::setw(6)  << std::setprecision(1) << std::showpos << chi_deg_init << std::endl;
            std::cout << "Turn     xi [deg]: " << std::fixed << std::setw(6)  << std::setprecision(1) << std::showpos <<       xi_deg << std::endl;
            std::cout << "Final   psi [deg]: " << std::fixed << std::setw(6)  << std::setprecision(1) << std::showpos << psi_deg_end  << std::endl;

            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, vtas_mps, elv_Hp_m, Hp_m, ail_chi_deg, chi_deg_init, rud_beta_deg, beta_deg, trgg_t_sec,         130.0));
            Pguid->add_op(1, new control::guid_op(thr_vtas_mps, vtas_mps, elv_Hp_m, Hp_m, ail_xi_deg,        xi_deg, rud_beta_deg, beta_deg, trgg_psi_deg, psi_deg_end));
            Pguid->add_op(2, new control::guid_op(thr_vtas_mps, vtas_mps, elv_Hp_m, Hp_m, ail_psi_deg,  psi_deg_end, rud_beta_deg, beta_deg, trgg_t_sec,        3800.0));
            return Pguid;
        }

        // ===========================================================
        // ===== ===== ===== Navigation Trajectories ===== ===== =====
        // ===========================================================

        case 31: { // short straight flight
            // Open: max errors are vtas [mps] -0.968, theta [deg] -0.512, xi [deg] -0.704, beta [deg] -0.975, Hp [m] 1.102 chi [deg] -0.198
            //       mean-std error vtas [mps] +2.17e-3 +1.05e-1, alpha [deg] +5.46e-3 +1.16e-1, beta [deg] -3.39e-3 +1.29e-1, Hp [m] -1.27e-2 +4.88e-1
            //       mean-std error rtov [deg] +3.75e-1 +2.44e-1, w_nbb [dps] +6.25e-2 +2.63e-2, bias_gyr [dps] +6.13e-3 +6.53e-3, bias_mag [nT] +6.84e+0 +9.02e-1
            // ** 01: No wind, no offsets, turb 1.0, init time -50, constant Bn, no realism gc nor B, ""old sensors with no M no R"", all init cond perfect, filter 01(01-01-01-02):
            // max control error vtas [mps] -1.147 theta [deg] +1.565 xi [deg]-0.779 beta [deg] -1.032 Hp [m]-7.600 chi [deg] +0.297
            // mean-std nav error rotv [deg] +1.796e-01 +1.532e-01 w_nbb [dps] +7.774e-02 +3.660e-02 E_gyr [dps] +1.925e-03 +7.420e-04 E_mag [nT]
            // ** 02: No wind, no offsets, turb 1.0, init time -50, constant Bn, no realism gc nor B, ""sensors with no M no R"", all init cond perfect, filter 01(01-01-01-02),
            //        Added Eacc as input to filter_att, modified Hp control as 35 was unstable, modified pos filter system noise
            // max control error vtas [mps] -0.917 theta [deg] +1.561 xi [deg] -0.794 beta [deg] -1.016 Hp [m] +12.912 chi [deg] +0.287
            // mean-std nav error rotv [deg] +1.281e-01 +9.160e-02 w_nbb [dps] +8.802e-02 +4.448e-02 E_gyr [dps] +1.887e-03 +8.446e-04 E_mag [nT] +8.150e-11 +7.180e-11
            // ** 03: No wind, no offsets, turb 1.0, init time -50, constant Bn, no realism gc nor B, ""gyr & acc with no R, mag complete"", all init cond perfect, filter 01(01-01-01-02)
            //        All differences due to magnetometer scale and cross coupling, nothing from gyroscope and accelerometer.
            //        Note that E_mag error was up a lot but continues to be very small. Does not matter because rotv error does not change.
            //        Multiplied magnetometer observation noise by 10 in filter_att to solve attitude errors in 33 and 34
            // max control error vtas [mps] -0.917 theta [deg] +1.560 xi [deg] -0.791 beta [deg] -1.017 Hp [m] +12.897 chi [deg] +0.287
            // mean-std nav error rotv [deg] +1.297e-01 +9.056e-02 w_nbb [dps] +8.800e-02 +4.446e-02 E_gyr [dps] +1.915e-03 +8.113e-04 E_mag [nT] +1.300e+00 +6.315e-01
            // ** 04: No wind, no offsets, turb 1.0, init time -50, constant Bn, no realism gc nor B, sensors complete, all init cond perfect, filter 01(01-01-01-02)
            // max control error vtas [mps] -0.917 theta [deg] +1.560 xi [deg] -0.792 beta [deg] -1.017  Hp [m] +12.897 chi [deg] +0.287
            // mean-std nav error rotv [deg] +1.297e-01 +9.057e-02 w_nbb [dps] +8.800e-02 +4.446e-02 E_gyr [dps] +1.915e-03 +8.114e-04 E_mag [nT] +1.300e+00 +6.315e-01
            // ** 05: No wind, no offsets, turb 1.0, init time -50, constant Bn, no realism B yes gc, sensors complete, all init cond perfect, filter 01(01-01-01-02)
            //        Note that computing gravity with position all the time in filter is not dangerous even if position is way off
            // max control error vtas [mps] -0.917 theta [deg] +1.560 xi [deg] -0.792 beta [deg] -1.017 Hp [m] +12.897 chi [deg] +0.286
            // mean-std nav error rotv [deg] +1.298e-01 +9.093e-02 w_nbb [dps] +8.800e-02 +4.446e-02 E_gyr [dps] +1.917e-03 +8.165e-04 E_mag [nT] +1.298e+00 +6.305e-01
            // ** 06: No wind, no offsets, turb 1.0, init time -50, variable Bn, no realism B yes gc, sensors complete, all init cond perfect, filter 01(01-01-01-02)
            //        Note that computing magnetism with position all the time in filter may be dangerous when no GPS if position is way off
            // max control error vtas [mps] -0.917 theta [deg] +1.560 xi [deg] -0.792 beta [deg] -1.018 Hp [m] +12.899 chi [deg] +0.286
            // mean-std nav error rotv [deg] +1.295e-01 +9.047e-02 w_nbb [dps] +8.800e-02 +4.446e-02 E_gyr [dps] +1.912e-03 +8.003e-04 E_mag [nT] +1.307e+00 +6.357e-01
            // ** 07: No wind, no offsets, turb 1.0, init time -50, variable Bn, realism in B and gc, sensors complete, all init cond perfect, filter 01(01-01-01-02)
            //        Results are not bad if they can be repeated with other seeds, backup option if nothing else works
            // max control error vtas [mps] -0.917 theta [deg] +1.559 xi [deg] -0.786 beta [deg] -1.017 Hp [m] +12.909 chi [deg] +0.296
            // mean-std nav error rotv [deg] +1.997e-01 +9.434e-02 w_nbb [dps] +8.802e-02 +4.446e-02 E_gyr [dps] +2.070e-03 +1.084e-03 E_mag [nT] +1.312e+00 +6.405e-01
            // ** 08: No wind, no offsets, turb 1.0, init time -50, variable Bn, realism in B and gc, sensors complete, all init cond perfect, filter 02(01-02-01-02)
            //        In general quality of results returns to what it was in **06, before realism in B was included
            // max control error vtas [mps] -0.917 theta [deg] +1.560 xi [deg] -0.792 beta [deg] -1.018 Hp [m] +12.898 chi [deg] +0.286
            // mean-std nav error rotv [deg] +1.291e-01 +8.994e-02 w_nbb [dps] +8.800e-02 +4.446e-02 E_gyr [dps] +1.910e-03 +7.944e-04 E_mag [nT] +1.308e+00 +6.375e-01 error_mag [nT] +8.172e-01 +4.521e-01
            // ** 09: No wind, no offsets, turb 1.0, init time -50, variable Bn, realism in B and gc, sensors complete, init euler perfect, filter 02(01-02-01-02)
            // max control error vtas [mps] -0.916 theta [deg] +1.560 xi [deg] -0.813 beta [deg] -1.018 Hp [m] +12.950 chi [deg] +0.269
            // mean-std nav error rotv [deg] +2.141e-01 +1.511e-01 w_nbb [dps] +8.802e-02 +4.446e-02 E_gyr [dps] +2.434e-03 +1.136e-03 E_mag [nT] +7.967e+00 +8.886e-01 error_mag [nT] +9.703e+00 +8.207e-01
            // ** 10: No wind, no offsets, turb 1.0, init time -50, variable Bn, realism in B and gc, sensors complete, no perfect inits, filter 02(01-02-01-02)
            //        Changed att state e from 1e-3 to 1e-12 and att init a from 1e-10 to 1e-8 (may create some att oscillations after 2500 sec, watch out)
            // max control error vtas [mps] -0.916 theta [deg] +1.560 xi [deg] -0.813 beta [deg] -1.018 Hp [m] +12.950 chi [deg] +0.269
            // mean-std nav error rotv [deg] +2.824e-01 +1.873e-01 w_nbb [dps] +8.805e-02 +4.446e-02 E_gyr [dps] +3.033e-03 +1.390e-03 E_mag [nT] +7.188e+00 +2.614e+00 error_mag [nT] +4.680e+01 +4.861e+00
            // ** 11: Same as 10, but twice initial Euler error
            // max control error vtas [mps] -0.916 theta [deg] +1.560 xi [deg] -0.809 beta [deg] -1.018 Hp [m] +12.967 chi [deg] +0.273
            // mean-std nav error rotv [deg] +3.537e-01 +2.183e-01 w_nbb [dps] +8.808e-02 +4.447e-02 E_gyr [dps] +3.741e-03 +1.520e-03 E_mag [nT] +2.233e+01 +2.966e+00 error_mag [nT] +6.807e+01 +3.712e+00
            // ** 12: Same as 10, but turbulence 0.5 (significant improvement in results)
            // max control error vtas [mps] +0.520 theta [deg] +1.121 xi [deg] -0.483 beta [deg] +0.574 Hp [m] +7.270 chi [deg] +0.200
            // mean-std nav error rotv [deg] +2.188e-01 +1.328e-01 w_nbb [dps] +8.121e-02 +4.135e-02 E_gyr [dps] +2.653e-03 +9.427e-04 E_mag [nT] +1.005e+01 +1.247e+00 error_mag [nT] +4.219e+01 +2.446e+00
            //        Modifications #2 in filter_att (everything improves except w_nbb that gets worse)
            // mean-std nav error rotv [deg] +1.565e-01 +9.844e-02 w_nbb [dps] +2.688e-01 +1.833e-01 E_gyr [dps] +2.187e-03 +6.545e-04 E_mag [nT] +4.400e+00 +5.030e-01 error_mag [nT] +2.910e+01 +1.227e+00
            //        Modifications #4 in fitler_att
            // mean-std nav error rotv [deg] +1.506e-01 +9.661e-02 w_nbb [dps] +2.687e-01 +1.832e-01 E_gyr [dps] +2.194e-03 +6.556e-04 E_mag [nT] +4.073e+00 +4.474e-01 error_mag [nT] +2.738e+01 +1.220e+00




            auto Pguid = new control::guid(1, 200.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid); // 200
            Pguid->_sti_id = st::logic::sti_h3000_tas30_psi30;
            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 30.0, rud_beta_deg, 0., trgg_t_sec, 200.0));
            return Pguid;
        }
        case 32: { // long straight flight
            // Open: max errors are vtas [mps] +1.272 theta [deg] +0.574 xi [deg] +0.722 beta [deg] +1.015 Hp [m] +1.363 chi [deg] +0.240
            //       mean-std error vtas [mps] +1.35e-3 +1.05e-1 alpha [deg] +1.07e-3 +1.15e-1 beta [deg] -1.82e-3 +1.31e-1 Hp [m] +7.54e-2 +5.32e-1
            //       mean-std error rtov [deg] +2.57e-1 +1.88e-1 w_nbb [dps] +6.24e-2 +2.64e-2 bias_gyr [dps] +2.91e-3 +3.65e-3 bias_mag [nT] +6.47e+0 +6.29e-1
            // ** 01: No wind, no offsets, turb 1.0, init time -50, constant Bn, no realism gc nor B, ""old sensors with no M no R"", all init cond perfect, filter 01(01-01-01-02):
            // max control error vtas [mps] -1.729 theta [deg] +1.565 xi [deg] -1.043 beta [deg] +1.126 Hp [m] +11.538 chi [deg] +0.372
            // mean-std nav error rotv [deg] +2.144e-01 +1.541e-01 w_nbb [dps] +7.749e-02 +3.671e-02 E_gyr [dps] +1.662e-03 +7.392e-04 E_mag [nT] +8.697e-10 +6.201e-10
            // ** 02: No wind, no offsets, turb 1.0, init time -50, constant Bn, no realism gc nor B, ""sensors with no M no R"", all init cond perfect, filter 01(01-01-01-02),
            //        Added Eacc as input to filter_att, modified Hp control as 35 was unstable, modified pos filter system noise
            // max control error vtas [mps] +1.056 theta [deg] +1.700 xi [deg] -0.994 beta [deg] +1.122 Hp [m] -16.133 chi [deg] + 0.298
            // mean-std nav error rotv [deg] +1.970e-01 +1.458e-01 w_nbb [dps] +8.744e-02 +4.445e-02 E_gyr [dps] +1.731e-03 +8.203e-04 E_mag [nT] +7.155e-10 +5.279e-10
            // ** 03: No wind, no offsets, turb 1.0, init time -50, constant Bn, no realism gc nor B, ""gyr & acc with no R, mag complete"", all init cond perfect, filter 01(01-01-01-02)
            //        All differences due to magnetometer scale and cross coupling, nothing from gyroscope and accelerometer.
            //        Note that E_mag error was up a lot but continues to be very small. Does not matter because rotv error does not change
            //        Multiplied magnetometer observation noise by 10 in filter_att to solve attitude errors in 33 and 34
            // max control error vtas [mps] +1.056 theta [deg] +1.700 xi [deg] -1.000 beta [deg] +1.123 Hp [m] -16.131 chi [deg] +0.292
            // mean-std nav error rotv [deg] +2.042e-01 +1.513e-01 w_nbb [dps] +8.742e-02 +4.444e-02 E_gyr [dps] +1.744e-03 +7.993e-04 E_mag [nT] +1.430e+00 +7.208e-01
            // ** 04: No wind, no offsets, turb 1.0, init time -50, constant Bn, no realism gc nor B, ""sensors complete"", all init cond perfect, filter 01(01-01-01-02)
            // max control error vtas [mps] +1.056 theta [deg] +1.700 xi [deg] -1.000 beta [deg] +1.123 Hp [m] -16.131 chi [deg] +0.292
            // mean-std nav error rotv [deg] +2.043e-01 +1.513e-01 w_nbb [dps] +8.742e-02 +4.444e-02 E_gyr [dps] +1.744e-03 +7.993e-04 E_mag [nT] +1.430e+00 +7.208e-01
            // ** 05: No wind, no offsets, turb 1.0, init time -50, constant Bn, no realism B yes gc, sensors complete, all init cond perfect, filter 01(01-01-01-02)
            //        Note that computing gravity with position all the time in filter is not dangerous even if position is way off
            // max control error vtas [mps] +1.056 theta [deg] +1.700 xi [deg] -1.001 beta [deg] +1.123 Hp [m] -16.131 chi [deg] +0.293
            // mean-std nav error rotv [deg] +2.058e-01 +1.523e-01 w_nbb [dps] +8.742e-02 +4.444e-02 E_gyr [dps] +1.747e-03 +8.020e-04 E_mag [nT] +1.428e+00 +7.202e-01
            // ** 06: No wind, no offsets, turb 1.0, init time -50, variable Bn, no realism B yes gc, sensors complete, all init cond perfect, filter 01(01-01-01-02)
            //        Note that computing magnetism with position all the time in filter may be dangerous when no GPS if position is way off
            // max control error vtas [mps] +1.056 theta [deg] +1.700 xi [deg] -1.003 beta [deg] +1.124 Hp [m] -16.142 chi [deg] +0.292
            // mean-std nav error rotv [deg] +2.044e-01 +1.509e-01 w_nbb [dps] +8.742e-02 +4.444e-02 E_gyr [dps] +1.738e-03 +7.872e-04 E_mag [nT] +1.445e+00 +7.191e-01
            // ** 07: No wind, no offsets, turb 1.0, init time -50, variable Bn, realism in B and gc, sensors complete, all init cond perfect, filter 01(01-01-01-02)
            //        Results are not bad if they can be repeated with other seeds, backup option if nothing else works
            // max control error vtas [mps] +1.058 theta [deg] +1.699 xi [deg] -1.042 beta [deg] +1.115 Hp [m] -16.286 chi [deg] -0.338
            // mean-std nav error rotv [deg] +6.454e-01 +3.436e-01 w_nbb [dps] +8.751e-02 +4.444e-02 E_gyr [dps] +2.739e-03 +1.312e-03 E_mag [nT] +1.417e+00 +7.118e-01
            // ** 08: No wind, no offsets, turb 1.0, init time -50, variable Bn, realism in B and gc, sensors complete, all init cond perfect, filter 02(01-02-01-02)
            // max control error vtas [mps] +1.056 theta [deg] +1.700 xi [deg] -1.004 beta [deg] +1.124 Hp [m] -16.141 chi [deg] +0.292
            // mean-std nav error rotv [deg] +2.030e-01 +1.491e-01 w_nbb [dps] +8.742e-02 +4.444e-02 E_gyr [dps] +1.733e-03 +7.791e-04 E_mag [nT] +1.444e+00 +7.176e-01 error_mag [nT] +3.778e+00 +3.405e+00
            // ** 09: No wind, no offsets, turb 1.0, init time -50, variable Bn, realism in B and gc, sensors complete, init euler perfect, filter 02(01-02-01-02)
            // max control error vtas [mps] +1.056 theta [deg] +1.700 xi [deg] -1.003 beta [deg] +1.125 Hp [m] -16.141 chi [deg] +0.292
            // mean-std nav error rotv [deg] +2.770e-01 +1.799e-01 w_nbb [dps] +8.742e-02 +4.444e-02 E_gyr [dps] +1.845e-03 +9.353e-04 E_mag [nT] +8.261e+00 +8.717e-01 error_mag [nT] +1.048e+01 +1.732e+00
            // ** 10: No wind, no offsets, turb 1.0, init time -50, variable Bn, realism in B and gc, sensors complete, no perfect inits, filter 02(01-02-01-02)
            //        Changed att state e from 1e-3 to 1e-12 and att init a from 1e-10 to 1e-8 (may create some att oscillations after 2500 sec, watch out)
            // max control error vtas [mps] +1.056 theta [deg] +1.700 xi [deg] -1.003 beta [deg] +1.123 Hp [m] -16.138 chi [deg] +0.297
            // mean-std nav error rotv [deg] +4.550e-01 +2.340e-01 w_nbb [dps] +8.744e-02 +4.445e-02 E_gyr [dps] +2.277e-03 +1.191e-03 E_mag [nT] +8.001e+00 +4.912e+00 error_mag [nT] +8.237e+01 +2.787e+01
            // ** 11: Same as 10, but twice initial Euler error
            // max control error vtas [mps] +1.058 theta [deg] +1.699 xi [deg] -1.017 beta [deg] +1.121 Hp [m] -16.275 chi [deg] +0.339
            // mean-std nav error rotv [deg] +5.928e-01 +2.867e-01 w_nbb [dps] +8.747e-02 +4.445e-02 E_gyr [dps] +2.772e-03 +1.395e-03 E_mag [nT] +1.686e+01 +4.365e+00 error_mag [nT] +1.014e+02 +2.682e+01
            // ** 12: Same as 10, but turbulence 0.5 (significant improvement in results)
            // max control error vtas [mps] +0.602 theta [deg] +1.302 xi [deg] -0.572 beta [deg] +0.694 Hp [m] -9.231 chi [deg] -0.239
            // mean-std nav error rotv [deg] +3.298e-01 +1.541e-01 w_nbb [dps] +8.044e-02 +4.117e-02 E_gyr [dps] +1.862e-03 +8.772e-04 E_mag [nT] +7.567e+00 +1.857e+00 error_mag [nT] +6.179e+01 +1.556e+01
            //        Modifications #2 in filter_att (everything improves except w_nbb that gets worse)
            // mean-std nav error rotv [deg] +2.796e-01 +1.163e-01 w_nbb [dps] +2.646e-01 +1.806e-01 E_gyr [dps] +1.545e-03 +7.023e-04 E_mag [nT] +3.258e+00 +8.626e-01 error_mag [nT] +3.863e+01 +7.708e+00
            //        Modifications #4 in fitler_att
            // mean-std nav error rotv [deg] +2.549e-01 +1.104e-01 w_nbb [dps] +2.645e-01 +1.805e-01 E_gyr [dps] +1.538e-03 +6.985e-04 E_mag [nT] +3.281e+00 +6.385e-01 error_mag [nT] +3.627e+01 +7.045e+00





            auto Pguid = new control::guid(1, 800.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid); // 800
            Pguid->_sti_id = st::logic::sti_h3000_tas30_psi30;
            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 30.0, rud_beta_deg, 0., trgg_t_sec, 800.0));
            return Pguid;
        }
        case 33: { // level & constant tas flight with three straight segments and two 180 [deg] turns at bank angle 10 [deg]
            // Open: max errors are vtas [mps] +1.272 theta [deg] +0.574 xi [deg] -10.304 beta [deg] +2.695 Hp [m] +1.364 chi [deg] -1.170
            //       mean-std error vtas [mps] +2.31e-3 +1.07e-1 alpha [deg] +1.77e-3 +1.16e-1 beta [deg] -2.36e-3 +1.32e-1 Hp [m] +5.00e-2 +5.18e-1
            //       mean-std error rtov [deg] +3.25e-1 +1.98e-1 w_nbb [dps] +6.41e-2 +3.75e-2 bias_gyr [dps] +3.93e-3 +5.02e-3 bias_mag [nT] +4.40e+0 +2.14e+0
            // ** 01: No wind, no offsets, turb 1.0, init time -50, constant Bn, no realism gc nor B, ""old sensors with no M no R"", all init cond perfect, filter 01(01-01-01-02):
            // max control error vtas [mps] -1.744 theta [deg] +1.547 xi [deg] -10.105 beta [deg] -3.036 Hp [m]+11.569 chi [deg] -1.200
            // mean-std nav error rotv [deg] +1.329e-01 +9.254e-02 w_nbb [dps] +7.968e-02 +4.503e-02 E_gyr [dps] +1.621e-03 +6.145e-04 E_mag [nT] +3.310e-10 +2.604e-10
            // ** 02: No wind, no offsets, turb 1.0, init time -50, constant Bn, no realism gc nor B, ""sensors with no M no R"", all init cond perfect, filter 01(01-01-01-02),
            //        Added Eacc as input to filter_att, modified Hp control as 35 was unstable, modified pos filter system noise
            // max control error vtas [mps] -0.923 theta [deg] +1.701 xi [deg] -10.178 beta [deg] -3.058 Hp [m] +12.912 chi [deg] -1.183
            // mean-std nav error rotv [deg] +1.576e-01 +1.256e-01 w_nbb [dps] +8.995e-02 +5.143e-02 E_gyr [dps] +1.809e-03 +7.603e-04 E_mag [nT] +3.212e-10 +2.611e-10
            // ** 03: No wind, no offsets, turb 1.0, init time -50, constant Bn, no realism gc nor B, ""gyr & acc with no R, mag complete"", all init cond perfect, filter 01(01-01-01-02)
            //        All differences due to magnetometer scale and cross coupling, nothing from gyroscope and accelerometer.
            //        Note that E_mag error was up a lot but continues to be very small. Does not matter because rotv error does not change
            //        Multiplied magnetometer observation noise by 10 in filter_att to solve attitude errors in 33 and 34
            // max control error vtas [mps] -0.924 theta [deg] +1.700 xi [deg] -10.173 beta [deg] -3.062 Hp [m] +12.897 chi [deg] -1.197
            // mean-std nav error rotv [deg] +1.907e-01 +1.385e-01 w_nbb [dps] +8.992e-02 +5.123e-02 E_gyr [dps] +2.126e-03 +7.788e-04 E_mag [nT] +1.530e+01 +1.533e+01
            // ** 04: No wind, no offsets, turb 1.0, init time -50, constant Bn, no realism gc nor B, ""sensors complete"", all init cond perfect, filter 01(01-01-01-02)
            // max control error vtas [mps] -0.924 theta [deg] +1.700 xi [deg] -10.174 beta [deg] -3.062 Hp [m] +12.897 chi [deg] -1.197
            // mean-std nav error rotv [deg] +1.909e-01 +1.389e-01 w_nbb [dps] +8.992e-02 +5.122e-02 E_gyr [dps] +2.128e-03 +7.797e-04 E_mag [nT] +1.530e+01 +1.533e+01
            // ** 05: No wind, no offsets, turb 1.0, init time -50, constant Bn, no realism B yes gc, sensors complete, all init cond perfect, filter 01(01-01-01-02)
            //        Note that computing gravity with position all the time in filter is not dangerous even if position is way off
            // max control error vtas [mps] -0.924 theta [deg] +1.700 xi [deg] -10.127 beta [deg] -3.062 Hp [m] +12.897 chi [deg] -1.201
            // mean-std nav error rotv [deg] +1.905e-01 +1.378e-01 w_nbb [dps] +8.992e-02 +5.133e-02 E_gyr [dps] +2.125e-03 +7.784e-04 E_mag [nT] +1.530e+01 +1.534e+01
            // ** 06: No wind, no offsets, turb 1.0, init time -50, variable Bn, no realism B yes gc, sensors complete, all init cond perfect, filter 01(01-01-01-02)
            //        Note that computing magnetism with position all the time in filter may be dangerous when no GPS if position is way off
            // max control error vtas [mps] -0.924 theta [deg] +1.701 xi [deg] -10.234 beta [deg] -3.062 Hp [m] +12.899 chi [deg] -1.212
            // mean-std nav error rotv [deg] +1.921e-01 +1.369e-01 w_nbb [dps] +8.993e-02 +5.141e-02 E_gyr [dps] +2.117e-03 +7.631e-04 E_mag [nT] +1.489e+01 +1.483e+01
            // ** 07: No wind, no offsets, turb 1.0, init time -50, variable Bn, realism in B and gc, sensors complete, all init cond perfect, filter 01(01-01-01-02)
            //        Results are not bad if they can be repeated with other seeds, backup option if nothing else works
            // max control error vtas [mps] -0.924 theta [deg] +1.700 xi [deg] -10.467 beta [deg] -3.059 Hp [m] +12.909 chi [deg] -1.211
            // mean-std nav error rotv [deg] +2.745e-01 +1.350e-01 w_nbb [dps] +9.001e-02 +5.159e-02 E_gyr [dps] +1.896e-03 +7.601e-04 E_mag [nT] +1.488e+01 +1.482e+01
            // ** 08: No wind, no offsets, turb 1.0, init time -50, variable Bn, realism in B and gc, sensors complete, all init cond perfect, filter 02(01-02-01-02)
            // max control error vtas [mps] -0.924 theta [deg] +1.700 xi [deg] -10.152 beta [deg] -3.062 Hp [m] +12.898  chi [deg] -1.192
            // mean-std nav error rotv [deg] +1.690e-01 +1.179e-01 w_nbb [dps] +8.992e-02 +5.137e-02 E_gyr [dps] +1.991e-03 +7.073e-04 E_mag [nT] +1.488e+01 +1.481e+01 error_mag [nT] +6.787e+00 +6.006e+00
            // ** 09: No wind, no offsets, turb 1.0, init time -50, variable Bn, realism in B and gc, sensors complete, init euler perfect, filter 02(01-02-01-02)
            // max control error vtas [mps] -0.924 theta [deg] +1.700 xi [deg] -10.109 beta [deg] -3.077 Hp [m] +12.934 chi [deg] -1.192
            // mean-std nav error rotv [deg] +2.066e-01 +1.572e-01 w_nbb [dps] +8.992e-02 +5.124e-02 E_gyr [dps] +2.276e-03 +9.903e-04 E_mag [nT] +1.964e+01 +1.123e+01 error_mag [nT] +1.465e+01 +4.271e+00
            // ** 10: No wind, no offsets, turb 1.0, init time -50, variable Bn, realism in B and gc, sensors complete, no perfect inits, filter 02(01-02-01-02)
            //        Changed att state e from 1e-3 to 1e-12 and att init a from 1e-10 to 1e-8 (may create some att oscillations after 2500 sec, watch out)
            // max control error vtas [mps] -0.924 theta [deg] +1.700 xi [deg] -10.234 beta [deg] -3.081 Hp [m] +12.949 chi [deg] -1.173
            // mean-std nav error rotv [deg] +2.059e-01 +1.435e-01 w_nbb [dps] +8.996e-02 +5.135e-02 E_gyr [dps] +2.358e-03 +1.203e-03 E_mag [nT] +3.223e+01 +2.581e+01 error_mag [nT] +6.431e+01 +1.560e+01
            // ** 11: Same as 10, but twice initial Euler error
            // max control error vtas [mps] -0.924 theta [deg] +1.700 xi [deg] -10.250 beta [deg] -3.083 Hp [m] +12.967 chi [deg] -1.158
            // mean-std nav error rotv [deg] +2.860e-01 +1.523e-01 w_nbb [dps] +8.997e-02 +5.130e-02 E_gyr [dps] +2.524e-03 +1.452e-03 E_mag [nT] +3.670e+01 +2.208e+01 error_mag [nT] +8.476e+01 +1.407e+01
            // ** 12: Same as 10, but turbulence 0.5 (improvement in results)
            // max control error vtas [mps] +0.526 theta [deg] +1.302 xi [deg] -10.267 beta [deg] +2.966 Hp [m] +7.270 chi [deg] -1.176
            // mean-std nav error rotv [deg] +2.159e-01 +1.013e-01 w_nbb [dps] +8.304e-02 +4.895e-02 E_gyr [dps] +1.925e-03 +9.322e-04 E_mag [nT] +2.400e+01 +1.739e+01 error_mag [nT] +5.292e+01 +9.177e+00
            //        Modifications #2 in filter_att (everything improves except w_nbb that gets worse)
            // mean-std nav error rotv [deg] +1.841e-01 +9.996e-02 w_nbb [dps] +2.795e-01 +2.179e-01 E_gyr [dps] +1.814e-03 +6.071e-04 E_mag [nT] +1.810e+01 +1.419e+01 error_mag [nT] +3.477e+01 +5.268e+00
            //        Modifications #3 in filter_att (same attitude as #2, no w_nbb worsening, and everything else in between)
            // mean-std nav error rotv [deg] +1.953e-01 +1.064e-01 w_nbb [dps] +8.303e-02 +4.888e-02 E_gyr [dps] +1.903e-03 +9.500e-04 E_mag [nT] +1.926e+01 +1.472e+01 error_mag [nT] +4.733e+01 +8.623e+00
            //        Modifications #4 in filter_att
            // mean-std nav error rotv [deg] +1.795e-01 +9.924e-02 w_nbb [dps] +2.794e-01 +2.179e-01 E_gyr [dps] +1.822e-03 +6.142e-04 E_mag [nT] +1.746e+01 +1.445e+01 error_mag [nT] +3.305e+01 +5.287e+00




            auto Pguid = new control::guid(5, 500.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid); // 500
            Pguid->_sti_id = st::logic::sti_h3000_tas30_psi30;
            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg,   30.0, rud_beta_deg, 0., trgg_t_sec,      100.0));
            Pguid->add_op(1, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,    10.0, rud_beta_deg, 0., trgg_chi_deg,   -150.0));
            Pguid->add_op(2, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, -150.0, rud_beta_deg, 0., trgg_Deltat_sec, 100.0));
            Pguid->add_op(3, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,    10.0, rud_beta_deg, 0., trgg_chi_deg,     30.0));
            Pguid->add_op(4, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg,   30.0, rud_beta_deg, 0., trgg_Deltat_sec, 100.0));
            return Pguid;
        }
        case 34: { // level & constant tas flight with three straight segments and two 180 [deg] turns at bank angle 20 [deg]
            // Open: max errors are vtas [mps] -0.975, theta [deg] +1.685, xi [deg] +20.422, beta [deg] +5.857, Hp [m] +1.102, chi [deg] -2.371 (seed 1)
            //       mean-std error vtas [mps] +1.98e-3 +1.06e-1 alpha [deg] +2.21e-3 +1.16e-1 beta [deg] -2.40e-3 +1.42e-1 Hp [m] +4.75e-2 +5.02e-1
            //       mean-std error rtov [deg] +3.03e-1 +2.09e-1 w_nbb [dps] +6.73e-2 +6.69e-2 bias_gyr [dps] +4.14e-3 +5.34e-3 bias_mag [nT] +6.48e+0 +2.64e+0
            // ** 01: No wind, no offsets, turb 1.0, init time -50, constant Bn, no realism gc nor B, ""old sensors with no M no R"", all init cond perfect, filter 01(01-01-01-02):
            // max control error vtas [mps] -1.742 theta [deg] +1.573 xi [deg] -20.515 beta [deg] +5.548 Hp [m] +9.825 chi [deg] -2.401
            // mean-std nav error rotv [deg] +5.094e-01 +3.939e-01 w_nbb [dps] +8.360e-02 +7.253e-02 E_gyr [dps] +2.649e-03 +2.156e-03 E_mag [nT] +4.490e-10 +4.020e-10
            // ** 02: No wind, no offsets, turb 1.0, init time -50, constant Bn, no realism gc nor B, ""sensors with no M no R"", all init cond perfect, filter 01(01-01-01-02),
            //        Added Eacc as input to filter_att, modified Hp control as 35 was unstable, modified pos filter system noise
            // max control error vtas [mps] -0.930 theta [deg] -1.756 xi [deg] -20.223 beta [deg] -5.452 Hp [m]+12.912 chi [deg] -2.329
            // mean-std nav error rotv [deg] +4.292e-01 +3.289e-01 w_nbb [dps] +9.363e-02 +7.553e-02 E_gyr [dps] +2.238e-03 +1.886e-03 E_mag [nT] +4.363e-10 +3.846e-10
            // ** 03: No wind, no offsets, turb 1.0, init time -50, constant Bn, no realism gc nor B, ""gyr & acc with no R, mag complete"", all init cond perfect, filter 01(01-01-01-02)
            //        All differences due to magnetometer scale and cross coupling, nothing from gyroscope and accelerometer.
            //        Note that E_mag error was up a lot but continues to be very small. Does not matter because rotv error does not change
            //        Multiplied magnetometer observation noise by 10 in filter_att to solve attitude errors in 33 and 34
            // max control error vtas [mps] -0.931 theta [deg] -1.755 xi [deg] -20.204 beta [deg] +5.458 Hp [m] +12.897 chi [deg] -2.308
            // mean-std nav error rotv [deg] +4.654e-01 +3.642e-01 w_nbb [dps] +9.363e-02 +7.534e-02 E_gyr [dps] +2.500e-03 +2.178e-03 E_mag [nT] +1.391e+01 +1.531e+01
            // ** 04: No wind, no offsets, turb 1.0, init time -50, constant Bn, no realism gc nor B, ""sensors complete"", all init cond perfect, filter 01(01-01-01-02)
            // max control error vtas [mps] -0.931 theta [deg] -1.755 xi [deg] -20.204 beta [deg] +5.458 Hp [m] +12.897 chi [deg] -2.309
            // mean-std nav error rotv [deg] +4.644e-01 +3.637e-01 w_nbb [dps] +9.363e-02 +7.534e-02 E_gyr [dps] +2.499e-03 +2.174e-03 E_mag [nT] +1.391e+01 +1.531e+01
            // ** 05: No wind, no offsets, turb 1.0, init time -50, constant Bn, no realism B yes gc, sensors complete, all init cond perfect, filter 01(01-01-01-02)
            //        Note that computing gravity with position all the time in filter is not dangerous even if position is way off
            // max control error vtas [mps] -0.931 theta [deg] -1.755 xi [deg] -20.240 beta [deg] -5.459 Hp [m] +12.897 chi [deg] -2.320
            // mean-std nav error rotv [deg] +4.653e-01 +3.647e-01 w_nbb [dps] +9.365e-02 +7.547e-02 E_gyr [dps] +2.502e-03 +2.179e-03 E_mag [nT] +1.391e+01 +1.532e+01
            // ** 06: No wind, no offsets, turb 1.0, init time -50, variable Bn, no realism B yes gc, sensors complete, all init cond perfect, filter 01(01-01-01-02)
            //        Note that computing magnetism with position all the time in filter may be dangerous when no GPS if position is way off
            // max control error vtas [mps] -0.931 theta [deg] -1.754 xi [deg] -20.242 beta [deg] -5.459 Hp [m] +12.899 chi [deg] -2.320
            // mean-std nav error rotv [deg] +4.650e-01 +3.612e-01 w_nbb [dps] +9.365e-02 +7.540e-02 E_gyr [dps] +2.484e-03 +2.169e-03 E_mag [nT] +1.350e+01 +1.482e+01
            // ** 07: No wind, no offsets, turb 1.0, init time -50, variable Bn, realism in B and gc, sensors complete, all init cond perfect, filter 01(01-01-01-02)
            //        Results are not bad if they can be repeated with other seeds, backup option if nothing else works
            // max control error vtas [mps] -0.931 theta [deg] -1.747 xi [deg] -20.099 beta [deg] -5.456 Hp [m] +12.909 chi [deg] -2.314
            // mean-std nav error rotv [deg] +5.414e-01 +3.089e-01 w_nbb [dps] +9.359e-02 +7.522e-02 E_gyr [dps] +2.976e-03 +1.647e-03 E_mag [nT] +1.348e+01 +1.480e+01
            // ** 08: No wind, no offsets, turb 1.0, init time -50, variable Bn, realism in B and gc, sensors complete, all init cond perfect, filter 02(01-02-01-02)
            // max control error vtas [mps] -0.931 theta [deg] -1.754 xi [deg] -20.271 beta [deg] -5.459 Hp [m] +12.898 chi [deg] -2.332
            // mean-std nav error rotv [deg] +4.618e-01 +3.340e-01 w_nbb [dps] +9.365e-02 +7.550e-02 E_gyr [dps] +2.439e-03 +2.060e-03 E_mag [nT] +1.348e+01 +1.479e+01 error_mag [nT] +8.137e+00 +6.048e+00
            // ** 09: No wind, no offsets, turb 1.0, init time -50, variable Bn, realism in B and gc, sensors complete, init euler perfect, filter 02(01-02-01-02)
            // max control error vtas [mps] -0.931 theta [deg] -1.757 xi [deg] -20.205 beta [deg] -5.472 Hp [m] +12.934 chi [deg] -2.296
            // mean-std nav error rotv [deg] +5.431e-01 +3.646e-01 w_nbb [dps] +9.365e-02 +7.543e-02 E_gyr [dps] +2.535e-03 +2.063e-03 E_mag [nT] +2.096e+01 +9.865e+00 error_mag [nT] +1.822e+01 +6.306e+00
            // ** 10: No wind, no offsets, turb 1.0, init time -50, variable Bn, realism in B and gc, sensors complete, no perfect inits, filter 02(01-02-01-02)
            //        Changed att state e from 1e-3 to 1e-12 and att init a from 1e-10 to 1e-8 (may create some att oscillations after 2500 sec, watch out)
            // max control error vtas [mps] -0.931 theta [deg] -1.755 xi [deg] -20.214 beta [deg] -5.474 Hp [m] +12.949 chi [deg] -2.316
            // mean-std nav error rotv [deg] +6.705e-01 +3.712e-01 w_nbb [dps] +9.368e-02 +7.529e-02 E_gyr [dps] +3.192e-03 +2.065e-03 E_mag [nT] +2.733e+01 +2.143e+01 error_mag [nT] +7.079e+01 +2.153e+01
            // ** 11: Same as 10, but twice initial Euler error
            // max control error vtas [mps] -0.931 theta [deg] -1.753 xi [deg] -20.217 beta [deg] -5.476 Hp [m] +12.967 chi [deg] -2.331
            // mean-std nav error rotv [deg] +7.180e-01 +3.780e-01 w_nbb [dps] +9.369e-02 +7.528e-02 E_gyr [dps] +3.513e-03 +2.075e-03 E_mag [nT] +3.185e+01 +1.803e+01 error_mag [nT] +9.066e+01 +1.961e+01
            // ** 12: Same as 10, but turbulence 0.5 (improvement in results)
            // max control error vtas [mps] +0.529 theta [deg] +1.555 xi [deg] -20.316 beta [deg] +5.489 Hp [m] +7.353 chi [deg] -2.372
            // mean-std nav error rotv [deg] +5.635e-01 +3.409e-01 w_nbb [dps] +8.645e-02 +7.388e-02 E_gyr [dps] +2.743e-03 +1.807e-03 E_mag [nT] +2.239e+01 +1.488e+01 error_mag [nT] +5.908e+01 +1.442e+01
            //        Modifications #2 in filter_att (everything improves except w_nbb that gets worse)
            // mean-std nav error rotv [deg] +4.832e-01 +3.392e-01 w_nbb [dps] +2.966e-01 +3.299e-01 E_gyr [dps] +2.513e-03 +1.526e-03 E_mag [nT] +1.780e+01 +1.284e+01 error_mag [nT] +3.997e+01 +9.249e+00




            // NECESIDAD DE SUAVIZAR LOS GIROS

            auto Pguid = new control::guid(5, 500.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid);
            Pguid->_sti_id = st::logic::sti_h3000_tas30_psi30;
            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg,   30.0, rud_beta_deg, 0., trgg_t_sec,      100.0));
            Pguid->add_op(1, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,    20.0, rud_beta_deg, 0., trgg_chi_deg,   -150.0));
            Pguid->add_op(2, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, -150.0, rud_beta_deg, 0., trgg_Deltat_sec, 100.0));
            Pguid->add_op(3, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,    20.0, rud_beta_deg, 0., trgg_chi_deg,     30.0));
            Pguid->add_op(4, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg,   30.0, rud_beta_deg, 0., trgg_Deltat_sec, 100.0));
            return Pguid;
        }
        case 35: { // straight flight with two changes of altitude and one change of tas
            // Open: max errors are vtas [mps] -5.390 theta [deg] -6.132 xi [deg] -1.590 beta [deg] -1.244 Hp [m] 1.102 gammaTAS [deg] +2.718 chi [deg] -0.550 (seed 1)
            //       mean-std error vtas [mps] +6.50e-3 +1.11e-1 alpha [deg] +1.54e-3 +1.26e-1 beta [deg] -1.77e-3 +1.36e-1 Hp [m] +7.51e-2 +2.85e-1
            //       mean-std error rtov [deg] +4.62e-1 +3.49e-1 w_nbb [dps] +6.23e-2 +3.49e-2 bias_gyr [dps] +4.36e-3 +4.31e-3 bias_mag [nT] +5.56e+0 +1.67e+0
            // ** 01: No wind, no offsets, turb 1.0, init time -50, constant Bn, no realism gc nor B, ""old sensors with no M no R"", all init cond perfect, filter 01(01-01-01-02):
            // max control error vtas [mps] -5.382 theta [deg] +7.049 xi [deg] -2.355 beta [deg] -1.387 Hp [m] +16.724 gammaTAS [deg] +3.537 chi [deg] +0.894
            // mean-std nav error rotv [deg] +4.179e-01 +4.398e-01 w_nbb [dps] +7.068e-02 +3.742e-02 E_gyr [dps] +2.679e-03 +1.888e-03 E_mag [nT] +5.786e-10 +4.202e-10
            // CONTROL LONGITUDINAL TIENE PROBLEMAS AL VOLAR A ALTURA CONSTANTE - ESTO NO PUEDE SER
            // ** 02: No wind, no offsets, turb 1.0, init time -50, constant Bn, no realism gc nor B, ""sensors with no M no R"", all init cond perfect, filter 01(01-01-01-02),
            //        Added Eacc as input to filter_att, modified Hp control as 35 was unstable, modified pos filter system noise
            // max control error vtas [mps] -5.455 theta [deg] +4.568 xi [deg] -2.097 beta [deg] -1.387 Hp [m] +12.912 gammaTAS [deg] +3.265 chi [deg] -0.839
            // mean-std nav error rotv [deg] +3.019e-01 +2.634e-01 w_nbb [dps] +7.637e-02 +4.024e-02 E_gyr [dps] +2.582e-03 +1.528e-03 E_mag [nT] +5.060e-10 +3.748e-10
            // ** 03: No wind, no offsets, turb 1.0, init time -50, constant Bn, no realism gc nor B, ""gyr & acc with no R, mag complete"", all init cond perfect, filter 01(01-01-01-02)
            //        All differences due to magnetometer scale and cross coupling, nothing from gyroscope and accelerometer.
            //        Note that E_mag error was up a lot but continues to be very small. Does not matter because rotv error does not change
            //        Multiplied magnetometer observation noise by 10 in filter_att to solve attitude errors in 33 and 34
            // max control error vtas [mps] -5.454 theta [deg] +4.563 xi [deg] -2.090 beta [deg] -1.388 Hp [m] +12.897 gammaTAS [deg] +3.263 chi [deg] -0.828
            // mean-std nav error rotv [deg] +2.779e-01 +2.493e-01 w_nbb [dps] +7.633e-02 +4.026e-02 E_gyr [dps] +2.378e-03 +1.504e-03 E_mag [nT] +1.600e+00 +6.817e-01
            // ** 04: No wind, no offsets, turb 1.0, init time -50, constant Bn, no realism gc nor B, ""sensors complete"", all init cond perfect, filter 01(01-01-01-02)
            // max control error vtas [mps] -5.454 theta [deg] +4.563 xi [deg] -2.090 beta [deg] -1.388 Hp [m] +12.897 gammaTAS [deg] +3.263 chi [deg] -0.828
            // mean-std nav error rotv [deg] +2.779e-01 +2.492e-01 w_nbb [dps] +7.633e-02 +4.025e-02 E_gyr [dps] +2.378e-03 +1.504e-03 E_mag [nT] +1.600e+00 +6.817e-01
            // ** 05: No wind, no offsets, turb 1.0, init time -50, constant Bn, no realism B yes gc, sensors complete, all init cond perfect, filter 01(01-01-01-02)
            //        Note that computing gravity with position all the time in filter is not dangerous even if position is way off
            // max control error vtas [mps] -5.454 theta [deg] +4.563 xi [deg] -2.091 beta [deg] -1.388 Hp [m] +12.897 gammaTAS [deg] +3.263 chi [deg] -0.828
            // mean-std nav error rotv [deg] +2.771e-01 +2.483e-01 w_nbb [dps] +7.637e-02 +4.033e-02 E_gyr [dps] +2.375e-03 +1.503e-03 E_mag [nT] +1.601e+00 +6.807e-01
            // ** 06: No wind, no offsets, turb 1.0, init time -50, variable Bn, no realism B yes gc, sensors complete, all init cond perfect, filter 01(01-01-01-02)
            //        Note that computing magnetism with position all the time in filter may be dangerous when no GPS if position is way off
            // max control error vtas [mps] -5.453 theta [deg] +4.570 xi [deg] -2.091 beta [deg] -1.389 Hp [m] +12.899 gammaTAS [deg] +3.267 chi [deg] -0.832
            // mean-std nav error rotv [deg] +2.709e-01 +2.357e-01 w_nbb [dps] +7.630e-02 +4.022e-02 E_gyr [dps] +2.317e-03 +1.417e-03 E_mag [nT] +1.634e+00 +6.803e-01
            // ** 07: No wind, no offsets, turb 1.0, init time -50, variable Bn, realism in B and gc, sensors complete, all init cond perfect, filter 01(01-01-01-02)
            //        Results are not bad if they can be repeated with other seeds, backup option if nothing else works
            // max control error vtas [mps] -5.243 theta [deg] +4.770 xi [deg] -2.218 beta [deg] -1.386 Hp [m] +12.909 gammaTAS [deg] +3.364 chi [deg] -0.788
            // mean-std nav error rotv [deg] +4.356e-01 +2.188e-01 w_nbb [dps] +7.631e-02 +4.047e-02 E_gyr [dps] +2.040e-03 +1.132e-03 E_mag [nT] +1.646e+00 +6.870e-01
            // ** 08: No wind, no offsets, turb 1.0, init time -50, variable Bn, realism in B and gc, sensors complete, all init cond perfect, filter 02(01-02-01-02)
            // max control error vtas [mps] -5.454 theta [deg] +4.569 xi [deg] -2.091 beta [deg] -1.389 Hp [m] +12.898 gammaTAS [deg] +3.266 chi [deg] -0.832
            // mean-std nav error rotv [deg] +2.616e-01 +2.262e-01 w_nbb [dps] +7.630e-02 +4.023e-02 E_gyr [dps] +2.258e-03 +1.378e-03 E_mag [nT] +1.634e+00 +6.805e-01 error_mag [nT] +2.596e+00 +1.941e+00
            // ** 09: No wind, no offsets, turb 1.0, init time -50, variable Bn, realism in B and gc, sensors complete, init euler perfect, filter 02(01-02-01-02)
            // max control error vtas [mps] -5.425 theta [deg] -4.729 xi [deg] -2.033 beta [deg] -1.389 Hp [m] +12.934 gammaTAS [deg] +3.220 chi [deg] -0.863
            // mean-std nav error rotv [deg] +2.946e-01 +2.221e-01 w_nbb [dps] +7.648e-02 +4.074e-02 E_gyr [dps] +2.435e-03 +1.531e-03 E_mag [nT] +7.347e+00 +1.075e+00 error_mag [nT] +1.228e+01 +3.049e+00
            // ** 10: No wind, no offsets, turb 1.0, init time -50, variable Bn, realism in B and gc, sensors complete, no perfect inits, filter 02(01-02-01-02)
            //        Changed att state e from 1e-3 to 1e-12 and att init a from 1e-10 to 1e-8 (may create some att oscillations after 2500 sec, watch out)
            // max control error vtas [mps] -5.460 theta [deg] -4.713 xi [deg] -1.964 beta [deg] -1.390 Hp [m] +12.949 gammaTAS [deg] +3.243 chi [deg] -0.900
            // mean-std nav error rotv [deg] +2.658e-01 +1.912e-01 w_nbb [dps] +7.642e-02 +4.096e-02 E_gyr [dps] +2.573e-03 +1.265e-03 E_mag [nT] +1.142e+01 +7.346e+00 error_mag [nT] +6.720e+01 +1.731e+01
            // ** 11: Same as 10, but twice initial Euler error
            // max control error vtas [mps] -5.482  theta [deg] +4.571 xi [deg] -2.126 beta [deg] -1.390 Hp [m] +12.967 gammaTAS [deg] +3.267 chi [deg] -0.877
            // mean-std nav error rotv [deg] +3.633e-01 +2.351e-01 w_nbb [dps] +7.630e-02 +4.033e-02 E_gyr [dps] +2.958e-03 +1.374e-03 E_mag [nT] +1.275e+01 +6.998e+00 error_mag [nT] +8.712e+01 +1.648e+01
            // ** 12: Same as 10, but turbulence 0.5 (significant improvement in results)
            // max control error vtas [mps] -5.237 theta [deg] -4.400 xi [deg] -2.001 beta [deg] -0.772 Hp [m] +11.682 gammaTAS [deg] +2.705 chi [deg] -0.722
            // mean-std nav error rotv [deg] +1.939e-01 +1.493e-01 w_nbb [dps] +6.951e-02 +3.671e-02 E_gyr [dps] +2.127e-03 +1.060e-03 E_mag [nT] +6.928e+00 +2.802e+00 error_mag [nT] +5.264e+01 +9.335e+00
            //        Modifications #2 in filter_att (everything improves except w_nbb that gets worse)
            // mean-std nav error rotv [deg] +1.803e-01 +1.286e-01 w_nbb [dps] +1.959e-01 +1.583e-01 E_gyr [dps] +1.599e-03 +7.459e-04 E_mag [nT] +4.709e+00 +6.903e-01 error_mag [nT] +3.357e+01 +4.099e+00
            //        Modifications #3 in filter_att (seems worse than #1 in this case)
            // mean-std nav error rotv [deg] +2.233e-01 +1.588e-01 w_nbb [dps] +6.945e-02 +3.660e-02 E_gyr [dps] +2.086e-03 +1.252e-03 E_mag [nT] +6.366e+00 +1.589e+00 error_mag [nT] +4.527e+01 +6.723e+00





            auto Pguid = new control::guid(6, 800.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid);
            Pguid->_sti_id = st::logic::sti_h3000_tas30_psi30;
            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         3000.0, ail_chi_deg, 30.0, rud_beta_deg, 0., trgg_t_sec,       100.0));
            Pguid->add_op(1, new control::guid_op(thr_vtas_mps, 30.0, elv_gammaTAS_deg,    2.0, ail_chi_deg, 30.0, rud_beta_deg, 0., trgg_Hp_m,       3100.0));
            Pguid->add_op(2, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         3100.0, ail_chi_deg, 30.0, rud_beta_deg, 0., trgg_Deltat_sec,  100.0));
            Pguid->add_op(3, new control::guid_op(thr_vtas_mps, 25.0, elv_Hp_m,         3100.0, ail_chi_deg, 30.0, rud_beta_deg, 0., trgg_Deltat_sec,  100.0));
            Pguid->add_op(4, new control::guid_op(thr_vtas_mps, 25.0, elv_gammaTAS_deg,   -2.0, ail_chi_deg, 30.0, rud_beta_deg, 0., trgg_Hp_m,       3000.0));
            Pguid->add_op(5, new control::guid_op(thr_vtas_mps, 25.0, elv_Hp_m,         3000.0, ail_chi_deg, 30.0, rud_beta_deg, 0., trgg_Deltat_sec,  100.0));
            return Pguid;
        }
        case 36: { // long flight with one turn close to the beginning (similar to GPS out) - !!!!!! REQUIRES LONGER TURBULENCE TIME !!!!!!
            // ** 01: No wind, no offsets, turb 1.0, init time -50, constant Bn, no realism gc nor B, ""old sensors with no M no R"", all init cond perfect, filter 01(01-01-01-02):
            // max control error vtas [mps] -1.742 theta [deg] +1.528 xi [deg] -9.932 beta [deg] -2.403 Hp [m] +13.505 chi [deg] -1.037
            // mean-std nav error rotv [deg] +2.102e-01 +1.670e-01 w_nbb [dps] +7.732e-02 +3.699e-02 E_gyr [dps] +1.996e-03 +1.127e-03 E_mag [nT] +1.120e-08 +9.436e-09
            // ** 02: No wind, no offsets, turb 1.0, init time -50, constant Bn, no realism gc nor B, ""sensors with no M no R"", all init cond perfect, filter 01(01-01-01-02),
            //        Added Eacc as input to filter_att, modified Hp control as 35 was unstable, modified pos filter system noise
            // max control error vtas [mps] -1.155 theta [deg] +1.697 xi [deg] -9.946 beta [deg] +2.947 Hp [m] -22.583 chi [deg] -1.064
            // mean-std nav error rotv [deg] +2.136e-01 +1.775e-01 w_nbb [dps] +8.739e-02 +4.472e-02 E_gyr [dps] +1.912e-03 +1.045e-03 E_mag [nT] +1.048e-08 +8.953e-09
            // ** 03: No wind, no offsets, turb 1.0, init time -50, constant Bn, no realism gc nor B, ""gyr & acc with no R, mag complete"", all init cond perfect, filter 01(01-01-01-02)
            //        All differences due to magnetometer scale and cross coupling, nothing from gyroscope and accelerometer.
            //        Note that E_mag error was up a lot but continues to be very small. Does not matter because rotv error does not change.
            //        Multiplied magnetometer observation noise by 10 in filter_att to solve attitude errors in 33 and 34
            // max control error vtas [mps] -1.152 theta [deg] +1.697 xi [deg] -10.033 beta [deg] +2.921 Hp [m] -22.580 chi [deg] -1.096
            // mean-std nav error rotv [deg] +2.522e-01 +1.812e-01 w_nbb [dps] +8.737e-02 +4.470e-02 E_gyr [dps] +1.900e-03 +1.051e-03 E_mag [nT] +3.272e+01 +6.636e+00
            // ** 04: No wind, no offsets, turb 1.0, init time -50, constant Bn, no realism gc nor B, ""sensors complete"", all init cond perfect, filter 01(01-01-01-02)
            // max control error vtas [mps] -1.152 theta [deg] +1.697 xi [deg] -10.033 beta [deg] +2.921 Hp [m] -22.580 chi [deg] -1.096
            // mean-std nav error rotv [deg] +2.523e-01 +1.813e-01 w_nbb [dps] +8.736e-02 +4.470e-02 E_gyr [dps] +1.900e-03 +1.051e-03 E_mag [nT] +3.272e+01 +6.636e+00
            // ** 05: No wind, no offsets, turb 1.0, init time -50, constant Bn, no realism B yes gc, sensors complete, all init cond perfect, filter 01(01-01-01-02)
            //        Note that computing gravity with position all the time in filter is not dangerous even if position is way off
            // max control error vtas [mps] -1.152 theta [deg] +1.697 xi [deg] -9.978 beta [deg] +2.956 Hp [m] -22.580 chi [deg] -1.077
            // mean-std nav error rotv [deg] +2.514e-01 +1.807e-01 w_nbb [dps] +8.736e-02 +4.470e-02 E_gyr [dps] +1.899e-03 +1.050e-03 E_mag [nT] +3.273e+01 +6.637e+00
            // ** 06: No wind, no offsets, turb 1.0, init time -50, variable Bn, no realism B yes gc, sensors complete, all init cond perfect, filter 01(01-01-01-02)
            //        Note that computing magnetism with position all the time in filter may be dangerous when no GPS if position is way off
            // max control error vtas [mps] -1.152 theta [deg] +1.696 xi [deg] -9.974 beta [deg] +2.954 Hp [m] -22.589 chi [deg] -1.075
            // mean-std nav error rotv [deg] +2.571e-01 +1.824e-01 w_nbb [dps] +8.736e-02 +4.470e-02 E_gyr [dps] +1.904e-03 +1.053e-03 E_mag [nT] +3.446e+01 +6.975e+00
            // ** 07: No wind, no offsets, turb 1.0, init time -50, variable Bn, realism in B and gc, sensors complete, all init cond perfect, filter 01(01-01-01-02)
            //        Results are not bad if they can be repeated with other seeds, backup option if nothing else works
            // max control error vtas [mps] -1.151 theta [deg] +1.696 xi [deg] -10.135 beta [deg] +3.120 Hp [m] -22.560 chi [deg] -1.133
            // mean-std nav error rotv [deg] +5.299e-01 +2.798e-01 w_nbb [dps] +8.739e-02 +4.472e-02 E_gyr [dps] +2.596e-03 +1.429e-03 E_mag [nT] +3.473e+01 +7.021e+00
            // ** 08: No wind, no offsets, turb 1.0, init time -50, variable Bn, realism in B and gc, sensors complete, all init cond perfect, filter 02(01-02-01-02)
            // max control error vtas [mps] -1.151 theta [deg] +1.696 xi [deg] -9.975 beta [deg] +2.954 Hp [m] -22.573 chi [deg] -1.076
            // mean-std nav error rotv [deg] +3.216e-01 +2.144e-01 w_nbb [dps] +8.736e-02 +4.470e-02 E_gyr [dps] +1.947e-03 +1.075e-03 E_mag [nT] +3.473e+01 +7.022e+00 error_mag [nT] +3.786e+01 +2.141e+01
            // ** 09: No wind, no offsets, turb 1.0, init time -50, variable Bn, realism in B and gc, sensors complete, init euler perfect, filter 02(01-02-01-02)
            // max control error vtas [mps] -1.151 theta [deg] +1.696 xi [deg] -10.015 beta [deg] +3.092 Hp [m] -22.571 chi [deg] -1.093
            // mean-std nav error rotv [deg] +4.365e-01 +2.521e-01 w_nbb [dps] +8.736e-02 +4.471e-02 E_gyr [dps] +1.957e-03 +1.083e-03 E_mag [nT] +2.175e+01 +3.748e+00 error_mag [nT] +5.380e+01 +2.833e+01
            // ** 10: No wind, no offsets, turb 1.0, init time -50, variable Bn, realism in B and gc, sensors complete, no perfect inits, filter 02(01-02-01-02)
            //        Changed att state e from 1e-3 to 1e-12 and att init a from 1e-10 to 1e-8 (may create some att oscillations after 2500 sec, watch out)
            // max control error vtas [mps] -1.151 theta [deg] +1.696 xi [deg] -10.153 beta [deg] +3.134 Hp [m] -22.515 chi [deg] -1.133
            // mean-std nav error rotv [deg] +3.539e-01 +2.316e-01 w_nbb [dps] +8.743e-02 +4.473e-02 E_gyr [dps] +3.016e-03 +2.231e-03 E_mag [nT] +1.452e+02 +3.868e+01 error_mag [nT] +2.258e+02 +8.409e+01
            //        Modifications #4 in filter_att
            // mean-std nav error rotv [deg] +2.697e-01 +1.650e-01 w_nbb [dps] +3.114e-01 +1.926e-01 E_gyr [dps] +1.546e-03 +7.313e-04 E_mag [nT] +4.507e+01 +1.739e+01 error_mag [nT] +1.320e+02 +5.824e+01
            // ** 11: Same as 10, but twice initial Euler error
            // max control error vtas [mps] -1.151 theta [deg] +1.696 xi [deg] -10.094 beta [deg] +3.116 Hp [m] -22.518 chi [deg] -1.120
            // mean-std nav error rotv [deg] +3.549e-01 +2.171e-01 w_nbb [dps] +8.742e-02 +4.473e-02 E_gyr [dps] +2.921e-03 +2.113e-03 E_mag [nT] +1.464e+02 +3.716e+01 error_mag [nT] +2.407e+02 +7.982e+01
            // ** 12: Same as 10, but turbulence 0.5 (significant improvement in results, oscillations still exist but start later at 2500 sec, origin seems to be Egyr)
            // max control error vtas [mps] +0.626 theta [deg] +1.393 xi [deg] -10.175 beta [deg] +2.910 Hp [m] -12.193 chi [deg] -1.148
            // mean-std nav error rotv [deg] +1.843e-01 +9.607e-02 w_nbb [dps] +8.056e-02 +4.163e-02 E_gyr [dps] +1.723e-03 +9.001e-04 E_mag [nT] +9.085e+01 +2.327e+01 error_mag [nT] +1.429e+02 +4.486e+01
            //        Modifications #2 in filter_att (everything improves except w_nbb that gets worse)
            // mean-std nav error rotv [deg] +1.404e-01 +8.405e-02 w_nbb [dps] +2.656e-01 +1.823e-01 E_gyr [dps] +1.362e-03 +5.883e-04 E_mag [nT] +3.440e+01 +1.053e+01 error_mag [nT] +8.613e+01 +3.126e+01
            //        Modifications #3 in filter_att (same attitude as #2, no w_nbb worsening, and everything else in between)
            // mean-std nav error rotv [deg] +1.419e-01 +7.910e-02 w_nbb [dps] +8.054e-02 +4.163e-02 E_gyr [dps] +1.541e-03 +7.841e-04 E_mag [nT] +5.732e+01 +1.869e+01 error_mag [nT] +1.286e+02 +4.613e+01
            //        Modifications #4 in fitler_att
            // mean-std nav error rotv [deg] +1.488e-01 +9.180e-02 w_nbb [dps] +2.655e-01 +1.822e-01 E_gyr [dps] +1.334e-03 +5.749e-04 E_mag [nT] +2.300e+01 +6.701e+00 error_mag [nT] +8.033e+01 +3.033e+01


            auto Pguid = new control::guid(3, 3800.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid); // 3800
            Pguid->_sti_id = st::logic::sti_h3000_tas30_psi30;
            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg,  30.0, rud_beta_deg, 0., trgg_t_sec,    130.0));
            Pguid->add_op(1, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,   10.0, rud_beta_deg, 0., trgg_chi_deg,  120.0));
            Pguid->add_op(2, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 120.0, rud_beta_deg, 0., trgg_t_sec,   3800.0));
            return Pguid;
        }


        case 37: { // long flight with turns left and right - !!!!!! REQUIRES LONGER TURBULENCE TIME !!!!!!
                   // MAY NEED TO ALSO INCLUDE SOME CHANGE OF SPEED OR MOST LIKELY PATH ANGLE TOO
            // ** 10: No wind, no offsets, turb 1.0, init time -50, variable Bn, realism in B and gc, sensors complete, no perfect inits, filter 02(01-02-01-02)
            //        Changed att state e from 1e-3 to 1e-12 and att init a from 1e-10 to 1e-8 (may create some att oscillations after 2500 sec, watch out)
            //        Oscillations start more or less at 1500 sec, seem to come from Egyr3
            // max control error vtas [mps] +1.186 theta [deg] +1.695 xi [deg] -10.543 beta [deg] +3.502 Hp [m] -22.565 chi [deg] +1.186
            // mean-std nav error rotv [deg] +5.670e-01 +4.583e-01 w_nbb [dps] +9.046e-02 +5.580e-02 E_gyr [dps] +5.182e-03 +3.714e-03 E_mag [nT] +4.127e+01 +1.657e+01 error_mag [nT] +2.041e+02 +8.864e+01
            // ** 12: Same as 10, but turbulence 0.5 (significant improvement in results, oscillation present, origin seems to be Egyr3)
            // max control error vtas [mps] +0.667 theta [deg] +1.401 xi [deg] +10.342 beta [deg] +3.114 Hp [m] -12.219 chi [deg] -1.262
            // mean-std nav error rotv [deg] +2.405e-01 +1.750e-01 w_nbb [dps] +8.347e-02 +5.361e-02 E_gyr [dps] +2.308e-03 +1.193e-03 E_mag [nT] +2.686e+01 +1.078e+01 error_mag [nT] +1.351e+02 +5.094e+01
            //        Modifications #2 in filter_att (everything improves except w_nbb that gets worse)
            // mean-std nav error rotv [deg] +1.942e-01 +1.367e-01 w_nbb [dps] +2.809e-01 +2.381e-01 E_gyr [dps] +1.466e-03 +7.200e-04 E_mag [nT] +1.886e+01 +6.490e+00 error_mag [nT] +8.357e+01 +3.374e+01


            auto Pguid = new control::guid(48, 3800.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid); // 500
            Pguid->_sti_id = st::logic::sti_h3000_tas30_psi30;
            Pguid->add_op(0,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg,   30.0, rud_beta_deg, 0., trgg_t_sec,      100.0));
            Pguid->add_op(1,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,    10.0, rud_beta_deg, 0., trgg_chi_deg,     90.0));
            Pguid->add_op(2,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg,   90.0, rud_beta_deg, 0., trgg_Deltat_sec, 100.0));
            Pguid->add_op(3,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,   -10.0, rud_beta_deg, 0., trgg_chi_deg,     30.0));
            Pguid->add_op(4,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg,   30.0, rud_beta_deg, 0., trgg_Deltat_sec, 100.0));
            Pguid->add_op(5,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,    10.0, rud_beta_deg, 0., trgg_chi_deg,     90.0));
            Pguid->add_op(6,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg,   90.0, rud_beta_deg, 0., trgg_Deltat_sec, 100.0));
            Pguid->add_op(7,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,   -10.0, rud_beta_deg, 0., trgg_chi_deg,     30.0));
            Pguid->add_op(8,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg,   30.0, rud_beta_deg, 0., trgg_Deltat_sec, 100.0));
            Pguid->add_op(9,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,    10.0, rud_beta_deg, 0., trgg_chi_deg,     90.0));
            Pguid->add_op(10, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg,   90.0, rud_beta_deg, 0., trgg_Deltat_sec, 100.0));
            Pguid->add_op(11, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,   -10.0, rud_beta_deg, 0., trgg_chi_deg,     30.0));
            Pguid->add_op(12, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg,   30.0, rud_beta_deg, 0., trgg_Deltat_sec, 100.0));
            Pguid->add_op(13, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,    10.0, rud_beta_deg, 0., trgg_chi_deg,     90.0));
            Pguid->add_op(14, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg,   90.0, rud_beta_deg, 0., trgg_Deltat_sec, 100.0));
            Pguid->add_op(15, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,   -10.0, rud_beta_deg, 0., trgg_chi_deg,     30.0));
            Pguid->add_op(16, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg,   30.0, rud_beta_deg, 0., trgg_Deltat_sec, 100.0));
            Pguid->add_op(17, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,    10.0, rud_beta_deg, 0., trgg_chi_deg,     90.0));
            Pguid->add_op(18, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg,   90.0, rud_beta_deg, 0., trgg_Deltat_sec, 100.0));
            Pguid->add_op(19, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,   -10.0, rud_beta_deg, 0., trgg_chi_deg,     30.0));
            Pguid->add_op(20, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg,   30.0, rud_beta_deg, 0., trgg_Deltat_sec, 100.0));
            Pguid->add_op(21, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,    10.0, rud_beta_deg, 0., trgg_chi_deg,     90.0));
            Pguid->add_op(22, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg,   90.0, rud_beta_deg, 0., trgg_Deltat_sec, 100.0));
            Pguid->add_op(23, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,   -10.0, rud_beta_deg, 0., trgg_chi_deg,     30.0));
            Pguid->add_op(24, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg,   30.0, rud_beta_deg, 0., trgg_Deltat_sec, 100.0));
            Pguid->add_op(25, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,    10.0, rud_beta_deg, 0., trgg_chi_deg,     90.0));
            Pguid->add_op(26, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg,   90.0, rud_beta_deg, 0., trgg_Deltat_sec, 100.0));
            Pguid->add_op(27, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,   -10.0, rud_beta_deg, 0., trgg_chi_deg,     30.0));
            Pguid->add_op(28, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg,   30.0, rud_beta_deg, 0., trgg_Deltat_sec, 100.0));
            Pguid->add_op(29, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,    10.0, rud_beta_deg, 0., trgg_chi_deg,     90.0));
            Pguid->add_op(30, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg,   90.0, rud_beta_deg, 0., trgg_Deltat_sec, 100.0));
            Pguid->add_op(31, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,   -10.0, rud_beta_deg, 0., trgg_chi_deg,     30.0));
            Pguid->add_op(32, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg,   30.0, rud_beta_deg, 0., trgg_Deltat_sec, 100.0));
            Pguid->add_op(33, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,    10.0, rud_beta_deg, 0., trgg_chi_deg,     90.0));
            Pguid->add_op(34, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg,   90.0, rud_beta_deg, 0., trgg_Deltat_sec, 100.0));
            Pguid->add_op(35, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,   -10.0, rud_beta_deg, 0., trgg_chi_deg,     30.0));
            Pguid->add_op(36, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg,   30.0, rud_beta_deg, 0., trgg_Deltat_sec, 100.0));
            Pguid->add_op(37, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,    10.0, rud_beta_deg, 0., trgg_chi_deg,     90.0));
            Pguid->add_op(38, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg,   90.0, rud_beta_deg, 0., trgg_Deltat_sec, 100.0));
            Pguid->add_op(39, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,   -10.0, rud_beta_deg, 0., trgg_chi_deg,     30.0));
            Pguid->add_op(40, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg,   30.0, rud_beta_deg, 0., trgg_Deltat_sec, 100.0));
            Pguid->add_op(41, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,    10.0, rud_beta_deg, 0., trgg_chi_deg,     90.0));
            Pguid->add_op(42, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg,   90.0, rud_beta_deg, 0., trgg_Deltat_sec, 100.0));
            Pguid->add_op(43, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,   -10.0, rud_beta_deg, 0., trgg_chi_deg,     30.0));
            Pguid->add_op(44, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg,   30.0, rud_beta_deg, 0., trgg_Deltat_sec, 100.0));
            Pguid->add_op(45, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,    10.0, rud_beta_deg, 0., trgg_chi_deg,     90.0));
            Pguid->add_op(46, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg,   90.0, rud_beta_deg, 0., trgg_Deltat_sec, 100.0));
            Pguid->add_op(47, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,   -10.0, rud_beta_deg, 0., trgg_chi_deg,     30.0));
            return Pguid;
        }


        case 38: { // long flight with one turn close to the beginning, and then full turns every 500 sec - !!!!!! REQUIRES LONGER TURBULENCE TIME !!!!!!
            // ** 10: No wind, no offsets, turb 1.0, init time -50, variable Bn, realism in B and gc, sensors complete, no perfect inits, filter 02(01-02-01-02)
            //        Changed att state e from 1e-3 to 1e-12 and att init a from 1e-10 to 1e-8 (may create some att oscillations after 2500 sec, watch out)
            //        Has oscillations, but look controlled at every turn.  Maybe results not as good as flying straight,
            // max control error vtas [mps] -1.156 theta [deg] +1.696 xi [deg] -10.409 beta [deg] +3.134 Hp [m] -22.501 chi [deg] -1.171
            // mean-std nav error rotv [deg] +4.420e-01 +2.959e-01 w_nbb [dps] +8.791e-02 +4.674e-02 E_gyr [dps] +3.971e-03 +2.320e-03 E_mag [nT] +8.913e+01 +3.832e+01 error_mag [nT] +2.315e+02 +8.873e+01
            // ** 12: Same as 10, but turbulence 0.5
            //        Much better results as it eliminates oscillations. Maybe results not as good as flying straight, but far more stable
            // max control error vtas [mps] +0.626 theta [deg] +1.393 xi [deg] -10.269 beta [deg] +2.910 Hp [m] -12.159 chi [deg]-1.150
            // mean-std nav error rotv [deg] +2.280e-01 +1.082e-01 w_nbb [dps] +8.102e-02 +4.382e-02 E_gyr [dps] +2.196e-03 +1.539e-03 E_mag [nT] +5.441e+01 +2.836e+01 error_mag [nT] +1.470e+02 +4.857e+01
            //        Modifications #2 in filter_att (everything improves except w_nbb that gets worse)
            // mean-std nav error rotv [deg] +1.967e-01 +1.089e-01 w_nbb [dps] +2.680e-01 +1.930e-01 E_gyr [dps] +1.562e-03 +6.457e-04 E_mag [nT] +2.605e+01 +1.973e+01 error_mag [nT] +9.186e+01 +3.546e+01


            auto Pguid = new control::guid(18, 3800.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid); // 3800
            Pguid->_sti_id = st::logic::sti_h3000_tas30_psi30;
            Pguid->add_op(0,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg,  30.0, rud_beta_deg, 0., trgg_t_sec,      130.0));
            Pguid->add_op(1,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,   10.0, rud_beta_deg, 0., trgg_chi_deg,    120.0));
            Pguid->add_op(2,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 120.0, rud_beta_deg, 0., trgg_Deltat_sec, 500.0));
            Pguid->add_op(3,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,   10.0, rud_beta_deg, 0., trgg_chi_deg,    -60.0));
            Pguid->add_op(4,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,   10.0, rud_beta_deg, 0., trgg_chi_deg,    120.0));
            Pguid->add_op(5,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 120.0, rud_beta_deg, 0., trgg_Deltat_sec, 500.0));
            Pguid->add_op(6,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,   10.0, rud_beta_deg, 0., trgg_chi_deg,    -60.0));
            Pguid->add_op(7,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,   10.0, rud_beta_deg, 0., trgg_chi_deg,    120.0));
            Pguid->add_op(8,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 120.0, rud_beta_deg, 0., trgg_Deltat_sec, 500.0));
            Pguid->add_op(9,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,   10.0, rud_beta_deg, 0., trgg_chi_deg,    -60.0));
            Pguid->add_op(10, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,   10.0, rud_beta_deg, 0., trgg_chi_deg,    120.0));
            Pguid->add_op(11, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 120.0, rud_beta_deg, 0., trgg_Deltat_sec, 500.0));
            Pguid->add_op(12, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,   10.0, rud_beta_deg, 0., trgg_chi_deg,    -60.0));
            Pguid->add_op(13, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,   10.0, rud_beta_deg, 0., trgg_chi_deg,    120.0));
            Pguid->add_op(14, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 120.0, rud_beta_deg, 0., trgg_Deltat_sec, 500.0));
            Pguid->add_op(15, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,   10.0, rud_beta_deg, 0., trgg_chi_deg,    -60.0));
            Pguid->add_op(16, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,   10.0, rud_beta_deg, 0., trgg_chi_deg,    120.0));
            Pguid->add_op(17, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 120.0, rud_beta_deg, 0., trgg_t_sec,     3800.0));
            return Pguid;
        }

        case 39: { // long flight with one turn close to the beginning, and then ups and downs every 500 sec - !!!!!! REQUIRES LONGER TURBULENCE TIME !!!!!!
            // ** 10: No wind, no offsets, turb 1.0, init time -50, variable Bn, realism in B and gc, sensors complete, no perfect inits, filter 02(01-02-01-02)
            //        Changed att state e from 1e-3 to 1e-12 and att init a from 1e-10 to 1e-8 (may create some att oscillations after 2500 sec, watch out)
            // max control error vtas [mps] -1.563 theta [deg] -10.418 xi [deg] -10.153 beta [deg] +3.134 Hp [m] -22.210 gammaTAS [deg] -6.007 chi [deg] -1.133
            // mean-std nav error rotv [deg] +3.451e-01 +2.333e-01 w_nbb [dps] +8.413e-02 +4.601e-02 E_gyr [dps] +2.897e-03 +2.201e-03 E_mag [nT] +1.448e+02 +3.929e+01 error_mag [nT] +2.152e+02 +7.798e+01
            // ** 12: Same as 10, but turbulence 0.5 (much better results but it does not elliminate oscillations)
            // max control error vtas [mps] -0.765 theta [deg] +3.838 xi [deg] -10.175 beta [deg] +2.910 Hp [m] -12.128 gammaTAS [deg] +2.847 chi [deg] -1.148
            // mean-std nav error rotv [deg] +1.819e-01 +9.553e-02 w_nbb [dps] +7.741e-02 +4.106e-02 E_gyr [dps] +1.688e-03 +9.075e-04 E_mag [nT] +8.942e+01 +2.318e+01 error_mag [nT] +1.339e+02 +3.957e+01
            //        Modifications #2 in filter_att (everything improves except w_nbb that gets worse)
            // mean-std nav error rotv [deg] +1.359e-01 +8.346e-02 w_nbb [dps] +2.448e-01 +1.809e-01 E_gyr [dps] +1.361e-03 +5.865e-04 E_mag [nT] +3.342e+01 +1.005e+01 error_mag [nT] +8.029e+01 +2.765e+01


            auto Pguid = new control::guid(15, 3800.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid); // 3800
            Pguid->_sti_id = st::logic::sti_h3000_tas30_psi30;
            Pguid->add_op(0,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         3000.0, ail_chi_deg,  30.0, rud_beta_deg, 0., trgg_t_sec,       130.0));
            Pguid->add_op(1,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         3000.0, ail_xi_deg,   10.0, rud_beta_deg, 0., trgg_chi_deg,     120.0));
            Pguid->add_op(2,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         3000.0, ail_chi_deg, 120.0, rud_beta_deg, 0., trgg_Deltat_sec,  500.0));
            Pguid->add_op(3,  new control::guid_op(thr_vtas_mps, 30.0, elv_gammaTAS_deg,    2.0, ail_chi_deg, 120.0, rud_beta_deg, 0., trgg_Hp_m,       3100.0));
            Pguid->add_op(4,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         3100.0, ail_chi_deg, 120.0, rud_beta_deg, 0., trgg_Deltat_sec,  500.0));
            Pguid->add_op(5,  new control::guid_op(thr_vtas_mps, 30.0, elv_gammaTAS_deg,   -2.0, ail_chi_deg, 120.0, rud_beta_deg, 0., trgg_Hp_m,       3000.0));
            Pguid->add_op(6,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         3000.0, ail_chi_deg, 120.0, rud_beta_deg, 0., trgg_Deltat_sec,  500.0));
            Pguid->add_op(7,  new control::guid_op(thr_vtas_mps, 30.0, elv_gammaTAS_deg,    2.0, ail_chi_deg, 120.0, rud_beta_deg, 0., trgg_Hp_m,       3100.0));
            Pguid->add_op(8,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         3100.0, ail_chi_deg, 120.0, rud_beta_deg, 0., trgg_Deltat_sec,  500.0));
            Pguid->add_op(9,  new control::guid_op(thr_vtas_mps, 30.0, elv_gammaTAS_deg,   -2.0, ail_chi_deg, 120.0, rud_beta_deg, 0., trgg_Hp_m,       3000.0));
            Pguid->add_op(10, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         3000.0, ail_chi_deg, 120.0, rud_beta_deg, 0., trgg_Deltat_sec,  500.0));
            Pguid->add_op(11, new control::guid_op(thr_vtas_mps, 30.0, elv_gammaTAS_deg,    2.0, ail_chi_deg, 120.0, rud_beta_deg, 0., trgg_Hp_m,       3100.0));
            Pguid->add_op(12, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         3100.0, ail_chi_deg, 120.0, rud_beta_deg, 0., trgg_Deltat_sec,  500.0));
            Pguid->add_op(13, new control::guid_op(thr_vtas_mps, 30.0, elv_gammaTAS_deg,   -2.0, ail_chi_deg, 120.0, rud_beta_deg, 0., trgg_Hp_m,       3000.0));
            Pguid->add_op(14, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         3000.0, ail_chi_deg, 120.0, rud_beta_deg, 0., trgg_t_sec,      3800.0));
            return Pguid;
        }

        case 40: { // long flight with one turn close to the beginning, and then full turns followed by ups and downs every 500 sec - !!!!!! REQUIRES LONGER TURBULENCE TIME !!!!!!
            // ** 10: No wind, no offsets, turb 1.0, init time -50, variable Bn, realism in B and gc, sensors complete, no perfect inits, filter 02(01-02-01-02)
            //        Changed att state e from 1e-3 to 1e-12 and att init a from 1e-10 to 1e-8 (may create some att oscillations after 2500 sec, watch out)
            // max control error
            // mean-std nav error
            // ** 12: Same as 10, but turbulence 0.5 (worse than 38 as there are still oscillations and errors are bigger)
            // max control error vtas [mps] +1.113 theta [deg] +7.231 xi [deg] +10.366 beta [deg] +3.066 Hp [m] -12.726 gammaTAS [deg] +4.605 chi [deg] +1.204
            // mean-std nav error rotv [deg] +2.629e-01 +1.556e-01 w_nbb [dps] +7.875e-02 +4.393e-02 E_gyr [dps] +2.703e-03 +1.433e-03 E_mag [nT] +5.218e+01 +2.394e+01 error_mag [nT] +1.395e+02 +4.218e+01
            //        Modifications #2 in filter_att (everything improves except w_nbb that gets worse)
            // mean-std nav error rotv [deg] +1.725e-01 +1.001e-01 w_nbb [dps] +2.530e-01 +1.958e-01 E_gyr [dps] +1.532e-03 +6.549e-04 E_mag [nT] +2.545e+01 +1.636e+01 error_mag [nT] +8.647e+01 +3.028e+01



            auto Pguid = new control::guid(28, 3800.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid); // 3800
            Pguid->_sti_id = st::logic::sti_h3000_tas30_psi30;
            Pguid->add_op(0,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         3000.0, ail_chi_deg,  30.0, rud_beta_deg, 0., trgg_t_sec,       130.0));
            Pguid->add_op(1,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         3000.0, ail_xi_deg,   10.0, rud_beta_deg, 0., trgg_chi_deg,     120.0));
            Pguid->add_op(2,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         3000.0, ail_chi_deg, 120.0, rud_beta_deg, 0., trgg_Deltat_sec,  400.0));
            Pguid->add_op(3,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         3000.0, ail_xi_deg,   10.0, rud_beta_deg, 0., trgg_chi_deg,     -60.0));
            Pguid->add_op(4,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         3000.0, ail_xi_deg,   10.0, rud_beta_deg, 0., trgg_chi_deg,     120.0));
            Pguid->add_op(5,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         3000.0, ail_chi_deg, 120.0, rud_beta_deg, 0., trgg_Deltat_sec,  100.0));
            Pguid->add_op(6,  new control::guid_op(thr_vtas_mps, 30.0, elv_gammaTAS_deg,    2.0, ail_chi_deg, 120.0, rud_beta_deg, 0., trgg_Hp_m,       3100.0));
            Pguid->add_op(7,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         3100.0, ail_chi_deg, 120.0, rud_beta_deg, 0., trgg_Deltat_sec,  400.0));
            Pguid->add_op(8,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         3100.0, ail_xi_deg,  -10.0, rud_beta_deg, 0., trgg_chi_deg,     -60.0));
            Pguid->add_op(9,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         3100.0, ail_xi_deg,  -10.0, rud_beta_deg, 0., trgg_chi_deg,     120.0));
            Pguid->add_op(10, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         3100.0, ail_chi_deg, 120.0, rud_beta_deg, 0., trgg_Deltat_sec,  100.0));
            Pguid->add_op(11, new control::guid_op(thr_vtas_mps, 30.0, elv_gammaTAS_deg,   -2.0, ail_chi_deg, 120.0, rud_beta_deg, 0., trgg_Hp_m,       3000.0));
            Pguid->add_op(12, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         3000.0, ail_chi_deg, 120.0, rud_beta_deg, 0., trgg_Deltat_sec,  400.0));
            Pguid->add_op(13, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         3000.0, ail_xi_deg,   10.0, rud_beta_deg, 0., trgg_chi_deg,     -60.0));
            Pguid->add_op(14, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         3000.0, ail_xi_deg,   10.0, rud_beta_deg, 0., trgg_chi_deg,     120.0));
            Pguid->add_op(15, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         3000.0, ail_chi_deg, 120.0, rud_beta_deg, 0., trgg_Deltat_sec,  100.0));
            Pguid->add_op(16, new control::guid_op(thr_vtas_mps, 30.0, elv_gammaTAS_deg,    2.0, ail_chi_deg, 120.0, rud_beta_deg, 0., trgg_Hp_m,       3100.0));
            Pguid->add_op(17, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         3100.0, ail_chi_deg, 120.0, rud_beta_deg, 0., trgg_Deltat_sec,  400.0));
            Pguid->add_op(18, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         3100.0, ail_xi_deg,  -10.0, rud_beta_deg, 0., trgg_chi_deg,     -60.0));
            Pguid->add_op(19, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         3100.0, ail_xi_deg,  -10.0, rud_beta_deg, 0., trgg_chi_deg,     120.0));
            Pguid->add_op(20, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         3100.0, ail_chi_deg, 120.0, rud_beta_deg, 0., trgg_Deltat_sec,  100.0));
            Pguid->add_op(21, new control::guid_op(thr_vtas_mps, 30.0, elv_gammaTAS_deg,   -2.0, ail_chi_deg, 120.0, rud_beta_deg, 0., trgg_Hp_m,       3000.0));
            Pguid->add_op(22, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         3000.0, ail_chi_deg, 120.0, rud_beta_deg, 0., trgg_Deltat_sec,  400.0));
            Pguid->add_op(23, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         3000.0, ail_xi_deg,   10.0, rud_beta_deg, 0., trgg_chi_deg,     -60.0));
            Pguid->add_op(24, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         3000.0, ail_xi_deg,   10.0, rud_beta_deg, 0., trgg_chi_deg,     120.0));
            Pguid->add_op(25, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         3000.0, ail_chi_deg, 120.0, rud_beta_deg, 0., trgg_Deltat_sec,  100.0));
            Pguid->add_op(26, new control::guid_op(thr_vtas_mps, 30.0, elv_gammaTAS_deg,    2.0, ail_chi_deg, 120.0, rud_beta_deg, 0., trgg_Hp_m,       3100.0));
            Pguid->add_op(27, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         3100.0, ail_chi_deg, 120.0, rud_beta_deg, 0., trgg_t_sec,      3800.0));
            return Pguid;
        }

        // ========================================================
        // ===== ===== ===== Control fine tuning ===== ===== =====
        // ========================================================

        case 51: { // turn initiation based on absolute speed bearing
            auto Pguid = new control::guid(3, 250.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid);
            Pguid->_sti_id = st::logic::sti_HpXX_tasXX_chiXX;
            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg,  30.0, rud_beta_deg, 0., trgg_t_sec,   50.0));
            Pguid->add_op(1, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,   10.0, rud_beta_deg, 0., trgg_t_sec,  150.0));
            Pguid->add_op(2, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,  -10.0, rud_beta_deg, 0., trgg_t_sec,  250.0));
            return Pguid;
        }
        case 52: { // turn initiation based on body heading
            auto Pguid = new control::guid(3, 250.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid);
            Pguid->_sti_id = st::logic::sti_HpXX_tasXX_psiXX;
            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_psi_deg, 30.0, rud_beta_deg, 0., trgg_t_sec, 50.0));
            Pguid->add_op(1, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,  10.0, rud_beta_deg, 0., trgg_t_sec, 150.0));
            Pguid->add_op(2, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg, -10.0, rud_beta_deg, 0., trgg_t_sec, 250.0));
            return Pguid;
        }
        case 53: { // speed changes
            auto Pguid = new control::guid(3, 250.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid);
            Pguid->_sti_id = st::logic::sti_HpXX_tasXX_chiXX;
            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 30.0, rud_beta_deg, 0., trgg_t_sec,   50.0));
            Pguid->add_op(1, new control::guid_op(thr_vtas_mps, 34.0, elv_Hp_m, 3000.0, ail_chi_deg, 30.0, rud_beta_deg, 0., trgg_t_sec,  150.0));
            Pguid->add_op(2, new control::guid_op(thr_vtas_mps, 26.0, elv_Hp_m, 3000.0, ail_chi_deg, 30.0, rud_beta_deg, 0., trgg_t_sec,  250.0));
            return Pguid;
        }
        case 54: { // body pitch changes
            auto Pguid = new control::guid(3, 250.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid);
            Pguid->_sti_id = st::logic::sti_HpXX_tasXX_chiXX;
            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,      3000.0, ail_chi_deg, 30.0, rud_beta_deg, 0., trgg_t_sec,   50.0));
            Pguid->add_op(1, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg,   +4.0, ail_chi_deg, 30.0, rud_beta_deg, 0., trgg_t_sec,  150.0));
            Pguid->add_op(2, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg,   -4.0, ail_chi_deg, 30.0, rud_beta_deg, 0., trgg_t_sec,  250.0));
            return Pguid;
        }
        case 55: { // turn conclusion based on absolute speed bearing (note trigger 5 deg less than next op)
            auto Pguid = new control::guid(3, 250.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid);
            Pguid->_sti_id = st::logic::sti_HpXX_tasXX_chiXX;
            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg,  30.0, rud_beta_deg, 0., trgg_t_sec,    50.0));
            Pguid->add_op(1, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,   10.0, rud_beta_deg, 0., trgg_chi_deg, 115.0));
            Pguid->add_op(2, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 120.0, rud_beta_deg, 0., trgg_t_sec,   250.0));
            return Pguid;
        }
        case 56: { // turn conclusion based on body heading (note trigger 5 deg less than next op)
            auto Pguid = new control::guid(3, 250.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid);
            Pguid->_sti_id = st::logic::sti_HpXX_tasXX_psiXX;
            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_psi_deg,  30.0, rud_beta_deg, 0., trgg_t_sec,    50.0));
            Pguid->add_op(1, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,   10.0, rud_beta_deg, 0., trgg_psi_deg, 115.0));
            Pguid->add_op(2, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_psi_deg, 120.0, rud_beta_deg, 0., trgg_t_sec,   250.0));
            return Pguid;
        }
        case 57: { // altitude changes changes (pressure altitude)
            auto Pguid = new control::guid(3, 250.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid);
            Pguid->_sti_id = st::logic::sti_HpXX_tasXX_chiXX;
            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         3000.0, ail_chi_deg, 30.0, rud_beta_deg, 0., trgg_t_sec,   50.0));
            Pguid->add_op(1, new control::guid_op(thr_vtas_mps, 30.0, elv_gammaTAS_deg,   +2.0, ail_chi_deg, 30.0, rud_beta_deg, 0., trgg_Hp_m,  3100.0));
            Pguid->add_op(2, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         3100.0, ail_chi_deg, 30.0, rud_beta_deg, 0., trgg_t_sec,  250.0));
            return Pguid;
        }
        case 58: {
            auto Pguid = new control::guid(3, 250.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid);
            Pguid->_sti_id = st::logic::sti_HpXX_tasXX_chiXX;
            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 29.0, elv_Hp_m, 2700.0, ail_chi_deg,  30.0, rud_beta_deg, 0., trgg_t_sec,    50.0));
            Pguid->add_op(1, new control::guid_op(thr_vtas_mps, 29.0, elv_Hp_m, 2700.0, ail_xi_deg,   20.0, rud_beta_deg, 0., trgg_chi_deg, 215.0));
            Pguid->add_op(2, new control::guid_op(thr_vtas_mps, 29.0, elv_Hp_m, 2700.0, ail_chi_deg, 220.0, rud_beta_deg, 0., trgg_t_sec,   250.0));
            return Pguid;
        }



        // ===========================================================
        // ===== ===== ===== GPS Denied Trajectories ===== ===== =====
        // ===========================================================

        case 61: { // same as 36, loses GPS, turns and maintains body heading - !!!!!! REQUIRES LONGER TURBULENCE TIME !!!!!!
            // Results from #36 above turbulence 1.0 (with filter_att modification #4):
            // mean-std nav error rotv [deg] +2.697e-01 +1.650e-01 w_nbb [dps] +3.114e-01 +1.926e-01 E_gyr [dps] +1.546e-03 +7.313e-04 E_mag [nT] +4.507e+01 +1.739e+01 error_mag [nT] +1.320e+02 +5.824e+01
            // Results here:
            // mean-st nav error  rotv [deg] +2.123e-01 +1.222e-01 w_nbb [dps] +3.091e-01 +1.938e-01 E_gyr [dps] +1.546e-03 +7.141e-04 E_mag [nT] +5.144e+01 +1.927e+01 error_mag [nT] +1.256e+02 +5.495e+01
            // Results from #36 above turbulence 0.5 (with filter_att modification #4):
            // mean-std nav error rotv [deg] +1.488e-01 +9.180e-02 w_nbb [dps] +2.655e-01 +1.822e-01 E_gyr [dps] +1.334e-03 +5.749e-04 E_mag [nT] +2.300e+01 +6.701e+00 error_mag [nT] +8.033e+01 +3.033e+01
            // Results here:
            // mean-std nav error rotv [deg] +1.114e-01 +6.602e-02 w_nbb [dps] +2.644e-01 +1.829e-01 E_gyr [dps] +1.308e-03 +5.455e-04 E_mag [nT] +2.604e+01 +7.904e+00 error_mag [nT] +7.435e+01 +2.771e+01

            auto Pguid = new control::guid(3, 3800.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid); // 3800
            Pguid->_sti_id = st::logic::sti_h3000_tas30_psi30;
            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg,  30.0, rud_beta_deg, 0., trgg_t_sec,    130.0));
            Pguid->add_op(1, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,   10.0, rud_beta_deg, 0., trgg_psi_deg,  120.0));
            Pguid->add_op(2, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_psi_deg, 120.0, rud_beta_deg, 0., trgg_t_sec,   3800.0));
            return Pguid;
        }

        // ===========================================================
        // ===== ===== ===== Moses Lake Trajectories ===== ===== =====
        // ===========================================================

        case 91: { // fly-over over runway with initial turn
            auto Pguid = new control::guid(5, 1000.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid); // 200
            Pguid->_sti_id = st::logic::sti_moses;
            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         1100.0, ail_chi_deg, -90.0, rud_beta_deg, 0., trgg_t_sec,        5.0));
            Pguid->add_op(1, new control::guid_op(thr_vtas_mps, 30.0, elv_gammaTAS_deg,   -4.0, ail_chi_deg, -90.0, rud_beta_deg, 0., trgg_Deltat_sec,  45.0));
            Pguid->add_op(2, new control::guid_op(thr_vtas_mps, 30.0, elv_gammaTAS_deg,   -4.0, ail_xi_deg,   10.0, rud_beta_deg, 0., trgg_chi_deg,    -20.0));
            Pguid->add_op(3, new control::guid_op(thr_vtas_mps, 30.0, elv_gammaTAS_deg,   -4.0, ail_chi_deg, -20.0, rud_beta_deg, 0., trgg_Hp_m,       700.0));
            Pguid->add_op(4, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,          700.0, ail_chi_deg, -20.0, rud_beta_deg, 0., trgg_t_sec,      400.0));
            return Pguid;
        }
        case 92: { // fly-over over runway with two initial turns
            auto Pguid = new control::guid(7, 5000.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid); // 200
            Pguid->_sti_id = st::logic::sti_moses2;
            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         1100.0, ail_chi_deg,   0.0, rud_beta_deg, 0., trgg_t_sec,       10.0));
            Pguid->add_op(1, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         1100.0, ail_xi_deg,  -10.0, rud_beta_deg, 0., trgg_chi_deg,    -90.0));
            Pguid->add_op(2, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         1100.0, ail_chi_deg, -90.0, rud_beta_deg, 0., trgg_Deltat_sec,  60.0));
            Pguid->add_op(3, new control::guid_op(thr_vtas_mps, 30.0, elv_gammaTAS_deg,   -4.0, ail_chi_deg, -90.0, rud_beta_deg, 0., trgg_Deltat_sec,  45.0));
            Pguid->add_op(4, new control::guid_op(thr_vtas_mps, 30.0, elv_gammaTAS_deg,   -4.0, ail_xi_deg,   10.0, rud_beta_deg, 0., trgg_chi_deg,    -20.0));
            Pguid->add_op(5, new control::guid_op(thr_vtas_mps, 30.0, elv_gammaTAS_deg,   -4.0, ail_chi_deg, -20.0, rud_beta_deg, 0., trgg_Hp_m,       700.0));
            Pguid->add_op(6, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,          700.0, ail_chi_deg, -20.0, rud_beta_deg, 0., trgg_t_sec,      550.0));
            return Pguid;
        }
        case 93: { // approximate reversal of 91
            auto Pguid = new control::guid(5, 1000.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid);
            Pguid->_sti_id = st::logic::sti_moses3;
            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,          700.0, ail_chi_deg, 160.0, rud_beta_deg, 0., trgg_t_sec, 300.0));
            Pguid->add_op(1, new control::guid_op(thr_vtas_mps, 30.0, elv_gammaTAS_deg,    4.0, ail_chi_deg, 160.0, rud_beta_deg, 0., trgg_Hp_m,        1000.0));
            Pguid->add_op(2, new control::guid_op(thr_vtas_mps, 30.0, elv_gammaTAS_deg,    4.0, ail_xi_deg,  -10.0, rud_beta_deg, 0., trgg_chi_deg,       90.0));
            Pguid->add_op(3, new control::guid_op(thr_vtas_mps, 30.0, elv_gammaTAS_deg,    4.0, ail_chi_deg,  90.0, rud_beta_deg, 0., trgg_Hp_m,        1100.0));
            Pguid->add_op(4, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         1100.0, ail_chi_deg,  90.0, rud_beta_deg, 0., trgg_Deltat_sec,   100.0));
            return Pguid;
        }

            // ======================================================
            // ===== ===== ===== Rozas Trajectories ===== ===== =====
            // ======================================================

        case 96: { // fly closing hipodrome over Rozas (airport is at 438 m over sea level, fly area ends at 4500 ft = 1370 m)
            auto Pguid = new control::guid(23, 1000.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid); // 200
            Pguid->_sti_id = st::logic::sti_rozas1;
            Pguid->add_op(0,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 1300.0, ail_chi_deg,  +40.0, rud_beta_deg, 0., trgg_t_sec,         +70.0));
            Pguid->add_op(1,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 1300.0, ail_xi_deg,   -10.0, rud_beta_deg, 0., trgg_chi_deg,       -50.0));
            Pguid->add_op(2,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 1300.0, ail_chi_deg,  -50.0, rud_beta_deg, 0., trgg_Deltat_sec,    +25.0));
            Pguid->add_op(3,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 1300.0, ail_xi_deg,   -10.0, rud_beta_deg, 0., trgg_chi_deg,      -140.0));
            Pguid->add_op(4,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 1300.0, ail_chi_deg, -140.0, rud_beta_deg, 0., trgg_Deltat_sec,    +60.0));
            Pguid->add_op(5,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 1300.0, ail_xi_deg,   -10.0, rud_beta_deg, 0., trgg_chi_deg,      +130.0));
            Pguid->add_op(6,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 1300.0, ail_chi_deg, +130.0, rud_beta_deg, 0., trgg_Deltat_sec,    +20.0));
            Pguid->add_op(7,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 1300.0, ail_xi_deg,   -10.0, rud_beta_deg, 0., trgg_chi_deg,       +40.0));
            Pguid->add_op(8,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 1300.0, ail_chi_deg,  +40.0, rud_beta_deg, 0., trgg_Deltat_sec,    +55.0));
            Pguid->add_op(9,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 1300.0, ail_xi_deg,   -10.0, rud_beta_deg, 0., trgg_chi_deg,       -50.0));
            Pguid->add_op(10, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 1300.0, ail_chi_deg,  -50.0, rud_beta_deg, 0., trgg_Deltat_sec,    +15.0));
            Pguid->add_op(11, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 1300.0, ail_xi_deg,   -10.0, rud_beta_deg, 0., trgg_chi_deg,      -140.0));
            Pguid->add_op(12, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 1300.0, ail_chi_deg, -140.0, rud_beta_deg, 0., trgg_Deltat_sec,    +50.0));
            Pguid->add_op(13, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 1300.0, ail_xi_deg,   -10.0, rud_beta_deg, 0., trgg_chi_deg,      +130.0));
            Pguid->add_op(14, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 1300.0, ail_chi_deg, +130.0, rud_beta_deg, 0., trgg_Deltat_sec,    +10.0));
            Pguid->add_op(15, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 1300.0, ail_xi_deg,   -10.0, rud_beta_deg, 0., trgg_chi_deg,       +40.0));
            Pguid->add_op(16, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 1300.0, ail_chi_deg,  +40.0, rud_beta_deg, 0., trgg_Deltat_sec,    +45.0));
            Pguid->add_op(17, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 1300.0, ail_xi_deg,   -10.0, rud_beta_deg, 0., trgg_chi_deg,       -50.0));
            Pguid->add_op(18, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 1300.0, ail_chi_deg,  -50.0, rud_beta_deg, 0., trgg_Deltat_sec,     +5.0));
            Pguid->add_op(19, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 1300.0, ail_xi_deg,   -10.0, rud_beta_deg, 0., trgg_chi_deg,      -140.0));
            Pguid->add_op(20, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 1300.0, ail_chi_deg, -140.0, rud_beta_deg, 0., trgg_Deltat_sec,    +40.0));
            Pguid->add_op(21, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 1300.0, ail_xi_deg,   -10.0, rud_beta_deg, 0., trgg_chi_deg,       +40.0));
            Pguid->add_op(22, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 1300.0, ail_chi_deg,  +40.0, rud_beta_deg, 0., trgg_Deltat_sec,    +35.0));

            return Pguid;
        }

        case 97: { // fly closing hipodrome over Rozas (airport is at 438 m over sea level, fly area ends at 4500 ft = 1370 m)
            auto Pguid = new control::guid(37, 3800.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid);
            Pguid->_sti_id = st::logic::sti_rozas1;
            Pguid->add_op(0,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         1300.0, ail_chi_deg,  +40.0, rud_beta_deg, 0., trgg_t_sec,         +70.0));
            Pguid->add_op(1,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         1300.0, ail_xi_deg,   -15.0, rud_beta_deg, 0., trgg_chi_deg,       -45.0));
            Pguid->add_op(2,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         1300.0, ail_chi_deg,  -50.0, rud_beta_deg, 0., trgg_Deltat_sec,    +25.0));
            Pguid->add_op(3,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         1300.0, ail_xi_deg,   -15.0, rud_beta_deg, 0., trgg_chi_deg,      -135.0));
            Pguid->add_op(4,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         1300.0, ail_chi_deg, -140.0, rud_beta_deg, 0., trgg_Deltat_sec,    +60.0));
            Pguid->add_op(5,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         1300.0, ail_xi_deg,   -15.0, rud_beta_deg, 0., trgg_chi_deg,      +135.0));
            Pguid->add_op(6,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         1300.0, ail_chi_deg, +130.0, rud_beta_deg, 0., trgg_Deltat_sec,    +20.0));
            Pguid->add_op(7,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         1300.0, ail_xi_deg,   -15.0, rud_beta_deg, 0., trgg_chi_deg,       +45.0));
            Pguid->add_op(8,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         1300.0, ail_chi_deg,  +40.0, rud_beta_deg, 0., trgg_Deltat_sec,    +55.0));
            Pguid->add_op(9,  new control::guid_op(thr_vtas_mps, 34.0, elv_Hp_m,         1300.0, ail_chi_deg,  +40.0, rud_beta_deg, 0., trgg_Deltat_sec,    +50.0));
            Pguid->add_op(10, new control::guid_op(thr_vtas_mps, 34.0, elv_Hp_m,         1300.0, ail_xi_deg,   +15.0, rud_beta_deg, 0., trgg_chi_deg,       +75.0));
            Pguid->add_op(11, new control::guid_op(thr_vtas_mps, 34.0, elv_Hp_m,         1300.0, ail_chi_deg,  +80.0, rud_beta_deg, 0., trgg_Deltat_sec,    +70.0));
            Pguid->add_op(12, new control::guid_op(thr_vtas_mps, 34.0, elv_Hp_m,         1300.0, ail_xi_deg,   +15.0, rud_beta_deg, 0., trgg_chi_deg,      -155.0));
            Pguid->add_op(13, new control::guid_op(thr_vtas_mps, 34.0, elv_Hp_m,         1300.0, ail_chi_deg, -150.0, rud_beta_deg, 0., trgg_Deltat_sec,    +70.0));
            Pguid->add_op(14, new control::guid_op(thr_vtas_mps, 34.0, elv_gammaTAS_deg, +2.0,   ail_chi_deg, -150.0, rud_beta_deg, 0., trgg_Hp_m,        +1800.0));
            Pguid->add_op(15, new control::guid_op(thr_vtas_mps, 34.0, elv_Hp_m,         1800.0, ail_chi_deg, -150.0, rud_beta_deg, 0., trgg_Deltat_sec,    +30.0));
            Pguid->add_op(16, new control::guid_op(thr_vtas_mps, 34.0, elv_Hp_m,         1800.0, ail_xi_deg,   +15.0, rud_beta_deg, 0., trgg_chi_deg,       -95.0));
            Pguid->add_op(17, new control::guid_op(thr_vtas_mps, 34.0, elv_Hp_m,         1800.0, ail_chi_deg,  -90.0, rud_beta_deg, 0., trgg_Deltat_sec,    +50.0));
            Pguid->add_op(18, new control::guid_op(thr_vtas_mps, 34.0, elv_Hp_m,         1800.0, ail_xi_deg,   +15.0, rud_beta_deg, 0., trgg_chi_deg,       +25.0));
            Pguid->add_op(19, new control::guid_op(thr_vtas_mps, 34.0, elv_Hp_m,         1800.0, ail_chi_deg,  +30.0, rud_beta_deg, 0., trgg_Deltat_sec,    +50.0));
            Pguid->add_op(20, new control::guid_op(thr_vtas_mps, 34.0, elv_Hp_m,         1800.0, ail_xi_deg,   +15.0, rud_beta_deg, 0., trgg_chi_deg,      +135.0));
            Pguid->add_op(21, new control::guid_op(thr_vtas_mps, 34.0, elv_Hp_m,         1800.0, ail_chi_deg, +140.0, rud_beta_deg, 0., trgg_Deltat_sec,    +30.0));
            Pguid->add_op(22, new control::guid_op(thr_vtas_mps, 34.0, elv_Hp_m,         1800.0, ail_xi_deg,   +15.0, rud_beta_deg, 0., trgg_chi_deg,      -165.0));
            Pguid->add_op(23, new control::guid_op(thr_vtas_mps, 34.0, elv_Hp_m,         1800.0, ail_chi_deg, -160.0, rud_beta_deg, 0., trgg_Deltat_sec,    +50.0));
            Pguid->add_op(24, new control::guid_op(thr_vtas_mps, 31.0, elv_Hp_m,         1800.0, ail_chi_deg, -160.0, rud_beta_deg, 0., trgg_Deltat_sec,    +50.0));
            Pguid->add_op(25, new control::guid_op(thr_vtas_mps, 31.0, elv_Hp_m,         1800.0, ail_xi_deg,   +15.0, rud_beta_deg, 0., trgg_chi_deg,      -105.0));
            Pguid->add_op(26, new control::guid_op(thr_vtas_mps, 31.0, elv_Hp_m,         1800.0, ail_chi_deg, -100.0, rud_beta_deg, 0., trgg_Deltat_sec,    +30.0));
            Pguid->add_op(27, new control::guid_op(thr_vtas_mps, 31.0, elv_gammaTAS_deg, -2.0,   ail_chi_deg, -100.0, rud_beta_deg, 0., trgg_Hp_m,        +1100.0));
            Pguid->add_op(28, new control::guid_op(thr_vtas_mps, 31.0, elv_Hp_m,         1100.0, ail_chi_deg, -100.0, rud_beta_deg, 0., trgg_Deltat_sec,    +30.0));
            Pguid->add_op(29, new control::guid_op(thr_vtas_mps, 31.0, elv_Hp_m,         1100.0, ail_xi_deg,   -15.0, rud_beta_deg, 0., trgg_chi_deg,      +175.0));
            Pguid->add_op(30, new control::guid_op(thr_vtas_mps, 31.0, elv_Hp_m,         1100.0, ail_chi_deg, +170.0, rud_beta_deg, 0., trgg_Deltat_sec,    +25.0));
            Pguid->add_op(31, new control::guid_op(thr_vtas_mps, 31.0, elv_Hp_m,         1100.0, ail_xi_deg,   -15.0, rud_beta_deg, 0., trgg_chi_deg,       +85.0));
            Pguid->add_op(32, new control::guid_op(thr_vtas_mps, 31.0, elv_Hp_m,         1100.0, ail_chi_deg,  +80.0, rud_beta_deg, 0., trgg_Deltat_sec,    +50.0));
            Pguid->add_op(33, new control::guid_op(thr_vtas_mps, 31.0, elv_Hp_m,         1100.0, ail_xi_deg,   -15.0, rud_beta_deg, 0., trgg_chi_deg,        -5.0));
            Pguid->add_op(34, new control::guid_op(thr_vtas_mps, 31.0, elv_Hp_m,         1100.0, ail_chi_deg,  -10.0, rud_beta_deg, 0., trgg_Deltat_sec,    +25.0));
            Pguid->add_op(35, new control::guid_op(thr_vtas_mps, 31.0, elv_Hp_m,         1100.0, ail_xi_deg,   -15.0, rud_beta_deg, 0., trgg_chi_deg,       -95.0));
            Pguid->add_op(36, new control::guid_op(thr_vtas_mps, 31.0, elv_Hp_m,         1100.0, ail_chi_deg, -100.0, rud_beta_deg, 0., trgg_Deltat_sec,    +50.0));

            return Pguid;
        }

        // ========================================================
        // ===== ===== ===== Control Trajectories ===== ===== =====
        // ========================================================

        case 150: { // easy test
            // Open: max errors are vtas [mps] -0.883, theta [deg] +1.011, xi [deg] -0.686, beta [deg] -1.278
            // NONE  mean-std error vtas [mps] +1.74e-3 +1.10e-1, alpha [deg] +5.04e-3 +1.22e-1, beta [deg] -3.39e-3 +1.40e-1, Hp [m] +1.29e0 +3.96e0
            // XACT  mean-std error rtov [deg] +7.53e-1 +3.48e-1, w_nbb [dps] +6.68e-2 +3.18e-2, bias_gyr [dps] +1.32e-2 +1.55e-2, bias_mag [nT] +2.16e+1 +5.03e+0
            // Closed (no wind, zero weather, default constant magnetism, init time -10):
            // max error vtas [mps] -0.945, theta [deg] +1.048, xi [deg] -0.762, beta [deg] -1.312
            // mean-std error rtov [deg] +5.55e-1 +3.56e-1, w_nbb [dps] +6.61e-2 +2.83e-2, bias_gyr [dps] +1.03e-2 +1.20e-2, bias_mag [nT] +2.12e+1 +7.68e-1
            auto Pguid = new control::guid(1, 200.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid); //105.0);
            Pguid->_sti_id = st::logic::sti_h3000_tas25_psi00;
            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 25.0, elv_theta_deg,  0.0, ail_xi_deg,  0.0, rud_beta_deg, 0., trgg_t_sec,     200.0));
            return Pguid;
        }
        case 151: { // primary loops - accelerate and decelerate in vtas +- 2 [mps] steps - starts at 3000 - 25
            // Calm: max errors are vtas [mps] +2.003, theta [deg] -0.591, xi [deg] +0.652, beta [deg] -0.208 (no turbulence) (GEO SPH, DeltaT = 10)
            // Open: max errors are vtas [mps] -2.4883, theta [deg] -1.226, xi [deg] -0.838, beta [deg] +1.091
            // NONE  mean-std error vtas [mps] +1.43e-3 +1.19e-1, alpha [deg] +2.18e-3 +1.15e-1, beta [deg] -1.63e-3 +1.32e-1, Hp [m] +8.59e-1 +4.63e0
            // XACT  mean-std error rtov [deg] +5.82e-1 +3.39e-1, w_nbb [dps] +6.61e-2 +3.08e-2, bias_gyr [dps] +9.61e-3 +1.41e-2, bias_mag [nT] +2.42e+1 +4.50e+0
            // Closed (no wind, zero weather, default constant magnetism, init time -10):
            // max error vtas [mps] -2.472, theta [deg] -1.275, xi [deg] -0.781, beta [deg] +1.2351
            // mean-std error rtov [deg] +5.35e-1 +3.87e-1, w_nbb [dps] +6.59e-2 +2.84e-2, bias_gyr [dps] +8.63e-3 +1.07e-2, bias_mag [nT] +2.44e+1 +1.90e+0
            auto Pguid = new control::guid(9, 280.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid);
            Pguid->_sti_id = st::logic::sti_h3000_tas25_psi00;
            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 25.0, elv_theta_deg,  0.0, ail_xi_deg,  0.0, rud_beta_deg, 0., trgg_t_sec,      40.0));
            Pguid->add_op(1, new control::guid_op(thr_vtas_mps, 27.0, elv_theta_deg,  0.0, ail_xi_deg,  0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(2, new control::guid_op(thr_vtas_mps, 29.0, elv_theta_deg,  0.0, ail_xi_deg,  0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(3, new control::guid_op(thr_vtas_mps, 31.0, elv_theta_deg,  0.0, ail_xi_deg,  0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(4, new control::guid_op(thr_vtas_mps, 33.0, elv_theta_deg,  0.0, ail_xi_deg,  0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(5, new control::guid_op(thr_vtas_mps, 31.0, elv_theta_deg,  0.0, ail_xi_deg,  0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(6, new control::guid_op(thr_vtas_mps, 29.0, elv_theta_deg,  0.0, ail_xi_deg,  0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(7, new control::guid_op(thr_vtas_mps, 27.0, elv_theta_deg,  0.0, ail_xi_deg,  0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(8, new control::guid_op(thr_vtas_mps, 25.0, elv_theta_deg,  0.0, ail_xi_deg,  0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            return Pguid;
        }
        case 152: { // primary loops - accelerate and decelerate in vtas +- 4 [mps] steps - starts at 3000 - 25
            // Calm: max errors are vtas [mps] +4.004, theta [deg] -1.058, xi [deg] +1.264, beta [deg] +0.398 (no turbulence) (GEO SPH, DeltaT = 10)
            // Open: max errors are vtas [mps] -4.279, theta [deg] -1.637, xi [deg] +1.179, beta [deg] +1.229
            // NONE  mean-std error vtas [mps] +9.25e-4 +1.42e-1, alpha [deg] +4.99e-3 +1.18e-1, beta [deg] -2.29e-3 +1.33e-1, Hp [m] +9.40e-1 +4.98e0
            // XACT  mean-std error rtov [deg] +6.05e-1 +3.78e-1, w_nbb [dps] +6.80e-2 +3.30e-2, bias_gyr [dps] +1.38e-2 +1.69e-2, bias_mag [nT] +2.42e+1 +5.60e+0
            // Closed (no wind, zero weather, default constant magnetism, init time -10):
            // max error vtas [mps] -4.533, theta [deg] -1.709, xi [deg] +1.253, beta [deg] +1.267
            // mean-std error rtov [deg] +6.17e-1 +3.61e-1, w_nbb [dps] +6.73e-2 +2.90e-2, bias_gyr [dps] +1.24e-2 +1.26e-2, bias_mag [nT] +2.38e+1 +1.98e+0
            auto Pguid = new control::guid(5, 160.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid);
            Pguid->_sti_id = st::logic::sti_h3000_tas25_psi00;
            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 25.0, elv_theta_deg,  0.0, ail_xi_deg,  0.0, rud_beta_deg, 0., trgg_t_sec,      40.0));
            Pguid->add_op(1, new control::guid_op(thr_vtas_mps, 29.0, elv_theta_deg,  0.0, ail_xi_deg,  0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(2, new control::guid_op(thr_vtas_mps, 33.0, elv_theta_deg,  0.0, ail_xi_deg,  0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(3, new control::guid_op(thr_vtas_mps, 29.0, elv_theta_deg,  0.0, ail_xi_deg,  0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(4, new control::guid_op(thr_vtas_mps, 25.0, elv_theta_deg,  0.0, ail_xi_deg,  0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            return Pguid;
        }
        case 153: { // primary loops - pull up and push down in theta +- 2 [deg] steps - starts at 3000 - 30
            // Calm: max errors are vtas [mps] -0.422, theta [deg] +2.002, xi [deg] -0.104, beta [deg] -0.109 (no turbulence) (GEO SPH, DeltaT = 10)
            // Open: max errors are vtas [mps] -0.972, theta [deg] -2.446, xi [deg] -0.546, beta [deg] -0.973 (seed 1)
            // NONE  mean-std error vtas [mps] +1.52e-3 +1.04e-1, alpha [deg] +2.15e-3 +1.15e-1, beta [deg] -1.65e-3 +1.29e-1, Hp [m] +2.32e-2 +5.63e0
            // XACT  mean-std error rtov [deg] +7.69e-1 +3.55e-1, w_nbb [dps] +6.79e-2 +3.50e-2, bias_gyr [dps] +1.05e-2 +1.17e-2, bias_mag [nT] +1.10e+1 +2.05e+0
            // Closed (no wind, zero weather, default constant magnetism, init time -10):
            // max error vtas [mps] -0.990, theta [deg] -2.445, xi [deg] -0.583, beta [deg] -1.018 (seed 1)
            // mean-std error rtov [deg] +5.79e-1 +3.31e-1, w_nbb [dps] +6.81e-2 +3.54e-2, bias_gyr [dps] +8.75e-3 +1.11e-2, bias_mag [nT] +1.13e+1 +1.30e+0
            auto Pguid = new control::guid(9, 280.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid);
            Pguid->_sti_id = st::logic::sti_h3000_tas30_psi00;
            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg,  0.0, ail_xi_deg,  0.0, rud_beta_deg, 0., trgg_t_sec,      40.0));
            Pguid->add_op(1, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg,  2.0, ail_xi_deg,  0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(2, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg,  4.0, ail_xi_deg,  0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(3, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg,  2.0, ail_xi_deg,  0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(4, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg,  0.0, ail_xi_deg,  0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(5, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg, -2.0, ail_xi_deg,  0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(6, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg, -4.0, ail_xi_deg,  0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(7, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg, -2.0, ail_xi_deg,  0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(8, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg,  0.0, ail_xi_deg,  0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            return Pguid;
        }
        case 154: { // primary loops - pull up and push down in theta +- 4 [deg] steps - starts at 3000 - 30
            // Calm: max errors are vtas [mps] -0.838, theta [deg] -4.001, xi [deg] -0.208, beta [deg] -0.109 (no turbulence) (GEO SPH, DeltaT = 10)
            // Open: max errors are vtas [mps] -1.257, theta [deg] -4.308, xi [deg] -0.588, beta [deg] -1.038 (seed 1)
            // NONE  mean-std error vtas [mps] +1.12e-3 +1.09e-1, alpha [deg] +4.96e-3 +1.22e-1, beta [deg] -2.30e-3 +1.29e-1, Hp [m] +3.98e-1 +5.91e0
            // XACT  mean-std error rtov [deg] +8.43e-1 +3.83e-1, w_nbb [dps] +7.16e-2 +4.77e-2, bias_gyr [dps] +1.55e-2 +1.31e-2, bias_mag [nT] +1.16e+1 +2.47e+0
            // Closed (no wind, zero weather, default constant magnetism, init time -10):
            // max error vtas [mps] -1.299, theta [deg] -4.318, xi [deg] -0.639, beta [deg] -1.082 (seed 1)
            // mean-std error rtov [deg] +6.79e-1 +3.51e-1, w_nbb [dps] +7.18e-2 +4.92e-2, bias_gyr [dps] +1.31e-2 +1.30e-2, bias_mag [nT] +1.16e+1 +9.41e-1
            auto Pguid = new control::guid(5, 160.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid);
            Pguid->_sti_id = st::logic::sti_h3000_tas30_psi00;
            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg,  0.0, ail_xi_deg,  0.0, rud_beta_deg, 0., trgg_t_sec,      40.0));
            Pguid->add_op(1, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg,  4.0, ail_xi_deg,  0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(2, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg,  0.0, ail_xi_deg,  0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(3, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg, -4.0, ail_xi_deg,  0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(4, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg,  0.0, ail_xi_deg,  0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            return Pguid;
        }
        case 155: { // primary loops - roll in and roll out in xi +- 5 [deg] steps - starts at 3000 - 30
            // Calm: max errors are vtas [mps] -0.054, theta [deg] +0.220, xi [deg] +5.003, beta [deg] +1.262 (no turbulence) (GEO SPH, DeltaT = 10)
            // Open: max errors are vtas [mps] -0.876, theta [deg] +0.839, xi [deg] +5.170, beta [deg] -2.093 (seed 1)
            // NONE  mean-std error vtas [mps] +1.99e-3 +1.05e-1, alpha [deg] +1.62e-3 +1.13e-1, beta [deg] -2.20e-3 +1.32e-1, Hp [m] -1.92e-1 +4.42e0
            // XACT  mean-std error rtov [deg] +3.70e-1 +3.63e-1, w_nbb [dps] +6.75e-2 +3.52e-2, bias_gyr [dps] +6.88e-3 +1.10e-2, bias_mag [nT] +5.55e+0 +3.50e+0
            // Closed (no wind, zero weather, default constant magnetism, init time -10):
            // max error vtas [mps] -0.931, theta [deg] +0.847, xi [deg] +5.190, beta [deg] -2.184 (seed 1)
            // mean-std error rtov [deg] +3.41e-1 +3.19e-1, w_nbb [dps] +6.79e-2 +3.55e-2, bias_gyr [dps] +6.13e-3 +1.01e-2, bias_mag [nT] +5.02e+0 +3.18e+0
            auto Pguid = new control::guid(13, 400.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid);
            Pguid->_sti_id = st::logic::sti_h3000_tas30_psi00;
            Pguid->add_op(0,  new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg, 0.0, ail_xi_deg,   0.0, rud_beta_deg, 0., trgg_t_sec,      40.0));
            Pguid->add_op(1,  new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg, 0.0, ail_xi_deg,   5.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(2,  new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg, 0.0, ail_xi_deg,  10.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(3,  new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg, 0.0, ail_xi_deg,  15.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(4,  new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg, 0.0, ail_xi_deg,  10.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(5,  new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg, 0.0, ail_xi_deg,   5.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(6,  new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg, 0.0, ail_xi_deg,   0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(7,  new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg, 0.0, ail_xi_deg,  -5.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(8,  new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg, 0.0, ail_xi_deg, -10.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(9,  new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg, 0.0, ail_xi_deg, -15.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(10, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg, 0.0, ail_xi_deg, -10.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(11, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg, 0.0, ail_xi_deg,  -5.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(12, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg, 0.0, ail_xi_deg,   0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            return Pguid;
        }
        case 156: { // primary loops - roll in and roll out in xi +- 10 [deg] steps - starts at 3000 - 30
            // Calm: max errors are vtas [mps] -0.193, theta [deg] +0.938, xi [deg] +10.010, beta [deg] +2.517 (no turbulence) (GEO SPH, DeltaT = 10)
            // Open: max errors are vtas [mps] +0.884, theta [deg] +1.440, xi [deg] +10.161, beta [deg] -3.142 (seed 1)
            // NONE  mean-std error vtas [mps] +1.99e-3 +1.05e-1, alpha [deg] +1.62e-3 +1.14e-1, beta [deg] -2.20e-3 +1.37e-1, Hp [m] -1.95e-1 +4.62e0
            // XACT  mean-std error rtov [deg] +3.84e-1 +3.84e-1, w_nbb [dps] +7.09e-2 +5.42e-2, bias_gyr [dps] +6.78e-3 +1.11e-2, bias_mag [nT] +5.34e+0 +3.95e+0
            // Closed (no wind, zero weather, default constant magnetism, init time -10):
            // max error vtas [mps] -0.914, theta [deg] +1.439, xi [deg] +10.190, beta [deg] -3.267 (seed 1)
            // mean-std error rtov [deg] +3.73e-1 +3.39e-1, w_nbb [dps] +7.14e-2 +5.50e-2, bias_gyr [dps] +6.09e-3 +1.03e-2, bias_mag [nT] +4.32e+0 +3.31e+0
            auto Pguid = new control::guid(13, 400.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid);
            Pguid->_sti_id = st::logic::sti_h3000_tas30_psi00;
            Pguid->add_op(0,  new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg, 0.0, ail_xi_deg,   0.0, rud_beta_deg, 0., trgg_t_sec,      40.0));
            Pguid->add_op(1,  new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg, 0.0, ail_xi_deg,  10.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(2,  new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg, 0.0, ail_xi_deg,  20.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(3,  new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg, 0.0, ail_xi_deg,  30.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(4,  new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg, 0.0, ail_xi_deg,  20.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(5,  new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg, 0.0, ail_xi_deg,  10.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(6,  new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg, 0.0, ail_xi_deg,   0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(7,  new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg, 0.0, ail_xi_deg, -10.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(8,  new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg, 0.0, ail_xi_deg, -20.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(9,  new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg, 0.0, ail_xi_deg, -30.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(10, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg, 0.0, ail_xi_deg, -20.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(11, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg, 0.0, ail_xi_deg, -10.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(12, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg, 0.0, ail_xi_deg,   0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            return Pguid;
        }
        case 157: { // primary loops - simultaneous acceleration - deceleration and push-pull maneuvers - starts at 3000 - 30
            // Calm: max errors are vtas [mps] -2.003, theta [deg] -2.001, xi [deg] +0.635, beta [deg] -0.210 (no turbulence) (GEO SPH, DeltaT = 10)
            // Open: max errors are vtas [mps] -2.481, theta [deg] -2.445, xi [deg] -0.809, beta [deg] +1.057 (seed 1)
            // NONE  mean-std error vtas [mps] +1.52e-3 +1.11e-1, alpha [deg] +2.15e-3 +1.14e-1, beta [deg] -1.65e-3 +1.29e-1, Hp [m] +4.71e-2 +5.53e0
            // XACT  mean-std error rtov [deg] +6.79e-1 +3.53e-1, w_nbb [dps] +6.75e-2 +3.41e-2, bias_gyr [dps] +9.85e-3 +1.17e-2, bias_mag [nT] +1.20e+1 +1.98e+0
            // Closed (no wind, zero weather, default constant magnetism, init time -10):
            // max error vtas [mps] -2.462, theta [deg] -2.454, xi [deg] -0.777, beta [deg] +1.097 (seed 1)
            // mean-std error rtov [deg] +4.98e-1 +3.26e-1, w_nbb [dps] +6.77e-2 +3.44e-2, bias_gyr [dps] +8.19e-3 +1.12e-2, bias_mag [nT] +1.24e+1 +9.73e-1
            auto Pguid = new control::guid(9, 280.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid);
            Pguid->_sti_id = st::logic::sti_h3000_tas30_psi00;
            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg,  0.0, ail_xi_deg,  0.0, rud_beta_deg, 0., trgg_t_sec,      40.0));
            Pguid->add_op(1, new control::guid_op(thr_vtas_mps, 32.0, elv_theta_deg,  2.0, ail_xi_deg,  0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(2, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg,  0.0, ail_xi_deg,  0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(3, new control::guid_op(thr_vtas_mps, 28.0, elv_theta_deg,  2.0, ail_xi_deg,  0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(4, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg,  0.0, ail_xi_deg,  0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(5, new control::guid_op(thr_vtas_mps, 32.0, elv_theta_deg, -2.0, ail_xi_deg,  0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(6, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg,  0.0, ail_xi_deg,  0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(7, new control::guid_op(thr_vtas_mps, 28.0, elv_theta_deg, -2.0, ail_xi_deg,  0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(8, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg,  0.0, ail_xi_deg,  0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            return Pguid;
        }
        case 158: { // primary loops - simultaneous acceleration - deceleration and roll in - roll out maneuvers - starts at 3000 - 30
            // Calm: max errors are vtas [mps] +2.004, theta [deg] -0.585, xi [deg] +5.008, beta [deg] +1.487 (no turbulence) (GEO SPH, DeltaT = 10)
            // Open: max errors are vtas [mps] -2.480, theta [deg] -1.189, xi [deg] +5.380, beta [deg] +1.873 (seed 1)
            // NONE  mean-std error vtas [mps] +1.52e-3 +1.15e-1, alpha [deg] +2.16e-3 +1.12e-1, beta [deg] -1.66e-3 +1.32e-1, Hp [m] +9.99e-3 +5.02e+0
            // XACT  mean-std error rtov [deg] +5.47e-1 +4.07e-1, w_nbb [dps] +6.82e-2 +3.78e-2, bias_gyr [dps] +9.39e-3 +1.27e-2, bias_mag [nT] +8.13e+0 +3.41e+0
            // Closed (no wind, zero weather, default constant magnetism, init time -10):
            // max error vtas [mps] -2.567, theta [deg] -1.246, xi [deg] +5.394, beta [deg] +1.912 (seed 1)
            // mean-std error rtov [deg] +6.27e-1 +3.46e-1, w_nbb [dps] +6.85e-2 +3.82e-2, bias_gyr [dps] +9.07e-3 +1.19e-2, bias_mag [nT] +8.95e+0 +2.26e+0
            auto Pguid = new control::guid(9, 280.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid);
            Pguid->_sti_id = st::logic::sti_h3000_tas30_psi00;
            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg, 0.0, ail_xi_deg,   0.0, rud_beta_deg, 0., trgg_t_sec,      40.0));
            Pguid->add_op(1, new control::guid_op(thr_vtas_mps, 32.0, elv_theta_deg, 0.0, ail_xi_deg,   5.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(2, new control::guid_op(thr_vtas_mps, 34.0, elv_theta_deg, 0.0, ail_xi_deg,  10.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(3, new control::guid_op(thr_vtas_mps, 32.0, elv_theta_deg, 0.0, ail_xi_deg,   5.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(4, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg, 0.0, ail_xi_deg,   0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(5, new control::guid_op(thr_vtas_mps, 28.0, elv_theta_deg, 0.0, ail_xi_deg,  -5.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(6, new control::guid_op(thr_vtas_mps, 26.0, elv_theta_deg, 0.0, ail_xi_deg, -10.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(7, new control::guid_op(thr_vtas_mps, 28.0, elv_theta_deg, 0.0, ail_xi_deg,  -5.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(8, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg, 0.0, ail_xi_deg,   0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            return Pguid;
        }
        case 159: { // primary loops - simultaneous push-pull and roll in - roll out maneuvers - starts at 3000 - 30
            // Calm: max errors are vtas [mps] +0.427, theta [deg] +2.002, xi [deg] +5.006, beta [deg] +1.253 (no turbulence) (GEO SPH, DeltaT = 10)
            // Open: max errors are vtas [mps] +0.941, theta [deg] -2.446, xi [deg] +5.268, beta [deg] +1.564 (seed 1)
            // NONE  mean-std error vtas [mps] +1.52e-3 +1.04e-1, alpha [deg] +2.16e-3 +1.15e-1, beta [deg] -1.66e-3 +1.31e-1, Hp [m] +2.40e-2 +5.74e+0
            // XACT  mean-std error rtov [deg] +4.15e-1 +3.42e-1, w_nbb [dps] +6.89e-2 +4.17e-2, bias_gyr [dps] +7.79e-3 +1.22e-2, bias_mag [nT] +7.81e+0 +3.34e+0
            // Closed (no wind, zero weather, default constant magnetism, init time -10):
            // max error vtas [mps] +0.990, theta [deg] -2.454, xi [deg] +5.286, beta [deg] +1.609 (seed 1)
            // mean-std error rtov [deg] +3.63e-1 +3.25e-1, w_nbb [dps] +6.93e-2 +4.25e-2, bias_gyr [dps] +6.59e-3 +1.14e-2, bias_mag [nT] +8.37e+0 +1.78e+0
            auto Pguid = new control::guid(9, 280.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid);
            Pguid->_sti_id = st::logic::sti_h3000_tas30_psi00;
            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg,  0.0, ail_xi_deg,   0.0, rud_beta_deg, 0., trgg_t_sec,      40.0));
            Pguid->add_op(1, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg,  2.0, ail_xi_deg,   5.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(2, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg,  4.0, ail_xi_deg,  10.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(3, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg,  2.0, ail_xi_deg,   5.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(4, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg,  0.0, ail_xi_deg,   0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(5, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg, -2.0, ail_xi_deg,  -5.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(6, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg, -4.0, ail_xi_deg, -10.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(7, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg, -2.0, ail_xi_deg,  -5.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(8, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg,  0.0, ail_xi_deg,   0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            return Pguid;
        }
        case 160: { // primary loops - simultaneous acceleration-deceleration, push-pull and roll in - roll out maneuvers - starts at 3000 - 30
            // Calm: max errors are vtas [mps] -2.009, theta [deg] +2.002, xi [deg] -5.005, beta [deg] -1.444 (no turbulence) (GEO SPH, DeltaT = 10)
            // Open: max errors are vtas [mps] -2.562, theta [deg] -2.421, xi [deg] -5.105, beta [deg] -1.672 (seed 1)
            // NONE  mean-std error vtas [mps] +1.52e-3 +1.16e-1, alpha [deg] +2.11e-3 +1.15e-1, beta [deg] -1.64e-3 +1.29e-1, Hp [m] -7.12e-1 +5.10e+0
            // XACT  mean-std error rtov [deg] +4.21e-1 +3.59e-1, w_nbb [dps] +6.86e-2 +4.12e-2, bias_gyr [dps] +8.18e-3 +1.23e-2, bias_mag [nT] +7.16e+0 +3.52e+0
            // Closed (no wind, zero weather, default constant magnetism, init time -10):
            // max error vtas [mps] -2.562, theta [deg] -2.424, xi [deg] -5.151, beta [deg] -1.748 (seed 1)
            // mean-std error rtov [deg] +4.08e-1 +3.17e-1, w_nbb [dps] +6.90e-2 +4.18e-2, bias_gyr [dps] +7.33e-3 +1.15e-2, bias_mag [nT] +6.44e+0 +3.22e+0
            auto Pguid = new control::guid(9, 280.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid);
            Pguid->_sti_id = st::logic::sti_h3000_tas30_psi00;
            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg,  0.0, ail_xi_deg,   0.0, rud_beta_deg, 0., trgg_t_sec,      40.0));
            Pguid->add_op(1, new control::guid_op(thr_vtas_mps, 32.0, elv_theta_deg,  2.0, ail_xi_deg,   5.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(2, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg,  4.0, ail_xi_deg,  10.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(3, new control::guid_op(thr_vtas_mps, 32.0, elv_theta_deg,  2.0, ail_xi_deg,   5.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(4, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg,  4.0, ail_xi_deg,  10.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(5, new control::guid_op(thr_vtas_mps, 32.0, elv_theta_deg,  2.0, ail_xi_deg,  15.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(6, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg,  4.0, ail_xi_deg,  10.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(7, new control::guid_op(thr_vtas_mps, 28.0, elv_theta_deg,  2.0, ail_xi_deg,   5.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(8, new control::guid_op(thr_vtas_mps, 30.0, elv_theta_deg,  4.0, ail_xi_deg,   0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            return Pguid;
        }
        case 161: { // secondary loops - evaluation of Hp target in straight constant speed level flight - starts at 3000 - 30
            // Open: max errors are vtas [mps] +9.42e-1 theta [deg] +7.54e-1 xi [deg] -4.56e-1 beta [deg] -1.01e+0 Hp [m] +7.75e+0
            //       mean-std error vtas [mps] +2.03e-3 +1.03e-1, alpha [deg] +5.47e-3 +1.13e-1, beta [deg] -3.39e-3 +1.30e-1, Hp [m] +1.92e-2 +8.80e-1
            //       mean-std error rtov [deg] +7.25e-1 +3.27e-1, w_nbb [dps] +6.61e-2 +2.83e-2, bias_gyr [dps] +1.19e-2 +1.27e-2, bias_mag [nT] +1.07e+1 +9.67e-1
            // Closed (no wind, zero weather, default constant magnetism, init time -10):
            // max error vtas [mps] +9.39e-1 theta [deg]-1.39e+0 xi [deg] +5.45e-1 beta [deg] -1.06e+0 Hp [m] +8.64e+0
            // mean-std error rtov [deg] +6.96e-1 +3.47e-1, w_nbb [dps] +7.95e-2 +3.71e-2, bias_gyr [dps] +1.17e-2 +1.27e-2, bias_mag [nT] +1.10e+1 +1.21e+0
            auto Pguid = new control::guid(1, 200.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid); // 30.0 // 200.0
            Pguid->_sti_id = st::logic::sti_h3000_tas30_psi00;
            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg, 0.0, rud_beta_deg, 0., trgg_t_sec, 200.0));
            return Pguid;
        }
        case 162: { // secondary loops - evaluation of Hp target in constant absolute bearing constant speed level flight - starts at 3000 - 30
            // Open: max errors are vtas [mps] -0.968 theta [deg] -0.512 xi [deg] -0.704 beta [deg] -0.975 Hp [m] +1.10 chi [deg] -0.197
            //       mean-std error vtas [mps] +2.17e-3 +1.05e-1, alpha [deg] +5.46e-3 +1.16e-1, beta [deg] -3.39e-3 +1.29e-1, Hp [m] +1.27e-2 +4.880e-1
            //       mean-std error rtov [deg] +6.93e-1 +3.06e-1, w_nbb [dps] +6.38e-2 +2.71e-2, bias_gyr [dps] +1.15e-2 +1.19e-2, bias_mag [nT] +7.62e+0 +9.65e-1
            // Closed (no wind, zero weather, default constant magnetism, init time -10):
            // max error vtas [mps] -0.940 theta [deg] -1.394 xi [deg] -0.795 beta [deg] -1.069 Hp [m] +8.702 chi [deg] -0.457
            // mean-std error rtov [deg] +5.73e-1 +3.20e-1, w_nbb [dps] +7.93e-2 +3.70e-2, bias_gyr [dps] +9.73e-3 +1.10e-2, bias_mag [nT] +7.82e+0 +1.04e+0
            auto Pguid = new control::guid(1, 200.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid);
            Pguid->_sti_id = st::logic::sti_h3000_tas30_psi00;
            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 0.0, rud_beta_deg, 0., trgg_t_sec, 200.0));
            return Pguid;
        }
        case 163: { // secondary loops - altitude changes (both Hp and h with CONNECTIONS using gammaTAS) - starts at 3000 - 30
            // IF I DO NOT WANT CONNECTIONS, I NEED TO FIND A WAY TO DEACTIVATE THE INTEGRAL CONTROL
            // PROBLEM WITH CONNECTIONS IS OVERSHOOTING AS THE AIRCRAFT ARRIVES TO THE TARGET AND ONLY REACTS ONCE IT HAS REACHED IT, WITH INERTIA CARRYING IT OVER THE VALUE
            // Open: max errors are vtas [mps] +1.489, theta [deg] +8.156, xi [deg] -0.570, beta [deg] -1.035, Hp [m] +1.105, h [m] -0.611, gammaTAS [deg] +1.703 (seed 1)
            auto Pguid = new control::guid(9, 360.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid);
            Pguid->_sti_id = st::logic::sti_h3000_tas30_psi00;
            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         3000.0, ail_xi_deg, 0.0, rud_beta_deg, 0., trgg_t_sec,        40.0));
            Pguid->add_op(1, new control::guid_op(thr_vtas_mps, 30.0, elv_gammaTAS_deg,    2.0, ail_xi_deg, 0.0, rud_beta_deg, 0., trgg_Hp_m,       3050.0));
            Pguid->add_op(2, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         3050.0, ail_xi_deg, 0.0, rud_beta_deg, 0., trgg_Deltat_sec,   30.0));
            Pguid->add_op(3, new control::guid_op(thr_vtas_mps, 30.0, elv_gammaTAS_deg,   -2.0, ail_xi_deg, 0.0, rud_beta_deg, 0., trgg_Hp_m,       3000.0));
            Pguid->add_op(4, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m,         3000.0, ail_xi_deg, 0.0, rud_beta_deg, 0., trgg_Deltat_sec,   30.0));
            Pguid->add_op(5, new control::guid_op(thr_vtas_mps, 30.0, elv_gammaTAS_deg,   -2.0, ail_xi_deg, 0.0, rud_beta_deg, 0., trgg_h_m,        2950.0));
            Pguid->add_op(6, new control::guid_op(thr_vtas_mps, 30.0, elv_h_m,          2950.0, ail_xi_deg, 0.0, rud_beta_deg, 0., trgg_Deltat_sec,   30.0));
            Pguid->add_op(7, new control::guid_op(thr_vtas_mps, 30.0, elv_gammaTAS_deg,    2.0, ail_xi_deg, 0.0, rud_beta_deg, 0., trgg_h_m,        3000.0));
            Pguid->add_op(8, new control::guid_op(thr_vtas_mps, 30.0, elv_h_m,          3000.0, ail_xi_deg, 0.0, rud_beta_deg, 0., trgg_Deltat_sec,  30.0));
            return Pguid;
            }
        case 164: { // secondary loops - gammaTAS changes - starts at 3000 - 30
            // maximum errors are vtas [mps] -1.006, theta [deg] +2.828, xi [deg] -0.553, beta [deg] -1.044, gammaTAS [deg] +2.763 (seed 1)
            auto Pguid = new control::guid(5, 160.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid);
            Pguid->_sti_id = st::logic::sti_h3000_tas30_psi00;
            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_gammaTAS_deg, 0.0, ail_xi_deg, 0.0, rud_beta_deg, 0., trgg_t_sec,      40.0));
            Pguid->add_op(1, new control::guid_op(thr_vtas_mps, 30.0, elv_gammaTAS_deg, 2.0, ail_xi_deg, 0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(2, new control::guid_op(thr_vtas_mps, 30.0, elv_gammaTAS_deg, 4.0, ail_xi_deg, 0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(3, new control::guid_op(thr_vtas_mps, 30.0, elv_gammaTAS_deg, 2.0, ail_xi_deg, 0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(4, new control::guid_op(thr_vtas_mps, 30.0, elv_gammaTAS_deg, 4.0, ail_xi_deg, 0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            return Pguid;
        }
        case 165: { // secondary loops - bearing changes (chi, chiTAS, and psi with CONNECTIONS using xi) - starts at 3000 - 30
            // IF I DO NOT WANT CONNECTIONS, I NEED TO FIND A WAY TO DEACTIVATE THE INTEGRAL CONTROL
            // PROBLEM WITH CONNECTIONS IS OVERSHOOTING AS THE AIRCRAFT ARRIVES TO THE TARGET AND ONLY REACTS ONCE IT HAS REACHED IT, WITH INERTIA CARRYING IT OVER THE VALUE
            // maximum errors are vtas [mps] -0.976, theta [deg] +0.597, xi [deg] -11.639, beta [deg] +3.283,
            // Hp [m] +1.102, chi [deg] -1.132, psi [deg] -4.548, chiTAS [deg] -1.344 (seed 1)
            auto Pguid = new control::guid(13, 300.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid);
            Pguid->_sti_id = st::logic::sti_h3000_tas30_psi00;
            Pguid->add_op(0,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg,      0.0, rud_beta_deg, 0., trgg_t_sec,       40.0));
            Pguid->add_op(1,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,      10.0, rud_beta_deg, 0., trgg_chi_deg,     45.0));
            Pguid->add_op(2,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg,     45.0, rud_beta_deg, 0., trgg_Deltat_sec,  30.0));
            Pguid->add_op(3,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,     -10.0, rud_beta_deg, 0., trgg_chi_deg,    -25.0));
            Pguid->add_op(4,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg,    -25.0, rud_beta_deg, 0., trgg_Deltat_sec,  30.0));
            Pguid->add_op(5,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,     -10.0, rud_beta_deg, 0., trgg_chi_deg,    -50.0)); // no chiTAs trigger
            Pguid->add_op(6,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chiTAS_deg, -50.0, rud_beta_deg, 0., trgg_Deltat_sec,  30.0));
            Pguid->add_op(7,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,      10.0, rud_beta_deg, 0., trgg_chi_deg,    -10.0)); // no chiTAS trigger
            Pguid->add_op(8,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chiTAS_deg, -10.0, rud_beta_deg, 0., trgg_Deltat_sec,  30.0));
            Pguid->add_op(9,  new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,      10.0, rud_beta_deg, 0., trgg_psi_deg,     30.0));
            Pguid->add_op(10, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_psi_deg,     30.0, rud_beta_deg, 0., trgg_Deltat_sec,  30.0));
            Pguid->add_op(11, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_xi_deg,     -10.0, rud_beta_deg, 0., trgg_psi_deg,    -35.0));
            Pguid->add_op(12, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_psi_deg,    -35.0, rud_beta_deg, 0., trgg_Deltat_sec,  30.0));
            return Pguid;
        }
        case 166: { // secondary loops - muTAS changes - starts at 3000 - 30
            // maximum errors are vtas [mps] -0.886, theta [deg] +1.147, xi [deg] +22.518, beta [deg] +4.554, gammaTAS [deg] +1.128, muTAS [deg] -20.023 (seed 1)
            auto Pguid = new control::guid(5, 160.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid);
            Pguid->_sti_id = st::logic::sti_h3000_tas30_psi00;
            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_gammaTAS_deg, 0.0, ail_muTAS_deg,   0.0, rud_beta_deg, 0., trgg_t_sec,      40.0));
            Pguid->add_op(1, new control::guid_op(thr_vtas_mps, 30.0, elv_gammaTAS_deg, 0.0, ail_muTAS_deg,  15.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(2, new control::guid_op(thr_vtas_mps, 30.0, elv_gammaTAS_deg, 0.0, ail_muTAS_deg,   5.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(3, new control::guid_op(thr_vtas_mps, 30.0, elv_gammaTAS_deg, 0.0, ail_muTAS_deg, -15.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(4, new control::guid_op(thr_vtas_mps, 30.0, elv_gammaTAS_deg, 0.0, ail_muTAS_deg,   0.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            return Pguid;
        }
        case 167: { // secondary loops - bearing changes (chi, chiTAS, and psi with CONNECTIONS using muTAS) - starts at 3000 - 30
            // IF I DO NOT WANT CONNECTIONS, I NEED TO FIND A WAY TO DEACTIVATE THE INTEGRAL CONTROL
            // PROBLEM WITH CONNECTIONS IS OVERSHOOTING AS THE AIRCRAFT ARRIVES TO THE TARGET AND ONLY REACTS ONCE IT HAS REACHED IT, WITH INERTIA CARRYING IT OVER THE VALUE
            // maximum errors are vtas [mps] -0.972, theta [deg] +0.559, xi [deg] +15.616, beta [deg] -3.867,
            // Hp [m] +1.102, chi [deg] +1.166, psi [deg] +3.724, muTAS [deg] +10.619, chiTAS [deg] +2.682 (seeds1)
            auto Pguid = new control::guid(13, 300.0, Oseeder.provide_seed(math::seeder::seeder_guid), case_guid);
            Pguid->_sti_id = st::logic::sti_h3000_tas30_psi00;
            Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 0.0, rud_beta_deg, 0., trgg_t_sec, 40.0));
            Pguid->add_op(1, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_muTAS_deg, 10.0, rud_beta_deg, 0., trgg_chi_deg, 45.0));
            Pguid->add_op(2, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 45.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(3, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_muTAS_deg, -10.0, rud_beta_deg, 0., trgg_chi_deg, -25.0));
            Pguid->add_op(4, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, -25.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(5, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_muTAS_deg, -10.0, rud_beta_deg, 0., trgg_chi_deg, -50.0)); // no chiTAs trigger
            Pguid->add_op(6, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chiTAS_deg, -50.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(7, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_muTAS_deg, 10.0, rud_beta_deg, 0., trgg_chi_deg, -10.0)); // no chiTAS trigger
            Pguid->add_op(8, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chiTAS_deg, -10.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(9, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_muTAS_deg, 10.0, rud_beta_deg, 0., trgg_psi_deg, 30.0));
            Pguid->add_op(10, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_psi_deg, 30.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            Pguid->add_op(11, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_muTAS_deg, -10.0, rud_beta_deg, 0., trgg_psi_deg, -35.0));
            Pguid->add_op(12, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_psi_deg, -35.0, rud_beta_deg, 0., trgg_Deltat_sec, 30.0));
            return Pguid;
        }



        default:
            throw std::runtime_error("Guidance case not available");
    }
}
/* returns pointer to guidance objectives based on seeder and case identifier. */

void control::guid::create_text(std::ostream& Ostream) const {
    switch (_case_guid) {
        case 1:
            Ostream << std::endl << "INTENT:" << std::endl << std::endl;
            Ostream << "Initial  chi [deg]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[0]->get_guid_val(control::logic::cntr_AIL) << std::endl;
            Ostream << "Turn       t [sec]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[0]->get_guid_trg_val()                     << std::endl;
            Ostream << "Turn      xi [deg]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[1]->get_guid_val(control::logic::cntr_AIL) << std::endl;
            Ostream << "Final    chi [deg]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[5]->get_guid_val(control::logic::cntr_AIL) << std::endl;
            Ostream << "Initial vtas [mps]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[0]->get_guid_val(control::logic::cntr_THR) << std::endl;
            Ostream << "Speed Deltat [sec]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[2]->get_guid_trg_val()                     << std::endl;
            Ostream << "Final   vtas [mps]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[5]->get_guid_val(control::logic::cntr_THR) << std::endl;
            Ostream << "Initial   Hp   [m]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[0]->get_guid_val(control::logic::cntr_ELV) << std::endl;
            Ostream << "Alt   Deltat [sec]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[3]->get_guid_trg_val()                     << std::endl;
            Ostream << "Alt gammaTAS [deg]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[4]->get_guid_val(control::logic::cntr_ELV) << std::endl;
            Ostream << "Final     Hp   [m]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[5]->get_guid_val(control::logic::cntr_ELV) << std::endl;
            break;
        case 2:
            Ostream << std::endl << "INTENT:" << std::endl << std::endl;
            Ostream << "Initial  chi [deg]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[0]->get_guid_val(control::logic::cntr_AIL) << std::endl;
            Ostream << "Turn       t [sec]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[0]->get_guid_trg_val()                     << std::endl;
            Ostream << "Turn      xi [deg]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[1]->get_guid_val(control::logic::cntr_AIL) << std::endl;
            Ostream << "Final    chi [deg]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[2]->get_guid_val(control::logic::cntr_AIL) << std::endl;
            Ostream << "Initial vtas [mps]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[0]->get_guid_val(control::logic::cntr_THR) << std::endl;
            Ostream << "Initial   Hp   [m]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[0]->get_guid_val(control::logic::cntr_ELV) << std::endl;
            break;
        case 3:
            Ostream << std::endl << "INTENT:" << std::endl << std::endl;
            Ostream << "Initial    chi [deg]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[0]->get_guid_val(control::logic::cntr_AIL) << std::endl;
            Ostream << "Turn #1      t [sec]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[0]->get_guid_trg_val()                     << std::endl;
            Ostream << "Turn #1     xi [deg]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[1]->get_guid_val(control::logic::cntr_AIL) << std::endl;
            Ostream << "Turn #1    chi [deg]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[2]->get_guid_val(control::logic::cntr_AIL) << std::endl;
            Ostream << "vtas           [mps]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[0]->get_guid_val(control::logic::cntr_THR) << std::endl;
            Ostream << "Hp               [m]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[0]->get_guid_val(control::logic::cntr_ELV) << std::endl;
            Ostream << "Turn #2 Deltat [sec]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[2]->get_guid_trg_val()                     << std::endl;
            Ostream << "Turn #2     xi [deg]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[3]->get_guid_val(control::logic::cntr_AIL) << std::endl;
            Ostream << "Turn #2    chi [deg]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[4]->get_guid_val(control::logic::cntr_AIL) << std::endl;
            Ostream << "Turn #3 Deltat [sec]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[4]->get_guid_trg_val()                     << std::endl;
            Ostream << "Turn #3     xi [deg]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[5]->get_guid_val(control::logic::cntr_AIL) << std::endl;
            Ostream << "Turn #3    chi [deg]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[6]->get_guid_val(control::logic::cntr_AIL) << std::endl;
            Ostream << "Turn #4 Deltat [sec]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[6]->get_guid_trg_val()                     << std::endl;
            Ostream << "Turn #4     xi [deg]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[7]->get_guid_val(control::logic::cntr_AIL) << std::endl;
            Ostream << "Turn #4    chi [deg]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[8]->get_guid_val(control::logic::cntr_AIL) << std::endl;
            Ostream << "Turn #5 Deltat [sec]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[8]->get_guid_trg_val()                     << std::endl;
            Ostream << "Turn #5     xi [deg]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[9]->get_guid_val(control::logic::cntr_AIL) << std::endl;
            Ostream << "Turn #5    chi [deg]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[10]->get_guid_val(control::logic::cntr_AIL) << std::endl;
            Ostream << "Turn #6 Deltat [sec]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[10]->get_guid_trg_val()                     << std::endl;
            Ostream << "Turn #6     xi [deg]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[11]->get_guid_val(control::logic::cntr_AIL) << std::endl;
            Ostream << "Turn #6    chi [deg]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[12]->get_guid_val(control::logic::cntr_AIL) << std::endl;
            Ostream << "Turn #7 Deltat [sec]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[12]->get_guid_trg_val()                     << std::endl;
            Ostream << "Turn #7     xi [deg]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[13]->get_guid_val(control::logic::cntr_AIL) << std::endl;
            Ostream << "Turn #7    chi [deg]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[14]->get_guid_val(control::logic::cntr_AIL) << std::endl;
            Ostream << "Turn #8 Deltat [sec]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[14]->get_guid_trg_val()                     << std::endl;
            Ostream << "Turn #8     xi [deg]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[15]->get_guid_val(control::logic::cntr_AIL) << std::endl;
            Ostream << "Turn #8    chi [deg]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[16]->get_guid_val(control::logic::cntr_AIL) << std::endl;
            break;
        case 9:
            Ostream << std::endl << "INTENT:" << std::endl << std::endl;
            Ostream << "Initial  psi [deg]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[0]->get_guid_val(control::logic::cntr_AIL) << std::endl;
            Ostream << "Turn       t [sec]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[0]->get_guid_trg_val()                     << std::endl;
            Ostream << "Turn      xi [deg]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[1]->get_guid_val(control::logic::cntr_AIL) << std::endl;
            Ostream << "Final    psi [deg]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[5]->get_guid_val(control::logic::cntr_AIL) << std::endl;
            Ostream << "Initial vtas [mps]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[0]->get_guid_val(control::logic::cntr_THR) << std::endl;
            Ostream << "Speed Deltat [sec]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[2]->get_guid_trg_val()                     << std::endl;
            Ostream << "Final   vtas [mps]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[5]->get_guid_val(control::logic::cntr_THR) << std::endl;
            Ostream << "Initial   Hp   [m]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[0]->get_guid_val(control::logic::cntr_ELV) << std::endl;
            Ostream << "Alt   Deltat [sec]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[3]->get_guid_trg_val()                     << std::endl;
            Ostream << "Alt gammaTAS [deg]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[4]->get_guid_val(control::logic::cntr_ELV) << std::endl;
            Ostream << "Final     Hp   [m]: " << std::fixed << std::setw(7)  << std::setprecision(1) << std::showpos << _Vguid_op[5]->get_guid_val(control::logic::cntr_ELV) << std::endl;
            break;
        default:
            std::cout << "Guidance not described in text file." << std::endl;
            break;
    }
}
/* describe guidance in stream */

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
















