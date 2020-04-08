#include "filter_nav.h"
#include "../air/filter_air01.h"
#include "../air/filter_air02.h"
#include "../att/filter_att01.h"
#include "../att/filter_att02.h"
#include "../att/filter_att03.h"
#include "../att/filter_att04.h"
#include "../gps/filter_gps01.h"
#include "../pos/filter_pos01.h"
#include "../pos/filter_pos02.h"
#include "../pos/filter_pos03.h"

// CLASS EXECUTE_STEP_FUNCTOR
// ==========================
// ==========================

void nav::execute_step_functor_gps::execute_step(st::st_nav_out& Ost_nav_out, const st::st_nav_out& Ost_nav_out_prev, const st::st_sens_out& Ost_sens_out,
                                                 const st::st_gps_out& Ost_gps_out, const st::st_nav_in& Ost_nav_in, const unsigned int& s, const unsigned int& g, bool flag_gps) {
    Ost_nav_out.get_t_sec() = Ost_sens_out.get_t_sec();
    // run air data filter (fills up Ost_nav: vtas_mps, euler_wfsbfs_rad, T_degK, Hp_m, roc_mps, plus DeltaT_degK)
    _Pnav->_Pair->execute_step(Ost_nav_out, Ost_sens_out, Ost_nav_in, s);
    // run attitude filter (fills up Ost_nav: q_nedbfs, w_nbb_rps, E_gyr, E_mag)
    _Pnav->_Patt->execute_step(Ost_nav_out, Ost_nav_out_prev, Ost_sens_out, Ost_nav_in, s);
    // run gps filter (fills up Ost_nav: x_gdt_rad_m, v_ned_mps, f_ibb_mps2, E_acc, plus Deltap_pa)
    _Pnav->_Pgps->execute_step(Ost_nav_out, Ost_sens_out, Ost_gps_out, Ost_nav_in, s, g, flag_gps);
}
/* execute step */

void nav::execute_step_functor_gps::resize(const unsigned long& s, const unsigned long& g) {
    _Pnav->_Pair->resize(s);
    _Pnav->_Patt->resize(s);
    _Pnav->_Pgps->resize(s, g);
    _Pnav->_Ppos->resize(1);
}
/* resize of all filter components to input size */

void nav::execute_step_functor_pos::execute_step(st::st_nav_out& Ost_nav_out, const st::st_nav_out& Ost_nav_out_prev, const st::st_sens_out& Ost_sens_out,
                                                 const st::st_gps_out& Ost_gps_out, const st::st_nav_in& Ost_nav_in, const unsigned int& s, const unsigned int& g, bool flag_gps) {
    Ost_nav_out.get_t_sec() = Ost_sens_out.get_t_sec();
    // run air data filter (fills up Ost_nav: vtas_mps, euler_wfsbfs_rad, T_degK, Hp_m, roc_mps, plus DeltaT_degK)
    _Pnav->_Pair->execute_step(Ost_nav_out, Ost_sens_out, Ost_nav_in, s);
    // run attitude filter (fills up Ost_nav: q_nedbfs, w_nbb_rps, E_gyr, E_mag)
    _Pnav->_Patt->execute_step(Ost_nav_out, Ost_nav_out_prev, Ost_sens_out, Ost_nav_in, s);
    // run position filter (fills up Ost_nav: x_gdt_rad_m, v_ned_mps, f_ibb_mps2, E_acc, plus Deltap_pa)
    _Pnav->_Ppos->execute_step(Ost_nav_out, Ost_nav_out_prev, Ost_sens_out, Ost_nav_in, s);
}
/* execute step */

void nav::execute_step_functor_pos::resize(const unsigned long& s, const unsigned long& g) {
    _Pnav->_Pair->resize(s);
    _Pnav->_Patt->resize(s);
    _Pnav->_Ppos->resize(s + 1 - _Pnav->_Pgps->get_size_fast());
}
/* resize of all filter components to input size */

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER_NAV
// ================
// ================

nav::filter_nav::filter_nav(nav::logic::NAV_ID nav_id, const sens::suite& Osuite, const env::earth& Oearth, const double& turb_factor)
        : _Pgeo(&(Oearth.get_geo())),  _Pair(nullptr), _Patt(nullptr), _Pgps(nullptr), _Ppos(nullptr), _Pstep(nullptr) {
    switch(nav_id) {
        case nav::logic::nav_id010201:
            _Pair  = new nav::filter_air01(Osuite);
            _Patt  = new nav::filter_att02(Osuite, Oearth.get_geo());
            _Pgps  = new nav::filter_gps01(Osuite, Oearth.get_geo());
            _Ppos  = new nav::filter_pos01B(Osuite, Oearth.get_geo());
            _Pstep = new nav::execute_step_functor_gps(*this);
            break;
        case nav::logic::nav_id010202:
            _Pair  = new nav::filter_air01(Osuite);
            _Patt  = new nav::filter_att02(Osuite, Oearth.get_geo());
            _Pgps  = new nav::filter_gps01(Osuite, Oearth.get_geo());
            _Ppos  = new nav::filter_pos02B(Osuite, Oearth.get_geo());
            _Pstep = new nav::execute_step_functor_gps(*this);
            break;
        case nav::logic::nav_id020203:
            _Pair  = new nav::filter_air02(Osuite);
            _Patt  = new nav::filter_att02(Osuite, Oearth.get_geo());
            _Pgps  = new nav::filter_gps01(Osuite, Oearth.get_geo());
            _Ppos  = new nav::filter_pos03A(Osuite, Oearth.get_geo());
            _Pstep = new nav::execute_step_functor_gps(*this);
            break;
        case nav::logic::nav_id020202:
            _Pair  = new nav::filter_air02(Osuite);
            _Patt  = new nav::filter_att02(Osuite, Oearth.get_geo());
            _Pgps  = new nav::filter_gps01(Osuite, Oearth.get_geo());
            _Ppos  = new nav::filter_pos02B(Osuite, Oearth.get_geo()); // default configuration is 2B
            _Pstep = new nav::execute_step_functor_gps(*this);
            break;
        case nav::logic::nav_id020302:
            _Pair  = new nav::filter_air02(Osuite);
            _Patt  = new nav::filter_att03(Osuite, Oearth.get_geo());
            _Pgps  = new nav::filter_gps01(Osuite, Oearth.get_geo());
            _Ppos  = new nav::filter_pos02B(Osuite, Oearth.get_geo()); // default configuration is 2B
            _Pstep = new nav::execute_step_functor_gps(*this);
            break;
        case nav::logic::nav_id020402:
            _Pair  = new nav::filter_air02(Osuite);
            _Patt  = new nav::filter_att04(Osuite, Oearth.get_geo());
            _Pgps  = new nav::filter_gps01(Osuite, Oearth.get_geo());
            _Ppos  = new nav::filter_pos02B(Osuite, Oearth.get_geo()); // default configuration is 2B
            _Pstep = new nav::execute_step_functor_gps(*this);
            break;
        case nav::logic::nav_size:
            throw std::runtime_error("Navigation filter model not available");
        default:
            throw std::runtime_error("Navigation filter model not available");
    }
}
/* constructor based on enumeration specifying filter type, suite of sensors, earth model, and turbulence factor */

nav::filter_nav::~filter_nav() {
    delete _Pair;
    delete _Patt;
    delete _Pgps;
    delete _Ppos;
    delete _Pstep;
}
/* destructor */

void nav::filter_nav::finalize_constructor(const double& Deltat_sec_nav, const unsigned int& nel_nav, const unsigned int& nel_gps, const unsigned int& nel_nav_gps_actv, const unsigned int& nel_nav_gps_lost) {
    _Pair->finalize_constructor(Deltat_sec_nav, nel_nav);
    _Patt->finalize_constructor(Deltat_sec_nav, nel_nav);
    _Pgps->finalize_constructor(Deltat_sec_nav, nel_nav_gps_actv, nel_gps);
    _Ppos->finalize_constructor(Deltat_sec_nav, nel_nav_gps_lost);
}
/* complete constructor with navigation time and sizes of navigation vector, gps vector, gps filter, and pos filter, which are not available at construction time */

void nav::filter_nav::initialize(st::st_nav_out& Ost_nav_out_init, const st::st_sens_out& Ost_sens_out_init, const st::st_gps_out& Ost_gps_out_init, const st::st_nav_in& Ost_nav_in_init,
                                  const ang::rodrigues& q_nb_init,
                                  const Eigen::Vector3d& E_gyr_init, const Eigen::Vector3d& E_gyr_std_init,
                                  const Eigen::Vector3d& E_mag_init, const Eigen::Vector3d& E_mag_std_init,
                                  const Eigen::Vector3d& E_acc_init, const Eigen::Vector3d& E_acc_std_init,
                                  const Eigen::Vector3d& E_B_n_init, const Eigen::Vector3d& E_B_n_std_init) {
    _Pair->initialize(Ost_nav_out_init, Ost_sens_out_init, Ost_nav_in_init);
    _Patt->initialize(Ost_nav_out_init, Ost_sens_out_init, Ost_nav_in_init, q_nb_init, E_gyr_init, E_gyr_std_init, E_mag_init, E_mag_std_init, E_B_n_init, E_B_n_std_init);
    _Pgps->initialize(Ost_nav_out_init, Ost_sens_out_init, Ost_gps_out_init, Ost_nav_in_init, E_acc_init, E_acc_std_init);
    // Note that position filter is not initialized at this time
}
/* initialize filter filling up the initial navigation output state vector Ost_nav_out_init, based on the initial sensors output state vector Ost_sens_out_init,
 * the initial GPS output state vector Ost_gps_out_init, the initial navigation input state vector Ost_nav_in_init (for filter evaluation purposes only), and other
 * randomly computed variables required for filter initialization, such as initial attitude, initial gyroscope error, initial magnetometer error, initial gravity
 * vector, and initial magnetic field). */

void nav::filter_nav::execute_step(st::st_nav_out& Ost_nav_out, const st::st_nav_out& Ost_nav_out_prev, const st::st_sens_out& Ost_sens_out,
                                    const st::st_gps_out& Ost_gps_out, const st::st_nav_in& Ost_nav_in, const unsigned int& s, const unsigned int& g, bool flag_gps) {
    _Pstep->execute_step(Ost_nav_out, Ost_nav_out_prev, Ost_sens_out, Ost_gps_out, Ost_nav_in, s, g, flag_gps);
}
/* execute filter step filling up the navigation output state vector Ost_nav_out, based on the previous navigation output state vector Ost_nav_out_prev,
 * the sensors output state vector Ost_sens_out, the GPS output state vector Ost_gps_out, the navigation input state vector Ost_nav_in (for filter evaluation
 * purposes only), the current positions for navigation and GPS in their respective trajectory vectors, and the flag indicating whether there is a new GPS
 * measurement or not. */

void nav::filter_nav::resize(const unsigned long& s, const unsigned long& g) {
    _Pstep->resize(s, g);
}
/* resize of all filter components to input size */

void nav::filter_nav::switch_to_gps_lost_mode(const st::st_nav_out& Ost_nav_out_init, const unsigned int& s_init) {
    delete _Pstep;
    _Pstep = new execute_step_functor_pos(*this);
    _Patt->switch_to_gps_lost_mode();
    _Ppos->initialize(*_Pgps, Ost_nav_out_init, s_init);
}
/* move from gps active to gps lost mode */

void nav::filter_nav::text_filter(const boost::filesystem::path& path_folder,
                                   const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out, const st::trj_gps_out& Otrj_gps_out) const {
    _Pair->textplot(path_folder, Otrj_sens_in, Otrj_sens_out, Otrj_nav_in, Otrj_nav_out);
    _Patt->textplot(path_folder, Otrj_sens_in, Otrj_sens_out, Otrj_nav_in, Otrj_nav_out);
    _Pgps->textplot(path_folder, Otrj_sens_in, Otrj_sens_out, Otrj_nav_in, Otrj_nav_out, Otrj_gps_out);
    _Ppos->textplot(path_folder, Otrj_sens_in, Otrj_sens_out, Otrj_nav_in, Otrj_nav_out);
}
/* create text files with filter results for later plotting in Matlab */










