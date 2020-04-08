#include "Ttrj_sizes.h"

#include "nav/kalman/kf_.h"
#include "nav/kalman/kf_handler_.h"
#include "ang/auxiliary.h"
#include "ang/tools.h"
#include "ang/rotate/euler.h"
#include "ang/rotate/rodrigues.h"
#include "acft/acft/aero0.h"
#include "acft/control/cntr.h"
#include "acft/sens/suite.h"
#include "env/geo.h"
#include "env/turb.h"
#include "nav/motion/motion.h"
#include "nav/motion/textplot.h"
#include "nav/nav/filter_nav.h"
#include <fstream>

using namespace control::logic;

nav::test::Ttrj_sizes::Ttrj_sizes(jail::counter& Ocounter)
: ::jail::unit_test(Ocounter) {
}
/* constructor based on counter */

void nav::test::Ttrj_sizes::run() {
	::jail::unit_test::run();

    test_sizes01();
    test_sizes02();
    test_sizes03();
    test_sizes04();
    test_sizes05();
    test_sizes06();
    test_sizes07();
    test_sizes08();
    test_sizes11();
    test_sizes12();
    test_sizes13();
    test_sizes14();
    test_sizes15();
    test_sizes16();
    test_sizes17();
    test_sizes18();
    test_sizes21();
    test_sizes22();
    test_sizes23();

    // #op    t_end_trj    t_end_op    t_end_gps    comments
    //  01      10          10           5          trj & op conclude simultaneously
    //  02      10          10          10
    //  03      10          10          15
    //  04      20          10  20       5
    //  05      20          10  20      10
    //  06      20          10  20      15
    //  07      20          10  20      20
    //  08      20          10  20      25
    //  11      20          10           5           op concludes before trj
    //  12      20          10          10
    //  13      20          10          15
    //  14      30          10  20       5
    //  15      30          10  20      10
    //  16      30          10  20      15
    //  17      30          10  20      20
    //  18      30          10  20      25
    //  21      10          20           5           trj concludes before op
    //  22      10          20          10
    //  23      10          20          15

	finished();
}
/* execute tests and write results on console */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void nav::test::Ttrj_sizes::test_sizes01() {
    unsigned short seed_order = 1;
    double turb_factor        = 1.0;

    double t_sec_init    = -5.;
    double t_sec_end     = 10.0;
    int    t_sec_turb    = 1000;
    double t_sec_gpsloss = 5.0;

    sens::logic::SENS_COMPLETE_ID sens_complete_id = sens::logic::sens_complete_bias_scale_par;
    nav::logic::NAV_ID nav_id                      = nav::logic::nav_id010202;

    env::logic::WIND_ID wind_id         = env::logic::wind_id00;
    env::logic::OFFSETS_ID offsets_id   = env::logic::offsets_id00;
    env::logic::MAG_ID env_mag_id       = env::logic::mag_default;
    env::logic::REALISM_ID realism_id   = env::logic::realism_grav_no_magn_no;

    nav::logic::INITEUL_ID initeul_id   = nav::logic::initeul_id_zero;
    nav::logic::INITACC_ID initacc_id   = nav::logic::initacc_id_zero;
    nav::logic::INITGYR_ID initgyr_id   = nav::logic::initgyr_id_zero;
    nav::logic::INITMAG_ID initmag_id   = nav::logic::initmag_id_zero;
    nav::logic::INITMGN_ID initmgn_id   = nav::logic::initmgn_id_zero;

    sens::logic::GYR_ID gyr_id          = sens::logic::gyr_id_base;
    sens::logic::ACC_ID acc_id          = sens::logic::acc_id_base;
    sens::logic::MAG_ID mag_id          = sens::logic::mag_id_base;
    sens::logic::OSP_ID osp_id          = sens::logic::osp_id_base;
    sens::logic::OAT_ID oat_id          = sens::logic::oat_id_base;
    sens::logic::TAS_ID tas_id          = sens::logic::tas_id_base;
    sens::logic::AOA_ID aoa_id          = sens::logic::aoa_id_base;
    sens::logic::AOS_ID aos_id          = sens::logic::aos_id_base;
    sens::logic::GPS_ID gps_id          = sens::logic::gps_id_base;
    sens::logic::BAND_ID band_id        = sens::logic::band_id_base;

    st::logic::STI_ID sti_id = st::logic::sti_Hp3000_tas30_chi00;
    unsigned short nel_op = 1;

    math::seeder Oseeder(seed_order);
    auto Paero                = new acft::aero0();
    auto Pprop                = new acft::prop();
    auto Piner                = new acft::iner();
    auto Pearth               = new env::earth(Oseeder, wind_id, offsets_id, env_mag_id, realism_id, turb_factor, t_sec_init, t_sec_turb);
    auto Psuite               = new sens::suite(Oseeder, acc_id, gyr_id, mag_id, osp_id, oat_id, tas_id, aoa_id, aos_id, gps_id, band_id);
    auto Perr0                = new nav::init_error(Oseeder, initeul_id, initacc_id, initgyr_id, initmag_id, initmgn_id);
    auto Pguid                = new control::guid(nel_op, t_sec_end, Oseeder.provide_seed(math::seeder::seeder_guid),0);
    Pguid->get_sti_id() = sti_id;
    Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 0.0, rud_beta_deg, 0., trgg_t_sec, t_sec_end));
    st::sti* Psti             = st::sti::create_sti(*Pguid, t_sec_init);
    auto Pcntr                = new control::cntr(*Pguid);
    auto             Pnav     = new nav::filter_nav(nav_id, *Psuite, *Pearth, turb_factor);
    ang::euler euler_bc(0.0, 0.0, 0.0);
    auto Pcam                 = new sens::camera(768, 1024, 49.2255, euler_bc);

    nav::motion Omotion(Psti, Pguid, Pcntr, Pnav, Pearth, Psuite, Perr0, Piner, Paero, Pprop, Pcam, sens_complete_id);
    Omotion.solve_2nd(t_sec_end, t_sec_gpsloss);

    double Deltat_sec_truth = 0.002;
    double Deltat_sec_sens  = 0.01;
    double Deltat_sec_gps   = 1.0;
    double Deltat_sec_cntr  = 0.02;
    double Deltat_sec_out   = 0.1;

    check("01 truth nel           ", 1 + ((- t_sec_init + t_sec_end) / Deltat_sec_truth),   Omotion.get_trj_truth().get_nel(),              1e-10);
    check("01 truth nel_op        ", nel_op,                                                Omotion.get_trj_truth().get_nel_op(),           1e-10);
    check("01 truth last time     ", t_sec_end,                                             Omotion.get_trj_truth()().back().get_t_sec(),   1e-10);
    check("01 truth op 0 start    ", 0,                                                     Omotion.get_trj_truth().get_op_start(0),        1e-10);
    check("01 truth op 0 end      ", (- t_sec_init + t_sec_end) / Deltat_sec_truth,         Omotion.get_trj_truth().get_op_end(0),          1e-10);

    check("01 sens_in nel         ", 1 + ((- t_sec_init + t_sec_end) / Deltat_sec_sens),    Omotion.get_trj_sens_in().get_nel(),            1e-10);
    check("01 sens_in nel_op      ", nel_op,                                                Omotion.get_trj_sens_in().get_nel_op(),         1e-10);
    check("01 sens_in last time   ", t_sec_end,                                             Omotion.get_trj_sens_in()().back().get_t_sec(), 1e-10);
    check("01 sens_in op 0 start  ", 0,                                                     Omotion.get_trj_sens_in().get_op_start(0),      1e-10);
    check("01 sens_in op 0 end    ", (- t_sec_init + t_sec_end) / Deltat_sec_sens,          Omotion.get_trj_sens_in().get_op_end(0),        1e-10);

    check("01 sens_out nel        ", 1 + ((- t_sec_init + t_sec_end) / Deltat_sec_sens),    Omotion.get_trj_sens_out().get_nel(),           1e-10);
    check("01 sens_out nel_op     ", nel_op,                                                Omotion.get_trj_sens_out().get_nel_op(),        1e-10);
    check("01 sens_out last time  ", t_sec_end,                                             Omotion.get_trj_sens_out()().back().get_t_sec(),1e-10);
    check("01 sens_out op 0 start ", 0,                                                     Omotion.get_trj_sens_out().get_op_start(0),     1e-10);
    check("01 sens_out op 0 end   ", (- t_sec_init + t_sec_end) / Deltat_sec_sens,          Omotion.get_trj_sens_out().get_op_end(0),       1e-10);

    check("01 cntr nel            ", 1 + ((- t_sec_init + t_sec_end) / Deltat_sec_cntr),    Omotion.get_trj_cntr().get_nel(),               1e-10);
    check("01 cntr nel_op         ", nel_op,                                                Omotion.get_trj_cntr().get_nel_op(),            1e-10);
    check("01 cntr last time      ", t_sec_end,                                             Omotion.get_trj_cntr()().back().get_t_sec(),    1e-10);
    check("01 cntr op 0 start     ", 0,                                                     Omotion.get_trj_cntr().get_op_start(0),         1e-10);
    check("01 cntr op 0 end       ", (- t_sec_init + t_sec_end) / Deltat_sec_cntr,          Omotion.get_trj_cntr().get_op_end(0),           1e-10);

    check("01 nav_in nel          ", 1 + ((- t_sec_init + t_sec_end) / Deltat_sec_sens),    Omotion.get_trj_nav_in().get_nel(),             1e-10);
    check("01 nav_in nel_op       ", nel_op,                                                Omotion.get_trj_nav_in().get_nel_op(),          1e-10);
    check("01 nav_in last time    ", t_sec_end,                                             Omotion.get_trj_nav_in()().back().get_t_sec(),  1e-10);
    check("01 nav_in op 0 start   ", 0,                                                     Omotion.get_trj_nav_in().get_op_start(0),       1e-10);
    check("01 nav_in op 0 end     ", (- t_sec_init + t_sec_end) / Deltat_sec_sens,          Omotion.get_trj_nav_in().get_op_end(0),         1e-10);

    check("01 nav_out nel         ", 1 + ((- t_sec_init + t_sec_end) / Deltat_sec_sens),    Omotion.get_trj_nav_out().get_nel(),            1e-10);
    check("01 nav_out nel_op      ", nel_op,                                                Omotion.get_trj_nav_out().get_nel_op(),         1e-10);
    check("01 nav_out last time   ", t_sec_end,                                             Omotion.get_trj_nav_out()().back().get_t_sec(), 1e-10);
    check("01 nav_out op 0 start  ", 0,                                                     Omotion.get_trj_nav_out().get_op_start(0),      1e-10);
    check("01 nav_out op 0 end    ", (- t_sec_init + t_sec_end) / Deltat_sec_sens,          Omotion.get_trj_nav_out().get_op_end(0),        1e-10);

    check("01 gps_out nel         ", 1 + ((- t_sec_init + t_sec_gpsloss) / Deltat_sec_gps), Omotion.get_trj_gps_out().get_nel(),            1e-10);
    check("01 gps_out nel_op      ", nel_op,                                                Omotion.get_trj_gps_out().get_nel_op(),         1e-10);
    check("01 gps_out last time   ", t_sec_gpsloss,                                         Omotion.get_trj_gps_out()().back().get_t_sec(), 1e-10);
    check("01 gps_out op 0 start  ", 0,                                                     Omotion.get_trj_gps_out().get_op_start(0),      1e-10);
    check("01 gps_out op 0 end    ", (- t_sec_init + t_sec_gpsloss) / Deltat_sec_gps,       Omotion.get_trj_gps_out().get_op_end(0),        1e-10);

    check("01 out nel             ", 1 + ((- t_sec_init + t_sec_end) / Deltat_sec_out),     Omotion.get_trj_out().get_nel(),                1e-10);
    check("01 out nel_op          ", nel_op,                                                Omotion.get_trj_out().get_nel_op(),             1e-10);
    check("01 out last time       ", t_sec_end,                                             Omotion.get_trj_out()().back().get_t_sec(),     1e-10);
    check("01 out op 0 start      ", 0,                                                     Omotion.get_trj_out().get_op_start(0),          1e-10);
    check("01 out op 0 end        ", (- t_sec_init + t_sec_end) / Deltat_sec_out,           Omotion.get_trj_out().get_op_end(0),            1e-10);

    check("01 filter air          ", 1 + (- t_sec_init  + t_sec_end) / Deltat_sec_sens,     Omotion.get_filter_nav().get_filter_air().get_size());
    check("01 filter att          ", 1 + (- t_sec_init  + t_sec_end) / Deltat_sec_sens,     Omotion.get_filter_nav().get_filter_att().get_size());
    check("01 filter gps fast     ", 1 + (- t_sec_init  + t_sec_gpsloss) / Deltat_sec_sens, Omotion.get_filter_nav().get_filter_gps().get_size_fast());
    check("01 filter gps slow     ", 1 + (- t_sec_init  + t_sec_gpsloss) / Deltat_sec_gps,  Omotion.get_filter_nav().get_filter_gps().get_size_slow());
    check("01 filter pos          ", 1 + (- t_sec_gpsloss + t_sec_end) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_pos().get_size());
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void nav::test::Ttrj_sizes::test_sizes02() {
    unsigned short seed_order = 1;
    double turb_factor        = 1.0;

    double t_sec_init    = -5.;
    double t_sec_end     = 10.0;
    int    t_sec_turb    = 1000;
    double t_sec_gpsloss = 10.0;

    sens::logic::SENS_COMPLETE_ID sens_complete_id = sens::logic::sens_complete_bias_scale_par;
    nav::logic::NAV_ID nav_id                      = nav::logic::nav_id010202;

    env::logic::WIND_ID wind_id         = env::logic::wind_id00;
    env::logic::OFFSETS_ID offsets_id   = env::logic::offsets_id00;
    env::logic::MAG_ID env_mag_id       = env::logic::mag_default;
    env::logic::REALISM_ID realism_id   = env::logic::realism_grav_no_magn_no;

    nav::logic::INITEUL_ID initeul_id   = nav::logic::initeul_id_zero;
    nav::logic::INITACC_ID initacc_id   = nav::logic::initacc_id_zero;
    nav::logic::INITGYR_ID initgyr_id   = nav::logic::initgyr_id_zero;
    nav::logic::INITMAG_ID initmag_id   = nav::logic::initmag_id_zero;
    nav::logic::INITMGN_ID initmgn_id   = nav::logic::initmgn_id_zero;

    sens::logic::GYR_ID gyr_id          = sens::logic::gyr_id_base;
    sens::logic::ACC_ID acc_id          = sens::logic::acc_id_base;
    sens::logic::MAG_ID mag_id          = sens::logic::mag_id_base;
    sens::logic::OSP_ID osp_id          = sens::logic::osp_id_base;
    sens::logic::OAT_ID oat_id          = sens::logic::oat_id_base;
    sens::logic::TAS_ID tas_id          = sens::logic::tas_id_base;
    sens::logic::AOA_ID aoa_id          = sens::logic::aoa_id_base;
    sens::logic::AOS_ID aos_id          = sens::logic::aos_id_base;
    sens::logic::GPS_ID gps_id          = sens::logic::gps_id_base;
    sens::logic::BAND_ID band_id        = sens::logic::band_id_base;

    st::logic::STI_ID sti_id = st::logic::sti_Hp3000_tas30_chi00;
    unsigned short nel_op = 1;

    math::seeder Oseeder(seed_order);
    auto Paero                = new acft::aero0();
    auto Pprop                = new acft::prop();
    auto Piner                = new acft::iner();
    auto Pearth               = new env::earth(Oseeder, wind_id, offsets_id, env_mag_id, realism_id, turb_factor, t_sec_init, t_sec_turb);
    auto Psuite               = new sens::suite(Oseeder, acc_id, gyr_id, mag_id, osp_id, oat_id, tas_id, aoa_id, aos_id, gps_id, band_id);
    auto Perr0                = new nav::init_error(Oseeder, initeul_id, initacc_id, initgyr_id, initmag_id, initmgn_id);
    auto Pguid                = new control::guid(nel_op, t_sec_end, Oseeder.provide_seed(math::seeder::seeder_guid),0);
    Pguid->get_sti_id() = sti_id;
    Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 0.0, rud_beta_deg, 0., trgg_t_sec, t_sec_end));
    st::sti* Psti             = st::sti::create_sti(*Pguid, t_sec_init);
    auto Pcntr                = new control::cntr(*Pguid);
    auto             Pnav     = new nav::filter_nav(nav_id, *Psuite, *Pearth, turb_factor);
    ang::euler euler_bc(0.0, 0.0, 0.0);
    auto Pcam                 = new sens::camera(768, 1024, 49.2255, euler_bc);

    nav::motion Omotion(Psti, Pguid, Pcntr, Pnav, Pearth, Psuite, Perr0, Piner, Paero, Pprop, Pcam, sens_complete_id);
    Omotion.solve_2nd(t_sec_end, t_sec_gpsloss);

    double Deltat_sec_truth = 0.002;
    double Deltat_sec_sens  = 0.01;
    double Deltat_sec_gps   = 1.0;
    double Deltat_sec_cntr  = 0.02;
    double Deltat_sec_out   = 0.1;

    check("02 truth nel           ", 1 + ((- t_sec_init + t_sec_end) / Deltat_sec_truth),   Omotion.get_trj_truth().get_nel(),               1e-10);
    check("02 truth nel_op        ", nel_op,                                                Omotion.get_trj_truth().get_nel_op(),            1e-10);
    check("02 truth last time     ", t_sec_end,                                             Omotion.get_trj_truth()().back().get_t_sec(),    1e-10);
    check("02 truth op 0 start    ", 0,                                                     Omotion.get_trj_truth().get_op_start(0),         1e-10);
    check("02 truth op 0 end      ", (- t_sec_init + t_sec_end) / Deltat_sec_truth,         Omotion.get_trj_truth().get_op_end(0),           1e-10);

    check("02 sens_in nel         ", 1 + ((- t_sec_init + t_sec_end) / Deltat_sec_sens),    Omotion.get_trj_sens_in().get_nel(),             1e-10);
    check("02 sens_in nel_op      ", nel_op,                                                Omotion.get_trj_sens_in().get_nel_op(),          1e-10);
    check("02 sens_in last time   ", t_sec_end,                                             Omotion.get_trj_sens_in()().back().get_t_sec(),  1e-10);
    check("02 sens_in op 0 start  ", 0,                                                     Omotion.get_trj_sens_in().get_op_start(0),       1e-10);
    check("02 sens_in op 0 end    ", (- t_sec_init + t_sec_end) / Deltat_sec_sens,          Omotion.get_trj_sens_in().get_op_end(0),         1e-10);

    check("02 sens_out nel        ", 1 + ((- t_sec_init + t_sec_end) / Deltat_sec_sens),    Omotion.get_trj_sens_out().get_nel(),            1e-10);
    check("02 sens_out nel_op     ", nel_op,                                                Omotion.get_trj_sens_out().get_nel_op(),         1e-10);
    check("02 sens_out last time  ", t_sec_end,                                             Omotion.get_trj_sens_out()().back().get_t_sec(), 1e-10);
    check("02 sens_out op 0 start ", 0,                                                     Omotion.get_trj_sens_out().get_op_start(0),      1e-10);
    check("02 sens_out op 0 end   ", (- t_sec_init + t_sec_end) / Deltat_sec_sens,          Omotion.get_trj_sens_out().get_op_end(0),        1e-10);

    check("02 cntr nel            ", 1 + ((- t_sec_init + t_sec_end) / Deltat_sec_cntr),    Omotion.get_trj_cntr().get_nel(),                1e-10);
    check("02 cntr nel_op         ", nel_op,                                                Omotion.get_trj_cntr().get_nel_op(),             1e-10);
    check("02 cntr last time      ", t_sec_end,                                             Omotion.get_trj_cntr()().back().get_t_sec(),     1e-10);
    check("02 cntr op 0 start     ", 0,                                                     Omotion.get_trj_cntr().get_op_start(0),          1e-10);
    check("02 cntr op 0 end       ", (- t_sec_init + t_sec_end) / Deltat_sec_cntr,          Omotion.get_trj_cntr().get_op_end(0),            1e-10);

    check("02 nav_in nel          ", 1 + ((- t_sec_init + t_sec_end) / Deltat_sec_sens),    Omotion.get_trj_nav_in().get_nel(),              1e-10);
    check("02 nav_in nel_op       ", nel_op,                                                Omotion.get_trj_nav_in().get_nel_op(),           1e-10);
    check("02 nav_in last time    ", t_sec_end,                                             Omotion.get_trj_nav_in()().back().get_t_sec(),   1e-10);
    check("02 nav_in op 0 start   ", 0,                                                     Omotion.get_trj_nav_in().get_op_start(0),        1e-10);
    check("02 nav_in op 0 end     ", (- t_sec_init + t_sec_end) / Deltat_sec_sens,          Omotion.get_trj_nav_in().get_op_end(0),          1e-10);

    check("02 nav_out nel         ", 1 + ((- t_sec_init + t_sec_end) / Deltat_sec_sens),    Omotion.get_trj_nav_out().get_nel(),             1e-10);
    check("02 nav_out nel_op      ", nel_op,                                                Omotion.get_trj_nav_out().get_nel_op(),          1e-10);
    check("02 nav_out last time   ", t_sec_end,                                             Omotion.get_trj_nav_out()().back().get_t_sec(),  1e-10);
    check("02 nav_out op 0 start  ", 0,                                                     Omotion.get_trj_nav_out().get_op_start(0),       1e-10);
    check("02 nav_out op 0 end    ", (- t_sec_init + t_sec_end) / Deltat_sec_sens,          Omotion.get_trj_nav_out().get_op_end(0),         1e-10);

    check("02 gps_out nel         ", 1 + ((- t_sec_init + t_sec_gpsloss) / Deltat_sec_gps), Omotion.get_trj_gps_out().get_nel(),             1e-10);
    check("02 gps_out nel_op      ", nel_op,                                                Omotion.get_trj_gps_out().get_nel_op(),          1e-10);
    check("02 gps_out last time   ", t_sec_gpsloss,                                         Omotion.get_trj_gps_out()().back().get_t_sec(),  1e-10);
    check("02 gps_out op 0 start  ", 0,                                                     Omotion.get_trj_gps_out().get_op_start(0),       1e-10);
    check("02 gps_out op 0 end    ", (- t_sec_init + t_sec_gpsloss) / Deltat_sec_gps,       Omotion.get_trj_gps_out().get_op_end(0),         1e-10);

    check("02 out nel             ", 1 + ((- t_sec_init + t_sec_end) / Deltat_sec_out),     Omotion.get_trj_out().get_nel(),                 1e-10);
    check("02 out nel_op          ", nel_op,                                                Omotion.get_trj_out().get_nel_op(),              1e-10);
    check("02 out last time       ", t_sec_end,                                             Omotion.get_trj_out()().back().get_t_sec(),      1e-10);
    check("02 out op 0 start      ", 0,                                                     Omotion.get_trj_out().get_op_start(0),           1e-10);
    check("02 out op 0 end        ", (- t_sec_init + t_sec_end) / Deltat_sec_out,           Omotion.get_trj_out().get_op_end(0),             1e-10);

    check("02 filter air          ", 1 + (- t_sec_init  + t_sec_end) / Deltat_sec_sens,     Omotion.get_filter_nav().get_filter_air().get_size());
    check("02 filter att          ", 1 + (- t_sec_init  + t_sec_end) / Deltat_sec_sens,     Omotion.get_filter_nav().get_filter_att().get_size());
    check("02 filter gps fast     ", 1 + (- t_sec_init  + t_sec_gpsloss) / Deltat_sec_sens, Omotion.get_filter_nav().get_filter_gps().get_size_fast());
    check("02 filter gps slow     ", 1 + (- t_sec_init  + t_sec_gpsloss) / Deltat_sec_gps,  Omotion.get_filter_nav().get_filter_gps().get_size_slow());
    check("02 filter pos          ", 1 + (- t_sec_gpsloss + t_sec_end) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_pos().get_size());
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void nav::test::Ttrj_sizes::test_sizes03() {
    unsigned short seed_order = 1;
    double turb_factor        = 1.0;

    double t_sec_init    = -5.;
    double t_sec_end     = 10.0;
    int    t_sec_turb    = 1000;
    double t_sec_gpsloss = 15.0;

    sens::logic::SENS_COMPLETE_ID sens_complete_id = sens::logic::sens_complete_bias_scale_par;
    nav::logic::NAV_ID nav_id                      = nav::logic::nav_id010202;

    env::logic::WIND_ID wind_id         = env::logic::wind_id00;
    env::logic::OFFSETS_ID offsets_id   = env::logic::offsets_id00;
    env::logic::MAG_ID env_mag_id       = env::logic::mag_default;
    env::logic::REALISM_ID realism_id   = env::logic::realism_grav_no_magn_no;

    nav::logic::INITEUL_ID initeul_id   = nav::logic::initeul_id_zero;
    nav::logic::INITACC_ID initacc_id   = nav::logic::initacc_id_zero;
    nav::logic::INITGYR_ID initgyr_id   = nav::logic::initgyr_id_zero;
    nav::logic::INITMAG_ID initmag_id   = nav::logic::initmag_id_zero;
    nav::logic::INITMGN_ID initmgn_id   = nav::logic::initmgn_id_zero;

    sens::logic::GYR_ID gyr_id          = sens::logic::gyr_id_base;
    sens::logic::ACC_ID acc_id          = sens::logic::acc_id_base;
    sens::logic::MAG_ID mag_id          = sens::logic::mag_id_base;
    sens::logic::OSP_ID osp_id          = sens::logic::osp_id_base;
    sens::logic::OAT_ID oat_id          = sens::logic::oat_id_base;
    sens::logic::TAS_ID tas_id          = sens::logic::tas_id_base;
    sens::logic::AOA_ID aoa_id          = sens::logic::aoa_id_base;
    sens::logic::AOS_ID aos_id          = sens::logic::aos_id_base;
    sens::logic::GPS_ID gps_id          = sens::logic::gps_id_base;
    sens::logic::BAND_ID band_id        = sens::logic::band_id_base;

    st::logic::STI_ID sti_id = st::logic::sti_Hp3000_tas30_chi00;
    unsigned short nel_op = 1;

    math::seeder Oseeder(seed_order);
    auto Paero                = new acft::aero0();
    auto Pprop                = new acft::prop();
    auto Piner                = new acft::iner();
    auto Pearth               = new env::earth(Oseeder, wind_id, offsets_id, env_mag_id, realism_id, turb_factor, t_sec_init, t_sec_turb);
    auto Psuite               = new sens::suite(Oseeder, acc_id, gyr_id, mag_id, osp_id, oat_id, tas_id, aoa_id, aos_id, gps_id, band_id);
    auto Perr0                = new nav::init_error(Oseeder, initeul_id, initacc_id, initgyr_id, initmag_id, initmgn_id);
    auto Pguid                = new control::guid(nel_op, t_sec_end, Oseeder.provide_seed(math::seeder::seeder_guid),0);
    Pguid->get_sti_id() = sti_id;
    Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 0.0, rud_beta_deg, 0., trgg_t_sec, t_sec_end));
    st::sti* Psti             = st::sti::create_sti(*Pguid, t_sec_init);
    auto Pcntr                = new control::cntr(*Pguid);
    auto             Pnav     = new nav::filter_nav(nav_id, *Psuite, *Pearth, turb_factor);
    ang::euler euler_bc(0.0, 0.0, 0.0);
    auto Pcam                 = new sens::camera(768, 1024, 49.2255, euler_bc);

    nav::motion Omotion(Psti, Pguid, Pcntr, Pnav, Pearth, Psuite, Perr0, Piner, Paero, Pprop, Pcam, sens_complete_id);
    Omotion.solve_2nd(t_sec_end, t_sec_gpsloss);

    double Deltat_sec_truth = 0.002;
    double Deltat_sec_sens  = 0.01;
    double Deltat_sec_gps   = 1.0;
    double Deltat_sec_cntr  = 0.02;
    double Deltat_sec_out   = 0.1;

    check("03 truth nel           ", 1 + ((- t_sec_init + t_sec_end) / Deltat_sec_truth), Omotion.get_trj_truth().get_nel(),               1e-10);
    check("03 truth nel_op        ", nel_op,                                              Omotion.get_trj_truth().get_nel_op(),            1e-10);
    check("03 truth last time     ", t_sec_end,                                           Omotion.get_trj_truth()().back().get_t_sec(),    1e-10);
    check("03 truth op 0 start    ", 0,                                                   Omotion.get_trj_truth().get_op_start(0),         1e-10);
    check("03 truth op 0 end      ", (- t_sec_init + t_sec_end) / Deltat_sec_truth,       Omotion.get_trj_truth().get_op_end(0),           1e-10);

    check("03 sens_in nel         ", 1 + ((- t_sec_init + t_sec_end) / Deltat_sec_sens),  Omotion.get_trj_sens_in().get_nel(),             1e-10);
    check("03 sens_in nel_op      ", nel_op,                                              Omotion.get_trj_sens_in().get_nel_op(),          1e-10);
    check("03 sens_in last time   ", t_sec_end,                                           Omotion.get_trj_sens_in()().back().get_t_sec(),  1e-10);
    check("03 sens_in op 0 start  ", 0,                                                   Omotion.get_trj_sens_in().get_op_start(0),       1e-10);
    check("03 sens_in op 0 end    ", (- t_sec_init + t_sec_end) / Deltat_sec_sens,        Omotion.get_trj_sens_in().get_op_end(0),         1e-10);

    check("03 sens_out nel        ", 1 + ((- t_sec_init + t_sec_end) / Deltat_sec_sens),  Omotion.get_trj_sens_out().get_nel(),            1e-10);
    check("03 sens_out nel_op     ", nel_op,                                              Omotion.get_trj_sens_out().get_nel_op(),         1e-10);
    check("03 sens_out last time  ", t_sec_end,                                           Omotion.get_trj_sens_out()().back().get_t_sec(), 1e-10);
    check("03 sens_out op 0 start ", 0,                                                   Omotion.get_trj_sens_out().get_op_start(0),      1e-10);
    check("03 sens_out op 0 end   ", (- t_sec_init + t_sec_end) / Deltat_sec_sens,        Omotion.get_trj_sens_out().get_op_end(0),        1e-10);

    check("03 cntr nel            ", 1 + ((- t_sec_init + t_sec_end) / Deltat_sec_cntr),  Omotion.get_trj_cntr().get_nel(),                1e-10);
    check("03 cntr nel_op         ", nel_op,                                              Omotion.get_trj_cntr().get_nel_op(),             1e-10);
    check("03 cntr last time      ", t_sec_end,                                           Omotion.get_trj_cntr()().back().get_t_sec(),     1e-10);
    check("03 cntr op 0 start     ", 0,                                                   Omotion.get_trj_cntr().get_op_start(0),          1e-10);
    check("03 cntr op 0 end       ", (- t_sec_init + t_sec_end) / Deltat_sec_cntr,        Omotion.get_trj_cntr().get_op_end(0),            1e-10);

    check("03 nav_in nel          ", 1 + ((- t_sec_init + t_sec_end) / Deltat_sec_sens),  Omotion.get_trj_nav_in().get_nel(),              1e-10);
    check("03 nav_in nel_op       ", nel_op,                                              Omotion.get_trj_nav_in().get_nel_op(),           1e-10);
    check("03 nav_in last time    ", t_sec_end,                                           Omotion.get_trj_nav_in()().back().get_t_sec(),   1e-10);
    check("03 nav_in op 0 start   ", 0,                                                   Omotion.get_trj_nav_in().get_op_start(0),        1e-10);
    check("03 nav_in op 0 end     ", (- t_sec_init + t_sec_end) / Deltat_sec_sens,        Omotion.get_trj_nav_in().get_op_end(0),          1e-10);

    check("03 nav_out nel         ", 1 + ((- t_sec_init + t_sec_end) / Deltat_sec_sens),  Omotion.get_trj_nav_out().get_nel(),             1e-10);
    check("03 nav_out nel_op      ", nel_op,                                              Omotion.get_trj_nav_out().get_nel_op(),          1e-10);
    check("03 nav_out last time   ", t_sec_end,                                           Omotion.get_trj_nav_out()().back().get_t_sec(),  1e-10);
    check("03 nav_out op 0 start  ", 0,                                                   Omotion.get_trj_nav_out().get_op_start(0),       1e-10);
    check("03 nav_out op 0 end    ", (- t_sec_init + t_sec_end) / Deltat_sec_sens,        Omotion.get_trj_nav_out().get_op_end(0),         1e-10);

    check("03 gps_out nel         ", 1 + ((- t_sec_init + t_sec_end) / Deltat_sec_gps),   Omotion.get_trj_gps_out().get_nel(),             1e-10);
    check("03 gps_out nel_op      ", nel_op,                                              Omotion.get_trj_gps_out().get_nel_op(),          1e-10);
    check("03 gps_out last time   ", t_sec_end,                                           Omotion.get_trj_gps_out()().back().get_t_sec(),  1e-10);
    check("03 gps_out op 0 start  ", 0,                                                   Omotion.get_trj_gps_out().get_op_start(0),       1e-10);
    check("03 gps_out op 0 end    ", (- t_sec_init + t_sec_end) / Deltat_sec_gps,         Omotion.get_trj_gps_out().get_op_end(0),         1e-10);

    check("03 out nel             ", 1 + ((- t_sec_init + t_sec_end) / Deltat_sec_out),   Omotion.get_trj_out().get_nel(),                 1e-10);
    check("03 out nel_op          ", nel_op,                                              Omotion.get_trj_out().get_nel_op(),              1e-10);
    check("03 out last time       ", t_sec_end,                                           Omotion.get_trj_out()().back().get_t_sec(),      1e-10);
    check("03 out op 0 start      ", 0,                                                   Omotion.get_trj_out().get_op_start(0),           1e-10);
    check("03 out op 0 end        ", (- t_sec_init + t_sec_end) / Deltat_sec_out,         Omotion.get_trj_out().get_op_end(0),             1e-10);

    check("03 filter air          ", 1 + (- t_sec_init  + t_sec_end) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_air().get_size());
    check("03 filter att          ", 1 + (- t_sec_init  + t_sec_end) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_att().get_size());
    check("03 filter gps fast     ", 1 + (- t_sec_init  + t_sec_end) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_gps().get_size_fast());
    check("03 filter gps slow     ", 1 + (- t_sec_init  + t_sec_end) / Deltat_sec_gps,    Omotion.get_filter_nav().get_filter_gps().get_size_slow());
    check("03 filter pos          ", 1 + (- t_sec_end + t_sec_end) / Deltat_sec_sens,     Omotion.get_filter_nav().get_filter_pos().get_size());
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void nav::test::Ttrj_sizes::test_sizes04() {
    unsigned short seed_order = 1;
    double turb_factor        = 1.0;

    double t_sec_init    = -5.;
    double t_sec_end_trj = 20.0;
    double t_sec_end_op0 = 10.0;
    double t_sec_end_op1 = 20.0;
    double t_sec_gpsloss = 5.0;
    int    t_sec_turb    = 1000;

    sens::logic::SENS_COMPLETE_ID sens_complete_id = sens::logic::sens_complete_bias_scale_par;
    nav::logic::NAV_ID nav_id                      = nav::logic::nav_id010202;

    env::logic::WIND_ID wind_id         = env::logic::wind_id00;
    env::logic::OFFSETS_ID offsets_id   = env::logic::offsets_id00;
    env::logic::MAG_ID env_mag_id       = env::logic::mag_default;
    env::logic::REALISM_ID realism_id   = env::logic::realism_grav_no_magn_no;

    nav::logic::INITEUL_ID initeul_id   = nav::logic::initeul_id_zero;
    nav::logic::INITACC_ID initacc_id   = nav::logic::initacc_id_zero;
    nav::logic::INITGYR_ID initgyr_id   = nav::logic::initgyr_id_zero;
    nav::logic::INITMAG_ID initmag_id   = nav::logic::initmag_id_zero;
    nav::logic::INITMGN_ID initmgn_id   = nav::logic::initmgn_id_zero;

    sens::logic::GYR_ID gyr_id          = sens::logic::gyr_id_base;
    sens::logic::ACC_ID acc_id          = sens::logic::acc_id_base;
    sens::logic::MAG_ID mag_id          = sens::logic::mag_id_base;
    sens::logic::OSP_ID osp_id          = sens::logic::osp_id_base;
    sens::logic::OAT_ID oat_id          = sens::logic::oat_id_base;
    sens::logic::TAS_ID tas_id          = sens::logic::tas_id_base;
    sens::logic::AOA_ID aoa_id          = sens::logic::aoa_id_base;
    sens::logic::AOS_ID aos_id          = sens::logic::aos_id_base;
    sens::logic::GPS_ID gps_id          = sens::logic::gps_id_base;
    sens::logic::BAND_ID band_id        = sens::logic::band_id_base;

    st::logic::STI_ID sti_id = st::logic::sti_Hp3000_tas30_chi00;
    unsigned short nel_op = 2;

    math::seeder Oseeder(seed_order);
    auto Paero                = new acft::aero0();
    auto Pprop                = new acft::prop();
    auto Piner                = new acft::iner();
    auto Pearth               = new env::earth(Oseeder, wind_id, offsets_id, env_mag_id, realism_id, turb_factor, t_sec_init, t_sec_turb);
    auto Psuite               = new sens::suite(Oseeder, acc_id, gyr_id, mag_id, osp_id, oat_id, tas_id, aoa_id, aos_id, gps_id, band_id);
    auto Perr0                = new nav::init_error(Oseeder, initeul_id, initacc_id, initgyr_id, initmag_id, initmgn_id);
    auto Pguid                = new control::guid(nel_op, t_sec_end_trj, Oseeder.provide_seed(math::seeder::seeder_guid),0);
    Pguid->get_sti_id() = sti_id;
    Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 0.0, rud_beta_deg, 0., trgg_t_sec, t_sec_end_op0));
    Pguid->add_op(1, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 0.0, rud_beta_deg, 0., trgg_t_sec, t_sec_end_op1));
    st::sti* Psti             = st::sti::create_sti(*Pguid, t_sec_init);
    auto Pcntr                = new control::cntr(*Pguid);
    auto             Pnav     = new nav::filter_nav(nav_id, *Psuite, *Pearth, turb_factor);
    ang::euler euler_bc(0.0, 0.0, 0.0);
    auto Pcam                 = new sens::camera(768, 1024, 49.2255, euler_bc);

    nav::motion Omotion(Psti, Pguid, Pcntr, Pnav, Pearth, Psuite, Perr0, Piner, Paero, Pprop, Pcam, sens_complete_id);
    Omotion.solve_2nd(t_sec_end_trj, t_sec_gpsloss);

    double Deltat_sec_truth = 0.002;
    double Deltat_sec_sens  = 0.01;
    double Deltat_sec_gps   = 1.0;
    double Deltat_sec_cntr  = 0.02;
    double Deltat_sec_out   = 0.1;

    check("04 truth nel           ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_truth), Omotion.get_trj_truth().get_nel(), 1e-10);
    check("04 truth nel_op        ", nel_op,                                                  Omotion.get_trj_truth().get_nel_op(),  1e-10);
    check("04 truth last time     ", t_sec_end_op1,                                           Omotion.get_trj_truth()().back().get_t_sec(), 1e-10);
    check("04 truth op 0 start    ", 0,                                                       Omotion.get_trj_truth().get_op_start(0), 1e-10);
    check("04 truth op 0 end      ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_truth,       Omotion.get_trj_truth().get_op_end(0), 1e-10);
    check("04 truth op 1 start    ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_truth,   Omotion.get_trj_truth().get_op_start(1), 1e-10);
    check("04 truth op 1 end      ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_truth,       Omotion.get_trj_truth().get_op_end(1), 1e-10);

    check("04 sens_in nel         ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_sens),  Omotion.get_trj_sens_in().get_nel(), 1e-10);
    check("04 sens_in nel_op      ", nel_op,                                                  Omotion.get_trj_sens_in().get_nel_op(), 1e-10);
    check("04 sens_in last time   ", t_sec_end_op1,                                           Omotion.get_trj_sens_in()().back().get_t_sec(), 1e-10);
    check("04 sens_in op 0 start  ", 0,                                                       Omotion.get_trj_sens_in().get_op_start(0), 1e-10);
    check("04 sens_in op 0 end    ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_sens_in().get_op_end(0), 1e-10);
    check("04 sens_in op 1 start  ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,    Omotion.get_trj_sens_in().get_op_start(1), 1e-10);
    check("04 sens_in op 1 end    ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_sens,        Omotion.get_trj_sens_in().get_op_end(1), 1e-10);

    check("04 sens_out nel        ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_sens),  Omotion.get_trj_sens_out().get_nel(), 1e-10);
    check("04 sens_out nel_op     ", nel_op,                                                  Omotion.get_trj_sens_out().get_nel_op(), 1e-10);
    check("04 sens_out last time  ", t_sec_end_op1,                                           Omotion.get_trj_sens_out()().back().get_t_sec(), 1e-10);
    check("04 sens_out op 0 start ", 0,                                                       Omotion.get_trj_sens_out().get_op_start(0), 1e-10);
    check("04 sens_out op 0 end   ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_sens_out().get_op_end(0), 1e-10);
    check("04 sens_out op 1 start ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,    Omotion.get_trj_sens_out().get_op_start(1), 1e-10);
    check("04 sens_out op 1 end   ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_sens,        Omotion.get_trj_sens_out().get_op_end(1), 1e-10);

    check("04 cntr nel            ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_cntr),  Omotion.get_trj_cntr().get_nel(), 1e-10);
    check("04 cntr nel_op         ", nel_op,                                                  Omotion.get_trj_cntr().get_nel_op(), 1e-10);
    check("04 cntr last time      ", t_sec_end_op1,                                           Omotion.get_trj_cntr()().back().get_t_sec(), 1e-10);
    check("04 cntr op 0 start     ", 0,                                                       Omotion.get_trj_cntr().get_op_start(0), 1e-10);
    check("04 cntr op 0 end       ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_cntr,        Omotion.get_trj_cntr().get_op_end(0), 1e-10);
    check("04 cntr op 1 start     ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_cntr,    Omotion.get_trj_cntr().get_op_start(1), 1e-10);
    check("04 cntr op 1 end       ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_cntr,        Omotion.get_trj_cntr().get_op_end(1), 1e-10);

    check("04 nav_in nel          ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_sens),  Omotion.get_trj_nav_in().get_nel(), 1e-10);
    check("04 nav_in nel_op       ", nel_op,                                                  Omotion.get_trj_nav_in().get_nel_op(), 1e-10);
    check("04 nav_in last time    ", t_sec_end_op1,                                           Omotion.get_trj_nav_in()().back().get_t_sec(), 1e-10);
    check("04 nav_in op 0 start   ", 0,                                                       Omotion.get_trj_nav_in().get_op_start(0), 1e-10);
    check("04 nav_in op 0 end     ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_nav_in().get_op_end(0), 1e-10);
    check("04 nav_in op 1 start   ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,    Omotion.get_trj_nav_in().get_op_start(1), 1e-10);
    check("04 nav_in op 1 end     ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_sens,        Omotion.get_trj_nav_in().get_op_end(1), 1e-10);

    check("04 nav_out nel         ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_sens),  Omotion.get_trj_nav_out().get_nel(), 1e-10);
    check("04 nav_out nel_op      ", nel_op,                                                  Omotion.get_trj_nav_out().get_nel_op(), 1e-10);
    check("04 nav_out last time   ", t_sec_end_op1,                                           Omotion.get_trj_nav_out()().back().get_t_sec(), 1e-10);
    check("04 nav_out op 0 start  ", 0,                                                       Omotion.get_trj_nav_out().get_op_start(0), 1e-10);
    check("04 nav_out op 0 end    ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_nav_out().get_op_end(0), 1e-10);
    check("04 nav_out op 1 start  ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,    Omotion.get_trj_nav_out().get_op_start(1), 1e-10);
    check("04 nav_out op 1 end    ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_sens,        Omotion.get_trj_nav_out().get_op_end(1), 1e-10);

    check("04 gps_out nel         ", 1 + ((- t_sec_init + t_sec_gpsloss) / Deltat_sec_gps),   Omotion.get_trj_gps_out().get_nel(), 1e-10);
    check("04 gps_out nel_op      ", nel_op - 1,                                              Omotion.get_trj_gps_out().get_nel_op(), 1e-10);
    check("04 gps_out last time   ", t_sec_gpsloss,                                           Omotion.get_trj_gps_out()().back().get_t_sec(), 1e-10);
    check("04 gps_out op 0 start  ", 0,                                                       Omotion.get_trj_gps_out().get_op_start(0), 1e-10);
    check("04 gps_out op 0 end    ", (- t_sec_init + t_sec_gpsloss) / Deltat_sec_gps,         Omotion.get_trj_gps_out().get_op_end(0), 1e-10);

    check("04 out nel             ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_out),   Omotion.get_trj_out().get_nel(), 1e-10);
    check("04 out nel_op          ", nel_op,                                                  Omotion.get_trj_out().get_nel_op(), 1e-10);
    check("04 out last time       ", t_sec_end_op1,                                           Omotion.get_trj_out()().back().get_t_sec(), 1e-10);
    check("04 out op 0 start      ", 0,                                                       Omotion.get_trj_out().get_op_start(0), 1e-10);
    check("04 out op 0 end        ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_out,         Omotion.get_trj_out().get_op_end(0), 1e-10);
    check("04 out op 1 start      ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_out,     Omotion.get_trj_out().get_op_start(1), 1e-10);
    check("04 out op 1 end        ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_out,         Omotion.get_trj_out().get_op_end(1), 1e-10);

    check("04 filter air          ", 1 + (- t_sec_init  + t_sec_end_trj) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_air().get_size());
    check("04 filter att          ", 1 + (- t_sec_init  + t_sec_end_trj) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_att().get_size());
    check("04 filter gps fast     ", 1 + (- t_sec_init  + t_sec_gpsloss) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_gps().get_size_fast());
    check("04 filter gps slow     ", 1 + (- t_sec_init  + t_sec_gpsloss) / Deltat_sec_gps,    Omotion.get_filter_nav().get_filter_gps().get_size_slow());
    check("04 filter pos          ", 1 + (- t_sec_gpsloss + t_sec_end_trj) / Deltat_sec_sens, Omotion.get_filter_nav().get_filter_pos().get_size());
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void nav::test::Ttrj_sizes::test_sizes05() {
    unsigned short seed_order = 1;
    double turb_factor        = 1.0;

    double t_sec_init    = -5.;
    double t_sec_end_trj = 20.0;
    double t_sec_end_op0 = 10.0;
    double t_sec_end_op1 = 20.0;
    double t_sec_gpsloss = 10.0;
    int    t_sec_turb    = 1000;

    sens::logic::SENS_COMPLETE_ID sens_complete_id = sens::logic::sens_complete_bias_scale_par;
    nav::logic::NAV_ID nav_id                      = nav::logic::nav_id010202;

    env::logic::WIND_ID wind_id         = env::logic::wind_id00;
    env::logic::OFFSETS_ID offsets_id   = env::logic::offsets_id00;
    env::logic::MAG_ID env_mag_id       = env::logic::mag_default;
    env::logic::REALISM_ID realism_id   = env::logic::realism_grav_no_magn_no;

    nav::logic::INITEUL_ID initeul_id   = nav::logic::initeul_id_zero;
    nav::logic::INITACC_ID initacc_id   = nav::logic::initacc_id_zero;
    nav::logic::INITGYR_ID initgyr_id   = nav::logic::initgyr_id_zero;
    nav::logic::INITMAG_ID initmag_id   = nav::logic::initmag_id_zero;
    nav::logic::INITMGN_ID initmgn_id   = nav::logic::initmgn_id_zero;

    sens::logic::GYR_ID gyr_id          = sens::logic::gyr_id_base;
    sens::logic::ACC_ID acc_id          = sens::logic::acc_id_base;
    sens::logic::MAG_ID mag_id          = sens::logic::mag_id_base;
    sens::logic::OSP_ID osp_id          = sens::logic::osp_id_base;
    sens::logic::OAT_ID oat_id          = sens::logic::oat_id_base;
    sens::logic::TAS_ID tas_id          = sens::logic::tas_id_base;
    sens::logic::AOA_ID aoa_id          = sens::logic::aoa_id_base;
    sens::logic::AOS_ID aos_id          = sens::logic::aos_id_base;
    sens::logic::GPS_ID gps_id          = sens::logic::gps_id_base;
    sens::logic::BAND_ID band_id        = sens::logic::band_id_base;

    st::logic::STI_ID sti_id = st::logic::sti_Hp3000_tas30_chi00;
    unsigned short nel_op = 2;

    math::seeder Oseeder(seed_order);
    auto Paero                = new acft::aero0();
    auto Pprop                = new acft::prop();
    auto Piner                = new acft::iner();
    auto Pearth               = new env::earth(Oseeder, wind_id, offsets_id, env_mag_id, realism_id, turb_factor, t_sec_init, t_sec_turb);
    auto Psuite               = new sens::suite(Oseeder, acc_id, gyr_id, mag_id, osp_id, oat_id, tas_id, aoa_id, aos_id, gps_id, band_id);
    auto Perr0                = new nav::init_error(Oseeder, initeul_id, initacc_id, initgyr_id, initmag_id, initmgn_id);
    auto Pguid                = new control::guid(nel_op, t_sec_end_trj, Oseeder.provide_seed(math::seeder::seeder_guid),0);
    Pguid->get_sti_id() = sti_id;
    Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 0.0, rud_beta_deg, 0., trgg_t_sec, t_sec_end_op0));
    Pguid->add_op(1, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 0.0, rud_beta_deg, 0., trgg_t_sec, t_sec_end_op1));
    st::sti* Psti             = st::sti::create_sti(*Pguid, t_sec_init);
    auto Pcntr                = new control::cntr(*Pguid);
    auto             Pnav     = new nav::filter_nav(nav_id, *Psuite, *Pearth, turb_factor);
    ang::euler euler_bc(0.0, 0.0, 0.0);
    auto Pcam                 = new sens::camera(768, 1024, 49.2255, euler_bc);

    nav::motion Omotion(Psti, Pguid, Pcntr, Pnav, Pearth, Psuite, Perr0, Piner, Paero, Pprop, Pcam, sens_complete_id);
    Omotion.solve_2nd(t_sec_end_trj, t_sec_gpsloss);

    double Deltat_sec_truth = 0.002;
    double Deltat_sec_sens  = 0.01;
    double Deltat_sec_gps   = 1.0;
    double Deltat_sec_cntr  = 0.02;
    double Deltat_sec_out   = 0.1;

    check("05 truth nel           ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_truth), Omotion.get_trj_truth().get_nel(),              1e-10);
    check("05 truth nel_op        ", nel_op,                                                  Omotion.get_trj_truth().get_nel_op(),           1e-10);
    check("05 truth last time     ", t_sec_end_op1,                                           Omotion.get_trj_truth()().back().get_t_sec(),   1e-10);
    check("05 truth op 0 start    ", 0,                                                       Omotion.get_trj_truth().get_op_start(0),        1e-10);
    check("05 truth op 0 end      ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_truth,       Omotion.get_trj_truth().get_op_end(0),          1e-10);
    check("05 truth op 1 start    ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_truth,   Omotion.get_trj_truth().get_op_start(1),        1e-10);
    check("05 truth op 1 end      ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_truth,       Omotion.get_trj_truth().get_op_end(1),          1e-10);

    check("05 sens_in nel         ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_sens),  Omotion.get_trj_sens_in().get_nel(),            1e-10);
    check("05 sens_in nel_op      ", nel_op,                                                  Omotion.get_trj_sens_in().get_nel_op(),         1e-10);
    check("05 sens_in last time   ", t_sec_end_op1,                                           Omotion.get_trj_sens_in()().back().get_t_sec(), 1e-10);
    check("05 sens_in op 0 start  ", 0,                                                       Omotion.get_trj_sens_in().get_op_start(0),      1e-10);
    check("05 sens_in op 0 end    ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_sens_in().get_op_end(0),        1e-10);
    check("05 sens_in op 1 start  ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,    Omotion.get_trj_sens_in().get_op_start(1),      1e-10);
    check("05 sens_in op 1 end    ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_sens,        Omotion.get_trj_sens_in().get_op_end(1),        1e-10);

    check("05 sens_out nel        ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_sens),  Omotion.get_trj_sens_out().get_nel(),           1e-10);
    check("05 sens_out nel_op     ", nel_op,                                                  Omotion.get_trj_sens_out().get_nel_op(),        1e-10);
    check("05 sens_out last time  ", t_sec_end_op1,                                           Omotion.get_trj_sens_out()().back().get_t_sec(),1e-10);
    check("05 sens_out op 0 start ", 0,                                                       Omotion.get_trj_sens_out().get_op_start(0),     1e-10);
    check("05 sens_out op 0 end   ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_sens_out().get_op_end(0),       1e-10);
    check("05 sens_out op 1 start ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,    Omotion.get_trj_sens_out().get_op_start(1),     1e-10);
    check("05 sens_out op 1 end   ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_sens,        Omotion.get_trj_sens_out().get_op_end(1),       1e-10);

    check("05 cntr nel            ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_cntr),  Omotion.get_trj_cntr().get_nel(),               1e-10);
    check("05 cntr nel_op         ", nel_op,                                                  Omotion.get_trj_cntr().get_nel_op(),            1e-10);
    check("05 cntr last time      ", t_sec_end_op1,                                           Omotion.get_trj_cntr()().back().get_t_sec(),    1e-10);
    check("05 cntr op 0 start     ", 0,                                                       Omotion.get_trj_cntr().get_op_start(0),         1e-10);
    check("05 cntr op 0 end       ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_cntr,        Omotion.get_trj_cntr().get_op_end(0),           1e-10);
    check("05 cntr op 1 start     ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_cntr,    Omotion.get_trj_cntr().get_op_start(1),         1e-10);
    check("05 cntr op 1 end       ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_cntr,        Omotion.get_trj_cntr().get_op_end(1),           1e-10);

    check("05 nav_in nel          ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_sens),  Omotion.get_trj_nav_in().get_nel(),             1e-10);
    check("05 nav_in nel_op       ", nel_op,                                                  Omotion.get_trj_nav_in().get_nel_op(),          1e-10);
    check("05 nav_in last time    ", t_sec_end_op1,                                           Omotion.get_trj_nav_in()().back().get_t_sec(),  1e-10);
    check("05 nav_in op 0 start   ", 0,                                                       Omotion.get_trj_nav_in().get_op_start(0),       1e-10);
    check("05 nav_in op 0 end     ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_nav_in().get_op_end(0),         1e-10);
    check("05 nav_in op 1 start   ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,    Omotion.get_trj_nav_in().get_op_start(1),       1e-10);
    check("05 nav_in op 1 end     ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_sens,        Omotion.get_trj_nav_in().get_op_end(1),         1e-10);

    check("05 nav_out nel         ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_sens),  Omotion.get_trj_nav_out().get_nel(),            1e-10);
    check("05 nav_out nel_op      ", nel_op,                                                  Omotion.get_trj_nav_out().get_nel_op(),         1e-10);
    check("05 nav_out last time   ", t_sec_end_op1,                                           Omotion.get_trj_nav_out()().back().get_t_sec(), 1e-10);
    check("05 nav_out op 0 start  ", 0,                                                       Omotion.get_trj_nav_out().get_op_start(0),      1e-10);
    check("05 nav_out op 0 end    ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_nav_out().get_op_end(0),        1e-10);
    check("05 nav_out op 1 start  ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,    Omotion.get_trj_nav_out().get_op_start(1),      1e-10);
    check("05 nav_out op 1 end    ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_sens,        Omotion.get_trj_nav_out().get_op_end(1),        1e-10);

    check("05 gps_out nel         ", 1 + ((- t_sec_init + t_sec_gpsloss) / Deltat_sec_gps),   Omotion.get_trj_gps_out().get_nel(),            1e-10);
    check("05 gps_out nel_op      ", nel_op - 1,                                              Omotion.get_trj_gps_out().get_nel_op(),         1e-10);
    check("05 gps_out last time   ", t_sec_gpsloss,                                           Omotion.get_trj_gps_out()().back().get_t_sec(), 1e-10);
    check("05 gps_out op 0 start  ", 0,                                                       Omotion.get_trj_gps_out().get_op_start(0),      1e-10);
    check("05 gps_out op 0 end    ", (- t_sec_init + t_sec_gpsloss) / Deltat_sec_gps,         Omotion.get_trj_gps_out().get_op_end(0),        1e-10);

    check("05 out nel             ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_out),   Omotion.get_trj_out().get_nel(),                1e-10);
    check("05 out nel_op          ", nel_op,                                                  Omotion.get_trj_out().get_nel_op(),             1e-10);
    check("05 out last time       ", t_sec_end_op1,                                           Omotion.get_trj_out()().back().get_t_sec(),     1e-10);
    check("05 out op 0 start      ", 0,                                                       Omotion.get_trj_out().get_op_start(0),          1e-10);
    check("05 out op 0 end        ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_out,         Omotion.get_trj_out().get_op_end(0),            1e-10);
    check("05 out op 1 start      ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_out,     Omotion.get_trj_out().get_op_start(1),          1e-10);
    check("05 out op 1 end        ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_out,         Omotion.get_trj_out().get_op_end(1),            1e-10);

    check("05 filter air          ", 1 + (- t_sec_init  + t_sec_end_trj) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_air().get_size());
    check("05 filter att          ", 1 + (- t_sec_init  + t_sec_end_trj) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_att().get_size());
    check("05 filter gps fast     ", 1 + (- t_sec_init  + t_sec_gpsloss) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_gps().get_size_fast());
    check("05 filter gps slow     ", 1 + (- t_sec_init  + t_sec_gpsloss) / Deltat_sec_gps,    Omotion.get_filter_nav().get_filter_gps().get_size_slow());
    check("05 filter pos          ", 1 + (- t_sec_gpsloss + t_sec_end_trj) / Deltat_sec_sens, Omotion.get_filter_nav().get_filter_pos().get_size());
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void nav::test::Ttrj_sizes::test_sizes06() {
    unsigned short seed_order = 1;
    double turb_factor        = 1.0;

    double t_sec_init    = -5.;
    double t_sec_end_trj = 20.0;
    double t_sec_end_op0 = 10.0;
    double t_sec_end_op1 = 20.0;
    double t_sec_gpsloss = 15.0;
    int    t_sec_turb    = 1000;

    sens::logic::SENS_COMPLETE_ID sens_complete_id = sens::logic::sens_complete_bias_scale_par;
    nav::logic::NAV_ID nav_id                      = nav::logic::nav_id010202;

    env::logic::WIND_ID wind_id         = env::logic::wind_id00;
    env::logic::OFFSETS_ID offsets_id   = env::logic::offsets_id00;
    env::logic::MAG_ID env_mag_id       = env::logic::mag_default;
    env::logic::REALISM_ID realism_id   = env::logic::realism_grav_no_magn_no;

    nav::logic::INITEUL_ID initeul_id   = nav::logic::initeul_id_zero;
    nav::logic::INITACC_ID initacc_id   = nav::logic::initacc_id_zero;
    nav::logic::INITGYR_ID initgyr_id   = nav::logic::initgyr_id_zero;
    nav::logic::INITMAG_ID initmag_id   = nav::logic::initmag_id_zero;
    nav::logic::INITMGN_ID initmgn_id   = nav::logic::initmgn_id_zero;

    sens::logic::GYR_ID gyr_id          = sens::logic::gyr_id_base;
    sens::logic::ACC_ID acc_id          = sens::logic::acc_id_base;
    sens::logic::MAG_ID mag_id          = sens::logic::mag_id_base;
    sens::logic::OSP_ID osp_id          = sens::logic::osp_id_base;
    sens::logic::OAT_ID oat_id          = sens::logic::oat_id_base;
    sens::logic::TAS_ID tas_id          = sens::logic::tas_id_base;
    sens::logic::AOA_ID aoa_id          = sens::logic::aoa_id_base;
    sens::logic::AOS_ID aos_id          = sens::logic::aos_id_base;
    sens::logic::GPS_ID gps_id          = sens::logic::gps_id_base;
    sens::logic::BAND_ID band_id        = sens::logic::band_id_base;

    st::logic::STI_ID sti_id = st::logic::sti_Hp3000_tas30_chi00;
    unsigned short nel_op = 2;

    math::seeder Oseeder(seed_order);
    auto Paero                = new acft::aero0();
    auto Pprop                = new acft::prop();
    auto Piner                = new acft::iner();
    auto Pearth               = new env::earth(Oseeder, wind_id, offsets_id, env_mag_id, realism_id, turb_factor, t_sec_init, t_sec_turb);
    auto Psuite               = new sens::suite(Oseeder, acc_id, gyr_id, mag_id, osp_id, oat_id, tas_id, aoa_id, aos_id, gps_id, band_id);
    auto Perr0                = new nav::init_error(Oseeder, initeul_id, initacc_id, initgyr_id, initmag_id, initmgn_id);
    auto Pguid                = new control::guid(nel_op, t_sec_end_trj, Oseeder.provide_seed(math::seeder::seeder_guid),0);
    Pguid->get_sti_id() = sti_id;
    Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 0.0, rud_beta_deg, 0., trgg_t_sec, t_sec_end_op0));
    Pguid->add_op(1, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 0.0, rud_beta_deg, 0., trgg_t_sec, t_sec_end_op1));
    st::sti* Psti             = st::sti::create_sti(*Pguid, t_sec_init);
    auto Pcntr                = new control::cntr(*Pguid);
    auto             Pnav     = new nav::filter_nav(nav_id, *Psuite, *Pearth, turb_factor);
    ang::euler euler_bc(0.0, 0.0, 0.0);
    auto Pcam                 = new sens::camera(768, 1024, 49.2255, euler_bc);

    nav::motion Omotion(Psti, Pguid, Pcntr, Pnav, Pearth, Psuite, Perr0, Piner, Paero, Pprop, Pcam, sens_complete_id);
    Omotion.solve_2nd(t_sec_end_trj, t_sec_gpsloss);

    double Deltat_sec_truth = 0.002;
    double Deltat_sec_sens  = 0.01;
    double Deltat_sec_gps   = 1.0;
    double Deltat_sec_cntr  = 0.02;
    double Deltat_sec_out   = 0.1;

    check("06 truth nel           ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_truth), Omotion.get_trj_truth().get_nel(),              1e-10);
    check("06 truth nel_op        ", nel_op,                                                  Omotion.get_trj_truth().get_nel_op(),           1e-10);
    check("06 truth last time     ", t_sec_end_op1,                                           Omotion.get_trj_truth()().back().get_t_sec(),   1e-10);
    check("06 truth op 0 start    ", 0,                                                       Omotion.get_trj_truth().get_op_start(0),        1e-10);
    check("06 truth op 0 end      ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_truth,       Omotion.get_trj_truth().get_op_end(0),          1e-10);
    check("06 truth op 1 start    ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_truth,   Omotion.get_trj_truth().get_op_start(1),        1e-10);
    check("06 truth op 1 end      ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_truth,       Omotion.get_trj_truth().get_op_end(1),          1e-10);

    check("06 sens_in nel         ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_sens),  Omotion.get_trj_sens_in().get_nel(),            1e-10);
    check("06 sens_in nel_op      ", nel_op,                                                  Omotion.get_trj_sens_in().get_nel_op(),         1e-10);
    check("06 sens_in last time   ", t_sec_end_op1,                                           Omotion.get_trj_sens_in()().back().get_t_sec(), 1e-10);
    check("06 sens_in op 0 start  ", 0,                                                       Omotion.get_trj_sens_in().get_op_start(0),      1e-10);
    check("06 sens_in op 0 end    ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_sens_in().get_op_end(0),        1e-10);
    check("06 sens_in op 1 start  ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,    Omotion.get_trj_sens_in().get_op_start(1),      1e-10);
    check("06 sens_in op 1 end    ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_sens,        Omotion.get_trj_sens_in().get_op_end(1),        1e-10);

    check("06 sens_out nel        ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_sens),  Omotion.get_trj_sens_out().get_nel(),           1e-10);
    check("06 sens_out nel_op     ", nel_op,                                                  Omotion.get_trj_sens_out().get_nel_op(),        1e-10);
    check("06 sens_out last time  ", t_sec_end_op1,                                           Omotion.get_trj_sens_out()().back().get_t_sec(),1e-10);
    check("06 sens_out op 0 start ", 0,                                                       Omotion.get_trj_sens_out().get_op_start(0),     1e-10);
    check("06 sens_out op 0 end   ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_sens_out().get_op_end(0),       1e-10);
    check("06 sens_out op 1 start ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,    Omotion.get_trj_sens_out().get_op_start(1),     1e-10);
    check("06 sens_out op 1 end   ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_sens,        Omotion.get_trj_sens_out().get_op_end(1),       1e-10);

    check("06 cntr nel            ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_cntr),  Omotion.get_trj_cntr().get_nel(),               1e-10);
    check("06 cntr nel_op         ", nel_op,                                                  Omotion.get_trj_cntr().get_nel_op(),            1e-10);
    check("06 cntr last time      ", t_sec_end_op1,                                           Omotion.get_trj_cntr()().back().get_t_sec(),    1e-10);
    check("06 cntr op 0 start     ", 0,                                                       Omotion.get_trj_cntr().get_op_start(0),         1e-10);
    check("06 cntr op 0 end       ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_cntr,        Omotion.get_trj_cntr().get_op_end(0),           1e-10);
    check("06 cntr op 1 start     ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_cntr,    Omotion.get_trj_cntr().get_op_start(1),         1e-10);
    check("06 cntr op 1 end       ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_cntr,        Omotion.get_trj_cntr().get_op_end(1),           1e-10);

    check("06 nav_in nel          ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_sens),  Omotion.get_trj_nav_in().get_nel(),             1e-10);
    check("06 nav_in nel_op       ", nel_op,                                                  Omotion.get_trj_nav_in().get_nel_op(),          1e-10);
    check("06 nav_in last time    ", t_sec_end_op1,                                           Omotion.get_trj_nav_in()().back().get_t_sec(),  1e-10);
    check("06 nav_in op 0 start   ", 0,                                                       Omotion.get_trj_nav_in().get_op_start(0),       1e-10);
    check("06 nav_in op 0 end     ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_nav_in().get_op_end(0),         1e-10);
    check("06 nav_in op 1 start   ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,    Omotion.get_trj_nav_in().get_op_start(1),       1e-10);
    check("06 nav_in op 1 end     ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_sens,        Omotion.get_trj_nav_in().get_op_end(1),         1e-10);

    check("06 nav_out nel         ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_sens),  Omotion.get_trj_nav_out().get_nel(),            1e-10);
    check("06 nav_out nel_op      ", nel_op,                                                  Omotion.get_trj_nav_out().get_nel_op(),         1e-10);
    check("06 nav_out last time   ", t_sec_end_op1,                                           Omotion.get_trj_nav_out()().back().get_t_sec(), 1e-10);
    check("06 nav_out op 0 start  ", 0,                                                       Omotion.get_trj_nav_out().get_op_start(0),      1e-10);
    check("06 nav_out op 0 end    ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_nav_out().get_op_end(0),        1e-10);
    check("06 nav_out op 1 start  ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,    Omotion.get_trj_nav_out().get_op_start(1),      1e-10);
    check("06 nav_out op 1 end    ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_sens,        Omotion.get_trj_nav_out().get_op_end(1),        1e-10);

    check("06 gps_out nel         ", 1 + ((- t_sec_init + t_sec_gpsloss) / Deltat_sec_gps),   Omotion.get_trj_gps_out().get_nel(),            1e-10);
    check("06 gps_out nel_op      ", nel_op,                                                  Omotion.get_trj_gps_out().get_nel_op(),         1e-10);
    check("06 gps_out last time   ", t_sec_gpsloss,                                           Omotion.get_trj_gps_out()().back().get_t_sec(), 1e-10);
    check("06 gps_out op 0 start  ", 0,                                                       Omotion.get_trj_gps_out().get_op_start(0),      1e-10);
    check("06 gps_out op 0 end    ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_gps,         Omotion.get_trj_gps_out().get_op_end(0),        1e-10);
    check("06 gps_out op 1 start  ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_gps,     Omotion.get_trj_gps_out().get_op_start(1),      1e-10);
    check("06 gps_out op 1 end    ", (- t_sec_init + t_sec_gpsloss) / Deltat_sec_gps,         Omotion.get_trj_gps_out().get_op_end(1),        1e-10);

    check("06 out nel             ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_out),   Omotion.get_trj_out().get_nel(),                1e-10);
    check("06 out nel_op          ", nel_op,                                                  Omotion.get_trj_out().get_nel_op(),             1e-10);
    check("06 out last time       ", t_sec_end_op1,                                           Omotion.get_trj_out()().back().get_t_sec(),     1e-10);
    check("06 out op 0 start      ", 0,                                                       Omotion.get_trj_out().get_op_start(0),          1e-10);
    check("06 out op 0 end        ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_out,         Omotion.get_trj_out().get_op_end(0),            1e-10);
    check("06 out op 1 start      ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_out,     Omotion.get_trj_out().get_op_start(1),          1e-10);
    check("06 out op 1 end        ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_out,         Omotion.get_trj_out().get_op_end(1),            1e-10);

    check("06 filter air          ", 1 + (- t_sec_init  + t_sec_end_trj) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_air().get_size());
    check("06 filter att          ", 1 + (- t_sec_init  + t_sec_end_trj) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_att().get_size());
    check("06 filter gps fast     ", 1 + (- t_sec_init  + t_sec_gpsloss) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_gps().get_size_fast());
    check("06 filter gps slow     ", 1 + (- t_sec_init  + t_sec_gpsloss) / Deltat_sec_gps,    Omotion.get_filter_nav().get_filter_gps().get_size_slow());
    check("06 filter pos          ", 1 + (- t_sec_gpsloss + t_sec_end_trj) / Deltat_sec_sens, Omotion.get_filter_nav().get_filter_pos().get_size());
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void nav::test::Ttrj_sizes::test_sizes07() {
    unsigned short seed_order = 1;
    double turb_factor        = 1.0;

    double t_sec_init    = -5.;
    double t_sec_end_trj = 20.0;
    double t_sec_end_op0 = 10.0;
    double t_sec_end_op1 = 20.0;
    double t_sec_gpsloss = 20.0;
    int    t_sec_turb    = 1000;

    sens::logic::SENS_COMPLETE_ID sens_complete_id = sens::logic::sens_complete_bias_scale_par;
    nav::logic::NAV_ID nav_id                      = nav::logic::nav_id010202;

    env::logic::WIND_ID wind_id         = env::logic::wind_id00;
    env::logic::OFFSETS_ID offsets_id   = env::logic::offsets_id00;
    env::logic::MAG_ID env_mag_id       = env::logic::mag_default;
    env::logic::REALISM_ID realism_id   = env::logic::realism_grav_no_magn_no;

    nav::logic::INITEUL_ID initeul_id   = nav::logic::initeul_id_zero;
    nav::logic::INITACC_ID initacc_id   = nav::logic::initacc_id_zero;
    nav::logic::INITGYR_ID initgyr_id   = nav::logic::initgyr_id_zero;
    nav::logic::INITMAG_ID initmag_id   = nav::logic::initmag_id_zero;
    nav::logic::INITMGN_ID initmgn_id   = nav::logic::initmgn_id_zero;

    sens::logic::GYR_ID gyr_id          = sens::logic::gyr_id_base;
    sens::logic::ACC_ID acc_id          = sens::logic::acc_id_base;
    sens::logic::MAG_ID mag_id          = sens::logic::mag_id_base;
    sens::logic::OSP_ID osp_id          = sens::logic::osp_id_base;
    sens::logic::OAT_ID oat_id          = sens::logic::oat_id_base;
    sens::logic::TAS_ID tas_id          = sens::logic::tas_id_base;
    sens::logic::AOA_ID aoa_id          = sens::logic::aoa_id_base;
    sens::logic::AOS_ID aos_id          = sens::logic::aos_id_base;
    sens::logic::GPS_ID gps_id          = sens::logic::gps_id_base;
    sens::logic::BAND_ID band_id        = sens::logic::band_id_base;

    st::logic::STI_ID sti_id = st::logic::sti_Hp3000_tas30_chi00;
    unsigned short nel_op = 2;

    math::seeder Oseeder(seed_order);
    auto Paero                = new acft::aero0();
    auto Pprop                = new acft::prop();
    auto Piner                = new acft::iner();
    auto Pearth               = new env::earth(Oseeder, wind_id, offsets_id, env_mag_id, realism_id, turb_factor, t_sec_init, t_sec_turb);
    auto Psuite               = new sens::suite(Oseeder, acc_id, gyr_id, mag_id, osp_id, oat_id, tas_id, aoa_id, aos_id, gps_id, band_id);
    auto Perr0                = new nav::init_error(Oseeder, initeul_id, initacc_id, initgyr_id, initmag_id, initmgn_id);
    auto Pguid                = new control::guid(nel_op, t_sec_end_trj, Oseeder.provide_seed(math::seeder::seeder_guid),0);
    Pguid->get_sti_id() = sti_id;
    Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 0.0, rud_beta_deg, 0., trgg_t_sec, t_sec_end_op0));
    Pguid->add_op(1, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 0.0, rud_beta_deg, 0., trgg_t_sec, t_sec_end_op1));
    st::sti* Psti             = st::sti::create_sti(*Pguid, t_sec_init);
    auto Pcntr                = new control::cntr(*Pguid);
    auto             Pnav     = new nav::filter_nav(nav_id, *Psuite, *Pearth, turb_factor);
    ang::euler euler_bc(0.0, 0.0, 0.0);
    auto Pcam                 = new sens::camera(768, 1024, 49.2255, euler_bc);

    nav::motion Omotion(Psti, Pguid, Pcntr, Pnav, Pearth, Psuite, Perr0, Piner, Paero, Pprop, Pcam, sens_complete_id);
    Omotion.solve_2nd(t_sec_end_trj, t_sec_gpsloss);

    double Deltat_sec_truth = 0.002;
    double Deltat_sec_sens  = 0.01;
    double Deltat_sec_gps   = 1.0;
    double Deltat_sec_cntr  = 0.02;
    double Deltat_sec_out   = 0.1;

    check("07 truth nel           ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_truth), Omotion.get_trj_truth().get_nel(),              1e-10);
    check("07 truth nel_op        ", nel_op,                                                  Omotion.get_trj_truth().get_nel_op(),           1e-10);
    check("07 truth last time     ", t_sec_end_op1,                                           Omotion.get_trj_truth()().back().get_t_sec(),   1e-10);
    check("07 truth op 0 start    ", 0,                                                       Omotion.get_trj_truth().get_op_start(0),        1e-10);
    check("07 truth op 0 end      ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_truth,       Omotion.get_trj_truth().get_op_end(0),          1e-10);
    check("07 truth op 1 start    ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_truth,   Omotion.get_trj_truth().get_op_start(1),        1e-10);
    check("07 truth op 1 end      ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_truth,       Omotion.get_trj_truth().get_op_end(1),          1e-10);

    check("07 sens_in nel         ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_sens),  Omotion.get_trj_sens_in().get_nel(),            1e-10);
    check("07 sens_in nel_op      ", nel_op,                                                  Omotion.get_trj_sens_in().get_nel_op(),         1e-10);
    check("07 sens_in last time   ", t_sec_end_op1,                                           Omotion.get_trj_sens_in()().back().get_t_sec(), 1e-10);
    check("07 sens_in op 0 start  ", 0,                                                       Omotion.get_trj_sens_in().get_op_start(0),      1e-10);
    check("07 sens_in op 0 end    ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_sens_in().get_op_end(0),        1e-10);
    check("07 sens_in op 1 start  ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,    Omotion.get_trj_sens_in().get_op_start(1),      1e-10);
    check("07 sens_in op 1 end    ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_sens,        Omotion.get_trj_sens_in().get_op_end(1),        1e-10);

    check("07 sens_out nel        ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_sens),  Omotion.get_trj_sens_out().get_nel(),           1e-10);
    check("07 sens_out nel_op     ", nel_op,                                                  Omotion.get_trj_sens_out().get_nel_op(),        1e-10);
    check("07 sens_out last time  ", t_sec_end_op1,                                           Omotion.get_trj_sens_out()().back().get_t_sec(),1e-10);
    check("07 sens_out op 0 start ", 0,                                                       Omotion.get_trj_sens_out().get_op_start(0),     1e-10);
    check("07 sens_out op 0 end   ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_sens_out().get_op_end(0),       1e-10);
    check("07 sens_out op 1 start ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,    Omotion.get_trj_sens_out().get_op_start(1),     1e-10);
    check("07 sens_out op 1 end   ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_sens,        Omotion.get_trj_sens_out().get_op_end(1),       1e-10);

    check("07 cntr nel            ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_cntr),  Omotion.get_trj_cntr().get_nel(),               1e-10);
    check("07 cntr nel_op         ", nel_op,                                                  Omotion.get_trj_cntr().get_nel_op(),            1e-10);
    check("07 cntr last time      ", t_sec_end_op1,                                           Omotion.get_trj_cntr()().back().get_t_sec(),    1e-10);
    check("07 cntr op 0 start     ", 0,                                                       Omotion.get_trj_cntr().get_op_start(0),         1e-10);
    check("07 cntr op 0 end       ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_cntr,        Omotion.get_trj_cntr().get_op_end(0),           1e-10);
    check("07 cntr op 1 start     ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_cntr,    Omotion.get_trj_cntr().get_op_start(1),         1e-10);
    check("07 cntr op 1 end       ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_cntr,        Omotion.get_trj_cntr().get_op_end(1),           1e-10);

    check("07 nav_in nel          ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_sens),  Omotion.get_trj_nav_in().get_nel(),             1e-10);
    check("07 nav_in nel_op       ", nel_op,                                                  Omotion.get_trj_nav_in().get_nel_op(),          1e-10);
    check("07 nav_in last time    ", t_sec_end_op1,                                           Omotion.get_trj_nav_in()().back().get_t_sec(),  1e-10);
    check("07 nav_in op 0 start   ", 0,                                                       Omotion.get_trj_nav_in().get_op_start(0),       1e-10);
    check("07 nav_in op 0 end     ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_nav_in().get_op_end(0),         1e-10);
    check("07 nav_in op 1 start   ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,    Omotion.get_trj_nav_in().get_op_start(1),       1e-10);
    check("07 nav_in op 1 end     ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_sens,        Omotion.get_trj_nav_in().get_op_end(1),         1e-10);

    check("07 nav_out nel         ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_sens),  Omotion.get_trj_nav_out().get_nel(),            1e-10);
    check("07 nav_out nel_op      ", nel_op,                                                  Omotion.get_trj_nav_out().get_nel_op(),         1e-10);
    check("07 nav_out last time   ", t_sec_end_op1,                                           Omotion.get_trj_nav_out()().back().get_t_sec(), 1e-10);
    check("07 nav_out op 0 start  ", 0,                                                       Omotion.get_trj_nav_out().get_op_start(0),      1e-10);
    check("07 nav_out op 0 end    ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_nav_out().get_op_end(0),        1e-10);
    check("07 nav_out op 1 start  ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,    Omotion.get_trj_nav_out().get_op_start(1),      1e-10);
    check("07 nav_out op 1 end    ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_sens,        Omotion.get_trj_nav_out().get_op_end(1),        1e-10);

    check("07 gps_out nel         ", 1 + ((- t_sec_init + t_sec_gpsloss) / Deltat_sec_gps),   Omotion.get_trj_gps_out().get_nel(),            1e-10);
    check("07 gps_out nel_op      ", nel_op,                                                  Omotion.get_trj_gps_out().get_nel_op(),         1e-10);
    check("07 gps_out last time   ", t_sec_gpsloss,                                           Omotion.get_trj_gps_out()().back().get_t_sec(), 1e-10);
    check("07 gps_out op 0 start  ", 0,                                                       Omotion.get_trj_gps_out().get_op_start(0),      1e-10);
    check("07 gps_out op 0 end    ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_gps,         Omotion.get_trj_gps_out().get_op_end(0),        1e-10);
    check("07 gps_out op 1 start  ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_gps,     Omotion.get_trj_gps_out().get_op_start(1),      1e-10);
    check("07 gps_out op 1 end    ", (- t_sec_init + t_sec_gpsloss) / Deltat_sec_gps,         Omotion.get_trj_gps_out().get_op_end(1),        1e-10);

    check("07 out nel             ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_out),   Omotion.get_trj_out().get_nel(),                1e-10);
    check("07 out nel_op          ", nel_op,                                                  Omotion.get_trj_out().get_nel_op(),             1e-10);
    check("07 out last time       ", t_sec_end_op1,                                           Omotion.get_trj_out()().back().get_t_sec(),     1e-10);
    check("07 out op 0 start      ", 0,                                                       Omotion.get_trj_out().get_op_start(0),          1e-10);
    check("07 out op 0 end        ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_out,         Omotion.get_trj_out().get_op_end(0),            1e-10);
    check("07 out op 1 start      ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_out,     Omotion.get_trj_out().get_op_start(1),          1e-10);
    check("07 out op 1 end        ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_out,         Omotion.get_trj_out().get_op_end(1),            1e-10);

    check("07 filter air          ", 1 + (- t_sec_init  + t_sec_end_trj) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_air().get_size());
    check("07 filter att          ", 1 + (- t_sec_init  + t_sec_end_trj) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_att().get_size());
    check("07 filter gps fast     ", 1 + (- t_sec_init  + t_sec_gpsloss) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_gps().get_size_fast());
    check("07 filter gps slow     ", 1 + (- t_sec_init  + t_sec_gpsloss) / Deltat_sec_gps,    Omotion.get_filter_nav().get_filter_gps().get_size_slow());
    check("07 filter pos          ", 1 + (- t_sec_gpsloss + t_sec_end_trj) / Deltat_sec_sens, Omotion.get_filter_nav().get_filter_pos().get_size());
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void nav::test::Ttrj_sizes::test_sizes08() {
    unsigned short seed_order = 1;
    double turb_factor        = 1.0;

    double t_sec_init    = -5.;
    double t_sec_end_trj = 20.0;
    double t_sec_end_op0 = 10.0;
    double t_sec_end_op1 = 20.0;
    double t_sec_gpsloss = 25.0;
    int    t_sec_turb    = 1000;

    sens::logic::SENS_COMPLETE_ID sens_complete_id = sens::logic::sens_complete_bias_scale_par;
    nav::logic::NAV_ID nav_id                      = nav::logic::nav_id010202;

    env::logic::WIND_ID wind_id         = env::logic::wind_id00;
    env::logic::OFFSETS_ID offsets_id   = env::logic::offsets_id00;
    env::logic::MAG_ID env_mag_id       = env::logic::mag_default;
    env::logic::REALISM_ID realism_id   = env::logic::realism_grav_no_magn_no;

    nav::logic::INITEUL_ID initeul_id   = nav::logic::initeul_id_zero;
    nav::logic::INITACC_ID initacc_id   = nav::logic::initacc_id_zero;
    nav::logic::INITGYR_ID initgyr_id   = nav::logic::initgyr_id_zero;
    nav::logic::INITMAG_ID initmag_id   = nav::logic::initmag_id_zero;
    nav::logic::INITMGN_ID initmgn_id   = nav::logic::initmgn_id_zero;

    sens::logic::GYR_ID gyr_id          = sens::logic::gyr_id_base;
    sens::logic::ACC_ID acc_id          = sens::logic::acc_id_base;
    sens::logic::MAG_ID mag_id          = sens::logic::mag_id_base;
    sens::logic::OSP_ID osp_id          = sens::logic::osp_id_base;
    sens::logic::OAT_ID oat_id          = sens::logic::oat_id_base;
    sens::logic::TAS_ID tas_id          = sens::logic::tas_id_base;
    sens::logic::AOA_ID aoa_id          = sens::logic::aoa_id_base;
    sens::logic::AOS_ID aos_id          = sens::logic::aos_id_base;
    sens::logic::GPS_ID gps_id          = sens::logic::gps_id_base;
    sens::logic::BAND_ID band_id        = sens::logic::band_id_base;

    st::logic::STI_ID sti_id = st::logic::sti_Hp3000_tas30_chi00;
    unsigned short nel_op = 2;

    math::seeder Oseeder(seed_order);
    auto Paero                = new acft::aero0();
    auto Pprop                = new acft::prop();
    auto Piner                = new acft::iner();
    auto Pearth               = new env::earth(Oseeder, wind_id, offsets_id, env_mag_id, realism_id, turb_factor, t_sec_init, t_sec_turb);
    auto Psuite               = new sens::suite(Oseeder, acc_id, gyr_id, mag_id, osp_id, oat_id, tas_id, aoa_id, aos_id, gps_id, band_id);
    auto Perr0                = new nav::init_error(Oseeder, initeul_id, initacc_id, initgyr_id, initmag_id, initmgn_id);
    auto Pguid                = new control::guid(nel_op, t_sec_end_trj, Oseeder.provide_seed(math::seeder::seeder_guid),0);
    Pguid->get_sti_id() = sti_id;
    Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 0.0, rud_beta_deg, 0., trgg_t_sec, t_sec_end_op0));
    Pguid->add_op(1, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 0.0, rud_beta_deg, 0., trgg_t_sec, t_sec_end_op1));
    st::sti* Psti             = st::sti::create_sti(*Pguid, t_sec_init);
    auto Pcntr                = new control::cntr(*Pguid);
    auto             Pnav     = new nav::filter_nav(nav_id, *Psuite, *Pearth, turb_factor);
    ang::euler euler_bc(0.0, 0.0, 0.0);
    auto Pcam                 = new sens::camera(768, 1024, 49.2255, euler_bc);

    nav::motion Omotion(Psti, Pguid, Pcntr, Pnav, Pearth, Psuite, Perr0, Piner, Paero, Pprop, Pcam, sens_complete_id);
    Omotion.solve_2nd(t_sec_end_trj, t_sec_gpsloss);

    double Deltat_sec_truth = 0.002;
    double Deltat_sec_sens  = 0.01;
    double Deltat_sec_gps   = 1.0;
    double Deltat_sec_cntr  = 0.02;
    double Deltat_sec_out   = 0.1;

    check("08 truth nel           ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_truth), Omotion.get_trj_truth().get_nel(),              1e-10);
    check("08 truth nel_op        ", nel_op,                                                  Omotion.get_trj_truth().get_nel_op(),           1e-10);
    check("08 truth last time     ", t_sec_end_op1,                                           Omotion.get_trj_truth()().back().get_t_sec(),   1e-10);
    check("08 truth op 0 start    ", 0,                                                       Omotion.get_trj_truth().get_op_start(0),        1e-10);
    check("08 truth op 0 end      ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_truth,       Omotion.get_trj_truth().get_op_end(0),          1e-10);
    check("08 truth op 1 start    ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_truth,   Omotion.get_trj_truth().get_op_start(1),        1e-10);
    check("08 truth op 1 end      ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_truth,       Omotion.get_trj_truth().get_op_end(1),          1e-10);

    check("08 sens_in nel         ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_sens),  Omotion.get_trj_sens_in().get_nel(),            1e-10);
    check("08 sens_in nel_op      ", nel_op,                                                  Omotion.get_trj_sens_in().get_nel_op(),         1e-10);
    check("08 sens_in last time   ", t_sec_end_op1,                                           Omotion.get_trj_sens_in()().back().get_t_sec(), 1e-10);
    check("08 sens_in op 0 start  ", 0,                                                       Omotion.get_trj_sens_in().get_op_start(0),      1e-10);
    check("08 sens_in op 0 end    ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_sens_in().get_op_end(0),        1e-10);
    check("08 sens_in op 1 start  ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,    Omotion.get_trj_sens_in().get_op_start(1),      1e-10);
    check("08 sens_in op 1 end    ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_sens,        Omotion.get_trj_sens_in().get_op_end(1),        1e-10);

    check("08 sens_out nel        ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_sens),  Omotion.get_trj_sens_out().get_nel(),           1e-10);
    check("08 sens_out nel_op     ", nel_op,                                                  Omotion.get_trj_sens_out().get_nel_op(),        1e-10);
    check("08 sens_out last time  ", t_sec_end_op1,                                           Omotion.get_trj_sens_out()().back().get_t_sec(),1e-10);
    check("08 sens_out op 0 start ", 0,                                                       Omotion.get_trj_sens_out().get_op_start(0),     1e-10);
    check("08 sens_out op 0 end   ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_sens_out().get_op_end(0),       1e-10);
    check("08 sens_out op 1 start ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,    Omotion.get_trj_sens_out().get_op_start(1),     1e-10);
    check("08 sens_out op 1 end   ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_sens,        Omotion.get_trj_sens_out().get_op_end(1),       1e-10);

    check("08 cntr nel            ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_cntr),  Omotion.get_trj_cntr().get_nel(),               1e-10);
    check("08 cntr nel_op         ", nel_op,                                                  Omotion.get_trj_cntr().get_nel_op(),            1e-10);
    check("08 cntr last time      ", t_sec_end_op1,                                           Omotion.get_trj_cntr()().back().get_t_sec(),    1e-10);
    check("08 cntr op 0 start     ", 0,                                                       Omotion.get_trj_cntr().get_op_start(0),         1e-10);
    check("08 cntr op 0 end       ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_cntr,        Omotion.get_trj_cntr().get_op_end(0),           1e-10);
    check("08 cntr op 1 start     ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_cntr,    Omotion.get_trj_cntr().get_op_start(1),         1e-10);
    check("08 cntr op 1 end       ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_cntr,        Omotion.get_trj_cntr().get_op_end(1),           1e-10);

    check("08 nav_in nel          ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_sens),  Omotion.get_trj_nav_in().get_nel(),             1e-10);
    check("08 nav_in nel_op       ", nel_op,                                                  Omotion.get_trj_nav_in().get_nel_op(),          1e-10);
    check("08 nav_in last time    ", t_sec_end_op1,                                           Omotion.get_trj_nav_in()().back().get_t_sec(),  1e-10);
    check("08 nav_in op 0 start   ", 0,                                                       Omotion.get_trj_nav_in().get_op_start(0),       1e-10);
    check("08 nav_in op 0 end     ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_nav_in().get_op_end(0),         1e-10);
    check("08 nav_in op 1 start   ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,    Omotion.get_trj_nav_in().get_op_start(1),       1e-10);
    check("08 nav_in op 1 end     ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_sens,        Omotion.get_trj_nav_in().get_op_end(1),         1e-10);

    check("08 nav_out nel         ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_sens),  Omotion.get_trj_nav_out().get_nel(),            1e-10);
    check("08 nav_out nel_op      ", nel_op,                                                  Omotion.get_trj_nav_out().get_nel_op(),         1e-10);
    check("08 nav_out last time   ", t_sec_end_op1,                                           Omotion.get_trj_nav_out()().back().get_t_sec(), 1e-10);
    check("08 nav_out op 0 start  ", 0,                                                       Omotion.get_trj_nav_out().get_op_start(0),      1e-10);
    check("08 nav_out op 0 end    ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_nav_out().get_op_end(0),        1e-10);
    check("08 nav_out op 1 start  ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,    Omotion.get_trj_nav_out().get_op_start(1),      1e-10);
    check("08 nav_out op 1 end    ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_sens,        Omotion.get_trj_nav_out().get_op_end(1),        1e-10);

    check("08 gps_out nel         ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_gps),   Omotion.get_trj_gps_out().get_nel(),            1e-10);
    check("08 gps_out nel_op      ", nel_op,                                                  Omotion.get_trj_gps_out().get_nel_op(),         1e-10);
    check("08 gps_out last time   ", t_sec_end_op1,                                           Omotion.get_trj_gps_out()().back().get_t_sec(), 1e-10);
    check("08 gps_out op 0 start  ", 0,                                                       Omotion.get_trj_gps_out().get_op_start(0),      1e-10);
    check("08 gps_out op 0 end    ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_gps,         Omotion.get_trj_gps_out().get_op_end(0),        1e-10);
    check("08 gps_out op 1 start  ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_gps,     Omotion.get_trj_gps_out().get_op_start(1),      1e-10);
    check("08 gps_out op 1 end    ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_gps,         Omotion.get_trj_gps_out().get_op_end(1),        1e-10);

    check("08 out nel             ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_out),   Omotion.get_trj_out().get_nel(),                1e-10);
    check("08 out nel_op          ", nel_op,                                                  Omotion.get_trj_out().get_nel_op(),             1e-10);
    check("08 out last time       ", t_sec_end_op1,                                           Omotion.get_trj_out()().back().get_t_sec(),     1e-10);
    check("08 out op 0 start      ", 0,                                                       Omotion.get_trj_out().get_op_start(0),          1e-10);
    check("08 out op 0 end        ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_out,         Omotion.get_trj_out().get_op_end(0),            1e-10);
    check("08 out op 1 start      ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_out,     Omotion.get_trj_out().get_op_start(1),          1e-10);
    check("08 out op 1 end        ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_out,         Omotion.get_trj_out().get_op_end(1),            1e-10);

    check("08 filter air          ", 1 + (- t_sec_init  + t_sec_end_trj) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_air().get_size());
    check("08 filter att          ", 1 + (- t_sec_init  + t_sec_end_trj) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_att().get_size());
    check("08 filter gps fast     ", 1 + (- t_sec_init  + t_sec_end_trj) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_gps().get_size_fast());
    check("08 filter gps slow     ", 1 + (- t_sec_init  + t_sec_end_trj) / Deltat_sec_gps,    Omotion.get_filter_nav().get_filter_gps().get_size_slow());
    check("08 filter pos          ", 1 + (- t_sec_end_trj + t_sec_end_trj) / Deltat_sec_sens, Omotion.get_filter_nav().get_filter_pos().get_size());
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void nav::test::Ttrj_sizes::test_sizes11() {
    unsigned short seed_order = 1;
    double turb_factor        = 1.0;

    double t_sec_init    = -5.;
    double t_sec_end_trj = 20.0;
    double t_sec_end_op0 = 10.0;
    double t_sec_gpsloss = 5.0;
    int    t_sec_turb    = 1000;

    sens::logic::SENS_COMPLETE_ID sens_complete_id = sens::logic::sens_complete_bias_scale_par;
    nav::logic::NAV_ID nav_id                      = nav::logic::nav_id010202;

    env::logic::WIND_ID wind_id         = env::logic::wind_id00;
    env::logic::OFFSETS_ID offsets_id   = env::logic::offsets_id00;
    env::logic::MAG_ID env_mag_id       = env::logic::mag_default;
    env::logic::REALISM_ID realism_id   = env::logic::realism_grav_no_magn_no;

    nav::logic::INITEUL_ID initeul_id   = nav::logic::initeul_id_zero;
    nav::logic::INITACC_ID initacc_id   = nav::logic::initacc_id_zero;
    nav::logic::INITGYR_ID initgyr_id   = nav::logic::initgyr_id_zero;
    nav::logic::INITMAG_ID initmag_id   = nav::logic::initmag_id_zero;
    nav::logic::INITMGN_ID initmgn_id   = nav::logic::initmgn_id_zero;

    sens::logic::GYR_ID gyr_id          = sens::logic::gyr_id_base;
    sens::logic::ACC_ID acc_id          = sens::logic::acc_id_base;
    sens::logic::MAG_ID mag_id          = sens::logic::mag_id_base;
    sens::logic::OSP_ID osp_id          = sens::logic::osp_id_base;
    sens::logic::OAT_ID oat_id          = sens::logic::oat_id_base;
    sens::logic::TAS_ID tas_id          = sens::logic::tas_id_base;
    sens::logic::AOA_ID aoa_id          = sens::logic::aoa_id_base;
    sens::logic::AOS_ID aos_id          = sens::logic::aos_id_base;
    sens::logic::GPS_ID gps_id          = sens::logic::gps_id_base;
    sens::logic::BAND_ID band_id        = sens::logic::band_id_base;

    st::logic::STI_ID sti_id = st::logic::sti_Hp3000_tas30_chi00;
    unsigned short nel_op = 1;

    math::seeder Oseeder(seed_order);
    auto Paero                = new acft::aero0();
    auto Pprop                = new acft::prop();
    auto Piner                = new acft::iner();
    auto Pearth               = new env::earth(Oseeder, wind_id, offsets_id, env_mag_id, realism_id, turb_factor, t_sec_init, t_sec_turb);
    auto Psuite               = new sens::suite(Oseeder, acc_id, gyr_id, mag_id, osp_id, oat_id, tas_id, aoa_id, aos_id, gps_id, band_id);
    auto Perr0                = new nav::init_error(Oseeder, initeul_id, initacc_id, initgyr_id, initmag_id, initmgn_id);
    auto Pguid                = new control::guid(nel_op, t_sec_end_trj, Oseeder.provide_seed(math::seeder::seeder_guid),0);
    Pguid->get_sti_id() = sti_id;
    Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 0.0, rud_beta_deg, 0., trgg_t_sec, t_sec_end_op0));
    st::sti* Psti             = st::sti::create_sti(*Pguid, t_sec_init);
    auto Pcntr                = new control::cntr(*Pguid);
    auto             Pnav     = new nav::filter_nav(nav_id, *Psuite, *Pearth, turb_factor);
    ang::euler euler_bc(0.0, 0.0, 0.0);
    auto Pcam                 = new sens::camera(768, 1024, 49.2255, euler_bc);

    nav::motion Omotion(Psti, Pguid, Pcntr, Pnav, Pearth, Psuite, Perr0, Piner, Paero, Pprop, Pcam, sens_complete_id);
    Omotion.solve_2nd(t_sec_end_trj, t_sec_gpsloss);

    double Deltat_sec_truth = 0.002;
    double Deltat_sec_sens  = 0.01;
    double Deltat_sec_gps   = 1.0;
    double Deltat_sec_cntr  = 0.02;
    double Deltat_sec_out   = 0.1;

    check("11 truth nel           ", 1 + ((- t_sec_init + t_sec_end_op0) / Deltat_sec_truth), Omotion.get_trj_truth().get_nel(),              1e-10);
    check("11 truth nel_op        ", nel_op,                                                  Omotion.get_trj_truth().get_nel_op(),           1e-10);
    check("11 truth last time     ", t_sec_end_op0,                                           Omotion.get_trj_truth()().back().get_t_sec(),   1e-10);
    check("11 truth op 0 start    ", 0,                                                       Omotion.get_trj_truth().get_op_start(0),        1e-10);
    check("11 truth op 0 end      ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_truth,       Omotion.get_trj_truth().get_op_end(0),          1e-10);

    check("11 sens_in nel         ", 1 + ((- t_sec_init + t_sec_end_op0) / Deltat_sec_sens),  Omotion.get_trj_sens_in().get_nel(),            1e-10);
    check("11 sens_in nel_op      ", nel_op,                                                  Omotion.get_trj_sens_in().get_nel_op(),         1e-10);
    check("11 sens_in last time   ", t_sec_end_op0,                                           Omotion.get_trj_sens_in()().back().get_t_sec(), 1e-10);
    check("11 sens_in op 0 start  ", 0,                                                       Omotion.get_trj_sens_in().get_op_start(0),      1e-10);
    check("11 sens_in op 0 end    ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_sens_in().get_op_end(0),        1e-10);

    check("11 sens_out nel        ", 1 + ((- t_sec_init + t_sec_end_op0) / Deltat_sec_sens),  Omotion.get_trj_sens_out().get_nel(),           1e-10);
    check("11 sens_out nel_op     ", nel_op,                                                  Omotion.get_trj_sens_out().get_nel_op(),        1e-10);
    check("11 sens_out last time  ", t_sec_end_op0,                                           Omotion.get_trj_sens_out()().back().get_t_sec(),1e-10);
    check("11 sens_out op 0 start ", 0,                                                       Omotion.get_trj_sens_out().get_op_start(0),     1e-10);
    check("11 sens_out op 0 end   ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_sens_out().get_op_end(0),       1e-10);

    check("11 cntr nel            ", 1 + ((- t_sec_init + t_sec_end_op0) / Deltat_sec_cntr),  Omotion.get_trj_cntr().get_nel(),               1e-10);
    check("11 cntr nel_op         ", nel_op,                                                  Omotion.get_trj_cntr().get_nel_op(),            1e-10);
    check("11 cntr last time      ", t_sec_end_op0,                                           Omotion.get_trj_cntr()().back().get_t_sec(),    1e-10);
    check("11 cntr op 0 start     ", 0,                                                       Omotion.get_trj_cntr().get_op_start(0),         1e-10);
    check("11 cntr op 0 end       ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_cntr,        Omotion.get_trj_cntr().get_op_end(0),           1e-10);

    check("11 nav_in nel          ", 1 + ((- t_sec_init + t_sec_end_op0) / Deltat_sec_sens),  Omotion.get_trj_nav_in().get_nel(),             1e-10);
    check("11 nav_in nel_op       ", nel_op,                                                  Omotion.get_trj_nav_in().get_nel_op(),          1e-10);
    check("11 nav_in last time    ", t_sec_end_op0,                                           Omotion.get_trj_nav_in()().back().get_t_sec(),  1e-10);
    check("11 nav_in op 0 start   ", 0,                                                       Omotion.get_trj_nav_in().get_op_start(0),       1e-10);
    check("11 nav_in op 0 end     ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_nav_in().get_op_end(0),         1e-10);

    check("11 nav_out nel         ", 1 + ((- t_sec_init + t_sec_end_op0) / Deltat_sec_sens),  Omotion.get_trj_nav_out().get_nel(),            1e-10);
    check("11 nav_out nel_op      ", nel_op,                                                  Omotion.get_trj_nav_out().get_nel_op(),         1e-10);
    check("11 nav_out last time   ", t_sec_end_op0,                                           Omotion.get_trj_nav_out()().back().get_t_sec(), 1e-10);
    check("11 nav_out op 0 start  ", 0,                                                       Omotion.get_trj_nav_out().get_op_start(0),      1e-10);
    check("11 nav_out op 0 end    ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_nav_out().get_op_end(0),        1e-10);

    check("11 gps_out nel         ", 1 + ((- t_sec_init + t_sec_gpsloss) / Deltat_sec_gps),   Omotion.get_trj_gps_out().get_nel(),            1e-10);
    check("11 gps_out nel_op      ", nel_op,                                                  Omotion.get_trj_gps_out().get_nel_op(),         1e-10);
    check("11 gps_out last time   ", t_sec_gpsloss,                                           Omotion.get_trj_gps_out()().back().get_t_sec(), 1e-10);
    check("11 gps_out op 0 start  ", 0,                                                       Omotion.get_trj_gps_out().get_op_start(0),      1e-10);
    check("11 gps_out op 1 end    ", (- t_sec_init + t_sec_gpsloss) / Deltat_sec_gps,         Omotion.get_trj_gps_out().get_op_end(1),        1e-10);

    check("11 out nel             ", 1 + ((- t_sec_init + t_sec_end_op0) / Deltat_sec_out),   Omotion.get_trj_out().get_nel(),                1e-10);
    check("11 out nel_op          ", nel_op,                                                  Omotion.get_trj_out().get_nel_op(),             1e-10);
    check("11 out last time       ", t_sec_end_op0,                                           Omotion.get_trj_out()().back().get_t_sec(),     1e-10);
    check("11 out op 0 start      ", 0,                                                       Omotion.get_trj_out().get_op_start(0),          1e-10);
    check("11 out op 0 end        ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_out,         Omotion.get_trj_out().get_op_end(0),            1e-10);

    check("11 filter air          ", 1 + (- t_sec_init  + t_sec_end_op0) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_air().get_size());
    check("11 filter att          ", 1 + (- t_sec_init  + t_sec_end_op0) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_att().get_size());
    check("11 filter gps fast     ", 1 + (- t_sec_init  + t_sec_gpsloss) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_gps().get_size_fast());
    check("11 filter gps slow     ", 1 + (- t_sec_init  + t_sec_gpsloss) / Deltat_sec_gps,    Omotion.get_filter_nav().get_filter_gps().get_size_slow());
    check("11 filter pos          ", 1 + (- t_sec_gpsloss + t_sec_end_op0) / Deltat_sec_sens, Omotion.get_filter_nav().get_filter_pos().get_size());
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void nav::test::Ttrj_sizes::test_sizes12() {
    unsigned short seed_order = 1;
    double turb_factor        = 1.0;

    double t_sec_init    = -5.;
    double t_sec_end_trj = 20.0;
    double t_sec_end_op0 = 10.0;
    double t_sec_gpsloss = 10.0;
    int    t_sec_turb    = 1000;

    sens::logic::SENS_COMPLETE_ID sens_complete_id = sens::logic::sens_complete_bias_scale_par;
    nav::logic::NAV_ID nav_id                      = nav::logic::nav_id010202;

    env::logic::WIND_ID wind_id         = env::logic::wind_id00;
    env::logic::OFFSETS_ID offsets_id   = env::logic::offsets_id00;
    env::logic::MAG_ID env_mag_id       = env::logic::mag_default;
    env::logic::REALISM_ID realism_id   = env::logic::realism_grav_no_magn_no;

    nav::logic::INITEUL_ID initeul_id   = nav::logic::initeul_id_zero;
    nav::logic::INITACC_ID initacc_id   = nav::logic::initacc_id_zero;
    nav::logic::INITGYR_ID initgyr_id   = nav::logic::initgyr_id_zero;
    nav::logic::INITMAG_ID initmag_id   = nav::logic::initmag_id_zero;
    nav::logic::INITMGN_ID initmgn_id   = nav::logic::initmgn_id_zero;

    sens::logic::GYR_ID gyr_id          = sens::logic::gyr_id_base;
    sens::logic::ACC_ID acc_id          = sens::logic::acc_id_base;
    sens::logic::MAG_ID mag_id          = sens::logic::mag_id_base;
    sens::logic::OSP_ID osp_id          = sens::logic::osp_id_base;
    sens::logic::OAT_ID oat_id          = sens::logic::oat_id_base;
    sens::logic::TAS_ID tas_id          = sens::logic::tas_id_base;
    sens::logic::AOA_ID aoa_id          = sens::logic::aoa_id_base;
    sens::logic::AOS_ID aos_id          = sens::logic::aos_id_base;
    sens::logic::GPS_ID gps_id          = sens::logic::gps_id_base;
    sens::logic::BAND_ID band_id        = sens::logic::band_id_base;

    st::logic::STI_ID sti_id = st::logic::sti_Hp3000_tas30_chi00;
    unsigned short nel_op = 1;

    math::seeder Oseeder(seed_order);
    auto Paero                = new acft::aero0();
    auto Pprop                = new acft::prop();
    auto Piner                = new acft::iner();
    auto Pearth               = new env::earth(Oseeder, wind_id, offsets_id, env_mag_id, realism_id, turb_factor, t_sec_init, t_sec_turb);
    auto Psuite               = new sens::suite(Oseeder, acc_id, gyr_id, mag_id, osp_id, oat_id, tas_id, aoa_id, aos_id, gps_id, band_id);
    auto Perr0                = new nav::init_error(Oseeder, initeul_id, initacc_id, initgyr_id, initmag_id, initmgn_id);
    auto Pguid                = new control::guid(nel_op, t_sec_end_trj, Oseeder.provide_seed(math::seeder::seeder_guid),0);
    Pguid->get_sti_id() = sti_id;
    Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 0.0, rud_beta_deg, 0., trgg_t_sec, t_sec_end_op0));
    st::sti* Psti             = st::sti::create_sti(*Pguid, t_sec_init);
    auto Pcntr                = new control::cntr(*Pguid);
    auto             Pnav     = new nav::filter_nav(nav_id, *Psuite, *Pearth, turb_factor);
    ang::euler euler_bc(0.0, 0.0, 0.0);
    auto Pcam                 = new sens::camera(768, 1024, 49.2255, euler_bc);

    nav::motion Omotion(Psti, Pguid, Pcntr, Pnav, Pearth, Psuite, Perr0, Piner, Paero, Pprop, Pcam, sens_complete_id);
    Omotion.solve_2nd(t_sec_end_trj, t_sec_gpsloss);

    double Deltat_sec_truth = 0.002;
    double Deltat_sec_sens  = 0.01;
    double Deltat_sec_gps   = 1.0;
    double Deltat_sec_cntr  = 0.02;
    double Deltat_sec_out   = 0.1;

    check("12 truth nel           ", 1 + ((- t_sec_init + t_sec_end_op0) / Deltat_sec_truth), Omotion.get_trj_truth().get_nel(),              1e-10);
    check("12 truth nel_op        ", nel_op,                                                  Omotion.get_trj_truth().get_nel_op(),           1e-10);
    check("12 truth last time     ", t_sec_end_op0,                                           Omotion.get_trj_truth()().back().get_t_sec(),   1e-10);
    check("12 truth op 0 start    ", 0,                                                       Omotion.get_trj_truth().get_op_start(0),        1e-10);
    check("12 truth op 0 end      ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_truth,       Omotion.get_trj_truth().get_op_end(0),          1e-10);

    check("12 sens_in nel         ", 1 + ((- t_sec_init + t_sec_end_op0) / Deltat_sec_sens),  Omotion.get_trj_sens_in().get_nel(),            1e-10);
    check("12 sens_in nel_op      ", nel_op,                                                  Omotion.get_trj_sens_in().get_nel_op(),         1e-10);
    check("12 sens_in last time   ", t_sec_end_op0,                                           Omotion.get_trj_sens_in()().back().get_t_sec(), 1e-10);
    check("12 sens_in op 0 start  ", 0,                                                       Omotion.get_trj_sens_in().get_op_start(0),      1e-10);
    check("12 sens_in op 0 end    ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_sens_in().get_op_end(0),        1e-10);

    check("12 sens_out nel        ", 1 + ((- t_sec_init + t_sec_end_op0) / Deltat_sec_sens),  Omotion.get_trj_sens_out().get_nel(),           1e-10);
    check("12 sens_out nel_op     ", nel_op,                                                  Omotion.get_trj_sens_out().get_nel_op(),        1e-10);
    check("12 sens_out last time  ", t_sec_end_op0,                                           Omotion.get_trj_sens_out()().back().get_t_sec(),1e-10);
    check("12 sens_out op 0 start ", 0,                                                       Omotion.get_trj_sens_out().get_op_start(0),     1e-10);
    check("12 sens_out op 0 end   ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_sens_out().get_op_end(0),       1e-10);

    check("12 cntr nel            ", 1 + ((- t_sec_init + t_sec_end_op0) / Deltat_sec_cntr),  Omotion.get_trj_cntr().get_nel(),               1e-10);
    check("12 cntr nel_op         ", nel_op,                                                  Omotion.get_trj_cntr().get_nel_op(),            1e-10);
    check("12 cntr last time      ", t_sec_end_op0,                                           Omotion.get_trj_cntr()().back().get_t_sec(),    1e-10);
    check("12 cntr op 0 start     ", 0,                                                       Omotion.get_trj_cntr().get_op_start(0),         1e-10);
    check("12 cntr op 0 end       ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_cntr,        Omotion.get_trj_cntr().get_op_end(0),           1e-10);

    check("12 nav_in nel          ", 1 + ((- t_sec_init + t_sec_end_op0) / Deltat_sec_sens),  Omotion.get_trj_nav_in().get_nel(),             1e-10);
    check("12 nav_in nel_op       ", nel_op,                                                  Omotion.get_trj_nav_in().get_nel_op(),          1e-10);
    check("12 nav_in last time    ", t_sec_end_op0,                                           Omotion.get_trj_nav_in()().back().get_t_sec(),  1e-10);
    check("12 nav_in op 0 start   ", 0,                                                       Omotion.get_trj_nav_in().get_op_start(0),       1e-10);
    check("12 nav_in op 0 end     ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_nav_in().get_op_end(0),         1e-10);

    check("12 nav_out nel         ", 1 + ((- t_sec_init + t_sec_end_op0) / Deltat_sec_sens),  Omotion.get_trj_nav_out().get_nel(),            1e-10);
    check("12 nav_out nel_op      ", nel_op,                                                  Omotion.get_trj_nav_out().get_nel_op(),         1e-10);
    check("12 nav_out last time   ", t_sec_end_op0,                                           Omotion.get_trj_nav_out()().back().get_t_sec(), 1e-10);
    check("12 nav_out op 0 start  ", 0,                                                       Omotion.get_trj_nav_out().get_op_start(0),      1e-10);
    check("12 nav_out op 0 end    ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_nav_out().get_op_end(0),        1e-10);

    check("12 gps_out nel         ", 1 + ((- t_sec_init + t_sec_gpsloss) / Deltat_sec_gps),   Omotion.get_trj_gps_out().get_nel(),            1e-10);
    check("12 gps_out nel_op      ", nel_op,                                                  Omotion.get_trj_gps_out().get_nel_op(),         1e-10);
    check("12 gps_out last time   ", t_sec_gpsloss,                                           Omotion.get_trj_gps_out()().back().get_t_sec(), 1e-10);
    check("12 gps_out op 0 start  ", 0,                                                       Omotion.get_trj_gps_out().get_op_start(0),      1e-10);
    check("12 gps_out op 1 end    ", (- t_sec_init + t_sec_gpsloss) / Deltat_sec_gps,         Omotion.get_trj_gps_out().get_op_end(1),        1e-10);

    check("12 out nel             ", 1 + ((- t_sec_init + t_sec_end_op0) / Deltat_sec_out),   Omotion.get_trj_out().get_nel(),                1e-10);
    check("12 out nel_op          ", nel_op,                                                  Omotion.get_trj_out().get_nel_op(),             1e-10);
    check("12 out last time       ", t_sec_end_op0,                                           Omotion.get_trj_out()().back().get_t_sec(),     1e-10);
    check("12 out op 0 start      ", 0,                                                       Omotion.get_trj_out().get_op_start(0),          1e-10);
    check("12 out op 0 end        ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_out,         Omotion.get_trj_out().get_op_end(0),            1e-10);

    check("12 filter air          ", 1 + (- t_sec_init  + t_sec_end_op0) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_air().get_size());
    check("12 filter att          ", 1 + (- t_sec_init  + t_sec_end_op0) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_att().get_size());
    check("12 filter gps fast     ", 1 + (- t_sec_init  + t_sec_gpsloss) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_gps().get_size_fast());
    check("12 filter gps slow     ", 1 + (- t_sec_init  + t_sec_gpsloss) / Deltat_sec_gps,    Omotion.get_filter_nav().get_filter_gps().get_size_slow());
    check("12 filter pos          ", 1 + (- t_sec_gpsloss + t_sec_end_op0) / Deltat_sec_sens, Omotion.get_filter_nav().get_filter_pos().get_size());
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void nav::test::Ttrj_sizes::test_sizes13() {
    unsigned short seed_order = 1;
    double turb_factor        = 1.0;

    double t_sec_init    = -5.;
    double t_sec_end_trj = 20.0;
    double t_sec_end_op0 = 10.0;
    double t_sec_end_op1 = 20.0;
    double t_sec_gpsloss = 15.0;
    int    t_sec_turb    = 1000;

    sens::logic::SENS_COMPLETE_ID sens_complete_id = sens::logic::sens_complete_bias_scale_par;
    nav::logic::NAV_ID nav_id                      = nav::logic::nav_id010202;

    env::logic::WIND_ID wind_id         = env::logic::wind_id00;
    env::logic::OFFSETS_ID offsets_id   = env::logic::offsets_id00;
    env::logic::MAG_ID env_mag_id       = env::logic::mag_default;
    env::logic::REALISM_ID realism_id   = env::logic::realism_grav_no_magn_no;

    nav::logic::INITEUL_ID initeul_id   = nav::logic::initeul_id_zero;
    nav::logic::INITACC_ID initacc_id   = nav::logic::initacc_id_zero;
    nav::logic::INITGYR_ID initgyr_id   = nav::logic::initgyr_id_zero;
    nav::logic::INITMAG_ID initmag_id   = nav::logic::initmag_id_zero;
    nav::logic::INITMGN_ID initmgn_id   = nav::logic::initmgn_id_zero;

    sens::logic::GYR_ID gyr_id          = sens::logic::gyr_id_base;
    sens::logic::ACC_ID acc_id          = sens::logic::acc_id_base;
    sens::logic::MAG_ID mag_id          = sens::logic::mag_id_base;
    sens::logic::OSP_ID osp_id          = sens::logic::osp_id_base;
    sens::logic::OAT_ID oat_id          = sens::logic::oat_id_base;
    sens::logic::TAS_ID tas_id          = sens::logic::tas_id_base;
    sens::logic::AOA_ID aoa_id          = sens::logic::aoa_id_base;
    sens::logic::AOS_ID aos_id          = sens::logic::aos_id_base;
    sens::logic::GPS_ID gps_id          = sens::logic::gps_id_base;
    sens::logic::BAND_ID band_id        = sens::logic::band_id_base;

    st::logic::STI_ID sti_id = st::logic::sti_Hp3000_tas30_chi00;
    unsigned short nel_op = 1;

    math::seeder Oseeder(seed_order);
    auto Paero                = new acft::aero0();
    auto Pprop                = new acft::prop();
    auto Piner                = new acft::iner();
    auto Pearth               = new env::earth(Oseeder, wind_id, offsets_id, env_mag_id, realism_id, turb_factor, t_sec_init, t_sec_turb);
    auto Psuite               = new sens::suite(Oseeder, acc_id, gyr_id, mag_id, osp_id, oat_id, tas_id, aoa_id, aos_id, gps_id, band_id);
    auto Perr0                = new nav::init_error(Oseeder, initeul_id, initacc_id, initgyr_id, initmag_id, initmgn_id);
    auto Pguid                = new control::guid(nel_op, t_sec_end_trj, Oseeder.provide_seed(math::seeder::seeder_guid),0);
    Pguid->get_sti_id() = sti_id;
    Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 0.0, rud_beta_deg, 0., trgg_t_sec, t_sec_end_op0));
    st::sti* Psti             = st::sti::create_sti(*Pguid, t_sec_init);
    auto Pcntr                = new control::cntr(*Pguid);
    auto             Pnav     = new nav::filter_nav(nav_id, *Psuite, *Pearth, turb_factor);
    ang::euler euler_bc(0.0, 0.0, 0.0);
    auto Pcam                 = new sens::camera(768, 1024, 49.2255, euler_bc);

    nav::motion Omotion(Psti, Pguid, Pcntr, Pnav, Pearth, Psuite, Perr0, Piner, Paero, Pprop, Pcam, sens_complete_id);
    Omotion.solve_2nd(t_sec_end_trj, t_sec_gpsloss);

    double Deltat_sec_truth = 0.002;
    double Deltat_sec_sens  = 0.01;
    double Deltat_sec_gps   = 1.0;
    double Deltat_sec_cntr  = 0.02;
    double Deltat_sec_out   = 0.1;

    check("13 truth nel           ", 1 + ((- t_sec_init + t_sec_end_op0) / Deltat_sec_truth), Omotion.get_trj_truth().get_nel(),              1e-10);
    check("13 truth nel_op        ", nel_op,                                                  Omotion.get_trj_truth().get_nel_op(),           1e-10);
    check("13 truth last time     ", t_sec_end_op0,                                           Omotion.get_trj_truth()().back().get_t_sec(),   1e-10);
    check("13 truth op 0 start    ", 0,                                                       Omotion.get_trj_truth().get_op_start(0),        1e-10);
    check("13 truth op 0 end      ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_truth,       Omotion.get_trj_truth().get_op_end(0),          1e-10);

    check("13 sens_in nel         ", 1 + ((- t_sec_init + t_sec_end_op0) / Deltat_sec_sens),  Omotion.get_trj_sens_in().get_nel(),            1e-10);
    check("13 sens_in nel_op      ", nel_op,                                                  Omotion.get_trj_sens_in().get_nel_op(),         1e-10);
    check("13 sens_in last time   ", t_sec_end_op0,                                           Omotion.get_trj_sens_in()().back().get_t_sec(), 1e-10);
    check("13 sens_in op 0 start  ", 0,                                                       Omotion.get_trj_sens_in().get_op_start(0),      1e-10);
    check("13 sens_in op 0 end    ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_sens_in().get_op_end(0),        1e-10);

    check("13 sens_out nel        ", 1 + ((- t_sec_init + t_sec_end_op0) / Deltat_sec_sens),  Omotion.get_trj_sens_out().get_nel(),           1e-10);
    check("13 sens_out nel_op     ", nel_op,                                                  Omotion.get_trj_sens_out().get_nel_op(),        1e-10);
    check("13 sens_out last time  ", t_sec_end_op0,                                           Omotion.get_trj_sens_out()().back().get_t_sec(),1e-10);
    check("13 sens_out op 0 start ", 0,                                                       Omotion.get_trj_sens_out().get_op_start(0),     1e-10);
    check("13 sens_out op 0 end   ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_sens_out().get_op_end(0),       1e-10);

    check("13 cntr nel            ", 1 + ((- t_sec_init + t_sec_end_op0) / Deltat_sec_cntr),  Omotion.get_trj_cntr().get_nel(),               1e-10);
    check("13 cntr nel_op         ", nel_op,                                                  Omotion.get_trj_cntr().get_nel_op(),            1e-10);
    check("13 cntr last time      ", t_sec_end_op0,                                           Omotion.get_trj_cntr()().back().get_t_sec(),    1e-10);
    check("13 cntr op 0 start     ", 0,                                                       Omotion.get_trj_cntr().get_op_start(0),         1e-10);
    check("13 cntr op 0 end       ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_cntr,        Omotion.get_trj_cntr().get_op_end(0),           1e-10);

    check("13 nav_in nel          ", 1 + ((- t_sec_init + t_sec_end_op0) / Deltat_sec_sens),  Omotion.get_trj_nav_in().get_nel(),             1e-10);
    check("13 nav_in nel_op       ", nel_op,                                                  Omotion.get_trj_nav_in().get_nel_op(),          1e-10);
    check("13 nav_in last time    ", t_sec_end_op0,                                           Omotion.get_trj_nav_in()().back().get_t_sec(),  1e-10);
    check("13 nav_in op 0 start   ", 0,                                                       Omotion.get_trj_nav_in().get_op_start(0),       1e-10);
    check("13 nav_in op 0 end     ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_nav_in().get_op_end(0),         1e-10);

    check("13 nav_out nel         ", 1 + ((- t_sec_init + t_sec_end_op0) / Deltat_sec_sens),  Omotion.get_trj_nav_out().get_nel(),            1e-10);
    check("13 nav_out nel_op      ", nel_op,                                                  Omotion.get_trj_nav_out().get_nel_op(),         1e-10);
    check("13 nav_out last time   ", t_sec_end_op0,                                           Omotion.get_trj_nav_out()().back().get_t_sec(), 1e-10);
    check("13 nav_out op 0 start  ", 0,                                                       Omotion.get_trj_nav_out().get_op_start(0),      1e-10);
    check("13 nav_out op 0 end    ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_nav_out().get_op_end(0),        1e-10);

    check("13 gps_out nel         ", 1 + ((- t_sec_init + t_sec_end_op0) / Deltat_sec_gps),   Omotion.get_trj_gps_out().get_nel(),            1e-10);
    check("13 gps_out nel_op      ", nel_op,                                                  Omotion.get_trj_gps_out().get_nel_op(),         1e-10);
    check("13 gps_out last time   ", t_sec_end_op0,                                           Omotion.get_trj_gps_out()().back().get_t_sec(), 1e-10);
    check("13 gps_out op 0 start  ", 0,                                                       Omotion.get_trj_gps_out().get_op_start(0),      1e-10);
    check("13 gps_out op 1 end    ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_gps,         Omotion.get_trj_gps_out().get_op_end(1),        1e-10);

    check("13 out nel             ", 1 + ((- t_sec_init + t_sec_end_op0) / Deltat_sec_out),   Omotion.get_trj_out().get_nel(),                1e-10);
    check("13 out nel_op          ", nel_op,                                                  Omotion.get_trj_out().get_nel_op(),             1e-10);
    check("13 out last time       ", t_sec_end_op0,                                           Omotion.get_trj_out()().back().get_t_sec(),     1e-10);
    check("13 out op 0 start      ", 0,                                                       Omotion.get_trj_out().get_op_start(0),          1e-10);
    check("13 out op 0 end        ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_out,         Omotion.get_trj_out().get_op_end(0),            1e-10);

    check("13 filter air          ", 1 + (- t_sec_init  + t_sec_end_op0) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_air().get_size());
    check("13 filter att          ", 1 + (- t_sec_init  + t_sec_end_op0) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_att().get_size());
    check("13 filter gps fast     ", 1 + (- t_sec_init  + t_sec_end_op0) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_gps().get_size_fast());
    check("13 filter gps slow     ", 1 + (- t_sec_init  + t_sec_end_op0) / Deltat_sec_gps,    Omotion.get_filter_nav().get_filter_gps().get_size_slow());
    check("13 filter pos          ", 1 + (- t_sec_end_op0 + t_sec_end_op0) / Deltat_sec_sens, Omotion.get_filter_nav().get_filter_pos().get_size());
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void nav::test::Ttrj_sizes::test_sizes14() {
    unsigned short seed_order = 1;
    double turb_factor        = 1.0;

    double t_sec_init    = -5.;
    double t_sec_end_trj = 30.0;
    double t_sec_end_op0 = 10.0;
    double t_sec_end_op1 = 20.0;
    double t_sec_gpsloss = 5.0;
    int    t_sec_turb    = 1000;

    sens::logic::SENS_COMPLETE_ID sens_complete_id = sens::logic::sens_complete_bias_scale_par;
    nav::logic::NAV_ID nav_id                      = nav::logic::nav_id010202;

    env::logic::WIND_ID wind_id         = env::logic::wind_id00;
    env::logic::OFFSETS_ID offsets_id   = env::logic::offsets_id00;
    env::logic::MAG_ID env_mag_id       = env::logic::mag_default;
    env::logic::REALISM_ID realism_id   = env::logic::realism_grav_no_magn_no;

    nav::logic::INITEUL_ID initeul_id   = nav::logic::initeul_id_zero;
    nav::logic::INITACC_ID initacc_id   = nav::logic::initacc_id_zero;
    nav::logic::INITGYR_ID initgyr_id   = nav::logic::initgyr_id_zero;
    nav::logic::INITMAG_ID initmag_id   = nav::logic::initmag_id_zero;
    nav::logic::INITMGN_ID initmgn_id   = nav::logic::initmgn_id_zero;

    sens::logic::GYR_ID gyr_id          = sens::logic::gyr_id_base;
    sens::logic::ACC_ID acc_id          = sens::logic::acc_id_base;
    sens::logic::MAG_ID mag_id          = sens::logic::mag_id_base;
    sens::logic::OSP_ID osp_id          = sens::logic::osp_id_base;
    sens::logic::OAT_ID oat_id          = sens::logic::oat_id_base;
    sens::logic::TAS_ID tas_id          = sens::logic::tas_id_base;
    sens::logic::AOA_ID aoa_id          = sens::logic::aoa_id_base;
    sens::logic::AOS_ID aos_id          = sens::logic::aos_id_base;
    sens::logic::GPS_ID gps_id          = sens::logic::gps_id_base;
    sens::logic::BAND_ID band_id        = sens::logic::band_id_base;

    st::logic::STI_ID sti_id = st::logic::sti_Hp3000_tas30_chi00;
    unsigned short nel_op = 2;

    math::seeder Oseeder(seed_order);
    auto Paero                = new acft::aero0();
    auto Pprop                = new acft::prop();
    auto Piner                = new acft::iner();
    auto Pearth               = new env::earth(Oseeder, wind_id, offsets_id, env_mag_id, realism_id, turb_factor, t_sec_init, t_sec_turb);
    auto Psuite               = new sens::suite(Oseeder, acc_id, gyr_id, mag_id, osp_id, oat_id, tas_id, aoa_id, aos_id, gps_id, band_id);
    auto Perr0                = new nav::init_error(Oseeder, initeul_id, initacc_id, initgyr_id, initmag_id, initmgn_id);
    auto Pguid                = new control::guid(nel_op, t_sec_end_trj, Oseeder.provide_seed(math::seeder::seeder_guid),0);
    Pguid->get_sti_id() = sti_id;
    Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 0.0, rud_beta_deg, 0., trgg_t_sec, t_sec_end_op0));
    Pguid->add_op(1, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 0.0, rud_beta_deg, 0., trgg_t_sec, t_sec_end_op1));
    st::sti* Psti             = st::sti::create_sti(*Pguid, t_sec_init);
    auto Pcntr                = new control::cntr(*Pguid);
    auto             Pnav     = new nav::filter_nav(nav_id, *Psuite, *Pearth, turb_factor);
    ang::euler euler_bc(0.0, 0.0, 0.0);
    auto Pcam                 = new sens::camera(768, 1024, 49.2255, euler_bc);

    nav::motion Omotion(Psti, Pguid, Pcntr, Pnav, Pearth, Psuite, Perr0, Piner, Paero, Pprop, Pcam, sens_complete_id);
    Omotion.solve_2nd(t_sec_end_trj, t_sec_gpsloss);

    double Deltat_sec_truth = 0.002;
    double Deltat_sec_sens  = 0.01;
    double Deltat_sec_gps   = 1.0;
    double Deltat_sec_cntr  = 0.02;
    double Deltat_sec_out   = 0.1;

    check("14 truth nel           ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_truth), Omotion.get_trj_truth().get_nel(),              1e-10);
    check("14 truth nel_op        ", nel_op,                                                  Omotion.get_trj_truth().get_nel_op(),           1e-10);
    check("14 truth last time     ", t_sec_end_op1,                                           Omotion.get_trj_truth()().back().get_t_sec(),   1e-10);
    check("14 truth op 0 start    ", 0,                                                       Omotion.get_trj_truth().get_op_start(0),        1e-10);
    check("14 truth op 0 end      ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_truth,       Omotion.get_trj_truth().get_op_end(0),          1e-10);
    check("14 truth op 1 start    ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_truth,   Omotion.get_trj_truth().get_op_start(1),        1e-10);
    check("14 truth op 1 end      ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_truth,       Omotion.get_trj_truth().get_op_end(1),          1e-10);

    check("14 sens_in nel         ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_sens),  Omotion.get_trj_sens_in().get_nel(),            1e-10);
    check("14 sens_in nel_op      ", nel_op,                                                  Omotion.get_trj_sens_in().get_nel_op(),         1e-10);
    check("14 sens_in last time   ", t_sec_end_op1,                                           Omotion.get_trj_sens_in()().back().get_t_sec(), 1e-10);
    check("14 sens_in op 0 start  ", 0,                                                       Omotion.get_trj_sens_in().get_op_start(0),      1e-10);
    check("14 sens_in op 0 end    ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_sens_in().get_op_end(0),        1e-10);
    check("14 sens_in op 1 start  ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,    Omotion.get_trj_sens_in().get_op_start(1),      1e-10);
    check("14 sens_in op 1 end    ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_sens,        Omotion.get_trj_sens_in().get_op_end(1),        1e-10);

    check("14 sens_out nel        ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_sens),  Omotion.get_trj_sens_out().get_nel(),           1e-10);
    check("14 sens_out nel_op     ", nel_op,                                                  Omotion.get_trj_sens_out().get_nel_op(),        1e-10);
    check("14 sens_out last time  ", t_sec_end_op1,                                           Omotion.get_trj_sens_out()().back().get_t_sec(),1e-10);
    check("14 sens_out op 0 start ", 0,                                                       Omotion.get_trj_sens_out().get_op_start(0),     1e-10);
    check("14 sens_out op 0 end   ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_sens_out().get_op_end(0),       1e-10);
    check("14 sens_out op 1 start ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,    Omotion.get_trj_sens_out().get_op_start(1),     1e-10);
    check("14 sens_out op 1 end   ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_sens,        Omotion.get_trj_sens_out().get_op_end(1),       1e-10);

    check("14 cntr nel            ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_cntr),  Omotion.get_trj_cntr().get_nel(),               1e-10);
    check("14 cntr nel_op         ", nel_op,                                                  Omotion.get_trj_cntr().get_nel_op(),            1e-10);
    check("14 cntr last time      ", t_sec_end_op1,                                           Omotion.get_trj_cntr()().back().get_t_sec(),    1e-10);
    check("14 cntr op 0 start     ", 0,                                                       Omotion.get_trj_cntr().get_op_start(0),         1e-10);
    check("14 cntr op 0 end       ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_cntr,        Omotion.get_trj_cntr().get_op_end(0),           1e-10);
    check("14 cntr op 1 start     ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_cntr,    Omotion.get_trj_cntr().get_op_start(1),         1e-10);
    check("14 cntr op 1 end       ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_cntr,        Omotion.get_trj_cntr().get_op_end(1),           1e-10);

    check("14 nav_in nel          ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_sens),  Omotion.get_trj_nav_in().get_nel(),             1e-10);
    check("14 nav_in nel_op       ", nel_op,                                                  Omotion.get_trj_nav_in().get_nel_op(),          1e-10);
    check("14 nav_in last time    ", t_sec_end_op1,                                           Omotion.get_trj_nav_in()().back().get_t_sec(),  1e-10);
    check("14 nav_in op 0 start   ", 0,                                                       Omotion.get_trj_nav_in().get_op_start(0),       1e-10);
    check("14 nav_in op 0 end     ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_nav_in().get_op_end(0),         1e-10);
    check("14 nav_in op 1 start   ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,    Omotion.get_trj_nav_in().get_op_start(1),       1e-10);
    check("14 nav_in op 1 end     ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_sens,        Omotion.get_trj_nav_in().get_op_end(1),         1e-10);

    check("14 nav_out nel         ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_sens),  Omotion.get_trj_nav_out().get_nel(),            1e-10);
    check("14 nav_out nel_op      ", nel_op,                                                  Omotion.get_trj_nav_out().get_nel_op(),         1e-10);
    check("14 nav_out last time   ", t_sec_end_op1,                                           Omotion.get_trj_nav_out()().back().get_t_sec(), 1e-10);
    check("14 nav_out op 0 start  ", 0,                                                       Omotion.get_trj_nav_out().get_op_start(0),      1e-10);
    check("14 nav_out op 0 end    ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_nav_out().get_op_end(0),        1e-10);
    check("14 nav_out op 1 start  ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,    Omotion.get_trj_nav_out().get_op_start(1),      1e-10);
    check("14 nav_out op 1 end    ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_sens,        Omotion.get_trj_nav_out().get_op_end(1),        1e-10);

    check("14 gps_out nel         ", 1 + ((- t_sec_init + t_sec_gpsloss) / Deltat_sec_gps),   Omotion.get_trj_gps_out().get_nel(),            1e-10);
    check("14 gps_out nel_op      ", nel_op - 1,                                              Omotion.get_trj_gps_out().get_nel_op(),         1e-10);
    check("14 gps_out last time   ", t_sec_gpsloss,                                           Omotion.get_trj_gps_out()().back().get_t_sec(), 1e-10);
    check("14 gps_out op 0 start  ", 0,                                                       Omotion.get_trj_gps_out().get_op_start(0),      1e-10);
    check("14 gps_out op 0 end    ", (- t_sec_init + t_sec_gpsloss) / Deltat_sec_gps,         Omotion.get_trj_gps_out().get_op_end(0),        1e-10);

    check("14 out nel             ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_out),   Omotion.get_trj_out().get_nel(),                1e-10);
    check("14 out nel_op          ", nel_op,                                                  Omotion.get_trj_out().get_nel_op(),             1e-10);
    check("14 out last time       ", t_sec_end_op1,                                           Omotion.get_trj_out()().back().get_t_sec(),     1e-10);
    check("14 out op 0 start      ", 0,                                                       Omotion.get_trj_out().get_op_start(0),          1e-10);
    check("14 out op 0 end        ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_out,         Omotion.get_trj_out().get_op_end(0),            1e-10);
    check("14 out op 1 start      ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_out,     Omotion.get_trj_out().get_op_start(1),          1e-10);
    check("14 out op 1 end        ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_out,         Omotion.get_trj_out().get_op_end(1),            1e-10);

    check("14 filter air          ", 1 + (- t_sec_init  + t_sec_end_op1) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_air().get_size());
    check("14 filter att          ", 1 + (- t_sec_init  + t_sec_end_op1) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_att().get_size());
    check("14 filter gps fast     ", 1 + (- t_sec_init  + t_sec_gpsloss) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_gps().get_size_fast());
    check("14 filter gps slow     ", 1 + (- t_sec_init  + t_sec_gpsloss) / Deltat_sec_gps,    Omotion.get_filter_nav().get_filter_gps().get_size_slow());
    check("14 filter pos          ", 1 + (- t_sec_gpsloss + t_sec_end_op1) / Deltat_sec_sens, Omotion.get_filter_nav().get_filter_pos().get_size());
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void nav::test::Ttrj_sizes::test_sizes15() {
    unsigned short seed_order = 1;
    double turb_factor        = 1.0;

    double t_sec_init    = -5.;
    double t_sec_end_trj = 30.0;
    double t_sec_end_op0 = 10.0;
    double t_sec_end_op1 = 20.0;
    double t_sec_gpsloss = 10.0;
    int    t_sec_turb    = 1000;

    sens::logic::SENS_COMPLETE_ID sens_complete_id = sens::logic::sens_complete_bias_scale_par;
    nav::logic::NAV_ID nav_id                      = nav::logic::nav_id010202;

    env::logic::WIND_ID wind_id         = env::logic::wind_id00;
    env::logic::OFFSETS_ID offsets_id   = env::logic::offsets_id00;
    env::logic::MAG_ID env_mag_id       = env::logic::mag_default;
    env::logic::REALISM_ID realism_id   = env::logic::realism_grav_no_magn_no;

    nav::logic::INITEUL_ID initeul_id   = nav::logic::initeul_id_zero;
    nav::logic::INITACC_ID initacc_id   = nav::logic::initacc_id_zero;
    nav::logic::INITGYR_ID initgyr_id   = nav::logic::initgyr_id_zero;
    nav::logic::INITMAG_ID initmag_id   = nav::logic::initmag_id_zero;
    nav::logic::INITMGN_ID initmgn_id   = nav::logic::initmgn_id_zero;

    sens::logic::GYR_ID gyr_id          = sens::logic::gyr_id_base;
    sens::logic::ACC_ID acc_id          = sens::logic::acc_id_base;
    sens::logic::MAG_ID mag_id          = sens::logic::mag_id_base;
    sens::logic::OSP_ID osp_id          = sens::logic::osp_id_base;
    sens::logic::OAT_ID oat_id          = sens::logic::oat_id_base;
    sens::logic::TAS_ID tas_id          = sens::logic::tas_id_base;
    sens::logic::AOA_ID aoa_id          = sens::logic::aoa_id_base;
    sens::logic::AOS_ID aos_id          = sens::logic::aos_id_base;
    sens::logic::GPS_ID gps_id          = sens::logic::gps_id_base;
    sens::logic::BAND_ID band_id        = sens::logic::band_id_base;

    st::logic::STI_ID sti_id = st::logic::sti_Hp3000_tas30_chi00;
    unsigned short nel_op = 2;

    math::seeder Oseeder(seed_order);
    auto Paero                = new acft::aero0();
    auto Pprop                = new acft::prop();
    auto Piner                = new acft::iner();
    auto Pearth               = new env::earth(Oseeder, wind_id, offsets_id, env_mag_id, realism_id, turb_factor, t_sec_init, t_sec_turb);
    auto Psuite               = new sens::suite(Oseeder, acc_id, gyr_id, mag_id, osp_id, oat_id, tas_id, aoa_id, aos_id, gps_id, band_id);
    auto Perr0                = new nav::init_error(Oseeder, initeul_id, initacc_id, initgyr_id, initmag_id, initmgn_id);
    auto Pguid                = new control::guid(nel_op, t_sec_end_trj, Oseeder.provide_seed(math::seeder::seeder_guid),0);
    Pguid->get_sti_id() = sti_id;
    Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 0.0, rud_beta_deg, 0., trgg_t_sec, t_sec_end_op0));
    Pguid->add_op(1, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 0.0, rud_beta_deg, 0., trgg_t_sec, t_sec_end_op1));
    st::sti* Psti             = st::sti::create_sti(*Pguid, t_sec_init);
    auto Pcntr                = new control::cntr(*Pguid);
    auto             Pnav     = new nav::filter_nav(nav_id, *Psuite, *Pearth, turb_factor);
    ang::euler euler_bc(0.0, 0.0, 0.0);
    auto Pcam                 = new sens::camera(768, 1024, 49.2255, euler_bc);

    nav::motion Omotion(Psti, Pguid, Pcntr, Pnav, Pearth, Psuite, Perr0, Piner, Paero, Pprop, Pcam, sens_complete_id);
    Omotion.solve_2nd(t_sec_end_trj, t_sec_gpsloss);

    double Deltat_sec_truth = 0.002;
    double Deltat_sec_sens  = 0.01;
    double Deltat_sec_gps   = 1.0;
    double Deltat_sec_cntr  = 0.02;
    double Deltat_sec_out   = 0.1;

    check("15 truth nel           ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_truth), Omotion.get_trj_truth().get_nel(),              1e-10);
    check("15 truth nel_op        ", nel_op,                                                  Omotion.get_trj_truth().get_nel_op(),           1e-10);
    check("15 truth last time     ", t_sec_end_op1,                                           Omotion.get_trj_truth()().back().get_t_sec(),   1e-10);
    check("15 truth op 0 start    ", 0,                                                       Omotion.get_trj_truth().get_op_start(0),        1e-10);
    check("15 truth op 0 end      ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_truth,       Omotion.get_trj_truth().get_op_end(0),          1e-10);
    check("15 truth op 1 start    ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_truth,   Omotion.get_trj_truth().get_op_start(1),        1e-10);
    check("15 truth op 1 end      ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_truth,       Omotion.get_trj_truth().get_op_end(1),          1e-10);

    check("15 sens_in nel         ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_sens),  Omotion.get_trj_sens_in().get_nel(),            1e-10);
    check("15 sens_in nel_op      ", nel_op,                                                  Omotion.get_trj_sens_in().get_nel_op(),         1e-10);
    check("15 sens_in last time   ", t_sec_end_op1,                                           Omotion.get_trj_sens_in()().back().get_t_sec(), 1e-10);
    check("15 sens_in op 0 start  ", 0,                                                       Omotion.get_trj_sens_in().get_op_start(0),      1e-10);
    check("15 sens_in op 0 end    ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_sens_in().get_op_end(0),        1e-10);
    check("15 sens_in op 1 start  ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,    Omotion.get_trj_sens_in().get_op_start(1),      1e-10);
    check("15 sens_in op 1 end    ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_sens,        Omotion.get_trj_sens_in().get_op_end(1),        1e-10);

    check("15 sens_out nel        ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_sens),  Omotion.get_trj_sens_out().get_nel(),           1e-10);
    check("15 sens_out nel_op     ", nel_op,                                                  Omotion.get_trj_sens_out().get_nel_op(),        1e-10);
    check("15 sens_out last time  ", t_sec_end_op1,                                           Omotion.get_trj_sens_out()().back().get_t_sec(),1e-10);
    check("15 sens_out op 0 start ", 0,                                                       Omotion.get_trj_sens_out().get_op_start(0),     1e-10);
    check("15 sens_out op 0 end   ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_sens_out().get_op_end(0),       1e-10);
    check("15 sens_out op 1 start ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,    Omotion.get_trj_sens_out().get_op_start(1),     1e-10);
    check("15 sens_out op 1 end   ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_sens,        Omotion.get_trj_sens_out().get_op_end(1),       1e-10);

    check("15 cntr nel            ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_cntr),  Omotion.get_trj_cntr().get_nel(),               1e-10);
    check("15 cntr nel_op         ", nel_op,                                                  Omotion.get_trj_cntr().get_nel_op(),            1e-10);
    check("15 cntr last time      ", t_sec_end_op1,                                           Omotion.get_trj_cntr()().back().get_t_sec(),    1e-10);
    check("15 cntr op 0 start     ", 0,                                                       Omotion.get_trj_cntr().get_op_start(0),         1e-10);
    check("15 cntr op 0 end       ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_cntr,        Omotion.get_trj_cntr().get_op_end(0),           1e-10);
    check("15 cntr op 1 start     ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_cntr,    Omotion.get_trj_cntr().get_op_start(1),         1e-10);
    check("15 cntr op 1 end       ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_cntr,        Omotion.get_trj_cntr().get_op_end(1),           1e-10);

    check("15 nav_in nel          ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_sens),  Omotion.get_trj_nav_in().get_nel(),             1e-10);
    check("15 nav_in nel_op       ", nel_op,                                                  Omotion.get_trj_nav_in().get_nel_op(),          1e-10);
    check("15 nav_in last time    ", t_sec_end_op1,                                           Omotion.get_trj_nav_in()().back().get_t_sec(),  1e-10);
    check("15 nav_in op 0 start   ", 0,                                                       Omotion.get_trj_nav_in().get_op_start(0),       1e-10);
    check("15 nav_in op 0 end     ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_nav_in().get_op_end(0),         1e-10);
    check("15 nav_in op 1 start   ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,    Omotion.get_trj_nav_in().get_op_start(1),       1e-10);
    check("15 nav_in op 1 end     ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_sens,        Omotion.get_trj_nav_in().get_op_end(1),         1e-10);

    check("15 nav_out nel         ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_sens),  Omotion.get_trj_nav_out().get_nel(),            1e-10);
    check("15 nav_out nel_op      ", nel_op,                                                  Omotion.get_trj_nav_out().get_nel_op(),         1e-10);
    check("15 nav_out last time   ", t_sec_end_op1,                                           Omotion.get_trj_nav_out()().back().get_t_sec(), 1e-10);
    check("15 nav_out op 0 start  ", 0,                                                       Omotion.get_trj_nav_out().get_op_start(0),      1e-10);
    check("15 nav_out op 0 end    ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_nav_out().get_op_end(0),        1e-10);
    check("15 nav_out op 1 start  ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,    Omotion.get_trj_nav_out().get_op_start(1),      1e-10);
    check("15 nav_out op 1 end    ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_sens,        Omotion.get_trj_nav_out().get_op_end(1),        1e-10);

    check("15 gps_out nel         ", 1 + ((- t_sec_init + t_sec_gpsloss) / Deltat_sec_gps),   Omotion.get_trj_gps_out().get_nel(),            1e-10);
    check("15 gps_out nel_op      ", nel_op - 1,                                              Omotion.get_trj_gps_out().get_nel_op(),         1e-10);
    check("15 gps_out last time   ", t_sec_gpsloss,                                           Omotion.get_trj_gps_out()().back().get_t_sec(), 1e-10);
    check("15 gps_out op 0 start  ", 0,                                                       Omotion.get_trj_gps_out().get_op_start(0),      1e-10);
    check("15 gps_out op 0 end    ", (- t_sec_init + t_sec_gpsloss) / Deltat_sec_gps,         Omotion.get_trj_gps_out().get_op_end(0),        1e-10);

    check("15 out nel             ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_out),   Omotion.get_trj_out().get_nel(),                1e-10);
    check("15 out nel_op          ", nel_op,                                                  Omotion.get_trj_out().get_nel_op(),             1e-10);
    check("15 out last time       ", t_sec_end_op1,                                           Omotion.get_trj_out()().back().get_t_sec(),     1e-10);
    check("15 out op 0 start      ", 0,                                                       Omotion.get_trj_out().get_op_start(0),          1e-10);
    check("15 out op 0 end        ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_out,         Omotion.get_trj_out().get_op_end(0),            1e-10);
    check("15 out op 1 start      ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_out,     Omotion.get_trj_out().get_op_start(1),          1e-10);
    check("15 out op 1 end        ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_out,         Omotion.get_trj_out().get_op_end(1),            1e-10);

    check("15 filter air          ", 1 + (- t_sec_init  + t_sec_end_op1) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_air().get_size());
    check("15 filter att          ", 1 + (- t_sec_init  + t_sec_end_op1) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_att().get_size());
    check("15 filter gps fast     ", 1 + (- t_sec_init  + t_sec_gpsloss) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_gps().get_size_fast());
    check("15 filter gps slow     ", 1 + (- t_sec_init  + t_sec_gpsloss) / Deltat_sec_gps,    Omotion.get_filter_nav().get_filter_gps().get_size_slow());
    check("15 filter pos          ", 1 + (- t_sec_gpsloss + t_sec_end_op1) / Deltat_sec_sens, Omotion.get_filter_nav().get_filter_pos().get_size());
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void nav::test::Ttrj_sizes::test_sizes16() {
    unsigned short seed_order = 1;
    double turb_factor        = 1.0;

    double t_sec_init    = -5.;
    double t_sec_end_trj = 30.0;
    double t_sec_end_op0 = 10.0;
    double t_sec_end_op1 = 20.0;
    double t_sec_gpsloss = 15.0;
    int    t_sec_turb    = 1000;

    sens::logic::SENS_COMPLETE_ID sens_complete_id = sens::logic::sens_complete_bias_scale_par;
    nav::logic::NAV_ID nav_id                      = nav::logic::nav_id010202;

    env::logic::WIND_ID wind_id         = env::logic::wind_id00;
    env::logic::OFFSETS_ID offsets_id   = env::logic::offsets_id00;
    env::logic::MAG_ID env_mag_id       = env::logic::mag_default;
    env::logic::REALISM_ID realism_id   = env::logic::realism_grav_no_magn_no;

    nav::logic::INITEUL_ID initeul_id   = nav::logic::initeul_id_zero;
    nav::logic::INITACC_ID initacc_id   = nav::logic::initacc_id_zero;
    nav::logic::INITGYR_ID initgyr_id   = nav::logic::initgyr_id_zero;
    nav::logic::INITMAG_ID initmag_id   = nav::logic::initmag_id_zero;
    nav::logic::INITMGN_ID initmgn_id   = nav::logic::initmgn_id_zero;

    sens::logic::GYR_ID gyr_id          = sens::logic::gyr_id_base;
    sens::logic::ACC_ID acc_id          = sens::logic::acc_id_base;
    sens::logic::MAG_ID mag_id          = sens::logic::mag_id_base;
    sens::logic::OSP_ID osp_id          = sens::logic::osp_id_base;
    sens::logic::OAT_ID oat_id          = sens::logic::oat_id_base;
    sens::logic::TAS_ID tas_id          = sens::logic::tas_id_base;
    sens::logic::AOA_ID aoa_id          = sens::logic::aoa_id_base;
    sens::logic::AOS_ID aos_id          = sens::logic::aos_id_base;
    sens::logic::GPS_ID gps_id          = sens::logic::gps_id_base;
    sens::logic::BAND_ID band_id        = sens::logic::band_id_base;

    st::logic::STI_ID sti_id = st::logic::sti_Hp3000_tas30_chi00;
    unsigned short nel_op = 2;

    math::seeder Oseeder(seed_order);
    auto Paero                = new acft::aero0();
    auto Pprop                = new acft::prop();
    auto Piner                = new acft::iner();
    auto Pearth               = new env::earth(Oseeder, wind_id, offsets_id, env_mag_id, realism_id, turb_factor, t_sec_init, t_sec_turb);
    auto Psuite               = new sens::suite(Oseeder, acc_id, gyr_id, mag_id, osp_id, oat_id, tas_id, aoa_id, aos_id, gps_id, band_id);
    auto Perr0                = new nav::init_error(Oseeder, initeul_id, initacc_id, initgyr_id, initmag_id, initmgn_id);
    auto Pguid                = new control::guid(nel_op, t_sec_end_trj, Oseeder.provide_seed(math::seeder::seeder_guid),0);
    Pguid->get_sti_id() = sti_id;
    Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 0.0, rud_beta_deg, 0., trgg_t_sec, t_sec_end_op0));
    Pguid->add_op(1, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 0.0, rud_beta_deg, 0., trgg_t_sec, t_sec_end_op1));
    st::sti* Psti             = st::sti::create_sti(*Pguid, t_sec_init);
    auto Pcntr                = new control::cntr(*Pguid);
    auto             Pnav     = new nav::filter_nav(nav_id, *Psuite, *Pearth, turb_factor);
    ang::euler euler_bc(0.0, 0.0, 0.0);
    auto Pcam                 = new sens::camera(768, 1024, 49.2255, euler_bc);

    nav::motion Omotion(Psti, Pguid, Pcntr, Pnav, Pearth, Psuite, Perr0, Piner, Paero, Pprop, Pcam, sens_complete_id);
    Omotion.solve_2nd(t_sec_end_trj, t_sec_gpsloss);

    double Deltat_sec_truth = 0.002;
    double Deltat_sec_sens  = 0.01;
    double Deltat_sec_gps   = 1.0;
    double Deltat_sec_cntr  = 0.02;
    double Deltat_sec_out   = 0.1;

    check("16 truth nel           ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_truth), Omotion.get_trj_truth().get_nel(),              1e-10);
    check("16 truth nel_op        ", nel_op,                                                  Omotion.get_trj_truth().get_nel_op(),           1e-10);
    check("16 truth last time     ", t_sec_end_op1,                                           Omotion.get_trj_truth()().back().get_t_sec(),   1e-10);
    check("16 truth op 0 start    ", 0,                                                       Omotion.get_trj_truth().get_op_start(0),        1e-10);
    check("16 truth op 0 end      ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_truth,       Omotion.get_trj_truth().get_op_end(0),          1e-10);
    check("16 truth op 1 start    ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_truth,   Omotion.get_trj_truth().get_op_start(1),        1e-10);
    check("16 truth op 1 end      ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_truth,       Omotion.get_trj_truth().get_op_end(1),          1e-10);

    check("16 sens_in nel         ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_sens),  Omotion.get_trj_sens_in().get_nel(),            1e-10);
    check("16 sens_in nel_op      ", nel_op,                                                  Omotion.get_trj_sens_in().get_nel_op(),         1e-10);
    check("16 sens_in last time   ", t_sec_end_op1,                                           Omotion.get_trj_sens_in()().back().get_t_sec(), 1e-10);
    check("16 sens_in op 0 start  ", 0,                                                       Omotion.get_trj_sens_in().get_op_start(0),      1e-10);
    check("16 sens_in op 0 end    ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_sens_in().get_op_end(0),        1e-10);
    check("16 sens_in op 1 start  ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,    Omotion.get_trj_sens_in().get_op_start(1),      1e-10);
    check("16 sens_in op 1 end    ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_sens,        Omotion.get_trj_sens_in().get_op_end(1),        1e-10);

    check("16 sens_out nel        ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_sens),  Omotion.get_trj_sens_out().get_nel(),           1e-10);
    check("16 sens_out nel_op     ", nel_op,                                                  Omotion.get_trj_sens_out().get_nel_op(),        1e-10);
    check("16 sens_out last time  ", t_sec_end_op1,                                           Omotion.get_trj_sens_out()().back().get_t_sec(),1e-10);
    check("16 sens_out op 0 start ", 0,                                                       Omotion.get_trj_sens_out().get_op_start(0),     1e-10);
    check("16 sens_out op 0 end   ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_sens_out().get_op_end(0),       1e-10);
    check("16 sens_out op 1 start ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,    Omotion.get_trj_sens_out().get_op_start(1),     1e-10);
    check("16 sens_out op 1 end   ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_sens,        Omotion.get_trj_sens_out().get_op_end(1),       1e-10);

    check("16 cntr nel            ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_cntr),  Omotion.get_trj_cntr().get_nel(),               1e-10);
    check("16 cntr nel_op         ", nel_op,                                                  Omotion.get_trj_cntr().get_nel_op(),            1e-10);
    check("16 cntr last time      ", t_sec_end_op1,                                           Omotion.get_trj_cntr()().back().get_t_sec(),    1e-10);
    check("16 cntr op 0 start     ", 0,                                                       Omotion.get_trj_cntr().get_op_start(0),         1e-10);
    check("16 cntr op 0 end       ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_cntr,        Omotion.get_trj_cntr().get_op_end(0),           1e-10);
    check("16 cntr op 1 start     ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_cntr,    Omotion.get_trj_cntr().get_op_start(1),         1e-10);
    check("16 cntr op 1 end       ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_cntr,        Omotion.get_trj_cntr().get_op_end(1),           1e-10);

    check("16 nav_in nel          ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_sens),  Omotion.get_trj_nav_in().get_nel(),             1e-10);
    check("16 nav_in nel_op       ", nel_op,                                                  Omotion.get_trj_nav_in().get_nel_op(),          1e-10);
    check("16 nav_in last time    ", t_sec_end_op1,                                           Omotion.get_trj_nav_in()().back().get_t_sec(),  1e-10);
    check("16 nav_in op 0 start   ", 0,                                                       Omotion.get_trj_nav_in().get_op_start(0),       1e-10);
    check("16 nav_in op 0 end     ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_nav_in().get_op_end(0),         1e-10);
    check("16 nav_in op 1 start   ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,    Omotion.get_trj_nav_in().get_op_start(1),       1e-10);
    check("16 nav_in op 1 end     ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_sens,        Omotion.get_trj_nav_in().get_op_end(1),         1e-10);

    check("16 nav_out nel         ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_sens),  Omotion.get_trj_nav_out().get_nel(),            1e-10);
    check("16 nav_out nel_op      ", nel_op,                                                  Omotion.get_trj_nav_out().get_nel_op(),         1e-10);
    check("16 nav_out last time   ", t_sec_end_op1,                                           Omotion.get_trj_nav_out()().back().get_t_sec(), 1e-10);
    check("16 nav_out op 0 start  ", 0,                                                       Omotion.get_trj_nav_out().get_op_start(0),      1e-10);
    check("16 nav_out op 0 end    ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_nav_out().get_op_end(0),        1e-10);
    check("16 nav_out op 1 start  ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,    Omotion.get_trj_nav_out().get_op_start(1),      1e-10);
    check("16 nav_out op 1 end    ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_sens,        Omotion.get_trj_nav_out().get_op_end(1),        1e-10);

    check("16 gps_out nel         ", 1 + ((- t_sec_init + t_sec_gpsloss) / Deltat_sec_gps),   Omotion.get_trj_gps_out().get_nel(),            1e-10);
    check("16 gps_out nel_op      ", nel_op,                                                  Omotion.get_trj_gps_out().get_nel_op(),         1e-10);
    check("16 gps_out last time   ", t_sec_gpsloss,                                           Omotion.get_trj_gps_out()().back().get_t_sec(), 1e-10);
    check("16 gps_out op 0 start  ", 0,                                                       Omotion.get_trj_gps_out().get_op_start(0),      1e-10);
    check("16 gps_out op 0 end    ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_gps,         Omotion.get_trj_gps_out().get_op_end(0),        1e-10);
    check("16 gps_out op 1 start  ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_gps,     Omotion.get_trj_gps_out().get_op_start(1),      1e-10);
    check("16 gps_out op 1 end    ", (- t_sec_init + t_sec_gpsloss) / Deltat_sec_gps,         Omotion.get_trj_gps_out().get_op_end(1),        1e-10);

    check("16 out nel             ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_out),   Omotion.get_trj_out().get_nel(),                1e-10);
    check("16 out nel_op          ", nel_op,                                                  Omotion.get_trj_out().get_nel_op(),             1e-10);
    check("16 out last time       ", t_sec_end_op1,                                           Omotion.get_trj_out()().back().get_t_sec(),     1e-10);
    check("16 out op 0 start      ", 0,                                                       Omotion.get_trj_out().get_op_start(0),          1e-10);
    check("16 out op 0 end        ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_out,         Omotion.get_trj_out().get_op_end(0),            1e-10);
    check("16 out op 1 start      ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_out,     Omotion.get_trj_out().get_op_start(1),          1e-10);
    check("16 out op 1 end        ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_out,         Omotion.get_trj_out().get_op_end(1),            1e-10);

    check("16 filter air          ", 1 + (- t_sec_init  + t_sec_end_op1) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_air().get_size());
    check("16 filter att          ", 1 + (- t_sec_init  + t_sec_end_op1) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_att().get_size());
    check("16 filter gps fast     ", 1 + (- t_sec_init  + t_sec_gpsloss) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_gps().get_size_fast());
    check("16 filter gps slow     ", 1 + (- t_sec_init  + t_sec_gpsloss) / Deltat_sec_gps,    Omotion.get_filter_nav().get_filter_gps().get_size_slow());
    check("16 filter pos          ", 1 + (- t_sec_gpsloss + t_sec_end_op1) / Deltat_sec_sens, Omotion.get_filter_nav().get_filter_pos().get_size());
}

 /////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void nav::test::Ttrj_sizes::test_sizes17() {
    unsigned short seed_order = 1;
    double turb_factor        = 1.0;

    double t_sec_init    = -5.;
    double t_sec_end_trj = 30.0;
    double t_sec_end_op0 = 10.0;
    double t_sec_end_op1 = 20.0;
    double t_sec_gpsloss = 20.0;
    int    t_sec_turb    = 1000;

    sens::logic::SENS_COMPLETE_ID sens_complete_id = sens::logic::sens_complete_bias_scale_par;
    nav::logic::NAV_ID nav_id                      = nav::logic::nav_id010202;

    env::logic::WIND_ID wind_id         = env::logic::wind_id00;
    env::logic::OFFSETS_ID offsets_id   = env::logic::offsets_id00;
    env::logic::MAG_ID env_mag_id       = env::logic::mag_default;
    env::logic::REALISM_ID realism_id   = env::logic::realism_grav_no_magn_no;

    nav::logic::INITEUL_ID initeul_id   = nav::logic::initeul_id_zero;
    nav::logic::INITACC_ID initacc_id   = nav::logic::initacc_id_zero;
    nav::logic::INITGYR_ID initgyr_id   = nav::logic::initgyr_id_zero;
    nav::logic::INITMAG_ID initmag_id   = nav::logic::initmag_id_zero;
    nav::logic::INITMGN_ID initmgn_id   = nav::logic::initmgn_id_zero;

    sens::logic::GYR_ID gyr_id          = sens::logic::gyr_id_base;
    sens::logic::ACC_ID acc_id          = sens::logic::acc_id_base;
    sens::logic::MAG_ID mag_id          = sens::logic::mag_id_base;
    sens::logic::OSP_ID osp_id          = sens::logic::osp_id_base;
    sens::logic::OAT_ID oat_id          = sens::logic::oat_id_base;
    sens::logic::TAS_ID tas_id          = sens::logic::tas_id_base;
    sens::logic::AOA_ID aoa_id          = sens::logic::aoa_id_base;
    sens::logic::AOS_ID aos_id          = sens::logic::aos_id_base;
    sens::logic::GPS_ID gps_id          = sens::logic::gps_id_base;
    sens::logic::BAND_ID band_id        = sens::logic::band_id_base;

    st::logic::STI_ID sti_id = st::logic::sti_Hp3000_tas30_chi00;
    unsigned short nel_op = 2;

    math::seeder Oseeder(seed_order);
    auto Paero                = new acft::aero0();
    auto Pprop                = new acft::prop();
    auto Piner                = new acft::iner();
    auto Pearth               = new env::earth(Oseeder, wind_id, offsets_id, env_mag_id, realism_id, turb_factor, t_sec_init, t_sec_turb);
    auto Psuite               = new sens::suite(Oseeder, acc_id, gyr_id, mag_id, osp_id, oat_id, tas_id, aoa_id, aos_id, gps_id, band_id);
    auto Perr0                = new nav::init_error(Oseeder, initeul_id, initacc_id, initgyr_id, initmag_id, initmgn_id);
    auto Pguid                = new control::guid(nel_op, t_sec_end_trj, Oseeder.provide_seed(math::seeder::seeder_guid),0);
    Pguid->get_sti_id() = sti_id;
    Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 0.0, rud_beta_deg, 0., trgg_t_sec, t_sec_end_op0));
    Pguid->add_op(1, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 0.0, rud_beta_deg, 0., trgg_t_sec, t_sec_end_op1));
    st::sti* Psti             = st::sti::create_sti(*Pguid, t_sec_init);
    auto Pcntr                = new control::cntr(*Pguid);
    auto             Pnav     = new nav::filter_nav(nav_id, *Psuite, *Pearth, turb_factor);
    ang::euler euler_bc(0.0, 0.0, 0.0);
    auto Pcam                 = new sens::camera(768, 1024, 49.2255, euler_bc);

    nav::motion Omotion(Psti, Pguid, Pcntr, Pnav, Pearth, Psuite, Perr0, Piner, Paero, Pprop, Pcam, sens_complete_id);
    Omotion.solve_2nd(t_sec_end_trj, t_sec_gpsloss);

    double Deltat_sec_truth = 0.002;
    double Deltat_sec_sens  = 0.01;
    double Deltat_sec_gps   = 1.0;
    double Deltat_sec_cntr  = 0.02;
    double Deltat_sec_out   = 0.1;

    check("17 truth nel           ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_truth), Omotion.get_trj_truth().get_nel(),              1e-10);
    check("17 truth nel_op        ", nel_op,                                                  Omotion.get_trj_truth().get_nel_op(),           1e-10);
    check("17 truth last time     ", t_sec_end_op1,                                           Omotion.get_trj_truth()().back().get_t_sec(),   1e-10);
    check("17 truth op 0 start    ", 0,                                                       Omotion.get_trj_truth().get_op_start(0),        1e-10);
    check("17 truth op 0 end      ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_truth,       Omotion.get_trj_truth().get_op_end(0),          1e-10);
    check("17 truth op 1 start    ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_truth,   Omotion.get_trj_truth().get_op_start(1),        1e-10);
    check("17 truth op 1 end      ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_truth,       Omotion.get_trj_truth().get_op_end(1),          1e-10);

    check("17 sens_in nel         ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_sens),  Omotion.get_trj_sens_in().get_nel(),            1e-10);
    check("17 sens_in nel_op      ", nel_op,                                                  Omotion.get_trj_sens_in().get_nel_op(),         1e-10);
    check("17 sens_in last time   ", t_sec_end_op1,                                           Omotion.get_trj_sens_in()().back().get_t_sec(), 1e-10);
    check("17 sens_in op 0 start  ", 0,                                                       Omotion.get_trj_sens_in().get_op_start(0),      1e-10);
    check("17 sens_in op 0 end    ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_sens_in().get_op_end(0),        1e-10);
    check("17 sens_in op 1 start  ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,    Omotion.get_trj_sens_in().get_op_start(1),      1e-10);
    check("17 sens_in op 1 end    ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_sens,        Omotion.get_trj_sens_in().get_op_end(1),        1e-10);

    check("17 sens_out nel        ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_sens),  Omotion.get_trj_sens_out().get_nel(),           1e-10);
    check("17 sens_out nel_op     ", nel_op,                                                  Omotion.get_trj_sens_out().get_nel_op(),        1e-10);
    check("17 sens_out last time  ", t_sec_end_op1,                                           Omotion.get_trj_sens_out()().back().get_t_sec(),1e-10);
    check("17 sens_out op 0 start ", 0,                                                       Omotion.get_trj_sens_out().get_op_start(0),     1e-10);
    check("17 sens_out op 0 end   ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_sens_out().get_op_end(0),       1e-10);
    check("17 sens_out op 1 start ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,    Omotion.get_trj_sens_out().get_op_start(1),     1e-10);
    check("17 sens_out op 1 end   ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_sens,        Omotion.get_trj_sens_out().get_op_end(1),       1e-10);

    check("17 cntr nel            ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_cntr),  Omotion.get_trj_cntr().get_nel(),               1e-10);
    check("17 cntr nel_op         ", nel_op,                                                  Omotion.get_trj_cntr().get_nel_op(),            1e-10);
    check("17 cntr last time      ", t_sec_end_op1,                                           Omotion.get_trj_cntr()().back().get_t_sec(),    1e-10);
    check("17 cntr op 0 start     ", 0,                                                       Omotion.get_trj_cntr().get_op_start(0),         1e-10);
    check("17 cntr op 0 end       ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_cntr,        Omotion.get_trj_cntr().get_op_end(0),           1e-10);
    check("17 cntr op 1 start     ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_cntr,    Omotion.get_trj_cntr().get_op_start(1),         1e-10);
    check("17 cntr op 1 end       ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_cntr,        Omotion.get_trj_cntr().get_op_end(1),           1e-10);

    check("17 nav_in nel          ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_sens),  Omotion.get_trj_nav_in().get_nel(),             1e-10);
    check("17 nav_in nel_op       ", nel_op,                                                  Omotion.get_trj_nav_in().get_nel_op(),          1e-10);
    check("17 nav_in last time    ", t_sec_end_op1,                                           Omotion.get_trj_nav_in()().back().get_t_sec(),  1e-10);
    check("17 nav_in op 0 start   ", 0,                                                       Omotion.get_trj_nav_in().get_op_start(0),       1e-10);
    check("17 nav_in op 0 end     ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_nav_in().get_op_end(0),         1e-10);
    check("17 nav_in op 1 start   ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,    Omotion.get_trj_nav_in().get_op_start(1),       1e-10);
    check("17 nav_in op 1 end     ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_sens,        Omotion.get_trj_nav_in().get_op_end(1),         1e-10);

    check("17 nav_out nel         ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_sens),  Omotion.get_trj_nav_out().get_nel(),            1e-10);
    check("17 nav_out nel_op      ", nel_op,                                                  Omotion.get_trj_nav_out().get_nel_op(),         1e-10);
    check("17 nav_out last time   ", t_sec_end_op1,                                           Omotion.get_trj_nav_out()().back().get_t_sec(), 1e-10);
    check("17 nav_out op 0 start  ", 0,                                                       Omotion.get_trj_nav_out().get_op_start(0),      1e-10);
    check("17 nav_out op 0 end    ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_nav_out().get_op_end(0),        1e-10);
    check("17 nav_out op 1 start  ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,    Omotion.get_trj_nav_out().get_op_start(1),      1e-10);
    check("17 nav_out op 1 end    ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_sens,        Omotion.get_trj_nav_out().get_op_end(1),        1e-10);

    check("17 gps_out nel         ", 1 + ((- t_sec_init + t_sec_gpsloss) / Deltat_sec_gps),   Omotion.get_trj_gps_out().get_nel(),            1e-10);
    check("17 gps_out nel_op      ", nel_op,                                                  Omotion.get_trj_gps_out().get_nel_op(),         1e-10);
    check("17 gps_out last time   ", t_sec_gpsloss,                                           Omotion.get_trj_gps_out()().back().get_t_sec(), 1e-10);
    check("17 gps_out op 0 start  ", 0,                                                       Omotion.get_trj_gps_out().get_op_start(0),      1e-10);
    check("17 gps_out op 0 end    ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_gps,         Omotion.get_trj_gps_out().get_op_end(0),        1e-10);
    check("17 gps_out op 1 start  ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_gps,     Omotion.get_trj_gps_out().get_op_start(1),      1e-10);
    check("17 gps_out op 1 end    ", (- t_sec_init + t_sec_gpsloss) / Deltat_sec_gps,         Omotion.get_trj_gps_out().get_op_end(1),        1e-10);

    check("17 out nel             ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_out),   Omotion.get_trj_out().get_nel(),                1e-10);
    check("17 out nel_op          ", nel_op,                                                  Omotion.get_trj_out().get_nel_op(),             1e-10);
    check("17 out last time       ", t_sec_end_op1,                                           Omotion.get_trj_out()().back().get_t_sec(),     1e-10);
    check("17 out op 0 start      ", 0,                                                       Omotion.get_trj_out().get_op_start(0),          1e-10);
    check("17 out op 0 end        ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_out,         Omotion.get_trj_out().get_op_end(0),            1e-10);
    check("17 out op 1 start      ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_out,     Omotion.get_trj_out().get_op_start(1),          1e-10);
    check("17 out op 1 end        ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_out,         Omotion.get_trj_out().get_op_end(1),            1e-10);

    check("17 filter air          ", 1 + (- t_sec_init  + t_sec_end_op1) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_air().get_size());
    check("17 filter att          ", 1 + (- t_sec_init  + t_sec_end_op1) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_att().get_size());
    check("17 filter gps fast     ", 1 + (- t_sec_init  + t_sec_gpsloss) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_gps().get_size_fast());
    check("17 filter gps slow     ", 1 + (- t_sec_init  + t_sec_gpsloss) / Deltat_sec_gps,    Omotion.get_filter_nav().get_filter_gps().get_size_slow());
    check("17 filter pos          ", 1 + (- t_sec_gpsloss + t_sec_end_op1) / Deltat_sec_sens, Omotion.get_filter_nav().get_filter_pos().get_size());
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void nav::test::Ttrj_sizes::test_sizes18() {
    unsigned short seed_order = 1;
    double turb_factor        = 1.0;

    double t_sec_init    = -5.;
    double t_sec_end_trj = 30.0;
    double t_sec_end_op0 = 10.0;
    double t_sec_end_op1 = 20.0;
    double t_sec_gpsloss = 25.0;
    int    t_sec_turb    = 1000;

    sens::logic::SENS_COMPLETE_ID sens_complete_id = sens::logic::sens_complete_bias_scale_par;
    nav::logic::NAV_ID nav_id                      = nav::logic::nav_id010202;

    env::logic::WIND_ID wind_id         = env::logic::wind_id00;
    env::logic::OFFSETS_ID offsets_id   = env::logic::offsets_id00;
    env::logic::MAG_ID env_mag_id       = env::logic::mag_default;
    env::logic::REALISM_ID realism_id   = env::logic::realism_grav_no_magn_no;

    nav::logic::INITEUL_ID initeul_id   = nav::logic::initeul_id_zero;
    nav::logic::INITACC_ID initacc_id   = nav::logic::initacc_id_zero;
    nav::logic::INITGYR_ID initgyr_id   = nav::logic::initgyr_id_zero;
    nav::logic::INITMAG_ID initmag_id   = nav::logic::initmag_id_zero;
    nav::logic::INITMGN_ID initmgn_id   = nav::logic::initmgn_id_zero;

    sens::logic::GYR_ID gyr_id          = sens::logic::gyr_id_base;
    sens::logic::ACC_ID acc_id          = sens::logic::acc_id_base;
    sens::logic::MAG_ID mag_id          = sens::logic::mag_id_base;
    sens::logic::OSP_ID osp_id          = sens::logic::osp_id_base;
    sens::logic::OAT_ID oat_id          = sens::logic::oat_id_base;
    sens::logic::TAS_ID tas_id          = sens::logic::tas_id_base;
    sens::logic::AOA_ID aoa_id          = sens::logic::aoa_id_base;
    sens::logic::AOS_ID aos_id          = sens::logic::aos_id_base;
    sens::logic::GPS_ID gps_id          = sens::logic::gps_id_base;
    sens::logic::BAND_ID band_id        = sens::logic::band_id_base;

    st::logic::STI_ID sti_id = st::logic::sti_Hp3000_tas30_chi00;
    unsigned short nel_op = 2;

    math::seeder Oseeder(seed_order);
    auto Paero                = new acft::aero0();
    auto Pprop                = new acft::prop();
    auto Piner                = new acft::iner();
    auto Pearth               = new env::earth(Oseeder, wind_id, offsets_id, env_mag_id, realism_id, turb_factor, t_sec_init, t_sec_turb);
    auto Psuite               = new sens::suite(Oseeder, acc_id, gyr_id, mag_id, osp_id, oat_id, tas_id, aoa_id, aos_id, gps_id, band_id);
    auto Perr0                = new nav::init_error(Oseeder, initeul_id, initacc_id, initgyr_id, initmag_id, initmgn_id);
    auto Pguid                = new control::guid(nel_op, t_sec_end_trj, Oseeder.provide_seed(math::seeder::seeder_guid),0);
    Pguid->get_sti_id() = sti_id;
    Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 0.0, rud_beta_deg, 0., trgg_t_sec, t_sec_end_op0));
    Pguid->add_op(1, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 0.0, rud_beta_deg, 0., trgg_t_sec, t_sec_end_op1));
    st::sti* Psti             = st::sti::create_sti(*Pguid, t_sec_init);
    auto Pcntr                = new control::cntr(*Pguid);
    auto             Pnav     = new nav::filter_nav(nav_id, *Psuite, *Pearth, turb_factor);
    ang::euler euler_bc(0.0, 0.0, 0.0);
    auto Pcam                 = new sens::camera(768, 1024, 49.2255, euler_bc);

    nav::motion Omotion(Psti, Pguid, Pcntr, Pnav, Pearth, Psuite, Perr0, Piner, Paero, Pprop, Pcam, sens_complete_id);
    Omotion.solve_2nd(t_sec_end_trj, t_sec_gpsloss);

    double Deltat_sec_truth = 0.002;
    double Deltat_sec_sens  = 0.01;
    double Deltat_sec_gps   = 1.0;
    double Deltat_sec_cntr  = 0.02;
    double Deltat_sec_out   = 0.1;

    check("18 truth nel           ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_truth), Omotion.get_trj_truth().get_nel(),              1e-10);
    check("18 truth nel_op        ", nel_op,                                                  Omotion.get_trj_truth().get_nel_op(),           1e-10);
    check("18 truth last time     ", t_sec_end_op1,                                           Omotion.get_trj_truth()().back().get_t_sec(),   1e-10);
    check("18 truth op 0 start    ", 0,                                                       Omotion.get_trj_truth().get_op_start(0),        1e-10);
    check("18 truth op 0 end      ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_truth,       Omotion.get_trj_truth().get_op_end(0),          1e-10);
    check("18 truth op 1 start    ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_truth,   Omotion.get_trj_truth().get_op_start(1),        1e-10);
    check("18 truth op 1 end      ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_truth,       Omotion.get_trj_truth().get_op_end(1),          1e-10);

    check("18 sens_in nel         ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_sens),  Omotion.get_trj_sens_in().get_nel(),            1e-10);
    check("18 sens_in nel_op      ", nel_op,                                                  Omotion.get_trj_sens_in().get_nel_op(),         1e-10);
    check("18 sens_in last time   ", t_sec_end_op1,                                           Omotion.get_trj_sens_in()().back().get_t_sec(), 1e-10);
    check("18 sens_in op 0 start  ", 0,                                                       Omotion.get_trj_sens_in().get_op_start(0),      1e-10);
    check("18 sens_in op 0 end    ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_sens_in().get_op_end(0),        1e-10);
    check("18 sens_in op 1 start  ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,    Omotion.get_trj_sens_in().get_op_start(1),      1e-10);
    check("18 sens_in op 1 end    ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_sens,        Omotion.get_trj_sens_in().get_op_end(1),        1e-10);

    check("18 sens_out nel        ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_sens),  Omotion.get_trj_sens_out().get_nel(),           1e-10);
    check("18 sens_out nel_op     ", nel_op,                                                  Omotion.get_trj_sens_out().get_nel_op(),        1e-10);
    check("18 sens_out last time  ", t_sec_end_op1,                                           Omotion.get_trj_sens_out()().back().get_t_sec(),1e-10);
    check("18 sens_out op 0 start ", 0,                                                       Omotion.get_trj_sens_out().get_op_start(0),     1e-10);
    check("18 sens_out op 0 end   ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_sens_out().get_op_end(0),       1e-10);
    check("18 sens_out op 1 start ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,    Omotion.get_trj_sens_out().get_op_start(1),     1e-10);
    check("18 sens_out op 1 end   ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_sens,        Omotion.get_trj_sens_out().get_op_end(1),       1e-10);

    check("18 cntr nel            ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_cntr),  Omotion.get_trj_cntr().get_nel(),               1e-10);
    check("18 cntr nel_op         ", nel_op,                                                  Omotion.get_trj_cntr().get_nel_op(),            1e-10);
    check("18 cntr last time      ", t_sec_end_op1,                                           Omotion.get_trj_cntr()().back().get_t_sec(),    1e-10);
    check("18 cntr op 0 start     ", 0,                                                       Omotion.get_trj_cntr().get_op_start(0),         1e-10);
    check("18 cntr op 0 end       ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_cntr,        Omotion.get_trj_cntr().get_op_end(0),           1e-10);
    check("18 cntr op 1 start     ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_cntr,    Omotion.get_trj_cntr().get_op_start(1),         1e-10);
    check("18 cntr op 1 end       ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_cntr,        Omotion.get_trj_cntr().get_op_end(1),           1e-10);

    check("18 nav_in nel          ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_sens),  Omotion.get_trj_nav_in().get_nel(),             1e-10);
    check("18 nav_in nel_op       ", nel_op,                                                  Omotion.get_trj_nav_in().get_nel_op(),          1e-10);
    check("18 nav_in last time    ", t_sec_end_op1,                                           Omotion.get_trj_nav_in()().back().get_t_sec(),  1e-10);
    check("18 nav_in op 0 start   ", 0,                                                       Omotion.get_trj_nav_in().get_op_start(0),       1e-10);
    check("18 nav_in op 0 end     ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_nav_in().get_op_end(0),         1e-10);
    check("18 nav_in op 1 start   ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,    Omotion.get_trj_nav_in().get_op_start(1),       1e-10);
    check("18 nav_in op 1 end     ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_sens,        Omotion.get_trj_nav_in().get_op_end(1),         1e-10);

    check("18 nav_out nel         ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_sens),  Omotion.get_trj_nav_out().get_nel(),            1e-10);
    check("18 nav_out nel_op      ", nel_op,                                                  Omotion.get_trj_nav_out().get_nel_op(),         1e-10);
    check("18 nav_out last time   ", t_sec_end_op1,                                           Omotion.get_trj_nav_out()().back().get_t_sec(), 1e-10);
    check("18 nav_out op 0 start  ", 0,                                                       Omotion.get_trj_nav_out().get_op_start(0),      1e-10);
    check("18 nav_out op 0 end    ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,        Omotion.get_trj_nav_out().get_op_end(0),        1e-10);
    check("18 nav_out op 1 start  ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_sens,    Omotion.get_trj_nav_out().get_op_start(1),      1e-10);
    check("18 nav_out op 1 end    ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_sens,        Omotion.get_trj_nav_out().get_op_end(1),        1e-10);

    check("18 gps_out nel         ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_gps),   Omotion.get_trj_gps_out().get_nel(),            1e-10);
    check("18 gps_out nel_op      ", nel_op,                                                  Omotion.get_trj_gps_out().get_nel_op(),         1e-10);
    check("18 gps_out last time   ", t_sec_end_op1,                                           Omotion.get_trj_gps_out()().back().get_t_sec(), 1e-10);
    check("18 gps_out op 0 start  ", 0,                                                       Omotion.get_trj_gps_out().get_op_start(0),      1e-10);
    check("18 gps_out op 0 end    ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_gps,         Omotion.get_trj_gps_out().get_op_end(0),        1e-10);
    check("18 gps_out op 1 start  ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_gps,     Omotion.get_trj_gps_out().get_op_start(1),      1e-10);
    check("18 gps_out op 1 end    ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_gps,         Omotion.get_trj_gps_out().get_op_end(1),        1e-10);

    check("18 out nel             ", 1 + ((- t_sec_init + t_sec_end_op1) / Deltat_sec_out),   Omotion.get_trj_out().get_nel(),                1e-10);
    check("18 out nel_op          ", nel_op,                                                  Omotion.get_trj_out().get_nel_op(),             1e-10);
    check("18 out last time       ", t_sec_end_op1,                                           Omotion.get_trj_out()().back().get_t_sec(),     1e-10);
    check("18 out op 0 start      ", 0,                                                       Omotion.get_trj_out().get_op_start(0),          1e-10);
    check("18 out op 0 end        ", (- t_sec_init + t_sec_end_op0) / Deltat_sec_out,         Omotion.get_trj_out().get_op_end(0),            1e-10);
    check("18 out op 1 start      ", 1 + (- t_sec_init + t_sec_end_op0) / Deltat_sec_out,     Omotion.get_trj_out().get_op_start(1),          1e-10);
    check("18 out op 1 end        ", (- t_sec_init + t_sec_end_op1) / Deltat_sec_out,         Omotion.get_trj_out().get_op_end(1),            1e-10);

    check("18 filter air          ", 1 + (- t_sec_init  + t_sec_end_op1) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_air().get_size());
    check("18 filter att          ", 1 + (- t_sec_init  + t_sec_end_op1) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_att().get_size());
    check("18 filter gps fast     ", 1 + (- t_sec_init  + t_sec_end_op1) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_gps().get_size_fast());
    check("18 filter gps slow     ", 1 + (- t_sec_init  + t_sec_end_op1) / Deltat_sec_gps,    Omotion.get_filter_nav().get_filter_gps().get_size_slow());
    check("18 filter pos          ", 1 + (- t_sec_end_op1 + t_sec_end_op1) / Deltat_sec_sens, Omotion.get_filter_nav().get_filter_pos().get_size());
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void nav::test::Ttrj_sizes::test_sizes21() {
    unsigned short seed_order = 1;
    double turb_factor        = 1.0;

    double t_sec_init    = -5.;
    double t_sec_end_trj = 10.0;
    double t_sec_end_op0 = 20.0;
    double t_sec_gpsloss = 5.0;
    int    t_sec_turb    = 1000;

    sens::logic::SENS_COMPLETE_ID sens_complete_id = sens::logic::sens_complete_bias_scale_par;
    nav::logic::NAV_ID nav_id                      = nav::logic::nav_id010202;

    env::logic::WIND_ID wind_id         = env::logic::wind_id00;
    env::logic::OFFSETS_ID offsets_id   = env::logic::offsets_id00;
    env::logic::MAG_ID env_mag_id       = env::logic::mag_default;
    env::logic::REALISM_ID realism_id   = env::logic::realism_grav_no_magn_no;

    nav::logic::INITEUL_ID initeul_id   = nav::logic::initeul_id_zero;
    nav::logic::INITACC_ID initacc_id   = nav::logic::initacc_id_zero;
    nav::logic::INITGYR_ID initgyr_id   = nav::logic::initgyr_id_zero;
    nav::logic::INITMAG_ID initmag_id   = nav::logic::initmag_id_zero;
    nav::logic::INITMGN_ID initmgn_id   = nav::logic::initmgn_id_zero;

    sens::logic::GYR_ID gyr_id          = sens::logic::gyr_id_base;
    sens::logic::ACC_ID acc_id          = sens::logic::acc_id_base;
    sens::logic::MAG_ID mag_id          = sens::logic::mag_id_base;
    sens::logic::OSP_ID osp_id          = sens::logic::osp_id_base;
    sens::logic::OAT_ID oat_id          = sens::logic::oat_id_base;
    sens::logic::TAS_ID tas_id          = sens::logic::tas_id_base;
    sens::logic::AOA_ID aoa_id          = sens::logic::aoa_id_base;
    sens::logic::AOS_ID aos_id          = sens::logic::aos_id_base;
    sens::logic::GPS_ID gps_id          = sens::logic::gps_id_base;
    sens::logic::BAND_ID band_id        = sens::logic::band_id_base;

    st::logic::STI_ID sti_id = st::logic::sti_Hp3000_tas30_chi00;
    unsigned short nel_op = 1;

    math::seeder Oseeder(seed_order);
    auto Paero                = new acft::aero0();
    auto Pprop                = new acft::prop();
    auto Piner                = new acft::iner();
    auto Pearth               = new env::earth(Oseeder, wind_id, offsets_id, env_mag_id, realism_id, turb_factor, t_sec_init, t_sec_turb);
    auto Psuite               = new sens::suite(Oseeder, acc_id, gyr_id, mag_id, osp_id, oat_id, tas_id, aoa_id, aos_id, gps_id, band_id);
    auto Perr0                = new nav::init_error(Oseeder, initeul_id, initacc_id, initgyr_id, initmag_id, initmgn_id);
    auto Pguid                = new control::guid(nel_op, t_sec_end_trj, Oseeder.provide_seed(math::seeder::seeder_guid),0);
    Pguid->get_sti_id() = sti_id;
    Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 0.0, rud_beta_deg, 0., trgg_t_sec, t_sec_end_op0));
    st::sti* Psti             = st::sti::create_sti(*Pguid, t_sec_init);
    auto Pcntr                = new control::cntr(*Pguid);
    auto             Pnav     = new nav::filter_nav(nav_id, *Psuite, *Pearth, turb_factor);
    ang::euler euler_bc(0.0, 0.0, 0.0);
    auto Pcam                 = new sens::camera(768, 1024, 49.2255, euler_bc);

    nav::motion Omotion(Psti, Pguid, Pcntr, Pnav, Pearth, Psuite, Perr0, Piner, Paero, Pprop, Pcam, sens_complete_id);
    Omotion.solve_2nd(t_sec_end_trj, t_sec_gpsloss);

    double Deltat_sec_truth = 0.002;
    double Deltat_sec_sens  = 0.01;
    double Deltat_sec_gps   = 1.0;
    double Deltat_sec_cntr  = 0.02;
    double Deltat_sec_out   = 0.1;

    check("21 truth nel           ", 1 + ((- t_sec_init + t_sec_end_trj) / Deltat_sec_truth), Omotion.get_trj_truth().get_nel(),              1e-10);
    check("21 truth nel_op        ", nel_op,                                                  Omotion.get_trj_truth().get_nel_op(),           1e-10);
    check("21 truth last time     ", t_sec_end_trj,                                           Omotion.get_trj_truth()().back().get_t_sec(),   1e-10);
    check("21 truth op 0 start    ", 0,                                                       Omotion.get_trj_truth().get_op_start(0),        1e-10);
    check("21 truth op 0 end      ", (- t_sec_init + t_sec_end_trj) / Deltat_sec_truth,       Omotion.get_trj_truth().get_op_end(0),          1e-10);

    check("21 sens_in nel         ", 1 + ((- t_sec_init + t_sec_end_trj) / Deltat_sec_sens),  Omotion.get_trj_sens_in().get_nel(),            1e-10);
    check("21 sens_in nel_op      ", nel_op,                                                  Omotion.get_trj_sens_in().get_nel_op(),         1e-10);
    check("21 sens_in last time   ", t_sec_end_trj,                                           Omotion.get_trj_sens_in()().back().get_t_sec(), 1e-10);
    check("21 sens_in op 0 start  ", 0,                                                       Omotion.get_trj_sens_in().get_op_start(0),      1e-10);
    check("21 sens_in op 0 end    ", (- t_sec_init + t_sec_end_trj) / Deltat_sec_sens,        Omotion.get_trj_sens_in().get_op_end(0),        1e-10);

    check("21 sens_out nel        ", 1 + ((- t_sec_init + t_sec_end_trj) / Deltat_sec_sens),  Omotion.get_trj_sens_out().get_nel(),           1e-10);
    check("21 sens_out nel_op     ", nel_op,                                                  Omotion.get_trj_sens_out().get_nel_op(),        1e-10);
    check("21 sens_out last time  ", t_sec_end_trj,                                           Omotion.get_trj_sens_out()().back().get_t_sec(),1e-10);
    check("21 sens_out op 0 start ", 0,                                                       Omotion.get_trj_sens_out().get_op_start(0),     1e-10);
    check("21 sens_out op 0 end   ", (- t_sec_init + t_sec_end_trj) / Deltat_sec_sens,        Omotion.get_trj_sens_out().get_op_end(0),       1e-10);

    check("21 cntr nel            ", 1 + ((- t_sec_init + t_sec_end_trj) / Deltat_sec_cntr),  Omotion.get_trj_cntr().get_nel(),               1e-10);
    check("21 cntr nel_op         ", nel_op,                                                  Omotion.get_trj_cntr().get_nel_op(),            1e-10);
    check("21 cntr last time      ", t_sec_end_trj,                                           Omotion.get_trj_cntr()().back().get_t_sec(),    1e-10);
    check("21 cntr op 0 start     ", 0,                                                       Omotion.get_trj_cntr().get_op_start(0),         1e-10);
    check("21 cntr op 0 end       ", (- t_sec_init + t_sec_end_trj) / Deltat_sec_cntr,        Omotion.get_trj_cntr().get_op_end(0),           1e-10);

    check("21 nav_in nel          ", 1 + ((- t_sec_init + t_sec_end_trj) / Deltat_sec_sens),  Omotion.get_trj_nav_in().get_nel(),             1e-10);
    check("21 nav_in nel_op       ", nel_op,                                                  Omotion.get_trj_nav_in().get_nel_op(),          1e-10);
    check("21 nav_in last time    ", t_sec_end_trj,                                           Omotion.get_trj_nav_in()().back().get_t_sec(),  1e-10);
    check("21 nav_in op 0 start   ", 0,                                                       Omotion.get_trj_nav_in().get_op_start(0),       1e-10);
    check("21 nav_in op 0 end     ", (- t_sec_init + t_sec_end_trj) / Deltat_sec_sens,        Omotion.get_trj_nav_in().get_op_end(0),         1e-10);

    check("21 nav_out nel         ", 1 + ((- t_sec_init + t_sec_end_trj) / Deltat_sec_sens),  Omotion.get_trj_nav_out().get_nel(),            1e-10);
    check("21 nav_out nel_op      ", nel_op,                                                  Omotion.get_trj_nav_out().get_nel_op(),         1e-10);
    check("21 nav_out last time   ", t_sec_end_trj,                                           Omotion.get_trj_nav_out()().back().get_t_sec(), 1e-10);
    check("21 nav_out op 0 start  ", 0,                                                       Omotion.get_trj_nav_out().get_op_start(0),      1e-10);
    check("21 nav_out op 0 end    ", (- t_sec_init + t_sec_end_trj) / Deltat_sec_sens,        Omotion.get_trj_nav_out().get_op_end(0),        1e-10);

    check("21 gps_out nel         ", 1 + ((- t_sec_init + t_sec_gpsloss) / Deltat_sec_gps),   Omotion.get_trj_gps_out().get_nel(),            1e-10);
    check("21 gps_out nel_op      ", nel_op,                                                  Omotion.get_trj_gps_out().get_nel_op(),         1e-10);
    check("21 gps_out last time   ", t_sec_gpsloss,                                           Omotion.get_trj_gps_out()().back().get_t_sec(), 1e-10);
    check("21 gps_out op 0 start  ", 0,                                                       Omotion.get_trj_gps_out().get_op_start(0),      1e-10);
    check("21 gps_out op 0 end    ", (- t_sec_init + t_sec_gpsloss) / Deltat_sec_gps,         Omotion.get_trj_gps_out().get_op_end(0),        1e-10);

    check("21 out nel             ", 1 + ((- t_sec_init + t_sec_end_trj) / Deltat_sec_out),   Omotion.get_trj_out().get_nel(),                1e-10);
    check("21 out nel_op          ", nel_op,                                                  Omotion.get_trj_out().get_nel_op(),             1e-10);
    check("21 out last time       ", t_sec_end_trj,                                           Omotion.get_trj_out()().back().get_t_sec(),     1e-10);
    check("21 out op 0 start      ", 0,                                                       Omotion.get_trj_out().get_op_start(0),          1e-10);
    check("21 out op 0 end        ", (- t_sec_init + t_sec_end_trj) / Deltat_sec_out,         Omotion.get_trj_out().get_op_end(0),            1e-10);

    check("21 filter air          ", 1 + (- t_sec_init  + t_sec_end_trj) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_air().get_size());
    check("21 filter att          ", 1 + (- t_sec_init  + t_sec_end_trj) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_att().get_size());
    check("21 filter gps fast     ", 1 + (- t_sec_init  + t_sec_gpsloss) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_gps().get_size_fast());
    check("21 filter gps slow     ", 1 + (- t_sec_init  + t_sec_gpsloss) / Deltat_sec_gps,    Omotion.get_filter_nav().get_filter_gps().get_size_slow());
    check("21 filter pos          ", 1 + (- t_sec_gpsloss + t_sec_end_trj) / Deltat_sec_sens, Omotion.get_filter_nav().get_filter_pos().get_size());

}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void nav::test::Ttrj_sizes::test_sizes22() {
    unsigned short seed_order = 1;
    double turb_factor        = 1.0;

    double t_sec_init    = -5.;
    double t_sec_end_trj = 10.0;
    double t_sec_end_op0 = 20.0;
    double t_sec_gpsloss = 10.0;
    int    t_sec_turb    = 1000;

    sens::logic::SENS_COMPLETE_ID sens_complete_id = sens::logic::sens_complete_bias_scale_par;
    nav::logic::NAV_ID nav_id                      = nav::logic::nav_id010202;

    env::logic::WIND_ID wind_id         = env::logic::wind_id00;
    env::logic::OFFSETS_ID offsets_id   = env::logic::offsets_id00;
    env::logic::MAG_ID env_mag_id       = env::logic::mag_default;
    env::logic::REALISM_ID realism_id   = env::logic::realism_grav_no_magn_no;

    nav::logic::INITEUL_ID initeul_id   = nav::logic::initeul_id_zero;
    nav::logic::INITACC_ID initacc_id   = nav::logic::initacc_id_zero;
    nav::logic::INITGYR_ID initgyr_id   = nav::logic::initgyr_id_zero;
    nav::logic::INITMAG_ID initmag_id   = nav::logic::initmag_id_zero;
    nav::logic::INITMGN_ID initmgn_id   = nav::logic::initmgn_id_zero;

    sens::logic::GYR_ID gyr_id          = sens::logic::gyr_id_base;
    sens::logic::ACC_ID acc_id          = sens::logic::acc_id_base;
    sens::logic::MAG_ID mag_id          = sens::logic::mag_id_base;
    sens::logic::OSP_ID osp_id          = sens::logic::osp_id_base;
    sens::logic::OAT_ID oat_id          = sens::logic::oat_id_base;
    sens::logic::TAS_ID tas_id          = sens::logic::tas_id_base;
    sens::logic::AOA_ID aoa_id          = sens::logic::aoa_id_base;
    sens::logic::AOS_ID aos_id          = sens::logic::aos_id_base;
    sens::logic::GPS_ID gps_id          = sens::logic::gps_id_base;
    sens::logic::BAND_ID band_id        = sens::logic::band_id_base;

    st::logic::STI_ID sti_id = st::logic::sti_Hp3000_tas30_chi00;
    unsigned short nel_op = 1;

    math::seeder Oseeder(seed_order);
    auto Paero                = new acft::aero0();
    auto Pprop                = new acft::prop();
    auto Piner                = new acft::iner();
    auto Pearth               = new env::earth(Oseeder, wind_id, offsets_id, env_mag_id, realism_id, turb_factor, t_sec_init, t_sec_turb);
    auto Psuite               = new sens::suite(Oseeder, acc_id, gyr_id, mag_id, osp_id, oat_id, tas_id, aoa_id, aos_id, gps_id, band_id);
    auto Perr0                = new nav::init_error(Oseeder, initeul_id, initacc_id, initgyr_id, initmag_id, initmgn_id);
    auto Pguid                = new control::guid(nel_op, t_sec_end_trj, Oseeder.provide_seed(math::seeder::seeder_guid),0);
    Pguid->get_sti_id() = sti_id;
    Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 0.0, rud_beta_deg, 0., trgg_t_sec, t_sec_end_op0));
    st::sti* Psti             = st::sti::create_sti(*Pguid, t_sec_init);
    auto Pcntr                = new control::cntr(*Pguid);
    auto             Pnav     = new nav::filter_nav(nav_id, *Psuite, *Pearth, turb_factor);
    ang::euler euler_bc(0.0, 0.0, 0.0);
    auto Pcam                 = new sens::camera(768, 1024, 49.2255, euler_bc);

    nav::motion Omotion(Psti, Pguid, Pcntr, Pnav, Pearth, Psuite, Perr0, Piner, Paero, Pprop, Pcam, sens_complete_id);
    Omotion.solve_2nd(t_sec_end_trj, t_sec_gpsloss);

    double Deltat_sec_truth = 0.002;
    double Deltat_sec_sens  = 0.01;
    double Deltat_sec_gps   = 1.0;
    double Deltat_sec_cntr  = 0.02;
    double Deltat_sec_out   = 0.1;

    check("22 truth nel           ", 1 + ((- t_sec_init + t_sec_end_trj) / Deltat_sec_truth), Omotion.get_trj_truth().get_nel(),              1e-10);
    check("22 truth nel_op        ", nel_op,                                                  Omotion.get_trj_truth().get_nel_op(),           1e-10);
    check("22 truth last time     ", t_sec_end_trj,                                           Omotion.get_trj_truth()().back().get_t_sec(),   1e-10);
    check("22 truth op 0 start    ", 0,                                                       Omotion.get_trj_truth().get_op_start(0),        1e-10);
    check("22 truth op 0 end      ", (- t_sec_init + t_sec_end_trj) / Deltat_sec_truth,       Omotion.get_trj_truth().get_op_end(0),          1e-10);

    check("22 sens_in nel         ", 1 + ((- t_sec_init + t_sec_end_trj) / Deltat_sec_sens),  Omotion.get_trj_sens_in().get_nel(),            1e-10);
    check("22 sens_in nel_op      ", nel_op,                                                  Omotion.get_trj_sens_in().get_nel_op(),         1e-10);
    check("22 sens_in last time   ", t_sec_end_trj,                                           Omotion.get_trj_sens_in()().back().get_t_sec(), 1e-10);
    check("22 sens_in op 0 start  ", 0,                                                       Omotion.get_trj_sens_in().get_op_start(0),      1e-10);
    check("22 sens_in op 0 end    ", (- t_sec_init + t_sec_end_trj) / Deltat_sec_sens,        Omotion.get_trj_sens_in().get_op_end(0),        1e-10);

    check("22 sens_out nel        ", 1 + ((- t_sec_init + t_sec_end_trj) / Deltat_sec_sens),  Omotion.get_trj_sens_out().get_nel(),           1e-10);
    check("22 sens_out nel_op     ", nel_op,                                                  Omotion.get_trj_sens_out().get_nel_op(),        1e-10);
    check("22 sens_out last time  ", t_sec_end_trj,                                           Omotion.get_trj_sens_out()().back().get_t_sec(),1e-10);
    check("22 sens_out op 0 start ", 0,                                                       Omotion.get_trj_sens_out().get_op_start(0),     1e-10);
    check("22 sens_out op 0 end   ", (- t_sec_init + t_sec_end_trj) / Deltat_sec_sens,        Omotion.get_trj_sens_out().get_op_end(0),       1e-10);

    check("22 cntr nel            ", 1 + ((- t_sec_init + t_sec_end_trj) / Deltat_sec_cntr),  Omotion.get_trj_cntr().get_nel(),               1e-10);
    check("22 cntr nel_op         ", nel_op,                                                  Omotion.get_trj_cntr().get_nel_op(),            1e-10);
    check("22 cntr last time      ", t_sec_end_trj,                                           Omotion.get_trj_cntr()().back().get_t_sec(),    1e-10);
    check("22 cntr op 0 start     ", 0,                                                       Omotion.get_trj_cntr().get_op_start(0),         1e-10);
    check("22 cntr op 0 end       ", (- t_sec_init + t_sec_end_trj) / Deltat_sec_cntr,        Omotion.get_trj_cntr().get_op_end(0),           1e-10);

    check("22 nav_in nel          ", 1 + ((- t_sec_init + t_sec_end_trj) / Deltat_sec_sens),  Omotion.get_trj_nav_in().get_nel(),             1e-10);
    check("22 nav_in nel_op       ", nel_op,                                                  Omotion.get_trj_nav_in().get_nel_op(),          1e-10);
    check("22 nav_in last time    ", t_sec_end_trj,                                           Omotion.get_trj_nav_in()().back().get_t_sec(),  1e-10);
    check("22 nav_in op 0 start   ", 0,                                                       Omotion.get_trj_nav_in().get_op_start(0),       1e-10);
    check("22 nav_in op 0 end     ", (- t_sec_init + t_sec_end_trj) / Deltat_sec_sens,        Omotion.get_trj_nav_in().get_op_end(0),         1e-10);

    check("22 nav_out nel         ", 1 + ((- t_sec_init + t_sec_end_trj) / Deltat_sec_sens),  Omotion.get_trj_nav_out().get_nel(),            1e-10);
    check("22 nav_out nel_op      ", nel_op,                                                  Omotion.get_trj_nav_out().get_nel_op(),         1e-10);
    check("22 nav_out last time   ", t_sec_end_trj,                                           Omotion.get_trj_nav_out()().back().get_t_sec(), 1e-10);
    check("22 nav_out op 0 start  ", 0,                                                       Omotion.get_trj_nav_out().get_op_start(0),      1e-10);
    check("22 nav_out op 0 end    ", (- t_sec_init + t_sec_end_trj) / Deltat_sec_sens,        Omotion.get_trj_nav_out().get_op_end(0),        1e-10);

    check("22 gps_out nel         ", 1 + ((- t_sec_init + t_sec_gpsloss) / Deltat_sec_gps),   Omotion.get_trj_gps_out().get_nel(),            1e-10);
    check("22 gps_out nel_op      ", nel_op,                                                  Omotion.get_trj_gps_out().get_nel_op(),         1e-10);
    check("22 gps_out last time   ", t_sec_gpsloss,                                           Omotion.get_trj_gps_out()().back().get_t_sec(), 1e-10);
    check("22 gps_out op 0 start  ", 0,                                                       Omotion.get_trj_gps_out().get_op_start(0),      1e-10);
    check("22 gps_out op 0 end    ", (- t_sec_init + t_sec_gpsloss) / Deltat_sec_gps,         Omotion.get_trj_gps_out().get_op_end(0),        1e-10);

    check("22 out nel             ", 1 + ((- t_sec_init + t_sec_end_trj) / Deltat_sec_out),   Omotion.get_trj_out().get_nel(),                1e-10);
    check("22 out nel_op          ", nel_op,                                                  Omotion.get_trj_out().get_nel_op(),             1e-10);
    check("22 out last time       ", t_sec_end_trj,                                           Omotion.get_trj_out()().back().get_t_sec(),     1e-10);
    check("22 out op 0 start      ", 0,                                                       Omotion.get_trj_out().get_op_start(0),          1e-10);
    check("22 out op 0 end        ", (- t_sec_init + t_sec_end_trj) / Deltat_sec_out,         Omotion.get_trj_out().get_op_end(0),            1e-10);

    check("22 filter air          ", 1 + (- t_sec_init  + t_sec_end_trj) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_air().get_size());
    check("22 filter att          ", 1 + (- t_sec_init  + t_sec_end_trj) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_att().get_size());
    check("22 filter gps fast     ", 1 + (- t_sec_init  + t_sec_gpsloss) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_gps().get_size_fast());
    check("22 filter gps slow     ", 1 + (- t_sec_init  + t_sec_gpsloss) / Deltat_sec_gps,    Omotion.get_filter_nav().get_filter_gps().get_size_slow());
    check("22 filter pos          ", 1 + (- t_sec_gpsloss + t_sec_end_trj) / Deltat_sec_sens, Omotion.get_filter_nav().get_filter_pos().get_size());
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void nav::test::Ttrj_sizes::test_sizes23() {
    unsigned short seed_order = 1;
    double turb_factor        = 1.0;

    double t_sec_init    = -5.;
    double t_sec_end_trj = 10.0;
    double t_sec_end_op0 = 20.0;
    double t_sec_gpsloss = 15.0;
    int    t_sec_turb    = 1000;

    sens::logic::SENS_COMPLETE_ID sens_complete_id = sens::logic::sens_complete_bias_scale_par;
    nav::logic::NAV_ID nav_id                      = nav::logic::nav_id010202;

    env::logic::WIND_ID wind_id         = env::logic::wind_id00;
    env::logic::OFFSETS_ID offsets_id   = env::logic::offsets_id00;
    env::logic::MAG_ID env_mag_id       = env::logic::mag_default;
    env::logic::REALISM_ID realism_id   = env::logic::realism_grav_no_magn_no;

    nav::logic::INITEUL_ID initeul_id   = nav::logic::initeul_id_zero;
    nav::logic::INITACC_ID initacc_id   = nav::logic::initacc_id_zero;
    nav::logic::INITGYR_ID initgyr_id   = nav::logic::initgyr_id_zero;
    nav::logic::INITMAG_ID initmag_id   = nav::logic::initmag_id_zero;
    nav::logic::INITMGN_ID initmgn_id   = nav::logic::initmgn_id_zero;

    sens::logic::GYR_ID gyr_id          = sens::logic::gyr_id_base;
    sens::logic::ACC_ID acc_id          = sens::logic::acc_id_base;
    sens::logic::MAG_ID mag_id          = sens::logic::mag_id_base;
    sens::logic::OSP_ID osp_id          = sens::logic::osp_id_base;
    sens::logic::OAT_ID oat_id          = sens::logic::oat_id_base;
    sens::logic::TAS_ID tas_id          = sens::logic::tas_id_base;
    sens::logic::AOA_ID aoa_id          = sens::logic::aoa_id_base;
    sens::logic::AOS_ID aos_id          = sens::logic::aos_id_base;
    sens::logic::GPS_ID gps_id          = sens::logic::gps_id_base;
    sens::logic::BAND_ID band_id        = sens::logic::band_id_base;

    st::logic::STI_ID sti_id = st::logic::sti_Hp3000_tas30_chi00;
    unsigned short nel_op = 1;

    math::seeder Oseeder(seed_order);
    auto Paero                = new acft::aero0();
    auto Pprop                = new acft::prop();
    auto Piner                = new acft::iner();
    auto Pearth               = new env::earth(Oseeder, wind_id, offsets_id, env_mag_id, realism_id, turb_factor, t_sec_init, t_sec_turb);
    auto Psuite               = new sens::suite(Oseeder, acc_id, gyr_id, mag_id, osp_id, oat_id, tas_id, aoa_id, aos_id, gps_id, band_id);
    auto Perr0                = new nav::init_error(Oseeder, initeul_id, initacc_id, initgyr_id, initmag_id, initmgn_id);
    auto Pguid                = new control::guid(nel_op, t_sec_end_trj, Oseeder.provide_seed(math::seeder::seeder_guid),0);
    Pguid->get_sti_id() = sti_id;
    Pguid->add_op(0, new control::guid_op(thr_vtas_mps, 30.0, elv_Hp_m, 3000.0, ail_chi_deg, 0.0, rud_beta_deg, 0., trgg_t_sec, t_sec_end_op0));
    st::sti* Psti             = st::sti::create_sti(*Pguid, t_sec_init);
    auto Pcntr                = new control::cntr(*Pguid);
    auto             Pnav     = new nav::filter_nav(nav_id, *Psuite, *Pearth, turb_factor);
    ang::euler euler_bc(0.0, 0.0, 0.0);
    auto Pcam                 = new sens::camera(768, 1024, 49.2255, euler_bc);

    nav::motion Omotion(Psti, Pguid, Pcntr, Pnav, Pearth, Psuite, Perr0, Piner, Paero, Pprop, Pcam, sens_complete_id);
    Omotion.solve_2nd(t_sec_end_trj, t_sec_gpsloss);

    double Deltat_sec_truth = 0.002;
    double Deltat_sec_sens  = 0.01;
    double Deltat_sec_gps   = 1.0;
    double Deltat_sec_cntr  = 0.02;
    double Deltat_sec_out   = 0.1;

    check("23 truth nel           ", 1 + ((- t_sec_init + t_sec_end_trj) / Deltat_sec_truth), Omotion.get_trj_truth().get_nel(),              1e-10);
    check("23 truth nel_op        ", nel_op,                                                  Omotion.get_trj_truth().get_nel_op(),           1e-10);
    check("23 truth last time     ", t_sec_end_trj,                                           Omotion.get_trj_truth()().back().get_t_sec(),   1e-10);
    check("23 truth op 0 start    ", 0,                                                       Omotion.get_trj_truth().get_op_start(0),        1e-10);
    check("23 truth op 0 end      ", (- t_sec_init + t_sec_end_trj) / Deltat_sec_truth,       Omotion.get_trj_truth().get_op_end(0),          1e-10);

    check("23 sens_in nel         ", 1 + ((- t_sec_init + t_sec_end_trj) / Deltat_sec_sens),  Omotion.get_trj_sens_in().get_nel(),            1e-10);
    check("23 sens_in nel_op      ", nel_op,                                                  Omotion.get_trj_sens_in().get_nel_op(),         1e-10);
    check("23 sens_in last time   ", t_sec_end_trj,                                           Omotion.get_trj_sens_in()().back().get_t_sec(), 1e-10);
    check("23 sens_in op 0 start  ", 0,                                                       Omotion.get_trj_sens_in().get_op_start(0),      1e-10);
    check("23 sens_in op 0 end    ", (- t_sec_init + t_sec_end_trj) / Deltat_sec_sens,        Omotion.get_trj_sens_in().get_op_end(0),        1e-10);

    check("23 sens_out nel        ", 1 + ((- t_sec_init + t_sec_end_trj) / Deltat_sec_sens),  Omotion.get_trj_sens_out().get_nel(),           1e-10);
    check("23 sens_out nel_op     ", nel_op,                                                  Omotion.get_trj_sens_out().get_nel_op(),        1e-10);
    check("23 sens_out last time  ", t_sec_end_trj,                                           Omotion.get_trj_sens_out()().back().get_t_sec(),1e-10);
    check("23 sens_out op 0 start ", 0,                                                       Omotion.get_trj_sens_out().get_op_start(0),     1e-10);
    check("23 sens_out op 0 end   ", (- t_sec_init + t_sec_end_trj) / Deltat_sec_sens,        Omotion.get_trj_sens_out().get_op_end(0),       1e-10);

    check("23 cntr nel            ", 1 + ((- t_sec_init + t_sec_end_trj) / Deltat_sec_cntr),  Omotion.get_trj_cntr().get_nel(),               1e-10);
    check("23 cntr nel_op         ", nel_op,                                                  Omotion.get_trj_cntr().get_nel_op(),            1e-10);
    check("23 cntr last time      ", t_sec_end_trj,                                           Omotion.get_trj_cntr()().back().get_t_sec(),    1e-10);
    check("23 cntr op 0 start     ", 0,                                                       Omotion.get_trj_cntr().get_op_start(0),         1e-10);
    check("23 cntr op 0 end       ", (- t_sec_init + t_sec_end_trj) / Deltat_sec_cntr,        Omotion.get_trj_cntr().get_op_end(0),           1e-10);

    check("23 nav_in nel          ", 1 + ((- t_sec_init + t_sec_end_trj) / Deltat_sec_sens),  Omotion.get_trj_nav_in().get_nel(),             1e-10);
    check("23 nav_in nel_op       ", nel_op,                                                  Omotion.get_trj_nav_in().get_nel_op(),          1e-10);
    check("23 nav_in last time    ", t_sec_end_trj,                                           Omotion.get_trj_nav_in()().back().get_t_sec(),  1e-10);
    check("23 nav_in op 0 start   ", 0,                                                       Omotion.get_trj_nav_in().get_op_start(0),       1e-10);
    check("23 nav_in op 0 end     ", (- t_sec_init + t_sec_end_trj) / Deltat_sec_sens,        Omotion.get_trj_nav_in().get_op_end(0),         1e-10);

    check("23 nav_out nel         ", 1 + ((- t_sec_init + t_sec_end_trj) / Deltat_sec_sens),  Omotion.get_trj_nav_out().get_nel(),            1e-10);
    check("23 nav_out nel_op      ", nel_op,                                                  Omotion.get_trj_nav_out().get_nel_op(),         1e-10);
    check("23 nav_out last time   ", t_sec_end_trj,                                           Omotion.get_trj_nav_out()().back().get_t_sec(), 1e-10);
    check("23 nav_out op 0 start  ", 0,                                                       Omotion.get_trj_nav_out().get_op_start(0),      1e-10);
    check("23 nav_out op 0 end    ", (- t_sec_init + t_sec_end_trj) / Deltat_sec_sens,        Omotion.get_trj_nav_out().get_op_end(0),        1e-10);

    check("23 gps_out nel         ", 1 + ((- t_sec_init + t_sec_end_trj) / Deltat_sec_gps),   Omotion.get_trj_gps_out().get_nel(),            1e-10);
    check("23 gps_out nel_op      ", nel_op,                                                  Omotion.get_trj_gps_out().get_nel_op(),         1e-10);
    check("23 gps_out last time   ", t_sec_end_trj,                                           Omotion.get_trj_gps_out()().back().get_t_sec(), 1e-10);
    check("23 gps_out op 0 start  ", 0,                                                       Omotion.get_trj_gps_out().get_op_start(0),      1e-10);
    check("23 gps_out op 0 end    ", (- t_sec_init + t_sec_end_trj) / Deltat_sec_gps,         Omotion.get_trj_gps_out().get_op_end(0),        1e-10);

    check("23 out nel             ", 1 + ((- t_sec_init + t_sec_end_trj) / Deltat_sec_out),   Omotion.get_trj_out().get_nel(),                1e-10);
    check("23 out nel_op          ", nel_op,                                                  Omotion.get_trj_out().get_nel_op(),             1e-10);
    check("23 out last time       ", t_sec_end_trj,                                           Omotion.get_trj_out()().back().get_t_sec(),     1e-10);
    check("23 out op 0 start      ", 0,                                                       Omotion.get_trj_out().get_op_start(0),          1e-10);
    check("23 out op 0 end        ", (- t_sec_init + t_sec_end_trj) / Deltat_sec_out,         Omotion.get_trj_out().get_op_end(0),            1e-10);

    check("23 filter air          ", 1 + (- t_sec_init  + t_sec_end_trj) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_air().get_size());
    check("23 filter att          ", 1 + (- t_sec_init  + t_sec_end_trj) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_att().get_size());
    check("23 filter gps fast     ", 1 + (- t_sec_init  + t_sec_end_trj) / Deltat_sec_sens,   Omotion.get_filter_nav().get_filter_gps().get_size_fast());
    check("23 filter gps slow     ", 1 + (- t_sec_init  + t_sec_end_trj) / Deltat_sec_gps,    Omotion.get_filter_nav().get_filter_gps().get_size_slow());
    check("23 filter pos          ", 1 + (- t_sec_end_trj + t_sec_end_trj) / Deltat_sec_sens, Omotion.get_filter_nav().get_filter_pos().get_size());
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

