
#include "acft/acft/aero0.h"
#include "acft/control/cntr.h"
#include "acft/sens/suite.h"
#include "env/geo.h"
#include "env/turb.h"
#include "nav/motion/motion.h"
#include "nav/motion/textplot.h"
#include "nav/nav/filter_nav.h"

int main(int argc, char **argv) {
    unsigned short case_guid, seed_order_ini, seed_order_end;
    double turb_factor;
    switch (argc) {
        case 1: {
            case_guid = 3;
            turb_factor = 0.5;
            seed_order_ini = 1;
            seed_order_end = 100;
            break;
        }
        case 5: {
            std::stringstream str1, str2, str3, str4;
            str1 << argv[1];
            str1 >> case_guid;
            str2 << argv[2];
            str2 >> turb_factor;
            str3 << argv[3];
            str3 >> seed_order_ini;
            str4 << argv[4];
            str4 >> seed_order_end;
            break; }
        default:
            throw std::runtime_error("Wrong number of input parameters, four needed (case_guid & turb_factor & seed_order_ini & seed_order_end) ");
    }

    double t_sec_init       = -50.0;
    double t_sec_end        = 500.0;
    int    t_sec_turb       = 1000;
    double t_sec_gpsloss    = 100.0;

    sens::logic::SENS_COMPLETE_ID sens_complete_id = sens::logic::sens_complete_bias_scale_par;
    nav::logic::NAV_ID nav_id                      = nav::logic::nav_id020202;

    env::logic::WIND_ID wind_id         = env::logic::wind_id04;
    env::logic::OFFSETS_ID offsets_id   = env::logic::offsets_id03;
    env::logic::MAG_ID env_mag_id       = env::logic::mag_wisconsin;
    env::logic::REALISM_ID realism_id   = env::logic::realism_grav_yes_magn_yes;

    nav::logic::INITEUL_ID initeul_id   = nav::logic::initeul_id_base;
    nav::logic::INITACC_ID initacc_id   = nav::logic::initacc_id_base;
    nav::logic::INITGYR_ID initgyr_id   = nav::logic::initgyr_id_base;
    nav::logic::INITMAG_ID initmag_id   = nav::logic::initmag_id_base;
    nav::logic::INITMGN_ID initmgn_id   = nav::logic::initmgn_id_base;

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

    for (unsigned short seed_order = seed_order_ini; seed_order <= seed_order_end; ++seed_order) {
        //t_sec_gpsloss = 100.0; /////////////////////////////////////////////

        math::seeder Oseeder(seed_order);
        auto Paero = new acft::aero0();
        auto Pprop = new acft::prop();
        auto Piner = new acft::iner();
        auto Pearth = new env::earth(Oseeder, wind_id, offsets_id, env_mag_id, realism_id, turb_factor, t_sec_init, t_sec_turb);
        auto Psuite = new sens::suite(Oseeder, acc_id, gyr_id, mag_id, osp_id, oat_id, tas_id, aoa_id, aos_id, gps_id, band_id);
        auto Perr0 = new nav::init_error(Oseeder, initeul_id, initacc_id, initgyr_id, initmag_id, initmgn_id);
        control::guid *Pguid = control::guid::create_guid(Oseeder, case_guid, t_sec_gpsloss);

        st::sti *Psti = st::sti::create_sti(*Pguid, t_sec_init);
        auto Pcntr = new control::cntr(*Pguid);
        auto Pnav = new nav::filter_nav(nav_id, *Psuite, *Pearth, turb_factor);
        ang::euler euler_bc(0.0, 0.0, 0.0);
        auto Pcam = new sens::camera(768, 1024, 49.2255, euler_bc);

        //t_sec_gpsloss = t_sec_end; //////////////////////////////////////////

        nav::motion Omotion(Psti, Pguid, Pcntr, Pnav, Pearth, Psuite, Perr0, Piner, Paero, Pprop, Pcam, sens_complete_id);

        nav::textplot Otextplot(Omotion);
        Omotion.solve_2nd(t_sec_end, t_sec_gpsloss);

        std::string st_folder = Otextplot.obtain_folder(case_guid, seed_order, turb_factor, offsets_id, wind_id, t_sec_gpsloss);

        Otextplot.execute(st_folder);
        std::cout << std::endl << "Results saved at " << st_folder << std::endl;
    }
    return 0;
}
