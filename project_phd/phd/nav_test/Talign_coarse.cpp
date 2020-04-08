#include "Talign_coarse.h"

#include "math/templates/metrics_.h"
#include "env/mag.h"
#include "env/geo.h"
#include "acft/sens/suite.h"
#include "acft/st/sti.h"
#include "acft/guid/guid.h"
#include "nav/init/align_coarse.h"

nav::test::Talign_coarse::Talign_coarse(jail::counter& Ocounter)
: ::jail::unit_test(Ocounter) {
}
/* constructor based on counter */

void nav::test::Talign_coarse::run() {
	::jail::unit_test::run();

    {
        ang::euler euler_nb_truth01(30.0 * math::constant::D2R(), 0., 0.);
        test_align_single(euler_nb_truth01, sens::logic::gyr_id_base, sens::logic::acc_id_base, sens::logic::mag_id_base);
    }
    {
        test_align_multiple(sens::logic::gyr_id_base, sens::logic::acc_id_base, sens::logic::mag_id_base);
        //test_align_multiple(sens::logic::gyr_id52, sens::logic::acc_id52, sens::logic::mag_id51);
        //test_align_multiple(sens::logic::gyr_id53, sens::logic::acc_id52, sens::logic::mag_id51);
        //test_align_multiple(sens::logic::gyr_id54, sens::logic::acc_id53, sens::logic::mag_id51);
    }

    finished();
}
/* execute tests and write results on console */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void nav::test::Talign_coarse::test_align_single(const ang::euler& euler_nb_truth, sens::logic::GYR_ID gyr_id, sens::logic::ACC_ID acc_id, sens::logic::MAG_ID mag_id) {
    double t_sec              = 0.;
    double Deltat_sec_sens    = 0.01;
    unsigned int Deltat_sec   = 600;
    unsigned short seed_order = 1;

    env::logic::MAG_ID env_mag_id     = env::logic::mag_wisconsin;
    env::logic::REALISM_ID realism_id = env::logic::realism_grav_yes_magn_yes;
    sens::logic::BAND_ID band_id      = sens::logic::band_id_base;

    env::geodetic_coord x_gdt_rad_m_truth(270.0 * math::constant::D2R(), 45.0 * math::constant::D2R(), 3000.0);
    math::seeder Oseeder(seed_order);
    sens::platform* Pplat       = sens::platform::create_platform(Oseeder.provide_seed(math::seeder::seeder_acft_plat));
    sens::sens_triple_acc* Pacc = sens::sens_triple_acc::create_accelerometer(acc_id, band_id, Oseeder.provide_seed(math::seeder::seeder_acft_acc), Oseeder.provide_seed(math::seeder::seeder_acc), *Pplat, Deltat_sec_sens);
    sens::sens_triple_gyr* Pgyr = sens::sens_triple_gyr::create_gyroscope(gyr_id,     band_id, Oseeder.provide_seed(math::seeder::seeder_acft_gyr), Oseeder.provide_seed(math::seeder::seeder_gyr), *Pplat, Deltat_sec_sens);
    sens::sens_triple_mag* Pmag = sens::sens_triple_mag::create_magnetometer(mag_id,  Oseeder.provide_seed(math::seeder::seeder_acft_mag), Oseeder.provide_seed(math::seeder::seeder_mag), Oseeder.get_seed_order(), Deltat_sec_sens);
    env::geo_mix Ogeo(env_mag_id, realism_id, Oseeder.provide_seed(math::seeder::seeder_geo));

    nav::align_coarse Oalign_coarse(*Pplat, Ogeo, x_gdt_rad_m_truth, t_sec);

    std::cout << std::endl;
    std::cout << "TRUE INITIAL CONDITIONS: " << std::endl;
    std::cout << "psi [deg]  : " << std::fixed << std::setw(8) << std::setprecision(2) << std::showpos << euler_nb_truth.get_yaw_rad()   * math::constant::R2D() << std::endl;
    std::cout << "theta [deg]: " << std::fixed << std::setw(8) << std::setprecision(2) << std::showpos << euler_nb_truth.get_pitch_rad() * math::constant::R2D() << std::endl;
    std::cout << "xi [deg]   : " << std::fixed << std::setw(8) << std::setprecision(2) << std::showpos << euler_nb_truth.get_bank_rad()  * math::constant::R2D() << std::endl;
    std::cout << "Gyroscope " << gyr_id << ", accelerometer " << acc_id << ", magnetometer " << mag_id << std::endl;
    double theta_rad_est, xi_rad_est;
    Oalign_coarse.leveling(*Pacc, theta_rad_est, xi_rad_est, euler_nb_truth, Deltat_sec);
    double theta_deg_est = theta_rad_est * math::constant::R2D();
    double xi_deg_est    = xi_rad_est    * math::constant::R2D();
    std::cout << "SELF ALIGNMENT LEVELING: " << std::endl;
    std::cout << "theta [deg]: " << std::fixed << std::setw(8) << std::setprecision(2) << std::showpos << theta_deg_est << std::endl;
    std::cout << "xi [deg]   : " << std::fixed << std::setw(8) << std::setprecision(2) << std::showpos << xi_deg_est    << std::endl;

    double Apsi_rad_est;
    Oalign_coarse.gyrocompassing(*Pgyr, Apsi_rad_est, theta_rad_est, xi_rad_est, euler_nb_truth, Deltat_sec);
    double Apsi_deg_est = Apsi_rad_est * math::constant::R2D();
    std::cout << "SELF ALIGNMENT GYROCOMPASSING: " << std::endl;
    std::cout << "psi [deg]  : " << std::fixed << std::setw(8) << std::setprecision(2) << std::showpos << Apsi_deg_est << std::endl;

    double Bpsi_rad_est;
    Oalign_coarse.magnetic_alignment(*Pmag, Bpsi_rad_est, theta_rad_est, xi_rad_est, euler_nb_truth, Deltat_sec);
    double Bpsi_deg_est = Bpsi_rad_est * math::constant::R2D();
    std::cout << "SELF ALIGNMENT MAGNETIC ALIGNMENT: " << std::endl;
    std::cout << "psi [deg]  : " << std::fixed << std::setw(8) << std::setprecision(2) << std::showpos << Bpsi_deg_est << std::endl;

    delete Pplat;
    delete Pacc;
    delete Pgyr;
    delete Pmag;
    std::cout << std::endl;
} // closes test_align

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void nav::test::Talign_coarse::test_align_multiple(sens::logic::GYR_ID gyr_id, sens::logic::ACC_ID acc_id, sens::logic::MAG_ID mag_id) {
    double t_sec              = 0.0;
    double Deltat_sec_sens    = 0.01;
    unsigned int Deltat_sec   = 600;
    unsigned short seed_order = 1;

    env::logic::MAG_ID env_mag_id     = env::logic::mag_wisconsin;
    env::logic::REALISM_ID realism_id = env::logic::realism_grav_yes_magn_yes;
    sens::logic::BAND_ID band_id      = sens::logic::band_id_base;

    env::geodetic_coord x_gdt_rad_m_truth(270.0 * math::constant::D2R(), 45.0 * math::constant::D2R(), 3000.0);

    math::seeder Oseeder(seed_order);
    sens::platform* Pplat       = sens::platform::create_platform(Oseeder.provide_seed(math::seeder::seeder_acft_plat));
    sens::sens_triple_acc* Pacc = sens::sens_triple_acc::create_accelerometer(acc_id, band_id, Oseeder.provide_seed(math::seeder::seeder_acft_acc), Oseeder.provide_seed(math::seeder::seeder_acc), *Pplat, Deltat_sec_sens);
    sens::sens_triple_gyr* Pgyr = sens::sens_triple_gyr::create_gyroscope(gyr_id,     band_id, Oseeder.provide_seed(math::seeder::seeder_acft_gyr), Oseeder.provide_seed(math::seeder::seeder_gyr), *Pplat, Deltat_sec_sens);
    sens::sens_triple_mag* Pmag = sens::sens_triple_mag::create_magnetometer(mag_id,  Oseeder.provide_seed(math::seeder::seeder_acft_mag), Oseeder.provide_seed(math::seeder::seeder_mag), Oseeder.get_seed_order(), Deltat_sec_sens);
    env::geo_mix Ogeo(env_mag_id, realism_id, Oseeder.provide_seed(math::seeder::seeder_geo));

    nav::align_coarse Oalign_coarse(*Pplat, Ogeo, x_gdt_rad_m_truth, t_sec);

    // 36 equispaced headings (every 10 [deg])
    unsigned int sizA = 36;
    std::vector<ang::euler> Veuler_nb_truth(sizA);
    double d2r = math::constant::D2R();
    for (unsigned int i = 0; i != sizA; ++i) {
        Veuler_nb_truth[i].set_yaw_rad((10.0 * i + 5.0) * d2r);
        Veuler_nb_truth[i].set_pitch_rad(0.0);
        Veuler_nb_truth[i].set_bank_rad(0.0);
    }
    std::cout << "Gyroscope " << gyr_id << ", accelerometer " << acc_id << ", magnetometer " << mag_id << std::endl;
    std::vector<double> Vtheta_rad_est(sizA), Vxi_rad_est(sizA);
    Oalign_coarse.leveling_multiple(*Pacc, Vtheta_rad_est, Vxi_rad_est, Veuler_nb_truth, true, Deltat_sec); // show results in console

    std::vector<double> VApsi_rad_est(sizA);
    Oalign_coarse.gyrocompassing_multiple(*Pgyr, VApsi_rad_est, Vtheta_rad_est, Vxi_rad_est, Veuler_nb_truth, true, Deltat_sec); // show results in console

    std::vector<double> VBpsi_rad_est(sizA);
    Oalign_coarse.magnetic_alignment_multiple(*Pmag, VBpsi_rad_est, Vtheta_rad_est, Vxi_rad_est, Veuler_nb_truth, true, Deltat_sec); // show results in console

    delete Pplat;
    delete Pacc;
    delete Pgyr;
    delete Pmag;
} // closes test_level_multiple

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////



































