#include "Tplots_seeds.h"

#include "math/logic/share.h"
#include "ang/tools.h"
#include "ang/rotate/rotv.h"
#include "env/earth.h"
#include "acft/guid/guid.h"
#include "nav/init/init_error.h"
#include <boost/filesystem.hpp>

nav::test::Tplots_seeds::Tplots_seeds(unsigned short& seed_init, unsigned short& seed_end)
: _seed_init(seed_init), _seed_end(seed_end) {
}
/* constructor based on counter */

void nav::test::Tplots_seeds::run() {
    test_guidance_bearing  (_seed_init, _seed_end);
    test_guidance_alt_speed(_seed_init, _seed_end);
    test_offsets           (_seed_init, _seed_end);
    test_wind              (_seed_init, _seed_end);
    test_gravity           (_seed_init, _seed_end);
    test_magnetic          (_seed_init, _seed_end);
}
/* execute tests and write results on console */

using namespace std;

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void nav::test::Tplots_seeds::test_guidance_bearing(unsigned short& seed_init, unsigned short& seed_end) {

    unsigned short case_guid = 1;

    boost::filesystem::path path_outputs(math::share::phd_outputs_prefix);
    boost::filesystem::path path_folder("nav");
    boost::filesystem::path path_folder2("seeds");
    boost::filesystem::path path_file("seeds_mission_bearing.txt");
    
    boost::filesystem::create_directory((path_outputs / path_folder / path_folder2).string());
    std::string st_file = (path_outputs / path_folder / path_folder2 / path_file).string();

    std::ofstream Ostream;
    Ostream.open(st_file);

    std::string st_seeds;
    double t_sec_gpsloss    = 100.0;

    for (unsigned short seed_order = seed_init; seed_order != seed_end; seed_order++) {
        math::seeder Oseeder(seed_order);

        auto Pguid = control::guid::create_guid(Oseeder, case_guid, t_sec_gpsloss);

        st_seeds = std::to_string(seed_order);
        switch (st_seeds.size()) {
            case 1:
                st_seeds.insert(0, "0");
                break;
            case 2:
                break;
            default:
                throw std::runtime_error("Incorrect seeds choice.");
        }

        double b = (double) (seed_order / 25) / 10.0;

        double Deltachi_deg = Pguid->get()[2]->get_guid_val(control::logic::cntr_AIL) - Pguid->get()[0]->get_guid_val(control::logic::cntr_AIL);
        ang::tools::correct_yaw_deg(Deltachi_deg);

        Ostream << st_seeds
                << fixed << setw(10) << setprecision(2) << showpos << Pguid->get()[0]->get_guid_val(control::logic::cntr_AIL)
                << fixed << setw(6)  << setprecision(1) << showpos << 1.0 + b
                << fixed << setw(10) << setprecision(2) << showpos << Pguid->get()[2]->get_guid_val(control::logic::cntr_AIL)
                << fixed << setw(6)  << setprecision(1) << showpos << 1.7 + b
                << fixed << setw(10) << setprecision(2) << showpos << Deltachi_deg
                << endl;

        delete Pguid;
    }

    Ostream.close();
} // closes test_guidance_bearing

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void nav::test::Tplots_seeds::test_guidance_alt_speed(unsigned short& seed_init, unsigned short& seed_end) {

    unsigned short case_guid = 1;

    boost::filesystem::path path_outputs(math::share::phd_outputs_prefix);
    boost::filesystem::path path_folder("nav");
    boost::filesystem::path path_folder2("seeds");
    boost::filesystem::path path_file("seeds_mission_alt_speed.txt");
    boost::filesystem::create_directory((path_outputs / path_folder / path_folder2).string());
    std::string st_file = (path_outputs / path_folder / path_folder2 / path_file).string();

    std::ofstream Ostream;
    Ostream.open(st_file);

    std::string st_seeds;
    double t_sec_gpsloss    = 100.0;

    for (unsigned short seed_order = seed_init; seed_order != seed_end; seed_order++) {
        math::seeder Oseeder(seed_order);

        auto Pguid = control::guid::create_guid(Oseeder, case_guid, t_sec_gpsloss);

        st_seeds = std::to_string(seed_order);
        switch (st_seeds.size()) {
            case 1:
                st_seeds.insert(0, "0");
                break;
            case 2:
                break;
            default:
                throw std::runtime_error("Incorrect seeds choice.");
        }

        Ostream << st_seeds
                << fixed << setw(14) << setprecision(2) << showpos << Pguid->get()[0]->get_guid_val(control::logic::cntr_ELV)
                << fixed << setw(14) << setprecision(2) << showpos << Pguid->get()[5]->get_guid_val(control::logic::cntr_ELV)
                << fixed << setw(14) << setprecision(2) << showpos << Pguid->get()[5]->get_guid_val(control::logic::cntr_ELV) - Pguid->get()[0]->get_guid_val(control::logic::cntr_ELV)
                << fixed << setw(14) << setprecision(2) << showpos << Pguid->get()[0]->get_guid_val(control::logic::cntr_THR)
                << fixed << setw(14) << setprecision(2) << showpos << Pguid->get()[5]->get_guid_val(control::logic::cntr_THR)
                << fixed << setw(14) << setprecision(2) << showpos << Pguid->get()[5]->get_guid_val(control::logic::cntr_THR) - Pguid->get()[0]->get_guid_val(control::logic::cntr_THR)
                << endl;

        delete Pguid;
    }

    Ostream.close();
} // closes test_guidance_altitude

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void nav::test::Tplots_seeds::test_offsets(unsigned short& seed_init, unsigned short& seed_end) {

    env::logic::OFFSETS_ID offsets_id   = env::logic::offsets_id02;

    boost::filesystem::path path_outputs(math::share::phd_outputs_prefix);
    boost::filesystem::path path_folder("nav");
    boost::filesystem::path path_folder2("seeds");
    boost::filesystem::path path_file("seeds_weather_offsets.txt");
    boost::filesystem::create_directory((path_outputs / path_folder / path_folder2).string());
    std::string st_file = (path_outputs / path_folder / path_folder2 / path_file).string();

    std::ofstream Ostream;
    Ostream.open(st_file);

    std::string st_seeds;

    for (unsigned short seed_order = seed_init; seed_order != seed_end; seed_order++) {
        math::seeder Oseeder(seed_order);
        auto Poff = env::offsets::create_offsets(offsets_id, Oseeder.provide_seed(math::seeder::seeder_offsets));
        auto Poffsets = dynamic_cast<env::offsets_ramp*>(Poff);

        st_seeds = std::to_string(seed_order);
        switch (st_seeds.size()) {
            case 1:
                st_seeds.insert(0, "0");
                break;
            case 2:
                break;
            default:
                throw std::runtime_error("Incorrect seeds choice.");
        }

        double dDeltaT_dt_degK_min = (Poffsets->get_DeltaT_degK_end() - Poffsets->get_DeltaT_degK_ini()) /
                                     (Poffsets->get_t_sec_end_T() - Poffsets->get_t_sec_ini_T()) * 60;
        double dDeltap_dt_pa_min   = (Poffsets->get_Deltap_pa_end() - Poffsets->get_Deltap_pa_ini()) /
                                     (Poffsets->get_t_sec_end_p() - Poffsets->get_t_sec_ini_p()) * 60;

        Ostream << st_seeds
                << fixed << setw(10) << setprecision(2) << showpos << Poffsets->get_t_sec_ini_T()
                << fixed << setw(10) << setprecision(2) << showpos << Poffsets->get_DeltaT_degK_ini()
                << fixed << setw(10) << setprecision(2) << showpos << Poffsets->get_t_sec_end_T()
                << fixed << setw(10) << setprecision(2) << showpos << Poffsets->get_DeltaT_degK_end()
                << fixed << setw(10) << setprecision(2) << showpos << Poffsets->get_DeltaT_degK_end() - Poffsets->get_DeltaT_degK_ini()
                << fixed << setw(10) << setprecision(2) << showpos << Poffsets->get_t_sec_ini_p()
                << fixed << setw(10) << setprecision(2) << showpos << Poffsets->get_Deltap_pa_ini()
                << fixed << setw(10) << setprecision(2) << showpos << Poffsets->get_t_sec_end_p()
                << fixed << setw(10) << setprecision(2) << showpos << Poffsets->get_Deltap_pa_end()
                << fixed << setw(10) << setprecision(2) << showpos << Poffsets->get_Deltap_pa_end() - Poffsets->get_Deltap_pa_ini()
                << fixed << setw(10) << setprecision(2) << showpos << dDeltaT_dt_degK_min
                << fixed << setw(10) << setprecision(2) << showpos << dDeltap_dt_pa_min
                << endl;

        delete Poff;
    }

    Ostream.close();
} // closes test_offsets

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

void nav::test::Tplots_seeds::test_wind(unsigned short& seed_init, unsigned short& seed_end) {

    env::logic::WIND_ID wind_id = env::logic::wind_id03;

    boost::filesystem::path path_outputs(math::share::phd_outputs_prefix);
    boost::filesystem::path path_folder("nav");
    boost::filesystem::path path_folder2("seeds");
    boost::filesystem::path path_file("seeds_weather_wind.txt");
    boost::filesystem::create_directory((path_outputs / path_folder / path_folder2).string());
    std::string st_file = (path_outputs / path_folder / path_folder2 / path_file).string();

    std::ofstream Ostream;
    Ostream.open(st_file);

    std::string st_seeds;

    for (unsigned short seed_order = seed_init; seed_order != seed_end; seed_order++) {
        math::seeder Oseeder(seed_order);
        auto Pw = env::wind::create_wind(wind_id, Oseeder.provide_seed(math::seeder::seeder_wind));
        auto Pwind = dynamic_cast<env::wind_ramp*>(Pw);

        st_seeds = std::to_string(seed_order);
        switch (st_seeds.size()) {
            case 1:
                st_seeds.insert(0, "0");
                break;
            case 2:
                break;
            default:
                throw std::runtime_error("Incorrect seeds choice.");
        }

        double Deltawind_mps     = fabs(Pwind->get_wind_mps_end()     - Pwind->get_wind_mps_ini());
        double Deltachi_wind_deg = Pwind->get_chi_wind_deg_end() - Pwind->get_chi_wind_deg_ini();
        ang::tools::correct_yaw_deg(Deltachi_wind_deg);

        double dDeltawind_dt_mps_min     = (Pwind->get_wind_mps_end()     - Pwind->get_wind_mps_ini()) /
                                           (Pwind->get_t_sec_end() - Pwind->get_t_sec_ini()) * 60;
        double dDeltachi_wind_dt_deg_min = Deltachi_wind_deg / (Pwind->get_t_sec_end() - Pwind->get_t_sec_ini()) * 60;

        Ostream << st_seeds
                << fixed << setw(10) << setprecision(2) << showpos << Pwind->get_t_sec_ini()
                << fixed << setw(10) << setprecision(2) << showpos << Pwind->get_t_sec_end()
                << fixed << setw(10) << setprecision(2) << showpos << Pwind->get_wind_mps_ini()
                << fixed << setw(10) << setprecision(2) << showpos << Pwind->get_wind_mps_end()
                << fixed << setw(10) << setprecision(2) << showpos << Pwind->get_chi_wind_deg_ini()
                << fixed << setw(10) << setprecision(2) << showpos << Pwind->get_chi_wind_deg_end()
                << fixed << setw(10) << setprecision(2) << showpos << Deltawind_mps
                << fixed << setw(10) << setprecision(2) << showpos << Deltachi_wind_deg
                << fixed << setw(10) << setprecision(2) << showpos << dDeltawind_dt_mps_min
                << fixed << setw(10) << setprecision(2) << showpos << dDeltachi_wind_dt_deg_min
                << endl;
        delete Pw;
    }

    Ostream.close();
} // closes test_wind

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

void nav::test::Tplots_seeds::test_gravity(unsigned short& seed_init, unsigned short& seed_end) {
    env::logic::MAG_ID env_mag_id       = env::logic::mag_wisconsin; // plays no role
    env::logic::REALISM_ID realism_id   = env::logic::realism_grav_yes_magn_yes;

    boost::filesystem::path path_outputs(math::share::phd_outputs_prefix);

    boost::filesystem::path path_folder("nav");
    boost::filesystem::path path_folder2("seeds");
    boost::filesystem::path path_file("seeds_geo_gravity.txt");
    boost::filesystem::create_directory((path_outputs / path_folder / path_folder2).string());
    std::string st_file = (path_outputs / path_folder / path_folder2 / path_file).string();

    std::ofstream Ostream;
    Ostream.open(st_file);

    std::string st_seeds;
    Eigen::Vector3d gc_n_model_mps2, gc_n_truth_mps2;
    env::geodetic_coord x_gdt_rad_m(270.0 * math::constant::D2R(), 45.0 * math::constant::D2R(), 3000.0);

    for (unsigned short seed_order = seed_init; seed_order != seed_end; seed_order++) {
        math::seeder Oseeder(seed_order);
        auto Pgeo = new env::geo_mix(env_mag_id, realism_id, Oseeder.provide_seed(math::seeder::seeder_geo));

        st_seeds = std::to_string(seed_order);
        switch (st_seeds.size()) {
            case 1:
                st_seeds.insert(0, "0");
                break;
            case 2:
                break;
            default:
                throw std::runtime_error("Incorrect seeds choice.");
        }

        gc_n_model_mps2 = Pgeo->compute_gravity_n_model(x_gdt_rad_m);
        gc_n_truth_mps2 = Pgeo->compute_gravity_n_truth(gc_n_model_mps2);

        ang::rotv rotv_gc_error(gc_n_model_mps2, gc_n_truth_mps2);
        double gc_error_mps2 = fabs(gc_n_truth_mps2.norm() - gc_n_model_mps2.norm());
        double gc_error_deg  = rotv_gc_error.norm() * math::constant::R2D();

        Ostream << st_seeds
                << fixed << setw(15) << setprecision(8) << showpos << gc_error_mps2 * 10000
                << fixed << setw(15) << setprecision(5) << showpos << gc_error_deg * 1000
                << endl;
        delete Pgeo;
    }

    Ostream.close();
} // closes test_gravity

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

void nav::test::Tplots_seeds::test_magnetic(unsigned short& seed_init, unsigned short& seed_end) {
    env::logic::MAG_ID env_mag_id       = env::logic::mag_wisconsin;
    env::logic::REALISM_ID realism_id   = env::logic::realism_grav_yes_magn_yes; // plays no role

    boost::filesystem::path path_outputs(math::share::phd_outputs_prefix);

    boost::filesystem::path path_folder("nav");
    boost::filesystem::path path_folder2("seeds");
    boost::filesystem::path path_file("seeds_geo_magnetic.txt");
    boost::filesystem::create_directory((path_outputs / path_folder / path_folder2).string());
    std::string st_file = (path_outputs / path_folder / path_folder2 / path_file).string();

    std::ofstream Ostream;
    Ostream.open(st_file);

    std::string st_seeds;
    Eigen::Vector3d B_n_model_nT, B_n_truth_nT;
    env::geodetic_coord x_gdt_rad_m(270.0 * math::constant::D2R(), 45.0 * math::constant::D2R(), 3000.0);
    double t_sec = 0.0;

    for (unsigned short seed_order = seed_init; seed_order != seed_end; seed_order++) {
        math::seeder Oseeder(seed_order);
        auto Pgeo = new env::geo_mix(env_mag_id, realism_id, Oseeder.provide_seed(math::seeder::seeder_geo));

        st_seeds = std::to_string(seed_order);
        switch (st_seeds.size()) {
            case 1:
                st_seeds.insert(0, "0");
                break;
            case 2:
                break;
            default:
                throw std::runtime_error("Incorrect seeds choice.");
        }

        B_n_model_nT = Pgeo->get_mag().compute_B_n_nT_model(t_sec, x_gdt_rad_m);
        B_n_truth_nT = Pgeo->get_mag().compute_B_n_nT_truth(t_sec, x_gdt_rad_m);

        ang::rotv rotv_B_error(B_n_model_nT, B_n_truth_nT);
        double B_error_nT  = fabs(B_n_truth_nT.norm() - B_n_model_nT.norm());
        double B_error_deg = rotv_B_error.norm() * math::constant::R2D();

        Ostream << st_seeds
                << fixed << setw(10) << setprecision(2) << showpos << B_error_nT
                << fixed << setw(10) << setprecision(3) << showpos << B_error_deg * 10
                << endl;
        delete Pgeo;
    }

    Ostream.close();
} // closes test_magnetic

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////




























