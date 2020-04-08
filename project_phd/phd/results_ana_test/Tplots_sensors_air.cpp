#include "Tplots_sensors_air.h"

#include "math/logic/share.h"
#include "math/templates/metrics_.h"
#include "ang/tools.h"
#include "ang/rotate/rotv.h"
#include "env/earth.h"
#include "acft/guid/guid.h"
#include "nav/init/init_error.h"
#include <boost/filesystem.hpp>
#include <string>
#include <fstream>
#include <cmath>

nav::test::Tplots_sensors_air::Tplots_sensors_air(unsigned short& seed_init, unsigned short& seed_end)
: _seed_init(seed_init), _seed_end(seed_end) {
}
/* constructor based on counter */

void nav::test::Tplots_sensors_air::run() {
    obtain_other_air_metrics ("01_01_05_02_03_0100", _seed_init, _seed_end);
    obtain_euler_air_metrics ("01_01_05_02_03_0100", _seed_init, _seed_end);
    obtain_h_air_metrics     ("01_01_05_02_03_0100", _seed_init, _seed_end);
}
/* execute tests and write results on console */

using namespace std;

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////


void nav::test::Tplots_sensors_air:: obtain_other_air_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end) {
    std::string st_folder          = st_first_folder;
    std::string st_folder_one      = "MAIN";
    std::string st_folder_one_air2 = "AIR_WORSE";
    std::string st_folder_one_air3 = "AIR_WORST";

    boost::filesystem::path path_outputs_read(math::share::phd_outputs_store_prefix);
    boost::filesystem::path path_outputs_write(math::share::phd_outputs_prefix);
    std::string st_folder_main("nav");
    std::string st_extra_folder("versus_air");
    std::string st_file_atm("filter_air_atm.txt");
    std::string st_file_gps("filter_gps_wind_DeltaTp.txt");
    std::string st_file_pos("filter_pos_wind_DeltaTp.txt");

    std::string st_seeds, st_file_atm_complete, st_first_line, st_line, st_file_atm_complete_air2, st_file_atm_complete_air3;
    std::string st_file_gps_complete, st_file_gps_complete_air2, st_file_pos_complete, st_file_pos_complete_air2, st_file_gps_complete_air3, st_file_pos_complete_air3;
    unsigned long nel     = (unsigned long)(seed_end) - (unsigned long)(seed_init) + 1;
    unsigned long nel_all = 380001;
    unsigned long nel_sec = 381;
    unsigned long index_all, index_sec;
    std::vector<double> Vt_sec(nel_sec);
    std::vector<std::vector<double>> VVerror_T_degK(nel), VVerror_Hp_m(nel), VVerror_DeltaT_degK(nel);
    std::vector<std::vector<double>> ZZerror_T_degK(nel), ZZerror_Hp_m(nel), ZZerror_DeltaT_degK(nel);
    std::vector<std::vector<double>> YYerror_T_degK(nel), YYerror_Hp_m(nel), YYerror_DeltaT_degK(nel);
    std::vector<double> Verror_T_degK(nel_all), Verror_Hp_m(nel_all), Verror_DeltaT_degK(nel_all);
    std::vector<double> Zerror_T_degK(nel_all), Zerror_Hp_m(nel_all), Zerror_DeltaT_degK(nel_all);
    std::vector<double> Yerror_T_degK(nel_all), Yerror_Hp_m(nel_all), Yerror_DeltaT_degK(nel_all);
    std::vector<double> Vmean_error_T_degK(nel), Vstd_error_T_degK(nel), Vmean_error_Hp_m(nel), Vstd_error_Hp_m(nel), Vmean_error_DeltaT_degK(nel), Vstd_error_DeltaT_degK(nel);
    std::vector<double> Zmean_error_T_degK(nel), Zstd_error_T_degK(nel), Zmean_error_Hp_m(nel), Zstd_error_Hp_m(nel), Zmean_error_DeltaT_degK(nel), Zstd_error_DeltaT_degK(nel);
    std::vector<double> Ymean_error_T_degK(nel), Ystd_error_T_degK(nel), Ymean_error_Hp_m(nel), Ystd_error_Hp_m(nel), Ymean_error_DeltaT_degK(nel), Ystd_error_DeltaT_degK(nel);
    std::vector<double> Vsmax_error_T_degK(nel), Vabs_smax_error_T_degK(nel), Vsmax_error_Hp_m(nel), Vabs_smax_error_Hp_m(nel), Vsmax_error_DeltaT_degK(nel), Vabs_smax_error_DeltaT_degK(nel);
    std::vector<double> Zsmax_error_T_degK(nel), Zabs_smax_error_T_degK(nel), Zsmax_error_Hp_m(nel), Zabs_smax_error_Hp_m(nel), Zsmax_error_DeltaT_degK(nel), Zabs_smax_error_DeltaT_degK(nel);
    std::vector<double> Ysmax_error_T_degK(nel), Yabs_smax_error_T_degK(nel), Ysmax_error_Hp_m(nel), Yabs_smax_error_Hp_m(nel), Ysmax_error_DeltaT_degK(nel), Yabs_smax_error_DeltaT_degK(nel);
    double int_part, dec_part, t_sec;
    std::string st_t_sec, st_T_est_degK, st_T_truth_degK, st_Hp_est_m, st_Hp_truth_m, st_DeltaT_est_degK, st_DeltaT_truth_degK;

    // for each trajectory
    for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
        st_seeds = math::seeder::seed2string(seed);

        st_folder.replace(3,2,st_seeds);
        st_file_atm_complete      = (path_outputs_read / st_folder_main / st_folder_one      / st_folder / st_file_atm).string();
        st_file_atm_complete_air2 = (path_outputs_read / st_folder_main / st_folder_one_air2 / st_folder / st_file_atm).string();
        st_file_atm_complete_air3 = (path_outputs_read / st_folder_main / st_folder_one_air3 / st_folder / st_file_atm).string();

        std::ifstream Ostream_atm;
        Ostream_atm.open(st_file_atm_complete);
        std::getline(Ostream_atm, st_first_line);

        VVerror_T_degK[seed-seed_init].resize(nel_sec); // do not forget
        VVerror_Hp_m[seed-seed_init].resize(nel_sec); // do not forget

        index_all = 0;
        index_sec = 0;

        while (std::getline(Ostream_atm, st_line)) {
            st_t_sec        = st_line.substr(0,10);
            st_T_est_degK   = st_line.substr(12,14);
            st_T_truth_degK = st_line.substr(44,14);
            st_Hp_est_m     = st_line.substr(151,14);
            st_Hp_truth_m   = st_line.substr(183,14);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec / 10, &int_part);
                Verror_T_degK[index_all]   = std::stod(st_T_est_degK) - std::stod(st_T_truth_degK);
                Verror_Hp_m[index_all]     = std::stod(st_Hp_est_m)   - std::stod(st_Hp_truth_m);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec]                         = t_sec;
                    VVerror_T_degK[seed-seed_init][index_sec] = Verror_T_degK[index_all-1];
                    VVerror_Hp_m[seed-seed_init][index_sec]   = Verror_Hp_m[index_all-1];
                    index_sec++;
                }
            }
        }
        Ostream_atm.close();

        std::ifstream Ostream_atm_air2;
        Ostream_atm_air2.open(st_file_atm_complete_air2);
        std::getline(Ostream_atm_air2, st_first_line);

        ZZerror_T_degK[seed-seed_init].resize(nel_sec); // do not forget
        ZZerror_Hp_m[seed-seed_init].resize(nel_sec); // do not forget

        index_all = 0;
        index_sec = 0;

        while (std::getline(Ostream_atm_air2, st_line)) {
            st_t_sec        = st_line.substr(0,10);
            st_T_est_degK   = st_line.substr(12,14);
            st_T_truth_degK = st_line.substr(44,14);
            st_Hp_est_m     = st_line.substr(151,14);
            st_Hp_truth_m   = st_line.substr(183,14);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec / 10, &int_part);
                Zerror_T_degK[index_all]   = std::stod(st_T_est_degK) - std::stod(st_T_truth_degK);
                Zerror_Hp_m[index_all]     = std::stod(st_Hp_est_m)   - std::stod(st_Hp_truth_m);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec]                         = t_sec;
                    ZZerror_T_degK[seed-seed_init][index_sec] = Zerror_T_degK[index_all-1];
                    ZZerror_Hp_m[seed-seed_init][index_sec]   = Zerror_Hp_m[index_all-1];
                    index_sec++;
                }
            }
        }
        Ostream_atm_air2.close();

        std::ifstream Ostream_atm_air3;
        Ostream_atm_air3.open(st_file_atm_complete_air3);
        std::getline(Ostream_atm_air3, st_first_line);

        YYerror_T_degK[seed-seed_init].resize(nel_sec); // do not forget
        YYerror_Hp_m[seed-seed_init].resize(nel_sec); // do not forget

        index_all = 0;
        index_sec = 0;

        while (std::getline(Ostream_atm_air3, st_line)) {
            st_t_sec        = st_line.substr(0,10);
            st_T_est_degK   = st_line.substr(12,14);
            st_T_truth_degK = st_line.substr(44,14);
            st_Hp_est_m     = st_line.substr(151,14);
            st_Hp_truth_m   = st_line.substr(183,14);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec / 10, &int_part);
                Yerror_T_degK[index_all]   = std::stod(st_T_est_degK) - std::stod(st_T_truth_degK);
                Yerror_Hp_m[index_all]     = std::stod(st_Hp_est_m)   - std::stod(st_Hp_truth_m);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec]                         = t_sec;
                    YYerror_T_degK[seed-seed_init][index_sec] = Yerror_T_degK[index_all-1];
                    YYerror_Hp_m[seed-seed_init][index_sec]   = Yerror_Hp_m[index_all-1];
                    index_sec++;
                }
            }
        }
        Ostream_atm_air3.close();

        st_file_gps_complete      = (path_outputs_read / st_folder_main / st_folder_one      / st_folder / st_file_gps).string();
        st_file_gps_complete_air2 = (path_outputs_read / st_folder_main / st_folder_one_air2 / st_folder / st_file_gps).string();
        st_file_gps_complete_air3 = (path_outputs_read / st_folder_main / st_folder_one_air3 / st_folder / st_file_gps).string();
        st_file_pos_complete      = (path_outputs_read / st_folder_main / st_folder_one      / st_folder / st_file_pos).string();
        st_file_pos_complete_air2 = (path_outputs_read / st_folder_main / st_folder_one_air2 / st_folder / st_file_pos).string();
        st_file_pos_complete_air3 = (path_outputs_read / st_folder_main / st_folder_one_air3 / st_folder / st_file_pos).string();

        std::ifstream Ostream_gps;
        Ostream_gps.open(st_file_gps_complete);
        std::getline(Ostream_gps, st_first_line);

        VVerror_DeltaT_degK[seed-seed_init].resize(nel_sec); // do not forget

        index_all = 0;
        index_sec = 0;

        while (std::getline(Ostream_gps, st_line)) {
            st_t_sec = st_line.substr(0, 10);
            st_DeltaT_est_degK = st_line.substr(96, 12);
            st_DeltaT_truth_degK = st_line.substr(110, 12);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec / 10, &int_part);
                Verror_DeltaT_degK[index_all] = std::stod(st_DeltaT_est_degK) - std::stod(st_DeltaT_truth_degK);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = t_sec;
                    VVerror_DeltaT_degK[seed - seed_init][index_sec] = Verror_DeltaT_degK[index_all - 1];
                    index_sec++;
                }
            }
        }
        Ostream_gps.close();

        std::ifstream Ostream_pos;
        Ostream_pos.open(st_file_pos_complete);
        std::getline(Ostream_pos, st_first_line);
        std::getline(Ostream_pos, st_line); // line repeated at 100 [sec] from previous file
        while (std::getline(Ostream_pos, st_line)) {
            st_t_sec             = st_line.substr(0,10);
            st_DeltaT_est_degK   = st_line.substr(96,12);
            st_DeltaT_truth_degK = st_line.substr(110,12);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec / 10, &int_part);
                Verror_DeltaT_degK[index_all] = std::stod(st_DeltaT_est_degK) - std::stod(st_DeltaT_truth_degK);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = t_sec;
                    VVerror_DeltaT_degK[seed - seed_init][index_sec] = Verror_DeltaT_degK[index_all - 1];
                    index_sec++;
                }
            }
        }
        Ostream_pos.close();

        std::ifstream Ostream_gps_air2;
        Ostream_gps_air2.open(st_file_gps_complete_air2);
        std::getline(Ostream_gps_air2, st_first_line);

        ZZerror_DeltaT_degK[seed-seed_init].resize(nel_sec); // do not forget

        index_all = 0;
        index_sec = 0;

        while (std::getline(Ostream_gps_air2, st_line)) {
            st_t_sec = st_line.substr(0, 10);
            st_DeltaT_est_degK = st_line.substr(96, 12);
            st_DeltaT_truth_degK = st_line.substr(110, 12);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec / 10, &int_part);
                Zerror_DeltaT_degK[index_all] = std::stod(st_DeltaT_est_degK) - std::stod(st_DeltaT_truth_degK);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = t_sec;
                    ZZerror_DeltaT_degK[seed - seed_init][index_sec] = Zerror_DeltaT_degK[index_all - 1];
                    index_sec++;
                }
            }
        }
        Ostream_gps_air2.close();

        std::ifstream Ostream_pos_air2;
        Ostream_pos_air2.open(st_file_pos_complete_air2);
        std::getline(Ostream_pos_air2, st_first_line);
        std::getline(Ostream_pos_air2, st_line); // line repeated at 100 [sec] from previous file
        while (std::getline(Ostream_pos_air2, st_line)) {
            st_t_sec             = st_line.substr(0,10);
            st_DeltaT_est_degK   = st_line.substr(96,12);
            st_DeltaT_truth_degK = st_line.substr(110,12);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec / 10, &int_part);
                Zerror_DeltaT_degK[index_all] = std::stod(st_DeltaT_est_degK) - std::stod(st_DeltaT_truth_degK);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = t_sec;
                    ZZerror_DeltaT_degK[seed - seed_init][index_sec] = Zerror_DeltaT_degK[index_all - 1];
                    index_sec++;
                }
            }
        }
        Ostream_pos_air2.close();

        std::ifstream Ostream_gps_air3;
        Ostream_gps_air3.open(st_file_gps_complete_air3);
        std::getline(Ostream_gps_air3, st_first_line);

        YYerror_DeltaT_degK[seed-seed_init].resize(nel_sec); // do not forget

        index_all = 0;
        index_sec = 0;

        while (std::getline(Ostream_gps_air3, st_line)) {
            st_t_sec = st_line.substr(0, 10);
            st_DeltaT_est_degK = st_line.substr(96, 12);
            st_DeltaT_truth_degK = st_line.substr(110, 12);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec / 10, &int_part);
                Yerror_DeltaT_degK[index_all] = std::stod(st_DeltaT_est_degK) - std::stod(st_DeltaT_truth_degK);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = t_sec;
                    YYerror_DeltaT_degK[seed - seed_init][index_sec] = Yerror_DeltaT_degK[index_all - 1];
                    index_sec++;
                }
            }
        }
        Ostream_gps_air3.close();

        std::ifstream Ostream_pos_air3;
        Ostream_pos_air3.open(st_file_pos_complete_air3);
        std::getline(Ostream_pos_air3, st_first_line);
        std::getline(Ostream_pos_air3, st_line); // line repeated at 100 [sec] from previous file
        while (std::getline(Ostream_pos_air3, st_line)) {
            st_t_sec             = st_line.substr(0,10);
            st_DeltaT_est_degK   = st_line.substr(96,12);
            st_DeltaT_truth_degK = st_line.substr(110,12);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec / 10, &int_part);
                Yerror_DeltaT_degK[index_all] = std::stod(st_DeltaT_est_degK) - std::stod(st_DeltaT_truth_degK);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = t_sec;
                    YYerror_DeltaT_degK[seed - seed_init][index_sec] = Yerror_DeltaT_degK[index_all - 1];
                    index_sec++;
                }
            }
        }
        Ostream_pos_air3.close();

        // compute metrics for each trajectory
        Vmean_error_T_degK[seed-seed_init]      = math::mean(Verror_T_degK);
        Vstd_error_T_degK[seed-seed_init]       = math::std(Verror_T_degK, Vmean_error_T_degK[seed-seed_init]);
        Vsmax_error_T_degK[seed-seed_init]      = math::smax(Verror_T_degK);
        Vabs_smax_error_T_degK[seed-seed_init]  = fabs(Vsmax_error_T_degK[seed-seed_init]);

        Vmean_error_Hp_m[seed-seed_init]      = math::mean(Verror_Hp_m);
        Vstd_error_Hp_m[seed-seed_init]       = math::std(Verror_Hp_m, Vmean_error_Hp_m[seed-seed_init]);
        Vsmax_error_Hp_m[seed-seed_init]      = math::smax(Verror_Hp_m);
        Vabs_smax_error_Hp_m[seed-seed_init]  = fabs(Vsmax_error_Hp_m[seed-seed_init]);

        Vmean_error_DeltaT_degK[seed-seed_init]      = math::mean(Verror_DeltaT_degK);
        Vstd_error_DeltaT_degK[seed-seed_init]       = math::std(Verror_DeltaT_degK, Vmean_error_DeltaT_degK[seed-seed_init]);
        Vsmax_error_DeltaT_degK[seed-seed_init]      = math::smax(Verror_DeltaT_degK);
        Vabs_smax_error_DeltaT_degK[seed-seed_init]  = fabs(Vsmax_error_DeltaT_degK[seed-seed_init]);

        Zmean_error_T_degK[seed-seed_init]      = math::mean(Zerror_T_degK);
        Zstd_error_T_degK[seed-seed_init]       = math::std(Zerror_T_degK, Zmean_error_T_degK[seed-seed_init]);
        Zsmax_error_T_degK[seed-seed_init]      = math::smax(Zerror_T_degK);
        Zabs_smax_error_T_degK[seed-seed_init]  = fabs(Zsmax_error_T_degK[seed-seed_init]);

        Zmean_error_Hp_m[seed-seed_init]      = math::mean(Zerror_Hp_m);
        Zstd_error_Hp_m[seed-seed_init]       = math::std(Zerror_Hp_m, Zmean_error_Hp_m[seed-seed_init]);
        Zsmax_error_Hp_m[seed-seed_init]      = math::smax(Zerror_Hp_m);
        Zabs_smax_error_Hp_m[seed-seed_init]  = fabs(Zsmax_error_Hp_m[seed-seed_init]);

        Zmean_error_DeltaT_degK[seed-seed_init]      = math::mean(Zerror_DeltaT_degK);
        Zstd_error_DeltaT_degK[seed-seed_init]       = math::std(Zerror_DeltaT_degK, Zmean_error_DeltaT_degK[seed-seed_init]);
        Zsmax_error_DeltaT_degK[seed-seed_init]      = math::smax(Zerror_DeltaT_degK);
        Zabs_smax_error_DeltaT_degK[seed-seed_init]  = fabs(Zsmax_error_DeltaT_degK[seed-seed_init]);

        Ymean_error_T_degK[seed-seed_init]      = math::mean(Yerror_T_degK);
        Ystd_error_T_degK[seed-seed_init]       = math::std(Yerror_T_degK, Ymean_error_T_degK[seed-seed_init]);
        Ysmax_error_T_degK[seed-seed_init]      = math::smax(Yerror_T_degK);
        Yabs_smax_error_T_degK[seed-seed_init]  = fabs(Ysmax_error_T_degK[seed-seed_init]);

        Ymean_error_Hp_m[seed-seed_init]      = math::mean(Yerror_Hp_m);
        Ystd_error_Hp_m[seed-seed_init]       = math::std(Yerror_Hp_m, Ymean_error_Hp_m[seed-seed_init]);
        Ysmax_error_Hp_m[seed-seed_init]      = math::smax(Yerror_Hp_m);
        Yabs_smax_error_Hp_m[seed-seed_init]  = fabs(Ysmax_error_Hp_m[seed-seed_init]);

        Ymean_error_DeltaT_degK[seed-seed_init]      = math::mean(Yerror_DeltaT_degK);
        Ystd_error_DeltaT_degK[seed-seed_init]       = math::std(Yerror_DeltaT_degK, Ymean_error_DeltaT_degK[seed-seed_init]);
        Ysmax_error_DeltaT_degK[seed-seed_init]      = math::smax(Yerror_DeltaT_degK);
        Yabs_smax_error_DeltaT_degK[seed-seed_init]  = fabs(Ysmax_error_DeltaT_degK[seed-seed_init]);
    }

    // obtain aggregated metrics
    double mean_mean_error_T_degK      = math::mean(Vmean_error_T_degK);
    double std_mean_error_T_degK       = math::std(Vmean_error_T_degK, mean_mean_error_T_degK);
    double smax_mean_error_T_degK      = math::smax(Vmean_error_T_degK);
    double mean_mean_error_Hp_m        = math::mean(Vmean_error_Hp_m);
    double std_mean_error_Hp_m         = math::std(Vmean_error_Hp_m, mean_mean_error_Hp_m);
    double smax_mean_error_Hp_m        = math::smax(Vmean_error_Hp_m);
    double mean_mean_error_DeltaT_degK = math::mean(Vmean_error_DeltaT_degK);
    double std_mean_error_DeltaT_degK  = math::std(Vmean_error_DeltaT_degK, mean_mean_error_DeltaT_degK);
    double smax_mean_error_DeltaT_degK = math::smax(Vmean_error_DeltaT_degK);

    double mean_std_error_T_degK      = math::mean(Vstd_error_T_degK);
    double std_std_error_T_degK       = math::std(Vstd_error_T_degK, mean_std_error_T_degK);
    double smax_std_error_T_degK      = math::smax(Vstd_error_T_degK);
    double mean_std_error_Hp_m        = math::mean(Vstd_error_Hp_m);
    double std_std_error_Hp_m         = math::std(Vstd_error_Hp_m, mean_std_error_Hp_m);
    double smax_std_error_Hp_m        = math::smax(Vstd_error_Hp_m);
    double mean_std_error_DeltaT_degK = math::mean(Vstd_error_DeltaT_degK);
    double std_std_error_DeltaT_degK  = math::std(Vstd_error_DeltaT_degK, mean_std_error_DeltaT_degK);
    double smax_std_error_DeltaT_degK = math::smax(Vstd_error_DeltaT_degK);

    double mean_smax_error_T_degK      = math::mean(Vabs_smax_error_T_degK);
    double std_smax_error_T_degK       = math::std(Vabs_smax_error_T_degK, mean_smax_error_T_degK);
    double smax_smax_error_T_degK      = math::smax(Vsmax_error_T_degK);
    double mean_smax_error_Hp_m        = math::mean(Vabs_smax_error_Hp_m);
    double std_smax_error_Hp_m         = math::std(Vabs_smax_error_Hp_m, mean_smax_error_Hp_m);
    double smax_smax_error_Hp_m        = math::smax(Vsmax_error_Hp_m);
    double mean_smax_error_DeltaT_degK = math::mean(Vabs_smax_error_DeltaT_degK);
    double std_smax_error_DeltaT_degK  = math::std(Vabs_smax_error_DeltaT_degK, mean_smax_error_DeltaT_degK);
    double smax_smax_error_DeltaT_degK = math::smax(Vsmax_error_DeltaT_degK);

    double mean_mean_error_T_degK_air2      = math::mean(Zmean_error_T_degK);
    double std_mean_error_T_degK_air2       = math::std(Zmean_error_T_degK, mean_mean_error_T_degK_air2);
    double smax_mean_error_T_degK_air2      = math::smax(Zmean_error_T_degK);
    double mean_mean_error_Hp_m_air2        = math::mean(Zmean_error_Hp_m);
    double std_mean_error_Hp_m_air2         = math::std(Zmean_error_Hp_m, mean_mean_error_Hp_m_air2);
    double smax_mean_error_Hp_m_air2        = math::smax(Zmean_error_Hp_m);
    double mean_mean_error_DeltaT_degK_air2 = math::mean(Zmean_error_DeltaT_degK);
    double std_mean_error_DeltaT_degK_air2  = math::std(Zmean_error_DeltaT_degK, mean_mean_error_DeltaT_degK_air2);
    double smax_mean_error_DeltaT_degK_air2 = math::smax(Zmean_error_DeltaT_degK);

    double mean_std_error_T_degK_air2      = math::mean(Zstd_error_T_degK);
    double std_std_error_T_degK_air2       = math::std(Zstd_error_T_degK, mean_std_error_T_degK_air2);
    double smax_std_error_T_degK_air2      = math::smax(Zstd_error_T_degK);
    double mean_std_error_Hp_m_air2        = math::mean(Zstd_error_Hp_m);
    double std_std_error_Hp_m_air2         = math::std(Zstd_error_Hp_m, mean_std_error_Hp_m_air2);
    double smax_std_error_Hp_m_air2        = math::smax(Zstd_error_Hp_m);
    double mean_std_error_DeltaT_degK_air2 = math::mean(Zstd_error_DeltaT_degK);
    double std_std_error_DeltaT_degK_air2  = math::std(Zstd_error_DeltaT_degK, mean_std_error_DeltaT_degK_air2);
    double smax_std_error_DeltaT_degK_air2 = math::smax(Zstd_error_DeltaT_degK);

    double mean_smax_error_T_degK_air2      = math::mean(Zabs_smax_error_T_degK);
    double std_smax_error_T_degK_air2       = math::std(Zabs_smax_error_T_degK, mean_smax_error_T_degK_air2);
    double smax_smax_error_T_degK_air2      = math::smax(Zsmax_error_T_degK);
    double mean_smax_error_Hp_m_air2        = math::mean(Zabs_smax_error_Hp_m);
    double std_smax_error_Hp_m_air2         = math::std(Zabs_smax_error_Hp_m, mean_smax_error_Hp_m_air2);
    double smax_smax_error_Hp_m_air2        = math::smax(Zsmax_error_Hp_m);
    double mean_smax_error_DeltaT_degK_air2 = math::mean(Zabs_smax_error_DeltaT_degK);
    double std_smax_error_DeltaT_degK_air2  = math::std(Zabs_smax_error_DeltaT_degK, mean_smax_error_DeltaT_degK_air2);
    double smax_smax_error_DeltaT_degK_air2 = math::smax(Zsmax_error_DeltaT_degK);

    double mean_mean_error_T_degK_air3      = math::mean(Ymean_error_T_degK);
    double std_mean_error_T_degK_air3       = math::std(Ymean_error_T_degK, mean_mean_error_T_degK_air3);
    double smax_mean_error_T_degK_air3      = math::smax(Ymean_error_T_degK);
    double mean_mean_error_Hp_m_air3        = math::mean(Ymean_error_Hp_m);
    double std_mean_error_Hp_m_air3         = math::std(Ymean_error_Hp_m, mean_mean_error_Hp_m_air3);
    double smax_mean_error_Hp_m_air3        = math::smax(Ymean_error_Hp_m);
    double mean_mean_error_DeltaT_degK_air3 = math::mean(Ymean_error_DeltaT_degK);
    double std_mean_error_DeltaT_degK_air3  = math::std(Ymean_error_DeltaT_degK, mean_mean_error_DeltaT_degK_air3);
    double smax_mean_error_DeltaT_degK_air3 = math::smax(Ymean_error_DeltaT_degK);

    double mean_std_error_T_degK_air3      = math::mean(Ystd_error_T_degK);
    double std_std_error_T_degK_air3       = math::std(Ystd_error_T_degK, mean_std_error_T_degK_air3);
    double smax_std_error_T_degK_air3      = math::smax(Ystd_error_T_degK);
    double mean_std_error_Hp_m_air3        = math::mean(Ystd_error_Hp_m);
    double std_std_error_Hp_m_air3         = math::std(Ystd_error_Hp_m, mean_std_error_Hp_m_air3);
    double smax_std_error_Hp_m_air3        = math::smax(Ystd_error_Hp_m);
    double mean_std_error_DeltaT_degK_air3 = math::mean(Ystd_error_DeltaT_degK);
    double std_std_error_DeltaT_degK_air3  = math::std(Ystd_error_DeltaT_degK, mean_std_error_DeltaT_degK_air3);
    double smax_std_error_DeltaT_degK_air3 = math::smax(Ystd_error_DeltaT_degK);

    double mean_smax_error_T_degK_air3      = math::mean(Yabs_smax_error_T_degK);
    double std_smax_error_T_degK_air3       = math::std(Yabs_smax_error_T_degK, mean_smax_error_T_degK_air3);
    double smax_smax_error_T_degK_air3      = math::smax(Ysmax_error_T_degK);
    double mean_smax_error_Hp_m_air3        = math::mean(Yabs_smax_error_Hp_m);
    double std_smax_error_Hp_m_air3         = math::std(Yabs_smax_error_Hp_m, mean_smax_error_Hp_m_air3);
    double smax_smax_error_Hp_m_air3        = math::smax(Ysmax_error_Hp_m);
    double mean_smax_error_DeltaT_degK_air3 = math::mean(Yabs_smax_error_DeltaT_degK);
    double std_smax_error_DeltaT_degK_air3  = math::std(Yabs_smax_error_DeltaT_degK, mean_smax_error_DeltaT_degK_air3);
    double smax_smax_error_DeltaT_degK_air3 = math::smax(Ysmax_error_DeltaT_degK);

    std::cout << std::endl;
    std::cout << "T [degK] air data filter errors (est - truth)" << std::endl;
    std::cout << setw(10) << " " << setw(22) << "baseline" << setw(22) << "OSP-OAT worse" << setw(22) << "OSP-OAT worst"  << std::endl;
    std::cout << setw(10) << " "
              << setw(8) << "mean" << setw(7) << "std" << setw(7) << "smax" << setw(3)
              << setw(8) << "mean" << setw(7) << "std" << setw(7) << "smax" << setw(3)
              << setw(8) << "mean" << setw(7) << "std" << setw(7) << "smax" << setw(3)
              << std::endl;

    std::cout << "Mean:     "
              << fixed << setw(8) << setprecision(3) <<   showpos << mean_mean_error_T_degK
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_std_error_T_degK
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_smax_error_T_degK
              << fixed << setw(8) << setprecision(3) <<   showpos << mean_mean_error_T_degK_air2
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_std_error_T_degK_air2
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_smax_error_T_degK_air2
              << fixed << setw(8) << setprecision(3) <<   showpos << mean_mean_error_T_degK_air3
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_std_error_T_degK_air3
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_smax_error_T_degK_air3
              << std::endl;
    std::cout << "std:      "
              << fixed << setw(8) << setprecision(3) << noshowpos << std_mean_error_T_degK
              << fixed << setw(7) << setprecision(3) << noshowpos << std_std_error_T_degK
              << fixed << setw(7) << setprecision(3) << noshowpos << std_smax_error_T_degK
              << fixed << setw(8) << setprecision(3) << noshowpos << std_mean_error_T_degK_air2
              << fixed << setw(7) << setprecision(3) << noshowpos << std_std_error_T_degK_air2
              << fixed << setw(7) << setprecision(3) << noshowpos << std_smax_error_T_degK_air2
              << fixed << setw(8) << setprecision(3) << noshowpos << std_mean_error_T_degK_air3
              << fixed << setw(7) << setprecision(3) << noshowpos << std_std_error_T_degK_air3
              << fixed << setw(7) << setprecision(3) << noshowpos << std_smax_error_T_degK_air3
              << std::endl;
    std::cout << "smax:     "
              << fixed << setw(8) << setprecision(3) <<   showpos << smax_mean_error_T_degK
              << fixed << setw(7) << setprecision(3) << noshowpos << smax_std_error_T_degK
              << fixed << setw(7) << setprecision(3) <<   showpos << smax_smax_error_T_degK
              << fixed << setw(8) << setprecision(3) <<   showpos << smax_mean_error_T_degK_air2
              << fixed << setw(7) << setprecision(3) << noshowpos << smax_std_error_T_degK_air2
              << fixed << setw(7) << setprecision(3) <<   showpos << smax_smax_error_T_degK_air2
              << fixed << setw(8) << setprecision(3) <<   showpos << smax_mean_error_T_degK_air3
              << fixed << setw(7) << setprecision(3) << noshowpos << smax_std_error_T_degK_air3
              << fixed << setw(7) << setprecision(3) <<   showpos << smax_smax_error_T_degK_air3
              << std::endl;
    std::cout << std::endl;
    std::cout << "Hp [m] air data filter errors (est - truth)" << std::endl;
    std::cout << setw(10) << " " << setw(22) << "baseline" << setw(22) << "OSP-OAT worse" << setw(22) << "OSP-OAT worst"  << std::endl;
    std::cout << setw(10) << " "
              << setw(7) << "mean" << setw(6) << "std" << setw(7) << "smax" << setw(3)
              << setw(7) << "mean" << setw(6) << "std" << setw(7) << "smax" << setw(3)
              << setw(8) << "mean" << setw(7) << "std" << setw(7) << "smax" << setw(3)
              << std::endl;
    std::cout << "mean:     "
              << fixed << setw(7) << setprecision(2) <<   showpos << mean_mean_error_Hp_m
              << fixed << setw(6) << setprecision(2) << noshowpos << mean_std_error_Hp_m
              << fixed << setw(7) << setprecision(2) << noshowpos << mean_smax_error_Hp_m
              << fixed << setw(7) << setprecision(2) <<   showpos << mean_mean_error_Hp_m_air2
              << fixed << setw(6) << setprecision(2) << noshowpos << mean_std_error_Hp_m_air2
              << fixed << setw(7) << setprecision(2) << noshowpos << mean_smax_error_Hp_m_air2
              << fixed << setw(7) << setprecision(2) <<   showpos << mean_mean_error_Hp_m_air3
              << fixed << setw(6) << setprecision(2) << noshowpos << mean_std_error_Hp_m_air3
              << fixed << setw(7) << setprecision(2) << noshowpos << mean_smax_error_Hp_m_air3
              << std::endl;
    std::cout << "std:      "
              << fixed << setw(7) << setprecision(2) << noshowpos << std_mean_error_Hp_m
              << fixed << setw(6) << setprecision(2) << noshowpos << std_std_error_Hp_m
              << fixed << setw(7) << setprecision(2) << noshowpos << std_smax_error_Hp_m
              << fixed << setw(7) << setprecision(2) << noshowpos << std_mean_error_Hp_m_air2
              << fixed << setw(6) << setprecision(2) << noshowpos << std_std_error_Hp_m_air2
              << fixed << setw(7) << setprecision(2) << noshowpos << std_smax_error_Hp_m_air2
              << fixed << setw(7) << setprecision(2) << noshowpos << std_mean_error_Hp_m_air3
              << fixed << setw(6) << setprecision(2) << noshowpos << std_std_error_Hp_m_air3
              << fixed << setw(7) << setprecision(2) << noshowpos << std_smax_error_Hp_m_air3
              << std::endl;
    std::cout << "smax:     "
              << fixed << setw(7) << setprecision(2) <<   showpos << smax_mean_error_Hp_m
              << fixed << setw(6) << setprecision(2) << noshowpos << smax_std_error_Hp_m
              << fixed << setw(7) << setprecision(2) <<   showpos << smax_smax_error_Hp_m
              << fixed << setw(7) << setprecision(2) <<   showpos << smax_mean_error_Hp_m_air2
              << fixed << setw(6) << setprecision(2) << noshowpos << smax_std_error_Hp_m_air2
              << fixed << setw(7) << setprecision(2) <<   showpos << smax_smax_error_Hp_m_air2
              << fixed << setw(7) << setprecision(2) <<   showpos << smax_mean_error_Hp_m_air3
              << fixed << setw(6) << setprecision(2) << noshowpos << smax_std_error_Hp_m_air3
              << fixed << setw(7) << setprecision(2) <<   showpos << smax_smax_error_Hp_m_air3
              << std::endl;

    std::cout << std::endl;
    std::cout << "DeltaT [degK] air data filter errors (est - truth)" << std::endl;
    std::cout << setw(10) << " " << setw(22) << "baseline" << setw(22) << "OSP-OAT worse" << setw(22) << "OSP-OAT worst"  << std::endl;
    std::cout << setw(10) << " "
              << setw(8) << "mean" << setw(7) << "std" << setw(7) << "smax" << setw(3)
              << setw(8) << "mean" << setw(7) << "std" << setw(7) << "smax" << setw(3)
              << setw(8) << "mean" << setw(7) << "std" << setw(7) << "smax" << setw(3)
              << std::endl;
    std::cout << "std:      "
              << fixed << setw(8) << setprecision(3) <<   showpos << mean_mean_error_DeltaT_degK
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_std_error_DeltaT_degK
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_smax_error_DeltaT_degK
              << fixed << setw(8) << setprecision(3) <<   showpos << mean_mean_error_DeltaT_degK_air2
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_std_error_DeltaT_degK_air2
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_smax_error_DeltaT_degK_air2
              << fixed << setw(8) << setprecision(3) <<   showpos << mean_mean_error_DeltaT_degK_air3
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_std_error_DeltaT_degK_air3
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_smax_error_DeltaT_degK_air3
              << std::endl;
    std::cout << "std:      "
              << fixed << setw(8) << setprecision(3) << noshowpos << std_mean_error_DeltaT_degK
              << fixed << setw(7) << setprecision(3) << noshowpos << std_std_error_DeltaT_degK
              << fixed << setw(7) << setprecision(3) << noshowpos << std_smax_error_DeltaT_degK
              << fixed << setw(8) << setprecision(3) << noshowpos << std_mean_error_DeltaT_degK_air2
              << fixed << setw(7) << setprecision(3) << noshowpos << std_std_error_DeltaT_degK_air2
              << fixed << setw(7) << setprecision(3) << noshowpos << std_smax_error_DeltaT_degK_air2
              << fixed << setw(8) << setprecision(3) << noshowpos << std_mean_error_DeltaT_degK_air3
              << fixed << setw(7) << setprecision(3) << noshowpos << std_std_error_DeltaT_degK_air3
              << fixed << setw(7) << setprecision(3) << noshowpos << std_smax_error_DeltaT_degK_air3
              << std::endl;
    std::cout << "smax:     "
              << fixed << setw(8) << setprecision(3) <<   showpos << smax_mean_error_DeltaT_degK
              << fixed << setw(7) << setprecision(3) << noshowpos << smax_std_error_DeltaT_degK
              << fixed << setw(7) << setprecision(3) <<   showpos << smax_smax_error_DeltaT_degK
              << fixed << setw(8) << setprecision(3) <<   showpos << smax_mean_error_DeltaT_degK_air2
              << fixed << setw(7) << setprecision(3) << noshowpos << smax_std_error_DeltaT_degK_air2
              << fixed << setw(7) << setprecision(3) <<   showpos << smax_smax_error_DeltaT_degK_air2
              << fixed << setw(8) << setprecision(3) <<   showpos << smax_mean_error_DeltaT_degK_air3
              << fixed << setw(7) << setprecision(3) << noshowpos << smax_std_error_DeltaT_degK_air3
              << fixed << setw(7) << setprecision(3) <<   showpos << smax_smax_error_DeltaT_degK_air3
              << std::endl;

    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    boost::filesystem::path path_folder(path_outputs_write / st_folder_main / st_extra_folder);
    boost::filesystem::create_directory(path_folder);

    std::string st_file_T_Hp_DeltaT_out_tex    = "versus_air_T_Hp_DeltaT_table.tex";
    std::string st_file_T_Hp_DeltaT_output_tex = (path_outputs_write / st_folder_main / st_extra_folder / st_file_T_Hp_DeltaT_out_tex).string();

    ofstream Oout_tex;
    Oout_tex.open(st_file_T_Hp_DeltaT_output_tex);

    Oout_tex << "\\begin{center}" << std::endl;
    Oout_tex << "\\begin{tabular}{lrrrrrrrrr}" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\textbf{Benchmark} & \\multicolumn{9}{c}{\\textbf{OSP-OAT Configuration}} \\\\" << std::endl;
    Oout_tex << "\\nm{\\Test - T \\, \\lrsb{10^{-1} \\cdot ^{\\circ}K}} & \\multicolumn{3}{c}{\\textbf{Baseline}} & \\multicolumn{3}{c}{\\textbf{Worse}} & \\multicolumn{3}{c}{\\textbf{Worst}} \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\nm{\\mumu{T}, \\, \\musigma{T}, \\, \\mumax{T}} "
             << " & "
             << fixed << setprecision(2) <<   showpos << mean_mean_error_T_degK * 10
             << " & \\textbf{"
             << fixed << setprecision(2) << noshowpos << mean_std_error_T_degK * 10
             << "} & "
             << fixed << setprecision(2) << noshowpos << mean_smax_error_T_degK * 10
             << " & "
             << fixed << setprecision(2) <<   showpos << mean_mean_error_T_degK_air2 * 10
             << " & \\textbf{"
             << fixed << setprecision(2) << noshowpos << mean_std_error_T_degK_air2 * 10
             << "} & "
             << fixed << setprecision(2) << noshowpos << mean_smax_error_T_degK_air2 * 10
             << " & "
             << fixed << setprecision(2) <<   showpos << mean_mean_error_T_degK_air3 * 10
             << " & \\textbf{"
             << fixed << setprecision(2) << noshowpos << mean_std_error_T_degK_air3 * 10
             << "} & "
             << fixed << setprecision(2) << noshowpos << mean_smax_error_T_degK_air3 * 10
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\sigmamu{T}, \\, \\sigmasigma{T}, \\, \\sigmamax{T}} "
             << " & "
             << fixed << setprecision(2) << noshowpos << std_mean_error_T_degK * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << std_std_error_T_degK * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << std_smax_error_T_degK * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << std_mean_error_T_degK_air2 * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << std_std_error_T_degK_air2 * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << std_smax_error_T_degK_air2 * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << std_mean_error_T_degK_air3 * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << std_std_error_T_degK_air3 * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << std_smax_error_T_degK_air3 * 10
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\maxmu{T}, \\, \\maxsigma{T}, \\, \\maxmax{T}} "
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_mean_error_T_degK * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << smax_std_error_T_degK * 10
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_smax_error_T_degK * 10
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_mean_error_T_degK_air2 * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << smax_std_error_T_degK_air2 * 10
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_smax_error_T_degK_air2 * 10
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_mean_error_T_degK_air3 * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << smax_std_error_T_degK_air3 * 10
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_smax_error_T_degK_air3 * 10
             << " \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\nm{\\Hpest - \\Hp \\, \\lrsb{m}} & \\multicolumn{3}{c}{\\textbf{Baseline}} & \\multicolumn{3}{c}{\\textbf{Worse}} & \\multicolumn{3}{c}{\\textbf{Worst}} \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\nm{\\mumu{\\Hp}, \\, \\musigma{\\Hp}, \\, \\mumax{\\Hp}} "
             << " & "
             << fixed << setprecision(2) <<   showpos << mean_mean_error_Hp_m
             << " & \\textbf{"
             << fixed << setprecision(2) << noshowpos << mean_std_error_Hp_m
             << "} & "
             << fixed << setprecision(2) << noshowpos << mean_smax_error_Hp_m
             << " & "
             << fixed << setprecision(2) <<   showpos << mean_mean_error_Hp_m_air2
             << " & \\textbf{"
             << fixed << setprecision(2) << noshowpos << mean_std_error_Hp_m_air2
             << "} & "
             << fixed << setprecision(2) << noshowpos << mean_smax_error_Hp_m_air2
             << " & "
             << fixed << setprecision(2) <<   showpos << mean_mean_error_Hp_m_air3
             << " & \\textbf{"
             << fixed << setprecision(2) << noshowpos << mean_std_error_Hp_m_air3
             << "} & "
             << fixed << setprecision(2) << noshowpos << mean_smax_error_Hp_m_air3
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\sigmamu{\\Hp}, \\, \\sigmasigma{\\Hp}, \\, \\sigmamax{\\Hp}} "
             << " & "
             << fixed << setprecision(2) << noshowpos << std_mean_error_Hp_m
             << " & "
             << fixed << setprecision(2) << noshowpos << std_std_error_Hp_m
             << " & "
             << fixed << setprecision(2) << noshowpos << std_smax_error_Hp_m
             << " & "
             << fixed << setprecision(2) << noshowpos << std_mean_error_Hp_m_air2
             << " & "
             << fixed << setprecision(2) << noshowpos << std_std_error_Hp_m_air2
             << " & "
             << fixed << setprecision(2) << noshowpos << std_smax_error_Hp_m_air2
             << " & "
             << fixed << setprecision(2) << noshowpos << std_mean_error_Hp_m_air3
             << " & "
             << fixed << setprecision(2) << noshowpos << std_std_error_Hp_m_air3
             << " & "
             << fixed << setprecision(2) << noshowpos << std_smax_error_Hp_m_air3
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\maxmu{\\Hp}, \\, \\maxsigma{\\Hp}, \\, \\maxmax{\\Hp}} "
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_mean_error_Hp_m
             << " & "
             << fixed << setprecision(2) << noshowpos << smax_std_error_Hp_m
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_smax_error_Hp_m
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_mean_error_Hp_m_air2
             << " & "
             << fixed << setprecision(2) << noshowpos << smax_std_error_Hp_m_air2
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_smax_error_Hp_m_air2
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_mean_error_Hp_m_air3
             << " & "
             << fixed << setprecision(2) << noshowpos << smax_std_error_Hp_m_air3
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_smax_error_Hp_m_air3
             << " \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\nm{\\DeltaTest - \\DeltaT \\, \\lrsb{10^{-1} \\cdot ^{\\circ}K}} & \\multicolumn{3}{c}{\\textbf{Baseline}} & \\multicolumn{3}{c}{\\textbf{Worse}} & \\multicolumn{3}{c}{\\textbf{Worst}} \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\nm{\\mumu{\\DeltaT}, \\, \\musigma{\\DeltaT}, \\, \\mumax{\\DeltaT}} "
             << " & "
             << fixed << setprecision(2) <<   showpos << mean_mean_error_DeltaT_degK * 10
             << " & \\textbf{"
             << fixed << setprecision(2) << noshowpos << mean_std_error_DeltaT_degK * 10
             << "} & "
             << fixed << setprecision(2) << noshowpos << mean_smax_error_DeltaT_degK * 10
             << " & "
             << fixed << setprecision(2) <<   showpos << mean_mean_error_DeltaT_degK_air2 * 10
             << " & \\textbf{"
             << fixed << setprecision(2) << noshowpos << mean_std_error_DeltaT_degK_air2 * 10
             << "} & "
             << fixed << setprecision(2) << noshowpos << mean_smax_error_DeltaT_degK_air2 * 10
             << " & "
             << fixed << setprecision(2) <<   showpos << mean_mean_error_DeltaT_degK_air3 * 10
             << " & \\textbf{"
             << fixed << setprecision(2) << noshowpos << mean_std_error_DeltaT_degK_air3 * 10
             << "} & "
             << fixed << setprecision(2) << noshowpos << mean_smax_error_DeltaT_degK_air3 * 10
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\sigmamu{\\DeltaT}, \\, \\sigmasigma{\\DeltaT}, \\, \\sigmamax{\\DeltaT}} "
             << " & "
             << fixed << setprecision(2) << noshowpos << std_mean_error_DeltaT_degK * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << std_std_error_DeltaT_degK * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << std_smax_error_DeltaT_degK * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << std_mean_error_DeltaT_degK_air2 * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << std_std_error_DeltaT_degK_air2 * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << std_smax_error_DeltaT_degK_air2 * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << std_mean_error_DeltaT_degK_air3 * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << std_std_error_DeltaT_degK_air3 * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << std_smax_error_DeltaT_degK_air3 * 10
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\maxmu{\\DeltaT}, \\, \\maxsigma{\\DeltaT}, \\, \\maxmax{\\DeltaT}} "
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_mean_error_DeltaT_degK * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << smax_std_error_DeltaT_degK * 10
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_smax_error_DeltaT_degK * 10
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_mean_error_DeltaT_degK_air2 * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << smax_std_error_DeltaT_degK_air2 * 10
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_smax_error_DeltaT_degK_air2 * 10
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_mean_error_DeltaT_degK_air3 * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << smax_std_error_DeltaT_degK_air3 * 10
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_smax_error_DeltaT_degK_air3 * 10
             << " \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\end{tabular}" << std::endl;
    Oout_tex << "\\end{center}" << std::endl;
    Oout_tex.close();

}
/*< reads the air data estimation results from three sets of files, the baseline, one employing
     * worse air data sensors (OSP & OAT), and another employing even worse sensors.
 * It shows results on console and also generates the following files (for thesis, not Matlab):
 * - "versus_air_T_Hp_DeltaT_table.tex" --> script to directly generate table in Latex */

void nav::test::Tplots_sensors_air::obtain_euler_air_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end) {
    std::string st_folder          = st_first_folder;
    std::string st_folder_one      = "MAIN";
    std::string st_folder_one_air2 = "AIR_WORSE";
    std::string st_folder_one_air3 = "AIR_WORST";

    boost::filesystem::path path_outputs_read(math::share::phd_outputs_store_prefix);
    boost::filesystem::path path_outputs_write(math::share::phd_outputs_prefix);
    std::string st_folder_main("nav");
    std::string st_extra_folder("versus_air");
    std::string st_file("filter_att_euler.txt");

    std::string st_seeds, st_file_complete, st_first_line, st_file_complete_air2, st_file_complete_air3;
    double a01, a02, a03, a04, a05, a06, a07, a08, a09, a10, a11, a12;
    unsigned long nel     = (unsigned long)(seed_end) - (unsigned long)(seed_init) + 1;
    unsigned long nel_all = 380001;
    unsigned long nel_sec = 381;
    unsigned long index_all, index_sec;
    std::vector<double> Vt_sec(nel_sec);
    std::vector<std::vector<double>> VVangle_deg(nel),ZZangle_deg(nel), YYangle_deg(nel);
    std::vector<double> Vangle_deg_all(nel_all), Zangle_deg_all(nel_all), Yangle_deg_all(nel_all);
    std::vector<double> Vmean_angle_deg(nel), Vstd_angle_deg(nel), Vsmax_angle_deg(nel);
    std::vector<double> Zmean_angle_deg(nel), Zstd_angle_deg(nel), Zsmax_angle_deg(nel);
    std::vector<double> Ymean_angle_deg(nel), Ystd_angle_deg(nel), Ysmax_angle_deg(nel);
    double int_part, dec_part;

    // for each trajectory
    for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
        st_seeds = math::seeder::seed2string(seed);

        st_folder.replace(3,2,st_seeds);
        st_file_complete      = (path_outputs_read / st_folder_main / st_folder_one      / st_folder / st_file).string();
        st_file_complete_air2 = (path_outputs_read / st_folder_main / st_folder_one_air2 / st_folder / st_file).string();
        st_file_complete_air3 = (path_outputs_read / st_folder_main / st_folder_one_air3 / st_folder / st_file).string();

        std::ifstream Ostream;
        Ostream.open(st_file_complete);

        VVangle_deg[seed-seed_init].resize(nel_sec); // do not forget

        std::getline(Ostream, st_first_line);
        index_all = 0;
        index_sec = 0;

        do {
            Ostream >> a01 >> a02 >> a03 >> a04 >> a05 >> a06 >> a07 >> a08 >> a09 >> a10 >> a11 >> a12;
            dec_part = modf(a01 / 10, &int_part);
            if (a01 > (-1e-8)) {
                Vangle_deg_all[index_all] = a11;
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = a01;
                    VVangle_deg[seed-seed_init][index_sec] = a11;
                    index_sec++;
                }
            }
        } while (index_sec != nel_sec);

        Ostream.close();

        std::ifstream Ostream_air2;
        Ostream_air2.open(st_file_complete_air2);

        ZZangle_deg[seed-seed_init].resize(nel_sec); // do not forget

        std::getline(Ostream_air2, st_first_line);
        index_all = 0;
        index_sec = 0;

        do {
            Ostream_air2 >> a01 >> a02 >> a03 >> a04 >> a05 >> a06 >> a07 >> a08 >> a09 >> a10 >> a11 >> a12;
            dec_part = modf(a01 / 10, &int_part);
            if (a01 > (-1e-8)) {
                Zangle_deg_all[index_all] = a11;
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = a01;
                    ZZangle_deg[seed-seed_init][index_sec] = a11;
                    index_sec++;
                }
            }
        } while (index_sec != nel_sec);

        Ostream_air2.close();

        std::ifstream Ostream_air3;
        Ostream_air3.open(st_file_complete_air3);

        YYangle_deg[seed-seed_init].resize(nel_sec); // do not forget

        std::getline(Ostream_air3, st_first_line);
        index_all = 0;
        index_sec = 0;

        do {
            Ostream_air3 >> a01 >> a02 >> a03 >> a04 >> a05 >> a06 >> a07 >> a08 >> a09 >> a10 >> a11 >> a12;
            dec_part = modf(a01 / 10, &int_part);
            if (a01 > (-1e-8)) {
                Yangle_deg_all[index_all] = a11;
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = a01;
                    YYangle_deg[seed-seed_init][index_sec] = a11;
                    index_sec++;
                }
            }
        } while (index_sec != nel_sec);

        Ostream_air3.close();

        // compute metrics for each trajectory
        Vmean_angle_deg[seed-seed_init]  = math::mean(Vangle_deg_all);
        Vstd_angle_deg[seed-seed_init]   = math::std(Vangle_deg_all, Vmean_angle_deg[seed-seed_init]);
        Vsmax_angle_deg[seed-seed_init]  = math::smax(Vangle_deg_all);

        Zmean_angle_deg[seed-seed_init]  = math::mean(Zangle_deg_all);
        Zstd_angle_deg[seed-seed_init]   = math::std(Zangle_deg_all, Zmean_angle_deg[seed-seed_init]);
        Zsmax_angle_deg[seed-seed_init]  = math::smax(Zangle_deg_all);

        Ymean_angle_deg[seed-seed_init]  = math::mean(Yangle_deg_all);
        Ystd_angle_deg[seed-seed_init]   = math::std(Yangle_deg_all, Ymean_angle_deg[seed-seed_init]);
        Ysmax_angle_deg[seed-seed_init]  = math::smax(Yangle_deg_all);

        std::cout << (seed - seed_init)
                  << fixed << setw(7) << setprecision(3) << noshowpos << Vmean_angle_deg[seed-seed_init]
                  << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_angle_deg[seed-seed_init]
                  << fixed << setw(7) << setprecision(3) << noshowpos << Vsmax_angle_deg[seed-seed_init]
                  << fixed << setw(7) << setprecision(3) << noshowpos << Zmean_angle_deg[seed-seed_init]
                  << fixed << setw(7) << setprecision(3) << noshowpos << Zstd_angle_deg[seed-seed_init]
                  << fixed << setw(7) << setprecision(3) << noshowpos << Zsmax_angle_deg[seed-seed_init]
                  << fixed << setw(7) << setprecision(3) << noshowpos << Ymean_angle_deg[seed-seed_init]
                  << fixed << setw(7) << setprecision(3) << noshowpos << Ystd_angle_deg[seed-seed_init]
                  << fixed << setw(7) << setprecision(3) << noshowpos << Ysmax_angle_deg[seed-seed_init]
                  << std::endl;

    }

    // obtain aggregated metrics
    double mean_mean_angle_deg = math::mean(Vmean_angle_deg);
    double std_mean_angle_deg  = math::std(Vmean_angle_deg, mean_mean_angle_deg);
    double smax_mean_angle_deg = math::smax(Vmean_angle_deg);

    double mean_std_angle_deg  = math::mean(Vstd_angle_deg);
    double std_std_angle_deg   = math::std(Vstd_angle_deg, mean_std_angle_deg);
    double smax_std_angle_deg  = math::smax(Vstd_angle_deg);

    double mean_smax_angle_deg = math::mean(Vsmax_angle_deg);
    double std_smax_angle_deg  = math::std(Vsmax_angle_deg, mean_smax_angle_deg);
    double smax_smax_angle_deg = math::smax(Vsmax_angle_deg);

    double mean_mean_angle_deg_2 = math::mean(Zmean_angle_deg);
    double std_mean_angle_deg_2  = math::std(Zmean_angle_deg, mean_mean_angle_deg_2);
    double smax_mean_angle_deg_2 = math::smax(Zmean_angle_deg);

    double mean_std_angle_deg_2  = math::mean(Zstd_angle_deg);
    double std_std_angle_deg_2   = math::std(Zstd_angle_deg, mean_std_angle_deg_2);
    double smax_std_angle_deg_2  = math::smax(Zstd_angle_deg);

    double mean_smax_angle_deg_2 = math::mean(Zsmax_angle_deg);
    double std_smax_angle_deg_2  = math::std(Zsmax_angle_deg, mean_smax_angle_deg_2);
    double smax_smax_angle_deg_2 = math::smax(Zsmax_angle_deg);

    double mean_mean_angle_deg_3 = math::mean(Ymean_angle_deg);
    double std_mean_angle_deg_3  = math::std(Ymean_angle_deg, mean_mean_angle_deg_3);
    double smax_mean_angle_deg_3 = math::smax(Ymean_angle_deg);

    double mean_std_angle_deg_3  = math::mean(Ystd_angle_deg);
    double std_std_angle_deg_3   = math::std(Ystd_angle_deg, mean_std_angle_deg_3);
    double smax_std_angle_deg_3  = math::smax(Ystd_angle_deg);

    double mean_smax_angle_deg_3 = math::mean(Ysmax_angle_deg);
    double std_smax_angle_deg_3  = math::std(Ysmax_angle_deg, mean_smax_angle_deg_3);
    double smax_smax_angle_deg_3 = math::smax(Ysmax_angle_deg);

    std::cout << std::endl;
    std::cout << "phiNB [deg] attitude estimation errors (est - truth)" << std::endl;
    std::cout << setw(10) << " " << setw(21) << "baseline" << setw(21) << "OPS-OAT worse" << setw(21) << "OPS-OAT worst" << std::endl;
    std::cout << setw(10) << " "
              << setw(7) << "mean" << setw(7) << "std" << setw(7) << "smax"
              << setw(7) << "mean" << setw(7) << "std" << setw(7) << "smax"
              << setw(7) << "mean" << setw(7) << "std" << setw(7) << "smax"
              << std::endl;

    std::cout << "Mean:      "
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_mean_angle_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_std_angle_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_smax_angle_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_mean_angle_deg_2
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_std_angle_deg_2
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_smax_angle_deg_2
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_mean_angle_deg_3
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_std_angle_deg_3
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_smax_angle_deg_3
              << std::endl;
    std::cout << "std:       "
              << fixed << setw(7) << setprecision(3) << noshowpos << std_mean_angle_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << std_std_angle_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << std_smax_angle_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << std_mean_angle_deg_2
              << fixed << setw(7) << setprecision(3) << noshowpos << std_std_angle_deg_2
              << fixed << setw(7) << setprecision(3) << noshowpos << std_smax_angle_deg_2
              << fixed << setw(7) << setprecision(3) << noshowpos << std_mean_angle_deg_3
              << fixed << setw(7) << setprecision(3) << noshowpos << std_std_angle_deg_3
              << fixed << setw(7) << setprecision(3) << noshowpos << std_smax_angle_deg_3
              << std::endl;
    std::cout << "smax:      "
              << fixed << setw(7) << setprecision(3) << noshowpos << smax_mean_angle_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << smax_std_angle_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << smax_smax_angle_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << smax_mean_angle_deg_2
              << fixed << setw(7) << setprecision(3) << noshowpos << smax_std_angle_deg_2
              << fixed << setw(7) << setprecision(3) << noshowpos << smax_smax_angle_deg_2
              << fixed << setw(7) << setprecision(3) << noshowpos << smax_mean_angle_deg_3
              << fixed << setw(7) << setprecision(3) << noshowpos << smax_std_angle_deg_3
              << fixed << setw(7) << setprecision(3) << noshowpos << smax_smax_angle_deg_3
              << std::endl;

    //////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////

    boost::filesystem::path path_folder(path_outputs_write / st_folder_main / st_extra_folder);
    boost::filesystem::create_directory(path_folder);

    std::string st_file_out_tex = "versus_air_euler_table.tex";
    std::string st_file_output_tex = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_tex).string();

    ofstream Oout_tex;
    Oout_tex.open(st_file_output_tex);

    Oout_tex << "\\begin{center}" << std::endl;
    Oout_tex << "\\begin{tabular}{lrrrrrrrrr}" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\textbf{Benchmark} & \\multicolumn{9}{c}{\\textbf{OSP-OAT Configuration}} \\\\" << std::endl;
    Oout_tex << "\\nm{\\| \\phiNBest - \\phiNB \\| \\ \\lrsb{deg}} & \\multicolumn{3}{c}{\\textbf{Baseline}} & \\multicolumn{3}{c}{\\textbf{Worse}} & \\multicolumn{3}{c}{\\textbf{Even Worse}} \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\nm{\\mumu{}, \\, \\musigma{}, \\, \\mumax{\\cdot}} "
             << " & \\textbf{"
             << fixed << setprecision(3) << noshowpos << mean_mean_angle_deg
             << "} &"
             << fixed << setprecision(3) << noshowpos << mean_std_angle_deg
             << " & "
             << fixed << setprecision(3) << noshowpos << mean_smax_angle_deg
             << " & \\textbf{"
             << fixed << setprecision(3) << noshowpos << mean_mean_angle_deg_2
             << "} & "
             << fixed << setprecision(3) << noshowpos << mean_std_angle_deg_2
             << " & "
             << fixed << setprecision(3) << noshowpos << mean_smax_angle_deg_2
             << " & \\textbf{"
             << fixed << setprecision(3) << noshowpos << mean_mean_angle_deg_3
             << "} & "
             << fixed << setprecision(3) << noshowpos << mean_std_angle_deg_3
             << " & "
             << fixed << setprecision(3) << noshowpos << mean_smax_angle_deg_3
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\sigmamu{}, \\, \\sigmasigma{}, \\, \\sigmamax{\\cdot}}"
             << " & "
             << fixed << setprecision(3) << noshowpos << std_mean_angle_deg
             << " & "
             << fixed << setprecision(3) << noshowpos << std_std_angle_deg
             << " & "
             << fixed << setprecision(3) << noshowpos << std_smax_angle_deg
             << " & "
             << fixed << setprecision(3) << noshowpos << std_mean_angle_deg_2
             << " & "
             << fixed << setprecision(3) << noshowpos << std_std_angle_deg_2
             << " & "
             << fixed << setprecision(3) << noshowpos << std_smax_angle_deg_2
             << " & "
             << fixed << setprecision(3) << noshowpos << std_mean_angle_deg_3
             << " & "
             << fixed << setprecision(3) << noshowpos << std_std_angle_deg_3
             << " & "
             << fixed << setprecision(3) << noshowpos << std_smax_angle_deg_3
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\maxmu{}, \\, \\maxsigma{}, \\, \\maxmax{\\cdot}} "
             << " & "
             << fixed << setprecision(3) << noshowpos << smax_mean_angle_deg
             << " & "
             << fixed << setprecision(3) << noshowpos << smax_std_angle_deg
             << " & "
             << fixed << setprecision(3) << noshowpos << smax_smax_angle_deg
             << " & "
             << fixed << setprecision(3) << noshowpos << smax_mean_angle_deg_2
             << " & "
             << fixed << setprecision(3) << noshowpos << smax_std_angle_deg_2
             << " & "
             << fixed << setprecision(3) << noshowpos << smax_smax_angle_deg_2
             << " & "
             << fixed << setprecision(3) << noshowpos << smax_mean_angle_deg_3
             << " & "
             << fixed << setprecision(3) << noshowpos << smax_std_angle_deg_3
             << " & "
             << fixed << setprecision(3) << noshowpos << smax_smax_angle_deg_3
             << " \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\end{tabular}" << std::endl;
    Oout_tex << "\\end{center}" << std::endl;
    Oout_tex.close();
}
/* reads the body attitude estimation results from three sets of files, the baseline, one employing
     * worse air data sensors (OSP & OAT), and another employing even worse sensors.
 * It shows results on console and also generates the following files (for thesis, not Matlab):
 * - "versus_air_euler_table.tex" --> script to directly generate table in Latex */


void nav::test::Tplots_sensors_air::obtain_h_air_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end) {
    std::string st_folder        = st_first_folder;
    std::string st_folder_one    = "MAIN";
    std::string st_folder_one_2  = "AIR_WORSE";
    std::string st_folder_one_3  = "AIR_WORST";

    boost::filesystem::path path_outputs_read(math::share::phd_outputs_store_prefix);
    boost::filesystem::path path_outputs_write(math::share::phd_outputs_prefix);
    std::string st_folder_main("nav");
    std::string st_extra_folder("versus_air");
    std::string st_file_gps("filter_gps_xgdt.txt");
    std::string st_file_pos("filter_pos_xgdt.txt");
    std::string st_seeds, st_file_gps_complete, st_file_pos_complete, st_first_line, st_line, st_file_gps_complete_2, st_file_pos_complete_2, st_file_gps_complete_3, st_file_pos_complete_3;
    unsigned long nel     = (unsigned long)(seed_end) - (unsigned long)(seed_init) + 1;
    unsigned long nel_all = 380001;
    unsigned long nel_sec = 381;
    unsigned long index_all, index_sec;
    std::vector<double> Vt_sec(nel_sec);
    std::vector<std::vector<double>> VVerror_h_m(nel), ZZerror_h_m(nel), YYerror_h_m(nel);
    std::vector<double> Verror_h_m(nel_all), Vh_m_est(nel_all), Vh_m_truth(nel_all), Zerror_h_m(nel_all), Zh_m_est(nel_all), Zh_m_truth(nel_all), Yerror_h_m(nel_all), Yh_m_est(nel_all), Yh_m_truth(nel_all);
    std::vector<double> Vend_error_h_m(nel), Vabs_end_error_h_m(nel), Vend_h_m_est(nel), Vend_h_m_truth(nel);
    std::vector<double> Zend_error_h_m(nel), Zabs_end_error_h_m(nel), Zend_h_m_est(nel), Zend_h_m_truth(nel);
    std::vector<double> Yend_error_h_m(nel), Yabs_end_error_h_m(nel), Yend_h_m_est(nel), Yend_h_m_truth(nel);
    double int_part, dec_part, t_sec;
    std::string st_t_sec, st_h_est_m, st_h_truth_m;

    // for each trajectory
    for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
        st_seeds = math::seeder::seed2string(seed);

        st_folder.replace(3,2,st_seeds);
        st_file_gps_complete   = (path_outputs_read / st_folder_main / st_folder_one   / st_folder / st_file_gps).string();
        st_file_pos_complete   = (path_outputs_read / st_folder_main / st_folder_one   / st_folder / st_file_pos).string();
        st_file_gps_complete_2 = (path_outputs_read / st_folder_main / st_folder_one_2 / st_folder / st_file_gps).string();
        st_file_pos_complete_2 = (path_outputs_read / st_folder_main / st_folder_one_2 / st_folder / st_file_pos).string();
        st_file_gps_complete_3 = (path_outputs_read / st_folder_main / st_folder_one_3 / st_folder / st_file_gps).string();
        st_file_pos_complete_3 = (path_outputs_read / st_folder_main / st_folder_one_3 / st_folder / st_file_pos).string();

        std::ifstream Ostream_gps;
        Ostream_gps.open(st_file_gps_complete);
        std::getline(Ostream_gps, st_first_line);

        VVerror_h_m[seed-seed_init].resize(nel_sec); // do not forget

        index_all = 0;
        index_sec = 0;

        while (std::getline(Ostream_gps, st_line)) {
            st_t_sec     = st_line.substr(  0, 10);
            st_h_est_m   = st_line.substr( 46, 15);
            st_h_truth_m = st_line.substr(148, 15);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec / 10, &int_part);
                Verror_h_m[index_all] = std::stod(st_h_est_m) - std::stod(st_h_truth_m);
                Vh_m_est[index_all]   = std::stod(st_h_est_m);
                Vh_m_truth[index_all] = std::stod(st_h_truth_m);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = t_sec;
                    VVerror_h_m[seed - seed_init][index_sec] = Verror_h_m[index_all - 1];
                    index_sec++;
                }
            }
        }
        Ostream_gps.close();


        std::ifstream Ostream_pos;
        Ostream_pos.open(st_file_pos_complete);
        std::getline(Ostream_pos, st_first_line);
        std::getline(Ostream_pos, st_line); // line repeated at 100 [sec] from previous file
        while (std::getline(Ostream_pos, st_line)) {
            st_t_sec     = st_line.substr( 0,10);
            st_h_est_m   = st_line.substr(46,15);
            st_h_truth_m = st_line.substr(97,15);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec / 10, &int_part);
                Verror_h_m[index_all] = std::stod(st_h_est_m) - std::stod(st_h_truth_m);
                Vh_m_est[index_all]   = std::stod(st_h_est_m);
                Vh_m_truth[index_all] = std::stod(st_h_truth_m);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = t_sec;
                    VVerror_h_m[seed - seed_init][index_sec] = Verror_h_m[index_all - 1];
                    index_sec++;
                }
            }
        }
        Ostream_pos.close();

        std::ifstream Ostream_gps_2;
        Ostream_gps_2.open(st_file_gps_complete_2);
        std::getline(Ostream_gps_2, st_first_line);

        ZZerror_h_m[seed-seed_init].resize(nel_sec); // do not forget

        index_all = 0;
        index_sec = 0;

        while (std::getline(Ostream_gps_2, st_line)) {
            st_t_sec     = st_line.substr(  0, 10);
            st_h_est_m   = st_line.substr( 46, 15);
            st_h_truth_m = st_line.substr(148, 15);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec / 10, &int_part);
                Zerror_h_m[index_all] = std::stod(st_h_est_m) - std::stod(st_h_truth_m);
                Zh_m_est[index_all]   = std::stod(st_h_est_m);
                Zh_m_truth[index_all] = std::stod(st_h_truth_m);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = t_sec;
                    ZZerror_h_m[seed - seed_init][index_sec] = Zerror_h_m[index_all - 1];
                    index_sec++;
                }
            }
        }
        Ostream_gps_2.close();

        std::ifstream Ostream_pos_2;
        Ostream_pos_2.open(st_file_pos_complete_2);
        std::getline(Ostream_pos_2, st_first_line);
        std::getline(Ostream_pos_2, st_line); // line repeated at 100 [sec] from previous file
        while (std::getline(Ostream_pos_2, st_line)) {
            st_t_sec     = st_line.substr( 0,10);
            st_h_est_m   = st_line.substr(46,15);
            st_h_truth_m = st_line.substr(97,15);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec / 10, &int_part);
                Zerror_h_m[index_all] = std::stod(st_h_est_m) - std::stod(st_h_truth_m);
                Zh_m_est[index_all]   = std::stod(st_h_est_m);
                Zh_m_truth[index_all] = std::stod(st_h_truth_m);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = t_sec;
                    ZZerror_h_m[seed - seed_init][index_sec] = Zerror_h_m[index_all - 1];
                    index_sec++;
                }
            }
        }
        Ostream_pos_2.close();

        std::ifstream Ostream_gps_3;
        Ostream_gps_3.open(st_file_gps_complete_3);
        std::getline(Ostream_gps_3, st_first_line);

        YYerror_h_m[seed-seed_init].resize(nel_sec); // do not forget

        index_all = 0;
        index_sec = 0;

        while (std::getline(Ostream_gps_3, st_line)) {
            st_t_sec     = st_line.substr(  0, 10);
            st_h_est_m   = st_line.substr( 46, 15);
            st_h_truth_m = st_line.substr(148, 15);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec / 10, &int_part);
                Yerror_h_m[index_all] = std::stod(st_h_est_m) - std::stod(st_h_truth_m);
                Yh_m_est[index_all]   = std::stod(st_h_est_m);
                Yh_m_truth[index_all] = std::stod(st_h_truth_m);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = t_sec;
                    YYerror_h_m[seed - seed_init][index_sec] = Yerror_h_m[index_all - 1];
                    index_sec++;
                }
            }
        }
        Ostream_gps_3.close();

        std::ifstream Ostream_pos_3;
        Ostream_pos_3.open(st_file_pos_complete_3);
        std::getline(Ostream_pos_3, st_first_line);
        std::getline(Ostream_pos_3, st_line); // line repeated at 100 [sec] from previous file
        while (std::getline(Ostream_pos_3, st_line)) {
            st_t_sec     = st_line.substr( 0,10);
            st_h_est_m   = st_line.substr(46,15);
            st_h_truth_m = st_line.substr(97,15);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec / 10, &int_part);
                Yerror_h_m[index_all] = std::stod(st_h_est_m) - std::stod(st_h_truth_m);
                Yh_m_est[index_all]   = std::stod(st_h_est_m);
                Yh_m_truth[index_all] = std::stod(st_h_truth_m);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = t_sec;
                    YYerror_h_m[seed - seed_init][index_sec] = Yerror_h_m[index_all - 1];
                    index_sec++;
                }
            }
        }
        Ostream_pos_3.close();

        // compute metrics for each trajectory
        Vend_error_h_m[seed-seed_init]       = Verror_h_m.back();
        Vabs_end_error_h_m[seed-seed_init]   = fabs(Vend_error_h_m[seed-seed_init]);
        Vend_h_m_est[seed-seed_init]         = Vh_m_est.back();
        Vend_h_m_truth[seed-seed_init]       = Vh_m_truth.back();
        Zend_error_h_m[seed-seed_init]       = Zerror_h_m.back();
        Zabs_end_error_h_m[seed-seed_init]   = fabs(Zend_error_h_m[seed-seed_init]);
        Zend_h_m_est[seed-seed_init]         = Zh_m_est.back();
        Zend_h_m_truth[seed-seed_init]       = Zh_m_truth.back();
        Yend_error_h_m[seed-seed_init]       = Yerror_h_m.back();
        Yabs_end_error_h_m[seed-seed_init]   = fabs(Yend_error_h_m[seed-seed_init]);
        Yend_h_m_est[seed-seed_init]         = Yh_m_est.back();
        Yend_h_m_truth[seed-seed_init]       = Yh_m_truth.back();
    }

    // obtain aggregated metrics
    double mean_end_error_h_m   = math::mean(Vend_error_h_m);
    double std_end_error_h_m    = math::std(Vend_error_h_m, mean_end_error_h_m);
    double smax_end_error_h_m   = math::smax(Vend_error_h_m);
    double mean_end_error_h_m_2 = math::mean(Zend_error_h_m);
    double std_end_error_h_m_2  = math::std(Zend_error_h_m, mean_end_error_h_m_2);
    double smax_end_error_h_m_2 = math::smax(Zend_error_h_m);
    double mean_end_error_h_m_3 = math::mean(Yend_error_h_m);
    double std_end_error_h_m_3  = math::std(Yend_error_h_m, mean_end_error_h_m_3);
    double smax_end_error_h_m_3 = math::smax(Yend_error_h_m);

    std::cout << std::endl;
    std::cout << "h [m] position filter errors (est - truth)" << std::endl;
    std::cout << setw(10) << " " << setw(20) << "baseline"    << setw(20) << "OSP-OAT worse" << setw(20) << "OSP-OAT worst" << std::endl;
    std::cout << setw(10) << " " << setw(20) << "final error" << setw(20) << "final error" << setw(20) << "final error"  << std::endl;

    std::cout << "Mean:     "
              << fixed << setw(20) << setprecision(2) <<   showpos << mean_end_error_h_m
              << fixed << setw(20) << setprecision(2) <<   showpos << mean_end_error_h_m_2
              << fixed << setw(20) << setprecision(2) <<   showpos << mean_end_error_h_m_3
              << std::endl;
    std::cout << "std:      "
              << fixed << setw(20) << setprecision(2) << noshowpos << std_end_error_h_m
              << fixed << setw(20) << setprecision(2) << noshowpos << std_end_error_h_m_2
              << fixed << setw(20) << setprecision(2) << noshowpos << std_end_error_h_m_3
              << std::endl;
    std::cout << "smax:     "
              << fixed << setw(20) << setprecision(2) <<   showpos << smax_end_error_h_m
              << fixed << setw(20) << setprecision(2) <<   showpos << smax_end_error_h_m_2
              << fixed << setw(20) << setprecision(2) <<   showpos << smax_end_error_h_m_3
              << std::endl;

    // reverse data (rows to columns and viceversa) to compute other metrics
    std::vector<std::vector<double>> WWerror_h_m(nel_sec), WWerror_h_m_2(nel_sec), WWerror_h_m_3(nel_sec);
    std::vector<double> Wmean_error_h_m(nel_sec), Wstd_error_h_m(nel_sec), Wmean_error_h_m_2(nel_sec), Wstd_error_h_m_2(nel_sec), Wmean_error_h_m_3(nel_sec), Wstd_error_h_m_3(nel_sec);
    for (unsigned long i = 0; i != nel_sec; ++i) {
        WWerror_h_m[i].resize(nel);
        WWerror_h_m_2[i].resize(nel);
        WWerror_h_m_3[i].resize(nel);
        for (unsigned long j = 0; j != nel; ++j) {
            WWerror_h_m[i][j]   = VVerror_h_m[j][i];
            WWerror_h_m_2[i][j] = ZZerror_h_m[j][i];
            WWerror_h_m_3[i][j] = YYerror_h_m[j][i];
        }
        Wmean_error_h_m[i]   = math::mean(WWerror_h_m[i]);
        Wstd_error_h_m[i]    = math::std(WWerror_h_m[i], Wmean_error_h_m[i]);
        Wmean_error_h_m_2[i] = math::mean(WWerror_h_m_2[i]);
        Wstd_error_h_m_2[i]  = math::std(WWerror_h_m_2[i], Wmean_error_h_m_2[i]);
        Wmean_error_h_m_3[i] = math::mean(WWerror_h_m_3[i]);
        Wstd_error_h_m_3[i]  = math::std(WWerror_h_m_3[i], Wmean_error_h_m_3[i]);
    }

    boost::filesystem::path path_folder(path_outputs_write / st_folder_main / st_extra_folder);
    boost::filesystem::create_directory(path_folder);

    std::string st_file_out = "versus_air_h_m.txt";
    std::string st_file_output = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out).string();

    ofstream Oout;
    Oout.open(st_file_output);

    for (unsigned long i = 0; i != nel_sec; ++i) {
        Oout << fixed      << setw(8)  << setprecision(1) << showpos << Vt_sec[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_h_m[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_h_m[i] + Wstd_error_h_m[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_h_m[i] - Wstd_error_h_m[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_h_m_2[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_h_m_2[i] + Wstd_error_h_m_2[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_h_m_2[i] - Wstd_error_h_m_2[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_h_m_3[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_h_m_3[i] + Wstd_error_h_m_3[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_h_m_3[i] - Wstd_error_h_m_3[i];
        Oout << endl;
    }
    Oout.close();

    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    std::string st_file_out_tex = "versus_air_h_table.tex";
    std::string st_file_output_tex = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_tex).string();

    ofstream Oout_tex;
    Oout_tex.open(st_file_output_tex);

    Oout_tex << "\\begin{center}" << std::endl;
    Oout_tex << "\\begin{tabular}{lrp{0.2cm}rp{0.2cm}r}" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\textbf{Benchmark} & \\multicolumn{5}{c}{\\textbf{OSP-OAT Configuration}} \\\\" << std::endl;
    Oout_tex << "\\nm{\\hest - h \\ \\lrsb{m}} & \\textbf{Baseline} & & \\textbf{Worse}  & & \\textbf{Worst} \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\nm{\\muEND{h}} "
             << " & "
             << fixed << setprecision(2) <<   showpos << mean_end_error_h_m
             << " & & "
             << fixed << setprecision(2) <<   showpos << mean_end_error_h_m_2
             << " & & "
             << fixed << setprecision(2) <<   showpos << mean_end_error_h_m_3
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\sigmaEND{h}} "
             << " & \\textbf{"
             << fixed << setprecision(2) << noshowpos << std_end_error_h_m
             << "} & & \\textbf{"
             << fixed << setprecision(2) << noshowpos << std_end_error_h_m_2
             << "} & & \\textbf{"
             << fixed << setprecision(2) << noshowpos << std_end_error_h_m_3
             << "} \\\\" << std::endl;
    Oout_tex << "\\nm{\\maxEND{h}} "
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_end_error_h_m
             << " & & "
             << fixed << setprecision(2) <<   showpos << smax_end_error_h_m_2
             << " & & "
             << fixed << setprecision(2) <<   showpos << smax_end_error_h_m_3
             << " \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\end{tabular}" << std::endl;
    Oout_tex << "\\end{center}" << std::endl;
    Oout_tex.close();

}
/* reads the final altitude estimation results from three sets of files, the baseline, one employing
     * worse air data sensors (OSP & OAT), and another employing even worse sensors.
 * It shows results on console and also generates the following files (for thesis, not Matlab):
 * - "versus_air_h_m.txt" --> to plot altitude mean and std variation with time
 * - "versus_air_h.tex" --> script to directly generate table in Latex */




























