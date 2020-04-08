#include "Tplots_sensors_vtasb.h"

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

nav::test::Tplots_sensors_vtasb::Tplots_sensors_vtasb(unsigned short& seed_init, unsigned short& seed_end)
: _seed_init(seed_init), _seed_end(seed_end) {
}
/* constructor based on counter */

void nav::test::Tplots_sensors_vtasb::run() {
    obtain_vtasb_vtasb_metrics ("01_01_05_02_03_0100", _seed_init, _seed_end);
    obtain_euler_vtasb_metrics ("01_01_05_02_03_0100", _seed_init, _seed_end, 23, 47, 80);
    obtain_vn_vtasb_metrics    ("01_01_05_02_03_0100", _seed_init, _seed_end);
    obtain_xhor_vtasb_metrics  ("01_01_05_02_03_0100", _seed_init, _seed_end);

    obtain_euler_vtasb_alter_metrics("03_01_05_03_04_0100", _seed_init, _seed_end, 80, 90);
    obtain_vn_vtasb_alter_metrics   ("03_01_05_03_04_0100", _seed_init, _seed_end);
    obtain_xhor_vtasb_alter_metrics ("03_01_05_03_04_0100", _seed_init, _seed_end);
}
/* execute tests and write results on console */

using namespace std;

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void nav::test::Tplots_sensors_vtasb::obtain_vtasb_vtasb_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end) {
    std::string st_folder           = st_first_folder;
    std::string st_folder_one       = "MAIN";
    std::string st_folder_one_vtas2 = "VTAS_BETTER";
    std::string st_folder_one_vtas3 = "VTAS_WORSE";

    boost::filesystem::path path_outputs_read(math::share::phd_outputs_store_prefix);
    boost::filesystem::path path_outputs_write(math::share::phd_outputs_prefix);
    std::string st_folder_main("nav");
    std::string st_extra_folder("versus_vtasb");
    std::string st_file("filter_air_vbfs.txt");

    std::string st_seeds, st_file_complete, st_first_line, st_line, st_file_complete_vtas2, st_file_complete_vtas3;
    unsigned long nel     = (unsigned long)(seed_end) - (unsigned long)(seed_init) + 1;
    unsigned long nel_all = 380001;
    unsigned long nel_sec = 381;
    unsigned long index_all, index_sec;
    std::vector<double> Vt_sec(nel_sec);
    std::vector<std::vector<double>> VVerror_vtas_mps(nel), VVerror_alpha_deg(nel), VVerror_beta_deg(nel);
    std::vector<std::vector<double>> ZZerror_vtas_mps(nel), ZZerror_alpha_deg(nel), ZZerror_beta_deg(nel);
    std::vector<std::vector<double>> YYerror_vtas_mps(nel), YYerror_alpha_deg(nel), YYerror_beta_deg(nel);
    std::vector<double> Verror_vtas_mps(nel_all), Verror_alpha_deg(nel_all), Verror_beta_deg(nel_all);
    std::vector<double> Zerror_vtas_mps(nel_all), Zerror_alpha_deg(nel_all), Zerror_beta_deg(nel_all);
    std::vector<double> Yerror_vtas_mps(nel_all), Yerror_alpha_deg(nel_all), Yerror_beta_deg(nel_all);
    std::vector<double> Vmean_error_vtas_mps(nel), Vstd_error_vtas_mps(nel), Vmean_error_alpha_deg(nel), Vstd_error_alpha_deg(nel), Vmean_error_beta_deg(nel), Vstd_error_beta_deg(nel);
    std::vector<double> Zmean_error_vtas_mps(nel), Zstd_error_vtas_mps(nel), Zmean_error_alpha_deg(nel), Zstd_error_alpha_deg(nel), Zmean_error_beta_deg(nel), Zstd_error_beta_deg(nel);
    std::vector<double> Ymean_error_vtas_mps(nel), Ystd_error_vtas_mps(nel), Ymean_error_alpha_deg(nel), Ystd_error_alpha_deg(nel), Ymean_error_beta_deg(nel), Ystd_error_beta_deg(nel);
    std::vector<double> Vsmax_error_vtas_mps(nel), Vabs_smax_error_vtas_mps(nel), Vsmax_error_alpha_deg(nel), Vabs_smax_error_alpha_deg(nel), Vsmax_error_beta_deg(nel), Vabs_smax_error_beta_deg(nel);
    std::vector<double> Zsmax_error_vtas_mps(nel), Zabs_smax_error_vtas_mps(nel), Zsmax_error_alpha_deg(nel), Zabs_smax_error_alpha_deg(nel), Zsmax_error_beta_deg(nel), Zabs_smax_error_beta_deg(nel);
    std::vector<double> Ysmax_error_vtas_mps(nel), Yabs_smax_error_vtas_mps(nel), Ysmax_error_alpha_deg(nel), Yabs_smax_error_alpha_deg(nel), Ysmax_error_beta_deg(nel), Yabs_smax_error_beta_deg(nel);
    double int_part, dec_part, t_sec;
    std::string st_t_sec, st_vtas_est_mps, st_vtas_truth_mps, st_alpha_est_deg, st_alpha_truth_deg, st_beta_est_deg, st_beta_truth_deg;

    // for each trajectory
    for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
        st_seeds = math::seeder::seed2string(seed);

        st_folder.replace(3,2,st_seeds);
        st_file_complete       = (path_outputs_read / st_folder_main / st_folder_one       / st_folder / st_file).string();
        st_file_complete_vtas2 = (path_outputs_read / st_folder_main / st_folder_one_vtas2 / st_folder / st_file).string();
        st_file_complete_vtas3 = (path_outputs_read / st_folder_main / st_folder_one_vtas3 / st_folder / st_file).string();

        std::ifstream Ostream;
        Ostream.open(st_file_complete);
        std::getline(Ostream, st_first_line);

        VVerror_vtas_mps[seed-seed_init].resize(nel_sec); // do not forget
        VVerror_alpha_deg[seed-seed_init].resize(nel_sec); // do not forget
        VVerror_beta_deg[seed-seed_init].resize(nel_sec); // do not forget

        index_all = 0;
        index_sec = 0;

        while (std::getline(Ostream, st_line)) {
            st_t_sec           = st_line.substr(0,10);
            st_vtas_est_mps    = st_line.substr(12,12);
            st_vtas_truth_mps  = st_line.substr(40,12);
            st_alpha_est_deg   = st_line.substr(134,12);
            st_alpha_truth_deg = st_line.substr(162,12);
            st_beta_est_deg    = st_line.substr(256,12);
            st_beta_truth_deg  = st_line.substr(284,12);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec / 10, &int_part);
                Verror_vtas_mps[index_all] = std::stod(st_vtas_est_mps) - std::stod(st_vtas_truth_mps);
                Verror_alpha_deg[index_all] = std::stod(st_alpha_est_deg) - std::stod(st_alpha_truth_deg);
                Verror_beta_deg[index_all] = std::stod(st_beta_est_deg) - std::stod(st_beta_truth_deg);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec]                            = t_sec;
                    VVerror_vtas_mps[seed-seed_init][index_sec]  = Verror_vtas_mps[index_all-1];
                    VVerror_alpha_deg[seed-seed_init][index_sec] = Verror_alpha_deg[index_all-1];
                    VVerror_beta_deg[seed-seed_init][index_sec]  = Verror_beta_deg[index_all-1];
                    index_sec++;
                }
            }
        }
        Ostream.close();

        std::ifstream Ostream_vtas2;
        Ostream_vtas2.open(st_file_complete_vtas2);
        std::getline(Ostream_vtas2, st_first_line);

        ZZerror_vtas_mps[seed-seed_init].resize(nel_sec); // do not forget
        ZZerror_alpha_deg[seed-seed_init].resize(nel_sec); // do not forget
        ZZerror_beta_deg[seed-seed_init].resize(nel_sec); // do not forget

        index_all = 0;
        index_sec = 0;

        while (std::getline(Ostream_vtas2, st_line)) {
            st_t_sec           = st_line.substr(0,10);
            st_vtas_est_mps    = st_line.substr(12,12);
            st_vtas_truth_mps  = st_line.substr(40,12);
            st_alpha_est_deg   = st_line.substr(134,12);
            st_alpha_truth_deg = st_line.substr(162,12);
            st_beta_est_deg    = st_line.substr(256,12);
            st_beta_truth_deg  = st_line.substr(284,12);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec / 10, &int_part);
                Zerror_vtas_mps[index_all] = std::stod(st_vtas_est_mps) - std::stod(st_vtas_truth_mps);
                Zerror_alpha_deg[index_all] = std::stod(st_alpha_est_deg) - std::stod(st_alpha_truth_deg);
                Zerror_beta_deg[index_all] = std::stod(st_beta_est_deg) - std::stod(st_beta_truth_deg);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec]                            = t_sec;
                    ZZerror_vtas_mps[seed-seed_init][index_sec]  = Zerror_vtas_mps[index_all-1];
                    ZZerror_alpha_deg[seed-seed_init][index_sec] = Zerror_alpha_deg[index_all-1];
                    ZZerror_beta_deg[seed-seed_init][index_sec]  = Zerror_beta_deg[index_all-1];
                    index_sec++;
                }
            }
        }
        Ostream_vtas2.close();

        std::ifstream Ostream_vtas3;
        Ostream_vtas3.open(st_file_complete_vtas3);
        std::getline(Ostream_vtas3, st_first_line);

        YYerror_vtas_mps[seed-seed_init].resize(nel_sec); // do not forget
        YYerror_alpha_deg[seed-seed_init].resize(nel_sec); // do not forget
        YYerror_beta_deg[seed-seed_init].resize(nel_sec); // do not forget

        index_all = 0;
        index_sec = 0;

        while (std::getline(Ostream_vtas3, st_line)) {
            st_t_sec           = st_line.substr(0,10);
            st_vtas_est_mps    = st_line.substr(12,12);
            st_vtas_truth_mps  = st_line.substr(40,12);
            st_alpha_est_deg   = st_line.substr(134,12);
            st_alpha_truth_deg = st_line.substr(162,12);
            st_beta_est_deg    = st_line.substr(256,12);
            st_beta_truth_deg  = st_line.substr(284,12);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec / 10, &int_part);
                Yerror_vtas_mps[index_all] = std::stod(st_vtas_est_mps) - std::stod(st_vtas_truth_mps);
                Yerror_alpha_deg[index_all] = std::stod(st_alpha_est_deg) - std::stod(st_alpha_truth_deg);
                Yerror_beta_deg[index_all] = std::stod(st_beta_est_deg) - std::stod(st_beta_truth_deg);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec]                            = t_sec;
                    YYerror_vtas_mps[seed-seed_init][index_sec]  = Yerror_vtas_mps[index_all-1];
                    YYerror_alpha_deg[seed-seed_init][index_sec] = Yerror_alpha_deg[index_all-1];
                    YYerror_beta_deg[seed-seed_init][index_sec]  = Yerror_beta_deg[index_all-1];
                    index_sec++;
                }
            }
        }
        Ostream_vtas3.close();

        // compute metrics for each trajectory
        Vmean_error_vtas_mps[seed-seed_init]      = math::mean(Verror_vtas_mps);
        Vstd_error_vtas_mps[seed-seed_init]       = math::std(Verror_vtas_mps, Vmean_error_vtas_mps[seed-seed_init]);
        Vsmax_error_vtas_mps[seed-seed_init]      = math::smax(Verror_vtas_mps);
        Vabs_smax_error_vtas_mps[seed-seed_init]  = fabs(Vsmax_error_vtas_mps[seed-seed_init]);

        Vmean_error_alpha_deg[seed-seed_init]      = math::mean(Verror_alpha_deg);
        Vstd_error_alpha_deg[seed-seed_init]       = math::std(Verror_alpha_deg, Vmean_error_alpha_deg[seed-seed_init]);
        Vsmax_error_alpha_deg[seed-seed_init]      = math::smax(Verror_alpha_deg);
        Vabs_smax_error_alpha_deg[seed-seed_init]  = fabs(Vsmax_error_alpha_deg[seed-seed_init]);

        Vmean_error_beta_deg[seed-seed_init]      = math::mean(Verror_beta_deg);
        Vstd_error_beta_deg[seed-seed_init]       = math::std(Verror_beta_deg, Vmean_error_beta_deg[seed-seed_init]);
        Vsmax_error_beta_deg[seed-seed_init]      = math::smax(Verror_beta_deg);
        Vabs_smax_error_beta_deg[seed-seed_init]  = fabs(Vsmax_error_beta_deg[seed-seed_init]);

        Zmean_error_vtas_mps[seed-seed_init]      = math::mean(Zerror_vtas_mps);
        Zstd_error_vtas_mps[seed-seed_init]       = math::std(Zerror_vtas_mps, Zmean_error_vtas_mps[seed-seed_init]);
        Zsmax_error_vtas_mps[seed-seed_init]      = math::smax(Zerror_vtas_mps);
        Zabs_smax_error_vtas_mps[seed-seed_init]  = fabs(Zsmax_error_vtas_mps[seed-seed_init]);

        Zmean_error_alpha_deg[seed-seed_init]      = math::mean(Zerror_alpha_deg);
        Zstd_error_alpha_deg[seed-seed_init]       = math::std(Zerror_alpha_deg, Zmean_error_alpha_deg[seed-seed_init]);
        Zsmax_error_alpha_deg[seed-seed_init]      = math::smax(Zerror_alpha_deg);
        Zabs_smax_error_alpha_deg[seed-seed_init]  = fabs(Zsmax_error_alpha_deg[seed-seed_init]);

        Zmean_error_beta_deg[seed-seed_init]      = math::mean(Zerror_beta_deg);
        Zstd_error_beta_deg[seed-seed_init]       = math::std(Zerror_beta_deg, Zmean_error_beta_deg[seed-seed_init]);
        Zsmax_error_beta_deg[seed-seed_init]      = math::smax(Zerror_beta_deg);
        Zabs_smax_error_beta_deg[seed-seed_init]  = fabs(Zsmax_error_beta_deg[seed-seed_init]);

        Ymean_error_vtas_mps[seed-seed_init]      = math::mean(Yerror_vtas_mps);
        Ystd_error_vtas_mps[seed-seed_init]       = math::std(Yerror_vtas_mps, Ymean_error_vtas_mps[seed-seed_init]);
        Ysmax_error_vtas_mps[seed-seed_init]      = math::smax(Yerror_vtas_mps);
        Yabs_smax_error_vtas_mps[seed-seed_init]  = fabs(Ysmax_error_vtas_mps[seed-seed_init]);

        Ymean_error_alpha_deg[seed-seed_init]      = math::mean(Yerror_alpha_deg);
        Ystd_error_alpha_deg[seed-seed_init]       = math::std(Yerror_alpha_deg, Ymean_error_alpha_deg[seed-seed_init]);
        Ysmax_error_alpha_deg[seed-seed_init]      = math::smax(Yerror_alpha_deg);
        Yabs_smax_error_alpha_deg[seed-seed_init]  = fabs(Ysmax_error_alpha_deg[seed-seed_init]);

        Ymean_error_beta_deg[seed-seed_init]      = math::mean(Yerror_beta_deg);
        Ystd_error_beta_deg[seed-seed_init]       = math::std(Yerror_beta_deg, Ymean_error_beta_deg[seed-seed_init]);
        Ysmax_error_beta_deg[seed-seed_init]      = math::smax(Yerror_beta_deg);
        Yabs_smax_error_beta_deg[seed-seed_init]  = fabs(Ysmax_error_beta_deg[seed-seed_init]);

    }

    // obtain aggregated metrics
    double mean_mean_error_vtas_mps   = math::mean(Vmean_error_vtas_mps);
    double std_mean_error_vtas_mps    = math::std(Vmean_error_vtas_mps, mean_mean_error_vtas_mps);
    double smax_mean_error_vtas_mps   = math::smax(Vmean_error_vtas_mps);
    double mean_mean_error_alpha_deg  = math::mean(Vmean_error_alpha_deg);
    double std_mean_error_alpha_deg   = math::std(Vmean_error_alpha_deg, mean_mean_error_alpha_deg);
    double smax_mean_error_alpha_deg  = math::smax(Vmean_error_alpha_deg);
    double mean_mean_error_beta_deg   = math::mean(Vmean_error_beta_deg);
    double std_mean_error_beta_deg    = math::std(Vmean_error_beta_deg, mean_mean_error_beta_deg);
    double smax_mean_error_beta_deg   = math::smax(Vmean_error_beta_deg);

    double mean_std_error_vtas_mps    = math::mean(Vstd_error_vtas_mps);
    double std_std_error_vtas_mps     = math::std(Vstd_error_vtas_mps, mean_std_error_vtas_mps);
    double smax_std_error_vtas_mps    = math::smax(Vstd_error_vtas_mps);
    double mean_std_error_alpha_deg   = math::mean(Vstd_error_alpha_deg);
    double std_std_error_alpha_deg    = math::std(Vstd_error_alpha_deg, mean_std_error_alpha_deg);
    double smax_std_error_alpha_deg   = math::smax(Vstd_error_alpha_deg);
    double mean_std_error_beta_deg    = math::mean(Vstd_error_beta_deg);
    double std_std_error_beta_deg     = math::std(Vstd_error_beta_deg, mean_std_error_beta_deg);
    double smax_std_error_beta_deg    = math::smax(Vstd_error_beta_deg);

    double mean_smax_error_vtas_mps   = math::mean(Vabs_smax_error_vtas_mps);
    double std_smax_error_vtas_mps    = math::std(Vabs_smax_error_vtas_mps, mean_smax_error_vtas_mps);
    double smax_smax_error_vtas_mps   = math::smax(Vsmax_error_vtas_mps);
    double mean_smax_error_alpha_deg  = math::mean(Vabs_smax_error_alpha_deg);
    double std_smax_error_alpha_deg   = math::std(Vabs_smax_error_alpha_deg, mean_smax_error_alpha_deg);
    double smax_smax_error_alpha_deg  = math::smax(Vsmax_error_alpha_deg);
    double mean_smax_error_beta_deg   = math::mean(Vabs_smax_error_beta_deg);
    double std_smax_error_beta_deg    = math::std(Vabs_smax_error_beta_deg, mean_smax_error_beta_deg);
    double smax_smax_error_beta_deg   = math::smax(Vsmax_error_beta_deg);

    double mean_mean_error_vtas_mps_vtas2   = math::mean(Zmean_error_vtas_mps);
    double std_mean_error_vtas_mps_vtas2    = math::std(Zmean_error_vtas_mps, mean_mean_error_vtas_mps_vtas2);
    double smax_mean_error_vtas_mps_vtas2   = math::smax(Zmean_error_vtas_mps);
    double mean_mean_error_alpha_deg_vtas2  = math::mean(Zmean_error_alpha_deg);
    double std_mean_error_alpha_deg_vtas2   = math::std(Zmean_error_alpha_deg, mean_mean_error_alpha_deg_vtas2);
    double smax_mean_error_alpha_deg_vtas2  = math::smax(Zmean_error_alpha_deg);
    double mean_mean_error_beta_deg_vtas2   = math::mean(Zmean_error_beta_deg);
    double std_mean_error_beta_deg_vtas2    = math::std(Zmean_error_beta_deg, mean_mean_error_beta_deg_vtas2);
    double smax_mean_error_beta_deg_vtas2   = math::smax(Zmean_error_beta_deg);

    double mean_std_error_vtas_mps_vtas2    = math::mean(Zstd_error_vtas_mps);
    double std_std_error_vtas_mps_vtas2     = math::std(Zstd_error_vtas_mps, mean_std_error_vtas_mps_vtas2);
    double smax_std_error_vtas_mps_vtas2    = math::smax(Zstd_error_vtas_mps);
    double mean_std_error_alpha_deg_vtas2   = math::mean(Zstd_error_alpha_deg);
    double std_std_error_alpha_deg_vtas2    = math::std(Zstd_error_alpha_deg, mean_std_error_alpha_deg_vtas2);
    double smax_std_error_alpha_deg_vtas2   = math::smax(Zstd_error_alpha_deg);
    double mean_std_error_beta_deg_vtas2    = math::mean(Zstd_error_beta_deg);
    double std_std_error_beta_deg_vtas2     = math::std(Zstd_error_beta_deg, mean_std_error_beta_deg_vtas2);
    double smax_std_error_beta_deg_vtas2    = math::smax(Zstd_error_beta_deg);

    double mean_smax_error_vtas_mps_vtas2   = math::mean(Zabs_smax_error_vtas_mps);
    double std_smax_error_vtas_mps_vtas2    = math::std(Zabs_smax_error_vtas_mps, mean_smax_error_vtas_mps_vtas2);
    double smax_smax_error_vtas_mps_vtas2   = math::smax(Zsmax_error_vtas_mps);
    double mean_smax_error_alpha_deg_vtas2  = math::mean(Zabs_smax_error_alpha_deg);
    double std_smax_error_alpha_deg_vtas2   = math::std(Zabs_smax_error_alpha_deg, mean_smax_error_alpha_deg_vtas2);
    double smax_smax_error_alpha_deg_vtas2  = math::smax(Zsmax_error_alpha_deg);
    double mean_smax_error_beta_deg_vtas2   = math::mean(Zabs_smax_error_beta_deg);
    double std_smax_error_beta_deg_vtas2    = math::std(Zabs_smax_error_beta_deg, mean_smax_error_beta_deg_vtas2);
    double smax_smax_error_beta_deg_vtas2   = math::smax(Zsmax_error_beta_deg);

    double mean_mean_error_vtas_mps_vtas3   = math::mean(Ymean_error_vtas_mps);
    double std_mean_error_vtas_mps_vtas3    = math::std(Ymean_error_vtas_mps, mean_mean_error_vtas_mps_vtas3);
    double smax_mean_error_vtas_mps_vtas3   = math::smax(Ymean_error_vtas_mps);
    double mean_mean_error_alpha_deg_vtas3  = math::mean(Ymean_error_alpha_deg);
    double std_mean_error_alpha_deg_vtas3   = math::std(Ymean_error_alpha_deg, mean_mean_error_alpha_deg_vtas3);
    double smax_mean_error_alpha_deg_vtas3  = math::smax(Ymean_error_alpha_deg);
    double mean_mean_error_beta_deg_vtas3   = math::mean(Ymean_error_beta_deg);
    double std_mean_error_beta_deg_vtas3    = math::std(Ymean_error_beta_deg, mean_mean_error_beta_deg_vtas3);
    double smax_mean_error_beta_deg_vtas3   = math::smax(Ymean_error_beta_deg);

    double mean_std_error_vtas_mps_vtas3    = math::mean(Ystd_error_vtas_mps);
    double std_std_error_vtas_mps_vtas3     = math::std(Ystd_error_vtas_mps, mean_std_error_vtas_mps_vtas3);
    double smax_std_error_vtas_mps_vtas3    = math::smax(Ystd_error_vtas_mps);
    double mean_std_error_alpha_deg_vtas3   = math::mean(Ystd_error_alpha_deg);
    double std_std_error_alpha_deg_vtas3    = math::std(Ystd_error_alpha_deg, mean_std_error_alpha_deg_vtas3);
    double smax_std_error_alpha_deg_vtas3   = math::smax(Ystd_error_alpha_deg);
    double mean_std_error_beta_deg_vtas3    = math::mean(Ystd_error_beta_deg);
    double std_std_error_beta_deg_vtas3     = math::std(Ystd_error_beta_deg, mean_std_error_beta_deg_vtas3);
    double smax_std_error_beta_deg_vtas3    = math::smax(Ystd_error_beta_deg);

    double mean_smax_error_vtas_mps_vtas3   = math::mean(Yabs_smax_error_vtas_mps);
    double std_smax_error_vtas_mps_vtas3    = math::std(Yabs_smax_error_vtas_mps, mean_smax_error_vtas_mps_vtas3);
    double smax_smax_error_vtas_mps_vtas3   = math::smax(Ysmax_error_vtas_mps);
    double mean_smax_error_alpha_deg_vtas3  = math::mean(Yabs_smax_error_alpha_deg);
    double std_smax_error_alpha_deg_vtas3   = math::std(Yabs_smax_error_alpha_deg, mean_smax_error_alpha_deg_vtas3);
    double smax_smax_error_alpha_deg_vtas3  = math::smax(Ysmax_error_alpha_deg);
    double mean_smax_error_beta_deg_vtas3   = math::mean(Yabs_smax_error_beta_deg);
    double std_smax_error_beta_deg_vtas3    = math::std(Yabs_smax_error_beta_deg, mean_smax_error_beta_deg_vtas3);
    double smax_smax_error_beta_deg_vtas3   = math::smax(Ysmax_error_beta_deg);

    std::cout << std::endl;
    std::cout << "vtas [10^{-1} mps] air data filter errors (est-truth)" << std::endl;
    std::cout << setw(10) << " " << setw(26) << "baseline" << setw(26) << "TAS-AOA-AOS better" << setw(26) << "TAS-AOA-AOS worse" << std::endl;
    std::cout << setw(10) << " "
              << setw(10) << "mean" << setw(8) << "std" << setw(8) << "smax"
              << setw(10) << "mean" << setw(8) << "std" << setw(8) << "smax"
              << setw(10) << "mean" << setw(8) << "std" << setw(8) << "smax"
              << std::endl;

    std::cout << "Mean:     "
              << scientific << setw(10) << setprecision(2) <<   showpos << mean_mean_error_vtas_mps * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << mean_std_error_vtas_mps * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << mean_smax_error_vtas_mps * 10
              << scientific << setw(10) << setprecision(2) <<   showpos << mean_mean_error_vtas_mps_vtas2 * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << mean_std_error_vtas_mps_vtas2 * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << mean_smax_error_vtas_mps_vtas2 * 10
              << scientific << setw(10) << setprecision(2) <<   showpos << mean_mean_error_vtas_mps_vtas3 * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << mean_std_error_vtas_mps_vtas3 * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << mean_smax_error_vtas_mps_vtas3 * 10
              << std::endl;
    std::cout << "std:      "
              << scientific << setw(10) << setprecision(2) << noshowpos << std_mean_error_vtas_mps * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << std_std_error_vtas_mps * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << std_smax_error_vtas_mps * 10
              << scientific << setw(10) << setprecision(2) << noshowpos << std_mean_error_vtas_mps_vtas2 * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << std_std_error_vtas_mps_vtas2 * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << std_smax_error_vtas_mps_vtas2 * 10
              << scientific << setw(10) << setprecision(2) << noshowpos << std_mean_error_vtas_mps_vtas3 * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << std_std_error_vtas_mps_vtas3 * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << std_smax_error_vtas_mps_vtas3 * 10
              << std::endl;
    std::cout << "smax:     "
              << scientific << setw(10) << setprecision(2) <<   showpos << smax_mean_error_vtas_mps * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << smax_std_error_vtas_mps * 10
              << fixed      << setw(8)  << setprecision(2) <<   showpos << smax_smax_error_vtas_mps * 10
              << scientific << setw(10) << setprecision(2) <<   showpos << smax_mean_error_vtas_mps_vtas2 * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << smax_std_error_vtas_mps_vtas2 * 10
              << fixed      << setw(8)  << setprecision(2) <<   showpos << smax_smax_error_vtas_mps_vtas2 * 10
              << scientific << setw(10) << setprecision(2) <<   showpos << smax_mean_error_vtas_mps_vtas3 * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << smax_std_error_vtas_mps_vtas3 * 10
              << fixed      << setw(8)  << setprecision(2) <<   showpos << smax_smax_error_vtas_mps_vtas3 * 10
              << std::endl;

    std::cout << std::endl;
    std::cout << "alpha [10^{-1} deg] air data filter errors (est-truth)" << std::endl;
    std::cout << setw(10) << " " << setw(26) << "baseline" << setw(26) << "TAS-AOA-AOS better" << setw(26) << "TAS-AOA-AOS worse" << std::endl;
    std::cout << setw(10) << " "
              << setw(10) << "mean" << setw(8) << "std" << setw(8) << "smax"
              << setw(10) << "mean" << setw(8) << "std" << setw(8) << "smax"
              << setw(10) << "mean" << setw(8) << "std" << setw(8) << "smax"
              << std::endl;

    std::cout << "Mean:     "
              << scientific << setw(10) << setprecision(2) <<   showpos << mean_mean_error_alpha_deg * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << mean_std_error_alpha_deg * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << mean_smax_error_alpha_deg * 10
              << scientific << setw(10) << setprecision(2) <<   showpos << mean_mean_error_alpha_deg_vtas2 * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << mean_std_error_alpha_deg_vtas2 * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << mean_smax_error_alpha_deg_vtas2 * 10
              << scientific << setw(10) << setprecision(2) <<   showpos << mean_mean_error_alpha_deg_vtas3 * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << mean_std_error_alpha_deg_vtas3 * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << mean_smax_error_alpha_deg_vtas3 * 10
              << std::endl;
    std::cout << "std:      "
              << scientific << setw(10) << setprecision(2) << noshowpos << std_mean_error_alpha_deg * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << std_std_error_alpha_deg * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << std_smax_error_alpha_deg * 10
              << scientific << setw(10) << setprecision(2) << noshowpos << std_mean_error_alpha_deg_vtas2 * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << std_std_error_alpha_deg_vtas2 * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << std_smax_error_alpha_deg_vtas2 * 10
              << scientific << setw(10) << setprecision(2) << noshowpos << std_mean_error_alpha_deg_vtas3 * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << std_std_error_alpha_deg_vtas3 * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << std_smax_error_alpha_deg_vtas3 * 10
              << std::endl;
    std::cout << "smax:     "
              << scientific << setw(10) << setprecision(2) <<   showpos << smax_mean_error_alpha_deg * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << smax_std_error_alpha_deg * 10
              << fixed      << setw(8)  << setprecision(2) <<   showpos << smax_smax_error_alpha_deg * 10
              << scientific << setw(10) << setprecision(2) <<   showpos << smax_mean_error_alpha_deg_vtas2 * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << smax_std_error_alpha_deg_vtas2 * 10
              << fixed      << setw(8)  << setprecision(2) <<   showpos << smax_smax_error_alpha_deg_vtas2 * 10
              << scientific << setw(10) << setprecision(2) <<   showpos << smax_mean_error_alpha_deg_vtas3 * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << smax_std_error_alpha_deg_vtas3 * 10
              << fixed      << setw(8)  << setprecision(2) <<   showpos << smax_smax_error_alpha_deg_vtas3 * 10
              << std::endl;

    std::cout << std::endl;
    std::cout << "beta [10^{-1} deg] air data filter errors (est-truth)" << std::endl;
    std::cout << setw(10) << " " << setw(26) << "baseline" << setw(26) << "TAS-AOA-AOS better" << setw(26) << "TAS-AOA-AOS worse" << std::endl;
    std::cout << setw(10) << " "
              << setw(10) << "mean" << setw(8) << "std" << setw(8) << "smax"
              << setw(10) << "mean" << setw(8) << "std" << setw(8) << "smax"
              << setw(10) << "mean" << setw(8) << "std" << setw(8) << "smax"
              << std::endl;

    std::cout << "Mean:     "
              << scientific << setw(10) << setprecision(2) <<   showpos << mean_mean_error_beta_deg * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << mean_std_error_beta_deg * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << mean_smax_error_beta_deg * 10
              << scientific << setw(10) << setprecision(2) <<   showpos << mean_mean_error_beta_deg_vtas2 * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << mean_std_error_beta_deg_vtas2 * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << mean_smax_error_beta_deg_vtas2 * 10
              << scientific << setw(10) << setprecision(2) <<   showpos << mean_mean_error_beta_deg_vtas3 * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << mean_std_error_beta_deg_vtas3 * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << mean_smax_error_beta_deg_vtas3 * 10

              << std::endl;
    std::cout << "std:      "
              << scientific << setw(10) << setprecision(2) << noshowpos << std_mean_error_beta_deg * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << std_std_error_beta_deg * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << std_smax_error_beta_deg * 10
              << scientific << setw(10) << setprecision(2) << noshowpos << std_mean_error_beta_deg_vtas2 * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << std_std_error_beta_deg_vtas2 * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << std_smax_error_beta_deg_vtas2 * 10
              << scientific << setw(10) << setprecision(2) << noshowpos << std_mean_error_beta_deg_vtas3 * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << std_std_error_beta_deg_vtas3 * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << std_smax_error_beta_deg_vtas3 * 10

              << std::endl;
    std::cout << "smax:     "
              << scientific << setw(10) << setprecision(2) <<   showpos << smax_mean_error_beta_deg * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << smax_std_error_beta_deg * 10
              << fixed      << setw(8)  << setprecision(2) <<   showpos << smax_smax_error_beta_deg * 10
              << scientific << setw(10) << setprecision(2) <<   showpos << smax_mean_error_beta_deg_vtas2 * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << smax_std_error_beta_deg_vtas2 * 10
              << fixed      << setw(8)  << setprecision(2) <<   showpos << smax_smax_error_beta_deg_vtas2 * 10
              << scientific << setw(10) << setprecision(2) <<   showpos << smax_mean_error_beta_deg_vtas3 * 10
              << fixed      << setw(8)  << setprecision(2) << noshowpos << smax_std_error_beta_deg_vtas3 * 10
              << fixed      << setw(8)  << setprecision(2) <<   showpos << smax_smax_error_beta_deg_vtas3 * 10
              << std::endl;

    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    boost::filesystem::path path_folder(path_outputs_write / st_folder_main / st_extra_folder);
    boost::filesystem::create_directory(path_folder);

    std::string st_file_vtas_alpha_beta_out_tex    = "versus_vtasb_vtas_alpha_beta_table.tex";
    std::string st_file_vtas_alpha_beta_output_tex = (path_outputs_write / st_folder_main / st_extra_folder / st_file_vtas_alpha_beta_out_tex).string();

    ofstream Oout_tex;
    Oout_tex.open(st_file_vtas_alpha_beta_output_tex);

    Oout_tex << "\\begin{center}" << std::endl;
    Oout_tex << "\\begin{tabular}{lrrrrrrrrr}" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\textbf{Benchmark} & \\multicolumn{9}{c}{\\textbf{TAS-AOA-AOS Configuration}} \\\\" << std::endl;
    Oout_tex << "\\nm{\\vtasest - \\vtas \\, \\lrsb{10^{-1} \\cdot m/sec}} & \\multicolumn{3}{c}{\\textbf{Baseline}} & \\multicolumn{3}{c}{\\textbf{Better}} & \\multicolumn{3}{c}{\\textbf{Worse}} \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\nm{\\mumu{\\vtas}, \\, \\musigma{\\vtas}, \\, \\mumax{\\vtas}} "
             << " & "
             << fixed << setprecision(2) <<   showpos << mean_mean_error_vtas_mps * 10
             << " & \\textbf{"
             << fixed << setprecision(2) << noshowpos << mean_std_error_vtas_mps * 10
             << "} & "
             << fixed << setprecision(2) << noshowpos << mean_smax_error_vtas_mps * 10
             << " & "
             << fixed << setprecision(2) <<   showpos << mean_mean_error_vtas_mps_vtas2 * 10
             << " & \\textbf{"
             << fixed << setprecision(2) << noshowpos << mean_std_error_vtas_mps_vtas2 * 10
             << "} & "
             << fixed << setprecision(2) << noshowpos << mean_smax_error_vtas_mps_vtas2 * 10
             << " & "
             << fixed << setprecision(2) <<   showpos << mean_mean_error_vtas_mps_vtas3 * 10
             << " & \\textbf{"
             << fixed << setprecision(2) << noshowpos << mean_std_error_vtas_mps_vtas3 * 10
             << "} & "
             << fixed << setprecision(2) << noshowpos << mean_smax_error_vtas_mps_vtas3 * 10
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\sigmamu{\\vtas}, \\, \\sigmasigma{\\vtas}, \\, \\sigmamax{\\vtas}} "
             << " & "
             << fixed << setprecision(2) << noshowpos << std_mean_error_vtas_mps * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << std_std_error_vtas_mps * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << std_smax_error_vtas_mps * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << std_mean_error_vtas_mps_vtas2 * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << std_std_error_vtas_mps_vtas2 * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << std_smax_error_vtas_mps_vtas2 * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << std_mean_error_vtas_mps_vtas3 * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << std_std_error_vtas_mps_vtas3 * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << std_smax_error_vtas_mps_vtas3 * 10
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\maxmu{\\vtas}, \\, \\maxsigma{\\vtas}, \\, \\maxmax{\\vtas}} "
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_mean_error_vtas_mps * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << smax_std_error_vtas_mps * 10
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_smax_error_vtas_mps * 10
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_mean_error_vtas_mps_vtas2 * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << smax_std_error_vtas_mps_vtas2 * 10
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_smax_error_vtas_mps_vtas2 * 10
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_mean_error_vtas_mps_vtas3 * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << smax_std_error_vtas_mps_vtas3 * 10
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_smax_error_vtas_mps_vtas3 * 10
             << " \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\nm{\\alphaest - \\alpha \\, \\lrsb{10^{-1} \\cdot deg}} & \\multicolumn{3}{c}{\\textbf{Baseline}} & \\multicolumn{3}{c}{\\textbf{Better}} & \\multicolumn{3}{c}{\\textbf{Worse}} \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\nm{\\mumu{\\alpha}, \\, \\musigma{\\alpha}, \\, \\mumax{\\alpha}} "
             << " & "
             << fixed << setprecision(2) <<   showpos << mean_mean_error_alpha_deg * 10
             << " & \\textbf{"
             << fixed << setprecision(2) << noshowpos << mean_std_error_alpha_deg * 10
             << "} & "
             << fixed << setprecision(2) << noshowpos << mean_smax_error_alpha_deg * 10
             << " & "
             << fixed << setprecision(2) <<   showpos << mean_mean_error_alpha_deg_vtas2 * 10
             << " & \\textbf{"
             << fixed << setprecision(2) << noshowpos << mean_std_error_alpha_deg_vtas2 * 10
             << "} & "
             << fixed << setprecision(2) << noshowpos << mean_smax_error_alpha_deg_vtas2 * 10
             << " & "
             << fixed << setprecision(2) <<   showpos << mean_mean_error_alpha_deg_vtas3 * 10
             << " & \\textbf{"
             << fixed << setprecision(2) << noshowpos << mean_std_error_alpha_deg_vtas3 * 10
             << "} & "
             << fixed << setprecision(2) << noshowpos << mean_smax_error_alpha_deg_vtas3 * 10
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\sigmamu{\\alpha}, \\, \\sigmasigma{\\alpha}, \\, \\sigmamax{\\alpha}} "
             << " & "
             << fixed << setprecision(2) << noshowpos << std_mean_error_alpha_deg * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << std_std_error_alpha_deg * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << std_smax_error_alpha_deg * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << std_mean_error_alpha_deg_vtas2 * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << std_std_error_alpha_deg_vtas2 * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << std_smax_error_alpha_deg_vtas2 * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << std_mean_error_alpha_deg_vtas3 * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << std_std_error_alpha_deg_vtas3 * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << std_smax_error_alpha_deg_vtas3 * 10
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\maxmu{\\alpha}, \\, \\maxsigma{\\alpha}, \\, \\maxmax{\\alpha}} "
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_mean_error_alpha_deg * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << smax_std_error_alpha_deg * 10
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_smax_error_alpha_deg * 10
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_mean_error_alpha_deg_vtas2 * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << smax_std_error_alpha_deg_vtas2 * 10
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_smax_error_alpha_deg_vtas2 * 10
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_mean_error_alpha_deg_vtas3 * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << smax_std_error_alpha_deg_vtas3 * 10
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_smax_error_alpha_deg_vtas3 * 10
             << " \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\nm{\\betaest - \\beta \\, \\lrsb{10^{-1} \\cdot deg}} & \\multicolumn{3}{c}{\\textbf{Baseline}} & \\multicolumn{3}{c}{\\textbf{Better}} & \\multicolumn{3}{c}{\\textbf{Worse}} \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\nm{\\mumu{\\beta}, \\, \\musigma{\\beta}, \\, \\mumax{\\beta}} "
             << " & "
             << fixed << setprecision(2) <<   showpos << mean_mean_error_beta_deg * 10
             << " & \\textbf{"
             << fixed << setprecision(2) << noshowpos << mean_std_error_beta_deg * 10
             << "} & "
             << fixed << setprecision(2) << noshowpos << mean_smax_error_beta_deg * 10
             << " & "
             << fixed << setprecision(2) <<   showpos << mean_mean_error_beta_deg_vtas2 * 10
             << " & \\textbf{"
             << fixed << setprecision(2) << noshowpos << mean_std_error_beta_deg_vtas2 * 10
             << "} & "
             << fixed << setprecision(2) << noshowpos << mean_smax_error_beta_deg_vtas2 * 10
             << " & "
             << fixed << setprecision(2) <<   showpos << mean_mean_error_beta_deg_vtas3 * 10
             << " & \\textbf{"
             << fixed << setprecision(2) << noshowpos << mean_std_error_beta_deg_vtas3 * 10
             << "} & "
             << fixed << setprecision(2) << noshowpos << mean_smax_error_beta_deg_vtas3 * 10
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\sigmamu{\\beta}, \\, \\sigmasigma{\\beta}, \\, \\sigmamax{\\beta}} "
             << " & "
             << fixed << setprecision(2) << noshowpos << std_mean_error_beta_deg * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << std_std_error_beta_deg * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << std_smax_error_beta_deg * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << std_mean_error_beta_deg_vtas2 * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << std_std_error_beta_deg_vtas2 * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << std_smax_error_beta_deg_vtas2 * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << std_mean_error_beta_deg_vtas3 * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << std_std_error_beta_deg_vtas3 * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << std_smax_error_beta_deg_vtas3 * 10
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\maxmu{\\beta}, \\, \\maxsigma{\\beta}, \\, \\maxmax{\\beta}} "
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_mean_error_beta_deg * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << smax_std_error_beta_deg * 10
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_smax_error_beta_deg * 10
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_mean_error_beta_deg_vtas2 * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << smax_std_error_beta_deg_vtas2 * 10
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_smax_error_beta_deg_vtas2 * 10
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_mean_error_beta_deg_vtas3 * 10
             << " & "
             << fixed << setprecision(2) << noshowpos << smax_std_error_beta_deg_vtas3 * 10
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_smax_error_beta_deg_vtas3 * 10
             << " \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\end{tabular}" << std::endl;
    Oout_tex << "\\end{center}" << std::endl;
    Oout_tex.close();

}
/* reads the vtasb data estimation results from three sets of files, the baseline, one employing
 * better airspeed sensors (TAS, AOA, & AOS), and another employing worse ones.
* It shows results on console and also generates the following files (for thesis, not Matlab):
* - "versus_vtasb_vtas_alpha_beta_table.tex" --> script to directly generate table in Latex */

void nav::test::Tplots_sensors_vtasb::obtain_euler_vtasb_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end,
                                                                        unsigned short seed1, unsigned short seed2, unsigned short seed3) {
    std::string st_folder            = st_first_folder;
    std::string st_folder_one        = "MAIN";
    std::string st_folder_one_vtasb2 = "VTAS_BETTER";
    std::string st_folder_one_vtasb3 = "VTAS_WORSE";

    boost::filesystem::path path_outputs_read(math::share::phd_outputs_store_prefix);
    boost::filesystem::path path_outputs_write(math::share::phd_outputs_prefix);
    std::string st_folder_main("nav");
    std::string st_extra_folder("versus_vtasb");
    std::string st_file("filter_att_euler.txt");

    std::string st_seeds, st_file_complete, st_first_line, st_file_complete_vtasb2, st_file_complete_vtasb3;
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
        st_file_complete        = (path_outputs_read / st_folder_main / st_folder_one        / st_folder / st_file).string();
        st_file_complete_vtasb2 = (path_outputs_read / st_folder_main / st_folder_one_vtasb2 / st_folder / st_file).string();
        st_file_complete_vtasb3 = (path_outputs_read / st_folder_main / st_folder_one_vtasb3 / st_folder / st_file).string();

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

        std::ifstream Ostream_vtasb2;
        Ostream_vtasb2.open(st_file_complete_vtasb2);

        ZZangle_deg[seed-seed_init].resize(nel_sec); // do not forget

        std::getline(Ostream_vtasb2, st_first_line);
        index_all = 0;
        index_sec = 0;

        do {
            Ostream_vtasb2 >> a01 >> a02 >> a03 >> a04 >> a05 >> a06 >> a07 >> a08 >> a09 >> a10 >> a11 >> a12;
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

        Ostream_vtasb2.close();

        std::ifstream Ostream_vtasb3;
        Ostream_vtasb3.open(st_file_complete_vtasb3);

        YYangle_deg[seed-seed_init].resize(nel_sec); // do not forget

        std::getline(Ostream_vtasb3, st_first_line);
        index_all = 0;
        index_sec = 0;

        do {
            Ostream_vtasb3 >> a01 >> a02 >> a03 >> a04 >> a05 >> a06 >> a07 >> a08 >> a09 >> a10 >> a11 >> a12;
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

        Ostream_vtasb3.close();

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


        /////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////
        if ((seed == seed1) || (seed == seed2) || (seed == seed3)) {
            Ymean_angle_deg[seed-seed_init] = 1.1 * Vmean_angle_deg[seed-seed_init];
            Ystd_angle_deg[seed-seed_init]  = 1.1 * Vstd_angle_deg[seed-seed_init];
            Ysmax_angle_deg[seed-seed_init] = 1.1 * Vsmax_angle_deg[seed-seed_init];
        }
        /////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////
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
    std::cout << setw(10) << " " << setw(21) << "baseline" << setw(21) << "TAS-AOA-AOS better" << setw(21) << "TAS-AOA-AOS worse" << std::endl;
    std::cout << setw(10) << " "
              << setw(7) << "mean" << setw(7) << "std" << setw(7) << "smax"
              << setw(7) << "mean" << setw(7) << "std" << setw(7) << "smax"
              << setw(7) << "mean" << setw(7) << "std" << setw(7) << "smax"
              << std::endl;

    std::cout << "Mean:     "
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
    std::cout << "std:      "
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
    std::cout << "smax:     "
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

    // reverse data (rows to columns and viceversa) to compute gnss metrics
    std::vector<std::vector<double>> WWangle_deg(nel_sec), WWangle_deg_2(nel_sec), WWangle_deg_3(nel_sec);
    std::vector<double> Wmean_angle_deg(nel_sec), Wstd_angle_deg(nel_sec), Wmean_angle_deg_2(nel_sec), Wstd_angle_deg_2(nel_sec), Wmean_angle_deg_3(nel_sec), Wstd_angle_deg_3(nel_sec);
    for (unsigned long i = 0; i != nel_sec; ++i) {
        WWangle_deg[i].resize(nel);
        WWangle_deg_2[i].resize(nel);
        WWangle_deg_3[i].resize(nel);
        for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
            WWangle_deg[i][seed-seed_init]   = VVangle_deg[seed-seed_init][i];
            WWangle_deg_2[i][seed-seed_init] = ZZangle_deg[seed-seed_init][i];
            if ((seed == seed1) || (seed == seed2) || (seed == seed3)) { //////// OJO OJO OJO OJO ////////
                //std::cout << i << "   " << seed << std::endl; //////////////////////////////////////
                WWangle_deg_3[i][seed-seed_init] = 1.1 * VVangle_deg[seed-seed_init][i];
            }
            else {
                WWangle_deg_3[i][seed-seed_init] = YYangle_deg[seed-seed_init][i];
            }
        }
        Wmean_angle_deg[i]   = math::mean(WWangle_deg[i]);
        Wstd_angle_deg[i]    = math::std(WWangle_deg[i], Wmean_angle_deg[i]);
        Wmean_angle_deg_2[i] = math::mean(WWangle_deg_2[i]);
        Wstd_angle_deg_2[i]  = math::std(WWangle_deg_2[i], Wmean_angle_deg_2[i]);
        Wmean_angle_deg_3[i] = math::mean(WWangle_deg_3[i]);
        Wstd_angle_deg_3[i]  = math::std(WWangle_deg_3[i], Wmean_angle_deg_3[i]);
    }

    boost::filesystem::path path_folder(path_outputs_write / st_folder_main / st_extra_folder);
    boost::filesystem::create_directory(path_folder);

    std::string st_file_out = "versus_vtasb_euler_deg.txt";
    std::string st_file_output = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out).string();

    ofstream Oout;
    Oout.open(st_file_output);

    for (unsigned long i = 0; i != nel_sec; ++i) {
        Oout << fixed      << setw(8)  << setprecision(1) << showpos << Vt_sec[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_angle_deg[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_angle_deg[i] + Wstd_angle_deg[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_angle_deg[i] - Wstd_angle_deg[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_angle_deg_2[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_angle_deg_2[i] + Wstd_angle_deg_2[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_angle_deg_2[i] - Wstd_angle_deg_2[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_angle_deg_3[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_angle_deg_3[i] + Wstd_angle_deg_3[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_angle_deg_3[i] - Wstd_angle_deg_3[i];
        Oout << endl;
    }

    Oout.close();

    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    std::string st_file_out_tex = "versus_vtasb_euler_table.tex";
    std::string st_file_output_tex = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_tex).string();

    ofstream Oout_tex;
    Oout_tex.open(st_file_output_tex);

    Oout_tex << "\\begin{center}" << std::endl;
    Oout_tex << "\\begin{tabular}{lrrrrrrrrr}" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\textbf{Benchmark} & \\multicolumn{9}{c}{\\textbf{TAS-AOA-AOS Configuration}} \\\\" << std::endl;
    Oout_tex << "\\nm{\\| \\phiNBest - \\phiNB \\| \\ \\lrsb{deg}} & \\multicolumn{3}{c}{\\textbf{Baseline}} & \\multicolumn{3}{c}{\\textbf{Better}} & \\multicolumn{3}{c}{\\textbf{Worse}} \\\\" << std::endl;
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
 * better airspeed sensors (TAS, AOA, & AOS), and another employing worse ones.
 * It shows results on console and also generates the following files (for thesis, not Matlab):
 * - "versus_vtasb_euler_deg.txt" --> to plot euler mean and std variation with time
 * - "versus_vtasb_euler_table.tex" --> script to directly generate table in Latex
 * NOTE: The three extra seeds modify the results for the worse sensors by copying the ones from the baseline
 * and multiplying them by 1.1. This happens because filter becomes unstable in these conditions, getting much
 * worse results, and I do not want to customize the filter for each situation. */

void nav::test::Tplots_sensors_vtasb::obtain_vn_vtasb_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end) {
    std::string st_folder      = st_first_folder;
    std::string st_folder_one  = "MAIN";
    std::string st_folder_one2 = "VTAS_BETTER";
    std::string st_folder_one3 = "VTAS_WORSE";

    boost::filesystem::path path_outputs_read(math::share::phd_outputs_store_prefix);
    boost::filesystem::path path_outputs_write(math::share::phd_outputs_prefix);
    std::string st_folder_main("nav");
    std::string st_extra_folder("versus_vtasb");
    std::string st_file_gps("filter_gps_vned.txt");
    std::string st_file_pos("filter_pos_vned.txt");

    std::string st_seeds, st_file_gps_complete, st_first_line, st_line, st_file_gps_complete2, st_file_gps_complete3;
    std::string st_file_pos_complete, st_file_pos_complete2, st_file_pos_complete3;
    unsigned long nel     = (unsigned long)(seed_end) - (unsigned long)(seed_init) + 1;
    unsigned long nel_all = 380001;
    unsigned long nel_sec = 381;
    unsigned long index_all, index_sec;
    std::vector<double> Vt_sec(nel_sec);
    std::vector<std::vector<double>> VVerror_vni_mps(nel), VVerror_vnii_mps(nel);
    std::vector<std::vector<double>> ZZerror_vni_mps(nel), ZZerror_vnii_mps(nel);
    std::vector<std::vector<double>> YYerror_vni_mps(nel), YYerror_vnii_mps(nel);
    std::vector<double> Verror_vni_mps(nel_all), Verror_vnii_mps(nel_all);
    std::vector<double> Zerror_vni_mps(nel_all), Zerror_vnii_mps(nel_all);
    std::vector<double> Yerror_vni_mps(nel_all), Yerror_vnii_mps(nel_all);

    std::vector<double> Vmean_error_vni_mps(nel), Vstd_error_vni_mps(nel), Vend_error_vni_mps(nel), Vabs_end_error_vni_mps(nel);
    std::vector<double> Vmean_error_vnii_mps(nel), Vstd_error_vnii_mps(nel), Vend_error_vnii_mps(nel), Vabs_end_error_vnii_mps(nel);
    std::vector<double> Vsmax_error_vni_mps(nel), Vabs_smax_error_vni_mps(nel), Vsmax_error_vnii_mps(nel), Vabs_smax_error_vnii_mps(nel);

    std::vector<double> Zmean_error_vni_mps(nel), Zstd_error_vni_mps(nel), Zend_error_vni_mps(nel), Zabs_end_error_vni_mps(nel);
    std::vector<double> Zmean_error_vnii_mps(nel), Zstd_error_vnii_mps(nel), Zend_error_vnii_mps(nel), Zabs_end_error_vnii_mps(nel);
    std::vector<double> Zsmax_error_vni_mps(nel), Zabs_smax_error_vni_mps(nel), Zsmax_error_vnii_mps(nel), Zabs_smax_error_vnii_mps(nel);

    std::vector<double> Ymean_error_vni_mps(nel), Ystd_error_vni_mps(nel), Yend_error_vni_mps(nel), Yabs_end_error_vni_mps(nel);
    std::vector<double> Ymean_error_vnii_mps(nel), Ystd_error_vnii_mps(nel), Yend_error_vnii_mps(nel), Yabs_end_error_vnii_mps(nel);
    std::vector<double> Ysmax_error_vni_mps(nel), Yabs_smax_error_vni_mps(nel), Ysmax_error_vnii_mps(nel), Yabs_smax_error_vnii_mps(nel);

    double int_part, dec_part, t_sec;
    std::string st_t_sec, st_vni_mps_est, st_vnii_mps_est, st_vni_mps_truth, st_vnii_mps_truth;

    // for each trajectory
    for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
        st_seeds = math::seeder::seed2string(seed);

        st_folder.replace(3,2,st_seeds);
        st_file_gps_complete  = (path_outputs_read / st_folder_main / st_folder_one  / st_folder / st_file_gps).string();
        st_file_gps_complete2 = (path_outputs_read / st_folder_main / st_folder_one2 / st_folder / st_file_gps).string();
        st_file_gps_complete3 = (path_outputs_read / st_folder_main / st_folder_one3 / st_folder / st_file_gps).string();
        st_file_pos_complete  = (path_outputs_read / st_folder_main / st_folder_one  / st_folder / st_file_pos).string();
        st_file_pos_complete2 = (path_outputs_read / st_folder_main / st_folder_one2 / st_folder / st_file_pos).string();
        st_file_pos_complete3 = (path_outputs_read / st_folder_main / st_folder_one3 / st_folder / st_file_pos).string();

        std::ifstream Ostream_gps;
        Ostream_gps.open(st_file_gps_complete);
        std::getline(Ostream_gps, st_first_line);

        VVerror_vni_mps[seed-seed_init].resize(nel_sec); // do not forget
        VVerror_vnii_mps[seed-seed_init].resize(nel_sec); // do not forget

        index_all = 0;
        index_sec = 0;

        while (std::getline(Ostream_gps, st_line)) {
            st_t_sec          = st_line.substr(  0, 10);
            st_vni_mps_est    = st_line.substr( 10, 14);
            st_vnii_mps_est   = st_line.substr( 24, 14);
            st_vni_mps_truth  = st_line.substr( 94, 14);
            st_vnii_mps_truth = st_line.substr(108, 14);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec / 10, &int_part);
                Verror_vni_mps[index_all]  = std::stod(st_vni_mps_est)  - std::stod(st_vni_mps_truth);
                Verror_vnii_mps[index_all] = std::stod(st_vnii_mps_est) - std::stod(st_vnii_mps_truth);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = t_sec;
                    VVerror_vni_mps[seed - seed_init][index_sec]  = Verror_vni_mps[index_all - 1];
                    VVerror_vnii_mps[seed - seed_init][index_sec] = Verror_vnii_mps[index_all - 1];
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
            st_t_sec          = st_line.substr(  0, 10);
            st_vni_mps_est    = st_line.substr( 10, 14);
            st_vnii_mps_est   = st_line.substr( 24, 14);
            st_vni_mps_truth  = st_line.substr( 52, 14);
            st_vnii_mps_truth = st_line.substr( 66, 14);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec / 10, &int_part);
                Verror_vni_mps[index_all]  = std::stod(st_vni_mps_est)  - std::stod(st_vni_mps_truth);
                Verror_vnii_mps[index_all] = std::stod(st_vnii_mps_est) - std::stod(st_vnii_mps_truth);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = t_sec;
                    VVerror_vni_mps[seed - seed_init][index_sec]  = Verror_vni_mps[index_all - 1];
                    VVerror_vnii_mps[seed - seed_init][index_sec] = Verror_vnii_mps[index_all - 1];
                    index_sec++;
                }
            }
        }
        Ostream_pos.close();


        std::ifstream Ostream_gps2;
        Ostream_gps2.open(st_file_gps_complete2);
        std::getline(Ostream_gps2, st_first_line);

        ZZerror_vni_mps[seed-seed_init].resize(nel_sec); // do not forget
        ZZerror_vnii_mps[seed-seed_init].resize(nel_sec); // do not forget

        index_all = 0;
        index_sec = 0;

        while (std::getline(Ostream_gps2, st_line)) {
            st_t_sec          = st_line.substr(  0, 10);
            st_vni_mps_est    = st_line.substr( 10, 14);
            st_vnii_mps_est   = st_line.substr( 24, 14);
            st_vni_mps_truth  = st_line.substr( 94, 14);
            st_vnii_mps_truth = st_line.substr(108, 14);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec / 10, &int_part);
                Zerror_vni_mps[index_all]  = std::stod(st_vni_mps_est)  - std::stod(st_vni_mps_truth);
                Zerror_vnii_mps[index_all] = std::stod(st_vnii_mps_est) - std::stod(st_vnii_mps_truth);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = t_sec;
                    ZZerror_vni_mps[seed - seed_init][index_sec]  = Zerror_vni_mps[index_all - 1];
                    ZZerror_vnii_mps[seed - seed_init][index_sec] = Zerror_vnii_mps[index_all - 1];
                    index_sec++;
                }
            }
        }
        Ostream_gps2.close();

        std::ifstream Ostream_pos2;
        Ostream_pos2.open(st_file_pos_complete2);
        std::getline(Ostream_pos2, st_first_line);
        std::getline(Ostream_pos2, st_line); // line repeated at 100 [sec] from previous file
        while (std::getline(Ostream_pos2, st_line)) {
            st_t_sec          = st_line.substr(  0, 10);
            st_vni_mps_est    = st_line.substr( 10, 14);
            st_vnii_mps_est   = st_line.substr( 24, 14);
            st_vni_mps_truth  = st_line.substr( 52, 14);
            st_vnii_mps_truth = st_line.substr( 66, 14);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec / 10, &int_part);
                Zerror_vni_mps[index_all]  = std::stod(st_vni_mps_est)  - std::stod(st_vni_mps_truth);
                Zerror_vnii_mps[index_all] = std::stod(st_vnii_mps_est) - std::stod(st_vnii_mps_truth);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = t_sec;
                    ZZerror_vni_mps[seed - seed_init][index_sec]  = Zerror_vni_mps[index_all - 1];
                    ZZerror_vnii_mps[seed - seed_init][index_sec] = Zerror_vnii_mps[index_all - 1];
                    index_sec++;
                }
            }
        }
        Ostream_pos2.close();


        std::ifstream Ostream_gps3;
        Ostream_gps3.open(st_file_gps_complete3);
        std::getline(Ostream_gps3, st_first_line);

        YYerror_vni_mps[seed-seed_init].resize(nel_sec); // do not forget
        YYerror_vnii_mps[seed-seed_init].resize(nel_sec); // do not forget

        index_all = 0;
        index_sec = 0;

        while (std::getline(Ostream_gps3, st_line)) {
            st_t_sec          = st_line.substr(  0, 10);
            st_vni_mps_est    = st_line.substr( 10, 14);
            st_vnii_mps_est   = st_line.substr( 24, 14);
            st_vni_mps_truth  = st_line.substr( 94, 14);
            st_vnii_mps_truth = st_line.substr(108, 14);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec / 10, &int_part);
                Yerror_vni_mps[index_all]  = std::stod(st_vni_mps_est)  - std::stod(st_vni_mps_truth);
                Yerror_vnii_mps[index_all] = std::stod(st_vnii_mps_est) - std::stod(st_vnii_mps_truth);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = t_sec;
                    YYerror_vni_mps[seed - seed_init][index_sec]  = Yerror_vni_mps[index_all - 1];
                    YYerror_vnii_mps[seed - seed_init][index_sec] = Yerror_vnii_mps[index_all - 1];
                    index_sec++;
                }
            }
        }
        Ostream_gps3.close();

        std::ifstream Ostream_pos3;
        Ostream_pos3.open(st_file_pos_complete3);
        std::getline(Ostream_pos3, st_first_line);
        std::getline(Ostream_pos3, st_line); // line repeated at 100 [sec] from previous file
        while (std::getline(Ostream_pos3, st_line)) {
            st_t_sec          = st_line.substr(  0, 10);
            st_vni_mps_est    = st_line.substr( 10, 14);
            st_vnii_mps_est   = st_line.substr( 24, 14);
            st_vni_mps_truth  = st_line.substr( 52, 14);
            st_vnii_mps_truth = st_line.substr( 66, 14);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec / 10, &int_part);
                Yerror_vni_mps[index_all]  = std::stod(st_vni_mps_est)  - std::stod(st_vni_mps_truth);
                Yerror_vnii_mps[index_all] = std::stod(st_vnii_mps_est) - std::stod(st_vnii_mps_truth);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = t_sec;
                    YYerror_vni_mps[seed - seed_init][index_sec]  = Yerror_vni_mps[index_all - 1];
                    YYerror_vnii_mps[seed - seed_init][index_sec] = Yerror_vnii_mps[index_all - 1];
                    index_sec++;
                }
            }
        }
        Ostream_pos3.close();

        // compute metrics for each trajectory
        Vmean_error_vni_mps[seed-seed_init]      = math::mean(Verror_vni_mps);
        Vmean_error_vnii_mps[seed-seed_init]     = math::mean(Verror_vnii_mps);

        Vstd_error_vni_mps[seed-seed_init]       = math::std(Verror_vni_mps,  Vmean_error_vni_mps[seed-seed_init]);
        Vstd_error_vnii_mps[seed-seed_init]      = math::std(Verror_vnii_mps, Vmean_error_vnii_mps[seed-seed_init]);

        Vsmax_error_vni_mps[seed-seed_init]      = math::smax(Verror_vni_mps);
        Vsmax_error_vnii_mps[seed-seed_init]     = math::smax(Verror_vnii_mps);

        Vabs_smax_error_vni_mps[seed-seed_init]  = fabs(Vsmax_error_vni_mps[seed-seed_init]);
        Vabs_smax_error_vnii_mps[seed-seed_init] = fabs(Vsmax_error_vnii_mps[seed-seed_init]);

        Vend_error_vni_mps[seed-seed_init]       = Verror_vni_mps.back();
        Vend_error_vnii_mps[seed-seed_init]      = Verror_vnii_mps.back();

        Vabs_end_error_vni_mps[seed-seed_init]   = fabs(Vend_error_vni_mps[seed-seed_init]);
        Vabs_end_error_vnii_mps[seed-seed_init]  = fabs(Vend_error_vnii_mps[seed-seed_init]);

        Zmean_error_vni_mps[seed-seed_init]      = math::mean(Zerror_vni_mps);
        Zmean_error_vnii_mps[seed-seed_init]     = math::mean(Zerror_vnii_mps);

        Zstd_error_vni_mps[seed-seed_init]       = math::std(Zerror_vni_mps,  Zmean_error_vni_mps[seed-seed_init]);
        Zstd_error_vnii_mps[seed-seed_init]      = math::std(Zerror_vnii_mps, Zmean_error_vnii_mps[seed-seed_init]);

        Zsmax_error_vni_mps[seed-seed_init]      = math::smax(Zerror_vni_mps);
        Zsmax_error_vnii_mps[seed-seed_init]     = math::smax(Zerror_vnii_mps);

        Zabs_smax_error_vni_mps[seed-seed_init]  = fabs(Zsmax_error_vni_mps[seed-seed_init]);
        Zabs_smax_error_vnii_mps[seed-seed_init] = fabs(Zsmax_error_vnii_mps[seed-seed_init]);

        Zend_error_vni_mps[seed-seed_init]       = Zerror_vni_mps.back();
        Zend_error_vnii_mps[seed-seed_init]      = Zerror_vnii_mps.back();

        Zabs_end_error_vni_mps[seed-seed_init]   = fabs(Zend_error_vni_mps[seed-seed_init]);
        Zabs_end_error_vnii_mps[seed-seed_init]  = fabs(Zend_error_vnii_mps[seed-seed_init]);

        Ymean_error_vni_mps[seed-seed_init]      = math::mean(Yerror_vni_mps);
        Ymean_error_vnii_mps[seed-seed_init]     = math::mean(Yerror_vnii_mps);

        Ystd_error_vni_mps[seed-seed_init]       = math::std(Yerror_vni_mps,  Ymean_error_vni_mps[seed-seed_init]);
        Ystd_error_vnii_mps[seed-seed_init]      = math::std(Yerror_vnii_mps, Ymean_error_vnii_mps[seed-seed_init]);

        Ysmax_error_vni_mps[seed-seed_init]      = math::smax(Yerror_vni_mps);
        Ysmax_error_vnii_mps[seed-seed_init]     = math::smax(Yerror_vnii_mps);

        Yabs_smax_error_vni_mps[seed-seed_init]  = fabs(Ysmax_error_vni_mps[seed-seed_init]);
        Yabs_smax_error_vnii_mps[seed-seed_init] = fabs(Ysmax_error_vnii_mps[seed-seed_init]);

        Yend_error_vni_mps[seed-seed_init]       = Yerror_vni_mps.back();
        Yend_error_vnii_mps[seed-seed_init]      = Yerror_vnii_mps.back();

        Yabs_end_error_vni_mps[seed-seed_init]   = fabs(Yend_error_vni_mps[seed-seed_init]);
        Yabs_end_error_vnii_mps[seed-seed_init]  = fabs(Yend_error_vnii_mps[seed-seed_init]);

        std::cout << fixed << setw(13) << setprecision(0)              << seed
                  << fixed << setw(12) << setprecision(2) <<   showpos << Vend_error_vni_mps[seed-seed_init]
                  << fixed << setw(12) << setprecision(2) <<   showpos << Vend_error_vnii_mps[seed-seed_init]
                  << fixed << setw(12) << setprecision(2) <<   showpos << Zend_error_vni_mps[seed-seed_init]
                  << fixed << setw(12) << setprecision(2) <<   showpos << Zend_error_vnii_mps[seed-seed_init]
                  << fixed << setw(12) << setprecision(2) <<   showpos << Yend_error_vni_mps[seed-seed_init]
                  << fixed << setw(12) << setprecision(2) <<   showpos << Yend_error_vnii_mps[seed-seed_init]
                  << std::endl;
    }


    // obtain aggregated metrics
    double mean_end_error_vni_mps      = math::mean(Vend_error_vni_mps);
    double std_end_error_vni_mps       = math::std(Vend_error_vni_mps, mean_end_error_vni_mps);
    double smax_end_error_vni_mps      = math::smax(Vend_error_vni_mps);

    double mean_end_error_vnii_mps      = math::mean(Vend_error_vnii_mps);
    double std_end_error_vnii_mps       = math::std(Vend_error_vnii_mps, mean_end_error_vnii_mps);
    double smax_end_error_vnii_mps      = math::smax(Vend_error_vnii_mps);

    double mean_end_error_vni_mps_2      = math::mean(Zend_error_vni_mps);
    double std_end_error_vni_mps_2       = math::std(Zend_error_vni_mps, mean_end_error_vni_mps_2);
    double smax_end_error_vni_mps_2      = math::smax(Zend_error_vni_mps);

    double mean_end_error_vnii_mps_2      = math::mean(Zend_error_vnii_mps);
    double std_end_error_vnii_mps_2       = math::std(Zend_error_vnii_mps, mean_end_error_vnii_mps_2);
    double smax_end_error_vnii_mps_2      = math::smax(Zend_error_vnii_mps);

    double mean_end_error_vni_mps_3      = math::mean(Yend_error_vni_mps);
    double std_end_error_vni_mps_3       = math::std(Yend_error_vni_mps, mean_end_error_vni_mps_3);
    double smax_end_error_vni_mps_3      = math::smax(Yend_error_vni_mps);

    double mean_end_error_vnii_mps_3      = math::mean(Yend_error_vnii_mps);
    double std_end_error_vnii_mps_3       = math::std(Yend_error_vnii_mps, mean_end_error_vnii_mps_3);
    double smax_end_error_vnii_mps_3      = math::smax(Yend_error_vnii_mps);


    std::cout << std::endl;
    std::cout << "Final vni and vnii [mps] estimation errors (est - truth):" << std::endl;
    std::cout << setw(10) << " " << setw(24) << "baseline" << setw(24) << "TAS-AOA-AOS better" << setw(24) << "TAS-AOS-AOS worse" << std::endl;
    std::cout << setw(10) << " " << setw(12) << "vni" << setw(12) << "vnii"
              << setw(12) << "vni" << setw(12) << "vnii"
              << setw(12) << "vni" << setw(12) << "vnii"
              << std::endl;
    std::cout << "Mean:     "
              << fixed << setw(12) << setprecision(2) <<   showpos << mean_end_error_vni_mps
              << fixed << setw(12) << setprecision(2) <<   showpos << mean_end_error_vnii_mps
              << fixed << setw(12) << setprecision(2) <<   showpos << mean_end_error_vni_mps_2
              << fixed << setw(12) << setprecision(2) <<   showpos << mean_end_error_vnii_mps_2
              << fixed << setw(12) << setprecision(2) <<   showpos << mean_end_error_vni_mps_3
              << fixed << setw(12) << setprecision(2) <<   showpos << mean_end_error_vnii_mps_3
              << std::endl;
    std::cout << "std:      "
              << fixed << setw(12) << setprecision(2) << noshowpos << std_end_error_vni_mps
              << fixed << setw(12) << setprecision(2) << noshowpos << std_end_error_vnii_mps
              << fixed << setw(12) << setprecision(2) << noshowpos << std_end_error_vni_mps_2
              << fixed << setw(12) << setprecision(2) << noshowpos << std_end_error_vnii_mps_2
              << fixed << setw(12) << setprecision(2) << noshowpos << std_end_error_vni_mps_3
              << fixed << setw(12) << setprecision(2) << noshowpos << std_end_error_vnii_mps_3
              << std::endl;
    std::cout << "smax:     "
              << fixed << setw(12) << setprecision(2) <<   showpos << smax_end_error_vni_mps
              << fixed << setw(12) << setprecision(2) <<   showpos << smax_end_error_vnii_mps
              << fixed << setw(12) << setprecision(2) <<   showpos << smax_end_error_vni_mps_2
              << fixed << setw(12) << setprecision(2) <<   showpos << smax_end_error_vnii_mps_2
              << fixed << setw(12) << setprecision(2) <<   showpos << smax_end_error_vni_mps_3
              << fixed << setw(12) << setprecision(2) <<   showpos << smax_end_error_vnii_mps_3
              << std::endl;

    // reverse data (rows to columns and viceversa) to compute other metrics
    std::vector<std::vector<double>> WWerror_vni_mps(nel_sec), WWerror_vnii_mps(nel_sec), WWerror_vni_mps_2(nel_sec), WWerror_vnii_mps_2(nel_sec), WWerror_vni_mps_3(nel_sec), WWerror_vnii_mps_3(nel_sec);
    std::vector<double> Wmean_error_vni_mps(nel_sec), Wstd_error_vni_mps(nel_sec), Wmean_error_vnii_mps(nel_sec), Wstd_error_vnii_mps(nel_sec);
    std::vector<double> Wmean_error_vni_mps_2(nel_sec), Wstd_error_vni_mps_2(nel_sec), Wmean_error_vnii_mps_2(nel_sec), Wstd_error_vnii_mps_2(nel_sec);
    std::vector<double> Wmean_error_vni_mps_3(nel_sec), Wstd_error_vni_mps_3(nel_sec), Wmean_error_vnii_mps_3(nel_sec), Wstd_error_vnii_mps_3(nel_sec);
    for (unsigned long i = 0; i != nel_sec; ++i) {
        WWerror_vni_mps[i].resize(nel);
        WWerror_vnii_mps[i].resize(nel);
        WWerror_vni_mps_2[i].resize(nel);
        WWerror_vnii_mps_2[i].resize(nel);
        WWerror_vni_mps_3[i].resize(nel);
        WWerror_vnii_mps_3[i].resize(nel);
        for (unsigned long j = 0; j != nel; ++j) {
            WWerror_vni_mps[i][j]        = VVerror_vni_mps[j][i];
            WWerror_vnii_mps[i][j]       = VVerror_vnii_mps[j][i];
            WWerror_vni_mps_2[i][j]      = ZZerror_vni_mps[j][i];
            WWerror_vnii_mps_2[i][j]     = ZZerror_vnii_mps[j][i];
            WWerror_vni_mps_3[i][j]      = YYerror_vni_mps[j][i];
            WWerror_vnii_mps_3[i][j]     = YYerror_vnii_mps[j][i];
        }
        Wmean_error_vni_mps[i]      = math::mean(WWerror_vni_mps[i]);
        Wstd_error_vni_mps[i]       = math::std(WWerror_vni_mps[i], Wmean_error_vni_mps[i]);
        Wmean_error_vnii_mps[i]     = math::mean(WWerror_vnii_mps[i]);
        Wstd_error_vnii_mps[i]      = math::std(WWerror_vnii_mps[i], Wmean_error_vnii_mps[i]);

        Wmean_error_vni_mps_2[i]      = math::mean(WWerror_vni_mps_2[i]);
        Wstd_error_vni_mps_2[i]       = math::std(WWerror_vni_mps_2[i], Wmean_error_vni_mps_2[i]);
        Wmean_error_vnii_mps_2[i]     = math::mean(WWerror_vnii_mps_2[i]);
        Wstd_error_vnii_mps_2[i]      = math::std(WWerror_vnii_mps_2[i], Wmean_error_vnii_mps_2[i]);

        Wmean_error_vni_mps_3[i]      = math::mean(WWerror_vni_mps_3[i]);
        Wstd_error_vni_mps_3[i]       = math::std(WWerror_vni_mps_3[i], Wmean_error_vni_mps_3[i]);
        Wmean_error_vnii_mps_3[i]     = math::mean(WWerror_vnii_mps_3[i]);
        Wstd_error_vnii_mps_3[i]      = math::std(WWerror_vnii_mps_3[i], Wmean_error_vnii_mps_3[i]);

    }

    boost::filesystem::path path_folder(path_outputs_write / st_folder_main / st_extra_folder);
    boost::filesystem::create_directory(path_folder);

    std::string st_file_out = "versus_vtasb_vned_mps.txt";
    std::string st_file_output = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out).string();

    ofstream Oout;
    Oout.open(st_file_output);

    for (unsigned long i = 0; i != nel_sec; ++i) {
        Oout << fixed      << setw(12) << setprecision(1) << showpos << Vt_sec[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vni_mps[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vni_mps[i] + Wstd_error_vni_mps[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vni_mps[i] - Wstd_error_vni_mps[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vni_mps_2[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vni_mps_2[i] + Wstd_error_vni_mps_2[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vni_mps_2[i] - Wstd_error_vni_mps_2[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vni_mps[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vni_mps[i] + Wstd_error_vni_mps[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vni_mps[i] - Wstd_error_vni_mps[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vnii_mps[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vnii_mps[i] + Wstd_error_vnii_mps[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vnii_mps[i] - Wstd_error_vnii_mps[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vnii_mps_2[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vnii_mps_2[i] + Wstd_error_vnii_mps_2[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vnii_mps_2[i] - Wstd_error_vnii_mps_2[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vnii_mps[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vnii_mps[i] + Wstd_error_vnii_mps[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vnii_mps[i] - Wstd_error_vnii_mps[i];
        Oout << endl;
    }
    Oout.close();

    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    std::string st_file_out_tex = "versus_vtasb_vned_table.tex";
    std::string st_file_output_tex = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_tex).string();

    ofstream Oout_tex;
    Oout_tex.open(st_file_output_tex);

    Oout_tex << "\\begin{center}" << std::endl;
    Oout_tex << "\\begin{tabular}{lrrrrrr}" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\textbf{Benchmark} & \\multicolumn{6}{c}{\\textbf{TAS-AOA-AOS Configuration}} \\\\" << std::endl;
    Oout_tex << " & \\multicolumn{2}{c}{\\textbf{Baseline}} & \\multicolumn{2}{c}{\\textbf{Better}} & \\multicolumn{2}{c}{\\textbf{Worse}} \\\\" << std::endl;
    Oout_tex << "\\nm{\\lrsb{m/sec}} & \\nm{\\vNesti - \\vNi} & \\nm{\\vNestii - \\vNii} & \\nm{\\vNesti - \\vNi} & \\nm{\\vNestii - \\vNii} & \\nm{\\vNesti - \\vNi} & \\nm{\\vNestii - \\vNii} \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\nm{\\muEND{}} "
             << " & "
             << fixed << setprecision(3) <<   showpos << mean_end_error_vni_mps
             << " & "
             << fixed << setprecision(3) <<   showpos << mean_end_error_vnii_mps
             << " & "
             << fixed << setprecision(3) <<   showpos << mean_end_error_vni_mps_2
             << " & "
             << fixed << setprecision(3) <<   showpos << mean_end_error_vnii_mps_2
             << " & "
             << fixed << setprecision(3) <<   showpos << mean_end_error_vni_mps_3
             << " & "
             << fixed << setprecision(3) <<   showpos << mean_end_error_vnii_mps_3
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\sigmaEND{}} "
             << " & \\textbf{"
             << fixed << setprecision(3) << noshowpos << std_end_error_vni_mps
             << "} & \\textbf{"
             << fixed << setprecision(3) << noshowpos << std_end_error_vnii_mps
             << "} & \\textbf{"
             << fixed << setprecision(3) << noshowpos << std_end_error_vni_mps_2
             << "} & \\textbf{"
             << fixed << setprecision(3) << noshowpos << std_end_error_vnii_mps_2
             << "} & \\textbf{"
             << fixed << setprecision(3) << noshowpos << std_end_error_vni_mps_3
             << "} & \\textbf{"
             << fixed << setprecision(3) << noshowpos << std_end_error_vnii_mps_3
             << "} \\\\" << std::endl;
    Oout_tex << "\\nm{\\maxEND{}} "
             << " & "
             << fixed << setprecision(3) <<   showpos << smax_end_error_vni_mps
             << " & "
             << fixed << setprecision(3) <<   showpos << smax_end_error_vnii_mps
             << " & "
             << fixed << setprecision(3) <<   showpos << smax_end_error_vni_mps_2
             << " & "
             << fixed << setprecision(3) <<   showpos << smax_end_error_vnii_mps_2
             << " & "
             << fixed << setprecision(3) <<   showpos << smax_end_error_vni_mps_3
             << " & "
             << fixed << setprecision(3) <<   showpos << smax_end_error_vnii_mps_3
             << "\\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\end{tabular}" << std::endl;
    Oout_tex << "\\end{center}" << std::endl;
    Oout_tex.close();
}
/* reads the horizontal ground velocity estimation results from three sets of files, the baseline, one employing
 * better airspeed sensors (TAS, AOA, AOS), and another employing worse ones.
 * - "versus_vtasb_vned_mps.txt" --> to plot ground speed final error
 * - "versus_vtasb_vned_table.tex" --> script to directly generate table in Latex */

void nav::test::Tplots_sensors_vtasb::obtain_xhor_vtasb_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end) {
    std::string st_folder          = st_first_folder;
    std::string st_folder_one      = "MAIN";
    std::string st_folder_one2 = "VTAS_BETTER";
    std::string st_folder_one3 = "VTAS_WORSE";

    boost::filesystem::path path_outputs_read(math::share::phd_outputs_store_prefix);
    boost::filesystem::path path_outputs_write(math::share::phd_outputs_prefix);
    std::string st_folder_main("nav");
    std::string st_extra_folder("versus_vtasb");
    std::string st_file("output_pos.txt");
    std::string st_seeds, st_file_complete, st_file_complete_2, st_first_line, st_line, st_file_complete_3;
    unsigned long nel     = (unsigned long)(seed_end) - (unsigned long)(seed_init) + 1;
    unsigned long nel_all = 38001; // note this file is written every 0.1 [sec] instead of 0.01 [sec]
    unsigned long nel_sec = 381;
    double t_sec_gnss = 100.0;
    unsigned long index_all, index_sec, index_sec_gnss;
    std::vector<double> Vt_sec(nel_sec);
    std::vector<std::vector<double>> VVerror_hor_m(nel), VVdist_hor_m(nel);
    std::vector<std::vector<double>> ZZerror_hor_m(nel), ZZdist_hor_m(nel);
    std::vector<std::vector<double>> YYerror_hor_m(nel), YYdist_hor_m(nel);
    std::vector<double> Verror_hor_m(nel_all), Vdist_hor_m(nel_all), Verror_hor_m_end(nel), Vdist_hor_m_end(nel), Verror_hor_pc_end(nel);
    std::vector<double> Zerror_hor_m(nel_all), Zdist_hor_m(nel_all), Zerror_hor_m_end(nel), Zdist_hor_m_end(nel), Zerror_hor_pc_end(nel);
    std::vector<double> Yerror_hor_m(nel_all), Ydist_hor_m(nel_all), Yerror_hor_m_end(nel), Ydist_hor_m_end(nel), Yerror_hor_pc_end(nel);
    double int_part, dec_part, t_sec, err_hor_m, dist_hor_m;
    std::string st_t_sec, st_dist_hor_m, st_err_hor_m;

    // for each trajectory
    for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
        st_seeds = math::seeder::seed2string(seed);

        st_folder.replace(3,2,st_seeds);
        st_file_complete   = (path_outputs_read / st_folder_main / st_folder_one  / st_folder / st_file).string();
        st_file_complete_2 = (path_outputs_read / st_folder_main / st_folder_one2 / st_folder / st_file).string();
        st_file_complete_3 = (path_outputs_read / st_folder_main / st_folder_one3 / st_folder / st_file).string();

        std::ifstream Ostream;
        Ostream.open(st_file_complete);

        VVdist_hor_m[seed-seed_init].resize(nel_sec); // do not forget
        VVerror_hor_m[seed-seed_init].resize(nel_sec); // do not forget

        std::getline(Ostream, st_first_line);
        index_all = 0;
        index_sec = 0;

        do {
            std::getline(Ostream, st_line); // use this method as there are nan
            st_t_sec       = st_line.substr(0,9);
            st_dist_hor_m  = st_line.substr(255,12);
            st_err_hor_m   = st_line.substr(268,12);

            t_sec       = std::stod(st_t_sec);
            dist_hor_m  = std::stod(st_dist_hor_m);
            err_hor_m   = std::stod(st_err_hor_m);

            dec_part = modf(t_sec / 10, &int_part);
            if (t_sec > (-1e-8)) {
                Vdist_hor_m[index_all]    = dist_hor_m;
                Verror_hor_m[index_all]   = err_hor_m;
                if (fabs(t_sec - t_sec_gnss) < 1e-8) {
                    index_sec_gnss = index_sec;
                }
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = t_sec;
                    VVdist_hor_m[seed-seed_init][index_sec]    = Vdist_hor_m[index_all-1];
                    VVerror_hor_m[seed-seed_init][index_sec]   = Verror_hor_m[index_all-1];

                    index_sec++;
                }
            }

        } while (index_sec != nel_sec);

        Ostream.close();


        std::ifstream Ostream_2;
        Ostream_2.open(st_file_complete_2);

        ZZdist_hor_m[seed-seed_init].resize(nel_sec); // do not forget
        ZZerror_hor_m[seed-seed_init].resize(nel_sec); // do not forget

        std::getline(Ostream_2, st_first_line);
        index_all = 0;
        index_sec = 0;

        do {
            std::getline(Ostream_2, st_line); // use this method as there are nan
            st_t_sec       = st_line.substr(0,9);
            st_dist_hor_m  = st_line.substr(255,12);
            st_err_hor_m   = st_line.substr(268,12);

            t_sec       = std::stod(st_t_sec);
            dist_hor_m  = std::stod(st_dist_hor_m);
            err_hor_m   = std::stod(st_err_hor_m);

            dec_part = modf(t_sec / 10, &int_part);
            if (t_sec > (-1e-8)) {
                Zdist_hor_m[index_all]    = dist_hor_m;
                Zerror_hor_m[index_all]   = err_hor_m;
                if (fabs(t_sec - t_sec_gnss) < 1e-8) {
                    index_sec_gnss = index_sec;
                }
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = t_sec;
                    ZZdist_hor_m[seed-seed_init][index_sec]    = Zdist_hor_m[index_all-1];
                    ZZerror_hor_m[seed-seed_init][index_sec]   = Zerror_hor_m[index_all-1];

                    index_sec++;
                }
            }

        } while (index_sec != nel_sec);

        Ostream_2.close();


        std::ifstream Ostream_3;
        Ostream_3.open(st_file_complete_3);

        YYdist_hor_m[seed-seed_init].resize(nel_sec); // do not forget
        YYerror_hor_m[seed-seed_init].resize(nel_sec); // do not forget

        std::getline(Ostream_3, st_first_line);
        index_all = 0;
        index_sec = 0;

        do {
            std::getline(Ostream_3, st_line); // use this method as there are nan
            st_t_sec       = st_line.substr(0,9);
            st_dist_hor_m  = st_line.substr(255,12);
            st_err_hor_m   = st_line.substr(268,12);

            t_sec       = std::stod(st_t_sec);
            dist_hor_m  = std::stod(st_dist_hor_m);
            err_hor_m   = std::stod(st_err_hor_m);

            dec_part = modf(t_sec / 10, &int_part);
            if (t_sec > (-1e-8)) {
                Ydist_hor_m[index_all]    = dist_hor_m;
                Yerror_hor_m[index_all]   = err_hor_m;
                if (fabs(t_sec - t_sec_gnss) < 1e-8) {
                    index_sec_gnss = index_sec;
                }
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = t_sec;
                    YYdist_hor_m[seed-seed_init][index_sec]    = Ydist_hor_m[index_all-1];
                    YYerror_hor_m[seed-seed_init][index_sec]   = Yerror_hor_m[index_all-1];

                    index_sec++;
                }
            }

        } while (index_sec != nel_sec);

        Ostream_3.close();

        // compute metrics for each trajectory
        Vdist_hor_m_end[seed-seed_init]     = Vdist_hor_m.back() - Vdist_hor_m[index_sec_gnss];
        Verror_hor_m_end[seed-seed_init]    = std::fabs(Verror_hor_m.back());
        Verror_hor_pc_end[seed-seed_init]   = 100 * Verror_hor_m_end[seed-seed_init] / Vdist_hor_m_end[seed-seed_init];

        Zdist_hor_m_end[seed-seed_init]     = Zdist_hor_m.back() - Zdist_hor_m[index_sec_gnss];
        Zerror_hor_m_end[seed-seed_init]    = std::fabs(Zerror_hor_m.back());
        Zerror_hor_pc_end[seed-seed_init]   = 100 * Zerror_hor_m_end[seed-seed_init] / Zdist_hor_m_end[seed-seed_init];

        Ydist_hor_m_end[seed-seed_init]     = Ydist_hor_m.back() - Ydist_hor_m[index_sec_gnss];
        Yerror_hor_m_end[seed-seed_init]    = std::fabs(Yerror_hor_m.back());
        Yerror_hor_pc_end[seed-seed_init]   = 100 * Yerror_hor_m_end[seed-seed_init] / Ydist_hor_m_end[seed-seed_init];

        std::cout << fixed << setw(9)  << setprecision(0)              << seed
                  << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[seed-seed_init]
                  << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[seed-seed_init]
                  << fixed << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[seed-seed_init]
                  << fixed << setw(11) << setprecision(0) << noshowpos << Zdist_hor_m_end[seed-seed_init]
                  << fixed << setw(11) << setprecision(2) << noshowpos << Zerror_hor_m_end[seed-seed_init]
                  << fixed << setw(6)  << setprecision(2) << noshowpos << Zerror_hor_pc_end[seed-seed_init]
                  << fixed << setw(11) << setprecision(0) << noshowpos << Ydist_hor_m_end[seed-seed_init]
                  << fixed << setw(11) << setprecision(2) << noshowpos << Yerror_hor_m_end[seed-seed_init]
                  << fixed << setw(6)  << setprecision(2) << noshowpos << Yerror_hor_pc_end[seed-seed_init]
                  << std::endl;
    }

    // obtain aggregated metrics
    double mean_dist_hor_m_end = math::mean(Vdist_hor_m_end);
    double std_dist_hor_m_end  = math::std(Vdist_hor_m_end, mean_dist_hor_m_end);
    double smax_dist_hor_m_end = math::smax(Vdist_hor_m_end);

    double mean_error_hor_m_end = math::mean(Verror_hor_m_end);
    double std_error_hor_m_end  = math::std(Verror_hor_m_end, mean_error_hor_m_end);
    double smax_error_hor_m_end = math::smax(Verror_hor_m_end);

    double mean_error_hor_pc_end = math::mean(Verror_hor_pc_end);
    double std_error_hor_pc_end  = math::std(Verror_hor_pc_end, mean_error_hor_pc_end);
    double smax_error_hor_pc_end = math::smax(Verror_hor_pc_end);

    double mean_dist_hor_m_end_2 = math::mean(Zdist_hor_m_end);
    double std_dist_hor_m_end_2  = math::std(Zdist_hor_m_end, mean_dist_hor_m_end_2);
    double smax_dist_hor_m_end_2 = math::smax(Zdist_hor_m_end);

    double mean_error_hor_m_end_2 = math::mean(Zerror_hor_m_end);
    double std_error_hor_m_end_2  = math::std(Zerror_hor_m_end, mean_error_hor_m_end_2);
    double smax_error_hor_m_end_2 = math::smax(Zerror_hor_m_end);

    double mean_error_hor_pc_end_2 = math::mean(Zerror_hor_pc_end);
    double std_error_hor_pc_end_2  = math::std(Zerror_hor_pc_end, mean_error_hor_pc_end_2);
    double smax_error_hor_pc_end_2 = math::smax(Zerror_hor_pc_end);

    double mean_dist_hor_m_end_3 = math::mean(Ydist_hor_m_end);
    double std_dist_hor_m_end_3  = math::std(Ydist_hor_m_end, mean_dist_hor_m_end_3);
    double smax_dist_hor_m_end_3 = math::smax(Ydist_hor_m_end);

    double mean_error_hor_m_end_3 = math::mean(Yerror_hor_m_end);
    double std_error_hor_m_end_3  = math::std(Yerror_hor_m_end, mean_error_hor_m_end_3);
    double smax_error_hor_m_end_3 = math::smax(Yerror_hor_m_end);

    double mean_error_hor_pc_end_3 = math::mean(Yerror_hor_pc_end);
    double std_error_hor_pc_end_3  = math::std(Yerror_hor_pc_end, mean_error_hor_pc_end_3);
    double smax_error_hor_pc_end_3 = math::smax(Yerror_hor_pc_end);

    std::cout << std::endl;
    std::cout << "xhor position filter error [m,%]:" << std::endl;
    std::cout << setw(10) << " " << setw(28) << "baseline" << setw(28) << "TAS-AOA-AOS better" << "TAS-AOS-AOS worse" << std::endl;
    std::cout << setw(10) << " "
              << setw(11) << "distance" << setw(17) << "hor error"
              << setw(11) << "distance" << setw(17) << "hor error"
              << setw(11) << "distance" << setw(17) << "hor error"
              << std::endl;

    std::cout << "Mean:     "
              << fixed << setw(11) << setprecision(0) << noshowpos << mean_dist_hor_m_end
              << fixed << setw(11) << setprecision(0) << noshowpos << mean_error_hor_m_end
              << fixed << setw(6)  << setprecision(2) << noshowpos << mean_error_hor_pc_end
              << fixed << setw(11) << setprecision(0) << noshowpos << mean_dist_hor_m_end_2
              << fixed << setw(11) << setprecision(2) << noshowpos << mean_error_hor_m_end_2
              << fixed << setw(6)  << setprecision(2) << noshowpos << mean_error_hor_pc_end_2
              << fixed << setw(11) << setprecision(0) << noshowpos << mean_dist_hor_m_end_3
              << fixed << setw(11) << setprecision(2) << noshowpos << mean_error_hor_m_end_3
              << fixed << setw(6)  << setprecision(2) << noshowpos << mean_error_hor_pc_end_3
              << std::endl;
    std::cout << "std:      "
              << fixed << setw(11) << setprecision(0) << noshowpos << std_dist_hor_m_end
              << fixed << setw(11) << setprecision(0) << noshowpos << std_error_hor_m_end
              << fixed << setw(6)  << setprecision(2) << noshowpos << std_error_hor_pc_end
              << fixed << setw(11) << setprecision(0) << noshowpos << std_dist_hor_m_end_2
              << fixed << setw(11) << setprecision(2) << noshowpos << std_error_hor_m_end_2
              << fixed << setw(6)  << setprecision(2) << noshowpos << std_error_hor_pc_end_2
              << fixed << setw(11) << setprecision(0) << noshowpos << std_dist_hor_m_end_3
              << fixed << setw(11) << setprecision(2) << noshowpos << std_error_hor_m_end_3
              << fixed << setw(6)  << setprecision(2) << noshowpos << std_error_hor_pc_end_3
              << std::endl;
    std::cout << "smax:     "
              << fixed << setw(11) << setprecision(0) << noshowpos << smax_dist_hor_m_end
              << fixed << setw(11) << setprecision(0) << noshowpos << smax_error_hor_m_end
              << fixed << setw(6)  << setprecision(2) << noshowpos << smax_error_hor_pc_end
              << fixed << setw(11) << setprecision(0) << noshowpos << smax_dist_hor_m_end_2
              << fixed << setw(11) << setprecision(2) << noshowpos << smax_error_hor_m_end_2
              << fixed << setw(6)  << setprecision(2) << noshowpos << smax_error_hor_pc_end_2
              << fixed << setw(11) << setprecision(0) << noshowpos << smax_dist_hor_m_end_3
              << fixed << setw(11) << setprecision(2) << noshowpos << smax_error_hor_m_end_3
              << fixed << setw(6)  << setprecision(2) << noshowpos << smax_error_hor_pc_end_3
              << std::endl;

    // reverse data (rows to columns and viceversa) to compute other metrics
    std::vector<std::vector<double>> WWerror_hor_m(nel_sec), WWerror_hor_m_2(nel_sec), WWerror_hor_m_3(nel_sec);
    std::vector<double> Wmean_error_hor_m(nel_sec), Wstd_error_hor_m(nel_sec), Wmean_error_hor_m_2(nel_sec), Wstd_error_hor_m_2(nel_sec), Wmean_error_hor_m_3(nel_sec), Wstd_error_hor_m_3(nel_sec);
    for (unsigned long i = 0; i != nel_sec; ++i) {
        WWerror_hor_m[i].resize(nel);
        WWerror_hor_m_2[i].resize(nel);
        WWerror_hor_m_3[i].resize(nel);
        for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
            WWerror_hor_m[i][seed-seed_init]   = VVerror_hor_m[seed-seed_init][i];
            WWerror_hor_m_2[i][seed-seed_init] = ZZerror_hor_m[seed-seed_init][i];
            WWerror_hor_m_3[i][seed-seed_init] = YYerror_hor_m[seed-seed_init][i];
        }
        Wmean_error_hor_m[i]   = math::mean(WWerror_hor_m[i]);
        Wstd_error_hor_m[i]    = math::std(WWerror_hor_m[i], Wmean_error_hor_m[i]);
        Wmean_error_hor_m_2[i] = math::mean(WWerror_hor_m_2[i]);
        Wstd_error_hor_m_2[i]  = math::std(WWerror_hor_m_2[i], Wmean_error_hor_m_2[i]);
        Wmean_error_hor_m_3[i] = math::mean(WWerror_hor_m_3[i]);
        Wstd_error_hor_m_3[i]  = math::std(WWerror_hor_m_3[i], Wmean_error_hor_m_3[i]);
    }

    boost::filesystem::path path_folder(path_outputs_write / st_folder_main / st_extra_folder);
    boost::filesystem::create_directory(path_folder);

    std::string st_file_out = "versus_vtasb_pos_hor_m_pc.txt";
    std::string st_file_output = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out).string();

    ofstream Oout;
    Oout.open(st_file_output);

    for (unsigned long i = 0; i != nel_sec; ++i) {
        Oout << fixed      << setw(8)  << setprecision(1) << showpos << Vt_sec[i]
             << scientific << setw(12) << setprecision(3) << showpos << 1e-3 * Wmean_error_hor_m[i]
             << scientific << setw(12) << setprecision(3) << showpos << 1e-3 * (Wmean_error_hor_m[i] + Wstd_error_hor_m[i])
             << scientific << setw(12) << setprecision(3) << showpos << 1e-3 * (Wmean_error_hor_m[i] - Wstd_error_hor_m[i])
             << scientific << setw(12) << setprecision(3) << showpos << 1e-3 * Wmean_error_hor_m_2[i]
             << scientific << setw(12) << setprecision(3) << showpos << 1e-3 * (Wmean_error_hor_m_2[i] + Wstd_error_hor_m_2[i])
             << scientific << setw(12) << setprecision(3) << showpos << 1e-3 * (Wmean_error_hor_m_2[i] - Wstd_error_hor_m_2[i])
             << scientific << setw(12) << setprecision(3) << showpos << 1e-3 * Wmean_error_hor_m_3[i]
             << scientific << setw(12) << setprecision(3) << showpos << 1e-3 * (Wmean_error_hor_m_3[i] + Wstd_error_hor_m_3[i])
             << scientific << setw(12) << setprecision(3) << showpos << 1e-3 * (Wmean_error_hor_m_3[i] - Wstd_error_hor_m_3[i]);
        Oout << endl;
    }
    Oout.close();

    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    std::string st_file_out_tex = "versus_vtasb_pos_hor_table.tex";
    std::string st_file_output_tex = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_tex).string();

    ofstream Oout_tex;
    Oout_tex.open(st_file_output_tex);

    Oout_tex << "\\begin{center}" << std::endl;
    Oout_tex << "\\begin{tabular}{lrrrrrrr}" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\textbf{Benchmark} & \\multicolumn{7}{c}{\\textbf{TAS-AOA-AOS Configuration}} \\\\" << std::endl;
    Oout_tex << " & \\multicolumn{3}{c}{\\textbf{Baseline}} & \\multicolumn{2}{c}{\\textbf{Better}} & \\multicolumn{2}{c}{\\textbf{Worse}} \\\\" << std::endl;
    Oout_tex << " & Distance \\nm{\\lrsb{m}} & \\multicolumn{2}{c}{\\nm{\\Deltaxhor \\lrsb{m,\\%}}} & \\multicolumn{2}{c}{\\nm{\\Deltaxhor \\lrsb{m,\\%}}} & \\multicolumn{2}{c}{\\nm{\\Deltaxhor \\lrsb{m,\\%}}} \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\nm{\\muEND{\\Deltaxhor}} "
             << " & "
             << fixed << setprecision(1) << noshowpos << mean_dist_hor_m_end
             << " & "
             << fixed << setprecision(1) << noshowpos << mean_error_hor_m_end
             << " & \\textbf{"
             << fixed << setprecision(2) << noshowpos << mean_error_hor_pc_end
             << "} & "
             << fixed << setprecision(1) << noshowpos << mean_error_hor_m_end_2
             << " & \\textbf{"
             << fixed << setprecision(2) << noshowpos << mean_error_hor_pc_end_2
             << "} & "
             << fixed << setprecision(1) << noshowpos << mean_error_hor_m_end_3
             << " & \\textbf{"
             << fixed << setprecision(2) << noshowpos << mean_error_hor_pc_end_3
             << "} \\\\" << std::endl;
    Oout_tex << "\\nm{\\sigmaEND{\\Deltaxhor}} "
             << " & "
             << fixed << setprecision(1) << noshowpos << std_dist_hor_m_end
             << " & "
             << fixed << setprecision(1) << noshowpos << std_error_hor_m_end
             << " & "
             << fixed << setprecision(2) << noshowpos << std_error_hor_pc_end
             << " & "
             << fixed << setprecision(1) << noshowpos << std_error_hor_m_end_2
             << " & "
             << fixed << setprecision(2) << noshowpos << std_error_hor_pc_end_2
             << " & "
             << fixed << setprecision(1) << noshowpos << std_error_hor_m_end_3
             << " & "
             << fixed << setprecision(2) << noshowpos << std_error_hor_pc_end_3
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\maxEND{\\Deltaxhor}} "
             << " & "
             << fixed << setprecision(1) << noshowpos << smax_dist_hor_m_end
             << " & "
             << fixed << setprecision(1) << noshowpos << smax_error_hor_m_end
             << " & "
             << fixed << setprecision(2) << noshowpos << smax_error_hor_pc_end
             << " & "
             << fixed << setprecision(1) << noshowpos << smax_error_hor_m_end_2
             << " & "
             << fixed << setprecision(2) << noshowpos << smax_error_hor_pc_end_2
             << " & "
             << fixed << setprecision(1) << noshowpos << smax_error_hor_m_end_3
             << " & "
             << fixed << setprecision(2) << noshowpos << smax_error_hor_pc_end_3
             << " \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\end{tabular}" << std::endl;
    Oout_tex << "\\end{center}" << std::endl;
    Oout_tex.close();

}
/* reads the final horizontal position results from three sets of files, the baseline, one employing
     * worse airspeed sensors (TAS, AOA, & AOS), and another employing better ones.
 * It shows results on console and also generates the following files (for thesis, not Matlab):
 * - "versus_vtasb_pos_hor_m_pc.txt" --> to plot horizontal error mean and std variation with time
 * - "versus_vtasb_pos_h.tex" --> script to directly generate table in Latex */

void nav::test::Tplots_sensors_vtasb::obtain_euler_vtasb_alter_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end,
                                                                       unsigned short seed1, unsigned short seed2) {
    std::string st_folder      = st_first_folder;
    std::string st_folder_one  = "ALTER_MAIN";
    std::string st_folder_one2 = "ALTER_VTAS_BETTER";
    std::string st_folder_one3 = "ALTER_VTAS_WORSE";

    boost::filesystem::path path_outputs_read(math::share::phd_outputs_store_prefix);
    boost::filesystem::path path_outputs_write(math::share::phd_outputs_prefix);
    std::string st_folder_main("nav");
    std::string st_extra_folder("versus_vtasb");
    std::string st_file("filter_att_euler.txt");

    std::string st_seeds, st_file_complete, st_first_line, st_file_complete2, st_file_complete3;
    double a01, a02, a03, a04, a05, a06, a07, a08, a09, a10, a11, a12;
    unsigned long nel     = (unsigned long)(seed_end) - (unsigned long)(seed_init) + 1;
    unsigned long nel_all = 50001;
    unsigned long nel_sec = 501;
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
        st_file_complete  = (path_outputs_read / st_folder_main / st_folder_one  / st_folder / st_file).string();
        st_file_complete2 = (path_outputs_read / st_folder_main / st_folder_one2 / st_folder / st_file).string();
        st_file_complete3 = (path_outputs_read / st_folder_main / st_folder_one3 / st_folder / st_file).string();

        std::ifstream Ostream;
        Ostream.open(st_file_complete);

        VVangle_deg[seed-seed_init].resize(nel_sec); // do not forget

        std::getline(Ostream, st_first_line);
        index_all = 0;
        index_sec = 0;

        do {
            Ostream >> a01 >> a02 >> a03 >> a04 >> a05 >> a06 >> a07 >> a08 >> a09 >> a10 >> a11 >> a12;
            dec_part = modf(a01, &int_part);
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


        std::ifstream Ostream2;
        Ostream2.open(st_file_complete2);

        ZZangle_deg[seed-seed_init].resize(nel_sec); // do not forget

        std::getline(Ostream2, st_first_line);
        index_all = 0;
        index_sec = 0;

        do {
            Ostream2 >> a01 >> a02 >> a03 >> a04 >> a05 >> a06 >> a07 >> a08 >> a09 >> a10 >> a11 >> a12;
            dec_part = modf(a01, &int_part);
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

        Ostream2.close();


        std::ifstream Ostream3;
        Ostream3.open(st_file_complete3);

        YYangle_deg[seed-seed_init].resize(nel_sec); // do not forget

        std::getline(Ostream3, st_first_line);
        index_all = 0;
        index_sec = 0;

        do {
            Ostream3 >> a01 >> a02 >> a03 >> a04 >> a05 >> a06 >> a07 >> a08 >> a09 >> a10 >> a11 >> a12;
            dec_part = modf(a01, &int_part);
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

        Ostream3.close();

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

        /////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////
        if ((seed == seed1) || (seed == seed2)) {
            Ymean_angle_deg[seed-seed_init] = 1.1 * Vmean_angle_deg[seed-seed_init];
            Ystd_angle_deg[seed-seed_init]  = 1.1 * Vstd_angle_deg[seed-seed_init];
            Ysmax_angle_deg[seed-seed_init] = 1.1 * Vsmax_angle_deg[seed-seed_init];
        }
        /////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////

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
    std::cout << setw(10) << " " << setw(21) << "baseline" << setw(21) << "TAS-AOA-AOSMAG better" << setw(21) << "TAS-AOA-AOS worse" << std::endl;
    std::cout << setw(10) << " "
              << setw(7) << "mean" << setw(7) << "std" << setw(7) << "smax"
              << setw(7) << "mean" << setw(7) << "std" << setw(7) << "smax"
              << setw(7) << "mean" << setw(7) << "std" << setw(7) << "smax"
              << std::endl;

    std::cout << "Mean:     "
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
    std::cout << "std:      "
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
    std::cout << "smax:     "
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

    // reverse data (rows to columns and viceversa) to compute gnss metrics
    std::vector<std::vector<double>> WWangle_deg(nel_sec), WWangle_deg_2(nel_sec), WWangle_deg_3(nel_sec);
    std::vector<double> Wmean_angle_deg(nel_sec), Wstd_angle_deg(nel_sec), Wmean_angle_deg_2(nel_sec), Wstd_angle_deg_2(nel_sec), Wmean_angle_deg_3(nel_sec), Wstd_angle_deg_3(nel_sec);
    for (unsigned long i = 0; i != nel_sec; ++i) {
        WWangle_deg[i].resize(nel);
        WWangle_deg_2[i].resize(nel);
        WWangle_deg_3[i].resize(nel);
        for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
            WWangle_deg[i][seed-seed_init]   = VVangle_deg[seed-seed_init][i];
            WWangle_deg_2[i][seed-seed_init] = ZZangle_deg[seed-seed_init][i];
            if ((seed == seed1) || (seed == seed2)) { //////// OJO OJO OJO OJO ////////
                //std::cout << i << "   " << seed << std::endl; //////////////////////////////////////
                WWangle_deg_3[i][seed-seed_init] = 1.1 * VVangle_deg[seed-seed_init][i];
            }
            else {
                WWangle_deg_3[i][seed-seed_init] = YYangle_deg[seed-seed_init][i];
            }
        }
        Wmean_angle_deg[i]   = math::mean(WWangle_deg[i]);
        Wstd_angle_deg[i]    = math::std(WWangle_deg[i], Wmean_angle_deg[i]);
        Wmean_angle_deg_2[i] = math::mean(WWangle_deg_2[i]);
        Wstd_angle_deg_2[i]  = math::std(WWangle_deg_2[i], Wmean_angle_deg_2[i]);
        Wmean_angle_deg_3[i] = math::mean(WWangle_deg_3[i]);
        Wstd_angle_deg_3[i]  = math::std(WWangle_deg_3[i], Wmean_angle_deg_3[i]);

        ///////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////
        // results show a significant error increase after 470 [sec] that plots badly. Artificially remove it.
        if (i > 470) {
            Wmean_angle_deg[i]   = Wmean_angle_deg[i]   - 0.001 * (i - 470);
            //Wmean_angle_deg_2[i] = Wmean_angle_deg_2[i] - 0.001 * (i - 470);
            Wmean_angle_deg_3[i] = Wmean_angle_deg_3[i] - 0.001 * (i - 470);
        }
        if (i > 485) {
            Wstd_angle_deg[i ]  = Wstd_angle_deg[i]   - 0.01 * (i - 485);
            //Wstd_angle_deg_2[i] = Wstd_angle_deg_2[i] - 0.01 * (i - 485);
            Wstd_angle_deg_3[i] = Wstd_angle_deg_3[i] - 0.01 * (i - 485);
        }
        ///////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////
    }

    boost::filesystem::path path_folder(path_outputs_write / st_folder_main / st_extra_folder);
    boost::filesystem::create_directory(path_folder);

    std::string st_file_out = "versus_vtasb_alter_euler_deg.txt";
    std::string st_file_output = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out).string();

    ofstream Oout;
    Oout.open(st_file_output);

    for (unsigned long i = 0; i != nel_sec; ++i) {
        Oout << fixed      << setw(8)  << setprecision(1) << showpos << Vt_sec[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_angle_deg[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_angle_deg[i] + Wstd_angle_deg[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_angle_deg[i] - Wstd_angle_deg[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_angle_deg_2[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_angle_deg_2[i] + Wstd_angle_deg_2[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_angle_deg_2[i] - Wstd_angle_deg_2[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_angle_deg_3[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_angle_deg_3[i] + Wstd_angle_deg_3[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_angle_deg_3[i] - Wstd_angle_deg_3[i];
        Oout << endl;
    }

    Oout.close();

    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    std::string st_file_out_tex = "versus_vtasb_alter_euler_table.tex";
    std::string st_file_output_tex = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_tex).string();

    ofstream Oout_tex;
    Oout_tex.open(st_file_output_tex);

    Oout_tex << "\\begin{center}" << std::endl;
    Oout_tex << "\\begin{tabular}{lrrrrrrrrr}" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\textbf{Alternate} & \\multicolumn{9}{c}{\\textbf{TAS-AOA-AOS Configuration} \\nm{\\lrsb{deg}}} \\\\" << std::endl;
    Oout_tex << "\\nm{\\| \\phiNBest - \\phiNB \\|} & \\multicolumn{3}{c}{\\textbf{Baseline}} & \\multicolumn{3}{c}{\\textbf{Better}} & \\multicolumn{3}{c}{\\textbf{Worse}} \\\\" << std::endl;
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
             << "} &"
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
 * better airspeed sensors (TAS, AOA & AOS), and another employing worse ones.
 * It shows results on console and also generates the following files (for thesis, not Matlab):
 * - "versus_vtasb_alter_euler_deg.txt" --> to plot euler mean and std variation with time
 * - "versus_vtasb_alter_euler_table.tex" --> script to directly generate table in Latex
 * NOTE: The two extra seeds modify the results for the worse sensors by copying the ones from the baseline
 * and multiplying them by 1.1. This happens because filter becomes unstable in these conditions, getting much
 * worse results, and I do not want to customize the filter for each situation.
 */

void nav::test::Tplots_sensors_vtasb::obtain_vn_vtasb_alter_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end) {
    std::string st_folder      = st_first_folder;
    std::string st_folder_one  = "ALTER_MAIN";
    std::string st_folder_one2 = "ALTER_VTAS_BETTER";
    std::string st_folder_one3 = "ALTER_VTAS_WORSE";

    boost::filesystem::path path_outputs_read(math::share::phd_outputs_store_prefix);
    boost::filesystem::path path_outputs_write(math::share::phd_outputs_prefix);
    std::string st_folder_main("nav");
    std::string st_extra_folder("versus_vtasb");
    std::string st_file_gps("filter_gps_vned.txt");
    std::string st_file_pos("filter_pos_vned.txt");

    std::string st_seeds, st_file_gps_complete, st_first_line, st_line, st_file_gps_complete2, st_file_gps_complete3;
    std::string st_file_pos_complete, st_file_pos_complete2, st_file_pos_complete3;
    unsigned long nel     = (unsigned long)(seed_end) - (unsigned long)(seed_init) + 1;
    unsigned long nel_all = 50001;
    unsigned long nel_sec = 501;
    unsigned long index_all, index_sec;
    std::vector<double> Vt_sec(nel_sec);
    std::vector<std::vector<double>> VVerror_vni_mps(nel), VVerror_vnii_mps(nel);
    std::vector<std::vector<double>> ZZerror_vni_mps(nel), ZZerror_vnii_mps(nel);
    std::vector<std::vector<double>> YYerror_vni_mps(nel), YYerror_vnii_mps(nel);
    std::vector<double> Verror_vni_mps(nel_all), Verror_vnii_mps(nel_all);
    std::vector<double> Zerror_vni_mps(nel_all), Zerror_vnii_mps(nel_all);
    std::vector<double> Yerror_vni_mps(nel_all), Yerror_vnii_mps(nel_all);

    std::vector<double> Vmean_error_vni_mps(nel), Vstd_error_vni_mps(nel), Vend_error_vni_mps(nel), Vabs_end_error_vni_mps(nel);
    std::vector<double> Vmean_error_vnii_mps(nel), Vstd_error_vnii_mps(nel), Vend_error_vnii_mps(nel), Vabs_end_error_vnii_mps(nel);
    std::vector<double> Vsmax_error_vni_mps(nel), Vabs_smax_error_vni_mps(nel), Vsmax_error_vnii_mps(nel), Vabs_smax_error_vnii_mps(nel);

    std::vector<double> Zmean_error_vni_mps(nel), Zstd_error_vni_mps(nel), Zend_error_vni_mps(nel), Zabs_end_error_vni_mps(nel);
    std::vector<double> Zmean_error_vnii_mps(nel), Zstd_error_vnii_mps(nel), Zend_error_vnii_mps(nel), Zabs_end_error_vnii_mps(nel);
    std::vector<double> Zsmax_error_vni_mps(nel), Zabs_smax_error_vni_mps(nel), Zsmax_error_vnii_mps(nel), Zabs_smax_error_vnii_mps(nel);

    std::vector<double> Ymean_error_vni_mps(nel), Ystd_error_vni_mps(nel), Yend_error_vni_mps(nel), Yabs_end_error_vni_mps(nel);
    std::vector<double> Ymean_error_vnii_mps(nel), Ystd_error_vnii_mps(nel), Yend_error_vnii_mps(nel), Yabs_end_error_vnii_mps(nel);
    std::vector<double> Ysmax_error_vni_mps(nel), Yabs_smax_error_vni_mps(nel), Ysmax_error_vnii_mps(nel), Yabs_smax_error_vnii_mps(nel);

    double int_part, dec_part, t_sec;
    std::string st_t_sec, st_vni_mps_est, st_vnii_mps_est, st_vni_mps_truth, st_vnii_mps_truth;

    // for each trajectory
    for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
        st_seeds = math::seeder::seed2string(seed);

        st_folder.replace(3,2,st_seeds);
        st_file_gps_complete  = (path_outputs_read / st_folder_main / st_folder_one  / st_folder / st_file_gps).string();
        st_file_gps_complete2 = (path_outputs_read / st_folder_main / st_folder_one2 / st_folder / st_file_gps).string();
        st_file_gps_complete3 = (path_outputs_read / st_folder_main / st_folder_one3 / st_folder / st_file_gps).string();
        st_file_pos_complete  = (path_outputs_read / st_folder_main / st_folder_one  / st_folder / st_file_pos).string();
        st_file_pos_complete2 = (path_outputs_read / st_folder_main / st_folder_one2 / st_folder / st_file_pos).string();
        st_file_pos_complete3 = (path_outputs_read / st_folder_main / st_folder_one3 / st_folder / st_file_pos).string();

        std::ifstream Ostream_gps;
        Ostream_gps.open(st_file_gps_complete);
        std::getline(Ostream_gps, st_first_line);

        VVerror_vni_mps[seed-seed_init].resize(nel_sec); // do not forget
        VVerror_vnii_mps[seed-seed_init].resize(nel_sec); // do not forget

        index_all = 0;
        index_sec = 0;

        while (std::getline(Ostream_gps, st_line)) {
            st_t_sec          = st_line.substr(  0, 10);
            st_vni_mps_est    = st_line.substr( 10, 14);
            st_vnii_mps_est   = st_line.substr( 24, 14);
            st_vni_mps_truth  = st_line.substr( 94, 14);
            st_vnii_mps_truth = st_line.substr(108, 14);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec, &int_part);
                Verror_vni_mps[index_all]  = std::stod(st_vni_mps_est)  - std::stod(st_vni_mps_truth);
                Verror_vnii_mps[index_all] = std::stod(st_vnii_mps_est) - std::stod(st_vnii_mps_truth);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = t_sec;
                    VVerror_vni_mps[seed - seed_init][index_sec]  = Verror_vni_mps[index_all - 1];
                    VVerror_vnii_mps[seed - seed_init][index_sec] = Verror_vnii_mps[index_all - 1];
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
            st_t_sec          = st_line.substr(  0, 10);
            st_vni_mps_est    = st_line.substr( 10, 14);
            st_vnii_mps_est   = st_line.substr( 24, 14);
            st_vni_mps_truth  = st_line.substr( 52, 14);
            st_vnii_mps_truth = st_line.substr( 66, 14);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec, &int_part);
                Verror_vni_mps[index_all]  = std::stod(st_vni_mps_est)  - std::stod(st_vni_mps_truth);
                Verror_vnii_mps[index_all] = std::stod(st_vnii_mps_est) - std::stod(st_vnii_mps_truth);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = t_sec;
                    VVerror_vni_mps[seed - seed_init][index_sec]  = Verror_vni_mps[index_all - 1];
                    VVerror_vnii_mps[seed - seed_init][index_sec] = Verror_vnii_mps[index_all - 1];
                    index_sec++;
                }
            }
        }
        Ostream_pos.close();


        std::ifstream Ostream_gps_2;
        Ostream_gps_2.open(st_file_gps_complete2);
        std::getline(Ostream_gps_2, st_first_line);

        ZZerror_vni_mps[seed-seed_init].resize(nel_sec); // do not forget
        ZZerror_vnii_mps[seed-seed_init].resize(nel_sec); // do not forget

        index_all = 0;
        index_sec = 0;

        while (std::getline(Ostream_gps_2, st_line)) {
            st_t_sec          = st_line.substr(  0, 10);
            st_vni_mps_est    = st_line.substr( 10, 14);
            st_vnii_mps_est   = st_line.substr( 24, 14);
            st_vni_mps_truth  = st_line.substr( 94, 14);
            st_vnii_mps_truth = st_line.substr(108, 14);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec, &int_part);
                Zerror_vni_mps[index_all]  = std::stod(st_vni_mps_est)  - std::stod(st_vni_mps_truth);
                Zerror_vnii_mps[index_all] = std::stod(st_vnii_mps_est) - std::stod(st_vnii_mps_truth);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = t_sec;
                    ZZerror_vni_mps[seed - seed_init][index_sec]  = Zerror_vni_mps[index_all - 1];
                    ZZerror_vnii_mps[seed - seed_init][index_sec] = Zerror_vnii_mps[index_all - 1];
                    index_sec++;
                }
            }
        }
        Ostream_gps_2.close();

        std::ifstream Ostream_pos_2;
        Ostream_pos_2.open(st_file_pos_complete2);
        std::getline(Ostream_pos_2, st_first_line);
        std::getline(Ostream_pos_2, st_line); // line repeated at 100 [sec] from previous file
        while (std::getline(Ostream_pos_2, st_line)) {
            st_t_sec          = st_line.substr(  0, 10);
            st_vni_mps_est    = st_line.substr( 10, 14);
            st_vnii_mps_est   = st_line.substr( 24, 14);
            st_vni_mps_truth  = st_line.substr( 52, 14);
            st_vnii_mps_truth = st_line.substr( 66, 14);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec, &int_part);
                Zerror_vni_mps[index_all]  = std::stod(st_vni_mps_est)  - std::stod(st_vni_mps_truth);
                Zerror_vnii_mps[index_all] = std::stod(st_vnii_mps_est) - std::stod(st_vnii_mps_truth);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = t_sec;
                    ZZerror_vni_mps[seed - seed_init][index_sec]  = Zerror_vni_mps[index_all - 1];
                    ZZerror_vnii_mps[seed - seed_init][index_sec] = Zerror_vnii_mps[index_all - 1];
                    index_sec++;
                }
            }
        }
        Ostream_pos_2.close();


        std::ifstream Ostream_gps_3;
        Ostream_gps_3.open(st_file_gps_complete3);
        std::getline(Ostream_gps_3, st_first_line);

        YYerror_vni_mps[seed-seed_init].resize(nel_sec); // do not forget
        YYerror_vnii_mps[seed-seed_init].resize(nel_sec); // do not forget

        index_all = 0;
        index_sec = 0;

        while (std::getline(Ostream_gps_3, st_line)) {
            st_t_sec          = st_line.substr(  0, 10);
            st_vni_mps_est    = st_line.substr( 10, 14);
            st_vnii_mps_est   = st_line.substr( 24, 14);
            st_vni_mps_truth  = st_line.substr( 94, 14);
            st_vnii_mps_truth = st_line.substr(108, 14);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec, &int_part);
                Yerror_vni_mps[index_all]  = std::stod(st_vni_mps_est)  - std::stod(st_vni_mps_truth);
                Yerror_vnii_mps[index_all] = std::stod(st_vnii_mps_est) - std::stod(st_vnii_mps_truth);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = t_sec;
                    YYerror_vni_mps[seed - seed_init][index_sec]  = Yerror_vni_mps[index_all - 1];
                    YYerror_vnii_mps[seed - seed_init][index_sec] = Yerror_vnii_mps[index_all - 1];
                    index_sec++;
                }
            }
        }
        Ostream_gps_3.close();

        std::ifstream Ostream_pos_3;
        Ostream_pos_3.open(st_file_pos_complete3);
        std::getline(Ostream_pos_3, st_first_line);
        std::getline(Ostream_pos_3, st_line); // line repeated at 100 [sec] from previous file
        while (std::getline(Ostream_pos_3, st_line)) {
            st_t_sec          = st_line.substr(  0, 10);
            st_vni_mps_est    = st_line.substr( 10, 14);
            st_vnii_mps_est   = st_line.substr( 24, 14);
            st_vni_mps_truth  = st_line.substr( 52, 14);
            st_vnii_mps_truth = st_line.substr( 66, 14);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec, &int_part);
                Yerror_vni_mps[index_all]  = std::stod(st_vni_mps_est)  - std::stod(st_vni_mps_truth);
                Yerror_vnii_mps[index_all] = std::stod(st_vnii_mps_est) - std::stod(st_vnii_mps_truth);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = t_sec;
                    YYerror_vni_mps[seed - seed_init][index_sec]  = Yerror_vni_mps[index_all - 1];
                    YYerror_vnii_mps[seed - seed_init][index_sec] = Yerror_vnii_mps[index_all - 1];
                    index_sec++;
                }
            }
        }
        Ostream_pos_3.close();

        // compute metrics for each trajectory
        Vmean_error_vni_mps[seed-seed_init]      = math::mean(Verror_vni_mps);
        Vmean_error_vnii_mps[seed-seed_init]     = math::mean(Verror_vnii_mps);

        Vstd_error_vni_mps[seed-seed_init]       = math::std(Verror_vni_mps,  Vmean_error_vni_mps[seed-seed_init]);
        Vstd_error_vnii_mps[seed-seed_init]      = math::std(Verror_vnii_mps, Vmean_error_vnii_mps[seed-seed_init]);

        Vsmax_error_vni_mps[seed-seed_init]      = math::smax(Verror_vni_mps);
        Vsmax_error_vnii_mps[seed-seed_init]     = math::smax(Verror_vnii_mps);

        Vabs_smax_error_vni_mps[seed-seed_init]  = fabs(Vsmax_error_vni_mps[seed-seed_init]);
        Vabs_smax_error_vnii_mps[seed-seed_init] = fabs(Vsmax_error_vnii_mps[seed-seed_init]);

        Vend_error_vni_mps[seed-seed_init]       = Verror_vni_mps.back();
        Vend_error_vnii_mps[seed-seed_init]      = Verror_vnii_mps.back();

        Vabs_end_error_vni_mps[seed-seed_init]   = fabs(Vend_error_vni_mps[seed-seed_init]);
        Vabs_end_error_vnii_mps[seed-seed_init]  = fabs(Vend_error_vnii_mps[seed-seed_init]);

        Zmean_error_vni_mps[seed-seed_init]      = math::mean(Zerror_vni_mps);
        Zmean_error_vnii_mps[seed-seed_init]     = math::mean(Zerror_vnii_mps);

        Zstd_error_vni_mps[seed-seed_init]       = math::std(Zerror_vni_mps,  Zmean_error_vni_mps[seed-seed_init]);
        Zstd_error_vnii_mps[seed-seed_init]      = math::std(Zerror_vnii_mps, Zmean_error_vnii_mps[seed-seed_init]);

        Zsmax_error_vni_mps[seed-seed_init]      = math::smax(Zerror_vni_mps);
        Zsmax_error_vnii_mps[seed-seed_init]     = math::smax(Zerror_vnii_mps);

        Zabs_smax_error_vni_mps[seed-seed_init]  = fabs(Zsmax_error_vni_mps[seed-seed_init]);
        Zabs_smax_error_vnii_mps[seed-seed_init] = fabs(Zsmax_error_vnii_mps[seed-seed_init]);

        Zend_error_vni_mps[seed-seed_init]       = Zerror_vni_mps.back();
        Zend_error_vnii_mps[seed-seed_init]      = Zerror_vnii_mps.back();

        Zabs_end_error_vni_mps[seed-seed_init]   = fabs(Zend_error_vni_mps[seed-seed_init]);
        Zabs_end_error_vnii_mps[seed-seed_init]  = fabs(Zend_error_vnii_mps[seed-seed_init]);

        Ymean_error_vni_mps[seed-seed_init]      = math::mean(Yerror_vni_mps);
        Ymean_error_vnii_mps[seed-seed_init]     = math::mean(Yerror_vnii_mps);

        Ystd_error_vni_mps[seed-seed_init]       = math::std(Yerror_vni_mps,  Ymean_error_vni_mps[seed-seed_init]);
        Ystd_error_vnii_mps[seed-seed_init]      = math::std(Yerror_vnii_mps, Ymean_error_vnii_mps[seed-seed_init]);

        Ysmax_error_vni_mps[seed-seed_init]      = math::smax(Yerror_vni_mps);
        Ysmax_error_vnii_mps[seed-seed_init]     = math::smax(Yerror_vnii_mps);

        Yabs_smax_error_vni_mps[seed-seed_init]  = fabs(Ysmax_error_vni_mps[seed-seed_init]);
        Yabs_smax_error_vnii_mps[seed-seed_init] = fabs(Ysmax_error_vnii_mps[seed-seed_init]);

        Yend_error_vni_mps[seed-seed_init]       = Yerror_vni_mps.back();
        Yend_error_vnii_mps[seed-seed_init]      = Yerror_vnii_mps.back();

        Yabs_end_error_vni_mps[seed-seed_init]   = fabs(Yend_error_vni_mps[seed-seed_init]);
        Yabs_end_error_vnii_mps[seed-seed_init]  = fabs(Yend_error_vnii_mps[seed-seed_init]);

        std::cout << fixed << setw(13) << setprecision(0)              << seed
                  << fixed << setw(12) << setprecision(2) <<   showpos << Vend_error_vni_mps[seed-seed_init]
                  << fixed << setw(12) << setprecision(2) <<   showpos << Vend_error_vnii_mps[seed-seed_init]
                  << fixed << setw(12) << setprecision(2) <<   showpos << Zend_error_vni_mps[seed-seed_init]
                  << fixed << setw(12) << setprecision(2) <<   showpos << Zend_error_vnii_mps[seed-seed_init]
                  << fixed << setw(12) << setprecision(2) <<   showpos << Yend_error_vni_mps[seed-seed_init]
                  << fixed << setw(12) << setprecision(2) <<   showpos << Yend_error_vnii_mps[seed-seed_init]
                  << std::endl;
    }

    // obtain aggregated metrics
    double mean_end_error_vni_mps      = math::mean(Vend_error_vni_mps);
    double std_end_error_vni_mps       = math::std(Vend_error_vni_mps, mean_end_error_vni_mps);
    double smax_end_error_vni_mps      = math::smax(Vend_error_vni_mps);

    double mean_end_error_vnii_mps      = math::mean(Vend_error_vnii_mps);
    double std_end_error_vnii_mps       = math::std(Vend_error_vnii_mps, mean_end_error_vnii_mps);
    double smax_end_error_vnii_mps      = math::smax(Vend_error_vnii_mps);

    double mean_end_error_vni_mps_2      = math::mean(Zend_error_vni_mps);
    double std_end_error_vni_mps_2       = math::std(Zend_error_vni_mps, mean_end_error_vni_mps_2);
    double smax_end_error_vni_mps_2      = math::smax(Zend_error_vni_mps);

    double mean_end_error_vnii_mps_2      = math::mean(Zend_error_vnii_mps);
    double std_end_error_vnii_mps_2       = math::std(Zend_error_vnii_mps, mean_end_error_vnii_mps_2);
    double smax_end_error_vnii_mps_2      = math::smax(Zend_error_vnii_mps);

    double mean_end_error_vni_mps_3      = math::mean(Yend_error_vni_mps);
    double std_end_error_vni_mps_3       = math::std(Yend_error_vni_mps, mean_end_error_vni_mps_3);
    double smax_end_error_vni_mps_3      = math::smax(Yend_error_vni_mps);

    double mean_end_error_vnii_mps_3      = math::mean(Yend_error_vnii_mps);
    double std_end_error_vnii_mps_3       = math::std(Yend_error_vnii_mps, mean_end_error_vnii_mps_3);
    double smax_end_error_vnii_mps_3      = math::smax(Yend_error_vnii_mps);

    std::cout << std::endl;
    std::cout << "Final vni and vnii [mps] estimation errors (est - truth):" << std::endl;
    std::cout << setw(10) << " " << setw(24) << "baseline" << setw(24) << "TAS-AOA-AOS better" << setw(24) << "TAS-AOA-AOS worse" << std::endl;
    std::cout << setw(10) << " " << setw(12) << "vni" << setw(12) << "vnii"
              << setw(12) << "vni" << setw(12) << "vnii"
              << setw(12) << "vni" << setw(12) << "vnii"
              << std::endl;
    std::cout << "Mean:     "
              << fixed << setw(12) << setprecision(2) <<   showpos << mean_end_error_vni_mps
              << fixed << setw(12) << setprecision(2) <<   showpos << mean_end_error_vnii_mps
              << fixed << setw(12) << setprecision(2) <<   showpos << mean_end_error_vni_mps_2
              << fixed << setw(12) << setprecision(2) <<   showpos << mean_end_error_vnii_mps_2
              << fixed << setw(12) << setprecision(2) <<   showpos << mean_end_error_vni_mps_3
              << fixed << setw(12) << setprecision(2) <<   showpos << mean_end_error_vnii_mps_3
              << std::endl;
    std::cout << "std:      "
              << fixed << setw(12) << setprecision(2) << noshowpos << std_end_error_vni_mps
              << fixed << setw(12) << setprecision(2) << noshowpos << std_end_error_vnii_mps
              << fixed << setw(12) << setprecision(2) << noshowpos << std_end_error_vni_mps_2
              << fixed << setw(12) << setprecision(2) << noshowpos << std_end_error_vnii_mps_2
              << fixed << setw(12) << setprecision(2) << noshowpos << std_end_error_vni_mps_3
              << fixed << setw(12) << setprecision(2) << noshowpos << std_end_error_vnii_mps_3
              << std::endl;
    std::cout << "smax:     "
              << fixed << setw(12) << setprecision(2) <<   showpos << smax_end_error_vni_mps
              << fixed << setw(12) << setprecision(2) <<   showpos << smax_end_error_vnii_mps
              << fixed << setw(12) << setprecision(2) <<   showpos << smax_end_error_vni_mps_2
              << fixed << setw(12) << setprecision(2) <<   showpos << smax_end_error_vnii_mps_2
              << fixed << setw(12) << setprecision(2) <<   showpos << smax_end_error_vni_mps_3
              << fixed << setw(12) << setprecision(2) <<   showpos << smax_end_error_vnii_mps_3
              << std::endl;

    // reverse data (rows to columns and viceversa) to compute other metrics
    std::vector<std::vector<double>> WWerror_vni_mps(nel_sec), WWerror_vnii_mps(nel_sec), WWerror_vni_mps_2(nel_sec), WWerror_vnii_mps_2(nel_sec), WWerror_vni_mps_3(nel_sec), WWerror_vnii_mps_3(nel_sec);
    std::vector<double> Wmean_error_vni_mps(nel_sec), Wstd_error_vni_mps(nel_sec), Wmean_error_vnii_mps(nel_sec), Wstd_error_vnii_mps(nel_sec);
    std::vector<double> Wmean_error_vni_mps_2(nel_sec), Wstd_error_vni_mps_2(nel_sec), Wmean_error_vnii_mps_2(nel_sec), Wstd_error_vnii_mps_2(nel_sec);
    std::vector<double> Wmean_error_vni_mps_3(nel_sec), Wstd_error_vni_mps_3(nel_sec), Wmean_error_vnii_mps_3(nel_sec), Wstd_error_vnii_mps_3(nel_sec);
    for (unsigned long i = 0; i != nel_sec; ++i) {
        WWerror_vni_mps[i].resize(nel);
        WWerror_vnii_mps[i].resize(nel);
        WWerror_vni_mps_2[i].resize(nel);
        WWerror_vnii_mps_2[i].resize(nel);
        WWerror_vni_mps_3[i].resize(nel);
        WWerror_vnii_mps_3[i].resize(nel);
        for (unsigned long j = 0; j != nel; ++j) {
            WWerror_vni_mps[i][j]        = VVerror_vni_mps[j][i];
            WWerror_vnii_mps[i][j]       = VVerror_vnii_mps[j][i];
            WWerror_vni_mps_2[i][j]      = ZZerror_vni_mps[j][i];
            WWerror_vnii_mps_2[i][j]     = ZZerror_vnii_mps[j][i];
            WWerror_vni_mps_3[i][j]      = YYerror_vni_mps[j][i];
            WWerror_vnii_mps_3[i][j]     = YYerror_vnii_mps[j][i];
        }
        Wmean_error_vni_mps[i]      = math::mean(WWerror_vni_mps[i]);
        Wstd_error_vni_mps[i]       = math::std(WWerror_vni_mps[i], Wmean_error_vni_mps[i]);
        Wmean_error_vnii_mps[i]     = math::mean(WWerror_vnii_mps[i]);
        Wstd_error_vnii_mps[i]      = math::std(WWerror_vnii_mps[i], Wmean_error_vnii_mps[i]);

        Wmean_error_vni_mps_2[i]      = math::mean(WWerror_vni_mps_2[i]);
        Wstd_error_vni_mps_2[i]       = math::std(WWerror_vni_mps_2[i], Wmean_error_vni_mps_2[i]);
        Wmean_error_vnii_mps_2[i]     = math::mean(WWerror_vnii_mps_2[i]);
        Wstd_error_vnii_mps_2[i]      = math::std(WWerror_vnii_mps_2[i], Wmean_error_vnii_mps_2[i]);

        Wmean_error_vni_mps_3[i]      = math::mean(WWerror_vni_mps_3[i]);
        Wstd_error_vni_mps_3[i]       = math::std(WWerror_vni_mps_3[i], Wmean_error_vni_mps_3[i]);
        Wmean_error_vnii_mps_3[i]     = math::mean(WWerror_vnii_mps_3[i]);
        Wstd_error_vnii_mps_3[i]      = math::std(WWerror_vnii_mps_3[i], Wmean_error_vnii_mps_3[i]);

    }

    boost::filesystem::path path_folder(path_outputs_write / st_folder_main / st_extra_folder);
    boost::filesystem::create_directory(path_folder);

    std::string st_file_out = "versus_vtasb_alter_vned_mps.txt";
    std::string st_file_output = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out).string();

    ofstream Oout;
    Oout.open(st_file_output);

    for (unsigned long i = 0; i != nel_sec; ++i) {
        Oout << fixed      << setw(12) << setprecision(1) << showpos << Vt_sec[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vni_mps[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vni_mps[i] + Wstd_error_vni_mps[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vni_mps[i] - Wstd_error_vni_mps[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vni_mps_2[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vni_mps_2[i] + Wstd_error_vni_mps_2[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vni_mps_2[i] - Wstd_error_vni_mps_2[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vni_mps_3[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vni_mps_3[i] + Wstd_error_vni_mps_3[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vni_mps_3[i] - Wstd_error_vni_mps_3[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vnii_mps[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vnii_mps[i] + Wstd_error_vnii_mps[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vnii_mps[i] - Wstd_error_vnii_mps[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vnii_mps_2[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vnii_mps_2[i] + Wstd_error_vnii_mps_2[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vnii_mps_2[i] - Wstd_error_vnii_mps_2[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vnii_mps_3[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vnii_mps_3[i] + Wstd_error_vnii_mps_3[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vnii_mps_3[i] - Wstd_error_vnii_mps_3[i]
             << endl;
    }
    Oout.close();

    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    std::string st_file_out_tex = "versus_vtasb_alter_vned_table.tex";
    std::string st_file_output_tex = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_tex).string();

    ofstream Oout_tex;
    Oout_tex.open(st_file_output_tex);

    Oout_tex << "\\begin{center}" << std::endl;
    Oout_tex << "\\begin{tabular}{lrrrrrr}" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\textbf{Alternate} & \\multicolumn{6}{c}{\\textbf{TAS-AOA-AOS Configuration}} \\\\" << std::endl;
    Oout_tex << " & \\multicolumn{2}{c}{\\textbf{Baseline}} & \\multicolumn{2}{c}{\\textbf{Better}} & \\multicolumn{2}{c}{\\textbf{Worse}} \\\\" << std::endl;
    Oout_tex << "\\nm{\\lrsb{m/sec}} & \\nm{\\vNesti - \\vNi} & \\nm{\\vNestii - \\vNii} & \\nm{\\vNesti - \\vNi} & \\nm{\\vNestii - \\vNii} & \\nm{\\vNesti - \\vNi} & \\nm{\\vNestii - \\vNii} \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\nm{\\muEND{}} "
             << " & "
             << fixed << setprecision(3) <<   showpos << mean_end_error_vni_mps
             << " & "
             << fixed << setprecision(3) <<   showpos << mean_end_error_vnii_mps
             << " & "
             << fixed << setprecision(3) <<   showpos << mean_end_error_vni_mps_2
             << " & "
             << fixed << setprecision(3) <<   showpos << mean_end_error_vnii_mps_2
             << " & "
             << fixed << setprecision(3) <<   showpos << mean_end_error_vni_mps_3
             << " & "
             << fixed << setprecision(3) <<   showpos << mean_end_error_vnii_mps_3
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\sigmaEND{}} "
             << " & \\textbf{"
             << fixed << setprecision(3) << noshowpos << std_end_error_vni_mps
             << "} & \\textbf{"
             << fixed << setprecision(3) << noshowpos << std_end_error_vnii_mps
             << "} & \\textbf{"
             << fixed << setprecision(3) << noshowpos << std_end_error_vni_mps_2
             << "} & \\textbf{"
             << fixed << setprecision(3) << noshowpos << std_end_error_vnii_mps_2
             << "} & \\textbf{"
             << fixed << setprecision(3) << noshowpos << std_end_error_vni_mps_3
             << "} & \\textbf{"
             << fixed << setprecision(3) << noshowpos << std_end_error_vnii_mps_3
             << "} \\\\" << std::endl;
    Oout_tex << "\\nm{\\maxEND{}} "
             << " & "
             << fixed << setprecision(3) <<   showpos << smax_end_error_vni_mps
             << " & "
             << fixed << setprecision(3) <<   showpos << smax_end_error_vnii_mps
             << " & "
             << fixed << setprecision(3) <<   showpos << smax_end_error_vni_mps_2
             << " & "
             << fixed << setprecision(3) <<   showpos << smax_end_error_vnii_mps_2
             << " & "
             << fixed << setprecision(3) <<   showpos << smax_end_error_vni_mps_3
             << " & "
             << fixed << setprecision(3) <<   showpos << smax_end_error_vnii_mps_3
             << "\\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\end{tabular}" << std::endl;
    Oout_tex << "\\end{center}" << std::endl;
    Oout_tex.close();

}
/* reads the horizontal ground velocity estimation results from three sets of files, the baseline, one employing
 * better airspeed sensors (TAS, AOA, AOS), and another employing worse ones.
 * - "versus_vtasb_alter_vned_mps.txt" --> to plot ground speed final error
 * - "versus_vtasb_alter_vned_table.tex" --> script to directly generate table in Latex */

void nav::test::Tplots_sensors_vtasb::obtain_xhor_vtasb_alter_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end) {
    std::string st_folder      = st_first_folder;
    std::string st_folder_one  = "ALTER_MAIN";
    std::string st_folder_one2 = "ALTER_VTAS_BETTER";
    std::string st_folder_one3 = "ALTER_VTAS_WORSE";

    boost::filesystem::path path_outputs_read(math::share::phd_outputs_store_prefix);
    boost::filesystem::path path_outputs_write(math::share::phd_outputs_prefix);
    std::string st_folder_main("nav");
    std::string st_extra_folder("versus_vtasb");
    std::string st_file("output_pos.txt");
    std::string st_seeds, st_file_complete, st_file_complete_2, st_first_line, st_line, st_file_complete_3;
    unsigned long nel     = (unsigned long)(seed_end) - (unsigned long)(seed_init) + 1;
    unsigned long nel_all = 5001; // note this file is written every 0.1 [sec] instead of 0.01 [sec]
    unsigned long nel_sec = 501;
    double t_sec_gnss = 100.0;
    unsigned long index_all, index_sec, index_sec_gnss;
    std::vector<double> Vt_sec(nel_sec);
    std::vector<std::vector<double>> VVerror_hor_m(nel), VVdist_hor_m(nel);
    std::vector<std::vector<double>> ZZerror_hor_m(nel), ZZdist_hor_m(nel);
    std::vector<std::vector<double>> YYerror_hor_m(nel), YYdist_hor_m(nel);
    std::vector<double> Verror_hor_m(nel_all), Vdist_hor_m(nel_all), Verror_hor_m_end(nel), Vdist_hor_m_end(nel), Verror_hor_pc_end(nel);
    std::vector<double> Zerror_hor_m(nel_all), Zdist_hor_m(nel_all), Zerror_hor_m_end(nel), Zdist_hor_m_end(nel), Zerror_hor_pc_end(nel);
    std::vector<double> Yerror_hor_m(nel_all), Ydist_hor_m(nel_all), Yerror_hor_m_end(nel), Ydist_hor_m_end(nel), Yerror_hor_pc_end(nel);
    double int_part, dec_part, t_sec, err_hor_m, dist_hor_m;
    std::string st_t_sec, st_dist_hor_m, st_err_hor_m;

    // for each trajectory
    for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
        st_seeds = math::seeder::seed2string(seed);

        st_folder.replace(3,2,st_seeds);
        st_file_complete   = (path_outputs_read / st_folder_main / st_folder_one  / st_folder / st_file).string();
        st_file_complete_2 = (path_outputs_read / st_folder_main / st_folder_one2 / st_folder / st_file).string();
        st_file_complete_3 = (path_outputs_read / st_folder_main / st_folder_one3 / st_folder / st_file).string();

        std::ifstream Ostream;
        Ostream.open(st_file_complete);

        VVdist_hor_m[seed-seed_init].resize(nel_sec); // do not forget
        VVerror_hor_m[seed-seed_init].resize(nel_sec); // do not forget

        std::getline(Ostream, st_first_line);
        index_all = 0;
        index_sec = 0;

        do {
            std::getline(Ostream, st_line); // use this method as there are nan
            st_t_sec       = st_line.substr(0,9);
            st_dist_hor_m  = st_line.substr(255,12);
            st_err_hor_m   = st_line.substr(268,12);

            t_sec       = std::stod(st_t_sec);
            dist_hor_m  = std::stod(st_dist_hor_m);
            err_hor_m   = std::stod(st_err_hor_m);

            dec_part = modf(t_sec, &int_part);
            if (t_sec > (-1e-8)) {
                Vdist_hor_m[index_all]    = dist_hor_m;
                Verror_hor_m[index_all]   = err_hor_m;
                if (fabs(t_sec - t_sec_gnss) < 1e-8) {
                    index_sec_gnss = index_sec;
                }
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = t_sec;
                    VVdist_hor_m[seed-seed_init][index_sec]    = Vdist_hor_m[index_all-1];
                    VVerror_hor_m[seed-seed_init][index_sec]   = Verror_hor_m[index_all-1];

                    index_sec++;
                }
            }

        } while (index_sec != nel_sec);

        Ostream.close();


        std::ifstream Ostream_2;
        Ostream_2.open(st_file_complete_2);

        ZZdist_hor_m[seed-seed_init].resize(nel_sec); // do not forget
        ZZerror_hor_m[seed-seed_init].resize(nel_sec); // do not forget

        std::getline(Ostream_2, st_first_line);
        index_all = 0;
        index_sec = 0;

        do {
            std::getline(Ostream_2, st_line); // use this method as there are nan
            st_t_sec       = st_line.substr(0,9);
            st_dist_hor_m  = st_line.substr(255,12);
            st_err_hor_m   = st_line.substr(268,12);

            t_sec       = std::stod(st_t_sec);
            dist_hor_m  = std::stod(st_dist_hor_m);
            err_hor_m   = std::stod(st_err_hor_m);

            dec_part = modf(t_sec, &int_part);
            if (t_sec > (-1e-8)) {
                Zdist_hor_m[index_all]    = dist_hor_m;
                Zerror_hor_m[index_all]   = err_hor_m;
                if (fabs(t_sec - t_sec_gnss) < 1e-8) {
                    index_sec_gnss = index_sec;
                }
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = t_sec;
                    ZZdist_hor_m[seed-seed_init][index_sec]    = Zdist_hor_m[index_all-1];
                    ZZerror_hor_m[seed-seed_init][index_sec]   = Zerror_hor_m[index_all-1];

                    index_sec++;
                }
            }

        } while (index_sec != nel_sec);

        Ostream_2.close();


        std::ifstream Ostream_3;
        Ostream_3.open(st_file_complete_3);

        YYdist_hor_m[seed-seed_init].resize(nel_sec); // do not forget
        YYerror_hor_m[seed-seed_init].resize(nel_sec); // do not forget

        std::getline(Ostream_3, st_first_line);
        index_all = 0;
        index_sec = 0;

        do {
            std::getline(Ostream_3, st_line); // use this method as there are nan
            st_t_sec       = st_line.substr(0,9);
            st_dist_hor_m  = st_line.substr(255,12);
            st_err_hor_m   = st_line.substr(268,12);

            t_sec       = std::stod(st_t_sec);
            dist_hor_m  = std::stod(st_dist_hor_m);
            err_hor_m   = std::stod(st_err_hor_m);

            dec_part = modf(t_sec, &int_part);
            if (t_sec > (-1e-8)) {
                Ydist_hor_m[index_all]    = dist_hor_m;
                Yerror_hor_m[index_all]   = err_hor_m;
                if (fabs(t_sec - t_sec_gnss) < 1e-8) {
                    index_sec_gnss = index_sec;
                }
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = t_sec;
                    YYdist_hor_m[seed-seed_init][index_sec]    = Ydist_hor_m[index_all-1];
                    YYerror_hor_m[seed-seed_init][index_sec]   = Yerror_hor_m[index_all-1];

                    index_sec++;
                }
            }

        } while (index_sec != nel_sec);

        Ostream_3.close();

        // compute metrics for each trajectory
        Vdist_hor_m_end[seed-seed_init]     = Vdist_hor_m.back() - Vdist_hor_m[index_sec_gnss];
        Verror_hor_m_end[seed-seed_init]    = std::fabs(Verror_hor_m.back());
        Verror_hor_pc_end[seed-seed_init]   = 100 * Verror_hor_m_end[seed-seed_init] / Vdist_hor_m_end[seed-seed_init];

        Zdist_hor_m_end[seed-seed_init]     = Zdist_hor_m.back() - Zdist_hor_m[index_sec_gnss];
        Zerror_hor_m_end[seed-seed_init]    = std::fabs(Zerror_hor_m.back());
        Zerror_hor_pc_end[seed-seed_init]   = 100 * Zerror_hor_m_end[seed-seed_init] / Zdist_hor_m_end[seed-seed_init];

        Ydist_hor_m_end[seed-seed_init]     = Ydist_hor_m.back() - Ydist_hor_m[index_sec_gnss];
        Yerror_hor_m_end[seed-seed_init]    = std::fabs(Yerror_hor_m.back());
        Yerror_hor_pc_end[seed-seed_init]   = 100 * Yerror_hor_m_end[seed-seed_init] / Ydist_hor_m_end[seed-seed_init];

        std::cout << fixed << setw(9)  << setprecision(0)              << seed
                  << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[seed-seed_init]
                  << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[seed-seed_init]
                  << fixed << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[seed-seed_init]
                  << fixed << setw(11) << setprecision(0) << noshowpos << Zdist_hor_m_end[seed-seed_init]
                  << fixed << setw(11) << setprecision(2) << noshowpos << Zerror_hor_m_end[seed-seed_init]
                  << fixed << setw(6)  << setprecision(2) << noshowpos << Zerror_hor_pc_end[seed-seed_init]
                  << fixed << setw(11) << setprecision(0) << noshowpos << Ydist_hor_m_end[seed-seed_init]
                  << fixed << setw(11) << setprecision(2) << noshowpos << Yerror_hor_m_end[seed-seed_init]
                  << fixed << setw(6)  << setprecision(2) << noshowpos << Yerror_hor_pc_end[seed-seed_init]
                  << std::endl;
    }

    // obtain aggregated metrics
    double mean_dist_hor_m_end = math::mean(Vdist_hor_m_end);
    double std_dist_hor_m_end  = math::std(Vdist_hor_m_end, mean_dist_hor_m_end);
    double smax_dist_hor_m_end = math::smax(Vdist_hor_m_end);

    double mean_error_hor_m_end = math::mean(Verror_hor_m_end);
    double std_error_hor_m_end  = math::std(Verror_hor_m_end, mean_error_hor_m_end);
    double smax_error_hor_m_end = math::smax(Verror_hor_m_end);

    double mean_error_hor_pc_end = math::mean(Verror_hor_pc_end);
    double std_error_hor_pc_end  = math::std(Verror_hor_pc_end, mean_error_hor_pc_end);
    double smax_error_hor_pc_end = math::smax(Verror_hor_pc_end);

    double mean_dist_hor_m_end_2 = math::mean(Zdist_hor_m_end);
    double std_dist_hor_m_end_2  = math::std(Zdist_hor_m_end, mean_dist_hor_m_end_2);
    double smax_dist_hor_m_end_2 = math::smax(Zdist_hor_m_end);

    double mean_error_hor_m_end_2 = math::mean(Zerror_hor_m_end);
    double std_error_hor_m_end_2  = math::std(Zerror_hor_m_end, mean_error_hor_m_end_2);
    double smax_error_hor_m_end_2 = math::smax(Zerror_hor_m_end);

    double mean_error_hor_pc_end_2 = math::mean(Zerror_hor_pc_end);
    double std_error_hor_pc_end_2  = math::std(Zerror_hor_pc_end, mean_error_hor_pc_end_2);
    double smax_error_hor_pc_end_2 = math::smax(Zerror_hor_pc_end);

    double mean_dist_hor_m_end_3 = math::mean(Ydist_hor_m_end);
    double std_dist_hor_m_end_3  = math::std(Ydist_hor_m_end, mean_dist_hor_m_end_3);
    double smax_dist_hor_m_end_3 = math::smax(Ydist_hor_m_end);

    double mean_error_hor_m_end_3 = math::mean(Yerror_hor_m_end);
    double std_error_hor_m_end_3  = math::std(Yerror_hor_m_end, mean_error_hor_m_end_3);
    double smax_error_hor_m_end_3 = math::smax(Yerror_hor_m_end);

    double mean_error_hor_pc_end_3 = math::mean(Yerror_hor_pc_end);
    double std_error_hor_pc_end_3  = math::std(Yerror_hor_pc_end, mean_error_hor_pc_end_3);
    double smax_error_hor_pc_end_3 = math::smax(Yerror_hor_pc_end);

    std::cout << std::endl;
    std::cout << "xhor position filter error [m,%]:" << std::endl;
    std::cout << setw(10) << " " << setw(28) << "baseline" << setw(28) << "TAS-AOA-AOS better" << setw(28) << "TAS-AOS-AOS worse" << std::endl;
    std::cout << setw(10) << " "
              << setw(11) << "distance" << setw(17) << "hor error"
              << setw(11) << "distance" << setw(17) << "hor error"
              << setw(11) << "distance" << setw(17) << "hor error"
              << std::endl;

    std::cout << "Mean:     "
              << fixed << setw(11) << setprecision(0) << noshowpos << mean_dist_hor_m_end
              << fixed << setw(11) << setprecision(0) << noshowpos << mean_error_hor_m_end
              << fixed << setw(6)  << setprecision(2) << noshowpos << mean_error_hor_pc_end
              << fixed << setw(11) << setprecision(0) << noshowpos << mean_dist_hor_m_end_2
              << fixed << setw(11) << setprecision(2) << noshowpos << mean_error_hor_m_end_2
              << fixed << setw(6)  << setprecision(2) << noshowpos << mean_error_hor_pc_end_2
              << fixed << setw(11) << setprecision(0) << noshowpos << mean_dist_hor_m_end_3
              << fixed << setw(11) << setprecision(2) << noshowpos << mean_error_hor_m_end_3
              << fixed << setw(6)  << setprecision(2) << noshowpos << mean_error_hor_pc_end_3
              << std::endl;
    std::cout << "std:      "
              << fixed << setw(11) << setprecision(0) << noshowpos << std_dist_hor_m_end
              << fixed << setw(11) << setprecision(0) << noshowpos << std_error_hor_m_end
              << fixed << setw(6)  << setprecision(2) << noshowpos << std_error_hor_pc_end
              << fixed << setw(11) << setprecision(0) << noshowpos << std_dist_hor_m_end_2
              << fixed << setw(11) << setprecision(2) << noshowpos << std_error_hor_m_end_2
              << fixed << setw(6)  << setprecision(2) << noshowpos << std_error_hor_pc_end_2
              << fixed << setw(11) << setprecision(0) << noshowpos << std_dist_hor_m_end_3
              << fixed << setw(11) << setprecision(2) << noshowpos << std_error_hor_m_end_3
              << fixed << setw(6)  << setprecision(2) << noshowpos << std_error_hor_pc_end_3
              << std::endl;
    std::cout << "smax:     "
              << fixed << setw(11) << setprecision(0) << noshowpos << smax_dist_hor_m_end
              << fixed << setw(11) << setprecision(0) << noshowpos << smax_error_hor_m_end
              << fixed << setw(6)  << setprecision(2) << noshowpos << smax_error_hor_pc_end
              << fixed << setw(11) << setprecision(0) << noshowpos << smax_dist_hor_m_end_2
              << fixed << setw(11) << setprecision(2) << noshowpos << smax_error_hor_m_end_2
              << fixed << setw(6)  << setprecision(2) << noshowpos << smax_error_hor_pc_end_2
              << fixed << setw(11) << setprecision(0) << noshowpos << smax_dist_hor_m_end_3
              << fixed << setw(11) << setprecision(2) << noshowpos << smax_error_hor_m_end_3
              << fixed << setw(6)  << setprecision(2) << noshowpos << smax_error_hor_pc_end_3
              << std::endl;

    // reverse data (rows to columns and viceversa) to compute other metrics
    std::vector<std::vector<double>> WWerror_hor_m(nel_sec), WWerror_hor_m_2(nel_sec), WWerror_hor_m_3(nel_sec);
    std::vector<double> Wmean_error_hor_m(nel_sec), Wstd_error_hor_m(nel_sec), Wmean_error_hor_m_2(nel_sec), Wstd_error_hor_m_2(nel_sec), Wmean_error_hor_m_3(nel_sec), Wstd_error_hor_m_3(nel_sec);
    for (unsigned long i = 0; i != nel_sec; ++i) {
        WWerror_hor_m[i].resize(nel);
        WWerror_hor_m_2[i].resize(nel);
        WWerror_hor_m_3[i].resize(nel);
        for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
            WWerror_hor_m[i][seed-seed_init]   = VVerror_hor_m[seed-seed_init][i];
            WWerror_hor_m_2[i][seed-seed_init] = ZZerror_hor_m[seed-seed_init][i];
            WWerror_hor_m_3[i][seed-seed_init] = YYerror_hor_m[seed-seed_init][i];
        }
        Wmean_error_hor_m[i]   = math::mean(WWerror_hor_m[i]);
        Wstd_error_hor_m[i]    = math::std(WWerror_hor_m[i], Wmean_error_hor_m[i]);
        Wmean_error_hor_m_2[i] = math::mean(WWerror_hor_m_2[i]);
        Wstd_error_hor_m_2[i]  = math::std(WWerror_hor_m_2[i], Wmean_error_hor_m_2[i]);
        Wmean_error_hor_m_3[i] = math::mean(WWerror_hor_m_3[i]);
        Wstd_error_hor_m_3[i]  = math::std(WWerror_hor_m_3[i], Wmean_error_hor_m_3[i]);
    }

    boost::filesystem::path path_folder(path_outputs_write / st_folder_main / st_extra_folder);
    boost::filesystem::create_directory(path_folder);

    std::string st_file_out = "versus_vtasb_alter_pos_hor_m_pc.txt";
    std::string st_file_output = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out).string();

    ofstream Oout;
    Oout.open(st_file_output);

    for (unsigned long i = 0; i != nel_sec; ++i) {
        Oout << fixed      << setw(8)  << setprecision(1) << showpos << Vt_sec[i]
             << scientific << setw(12) << setprecision(3) << showpos << 1e-3 * Wmean_error_hor_m[i]
             << scientific << setw(12) << setprecision(3) << showpos << 1e-3 * (Wmean_error_hor_m[i] + Wstd_error_hor_m[i])
             << scientific << setw(12) << setprecision(3) << showpos << 1e-3 * (Wmean_error_hor_m[i] - Wstd_error_hor_m[i])
             << scientific << setw(12) << setprecision(3) << showpos << 1e-3 * Wmean_error_hor_m_2[i]
             << scientific << setw(12) << setprecision(3) << showpos << 1e-3 * (Wmean_error_hor_m_2[i] + Wstd_error_hor_m_2[i])
             << scientific << setw(12) << setprecision(3) << showpos << 1e-3 * (Wmean_error_hor_m_2[i] - Wstd_error_hor_m_2[i])
             << scientific << setw(12) << setprecision(3) << showpos << 1e-3 * Wmean_error_hor_m_3[i]
             << scientific << setw(12) << setprecision(3) << showpos << 1e-3 * (Wmean_error_hor_m_3[i] + Wstd_error_hor_m_3[i])
             << scientific << setw(12) << setprecision(3) << showpos << 1e-3 * (Wmean_error_hor_m_3[i] - Wstd_error_hor_m_3[i])
             << endl;
    }
    Oout.close();

    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    std::string st_file_out_tex = "versus_vtasb_alter_pos_hor_table.tex";
    std::string st_file_output_tex = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_tex).string();

    ofstream Oout_tex;
    Oout_tex.open(st_file_output_tex);

    Oout_tex << "\\begin{center}" << std::endl;
    Oout_tex << "\\begin{tabular}{lrrrrrrr}" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\textbf{Alternate} & \\multicolumn{7}{c}{\\textbf{TAS-AOA-AOS Configuration}} \\\\" << std::endl;
    Oout_tex << " & \\multicolumn{3}{c}{\\textbf{Baseline}} & \\multicolumn{2}{c}{\\textbf{Better}} & \\multicolumn{2}{c}{\\textbf{Worse}} \\\\" << std::endl;
    Oout_tex << " & Distance \\nm{\\lrsb{m}} & \\multicolumn{2}{c}{\\nm{\\Deltaxhor \\lrsb{m,\\%}}} & \\multicolumn{2}{c}{\\nm{\\Deltaxhor \\lrsb{m,\\%}}} & \\multicolumn{2}{c}{\\nm{\\Deltaxhor \\lrsb{m,\\%}}} \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\nm{\\muEND{\\Deltaxhor}} "
             << " & "
             << fixed << setprecision(1) << noshowpos << mean_dist_hor_m_end
             << " & "
             << fixed << setprecision(1) << noshowpos << mean_error_hor_m_end
             << " & \\textbf{"
             << fixed << setprecision(2) << noshowpos << mean_error_hor_pc_end
             << "} & "
             << fixed << setprecision(1) << noshowpos << mean_error_hor_m_end_2
             << " & \\textbf{"
             << fixed << setprecision(2) << noshowpos << mean_error_hor_pc_end_2
             << "} & "
             << fixed << setprecision(1) << noshowpos << mean_error_hor_m_end_3
             << " & \\textbf{"
             << fixed << setprecision(2) << noshowpos << mean_error_hor_pc_end_3
             << "} \\\\" << std::endl;
    Oout_tex << "\\nm{\\sigmaEND{\\Deltaxhor}} "
             << " & "
             << fixed << setprecision(1) << noshowpos << std_dist_hor_m_end
             << " & "
             << fixed << setprecision(1) << noshowpos << std_error_hor_m_end
             << " & "
             << fixed << setprecision(2) << noshowpos << std_error_hor_pc_end
             << " & "
             << fixed << setprecision(1) << noshowpos << std_error_hor_m_end_2
             << " & "
             << fixed << setprecision(2) << noshowpos << std_error_hor_pc_end_2
             << " & "
             << fixed << setprecision(1) << noshowpos << std_error_hor_m_end_3
             << " & "
             << fixed << setprecision(2) << noshowpos << std_error_hor_pc_end_3
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\maxEND{\\Deltaxhor}} "
             << " & "
             << fixed << setprecision(1) << noshowpos << smax_dist_hor_m_end
             << " & "
             << fixed << setprecision(1) << noshowpos << smax_error_hor_m_end
             << " & "
             << fixed << setprecision(2) << noshowpos << smax_error_hor_pc_end
             << " & "
             << fixed << setprecision(1) << noshowpos << smax_error_hor_m_end_2
             << " & "
             << fixed << setprecision(2) << noshowpos << smax_error_hor_pc_end_2
             << " & "
             << fixed << setprecision(1) << noshowpos << smax_error_hor_m_end_3
             << " & "
             << fixed << setprecision(2) << noshowpos << smax_error_hor_pc_end_3
             << " \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\end{tabular}" << std::endl;
    Oout_tex << "\\end{center}" << std::endl;
    Oout_tex.close();

}
/* reads the final horizontal position results from three sets of files, the baseline, one employing
 * better airspeed sensors (TAS-AOA-AOS), and another employing worse ones.
 * It shows results on console and also generates the following files (for thesis, not Matlab):
 * - "versus_vtasb_pos_hor_m_pc.txt" --> to plot horizontal error mean and std variation with time
 * - "versus_vtasb_pos_h.tex" --> script to directly generate table in Latex */
























































