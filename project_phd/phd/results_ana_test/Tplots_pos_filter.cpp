#include "Tplots_pos_filter.h"

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

nav::test::Tplots_pos_filter::Tplots_pos_filter(unsigned short& seed_init, unsigned short& seed_end)
: _seed_init(seed_init), _seed_end(seed_end) {
}
/* constructor based on counter */

void nav::test::Tplots_pos_filter::run() {

    obtain_xhor_metrics("01_01_05_02_03_0100", _seed_init, _seed_end);
    obtain_h_metrics   ("01_01_05_02_03_0100", _seed_init, _seed_end);

}
/* execute tests and write results on console */

using namespace std;

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void nav::test::Tplots_pos_filter::obtain_xhor_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end) {
    std::string st_folder        = st_first_folder;
    std::string st_folder_one    = "MAIN";
    std::string st_folder_one_1B = "POS_FILT_1B";
    std::string st_folder_one_3B = "POS_FILT_3B";

    boost::filesystem::path path_outputs_read(math::share::phd_outputs_store_prefix);
    boost::filesystem::path path_outputs_write(math::share::phd_outputs_prefix);

    std::string st_folder_main("nav");
    std::string st_extra_folder("versus_integr");
    std::string st_file("output_pos.txt");
    std::string st_seeds, st_file_complete, st_file_complete_1B, st_file_complete_3B, st_first_line, st_line;
    unsigned long nel     = (unsigned long)(seed_end) - (unsigned long)(seed_init) + 1;

    unsigned long nel_all_def = 38001, nel_all_new = 10001; // trajectories in 1B and 2B conclude at 1000 [sec]
    unsigned long nel_sec_def = 381, nel_sec_new = 101;
    double t_sec_gnss = 100.0;
    unsigned long index_all, index_sec, index_sec_gnss;
    std::vector<double> Vt_sec_def(nel_sec_def), Vt_sec_1B(nel_sec_new), Vt_sec_3B(nel_sec_new);
    std::vector<std::vector<double>> VVerror_hor_m(nel), VVdist_hor_m(nel);
    std::vector<std::vector<double>> YYerror_hor_m(nel), YYdist_hor_m(nel);
    std::vector<std::vector<double>> ZZerror_hor_m(nel), ZZdist_hor_m(nel);

    std::vector<double> Verror_hor_m(nel_all_def), Vdist_hor_m(nel_all_def), Verror_hor_m_end(nel), Vdist_hor_m_end(nel), Verror_hor_pc_end(nel);
    std::vector<double> Yerror_hor_m(nel_all_new), Ydist_hor_m(nel_all_new), Yerror_hor_m_end(nel), Ydist_hor_m_end(nel);
    std::vector<double> Zerror_hor_m(nel_all_new), Zdist_hor_m(nel_all_new), Zerror_hor_m_end(nel), Zdist_hor_m_end(nel);

    std::string st_t_sec, st_err_hor_m, st_dist_hor_m;
    double int_part, dec_part, t_sec, err_hor_m, dist_hor_m;

    // for each trajectory
    for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
        st_seeds = math::seeder::seed2string(seed);

        st_folder.replace(3,2,st_seeds);
        st_file_complete    = (path_outputs_read / st_folder_main / st_folder_one    / st_folder / st_file).string();
        st_file_complete_1B = (path_outputs_read / st_folder_main / st_folder_one_1B / st_folder / st_file).string();
        st_file_complete_3B = (path_outputs_read / st_folder_main / st_folder_one_3B / st_folder / st_file).string();

        std::ifstream Ostream;
        Ostream.open(st_file_complete);

        VVdist_hor_m[seed-seed_init].resize(nel_sec_def);
        VVerror_hor_m[seed-seed_init].resize(nel_sec_def);

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
                    Vt_sec_def[index_sec]                      = t_sec;
                    VVdist_hor_m[seed-seed_init][index_sec]    = Vdist_hor_m[index_all-1];
                    VVerror_hor_m[seed-seed_init][index_sec]   = Verror_hor_m[index_all-1];

                    index_sec++;
                }
            }

        } while (index_sec != nel_sec_def);

        Ostream.close();

        std::ifstream Ostream_1B;
        Ostream_1B.open(st_file_complete_1B);

        YYdist_hor_m[seed-seed_init].resize(nel_sec_new);
        YYerror_hor_m[seed-seed_init].resize(nel_sec_new);

        std::getline(Ostream_1B, st_first_line);
        index_all = 0;
        index_sec = 0;

        do {
            std::getline(Ostream_1B, st_line); // use this method as there are nan
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
                    Vt_sec_1B[index_sec]                       = t_sec;
                    YYdist_hor_m[seed-seed_init][index_sec]    = Ydist_hor_m[index_all-1];
                    YYerror_hor_m[seed-seed_init][index_sec]   = Yerror_hor_m[index_all-1];

                    index_sec++;
                }
            }

        } while (index_sec != nel_sec_new);

        Ostream_1B.close();

        std::ifstream Ostream_3B;
        Ostream_3B.open(st_file_complete_3B);

        ZZdist_hor_m[seed-seed_init].resize(nel_sec_new);
        ZZerror_hor_m[seed-seed_init].resize(nel_sec_new);

        std::getline(Ostream_3B, st_first_line);
        index_all = 0;
        index_sec = 0;

        do {
            std::getline(Ostream_3B, st_line); // use this method as there are nan
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
                    Vt_sec_3B[index_sec]                       = t_sec;
                    ZZdist_hor_m[seed-seed_init][index_sec]    = Zdist_hor_m[index_all-1];
                    ZZerror_hor_m[seed-seed_init][index_sec]   = Zerror_hor_m[index_all-1];

                    index_sec++;
                }
            }

        } while (index_sec != nel_sec_new);

        Ostream_3B.close();

    }

    // reverse data (rows to columns and viceversa) to compute other metrics
    std::vector<std::vector<double>> WWerror_hor_m(nel_sec_def), WWerror_hor_m_1B(nel_sec_new), WWerror_hor_m_3B(nel_sec_new);
    std::vector<double> Wmean_error_hor_m(nel_sec_def), Wstd_error_hor_m(nel_sec_def), Wmean_error_hor_m_1B(nel_sec_new), Wstd_error_hor_m_1B(nel_sec_new), Wmean_error_hor_m_3B(nel_sec_new), Wstd_error_hor_m_3B(nel_sec_new);
    for (unsigned long i = 0; i != nel_sec_def; ++i) {
        WWerror_hor_m[i].resize(nel);
        for (unsigned long j = 0; j != nel; ++j) {
            WWerror_hor_m[i][j]      = VVerror_hor_m[j][i];
        }
        Wmean_error_hor_m[i]      = math::mean(WWerror_hor_m[i]);
        Wstd_error_hor_m[i]       = math::std(WWerror_hor_m[i], Wmean_error_hor_m[i]);
    }
    for (unsigned long i = 0; i != nel_sec_new; ++i) {
        WWerror_hor_m_1B[i].resize(nel);
        WWerror_hor_m_3B[i].resize(nel);
        for (unsigned long j = 0; j != nel; ++j) {
            WWerror_hor_m_1B[i][j] = YYerror_hor_m[j][i];
            WWerror_hor_m_3B[i][j] = ZZerror_hor_m[j][i];
        }
        Wmean_error_hor_m_1B[i] = math::mean(WWerror_hor_m_1B[i]);
        Wstd_error_hor_m_1B[i]  = math::std(WWerror_hor_m_1B[i], Wmean_error_hor_m_1B[i]);
        Wmean_error_hor_m_3B[i] = math::mean(WWerror_hor_m_3B[i]);
        Wstd_error_hor_m_3B[i]  = math::std(WWerror_hor_m_3B[i], Wmean_error_hor_m_3B[i]);
    }

    boost::filesystem::path path_folder(path_outputs_write / st_folder_main / st_extra_folder);
    boost::filesystem::create_directory(path_folder);

    std::string st_file_out_part1 = "versus_integr_pos_hor_m_pc_part1.txt";
    std::string st_file_out_part2 = "versus_integr_pos_hor_m_pc_part2.txt";
    std::string st_file_output_part1 = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_part1).string();
    std::string st_file_output_part2 = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_part2).string();

    ofstream Oout_part1;
    Oout_part1.open(st_file_output_part1);

    for (unsigned long i = 0; i != nel_sec_def; ++i) {
        Oout_part1 << fixed      << setw(8)  << setprecision(1) << showpos << Vt_sec_def[i]
                   << scientific << setw(12) << setprecision(3) << showpos << 1e-3 * Wmean_error_hor_m[i]
                   << scientific << setw(12) << setprecision(3) << showpos << 1e-3 * (Wmean_error_hor_m[i] + Wstd_error_hor_m[i])
                   << scientific << setw(12) << setprecision(3) << showpos << 1e-3 * (Wmean_error_hor_m[i] - Wstd_error_hor_m[i]);
        Oout_part1 << endl;
    }

    Oout_part1.close();

    ofstream Oout_part2;
    Oout_part2.open(st_file_output_part2);

    for (unsigned long i = 0; i != nel_sec_new; ++i) {
        Oout_part2 << fixed      << setw(8)  << setprecision(1) << showpos << Vt_sec_1B[i]
             << scientific << setw(12) << setprecision(3) << showpos << 1e-3 * Wmean_error_hor_m_1B[i]
             << scientific << setw(12) << setprecision(3) << showpos << 1e-3 * (Wmean_error_hor_m_1B[i] + Wstd_error_hor_m_1B[i])
             << scientific << setw(12) << setprecision(3) << showpos << 1e-3 * (Wmean_error_hor_m_1B[i] - Wstd_error_hor_m_1B[i])
             << fixed      << setw(8)  << setprecision(1) << showpos << Vt_sec_3B[i]
             << scientific << setw(12) << setprecision(3) << showpos << 1e-3 * Wmean_error_hor_m_3B[i]
             << scientific << setw(12) << setprecision(3) << showpos << 1e-3 * (Wmean_error_hor_m_3B[i] + Wstd_error_hor_m_3B[i])
             << scientific << setw(12) << setprecision(3) << showpos << 1e-3 * (Wmean_error_hor_m_3B[i] - Wstd_error_hor_m_3B[i]);
        Oout_part2 << endl;
    }

    Oout_part2.close();

}
/* reads the final horizontal position results from three sets of files, the baseline and two alternatives.
 * It shows results on console and also generates the following files (for thesis, not Matlab):
 * - "versus_integr_pos_hor_m_pc_part1.txt" --> to plot horizontal error mean and std variation with time
 * - "versus_integr_pos_hor_m_pc_part2.txt" --> to plot horizontal error mean and std variation with time */

void nav::test::Tplots_pos_filter::obtain_h_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end) {
    std::string st_folder           = st_first_folder;
    std::string st_folder_one       = "MAIN";
    std::string st_folder_one_2C    = "POS_FILT_2C";
    std::string st_folder_one_2D    = "POS_FILT_2D";
    
    boost::filesystem::path path_outputs_read(math::share::phd_outputs_store_prefix);
    boost::filesystem::path path_outputs_write(math::share::phd_outputs_prefix);
    std::string st_folder_main("nav");
    std::string st_extra_folder("versus_integr");
    std::string st_file_gps("filter_gps_xgdt.txt");
    std::string st_file_pos("filter_pos_xgdt.txt");
    std::string st_seeds, st_file_gps_complete, st_file_pos_complete, st_first_line, st_line, st_file_gps_complete_2C, st_file_pos_complete_2C, st_file_gps_complete_2D, st_file_pos_complete_2D;
    unsigned long nel     = (unsigned long)(seed_end) - (unsigned long)(seed_init) + 1;

    unsigned long nel_all_def = 380001, nel_all_new = 100001; // trajectory in 2C concludes at 1000 [sec]
    unsigned long nel_sec_def = 381, nel_sec_new = 101;
    unsigned long index_all, index_sec;
    std::vector<double> Vt_sec_def(nel_sec_def), Vt_sec_2C(nel_sec_new), Vt_sec_2D(nel_sec_new);
    std::vector<std::vector<double>> VVerror_h_m(nel), YYerror_h_m(nel), ZZerror_h_m(nel);
    std::vector<double> Verror_h_m(nel_all_def), Yerror_h_m(nel_all_new), Zerror_h_m(nel_all_new);

    double int_part, dec_part, t_sec;
    std::string st_t_sec, st_h_est_m, st_h_truth_m;

    // for each trajectory
    for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
        st_seeds = math::seeder::seed2string(seed);

        st_folder.replace(3,2,st_seeds);

        st_file_gps_complete    = (path_outputs_read / st_folder_main / st_folder_one    / st_folder / st_file_gps).string();
        st_file_pos_complete    = (path_outputs_read / st_folder_main / st_folder_one    / st_folder / st_file_pos).string();
        st_file_gps_complete_2C = (path_outputs_read / st_folder_main / st_folder_one_2C / st_folder / st_file_gps).string();
        st_file_pos_complete_2C = (path_outputs_read / st_folder_main / st_folder_one_2C / st_folder / st_file_pos).string();
        st_file_gps_complete_2D = (path_outputs_read / st_folder_main / st_folder_one_2D / st_folder / st_file_gps).string();
        st_file_pos_complete_2D = (path_outputs_read / st_folder_main / st_folder_one_2D / st_folder / st_file_pos).string();
        
        std::ifstream Ostream_gps;
        Ostream_gps.open(st_file_gps_complete);
        std::getline(Ostream_gps, st_first_line);

        VVerror_h_m[seed-seed_init].resize(nel_sec_def);

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
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec_def[index_sec] = t_sec;
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
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec_def[index_sec] = t_sec;
                    VVerror_h_m[seed - seed_init][index_sec] = Verror_h_m[index_all - 1];
                    index_sec++;
                }
            }
        }
        Ostream_pos.close();


        std::ifstream Ostream_gps_2C;
        Ostream_gps_2C.open(st_file_gps_complete_2C);
        std::getline(Ostream_gps_2C, st_first_line);

        YYerror_h_m[seed-seed_init].resize(nel_sec_new);

        index_all = 0;
        index_sec = 0;

        while (std::getline(Ostream_gps_2C, st_line)) {
            st_t_sec     = st_line.substr(  0, 10);
            st_h_est_m   = st_line.substr( 46, 15);
            st_h_truth_m = st_line.substr(148, 15);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec / 10, &int_part);
                Yerror_h_m[index_all] = std::stod(st_h_est_m) - std::stod(st_h_truth_m);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec_2C[index_sec] = t_sec;
                    YYerror_h_m[seed - seed_init][index_sec] = Yerror_h_m[index_all - 1];
                    index_sec++;
                }
            }
        }
        Ostream_gps_2C.close();


        std::ifstream Ostream_pos_2C;
        Ostream_pos_2C.open(st_file_pos_complete_2C);
        std::getline(Ostream_pos_2C, st_first_line);
        std::getline(Ostream_pos_2C, st_line); // line repeated at 100 [sec] from previous file
        while (std::getline(Ostream_pos_2C, st_line)) {
            st_t_sec     = st_line.substr( 0,10);
            st_h_est_m   = st_line.substr(46,15);
            st_h_truth_m = st_line.substr(97,15);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec / 10, &int_part);
                Yerror_h_m[index_all] = std::stod(st_h_est_m) - std::stod(st_h_truth_m);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec_2C[index_sec] = t_sec;
                    YYerror_h_m[seed - seed_init][index_sec] = Yerror_h_m[index_all - 1];
                    index_sec++;
                }
            }
        }
        Ostream_pos_2C.close();

        
        std::ifstream Ostream_gps_2D;
        Ostream_gps_2D.open(st_file_gps_complete_2D);
        std::getline(Ostream_gps_2D, st_first_line);

        ZZerror_h_m[seed-seed_init].resize(nel_sec_new);

        index_all = 0;
        index_sec = 0;

        while (std::getline(Ostream_gps_2D, st_line)) {
            st_t_sec     = st_line.substr(  0, 10);
            st_h_est_m   = st_line.substr( 46, 15);
            st_h_truth_m = st_line.substr(148, 15);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec / 10, &int_part);
                Zerror_h_m[index_all] = std::stod(st_h_est_m) - std::stod(st_h_truth_m);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec_2D[index_sec] = t_sec;
                    ZZerror_h_m[seed - seed_init][index_sec] = Zerror_h_m[index_all - 1];
                    index_sec++;
                }
            }
        }
        Ostream_gps_2D.close();


        std::ifstream Ostream_pos_2D;
        Ostream_pos_2D.open(st_file_pos_complete_2D);
        std::getline(Ostream_pos_2D, st_first_line);
        std::getline(Ostream_pos_2D, st_line); // line repeated at 100 [sec] from previous file
        while (std::getline(Ostream_pos_2D, st_line)) {
            st_t_sec     = st_line.substr( 0,10);
            st_h_est_m   = st_line.substr(46,15);
            st_h_truth_m = st_line.substr(97,15);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec / 10, &int_part);
                Zerror_h_m[index_all] = std::stod(st_h_est_m) - std::stod(st_h_truth_m);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec_2D[index_sec] = t_sec;
                    ZZerror_h_m[seed - seed_init][index_sec] = Zerror_h_m[index_all - 1];
                    index_sec++;
                }
            }
        }
        Ostream_pos_2D.close();

    }

    // reverse data (rows to columns and viceversa) to compute other metrics
    std::vector<std::vector<double>> WWerror_h_m(nel_sec_def), WWerror_h_m_2C(nel_sec_new), WWerror_h_m_2D(nel_sec_new);
    std::vector<double> Wmean_error_h_m(nel_sec_def), Wstd_error_h_m(nel_sec_def), Wmean_error_h_m_2C(nel_sec_new), Wstd_error_h_m_2C(nel_sec_new), Wmean_error_h_m_2D(nel_sec_new), Wstd_error_h_m_2D(nel_sec_new);
    for (unsigned long i = 0; i != nel_sec_def; ++i) {
        WWerror_h_m[i].resize(nel);
        for (unsigned long j = 0; j != nel; ++j) {
            WWerror_h_m[i][j]      = VVerror_h_m[j][i];
        }
        Wmean_error_h_m[i]      = math::mean(WWerror_h_m[i]);
        Wstd_error_h_m[i]       = math::std(WWerror_h_m[i], Wmean_error_h_m[i]);
    }
    for (unsigned long i = 0; i != nel_sec_new; ++i) {
        WWerror_h_m_2C[i].resize(nel);
        WWerror_h_m_2D[i].resize(nel);
        for (unsigned long j = 0; j != nel; ++j) {
            WWerror_h_m_2C[i][j]  = YYerror_h_m[j][i];
            WWerror_h_m_2D[i][j]  = ZZerror_h_m[j][i];
        }
        Wmean_error_h_m_2C[i]      = math::mean(WWerror_h_m_2C[i]);
        Wstd_error_h_m_2C[i]       = math::std(WWerror_h_m_2C[i], Wmean_error_h_m_2C[i]);
        Wmean_error_h_m_2D[i]      = math::mean(WWerror_h_m_2D[i]);
        Wstd_error_h_m_2D[i]       = math::std(WWerror_h_m_2D[i], Wmean_error_h_m_2D[i]);
    }

    boost::filesystem::path path_folder(path_outputs_write / st_folder_main / st_extra_folder);
    boost::filesystem::create_directory(path_folder);

    std::string st_file_out_part1 = "versus_integr_pos_h_m_part1.txt";
    std::string st_file_out_part2 = "versus_integr_pos_h_m_part2.txt";
    std::string st_file_output_part1 = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_part1).string();
    std::string st_file_output_part2 = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_part2).string();

    ofstream Oout_part1;
    Oout_part1.open(st_file_output_part1);

    for (unsigned long i = 0; i != nel_sec_def; ++i) {
        Oout_part1 << fixed      << setw(8)  << setprecision(1) << showpos << Vt_sec_def[i]
                   << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_h_m[i]
                   << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_h_m[i] + Wstd_error_h_m[i]
                   << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_h_m[i] - Wstd_error_h_m[i];
        Oout_part1 << endl;
    }

    Oout_part1.close();

    ofstream Oout_part2;
    Oout_part2.open(st_file_output_part2);

    for (unsigned long i = 0; i != nel_sec_new; ++i) {
        Oout_part2 << fixed      << setw(8)  << setprecision(1) << showpos << Vt_sec_2C[i]
                << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_h_m_2C[i]
                << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_h_m_2C[i] + Wstd_error_h_m_2C[i]
                << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_h_m_2C[i] - Wstd_error_h_m_2C[i]
                << fixed      << setw(8)  << setprecision(1) << showpos << Vt_sec_2D[i]
                << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_h_m_2D[i]
                << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_h_m_2D[i] + Wstd_error_h_m_2D[i]
                << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_h_m_2D[i] - Wstd_error_h_m_2D[i];
        Oout_part2 << endl;
    }

    Oout_part2.close();

}
/* reads the final vertical position results from two sets of files, the baseline and one alternatives.
 * It shows results on console and also generates the following files (for thesis, not Matlab):
 * - "versus_integr_pos_h_m_part1.txt" --> to plot vertical error mean and std variation with time
 * - "versus_integr_pos_h_m_part2.txt" --> to plot vertical error mean and std variation with time */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////




























