#include "Tplots_gnss.h"

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

nav::test::Tplots_gnss::Tplots_gnss(unsigned short& seed_init, unsigned short& seed_end)
: _seed_init(seed_init), _seed_end(seed_end) {
}
/* constructor based on counter */

void nav::test::Tplots_gnss::run() {

    obtain_euler_metrics("01_01_05_02_03_0100", "01_01_05_02_03_3800", _seed_init, _seed_end);
    obtain_h_metrics    ("01_01_05_02_03_0100", "01_01_05_02_03_3800", _seed_init, _seed_end);
    obtain_xhor_metrics ("01_01_05_02_03_0100", "01_01_05_02_03_3800", _seed_init, _seed_end);

}
/* execute tests and write results on console */

using namespace std;

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void nav::test::Tplots_gnss::obtain_euler_metrics(const std::string& st_first_folder, const std::string& st_first_folder_gnss, unsigned short& seed_init, unsigned short& seed_end) {
    std::string st_folder           = st_first_folder;
    std::string st_folder_gnss      = st_first_folder_gnss;
    std::string st_folder_one       = "MAIN";
    std::string st_folder_one_gnss  = "GNSS";

    std::cout << std::endl;
    std::cout << "Body attitude estimation in attitude filter [deg]:" << std::endl;
    std::cout << setw(9) << " "
              << setw(21) << "total error"
              << setw(21) << "total error w GNSS"<< std::endl;
    std::cout << setw(9)  << "seed"
              << setw(7) << "mean" << setw(7) << "std" << setw(7) << "smax"
              << setw(7) << "mean" << setw(7) << "std" << setw(7) << "smax"
              << std::endl;

    boost::filesystem::path path_outputs_read(math::share::phd_outputs_store_prefix);
    boost::filesystem::path path_outputs_write(math::share::phd_outputs_prefix);
    std::string st_folder_main("nav");
    std::string st_extra_folder("versus_gnss");
    std::string st_file("filter_att_euler.txt");

    std::string st_seeds, st_file_complete, st_first_line, st_file_complete_gnss;
    double a01, a02, a03, a04, a05, a06, a07, a08, a09, a10, a11, a12;
    unsigned long nel     = (unsigned long)(seed_end) - (unsigned long)(seed_init) + 1;
    unsigned long nel_all = 380001;
    unsigned long nel_sec = 381;
    unsigned long index_all, index_sec;
    std::vector<double> Vt_sec(nel_sec);
    std::vector<std::vector<double>> VVangle_deg(nel),ZZangle_deg(nel);
    std::vector<double> Vangle_deg_all(nel_all), Zangle_deg_all(nel_all);
    std::vector<double> Vmean_angle_deg(nel), Vstd_angle_deg(nel), Vsmax_angle_deg(nel);
    std::vector<double> Zmean_angle_deg(nel), Zstd_angle_deg(nel), Zsmax_angle_deg(nel);
    double int_part, dec_part;

    // for each trajectory
    for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
        st_seeds = math::seeder::seed2string(seed);

        st_folder.replace(3,2,st_seeds);
        st_folder_gnss.replace(3,2,st_seeds);
        st_file_complete      = (path_outputs_read / st_folder_main / st_folder_one      / st_folder      / st_file).string();
        st_file_complete_gnss = (path_outputs_read / st_folder_main / st_folder_one_gnss / st_folder_gnss / st_file).string();

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

        std::ifstream Ostream_gnss;
        Ostream_gnss.open(st_file_complete_gnss);

        ZZangle_deg[seed-seed_init].resize(nel_sec); // do not forget

        std::getline(Ostream_gnss, st_first_line);
        index_all = 0;
        index_sec = 0;

        do {
            Ostream_gnss >> a01 >> a02 >> a03 >> a04 >> a05 >> a06 >> a07 >> a08 >> a09 >> a10 >> a11 >> a12;
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

        Ostream_gnss.close();

        // compute metrics for each trajectory
        Vmean_angle_deg[seed-seed_init]  = math::mean(Vangle_deg_all);
        Vstd_angle_deg[seed-seed_init]   = math::std(Vangle_deg_all, Vmean_angle_deg[seed-seed_init]);
        Vsmax_angle_deg[seed-seed_init]  = math::smax(Vangle_deg_all);

        Zmean_angle_deg[seed-seed_init]  = math::mean(Zangle_deg_all);
        Zstd_angle_deg[seed-seed_init]   = math::std(Zangle_deg_all, Zmean_angle_deg[seed-seed_init]);
        Zsmax_angle_deg[seed-seed_init]  = math::smax(Zangle_deg_all);

        std::cout << fixed << setw(9) << setprecision(0)              << seed
                  << fixed << setw(7) << setprecision(3) << noshowpos << Vmean_angle_deg[seed-seed_init]
                  << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_angle_deg[seed-seed_init]
                  << fixed << setw(7) << setprecision(3) << noshowpos << Vsmax_angle_deg[seed-seed_init]
                  << fixed << setw(7) << setprecision(3) << noshowpos << Zmean_angle_deg[seed-seed_init]
                  << fixed << setw(7) << setprecision(3) << noshowpos << Zstd_angle_deg[seed-seed_init]
                  << fixed << setw(7) << setprecision(3) << noshowpos << Zsmax_angle_deg[seed-seed_init]
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

    double mean_mean_angle_deg_gnss = math::mean(Zmean_angle_deg);
    double std_mean_angle_deg_gnss  = math::std(Zmean_angle_deg, mean_mean_angle_deg_gnss);
    double smax_mean_angle_deg_gnss = math::smax(Zmean_angle_deg);

    double mean_std_angle_deg_gnss  = math::mean(Zstd_angle_deg);
    double std_std_angle_deg_gnss   = math::std(Zstd_angle_deg, mean_std_angle_deg_gnss);
    double smax_std_angle_deg_gnss  = math::smax(Zstd_angle_deg);

    double mean_smax_angle_deg_gnss = math::mean(Zsmax_angle_deg);
    double std_smax_angle_deg_gnss  = math::std(Zsmax_angle_deg, mean_smax_angle_deg_gnss);
    double smax_smax_angle_deg_gnss = math::smax(Zsmax_angle_deg);

    std::cout << "Mean:    "
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_mean_angle_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_std_angle_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_smax_angle_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_mean_angle_deg_gnss
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_std_angle_deg_gnss
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_smax_angle_deg_gnss
              << std::endl;
    std::cout << "std:     "
              << fixed << setw(7) << setprecision(3) << noshowpos << std_mean_angle_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << std_std_angle_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << std_smax_angle_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << std_mean_angle_deg_gnss
              << fixed << setw(7) << setprecision(3) << noshowpos << std_std_angle_deg_gnss
              << fixed << setw(7) << setprecision(3) << noshowpos << std_smax_angle_deg_gnss
              << std::endl;
    std::cout << "smax:    "
              << fixed << setw(7) << setprecision(3) << noshowpos << smax_mean_angle_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << smax_std_angle_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << smax_smax_angle_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << smax_mean_angle_deg_gnss
              << fixed << setw(7) << setprecision(3) << noshowpos << smax_std_angle_deg_gnss
              << fixed << setw(7) << setprecision(3) << noshowpos << smax_smax_angle_deg_gnss
              << std::endl;

    // reverse data (rows to columns and viceversa) to compute gnss metrics
    std::vector<std::vector<double>> WWangle_deg(nel_sec), WWangle_deg_gnss(nel_sec);
    std::vector<double> Wmean_angle_deg(nel_sec), Wstd_angle_deg(nel_sec), Wmean_angle_deg_gnss(nel_sec), Wstd_angle_deg_gnss(nel_sec);;
    for (unsigned long i = 0; i != nel_sec; ++i) {
        WWangle_deg[i].resize(nel);
        WWangle_deg_gnss[i].resize(nel);
        for (unsigned long j = 0; j != nel; ++j) {
            WWangle_deg[i][j]      = VVangle_deg[j][i];
            WWangle_deg_gnss[i][j] = ZZangle_deg[j][i];
        }
        Wmean_angle_deg[i] = math::mean(WWangle_deg[i]);
        Wstd_angle_deg[i]  = math::std(WWangle_deg[i], Wmean_angle_deg[i]);
        Wmean_angle_deg_gnss[i] = math::mean(WWangle_deg_gnss[i]);
        Wstd_angle_deg_gnss[i]  = math::std(WWangle_deg_gnss[i], Wmean_angle_deg_gnss[i]);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////


    boost::filesystem::path path_folder(path_outputs_write / st_folder_main / st_extra_folder);
    boost::filesystem::create_directory(path_folder);

    std::string st_file_out = "versus_gnss_att_euler_deg.txt";
    std::string st_file_output = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out).string();

    ofstream Oout;
    Oout.open(st_file_output);

    for (unsigned long i = 0; i != nel_sec; ++i) {
        Oout << fixed      << setw(8)  << setprecision(1) << showpos << Vt_sec[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_angle_deg[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_angle_deg[i] + Wstd_angle_deg[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_angle_deg[i] - Wstd_angle_deg[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_angle_deg_gnss[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_angle_deg_gnss[i] + Wstd_angle_deg_gnss[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_angle_deg_gnss[i] - Wstd_angle_deg_gnss[i];
        Oout << endl;
    }

    Oout.close();

    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    std::string st_file_out_tex = "versus_gnss_att_euler_table.tex";
    std::string st_file_output_tex = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_tex).string();

    ofstream Oout_tex;
    Oout_tex.open(st_file_output_tex);

    Oout_tex << "\\begin{center}" << std::endl;
    Oout_tex << "\\begin{tabular}{lrrrp{0.5cm}rrr}" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\textbf{Benchmark} & \\multicolumn{7}{c}{\\textbf{Configuration}} \\\\" << std::endl;
    Oout_tex << "\\nm{\\| \\phiNBest - \\phiNB \\| \\ \\lrsb{deg}} & \\multicolumn{3}{c}{\\textbf{Baseline}} & & \\multicolumn{3}{c}{\\textbf{GNSS-Based}} \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\nm{\\mumu{}, \\, \\musigma{}, \\, \\mumax{\\cdot}} "
             << " & \\textbf{"
             << fixed      << setw(7)  << setprecision(3) << noshowpos << mean_mean_angle_deg
             << "} &"
             << fixed      << setw(7)  << setprecision(3) << noshowpos << mean_std_angle_deg
             << " & "
             << fixed      << setw(7)  << setprecision(3) << noshowpos << mean_smax_angle_deg
             << " & & \\textbf{"
             << fixed      << setw(7)  << setprecision(3) << noshowpos << mean_mean_angle_deg_gnss
             << "} & "
             << fixed      << setw(7)  << setprecision(3) << noshowpos << mean_std_angle_deg_gnss
             << " & "
             << fixed      << setw(7)  << setprecision(3) << noshowpos << mean_smax_angle_deg_gnss
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\sigmamu{}, \\, \\sigmasigma{}, \\, \\sigmamax{\\cdot}}"
             << " & "
             << fixed      << setw(7)  << setprecision(3) << noshowpos << std_mean_angle_deg
             << " & "
             << fixed      << setw(7)  << setprecision(3) << noshowpos << std_std_angle_deg
             << " & "
             << fixed      << setw(7)  << setprecision(3) << noshowpos << std_smax_angle_deg
             << " & & "
             << fixed      << setw(7)  << setprecision(3) << noshowpos << std_mean_angle_deg_gnss
             << " & "
             << fixed      << setw(7)  << setprecision(3) << noshowpos << std_std_angle_deg_gnss
             << " & "
             << fixed      << setw(7)  << setprecision(3) << noshowpos << std_smax_angle_deg_gnss
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\maxmu{}, \\, \\maxsigma{}, \\, \\maxmax{\\cdot}} "
             << " & "
             << fixed      << setw(7)  << setprecision(3) << noshowpos << smax_mean_angle_deg
             << " & "
             << fixed      << setw(7)  << setprecision(3) << noshowpos << smax_std_angle_deg
             << " & "
             << fixed      << setw(7)  << setprecision(3) << noshowpos << smax_smax_angle_deg
             << " & & "
             << fixed      << setw(7)  << setprecision(3) << noshowpos << smax_mean_angle_deg_gnss
             << " & "
             << fixed      << setw(7)  << setprecision(3) << noshowpos << smax_std_angle_deg_gnss
             << " & "
             << fixed      << setw(7)  << setprecision(3) << noshowpos << smax_smax_angle_deg_gnss
             << " \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\end{tabular}" << std::endl;
    Oout_tex << "\\end{center}" << std::endl;
    Oout_tex.close();
}
/* reads the body attitude estimation results from two sets of files, the baseline and the one employing GNSS.
 * It shows results on console and also generates the following files (for thesis, not Matlab):
 * - "versus_gnss_att_euler_deg.txt" --> to plot euler mean and std variation with time
 * - "versus_gnss_att_euler_table.tex" --> script to directly generate table in Latex */


void nav::test::Tplots_gnss::obtain_h_metrics(const std::string& st_first_folder, const std::string& st_first_folder_gnss, unsigned short& seed_init, unsigned short& seed_end) {
    std::string st_folder           = st_first_folder;
    std::string st_folder_gnss      = st_first_folder_gnss;
    std::string st_folder_one       = "MAIN";
    std::string st_folder_one_gnss  = "GNSS";

    std::cout << std::endl;
    std::cout << "Final geometric altitude estimation in position filter [m]:" << std::endl;
    std::cout << setw(9) << "seed"
              << setw(20) << "total error"
              << setw(20) << "total error w GNSS"<< std::endl;

    boost::filesystem::path path_outputs_read(math::share::phd_outputs_store_prefix);
    boost::filesystem::path path_outputs_write(math::share::phd_outputs_prefix);
    std::string st_folder_main("nav");
    std::string st_extra_folder("versus_gnss");
    std::string st_file_gps("filter_gps_xgdt.txt");
    std::string st_file_pos("filter_pos_xgdt.txt");
    std::string st_seeds, st_file_gps_complete, st_file_pos_complete, st_first_line, st_line, st_file_complete_gnss;
    unsigned long nel     = (unsigned long)(seed_end) - (unsigned long)(seed_init) + 1;
    unsigned long nel_all = 380001;
    unsigned long nel_sec = 381;
    unsigned long index_all, index_sec;
    std::vector<double> Vt_sec(nel_sec);
    std::vector<std::vector<double>> VVerror_h_m(nel), ZZerror_h_m(nel);
    std::vector<double> Verror_h_m(nel_all), Vh_m_est(nel_all), Vh_m_truth(nel_all), Zerror_h_m(nel_all), Zh_m_est(nel_all), Zh_m_truth(nel_all);
    std::vector<double> Vend_error_h_m(nel), Vabs_end_error_h_m(nel), Vend_h_m_est(nel), Vend_h_m_truth(nel);
    std::vector<double> Zend_error_h_m(nel), Zabs_end_error_h_m(nel), Zend_h_m_est(nel), Zend_h_m_truth(nel);
    double int_part, dec_part, t_sec;
    std::string st_t_sec, st_h_est_m, st_h_truth_m;

    // for each trajectory
    for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
        st_seeds = math::seeder::seed2string(seed);

        st_folder.replace(3,2,st_seeds);
        st_folder_gnss.replace(3,2,st_seeds);
        st_file_gps_complete  = (path_outputs_read / st_folder_main / st_folder_one      / st_folder      / st_file_gps).string();
        st_file_pos_complete  = (path_outputs_read / st_folder_main / st_folder_one      / st_folder      / st_file_pos).string();
        st_file_complete_gnss = (path_outputs_read / st_folder_main / st_folder_one_gnss / st_folder_gnss / st_file_gps).string();

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


        std::ifstream Ostream_gnss;
        Ostream_gnss.open(st_file_complete_gnss);
        std::getline(Ostream_gnss, st_first_line);

        ZZerror_h_m[seed - seed_init].resize(nel_sec); // do not forget

        index_all = 0;
        index_sec = 0;

        while (std::getline(Ostream_gnss, st_line)) {
            st_t_sec = st_line.substr(0, 10);
            st_h_est_m = st_line.substr(46, 15);
            st_h_truth_m = st_line.substr(148, 15);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec / 10, &int_part);
                Zerror_h_m[index_all] = std::stod(st_h_est_m) - std::stod(st_h_truth_m);
                Zh_m_est[index_all] = std::stod(st_h_est_m);
                Zh_m_truth[index_all] = std::stod(st_h_truth_m);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = t_sec;
                    ZZerror_h_m[seed - seed_init][index_sec] = Zerror_h_m[index_all - 1];
                    index_sec++;
                }
            }
        }
        Ostream_gnss.close();

        // compute metrics for each trajectory
        Vend_error_h_m[seed-seed_init]       = Verror_h_m.back();
        Vabs_end_error_h_m[seed-seed_init]   = fabs(Vend_error_h_m[seed-seed_init]);
        Vend_h_m_est[seed-seed_init]         = Vh_m_est.back();
        Vend_h_m_truth[seed-seed_init]       = Vh_m_truth.back();
        Zend_error_h_m[seed-seed_init]       = Zerror_h_m.back();
        Zabs_end_error_h_m[seed-seed_init]   = fabs(Zend_error_h_m[seed-seed_init]);
        Zend_h_m_est[seed-seed_init]         = Zh_m_est.back();
        Zend_h_m_truth[seed-seed_init]       = Zh_m_truth.back();

        std::cout << fixed << setw(9)  << setprecision(0)              << seed
                  << fixed << setw(20) << setprecision(2) <<   showpos << Vend_error_h_m[seed-seed_init]
                  << fixed << setw(20) << setprecision(2) <<   showpos << Zend_error_h_m[seed-seed_init]
                  << std::endl;
    }

    // obtain aggregated metrics
    double mean_end_error_h_m      = math::mean(Vend_error_h_m);
    double std_end_error_h_m       = math::std(Vend_error_h_m, mean_end_error_h_m);
    double smax_end_error_h_m      = math::smax(Vend_error_h_m);
    double mean_end_error_h_m_gnss = math::mean(Zend_error_h_m);
    double std_end_error_h_m_gnss  = math::std(Zend_error_h_m, mean_end_error_h_m_gnss);
    double smax_end_error_h_m_gnss = math::smax(Zend_error_h_m);

    std::cout << "Mean:    "
              << fixed << setw(20) << setprecision(2) <<   showpos << mean_end_error_h_m
              << fixed << setw(20) << setprecision(2) <<   showpos << mean_end_error_h_m_gnss
              << std::endl;
    std::cout << "std:     "
              << fixed << setw(20) << setprecision(2) << noshowpos << std_end_error_h_m
              << fixed << setw(20) << setprecision(2) << noshowpos << std_end_error_h_m_gnss
              << std::endl;
    std::cout << "smax:    "
              << fixed << setw(20) << setprecision(2) <<   showpos << smax_end_error_h_m
              << fixed << setw(20) << setprecision(2) <<   showpos << smax_end_error_h_m_gnss
              << std::endl;

    // reverse data (rows to columns and viceversa) to compute other metrics
    std::vector<std::vector<double>> WWerror_h_m(nel_sec), WWerror_h_m_gnss(nel_sec);
    std::vector<double> Wmean_error_h_m(nel_sec), Wstd_error_h_m(nel_sec), Wmean_error_h_m_gnss(nel_sec), Wstd_error_h_m_gnss(nel_sec);
    for (unsigned long i = 0; i != nel_sec; ++i) {
        WWerror_h_m[i].resize(nel);
        WWerror_h_m_gnss[i].resize(nel);
        for (unsigned long j = 0; j != nel; ++j) {
            WWerror_h_m[i][j]      = VVerror_h_m[j][i];
            WWerror_h_m_gnss[i][j] = ZZerror_h_m[j][i];
        }
        Wmean_error_h_m[i]      = math::mean(WWerror_h_m[i]);
        Wstd_error_h_m[i]       = math::std(WWerror_h_m[i], Wmean_error_h_m[i]);
        Wmean_error_h_m_gnss[i] = math::mean(WWerror_h_m_gnss[i]);
        Wstd_error_h_m_gnss[i]  = math::std(WWerror_h_m_gnss[i], Wmean_error_h_m_gnss[i]);

    }

    boost::filesystem::path path_folder(path_outputs_write / st_folder_main / st_extra_folder);
    boost::filesystem::create_directory(path_folder);

    std::string st_file_out = "versus_gnss_pos_h_m.txt";
    std::string st_file_output = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out).string();

    ofstream Oout;
    Oout.open(st_file_output);

    for (unsigned long i = 0; i != nel_sec; ++i) {
        Oout << fixed      << setw(8)  << setprecision(1) << showpos << Vt_sec[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_h_m[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_h_m[i] + Wstd_error_h_m[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_h_m[i] - Wstd_error_h_m[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_h_m_gnss[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_h_m_gnss[i] + Wstd_error_h_m_gnss[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_h_m_gnss[i] - Wstd_error_h_m_gnss[i];
        Oout << endl;
    }
    Oout.close();

    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    std::string st_file_out_tex = "versus_gnss_pos_h_table.tex";
    std::string st_file_output_tex = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_tex).string();

    ofstream Oout_tex;
    Oout_tex.open(st_file_output_tex);

    Oout_tex << "\\begin{center}" << std::endl;
    Oout_tex << "\\begin{tabular}{lrr}" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\textbf{Benchmark} & \\multicolumn{2}{c}{\\textbf{Configuration}} \\\\" << std::endl;
    Oout_tex << "\\nm{\\hest - h \\ \\lrsb{m}} & \\textbf{Baseline} & \\textbf{GNSS-Based} \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\nm{\\muEND{h}} "
             << " & "
             << fixed      << setw(7)  << setprecision(2) <<   showpos << mean_end_error_h_m
             << " & "
             << fixed      << setw(7)  << setprecision(2) <<   showpos << mean_end_error_h_m_gnss
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\sigmaEND{h}} "
             << " & \\textbf{"
             << fixed      << setw(7)  << setprecision(2) << noshowpos << std_end_error_h_m
             << "} & \\textbf{"
             << fixed      << setw(7)  << setprecision(2) << noshowpos << std_end_error_h_m_gnss
             << "} \\\\" << std::endl;
    Oout_tex << "\\nm{\\maxEND{h}} "
             << " & "
             << fixed      << setw(7)  << setprecision(2) <<   showpos << smax_end_error_h_m
             << " & "
             << fixed      << setw(7)  << setprecision(2) <<   showpos << smax_end_error_h_m_gnss
             << " \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\end{tabular}" << std::endl;
    Oout_tex << "\\end{center}" << std::endl;
    Oout_tex.close();

    //////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////

}
/* reads the final altitude estimation results from two sets of files, the baseline and the one employing GNSS.
 * It shows results on console and also generates the following files (for thesis, not Matlab):
 * - "versus_gnss_pos_h_m.txt" --> to plot altitude mean and std variation with time
 * - "versus_gnss_pos_h.tex" --> script to directly generate table in Latex */

void nav::test::Tplots_gnss::obtain_xhor_metrics(const std::string& st_first_folder, const std::string& st_first_folder_gnss, unsigned short& seed_init, unsigned short& seed_end) {
    std::string st_folder           = st_first_folder;
    std::string st_folder_gnss      = st_first_folder_gnss;
    std::string st_folder_one       = "MAIN";
    std::string st_folder_one_gnss  = "GNSS";

    std::cout << std::endl;
    std::cout << "Final horizontal position estimation in position filter [m,%]:" << std::endl;
    std::cout << setw(9) << "seed"
              << setw(27) << "total error"
              << setw(22) << "total error w GNSS"<< std::endl;
    std::cout << setw(9) << "seed"
              << setw(11) << "distance" << setw(16) << "hor error"
              << setw(11) << "distance" << setw(11) << "hor error"
              << std::endl;

    boost::filesystem::path path_outputs_read(math::share::phd_outputs_store_prefix);
    boost::filesystem::path path_outputs_write(math::share::phd_outputs_prefix);
    std::string st_folder_main("nav");
    std::string st_extra_folder("versus_gnss");
    std::string st_file("output_pos.txt");
    std::string st_seeds, st_file_complete, st_file_complete_gnss, st_first_line, st_line;
    unsigned long nel     = (unsigned long)(seed_end) - (unsigned long)(seed_init) + 1;
    unsigned long nel_all = 38001; // note this file is written every 0.1 [sec] instead of 0.01 [sec]
    unsigned long nel_sec = 381;
    double t_sec_gnss = 100.0;
    unsigned long index_all, index_sec, index_sec_gnss;
    std::vector<double> Vt_sec(nel_sec);
    std::vector<std::vector<double>> VVerror_hor_m(nel), VVdist_hor_m(nel);
    std::vector<std::vector<double>> ZZerror_hor_m(nel), ZZdist_hor_m(nel);
    std::vector<double> Verror_hor_m(nel_all), Vdist_hor_m(nel_all), Verror_hor_m_end(nel), Vdist_hor_m_end(nel), Verror_hor_pc_end(nel);
    std::vector<double> Zerror_hor_m(nel_all), Zdist_hor_m(nel_all), Zerror_hor_m_end(nel), Zdist_hor_m_end(nel);
    std::string st_t_sec, st_err_hor_m, st_dist_hor_m;
    double int_part, dec_part, t_sec, err_hor_m, dist_hor_m;

    // for each trajectory
    for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
        st_seeds = math::seeder::seed2string(seed);

        st_folder.replace(3,2,st_seeds);
        st_folder_gnss.replace(3,2,st_seeds);
        st_file_complete      = (path_outputs_read / st_folder_main / st_folder_one      / st_folder      / st_file).string();
        st_file_complete_gnss = (path_outputs_read / st_folder_main / st_folder_one_gnss / st_folder_gnss / st_file).string();

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

        std::ifstream Ostream_gnss;
        Ostream_gnss.open(st_file_complete_gnss);

        ZZdist_hor_m[seed-seed_init].resize(nel_sec); // do not forget
        ZZerror_hor_m[seed-seed_init].resize(nel_sec); // do not forget

        std::getline(Ostream_gnss, st_first_line);
        index_all = 0;
        index_sec = 0;

        do {
            std::getline(Ostream_gnss, st_line); // use this method as there are nan
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

        Ostream_gnss.close();

        // compute metrics for each trajectory
        Vdist_hor_m_end[seed-seed_init]     = Vdist_hor_m.back() - Vdist_hor_m[index_sec_gnss];
        Verror_hor_m_end[seed-seed_init]    = std::fabs(Verror_hor_m.back());
        Verror_hor_pc_end[seed-seed_init]   = 100 * Verror_hor_m_end[seed-seed_init] / Vdist_hor_m_end[seed-seed_init];

        Zdist_hor_m_end[seed-seed_init]     = Zdist_hor_m.back();
        Zerror_hor_m_end[seed-seed_init]    = std::fabs(Zerror_hor_m.back());

        std::cout << fixed << setw(9)  << setprecision(0)              << seed
                  << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[seed-seed_init]
                  << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[seed-seed_init]
                  << fixed << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[seed-seed_init]
                  << fixed << setw(11) << setprecision(0) << noshowpos << Zdist_hor_m_end[seed-seed_init]
                  << fixed << setw(11) << setprecision(2) << noshowpos << Zerror_hor_m_end[seed-seed_init]
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

    double mean_dist_hor_m_end_gnss = math::mean(Zdist_hor_m_end);
    double std_dist_hor_m_end_gnss  = math::std(Zdist_hor_m_end, mean_dist_hor_m_end_gnss);
    double smax_dist_hor_m_end_gnss = math::smax(Zdist_hor_m_end);

    double mean_error_hor_m_end_gnss = math::mean(Zerror_hor_m_end);
    double std_error_hor_m_end_gnss  = math::std(Zerror_hor_m_end, mean_error_hor_m_end_gnss);
    double smax_error_hor_m_end_gnss = math::smax(Zerror_hor_m_end);

    std::cout << "Mean:    "
              << fixed << setw(11) << setprecision(0) << noshowpos << mean_dist_hor_m_end
              << fixed << setw(10) << setprecision(0) << noshowpos << mean_error_hor_m_end
              << fixed << setw(6)  << setprecision(2) << noshowpos << mean_error_hor_pc_end
              << fixed << setw(11) << setprecision(0) << noshowpos << mean_dist_hor_m_end_gnss
              << fixed << setw(11) << setprecision(2) << noshowpos << mean_error_hor_m_end_gnss
              << std::endl;
    std::cout << "std:     "
              << fixed << setw(11) << setprecision(0) << noshowpos << std_dist_hor_m_end
              << fixed << setw(10) << setprecision(0) << noshowpos << std_error_hor_m_end
              << fixed << setw(6)  << setprecision(2) << noshowpos << std_error_hor_pc_end
              << fixed << setw(11) << setprecision(0) << noshowpos << std_dist_hor_m_end_gnss
              << fixed << setw(11) << setprecision(2) << noshowpos << std_error_hor_m_end_gnss
              << std::endl;
    std::cout << "smax:    "
              << fixed << setw(11) << setprecision(0) << noshowpos << smax_dist_hor_m_end
              << fixed << setw(10) << setprecision(0) << noshowpos << smax_error_hor_m_end
              << fixed << setw(6)  << setprecision(2) << noshowpos << smax_error_hor_pc_end
              << fixed << setw(11) << setprecision(0) << noshowpos << smax_dist_hor_m_end_gnss
              << fixed << setw(11) << setprecision(2) << noshowpos << smax_error_hor_m_end_gnss
              << std::endl;

    // reverse data (rows to columns and viceversa) to compute other metrics
    std::vector<std::vector<double>> WWerror_hor_m(nel_sec), WWerror_hor_m_gnss(nel_sec);
    std::vector<double> Wmean_error_hor_m(nel_sec), Wstd_error_hor_m(nel_sec), Wmean_error_hor_m_gnss(nel_sec), Wstd_error_hor_m_gnss(nel_sec);
    for (unsigned long i = 0; i != nel_sec; ++i) {
        WWerror_hor_m[i].resize(nel);
        WWerror_hor_m_gnss[i].resize(nel);
        for (unsigned long j = 0; j != nel; ++j) {
            WWerror_hor_m[i][j]      = VVerror_hor_m[j][i];
            WWerror_hor_m_gnss[i][j] = ZZerror_hor_m[j][i];
        }
        Wmean_error_hor_m[i]      = math::mean(WWerror_hor_m[i]);
        Wstd_error_hor_m[i]       = math::std(WWerror_hor_m[i], Wmean_error_hor_m[i]);
        Wmean_error_hor_m_gnss[i] = math::mean(WWerror_hor_m_gnss[i]);
        Wstd_error_hor_m_gnss[i]  = math::std(WWerror_hor_m_gnss[i], Wmean_error_hor_m_gnss[i]);
    }

    boost::filesystem::path path_folder(path_outputs_write / st_folder_main / st_extra_folder);
    boost::filesystem::create_directory(path_folder);

    std::string st_file_out = "versus_gnss_pos_hor_m_pc.txt";
    std::string st_file_output = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out).string();

    ofstream Oout;
    Oout.open(st_file_output);

    for (unsigned long i = 0; i != nel_sec; ++i) {
        Oout << fixed      << setw(8)  << setprecision(1) << showpos << Vt_sec[i]
             << scientific << setw(12) << setprecision(3) << showpos << 1e-3 * Wmean_error_hor_m[i]
             << scientific << setw(12) << setprecision(3) << showpos << 1e-3 * (Wmean_error_hor_m[i] + Wstd_error_hor_m[i])
             << scientific << setw(12) << setprecision(3) << showpos << 1e-3 * (Wmean_error_hor_m[i] - Wstd_error_hor_m[i])
             << scientific << setw(12) << setprecision(3) << showpos << 1e-3 * Wmean_error_hor_m_gnss[i]
             << scientific << setw(12) << setprecision(3) << showpos << 1e-3 * (Wmean_error_hor_m_gnss[i] + Wstd_error_hor_m_gnss[i])
             << scientific << setw(12) << setprecision(3) << showpos << 1e-3 * (Wmean_error_hor_m_gnss[i] - Wstd_error_hor_m_gnss[i]);
        Oout << endl;
    }
    Oout.close();

    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    std::string st_file_out_tex = "versus_gnss_pos_hor_table.tex";
    std::string st_file_output_tex = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_tex).string();

    ofstream Oout_tex;
    Oout_tex.open(st_file_output_tex);

    Oout_tex << "\\begin{center}" << std::endl;
    Oout_tex << "\\begin{tabular}{lrrp{0.5cm}rr}" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\textbf{Benchmark} & \\multicolumn{2}{c}{\\textbf{Baseline}} & & \\multicolumn{2}{c}{\\textbf{GNSS-Based}} \\\\" << std::endl;
    Oout_tex << " & Distance \\nm{\\lrsb{m}} \\footnotemark & \\nm{\\Deltaxhor \\lrsb{m}} & & Distance \\nm{\\lrsb{m}} \\footnotemark & \\nm{\\Deltaxhor \\lrsb{m}}  \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\nm{\\muEND{\\Deltaxhor}} "
             << " & "
             << fixed << setw(11) << setprecision(1) << noshowpos << mean_dist_hor_m_end
             << " & \\textbf{"
             << fixed << setw(11) << setprecision(1) << noshowpos << mean_error_hor_m_end
             << "} & & "
             << fixed << setw(11) << setprecision(1) << noshowpos << mean_dist_hor_m_end_gnss
             << " & \\textbf{"
             << fixed << setw(11) << setprecision(1) << noshowpos << mean_error_hor_m_end_gnss
             << "} \\\\" << std::endl;
    Oout_tex << "\\nm{\\sigmaEND{\\Deltaxhor}} "
             << " & "
             << fixed << setw(11) << setprecision(1) << noshowpos << std_dist_hor_m_end
             << " & "
             << fixed << setw(11) << setprecision(1) << noshowpos << std_error_hor_m_end
             << " & & "
             << fixed << setw(11) << setprecision(1) << noshowpos << std_dist_hor_m_end_gnss
             << " & "
             << fixed << setw(11) << setprecision(1) << noshowpos << std_error_hor_m_end_gnss
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\maxEND{\\Deltaxhor}} "
             << " & "
             << fixed << setw(11) << setprecision(1) << noshowpos << smax_dist_hor_m_end
             << " & "
             << fixed << setw(11) << setprecision(1) << noshowpos << smax_error_hor_m_end
             << " & & "
             << fixed << setw(11) << setprecision(1) << noshowpos << smax_dist_hor_m_end_gnss
             << " & "
             << fixed << setw(11) << setprecision(1) << noshowpos << smax_error_hor_m_end_gnss
             << " \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\end{tabular}" << std::endl;
    Oout_tex << "\\end{center}" << std::endl;
    Oout_tex.close();

}
/* reads the final horizontal position results from two sets of files, the baseline and the one employing GNSS.
 * It shows results on console and also generates the following files (for thesis, not Matlab):
 * - "versus_gnss_pos_hor_m_pc.txt" --> to plot horizontal error mean and std variation with time
 * - "versus_gnss_pos_h.tex" --> script to directly generate table in Latex */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////




























