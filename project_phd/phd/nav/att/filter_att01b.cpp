#include "filter_att01.h"
#include "ang/tools.h"
#include "env/geo.h"
#include "ang/rotate/dcm.h"
#include "ang/rotate/rotv.h"
#include "acft/st/trj_sens_in.h"
#include "acft/st/trj_nav_out.h"
#include "acft/st/trj_truth.h"
#include "acft/st/sti.h"
#include <fstream>
#include <iostream>

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER ATTITUDE01
// =======================
// =======================

using namespace std;

void nav::filter_att01::textplot(const boost::filesystem::path& path_folder, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out) const {
    boost::filesystem::path path_file_att_qnb     ("filter_att_qnb.txt");
    boost::filesystem::path path_file_att_euler   ("filter_att_euler.txt");
    boost::filesystem::path path_file_att_wnbb    ("filter_att_wnbb.txt");
    boost::filesystem::path path_file_att_E_gyr   ("filter_att_Egyr.txt");
    boost::filesystem::path path_file_att_E_mag   ("filter_att_Emag.txt");
    boost::filesystem::path path_file_att_obs_gyr ("filter_att_obs_gyr.txt");
    boost::filesystem::path path_file_att_obs_acc ("filter_att_obs_acc.txt");
    boost::filesystem::path path_file_att_obs_mag ("filter_att_obs_mag.txt");

    std::string txt_file_qnb      = (path_folder / path_file_att_qnb).string();
    std::string txt_file_euler    = (path_folder / path_file_att_euler).string();
    std::string txt_file_wnbb     = (path_folder / path_file_att_wnbb).string();
    std::string txt_file_E_gyr    = (path_folder / path_file_att_E_gyr).string();
    std::string txt_file_E_mag    = (path_folder / path_file_att_E_mag).string();
    std::string txt_file_obs_gyr  = (path_folder / path_file_att_obs_gyr).string();
    std::string txt_file_obs_acc  = (path_folder / path_file_att_obs_acc).string();
    std::string txt_file_obs_mag  = (path_folder / path_file_att_obs_mag).string();

    this->textplot_qnb     (txt_file_qnb,      Otrj_sens_in, Otrj_sens_out, Otrj_nav_in, Otrj_nav_out);
    this->textplot_euler   (txt_file_euler,    Otrj_sens_in, Otrj_sens_out, Otrj_nav_in, Otrj_nav_out);
    this->textplot_wnbb    (txt_file_wnbb,     Otrj_sens_in, Otrj_sens_out, Otrj_nav_in, Otrj_nav_out);
    this->textplot_E_gyr   (txt_file_E_gyr,    Otrj_sens_in, Otrj_sens_out, Otrj_nav_in, Otrj_nav_out);
    this->textplot_E_mag   (txt_file_E_mag,    Otrj_sens_in, Otrj_sens_out, Otrj_nav_in, Otrj_nav_out);
    //this->textplot_obs_gyr (txt_file_obs_gyr,  Otrj_sens_in, Otrj_sens_out, Otrj_nav_in, Otrj_nav_out);
    //this->textplot_obs_acc (txt_file_obs_acc,  Otrj_sens_in, Otrj_sens_out, Otrj_nav_in, Otrj_nav_out);
    //this->textplot_obs_mag (txt_file_obs_mag,  Otrj_sens_in, Otrj_sens_out, Otrj_nav_in, Otrj_nav_out);
}
/* creates text files describing the filter performance */

void nav::filter_att01::textplot_qnb(const std::string& txt_file_qnb, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out) const {
    double r2d = math::constant::R2D();

    ofstream Oout_qnb;
    Oout_qnb.open(txt_file_qnb);

    ang::rodrigues q_nb_apriori;
    Oout_qnb << setw(10) << "t_sec"
             << setw(40) << "q_nb +" << setw(7) << "|q_nb+|"
             << setw(40) << "q_nb -" << setw(7) << "|q_nb-|"
             << setw(40) << "q_nb T"
             << setw(52) << "q_nb error std +"
             << setw(52) << "q_nb error std -"
             << setw(52) << "q_nb error avg +"
             << setw(52) << "q_nb error avg -"
             << endl;

    for (unsigned int s = 0; s != (Otrj_sens_in.get_nel()); ++s) {
        q_nb_apriori = _Pekf_handler->get_xhat_bef()[s].segment<4>(0);
        Oout_qnb << fixed << setw(10) << setprecision(3) << showpos << Otrj_nav_out()[s].get_t_sec()                      // t_sec
                 << fixed << setw(10) << setprecision(6) << showpos << Otrj_nav_out()[s].get_q_nb()[0]                    // q_nb 0 estimated (a posteriori)
                 << fixed << setw(10) << setprecision(6) << showpos << Otrj_nav_out()[s].get_q_nb()[1]                    // q_nb 1
                 << fixed << setw(10) << setprecision(6) << showpos << Otrj_nav_out()[s].get_q_nb()[2]                    // q_nb 2
                 << fixed << setw(10) << setprecision(6) << showpos << Otrj_nav_out()[s].get_q_nb()[3]                    // q_nb 3
                 << fixed << setw(7) << setprecision(3) << showpos << Otrj_nav_out()[s].get_q_nb().norm()                 // q_nb norm estimated (a posteriori)
                 << fixed << setw(10) << setprecision(6) << showpos << q_nb_apriori(0)                                    // q_nb 0 estimated (a priori)
                 << fixed << setw(10) << setprecision(6) << showpos << q_nb_apriori(1)                                    // q_nb 1
                 << fixed << setw(10) << setprecision(6) << showpos << q_nb_apriori(2)                                    // q_nb 2
                 << fixed << setw(10) << setprecision(6) << showpos << q_nb_apriori(3)                                    // q_nb 3
                 << fixed << setw(7) << setprecision(3) << showpos << q_nb_apriori.norm()                                 // q_nb norm estimated (a priori)
                 << fixed << setw(10) << setprecision(6) << showpos << Otrj_nav_in()[s].get_q_nb()[0]                     // q_nb 0 truth
                 << fixed << setw(10) << setprecision(6) << showpos << Otrj_nav_in()[s].get_q_nb()[1]                     // q_nb 1
                 << fixed << setw(10) << setprecision(6) << showpos << Otrj_nav_in()[s].get_q_nb()[2]                     // q_nb 2
                 << fixed << setw(10) << setprecision(6) << showpos << Otrj_nav_in()[s].get_q_nb()[3]                     // q_nb 3
                 << scientific << setw(13) << setprecision(4) << showpos << sqrt(_Pekf_handler->get_P_aft()[s](0, 0))     // a posteriori q_nb 0 standard deviation
                 << scientific << setw(13) << setprecision(4) << showpos << sqrt(_Pekf_handler->get_P_aft()[s](1, 1))     // a posteriori q_nb 1 standard deviation
                 << scientific << setw(13) << setprecision(4) << showpos << sqrt(_Pekf_handler->get_P_aft()[s](2, 2))     // a posteriori q_nb 2 standard deviation
                 << scientific << setw(13) << setprecision(4) << showpos << sqrt(_Pekf_handler->get_P_aft()[s](3, 3))     // a posteriori q_nb 3 standard deviation
                 << scientific << setw(13) << setprecision(4) << showpos << sqrt(_Pekf_handler->get_P_bef()[s](0, 0))     // a priori q_nb 0 standard deviation
                 << scientific << setw(13) << setprecision(4) << showpos << sqrt(_Pekf_handler->get_P_bef()[s](1, 1))     // a priori q_nb 1 standard deviation
                 << scientific << setw(13) << setprecision(4) << showpos << sqrt(_Pekf_handler->get_P_bef()[s](2, 2))     // a priori q_nb 2 standard deviation
                 << scientific << setw(13) << setprecision(4) << showpos << sqrt(_Pekf_handler->get_P_bef()[s](3, 3))     // a priori q_nb 3 standard deviation
                 << scientific << setw(13) << setprecision(4) << showpos << _Pekf_handler->get_eps_xhat_aft_mean()[s](0)  // running mean of a posteriori q_nb 0
                 << scientific << setw(13) << setprecision(4) << showpos << _Pekf_handler->get_eps_xhat_aft_mean()[s](1)  // running mean of a posteriori q_nb 1
                 << scientific << setw(13) << setprecision(4) << showpos << _Pekf_handler->get_eps_xhat_aft_mean()[s](2)  // running mean of a posteriori q_nb 2
                 << scientific << setw(13) << setprecision(4) << showpos << _Pekf_handler->get_eps_xhat_aft_mean()[s](3)  // running mean of a posteriori q_nb 3
                 << scientific << setw(13) << setprecision(4) << showpos << _Pekf_handler->get_eps_xhat_bef_mean()[s](0)  // running mean of a priori q_nb 0
                 << scientific << setw(13) << setprecision(4) << showpos << _Pekf_handler->get_eps_xhat_bef_mean()[s](1)  // running mean of a priori q_nb 1
                 << scientific << setw(13) << setprecision(4) << showpos << _Pekf_handler->get_eps_xhat_bef_mean()[s](2)  // running mean of a priori q_nb 2
                 << scientific << setw(13) << setprecision(4) << showpos << _Pekf_handler->get_eps_xhat_bef_mean()[s](3)  // running mean of a priori q_nb 3
                 << endl;
    }
    Oout_qnb.close();
}
/* creates text file describing the evolution of q_nb */

void nav::filter_att01::textplot_euler(const std::string& txt_file_euler, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out) const {
    double r2d = math::constant::R2D();

    ofstream Oout_euler;
    Oout_euler.open(txt_file_euler);
    ang::rodrigues q_nb_apriori;
    ang::euler euler_nedbfs_rad_truth, euler_nedbfs_rad_nav, euler_nedbfs_rad_nav_apriori;
    ang::rotv rv_btruthnav_nav, rv_btruthnav_apriori;
    Oout_euler << setw(10) << "t_sec"
               << setw(30) << "euler_deg +"   << setw(30) << "euler_deg -"    << setw(30) << "euler_deg T"
               << setw(10) << "angle_deg +"   << setw(10) << "angle_deg -"
               << endl;

    for (unsigned int s = 0; s != (Otrj_sens_in.get_nel()); ++s) {
        euler_nedbfs_rad_nav   = Otrj_nav_out()[s].get_q_nb();
        euler_nedbfs_rad_truth = Otrj_nav_in()[s].get_q_nb();
        q_nb_apriori = _Pekf_handler->get_xhat_bef()[s].segment<4>(0);
        euler_nedbfs_rad_nav_apriori = q_nb_apriori;
        rv_btruthnav_nav     = Otrj_nav_in()[s].get_q_nb() / Otrj_nav_out()[s].get_q_nb();
        rv_btruthnav_apriori = Otrj_nav_in()[s].get_q_nb() / q_nb_apriori;

        Oout_euler << fixed << setw(10) << setprecision(3) << showpos << Otrj_nav_out()[s].get_t_sec()                       // t_sec
                   << fixed << setw(10) << setprecision(4) << showpos << euler_nedbfs_rad_nav.get_yaw_rad() * r2d            // psi_deg estimated (a posteriori)
                   << fixed << setw(10) << setprecision(4) << showpos << euler_nedbfs_rad_nav.get_pitch_rad() * r2d          // theta_deg
                   << fixed << setw(10) << setprecision(4) << showpos << euler_nedbfs_rad_nav.get_bank_rad() * r2d           // xi_deg
                   << fixed << setw(10) << setprecision(4) << showpos << euler_nedbfs_rad_nav_apriori.get_yaw_rad() * r2d    // psi_deg estimated (a priori)
                   << fixed << setw(10) << setprecision(4) << showpos << euler_nedbfs_rad_nav_apriori.get_pitch_rad() * r2d  // theta_deg
                   << fixed << setw(10) << setprecision(4) << showpos << euler_nedbfs_rad_nav_apriori.get_bank_rad() * r2d   // xi_deg
                   << fixed << setw(10) << setprecision(4) << showpos << euler_nedbfs_rad_truth.get_yaw_rad() * r2d          // psi_deg truth
                   << fixed << setw(10) << setprecision(4) << showpos << euler_nedbfs_rad_truth.get_pitch_rad() * r2d        // theta_deg
                   << fixed << setw(10) << setprecision(4) << showpos << euler_nedbfs_rad_truth.get_bank_rad() * r2d         // xi_deg
                   << fixed << setw(10) << setprecision(4) << showpos << rv_btruthnav_nav.norm() * r2d                       // angle error_deg (a posteriori)
                   << fixed << setw(10) << setprecision(4) << showpos << rv_btruthnav_apriori.norm() * r2d                   // angle error_deg (a priori)
                   << endl;
    }
    Oout_euler.close();
}
/* creates text file describing the evolution of the euler angles */

void nav::filter_att01::textplot_wnbb(const std::string& txt_file_wnbb, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out) const {
    double r2d = math::constant::R2D();

    ofstream Oout_wnbb;
    Oout_wnbb.open(txt_file_wnbb);

    Oout_wnbb << setw(10) << "t_sec"
              << setw(42) << "w_nbb_dps +" << setw(42) << "w_nbb_dps -" << setw(42) << "w_nbb_dps T"
              << setw(39) << "w_nbb_dps error std +"
              << setw(39) << "w_nbb_dps error std -"
              << setw(39) << "w_nbb_dps error avg +"
              << setw(39) << "w_nbb_dps error avg -"
              << endl;

    for (unsigned int s = 0; s != (Otrj_sens_in.get_nel()); ++s) {
        Oout_wnbb << fixed << setw(10) << setprecision(3) << showpos << Otrj_nav_out()[s].get_t_sec()                             // t_sec
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_out()[s].get_w_nbb_rps()(0) * r2d           // w_nbb_dps 0 estimated (a posteriori)
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_out()[s].get_w_nbb_rps()(1) * r2d           // w_nbb_dps 1
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_out()[s].get_w_nbb_rps()(2) * r2d           // w_nbb_dps 2
                  << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_xhat_bef()[s](4) * r2d            // w_nbb_dps 0 estimated (a priori)
                  << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_xhat_bef()[s](5) * r2d            // w_nbb_dps 1
                  << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_xhat_bef()[s](6) * r2d            // w_nbb_dps 2
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_in()[s].get_w_nbb_rps()(0) * r2d            // w_nbb_dps 0 truth
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_in()[s].get_w_nbb_rps()(1) * r2d            // w_nbb_dps 1
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_in()[s].get_w_nbb_rps()(2) * r2d            // w_nbb_dps 2
                  << scientific << setw(13) << setprecision(4) << showpos << sqrt(_Pekf_handler->get_P_aft()[s](4,4)) * r2d       // a posteriori w_nbb_dps 0 standard deviation
                  << scientific << setw(13) << setprecision(4) << showpos << sqrt(_Pekf_handler->get_P_aft()[s](5,5)) * r2d       // a posteriori w_nbb_dps standard deviation
                  << scientific << setw(13) << setprecision(4) << showpos << sqrt(_Pekf_handler->get_P_aft()[s](6,6)) * r2d       // a posteriori w_nbb_dps standard deviation
                  << scientific << setw(13) << setprecision(4) << showpos << sqrt(_Pekf_handler->get_P_bef()[s](4,4)) * r2d       // a priori w_nbb_dps 0 standard deviation
                  << scientific << setw(13) << setprecision(4) << showpos << sqrt(_Pekf_handler->get_P_bef()[s](5,5)) * r2d       // a priori w_nbb_dps 1 standard deviation
                  << scientific << setw(13) << setprecision(4) << showpos << sqrt(_Pekf_handler->get_P_bef()[s](6,6)) * r2d       // a priori w_nbb_dps 2 standard deviation
                  << scientific << setw(13) << setprecision(4) << showpos << _Pekf_handler->get_eps_xhat_aft_mean()[s](4) * r2d   // running mean of a posteriori w_nbb_dps 0
                  << scientific << setw(13) << setprecision(4) << showpos << _Pekf_handler->get_eps_xhat_aft_mean()[s](5) * r2d   // running mean of a posteriori w_nbb_dps 1
                  << scientific << setw(13) << setprecision(4) << showpos << _Pekf_handler->get_eps_xhat_aft_mean()[s](6) * r2d   // running mean of a posteriori w_nbb_dps 2
                  << scientific << setw(13) << setprecision(4) << showpos << _Pekf_handler->get_eps_xhat_bef_mean()[s](4) * r2d   // running mean of a priori w_nbb_dps 0
                  << scientific << setw(13) << setprecision(4) << showpos << _Pekf_handler->get_eps_xhat_bef_mean()[s](5) * r2d   // running mean of a priori w_nbb_dps 1
                  << scientific << setw(13) << setprecision(4) << showpos << _Pekf_handler->get_eps_xhat_bef_mean()[s](6) * r2d   // running mean of a priori w_nbb_dps 2
                  << endl;
    }
    Oout_wnbb.close();
}
/* creates text file describing the evolution of w_nbb */

void nav::filter_att01::textplot_E_gyr(const std::string& txt_file_E_gyr, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out) const {
    double r2d = math::constant::R2D();

    ofstream Oout_E_gyr;
    Oout_E_gyr.open(txt_file_E_gyr);

    Oout_E_gyr << setw(10) << "t_sec"
                  << setw(42) << "E_gyr_dps +"      << setw(42) << "E_gyr_dps -"     << setw(42) << "E_gyr_dps T"
                  << setw(42) << "E_gyr_dps error std +"
                  << setw(42) << "E_gyr_dps error std -"
                  << setw(42) << "E_gyr_dps error avg +"
                  << setw(42) << "E_gyr_dps error avg -"
                  << endl;

    for (unsigned int s = 0; s != (Otrj_sens_in.get_nel()); ++s) {
        Oout_E_gyr << fixed    << setw(10) << setprecision(3) << showpos << Otrj_nav_out()[s].get_t_sec()                             // t_sec
                      << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_out()[s].get_E_gyr_rps()[0] * r2d           // E_gyr0_dps estimated (a posteriori)
                      << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_out()[s].get_E_gyr_rps()[1] * r2d           // E_gyr1_dps estimated (a posteriori)
                      << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_out()[s].get_E_gyr_rps()[2] * r2d           // E_gyr2_dps estimated (a posteriori)
                      << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_xhat_bef()[s](7) * r2d            // E_gyr0_dps estimated (a priori)
                      << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_xhat_bef()[s](8) * r2d            // E_gyr1_dps estimated (a priori)
                      << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_xhat_bef()[s](9) * r2d            // E_gyr2_dps estimated (a priori)
                      << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_out()[s].get_E_gyr_rps()(0) * r2d          // E_gyr0_dps truth
                      << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_out()[s].get_E_gyr_rps()(1) * r2d          // E_gyr1_dps truth
                      << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_out()[s].get_E_gyr_rps()(2) * r2d          // E_gyr2_dps truth
                      << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekf_handler->get_P_aft()[s](7,7)) * r2d       // a posteriori E_gyr0_dps standard deviation
                      << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekf_handler->get_P_aft()[s](8,8)) * r2d       // a posteriori E_gyr1_dps standard deviation
                      << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekf_handler->get_P_aft()[s](9,9)) * r2d       // a posteriori E_gyr2_dps standard deviation
                      << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekf_handler->get_P_bef()[s](7,7)) * r2d       // a priori E_gyr0_dps standard deviation
                      << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekf_handler->get_P_bef()[s](8,8)) * r2d       // a priori E_gyr1_dps standard deviation
                      << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekf_handler->get_P_bef()[s](9,9)) * r2d       // a priori E_gyr2_dps standard deviation
                      << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_eps_xhat_aft_mean()[s](7) * r2d   // running mean of a posteriori E_gyr0_dps
                      << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_eps_xhat_aft_mean()[s](8) * r2d   // running mean of a posteriori E_gyr1_dps
                      << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_eps_xhat_aft_mean()[s](9) * r2d   // running mean of a posteriori E_gyr2_dps
                      << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_eps_xhat_bef_mean()[s](7) * r2d   // running mean of a priori E_gyr0_dps
                      << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_eps_xhat_bef_mean()[s](8) * r2d   // running mean of a priori E_gyr1_dps
                      << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_eps_xhat_bef_mean()[s](9) * r2d   // running mean of a priori E_gyr2_dps
                      << endl;
    }
    Oout_E_gyr.close();
}
/* creates text file describing the evolution of E_gyr */

void nav::filter_att01::textplot_E_mag(const std::string& txt_file_E_mag, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out) const {
    ofstream Oout_E_mag;
    Oout_E_mag.open(txt_file_E_mag);

    Oout_E_mag << setw(10) << "t_sec"
                  << setw(42) << "E_mag_nT +"      << setw(42) << "E_mag_nT -"     << setw(42) << "E_mag_nT T"
                  << setw(42) << "E_mag_nT error std +"
                  << setw(42) << "E_mag_nT error std -"
                  << setw(42) << "E_mag_nT error avg +"
                  << setw(42) << "E_mag_nT error avg -"
                  << endl;

    for (unsigned int s = 0; s != (Otrj_sens_in.get_nel()); ++s) {
        Oout_E_mag << fixed    << setw(10) << setprecision(3) << showpos << Otrj_nav_out()[s].get_t_sec()                      // t_sec
                      << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_out()[s].get_E_mag_nT()[0]           // E_mag0_nT estimated (a posteriori)
                      << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_out()[s].get_E_mag_nT()[1]           // E_mag1_nT estimated (a posteriori)
                      << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_out()[s].get_E_mag_nT()[2]           // E_mag2_nT estimated (a posteriori)
                      << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_xhat_bef()[s](10)          // E_mag0_nT estimated (a priori)
                      << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_xhat_bef()[s](11)          // E_mag1_nT estimated (a priori)
                      << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_xhat_bef()[s](12)          // E_mag2_nT estimated (a priori)
                      << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_out()[s].get_E_mag_nT()(0)          // E_mag0_nT truth
                      << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_out()[s].get_E_mag_nT()(1)          // E_mag1_nT truth
                      << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_out()[s].get_E_mag_nT()(2)          // E_mag2_nT truth
                      << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekf_handler->get_P_aft()[s](10,10))    // a posteriori E_mag0_nT standard deviation
                      << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekf_handler->get_P_aft()[s](11,11))    // a posteriori E_mag1_nT standard deviation
                      << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekf_handler->get_P_aft()[s](12,12))    // a posteriori E_mag2_nT standard deviation
                      << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekf_handler->get_P_bef()[s](10,10))    // a priori E_mag0_nT standard deviation
                      << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekf_handler->get_P_bef()[s](11,11))    // a priori E_mag1_nT standard deviation
                      << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekf_handler->get_P_bef()[s](12,12))    // a priori E_mag2_nT standard deviation
                      << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_eps_xhat_aft_mean()[s](10) // running mean of a posteriori E_mag0_nT
                      << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_eps_xhat_aft_mean()[s](11) // running mean of a posteriori E_mag1_nT
                      << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_eps_xhat_aft_mean()[s](12) // running mean of a posteriori E_mag2_nT
                      << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_eps_xhat_bef_mean()[s](10) // running mean of a priori E_mag0_nT
                      << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_eps_xhat_bef_mean()[s](11) // running mean of a priori E_mag1_nT
                      << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_eps_xhat_bef_mean()[s](12) // running mean of a priori E_mag2_nT
                      << endl;
    }
    Oout_E_mag.close();
}
/* creates text file describing the evolution of E_mag */

void nav::filter_att01::textplot_obs_gyr(const std::string& txt_file_obs_gyr, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out) const {
    double r2d = math::constant::R2D();

    ofstream Oout_obs_gyr;
    Oout_obs_gyr.open(txt_file_obs_gyr);

    Oout_obs_gyr << setw(10) << "t_sec"
                 << setw(42) << "w_ibb_dps truth"
                 << setw(42) << "w_ibb_dps sensed"
                 << setw(42) << "w_ibb_dps innov"
                 << setw(42) << "w_ibb_dps innov std"
                 << setw(42) << "w_ibb_dps_innov avg"
                 << endl;

    for (unsigned int s = 0; s != (Otrj_sens_in.get_nel()); ++s) {
        Oout_obs_gyr << fixed << setw(10) << setprecision(3) << showpos << Otrj_nav_out()[s].get_t_sec()                     // t_sec
                     << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_in()[s].get_w_ibb_rps()[0] * r2d   // w_ibb_dps 1 real
                     << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_in()[s].get_w_ibb_rps()[1] * r2d   // w_ibb_dps 2 real
                     << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_in()[s].get_w_ibb_rps()[2] * r2d   // w_ibb_dps 3 real
                     << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_out()[s].get_w_ibb_rps()[0] * r2d  // w_ibb_dps 1 sensed
                     << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_out()[s].get_w_ibb_rps()[1] * r2d  // w_ibb_dps 2 sensed
                     << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_out()[s].get_w_ibb_rps()[2] * r2d  // w_ibb_dps 3 sensed
                     << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_r()[s][0] * r2d           // w_ibb_dps 1 innovation
                     << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_r()[s][1] * r2d           // w_ibb_dps 2 innovation
                     << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_r()[s][2] * r2d           // w_ibb_dps 3 innovation
                     << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekf_handler->get_S()[s](0,0)) * r2d   // w_ibb_dps 1 innovation standard deviation
                     << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekf_handler->get_S()[s](1,1)) * r2d   // w_ibb_dps 2 innovation standard deviation
                     << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekf_handler->get_S()[s](2,2)) * r2d   // w_ibb_dps 3 innovation standard deviation
                     << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_r_mean()[s][0] * r2d      // w_ibb_dps 1 innovation_mean
                     << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_r_mean()[s][1] * r2d      // w_ibb_dps 2 innovation_mean
                     << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_r_mean()[s][2] * r2d      // w_ibb_dps 3 innovation_mean
                     << endl;
    }
    Oout_obs_gyr.close();
}
/* creates text file describing the evolution of gyroscope observations */

void nav::filter_att01::textplot_obs_acc(const std::string& txt_file_obs_acc, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out) const {
    ofstream Oout_obs_acc;
    Oout_obs_acc.open(txt_file_obs_acc);

    Oout_obs_acc << setw(10) << "t_sec"
                 << setw(42) << "f_ibb_mps2 truth"
                 << setw(42) << "f_ibb_mps2 sensed"
                 << setw(42) << "f_ibb_mps2 innov"
                 << setw(42) << "f_ibb_mps2 innov std"
                 << setw(42) << "f_ibb_mps2_innov avg"
                 << endl;

    for (unsigned int s = 0; s != (Otrj_sens_in.get_nel()); ++s) {
        Oout_obs_acc << fixed << setw(10) << setprecision(3) << showpos << Otrj_nav_out()[s].get_t_sec()               // t_sec
                     << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_in()[s].get_f_ibb_mps2()[0]  // f1_ibb_mps2 real
                     << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_in()[s].get_f_ibb_mps2()[1]  // f2_ibb_mps2 real
                     << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_in()[s].get_f_ibb_mps2()[2]  // f3_ibb_mps2 rea
                     << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_out()[s].get_f_ibb_mps2()[0] // f1_ibb_mps2 sensed
                     << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_out()[s].get_f_ibb_mps2()[1] // f2_ibb_mps2 sensed
                     << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_out()[s].get_f_ibb_mps2()[2] // f3_ibb_mps2 sensed
                     << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_r()[s][3]           // f1_ibb_mps2 innovation
                     << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_r()[s][4]           // f2_ibb_mps2 innovation
                     << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_r()[s][5]           // f3_ibb_mps2 innovation
                     << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekf_handler->get_S()[s](3,3))   // f1_ibb_mps2 innovation standard deviation
                     << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekf_handler->get_S()[s](4,4))   // f2_ibb_mps2 innovation standard deviation
                     << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekf_handler->get_S()[s](5,5))   // f3_ibb_mps2 innovation standard deviation
                     << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_r_mean()[s][3]      // f1_ibb_mps2 innovation_mean
                     << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_r_mean()[s][4]      // f2_ibb_mps2 innovation_mean
                     << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_r_mean()[s][5]      // f3_ibb_mps2 innovation_mean
                     << endl;
    }
    Oout_obs_acc.close();
}
/* creates text file describing the evolution of accelerometer observations */

void nav::filter_att01::textplot_obs_mag(const std::string& txt_file_obs_mag, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out) const {
    ofstream Oout_obs_mag;
    Oout_obs_mag.open(txt_file_obs_mag);

    Oout_obs_mag << setw(10) << "t_sec"
                 << setw(42) << "B_b_nT truth"
                 << setw(42) << "B_b_nT sensed"
                 << setw(42) << "B_b_nT innov"
                 << setw(42) << "B_b_nT innov std"
                 << setw(42) << "B_b_nT_innov avg"
                 << endl;

    for (unsigned int s = 0; s != (Otrj_sens_in.get_nel()); ++s) {
        Oout_obs_mag << fixed << setw(10) << setprecision(3) << showpos << Otrj_nav_out()[s].get_t_sec()               // t_sec
                     << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_in()[s].get_B_b_nT()[0]      // B1_b_nT real
                     << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_in()[s].get_B_b_nT()[1]      // B2_b_nT real
                     << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_in()[s].get_B_b_nT()[2]      // B3_b_nT real
                     << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_out()[s].get_B_b_nT()[0]     // B1_b_nT sensed
                     << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_out()[s].get_B_b_nT()[1]     // B2_b_nT sensed
                     << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_out()[s].get_B_b_nT()[2]     // B3_b_nT sensed
                     << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_r()[s][6]           // B1_b_nT innovation
                     << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_r()[s][7]           // B2_b_nT innovation
                     << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_r()[s][8]           // B3_b_nT innovation
                     << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekf_handler->get_S()[s](6,6))   // B1_b_nT innovation standard deviation
                     << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekf_handler->get_S()[s](7,7))   // B2_b_nT innovation standard deviation
                     << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekf_handler->get_S()[s](8,8))   // B3_b_nT innovation standard deviation
                     << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_r_mean()[s][6]      // B1_b_nT innovation_mean
                     << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_r_mean()[s][7]      // B2_b_nT innovation_mean
                     << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_r_mean()[s][8]      // B3_b_nT innovation_mean
                     << endl;
    }
    Oout_obs_mag.close();
}
/* creates text file describing the evolution of magnetometer observations */

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////







