#include "filter_gps01.h"
#include "ang/tools.h"
#include "env/geo.h"
#include "ang/rotate/dcm.h"
#include "ang/rotate/rotv.h"
#include "env/speed.h"
#include "acft/st/trj_sens_in.h"
#include "acft/st/trj_nav_out.h"
#include "acft/st/trj_truth.h"
#include "acft/st/sti.h"
#include <fstream>
#include <iostream>
#include <cmath>

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER GPS01
// ==================
// ==================

using namespace std;

void nav::filter_gps01::textplot(const boost::filesystem::path& path_folder, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out, const st::trj_gps_out& Otrj_gps_out) const {
    boost::filesystem::path path_file_gps_xgdt        ("filter_gps_xgdt.txt");
    boost::filesystem::path path_file_gps_vned        ("filter_gps_vned.txt");
    boost::filesystem::path path_file_gps_fibb        ("filter_gps_fibb.txt");
    boost::filesystem::path path_file_gps_Eacc        ("filter_gps_Eacc.txt");
    boost::filesystem::path path_file_gps_obs_acc     ("filter_gps_obs_acc.txt");
    boost::filesystem::path path_file_gps_obs_vned    ("filter_gps_obs_vned.txt");
    boost::filesystem::path path_file_gps_obs_xgdt    ("filter_gps_obs_xgdt.txt");
    boost::filesystem::path path_file_gps_wind_DeltaTp("filter_gps_wind_DeltaTp.txt");

    std::string txt_file_xgdt         = (path_folder / path_file_gps_xgdt).string();
    std::string txt_file_vned         = (path_folder / path_file_gps_vned).string();
    std::string txt_file_fibb         = (path_folder / path_file_gps_fibb).string();
    std::string txt_file_Eacc         = (path_folder / path_file_gps_Eacc).string();
    std::string txt_file_obs_acc      = (path_folder / path_file_gps_obs_acc).string();
    std::string txt_file_obs_vned     = (path_folder / path_file_gps_obs_vned).string();
    std::string txt_file_obs_xgdt     = (path_folder / path_file_gps_obs_xgdt).string();
    std::string txt_file_wind_DeltaTp = (path_folder / path_file_gps_wind_DeltaTp).string();

    this->textplot_xgdt        (txt_file_xgdt,         Otrj_sens_in, Otrj_sens_out, Otrj_nav_in, Otrj_nav_out, Otrj_gps_out);
    this->textplot_vned        (txt_file_vned,         Otrj_sens_in, Otrj_sens_out, Otrj_nav_in, Otrj_nav_out, Otrj_gps_out);
    this->textplot_fibb        (txt_file_fibb,         Otrj_sens_in, Otrj_sens_out, Otrj_nav_in, Otrj_nav_out, Otrj_gps_out);
    this->textplot_Eacc        (txt_file_Eacc,         Otrj_sens_in, Otrj_sens_out, Otrj_nav_in, Otrj_nav_out, Otrj_gps_out);
    //this->textplot_obs_acc     (txt_file_obs_acc,      Otrj_sens_in, Otrj_sens_out, Otrj_nav_in, Otrj_nav_out, Otrj_gps_out);
    //this->textplot_obs_vned    (txt_file_obs_vned,     Otrj_sens_in, Otrj_sens_out, Otrj_nav_in, Otrj_nav_out, Otrj_gps_out);
    //this->textplot_obs_xgdt    (txt_file_obs_xgdt,     Otrj_sens_in, Otrj_sens_out, Otrj_nav_in, Otrj_nav_out, Otrj_gps_out);
    this->textplot_wind_DeltaTp(txt_file_wind_DeltaTp, Otrj_sens_in, Otrj_sens_out, Otrj_nav_in, Otrj_nav_out, Otrj_gps_out);
}
/* creates text files describing the filter performance */

void nav::filter_gps01::textplot_xgdt(const std::string& txt_file_xhor, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out, const st::trj_gps_out& Otrj_gps_out) const {
    double r2d = math::constant::R2D();

    ofstream Oout_xgdt;
    Oout_xgdt.open(txt_file_xhor);

    std::vector<env::geodetic_coord> Vx_gdt_rad_m_sensed(_Pekfd_handler->get_size_fast());
    int ratio = Otrj_gps_out.get_quot() / Otrj_sens_in.get_quot();
    double mynan = std::nan("");

    for (unsigned int s = 0, g = 0; s != Vx_gdt_rad_m_sensed.size(); ++s) {
        if (g >= Otrj_gps_out.get_nel()) {
            Vx_gdt_rad_m_sensed[s] = env::geodetic_coord(mynan, mynan, mynan);
        }
        else if (fmod(s, ratio) <= math::constant::EPS()) {
            Vx_gdt_rad_m_sensed[s] = Otrj_gps_out()[g].get_x_gdt_rad_m();
            g++;
        }
        else {
            Vx_gdt_rad_m_sensed[s] = env::geodetic_coord(mynan, mynan, mynan);
        }
    }

    Oout_xgdt << setw(10) << "t_sec"
              << setw(51) << "x_gdt_deg_m +"      << setw(51) << "x_gdt_deg_m -"     << setw(51) << "x_gdt_deg_m truth" << setw(51) << "x_gdt_deg_m sensed"
              << setw(42) << "x_gdt_deg_m error std +"
              << setw(42) << "x_gdt_deg_m error std -"
              << setw(42) << "x_gdt_deg_m error avg +"
              << setw(42) << "x_gdt_deg_m error avg -"
              << endl;

    for (unsigned int s = 0; s != _Pekfd_handler->get_size_fast(); ++s) {
        Oout_xgdt << fixed    << setw(10) << setprecision(3) << showpos << Otrj_nav_out()[s].get_t_sec()                                 // t_sec
                  << scientific << setw(18) << setprecision(9) << showpos << Otrj_nav_out()[s].get_x_gdt_rad_m().get_lambda_rad() * r2d  // lambda_deg estimated (a posteriori)
                  << scientific << setw(18) << setprecision(9) << showpos << Otrj_nav_out()[s].get_x_gdt_rad_m().get_phi_rad() * r2d     // phi_deg    estimated (a posteriori)
                  << scientific << setw(15) << setprecision(6) << showpos << Otrj_nav_out()[s].get_x_gdt_rad_m().get_h_m()               // h_m        estimated (a posteriori)
                  << scientific << setw(18) << setprecision(9) << showpos << _Pekfd_handler->get_xhat_bef()[s](0) * r2d                  // lambda_deg estimated (a priori)
                  << scientific << setw(18) << setprecision(9) << showpos << _Pekfd_handler->get_xhat_bef()[s](1) * r2d                  // phi_deg    estimated (a priori)
                  << scientific << setw(15) << setprecision(6) << showpos << _Pekfd_handler->get_xhat_bef()[s](2)                        // h_m        estimated (a priori)
                  << scientific << setw(18) << setprecision(9) << showpos << Otrj_nav_in()[s].get_x_gdt_rad_m().get_lambda_rad() * r2d   // lambda_deg truth
                  << scientific << setw(18) << setprecision(9) << showpos << Otrj_nav_in()[s].get_x_gdt_rad_m().get_phi_rad() * r2d      // phi_deg    truth
                  << scientific << setw(15) << setprecision(6) << showpos << Otrj_nav_in()[s].get_x_gdt_rad_m().get_h_m()                // h_m        truth
                  << scientific << setw(18) << setprecision(9) << showpos << Vx_gdt_rad_m_sensed[s].get_lambda_rad() * r2d               // lambda_deg sensed
                  << scientific << setw(18) << setprecision(9) << showpos << Vx_gdt_rad_m_sensed[s].get_phi_rad() * r2d                  // phi_deg    sensed
                  << scientific << setw(15) << setprecision(6) << showpos << Vx_gdt_rad_m_sensed[s].get_h_m()                            // h_m        sensed
                  << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekfd_handler->get_P_aft()[s](0,0)) * r2d             // a posteriori lambda_deg standard deviation
                  << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekfd_handler->get_P_aft()[s](1,1)) * r2d             // a posteriori phi_deg    standard deviation
                  << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekfd_handler->get_P_aft()[s](2,2))                   // a posteriori h_m        standard deviation
                  << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekfd_handler->get_P_bef()[s](0,0)) * r2d             // a priori lambda_deg standard deviation
                  << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekfd_handler->get_P_bef()[s](1,1)) * r2d             // a priori phi_deg    standard deviation
                  << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekfd_handler->get_P_bef()[s](2,2))                   // a priori h_m        standard deviation
                  << scientific << setw(14) << setprecision(5) << showpos << _Pekfd_handler->get_eps_xhat_aft_mean()[s](0) * r2d         // running mean of a posteriori lambda_deg
                  << scientific << setw(14) << setprecision(5) << showpos << _Pekfd_handler->get_eps_xhat_aft_mean()[s](1) * r2d         // running mean of a posteriori phi_deg
                  << scientific << setw(14) << setprecision(5) << showpos << _Pekfd_handler->get_eps_xhat_aft_mean()[s](2)               // running mean of a posteriori h_m
                  << scientific << setw(14) << setprecision(5) << showpos << _Pekfd_handler->get_eps_xhat_bef_mean()[s](0) * r2d         // running mean of a priori lambda_deg
                  << scientific << setw(14) << setprecision(5) << showpos << _Pekfd_handler->get_eps_xhat_bef_mean()[s](1) * r2d         // running mean of a priori phi_deg
                  << scientific << setw(14) << setprecision(5) << showpos << _Pekfd_handler->get_eps_xhat_bef_mean()[s](2)               // running mean of a priori h_m
                  << endl;
    }
    Oout_xgdt.close();
}
/* creates text file describing the evolution of geodetic position */

void nav::filter_gps01::textplot_vned(const std::string& txt_file_vned, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out, const st::trj_gps_out& Otrj_gps_out) const {

    ofstream Oout_vn;
    Oout_vn.open(txt_file_vned);

    Oout_vn << setw(10) << "t_sec"
            << setw(42) << "v_ned_mps +"      << setw(42) << "v_ned_mps -"     << setw(42) << "v_ned_mps truth" << setw(42) << "v_ned_mps sensed"
            << setw(42) << "v_ned_mps error std +" << setw(42) << "v_ned_mps error std -"
            << setw(42) << "v_ned_mps error avg +" << setw(42) << "v_ned_mps error avg -"
            << endl;

    std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> Vv_ned_mps_sensed(_Pekfd_handler->get_size_fast());
    int ratio = Otrj_gps_out.get_quot() / Otrj_sens_in.get_quot();
    double mynan = std::nan("");

    for (unsigned int s = 0, g = 0; s != Vv_ned_mps_sensed.size(); ++s) {
        if (g >= Otrj_gps_out.get_nel()) {
            Vv_ned_mps_sensed[s] << mynan, mynan, mynan;
        }
        else if (fmod(s, ratio) <= math::constant::EPS()) {
            Vv_ned_mps_sensed[s] = Otrj_gps_out()[g].get_v_n_mps();
            g++;
        }
        else {
            Vv_ned_mps_sensed[s] << mynan, mynan, mynan;
        }
    }

    for (unsigned int s = 0; s != _Pekfd_handler->get_size_fast(); ++s) {
        Oout_vn << fixed    << setw(10) << setprecision(3) << showpos << Otrj_nav_out()[s].get_t_sec()                    // t_sec
                << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_out()[s].get_v_n_mps()[0]             // v_ned0_mps estimated (a posteriori)
                << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_out()[s].get_v_n_mps()[1]             // v_ned1_mps estimated (a posteriori)
                << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_out()[s].get_v_n_mps()[2]             // v_ned2_mps estimated (a posteriori)
                << scientific << setw(14) << setprecision(5) << showpos << _Pekfd_handler->get_xhat_bef()[s](3)           // v_ned0_mps estimated (a priori)
                << scientific << setw(14) << setprecision(5) << showpos << _Pekfd_handler->get_xhat_bef()[s](4)           // v_ned1_mps estimated (a priori)
                << scientific << setw(14) << setprecision(5) << showpos << _Pekfd_handler->get_xhat_bef()[s](5)           // v_ned2_mps estimated (a priori)
                << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_in()[s].get_v_n_mps()(0)              // v_ned0_mps truth
                << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_in()[s].get_v_n_mps()(1)              // v_ned1_mps truth
                << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_in()[s].get_v_n_mps()(2)              // v_ned2_mps truth
                << scientific << setw(14) << setprecision(5) << showpos << Vv_ned_mps_sensed[s](0)                        // v_ned0_mps sensed
                << scientific << setw(14) << setprecision(5) << showpos << Vv_ned_mps_sensed[s](1)                        // v_ned1_mps sensed
                << scientific << setw(14) << setprecision(5) << showpos << Vv_ned_mps_sensed[s](2)                        // v_ned2_mps sensed
                << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekfd_handler->get_P_aft()[s](3,3))      // a posteriori v_ned0_mps standard deviation
                << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekfd_handler->get_P_aft()[s](4,4))      // a posteriori v_ned1_mps standard deviation
                << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekfd_handler->get_P_aft()[s](5,5))      // a posteriori v_ned2_mps standard deviation
                << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekfd_handler->get_P_bef()[s](3,3))      // a priori v_ned0_mps standard deviation
                << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekfd_handler->get_P_bef()[s](4,4))      // a priori v_ned1_mps standard deviation
                << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekfd_handler->get_P_bef()[s](5,5))      // a priori v_ned2_mps standard deviation
                << scientific << setw(14) << setprecision(5) << showpos << _Pekfd_handler->get_eps_xhat_aft_mean()[s](3)  // running mean of a posteriori v_ned0_mps
                << scientific << setw(14) << setprecision(5) << showpos << _Pekfd_handler->get_eps_xhat_aft_mean()[s](4)  // running mean of a posteriori v_ned1_mps
                << scientific << setw(14) << setprecision(5) << showpos << _Pekfd_handler->get_eps_xhat_aft_mean()[s](5)  // running mean of a posteriori v_ned2_mps
                << scientific << setw(14) << setprecision(5) << showpos << _Pekfd_handler->get_eps_xhat_bef_mean()[s](3)  // running mean of a priori v_ned0_mps
                << scientific << setw(14) << setprecision(5) << showpos << _Pekfd_handler->get_eps_xhat_bef_mean()[s](4)  // running mean of a priori v_ned1_mps
                << scientific << setw(14) << setprecision(5) << showpos << _Pekfd_handler->get_eps_xhat_bef_mean()[s](5)  // running mean of a priori v_ned2_mps
                << endl;
    }
    Oout_vn.close();
}
/* creates text file describing the evolution of v_ned */

void nav::filter_gps01::textplot_fibb(const std::string& txt_file_fibb, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out, const st::trj_gps_out& Otrj_gps_out) const {
    ofstream Oout_fibb;
    Oout_fibb.open(txt_file_fibb);

    Oout_fibb << setw(10) << "t_sec"
              << setw(42) << "f_ibb_mps2 +"      << setw(42) << "f_ibb_mps2 -"     << setw(42) << "f_ibb_mps2 truth" << setw(42) << "f_ibb_mps2 sensed"
              << setw(42) << "f_ibb_mps2 error std +"
              << setw(42) << "f_ibb_mps2 error std -"
              << setw(42) << "f_ibb_mps2 error avg +"
              << setw(42) << "f_ibb_mps2 error avg -"
              << endl;

    for (unsigned int s = 0; s != _Pekfd_handler->get_size_fast(); ++s) {
        Oout_fibb << fixed    << setw(10) << setprecision(3) << showpos << Otrj_nav_out()[s].get_t_sec()                    // t_sec
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_out()[s].get_f_ibb_mps2()[0]          // f_ibb0_mps2 estimated (a posteriori)
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_out()[s].get_f_ibb_mps2()[1]          // f_ibb1_mps2 estimated (a posteriori)
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_out()[s].get_f_ibb_mps2()[2]          // f_ibb2_mps2 estimated (a posteriori)
                  << scientific << setw(14) << setprecision(5) << showpos << _Pekfd_handler->get_xhat_bef()[s](6)           // f_ibb0_mps2 estimated (a priori)
                  << scientific << setw(14) << setprecision(5) << showpos << _Pekfd_handler->get_xhat_bef()[s](7)           // f_ibb1_mps2 estimated (a priori)
                  << scientific << setw(14) << setprecision(5) << showpos << _Pekfd_handler->get_xhat_bef()[s](8)           // f_ibb2_mps2 estimated (a priori)
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_in()[s].get_f_ibb_mps2()(0)           // f_ibb0_mps2 truth
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_in()[s].get_f_ibb_mps2()(1)           // f_ibb1_mps2 truth
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_in()[s].get_f_ibb_mps2()(2)           // f_ibb2_mps2 truth
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_out()[s].get_f_ibb_mps2()(0)         // f_ibb0_mps2 sensed
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_out()[s].get_f_ibb_mps2()(1)         // f_ibb1_mps2 sensed
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_out()[s].get_f_ibb_mps2()(2)         // f_ibb2_mps2 sensed
                  << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekfd_handler->get_P_aft()[s](6,6))      // a posteriori f_ibb0_mps2 standard deviation
                  << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekfd_handler->get_P_aft()[s](7,7))      // a posteriori f_ibb1_mps2 standard deviation
                  << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekfd_handler->get_P_aft()[s](8,8))      // a posteriori f_ibb2_mps2 standard deviation
                  << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekfd_handler->get_P_bef()[s](6,6))      // a priori f_ibb0_mps2 standard deviation
                  << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekfd_handler->get_P_bef()[s](7,7))      // a priori f_ibb1_mps2 standard deviation
                  << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekfd_handler->get_P_bef()[s](8,8))      // a priori f_ibb2_mps2 standard deviation
                  << scientific << setw(14) << setprecision(5) << showpos << _Pekfd_handler->get_eps_xhat_aft_mean()[s](6)  // running mean of a posteriori f_ibb0_mps2
                  << scientific << setw(14) << setprecision(5) << showpos << _Pekfd_handler->get_eps_xhat_aft_mean()[s](7)  // running mean of a posteriori f_ibb1_mps2
                  << scientific << setw(14) << setprecision(5) << showpos << _Pekfd_handler->get_eps_xhat_aft_mean()[s](8)  // running mean of a posteriori f_ibb2_mps2
                  << scientific << setw(14) << setprecision(5) << showpos << _Pekfd_handler->get_eps_xhat_bef_mean()[s](6)  // running mean of a priori f_ibb0_mps2
                  << scientific << setw(14) << setprecision(5) << showpos << _Pekfd_handler->get_eps_xhat_bef_mean()[s](7)  // running mean of a priori f_ibb1_mps2
                  << scientific << setw(14) << setprecision(5) << showpos << _Pekfd_handler->get_eps_xhat_bef_mean()[s](8)  // running mean of a priori f_ibb2_mps2
                  << endl;
    }
    Oout_fibb.close();
}
/* creates text file describing the evolution of f_ibb */

void nav::filter_gps01::textplot_Eacc(const std::string& txt_file_E_acc, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out, const st::trj_gps_out& Otrj_gps_out) const {
    ofstream Oout_E_acc;
    Oout_E_acc.open(txt_file_E_acc);

    Oout_E_acc << setw(10) << "t_sec"
                  << setw(48) << "E_acc_mps2 +"      << setw(48) << "E_acc_mps2 -"     << setw(48) << "E_acc_mps2 truth" << setw(48) << "E_acc_mps2 sensed"
                  << setw(48) << "E_acc_mps2 error std +"
                  << setw(48) << "E_acc_mps2 error std -"
                  << setw(48) << "E_acc_mps2 error avg +"
                  << setw(48) << "E_acc_mps2 error avg -"
                  << endl;

    for (unsigned int s = 0; s != _Pekfd_handler->get_size_fast(); ++s) {
        Oout_E_acc << fixed    << setw(10) << setprecision(3) << showpos << Otrj_nav_out()[s].get_t_sec()                       // t_sec
                      << scientific << setw(16) << setprecision(7) << showpos << Otrj_nav_out()[s].get_E_acc_mps2()[0]          // E_acc0_mps2 estimated (a posteriori)
                      << scientific << setw(16) << setprecision(7) << showpos << Otrj_nav_out()[s].get_E_acc_mps2()[1]          // E_acc1_mps2 estimated (a posteriori)
                      << scientific << setw(16) << setprecision(7) << showpos << Otrj_nav_out()[s].get_E_acc_mps2()[2]          // E_acc2_mps2 estimated (a posteriori)
                      << scientific << setw(16) << setprecision(7) << showpos << _Pekfd_handler->get_xhat_bef()[s](9)           // E_acc0_mps2 estimated (a priori)
                      << scientific << setw(16) << setprecision(7) << showpos << _Pekfd_handler->get_xhat_bef()[s](10)          // E_acc1_mps2 estimated (a priori)
                      << scientific << setw(16) << setprecision(7) << showpos << _Pekfd_handler->get_xhat_bef()[s](11)          // E_acc2_mps2 estimated (a priori)
                      << scientific << setw(16) << setprecision(7) << showpos << Otrj_sens_out()[s].get_E_acc_mps2()(0)         // E_acc0_mps2 truth
                      << scientific << setw(16) << setprecision(7) << showpos << Otrj_sens_out()[s].get_E_acc_mps2()(1)         // E_acc1_mps2 truth
                      << scientific << setw(16) << setprecision(7) << showpos << Otrj_sens_out()[s].get_E_acc_mps2()(2)         // E_acc2_mps2 truth
                      << scientific << setw(16) << setprecision(7) << showpos << sqrt(_Pekfd_handler->get_P_aft()[s](9,9))      // a posteriori E_acc0_mps2 standard deviation
                      << scientific << setw(16) << setprecision(7) << showpos << sqrt(_Pekfd_handler->get_P_aft()[s](10,10))    // a posteriori E_acc1_mps2 standard deviation
                      << scientific << setw(16) << setprecision(7) << showpos << sqrt(_Pekfd_handler->get_P_aft()[s](11,11))    // a posteriori E_acc2_mps2 standard deviation
                      << scientific << setw(16) << setprecision(7) << showpos << sqrt(_Pekfd_handler->get_P_bef()[s](9,9))      // a priori E_acc0_mps2 standard deviation
                      << scientific << setw(16) << setprecision(7) << showpos << sqrt(_Pekfd_handler->get_P_bef()[s](10,10))    // a priori E_acc1_mps2 standard deviation
                      << scientific << setw(16) << setprecision(7) << showpos << sqrt(_Pekfd_handler->get_P_bef()[s](11,11))    // a priori E_acc2_mps2 standard deviation
                      << scientific << setw(16) << setprecision(7) << showpos << _Pekfd_handler->get_eps_xhat_aft_mean()[s](9)  // running mean of a posteriori E_acc0_mps2
                      << scientific << setw(16) << setprecision(7) << showpos << _Pekfd_handler->get_eps_xhat_aft_mean()[s](10) // running mean of a posteriori E_acc1_mps2
                      << scientific << setw(16) << setprecision(7) << showpos << _Pekfd_handler->get_eps_xhat_aft_mean()[s](11) // running mean of a posteriori E_acc2_mps2
                      << scientific << setw(16) << setprecision(7) << showpos << _Pekfd_handler->get_eps_xhat_bef_mean()[s](9)  // running mean of a priori E_acc0_mps2
                      << scientific << setw(16) << setprecision(7) << showpos << _Pekfd_handler->get_eps_xhat_bef_mean()[s](10) // running mean of a priori E_acc1_mps2
                      << scientific << setw(16) << setprecision(7) << showpos << _Pekfd_handler->get_eps_xhat_bef_mean()[s](11) // running mean of a priori E_acc2_mps2
                      << endl;
    }
    Oout_E_acc.close();
}
/* creates text file describing the evolution of the accelerometer bias */

void nav::filter_gps01::textplot_obs_acc(const std::string& txt_file_obs_acc, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out, const st::trj_gps_out& Otrj_gps_out) const {
    ofstream Oout_obs_acc;
    Oout_obs_acc.open(txt_file_obs_acc);

    Oout_obs_acc << setw(10) << "t_sec"
                 << setw(42) << "f_ibb_mps2 truth"
                 << setw(42) << "f_ibb_mps2 sensed"
                 << setw(42) << "f_ibb_mps2 innov"
                 << setw(42) << "f_ibb_mps2 innov std"
                 << setw(42) << "f_ibb_mps2_innov avg"
                 << endl;

    for (unsigned int s = 0; s != _Pekfd_handler->get_size_fast(); ++s) {
        Oout_obs_acc << fixed << setw(10) << setprecision(3) << showpos << Otrj_nav_out()[s].get_t_sec()                   // t_sec
                     << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_in()[s].get_f_ibb_mps2()[0]      // f1_ibb_mps2 real
                     << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_in()[s].get_f_ibb_mps2()[1]      // f2_ibb_mps2 real
                     << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_in()[s].get_f_ibb_mps2()[2]      // f3_ibb_mps2 real
                     << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_out()[s].get_f_ibb_mps2()[0]     // f1_ibb_mps2 sensed
                     << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_out()[s].get_f_ibb_mps2()[1]     // f2_ibb_mps2 sensed
                     << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_out()[s].get_f_ibb_mps2()[2]     // f3_ibb_mps2 sensed
                     << scientific << setw(14) << setprecision(5) << showpos << _Pekfd_handler->get_r_fast()[s][0]         // f1_ibb_mps2 innovation
                     << scientific << setw(14) << setprecision(5) << showpos << _Pekfd_handler->get_r_fast()[s][1]         // f2_ibb_mps2 innovation
                     << scientific << setw(14) << setprecision(5) << showpos << _Pekfd_handler->get_r_fast()[s][2]         // f3_ibb_mps2 innovation
                     << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekfd_handler->get_S_fast()[s](0,0)) // f1_ibb_mps2 innovation standard deviation
                     << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekfd_handler->get_S_fast()[s](1,1)) // f2_ibb_mps2 innovation standard deviation
                     << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekfd_handler->get_S_fast()[s](2,2)) // f3_ibb_mps2 innovation standard deviation
                     << scientific << setw(14) << setprecision(5) << showpos << _Pekfd_handler->get_r_mean_fast()[s][0]    // f1_ibb_mps2 innovation_mean
                     << scientific << setw(14) << setprecision(5) << showpos << _Pekfd_handler->get_r_mean_fast()[s][1]    // f2_ibb_mps2 innovation_mean
                     << scientific << setw(14) << setprecision(5) << showpos << _Pekfd_handler->get_r_mean_fast()[s][2]    // f3_ibb_mps2 innovation_mean
                     << endl;
    }
    Oout_obs_acc.close();
}
/* creates text file describing the evolution of the accelerometer observations */

void nav::filter_gps01::textplot_obs_vned(const std::string& txt_file_obs_vned, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out, const st::trj_gps_out& Otrj_gps_out) const {
    ofstream Oout_obs_vned;
    Oout_obs_vned.open(txt_file_obs_vned);

    Oout_obs_vned << setw(10) << "t_sec"
                  << setw(42) << "v_ned_mps truth"
                  << setw(42) << "v_ned_mps sensed"
                  << setw(42) << "v_ned_mps innov"
                  << setw(42) << "v_ned_mps innov std"
                  << setw(42) << "v_ned_mps_innov avg"
                  << endl;

    for (unsigned int g = 0, s = 0; g != (Otrj_gps_out.get_nel()); ++g) {
        Oout_obs_vned << fixed      << setw(10) << setprecision(3) << showpos << Otrj_gps_out()[g].get_t_sec()               // t_sec
                      << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_in()[s].get_v_n_mps()[0]          // v1_ned_mps real
                      << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_in()[s].get_v_n_mps()[1]          // v2_ned_mps real
                      << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_in()[s].get_v_n_mps()[2]          // v3_ned_mps real
                      << scientific << setw(14) << setprecision(5) << showpos << Otrj_gps_out()[g].get_v_n_mps()[0]          // v1_ned_mps sensed
                      << scientific << setw(14) << setprecision(5) << showpos << Otrj_gps_out()[g].get_v_n_mps()[1]          // v2_ned_mps sensed
                      << scientific << setw(14) << setprecision(5) << showpos << Otrj_gps_out()[g].get_v_n_mps()[2]          // v3_ned_mps sensed
                      << scientific << setw(14) << setprecision(5) << showpos << _Pekfd_handler->get_r_slow()[g][3]          // v1_ned_mps innovation
                      << scientific << setw(14) << setprecision(5) << showpos << _Pekfd_handler->get_r_slow()[g][4]          // v2_ned_mps innovation
                      << scientific << setw(14) << setprecision(5) << showpos << _Pekfd_handler->get_r_slow()[g][5]          // v3_ned_mps innovation
                      << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekfd_handler->get_S_slow()[g](3,3))  // v1_ned_mps innovation standard deviation
                      << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekfd_handler->get_S_slow()[g](4,4))  // v2_ned_mps innovation standard deviation
                      << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekfd_handler->get_S_slow()[g](5,5))  // v3_ned_mps innovation standard deviation
                      << scientific << setw(14) << setprecision(5) << showpos << _Pekfd_handler->get_r_mean_slow()[g][3]     // v1_ned_mps innovation_mean
                      << scientific << setw(14) << setprecision(5) << showpos << _Pekfd_handler->get_r_mean_slow()[g][4]     // v2_ned_mps innovation_mean
                      << scientific << setw(14) << setprecision(5) << showpos << _Pekfd_handler->get_r_mean_slow()[g][5]     // v3_ned_mps innovation_mean
                      << endl;
        s = s + Otrj_gps_out.get_quot() / Otrj_sens_in.get_quot();
    }
    Oout_obs_vned.close();
}
/* creates text file describing the evolution of the ground speed observations */

void nav::filter_gps01::textplot_obs_xgdt(const std::string& txt_file_obs_xgdt, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out, const st::trj_gps_out& Otrj_gps_out) const {
    double r2d = math::constant::R2D();

    ofstream Oout_obs_xgdt;
    Oout_obs_xgdt.open(txt_file_obs_xgdt);

    Oout_obs_xgdt << setw(10) << "t_sec"
                  << setw(51) << "x_gdt_deg_m truth"
                  << setw(51) << "x_gdt_deg_m sensed"
                  << setw(51) << "x_gdt_deg_m innov"
                  << setw(51) << "x_gdt_deg_m innov std"
                  << setw(51) << "x_gdt_deg_m_innov avg"
                  << endl;

    for (unsigned int g = 0, s = 0; g != (Otrj_gps_out.get_nel()); ++g) {
        Oout_obs_xgdt << fixed      << setw(10) << setprecision(3) << showpos << Otrj_gps_out()[g].get_t_sec()                                // t_sec
                      << scientific << setw(18) << setprecision(9) << showpos << Otrj_sens_in()[s].get_x_gdt_rad_m().get_lambda_rad() * r2d   // lambda_deg real
                      << scientific << setw(18) << setprecision(9) << showpos << Otrj_sens_in()[s].get_x_gdt_rad_m().get_phi_rad() * r2d      // phi_deg    real
                      << scientific << setw(15) << setprecision(6) << showpos << Otrj_sens_in()[s].get_x_gdt_rad_m().get_h_m()                // h_m        real
                      << scientific << setw(18) << setprecision(9) << showpos << Otrj_gps_out()[g].get_x_gdt_rad_m().get_lambda_rad() * r2d   // lambda_deg sensed
                      << scientific << setw(18) << setprecision(9) << showpos << Otrj_gps_out()[g].get_x_gdt_rad_m().get_phi_rad() * r2d      // phi_deg    sensed
                      << scientific << setw(15) << setprecision(6) << showpos << Otrj_gps_out()[g].get_x_gdt_rad_m().get_h_m()                // h_m        sensed
                      << scientific << setw(18) << setprecision(9) << showpos << _Pekfd_handler->get_r_slow()[g][6] * r2d                     // lambda_deg innovation
                      << scientific << setw(18) << setprecision(9) << showpos << _Pekfd_handler->get_r_slow()[g][7] * r2d                     // phi_deg    innovation
                      << scientific << setw(15) << setprecision(6) << showpos << _Pekfd_handler->get_r_slow()[g][8]                           // h_m        innovation
                      << scientific << setw(18) << setprecision(9) << showpos << sqrt(_Pekfd_handler->get_S_slow()[g](6,6)) * r2d             // lambda_deg innovation standard deviation
                      << scientific << setw(18) << setprecision(9) << showpos << sqrt(_Pekfd_handler->get_S_slow()[g](7,7)) * r2d             // phi_deg    innovation standard deviation
                      << scientific << setw(15) << setprecision(6) << showpos << sqrt(_Pekfd_handler->get_S_slow()[g](8,8))                   // h_m        innovation standard deviation
                      << scientific << setw(18) << setprecision(9) << showpos << _Pekfd_handler->get_r_mean_slow()[g][6] * r2d                // lambda_deg innovation_mean
                      << scientific << setw(18) << setprecision(9) << showpos << _Pekfd_handler->get_r_mean_slow()[g][7] * r2d                // phi_deg    innovation_mean
                      << scientific << setw(15) << setprecision(6) << showpos << _Pekfd_handler->get_r_mean_slow()[g][8]                      // h_m        innovation_mean
                      << endl;
        s = s + Otrj_gps_out.get_quot() / Otrj_sens_in.get_quot();
    }
    Oout_obs_xgdt.close();
}
/* creates text file describing the evolution of the position observations */

void nav::filter_gps01::textplot_wind_DeltaTp(const std::string& txt_file_wind_DeltaTp, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out, const st::trj_gps_out& Otrj_gps_out) const {
    ofstream Oout_wind;
    Oout_wind.open(txt_file_wind_DeltaTp);

    Oout_wind << setw(10) << "t_sec"
              << setw(42) << "vlf_n_mps estimated"
              << setw(42) << "vlf_n_mps truth"
              << setw(14) << "DeltaT_degK est"
              << setw(14) << "DeltaT_degK truth"
              << setw(14) << "Deltap_pa est"
              << setw(14) << "Deltap_pa truth"
              << endl;

    for (unsigned int s = 0; s != _Pekfd_handler->get_size_fast(); ++s) {
        Oout_wind << fixed    << setw(10) << setprecision(3) << showpos << Otrj_nav_out()[s].get_t_sec()                    // t_sec
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_out()[s].get_vlf_n_mps()[0]           // vlf_n0_mps estimated
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_out()[s].get_vlf_n_mps()[1]           // vlf_n1_mps estimated
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_out()[s].get_vlf_n_mps()[2]           // vlf_n2_mps estimated
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_in()[s].get_vlf_n_mps()(0)            // vlf_n0_mps truth
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_in()[s].get_vlf_n_mps()(1)            // vlf_n1_mps truth
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_in()[s].get_vlf_n_mps()(2)            // vlf_n2_mps truth
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_out()[s].get_DeltaT_degK()            // DeltaT degK estimated
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_in()[s].get_DeltaT_degK()             // DeltaT_degK truth
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_out()[s].get_Deltap_pa()              // Deltap_pa estimated
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_in()[s].get_Deltap_pa()               // Deltap_pa truth
                  << endl;
    }
    Oout_wind.close();
}
/* creates text file describing the evolution of vlf_n_mps, DeltaT, and Deltap */

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////







