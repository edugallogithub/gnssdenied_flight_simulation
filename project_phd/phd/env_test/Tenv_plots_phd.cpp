#include "Tenv_plots_phd.h"
#include "math/logic/share.h"
#include "env/geo.h"
#include "env/atm.h"
#include <boost/filesystem.hpp>

env::test::Tenv_plots_phd::Tenv_plots_phd(jail::counter& Ocounter)
        : ::jail::unit_test(Ocounter) {
}
/* constructor based on counter */

void env::test::Tenv_plots_phd::run() {
    ::jail::unit_test::run();

    boost::filesystem::path path_outputs(math::share::phd_outputs_prefix);
    boost::filesystem::path path_env_test("env");
    boost::filesystem::create_directory((path_outputs / path_env_test).string());

    test_Tp_Hp__DeltaT();
    test_H_h__phi();
    test_H_vs_Hp__DeltaT();

    finished();
}
/* execute tests and write results on console */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void env::test::Tenv_plots_phd::test_Tp_Hp__DeltaT() {
    env::atm Oam_n20(-20.0, 0.0);
    env::atm Oam_n10(-10.0, 0.0);
    env::atm Oam_p00(0.0, 0.0);
    env::atm Oam_p10(+10.0, 0.0);
    env::atm Oam_p20(+20.0, 0.0);

    boost::filesystem::path path_outputs(math::share::phd_outputs_prefix);
    boost::filesystem::path path_env_test("env");
    boost::filesystem::path path_file_p_vs_Hp("p_vs_Hp.txt");
    boost::filesystem::path path_file_T_vs_Hp__DeltaT("T_vs_Hp__DeltaT.txt");

    std::ofstream Oout_p_vs_Hp, Oout_T_vs_Hp__DeltaT;
    Oout_p_vs_Hp.open((path_outputs / path_env_test / path_file_p_vs_Hp).string());
    Oout_T_vs_Hp__DeltaT.open((path_outputs / path_env_test / path_file_T_vs_Hp__DeltaT).string());

    size_t nel = 101;
    std::vector<double>VHp_m(nel);
    double p_pa;
    double T_degK_n20, T_degK_n10, T_degK_p00, T_degK_p10, T_degK_p20;

    for (int i = 0; i != nel; ++i) {
        VHp_m[i] = 50.0 * i;
        p_pa = Oam_p00.Hp2p(VHp_m[i]);
        T_degK_n20 = Oam_n20.Hp2T(VHp_m[i]);
        T_degK_n10 = Oam_n10.Hp2T(VHp_m[i]);
        T_degK_p00 = Oam_p00.Hp2T(VHp_m[i]);
        T_degK_p10 = Oam_p10.Hp2T(VHp_m[i]);
        T_degK_p20 = Oam_p20.Hp2T(VHp_m[i]);

        Oout_p_vs_Hp << std::fixed << std::showpos
                     << std::setprecision(2) << std::setw(8)  << VHp_m[i]
                     << std::setprecision(8) << std::setw(14) << p_pa / 1e4
                     << std::endl;

        Oout_T_vs_Hp__DeltaT << std::fixed << std::showpos
                     << std::setprecision(2) << std::setw(8)  << VHp_m[i]
                     << std::setprecision(2) << std::setw(10) << T_degK_n20
                     << std::setprecision(2) << std::setw(10) << T_degK_n10
                     << std::setprecision(2) << std::setw(10) << T_degK_p00
                     << std::setprecision(2) << std::setw(10) << T_degK_p10
                     << std::setprecision(2) << std::setw(10) << T_degK_p20
                     << std::endl;
    }

    Oout_p_vs_Hp.close();
    Oout_T_vs_Hp__DeltaT.close();
} // closes test_Tp_Hp__DeltaT


/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void env::test::Tenv_plots_phd::test_H_h__phi() {
    env::geo_mix Ogeo_mix(env::logic::mag_default);
    //env::geo_ell Ogeo_ell(env::logic::mag_default);
    double d2r = math::constant::D2R();

    //boost::filesystem::path path_outputs(math::share::phd_outputs_prefix);
    //boost::filesystem::path path_env_test("env");
    //boost::filesystem::path path_file_H_vs_h__phi("H_vs_h__phi.txt");

    //std::ofstream Oout_H_vs_h__phi;
    //Oout_H_vs_h__phi.open((path_outputs / path_env_test / path_file_H_vs_h__phi).string());

    size_t nel = 6;
    std::vector<double>Vh_m(nel);
    double H_m_00, H_m_15, H_m_30, H_m_45, H_m_60, H_m_75, H_m_90, H_m_sph;

    double phi_deg_00 =  0.0;       double phi_rad_00 = phi_deg_00 * d2r;
    double phi_deg_15 = 15.0;       double phi_rad_15 = phi_deg_15 * d2r;
    double phi_deg_30 = 30.0;       double phi_rad_30 = phi_deg_30 * d2r;
    double phi_deg_45 = 45.0;       double phi_rad_45 = phi_deg_45 * d2r;
    double phi_deg_60 = 60.0;       double phi_rad_60 = phi_deg_60 * d2r;
    double phi_deg_75 = 75.0;       double phi_rad_75 = phi_deg_75 * d2r;
    double phi_deg_90 = 90.0;       double phi_rad_90 = phi_deg_90 * d2r;

    for (int i = 0; i != nel; ++i) {
        Vh_m[i] = 1000.0 * i;
        // H_m_00  = Ogeo_ell.htoH(Vh_m[i], phi_rad_00);
        //H_m_15  = Ogeo_ell.htoH(Vh_m[i], phi_rad_15);
        //H_m_30  = Ogeo_ell.htoH(Vh_m[i], phi_rad_30);
        //H_m_45  = Ogeo_ell.htoH(Vh_m[i], phi_rad_45);
        //H_m_60  = Ogeo_ell.htoH(Vh_m[i], phi_rad_60);
        //H_m_75  = Ogeo_ell.htoH(Vh_m[i], phi_rad_75);
        //H_m_90  = Ogeo_ell.htoH(Vh_m[i], phi_rad_90);
        //H_m_sph = Ogeo_mix.htoH(Vh_m[i], 9999999);

        //Oout_H_vs_h__phi << std::fixed
        //             << std::setprecision(2) << std::setw(12) << Vh_m[i]
        //             << std::setprecision(2) << std::setw(12) << H_m_00
        //             << std::setprecision(2) << std::setw(12) << H_m_15
        //             << std::setprecision(2) << std::setw(12) << H_m_30
        //             << std::setprecision(2) << std::setw(12) << H_m_45
        //             << std::setprecision(2) << std::setw(12) << H_m_60
        //             << std::setprecision(2) << std::setw(12) << H_m_75
        //             << std::setprecision(2) << std::setw(12) << H_m_90
        //             << std::setprecision(2) << std::setw(12) << H_m_sph
        //             << std::endl;
    }

    //Oout_H_vs_h__phi.close();
} // closes test_H_h__phi

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void env::test::Tenv_plots_phd::test_H_vs_Hp__DeltaT() {
    env::atm Oam_n20(-20.0, 0.0);
    env::atm Oam_n10(-10.0, 0.0);
    env::atm Oam_p00(0.0, 0.0);
    env::atm Oam_p10(+10.0, 0.0);
    env::atm Oam_p20(+20.0, 0.0);
    env::atm Oam_n20n4000(-20.0,-4000.0);
    env::atm Oam_n20p4000(-20.0,+4000.0);
    env::atm Oam_p00p0000( +0.0,   +0.0);
    env::atm Oam_p20n4000(+20.0,-4000.0);
    env::atm Oam_p20p4000(+20.0,+4000.0);

    boost::filesystem::path path_outputs(math::share::phd_outputs_prefix);
    boost::filesystem::path path_env_test("env");
    boost::filesystem::path path_file_dHdHp_vs_Hp__DeltaT("dHdHp_vs_Hp__DeltaT.txt");
    boost::filesystem::path path_file_H_vs_Hp__DeltaT("H_vs_Hp__DeltaT.txt");

    std::ofstream  Oout_dHdHp_vs_Hp__DeltaT, Oout_H_vs_Hp__DeltaT;
    Oout_dHdHp_vs_Hp__DeltaT.open((path_outputs / path_env_test / path_file_dHdHp_vs_Hp__DeltaT).string());
    Oout_H_vs_Hp__DeltaT.open((path_outputs / path_env_test / path_file_H_vs_Hp__DeltaT).string());

    size_t nel = 101;
    std::vector<double>VHp_m(nel);
    double dHdHp_n20, dHdHp_n10, dHdHp_p00, dHdHp_p10, dHdHp_p20;
    double H_m_n20n4000, H_m_n20p4000, H_m_p00p0000, H_m_p20n4000, H_m_p20p4000;
    double Tisa, T;

    for (int i = 0; i != nel; ++i) {
        VHp_m[i] = 50.0 * i;

        Tisa = Oam_n20.Hp2Tisa(VHp_m[i]);
        T    = Oam_n20.Hp2T(VHp_m[i]);
        dHdHp_n20 = T / Tisa;

        Tisa = Oam_n10.Hp2Tisa(VHp_m[i]);
        T    = Oam_n10.Hp2T(VHp_m[i]);
        dHdHp_n10 = T / Tisa;

        Tisa = Oam_p00.Hp2Tisa(VHp_m[i]);
        T    = Oam_p00.Hp2T(VHp_m[i]);
        dHdHp_p00 = T / Tisa;

        Tisa = Oam_p10.Hp2Tisa(VHp_m[i]);
        T    = Oam_p10.Hp2T(VHp_m[i]);
        dHdHp_p10 = T / Tisa;

        Tisa = Oam_p20.Hp2Tisa(VHp_m[i]);
        T    = Oam_p20.Hp2T(VHp_m[i]);
        dHdHp_p20 = T / Tisa;

        H_m_n20n4000 = Oam_n20n4000.Hp2H(VHp_m[i]);
        H_m_n20p4000 = Oam_n20p4000.Hp2H(VHp_m[i]);
        H_m_p00p0000 = Oam_p00p0000.Hp2H(VHp_m[i]);
        H_m_p20n4000 = Oam_p20n4000.Hp2H(VHp_m[i]);
        H_m_p20p4000 = Oam_p20p4000.Hp2H(VHp_m[i]);


        Oout_dHdHp_vs_Hp__DeltaT << std::fixed << std::showpos
                             << std::setprecision(2) << std::setw(8)  << VHp_m[i]
                             << std::setprecision(6) << std::setw(12) << dHdHp_n20
                             << std::setprecision(6) << std::setw(12) << dHdHp_n10
                             << std::setprecision(6) << std::setw(12) << dHdHp_p00
                             << std::setprecision(6) << std::setw(12) << dHdHp_p10
                             << std::setprecision(6) << std::setw(12) << dHdHp_p20
                             << std::endl;

        Oout_H_vs_Hp__DeltaT << std::fixed << std::showpos
                                 << std::setprecision(2) << std::setw(10)  << VHp_m[i]
                                 << std::setprecision(2) << std::setw(10) << H_m_n20n4000
                                 << std::setprecision(2) << std::setw(10) << H_m_n20p4000
                                 << std::setprecision(2) << std::setw(10) << H_m_p00p0000
                                 << std::setprecision(2) << std::setw(10) << H_m_p20n4000
                                 << std::setprecision(2) << std::setw(10) << H_m_p20p4000
                                 << std::endl;
    }

    Oout_dHdHp_vs_Hp__DeltaT.close();
    Oout_H_vs_Hp__DeltaT;
} // closes test_H_vs_Hp__DeltaT


/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////





