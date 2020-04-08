#include "Tapm_plots.h"
#include "math/logic/share.h"
#include "env/atm.h"
#include "acft/acft/prop.h"
#include "acft/acft/iner.h"
#include "acft/acft/aero3.h"
#include "acft/acft/aero2.h"
#include "acft/acft/aero1.h"
#include "acft/acft/aero0.h"
#include "ang/quat.h"
#include <boost/filesystem.hpp>

acft::test::Tapm_plots::Tapm_plots(jail::counter& Ocounter)
        : ::jail::unit_test(Ocounter) {
}
/* constructor based on counter */

void acft::test::Tapm_plots::run() {
    ::jail::unit_test::run();

    boost::filesystem::path path_outputs(math::share::phd_outputs_prefix);
    boost::filesystem::path path_acft_test("acft");
    boost::filesystem::create_directory((path_outputs / path_acft_test).string());
    
    test1_cl_cd_cmm_alpha__deltaE();
    test2_cl_cd_cmm_alpha__beta();
    test3_cy_cml_cmn_beta__deltaR();
    test4_cy_cml_cmn_beta__deltaA();
    test5_cmn_alpha__deltaR();
    test6_cml_alpha__deltaA();
    test7_P_F_Hp__deltaT();
    test8_ct_cp_eta__J();
    test9_n_T_M_P__vtas();

    finished();
}
/* execute tests and write results on console */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void acft::test::Tapm_plots::test1_cl_cd_cmm_alpha__deltaE() {
    acft::aero0 Oaero(math::logic::biparabolic);
    double d2r = math::constant::D2R();

    boost::filesystem::path path_outputs(math::share::phd_outputs_prefix);
    boost::filesystem::path path_acft_test("acft");
    boost::filesystem::path path_file_cl_vs_alpha__deltaE("test1_cl_vs_alpha__deltaE.txt");
    boost::filesystem::path path_file_cd_vs_alpha__deltaE("test1_cd_vs_alpha__deltaE.txt");
    boost::filesystem::path path_file_cd_vs_cl__deltaE("test1_cd_vs_cl__deltaE.txt");
    boost::filesystem::path path_file_cmm_vs_alpha__deltaE("test1_cmm_vs_alpha__deltaE.txt");

    std::ofstream Oout_cl_vs_alpha__deltaE, Oout_cd_vs_alpha__deltaE, Oout_cd_vs_cl__deltaE, Oout_cmm_vs_alpha__deltaE;
    Oout_cl_vs_alpha__deltaE.open((path_outputs / path_acft_test / path_file_cl_vs_alpha__deltaE).string());
    Oout_cd_vs_alpha__deltaE.open((path_outputs / path_acft_test / path_file_cd_vs_alpha__deltaE).string());
    Oout_cd_vs_cl__deltaE.open((path_outputs / path_acft_test / path_file_cd_vs_cl__deltaE).string());
    Oout_cmm_vs_alpha__deltaE.open((path_outputs / path_acft_test / path_file_cmm_vs_alpha__deltaE).string());

    size_t nel = 176;
    std::vector<double>Valpha_deg(nel);
    double alpha_rad;

    ang::quat delta_control_n8(0.0, -8.0, 0.0, 0.0);
    ang::quat delta_control_n4(0.0, -4.0, 0.0, 0.0);
    ang::quat delta_control_p0(0.0, +0.0, 0.0, 0.0);
    ang::quat delta_control_p4(0.0, +4.0, 0.0, 0.0);
    ang::quat delta_control_p8(0.0, +8.0, 0.0, 0.0);

    double vtas_mps = 30.0;
    double beta_rad = 0.0;
    ang::euler euler_wfsbfs_rad(0.0, 0.0, 0.0);
    Eigen::Vector3d w_nedbfsbfs_rps(0.0, 0.0, 0.0);
    Eigen::Vector3d cf_aer_wfs_n8, cf_aer_wfs_n4, cf_aer_wfs_p0, cf_aer_wfs_p4, cf_aer_wfs_p8;
    Eigen::Vector3d cm_aer_bfs_n8, cm_aer_bfs_n4, cm_aer_bfs_p0, cm_aer_bfs_p4, cm_aer_bfs_p8;

    for (int i = 0; i != nel; ++i) {
        Valpha_deg[i] = -5 + 0.1 * i;
        alpha_rad = Valpha_deg[i] * d2r;
        euler_wfsbfs_rad.set_yaw_rad(-beta_rad);
        euler_wfsbfs_rad.set_pitch_rad(alpha_rad);

        cf_aer_wfs_n8 = euler_wfsbfs_rad * Oaero.obtain_cf_aer(vtas_mps, euler_wfsbfs_rad, delta_control_n8, w_nedbfsbfs_rps);
        cf_aer_wfs_n4 = euler_wfsbfs_rad * Oaero.obtain_cf_aer(vtas_mps, euler_wfsbfs_rad, delta_control_n4, w_nedbfsbfs_rps);
        cf_aer_wfs_p0 = euler_wfsbfs_rad * Oaero.obtain_cf_aer(vtas_mps, euler_wfsbfs_rad, delta_control_p0, w_nedbfsbfs_rps);
        cf_aer_wfs_p4 = euler_wfsbfs_rad * Oaero.obtain_cf_aer(vtas_mps, euler_wfsbfs_rad, delta_control_p4, w_nedbfsbfs_rps);
        cf_aer_wfs_p8 = euler_wfsbfs_rad * Oaero.obtain_cf_aer(vtas_mps, euler_wfsbfs_rad, delta_control_p8, w_nedbfsbfs_rps);

        cm_aer_bfs_n8 =  Oaero.obtain_cm_aer(vtas_mps, euler_wfsbfs_rad, delta_control_n8, w_nedbfsbfs_rps);
        cm_aer_bfs_n4 =  Oaero.obtain_cm_aer(vtas_mps, euler_wfsbfs_rad, delta_control_n4, w_nedbfsbfs_rps);
        cm_aer_bfs_p0 =  Oaero.obtain_cm_aer(vtas_mps, euler_wfsbfs_rad, delta_control_p0, w_nedbfsbfs_rps);
        cm_aer_bfs_p4 =  Oaero.obtain_cm_aer(vtas_mps, euler_wfsbfs_rad, delta_control_p4, w_nedbfsbfs_rps);
        cm_aer_bfs_p8 =  Oaero.obtain_cm_aer(vtas_mps, euler_wfsbfs_rad, delta_control_p8, w_nedbfsbfs_rps);

        Oout_cl_vs_alpha__deltaE << std::fixed << std::showpos
                                 << std::setprecision(2) << std::setw(6) << Valpha_deg[i]
                                 << std::setprecision(5) << std::setw(11) << - cf_aer_wfs_n8(2)
                                 << std::setprecision(5) << std::setw(11) << - cf_aer_wfs_n4(2)
                                 << std::setprecision(5) << std::setw(11) << - cf_aer_wfs_p0(2)
                                 << std::setprecision(5) << std::setw(11) << - cf_aer_wfs_p4(2)
                                 << std::setprecision(5) << std::setw(11) << - cf_aer_wfs_p8(2)
                                 << std::endl;

        Oout_cd_vs_alpha__deltaE << std::fixed << std::showpos
                                 << std::setprecision(2) << std::setw(6) << Valpha_deg[i]
                                 << std::setprecision(5) << std::setw(11) << - cf_aer_wfs_n8(0)
                                 << std::setprecision(5) << std::setw(11) << - cf_aer_wfs_n4(0)
                                 << std::setprecision(5) << std::setw(11) << - cf_aer_wfs_p0(0)
                                 << std::setprecision(5) << std::setw(11) << - cf_aer_wfs_p4(0)
                                 << std::setprecision(5) << std::setw(11) << - cf_aer_wfs_p8(0)
                                 << std::endl;

        Oout_cd_vs_cl__deltaE << std::fixed << std::showpos
                              << std::setprecision(5) << std::setw(11) << - cf_aer_wfs_n8(2)
                              << std::setprecision(5) << std::setw(11) << - cf_aer_wfs_n8(0)
                              << std::setprecision(5) << std::setw(11) << - cf_aer_wfs_n4(2)
                              << std::setprecision(5) << std::setw(11) << - cf_aer_wfs_n4(0)
                              << std::setprecision(5) << std::setw(11) << - cf_aer_wfs_p0(2)
                              << std::setprecision(5) << std::setw(11) << - cf_aer_wfs_p0(0)
                              << std::setprecision(5) << std::setw(11) << - cf_aer_wfs_p4(2)
                              << std::setprecision(5) << std::setw(11) << - cf_aer_wfs_p4(0)
                              << std::setprecision(5) << std::setw(11) << - cf_aer_wfs_p8(2)
                              << std::setprecision(5) << std::setw(11) << - cf_aer_wfs_p8(0)
                              << std::endl;

        Oout_cmm_vs_alpha__deltaE << std::fixed << std::showpos
                                 << std::setprecision(2) << std::setw(6) << Valpha_deg[i]
                                 << std::setprecision(5) << std::setw(11) << cm_aer_bfs_n8(1)
                                 << std::setprecision(5) << std::setw(11) << cm_aer_bfs_n4(1)
                                 << std::setprecision(5) << std::setw(11) << cm_aer_bfs_p0(1)
                                 << std::setprecision(5) << std::setw(11) << cm_aer_bfs_p4(1)
                                 << std::setprecision(5) << std::setw(11) << cm_aer_bfs_p8(1)
                                 << std::endl;
    }

    Oout_cl_vs_alpha__deltaE.close();
    Oout_cd_vs_alpha__deltaE.close();
    Oout_cd_vs_cl__deltaE.close();
    Oout_cmm_vs_alpha__deltaE.close();
} // closes test1_cl_cd_cmm_alpha__deltaE

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void acft::test::Tapm_plots::test2_cl_cd_cmm_alpha__beta() {
    acft::aero0 Oaero(math::logic::biparabolic);
    double d2r = math::constant::D2R();

    boost::filesystem::path path_outputs(math::share::phd_outputs_prefix);
    boost::filesystem::path path_acft_test("acft");
    boost::filesystem::path path_file_cl_vs_alpha__beta("test2_cl_vs_alpha__beta.txt");
    boost::filesystem::path path_file_cd_vs_alpha__beta("test2_cd_vs_alpha__beta.txt");
    boost::filesystem::path path_file_cd_vs_cl__beta("test2_cd_vs_cl__beta.txt");
    boost::filesystem::path path_file_cmm_vs_alpha__beta("test2_cmm_vs_alpha__beta.txt");

    std::ofstream Oout_cl_vs_alpha__beta, Oout_cd_vs_alpha__beta, Oout_cd_vs_cl__beta, Oout_cmm_vs_alpha__beta;
    Oout_cl_vs_alpha__beta.open((path_outputs / path_acft_test / path_file_cl_vs_alpha__beta).string());
    Oout_cd_vs_alpha__beta.open((path_outputs / path_acft_test / path_file_cd_vs_alpha__beta).string());
    Oout_cd_vs_cl__beta.open((path_outputs / path_acft_test / path_file_cd_vs_cl__beta).string());
    Oout_cmm_vs_alpha__beta.open((path_outputs / path_acft_test / path_file_cmm_vs_alpha__beta).string());

    size_t nel = 176;
    std::vector<double>Valpha_deg(nel);
    double alpha_rad;
    ang::quat delta_control(0.0, 0.0, 0.0, 0.0);

    ang::euler euler_wfsbfs_rad_n100(10 * d2r, 0.0, 0.0);
    ang::euler euler_wfsbfs_rad_n075(7.5 * d2r, 0.0, 0.0);
    ang::euler euler_wfsbfs_rad_n050(5.0 * d2r, 0.0, 0.0);
    ang::euler euler_wfsbfs_rad_n025(2.5 * d2r, 0.0, 0.0);
    ang::euler euler_wfsbfs_rad_p000(0 * d2r, 0.0, 0.0);
    ang::euler euler_wfsbfs_rad_p025(-2.5 * d2r, 0.0, 0.0);
    ang::euler euler_wfsbfs_rad_p050(-5.0 * d2r, 0.0, 0.0);
    ang::euler euler_wfsbfs_rad_p075(-7.5 * d2r, 0.0, 0.0);
    ang::euler euler_wfsbfs_rad_p100(-10 * d2r, 0.0, 0.0);

    double vtas_mps = 30.0;
    Eigen::Vector3d w_nedbfsbfs_rps(0.0, 0.0, 0.0);
    Eigen::Vector3d cf_aer_wfs_p000, cf_aer_wfs_p025, cf_aer_wfs_p050, cf_aer_wfs_p075, cf_aer_wfs_p100;
    Eigen::Vector3d cm_aer_bfs_p000, cm_aer_bfs_p025, cm_aer_bfs_p050, cm_aer_bfs_p075, cm_aer_bfs_p100;

    for (int i = 0; i != nel; ++i) {
        Valpha_deg[i] = -5 + 0.1 * i;
        alpha_rad = Valpha_deg[i] * d2r;
        euler_wfsbfs_rad_n100.set_pitch_rad(alpha_rad);
        euler_wfsbfs_rad_n075.set_pitch_rad(alpha_rad);
        euler_wfsbfs_rad_n050.set_pitch_rad(alpha_rad);
        euler_wfsbfs_rad_n025.set_pitch_rad(alpha_rad);
        euler_wfsbfs_rad_p000.set_pitch_rad(alpha_rad);
        euler_wfsbfs_rad_p025.set_pitch_rad(alpha_rad);
        euler_wfsbfs_rad_p050.set_pitch_rad(alpha_rad);
        euler_wfsbfs_rad_p075.set_pitch_rad(alpha_rad);
        euler_wfsbfs_rad_p100.set_pitch_rad(alpha_rad);

        cf_aer_wfs_p000 = euler_wfsbfs_rad_p000 * Oaero.obtain_cf_aer(vtas_mps, euler_wfsbfs_rad_p000, delta_control, w_nedbfsbfs_rps);
        cf_aer_wfs_p025 = euler_wfsbfs_rad_p025 * Oaero.obtain_cf_aer(vtas_mps, euler_wfsbfs_rad_p025, delta_control, w_nedbfsbfs_rps);
        cf_aer_wfs_p050 = euler_wfsbfs_rad_p050 * Oaero.obtain_cf_aer(vtas_mps, euler_wfsbfs_rad_p050, delta_control, w_nedbfsbfs_rps);
        cf_aer_wfs_p075 = euler_wfsbfs_rad_p075 * Oaero.obtain_cf_aer(vtas_mps, euler_wfsbfs_rad_p075, delta_control, w_nedbfsbfs_rps);
        cf_aer_wfs_p100 = euler_wfsbfs_rad_p100 * Oaero.obtain_cf_aer(vtas_mps, euler_wfsbfs_rad_p100, delta_control, w_nedbfsbfs_rps);

        cm_aer_bfs_p000 = Oaero.obtain_cm_aer(vtas_mps, euler_wfsbfs_rad_p000, delta_control, w_nedbfsbfs_rps);
        cm_aer_bfs_p025 = Oaero.obtain_cm_aer(vtas_mps, euler_wfsbfs_rad_p025, delta_control, w_nedbfsbfs_rps);
        cm_aer_bfs_p050 = Oaero.obtain_cm_aer(vtas_mps, euler_wfsbfs_rad_p050, delta_control, w_nedbfsbfs_rps);
        cm_aer_bfs_p075 = Oaero.obtain_cm_aer(vtas_mps, euler_wfsbfs_rad_p075, delta_control, w_nedbfsbfs_rps);
        cm_aer_bfs_p100 = Oaero.obtain_cm_aer(vtas_mps, euler_wfsbfs_rad_p100, delta_control, w_nedbfsbfs_rps);

        Oout_cl_vs_alpha__beta << std::fixed << std::showpos
            << std::setprecision(2) << std::setw(6) << Valpha_deg[i]
            << std::setprecision(5) << std::setw(11) << - cf_aer_wfs_p000(2)
            << std::setprecision(5) << std::setw(11) << - cf_aer_wfs_p025(2)
            << std::setprecision(5) << std::setw(11) << - cf_aer_wfs_p050(2)
            << std::setprecision(5) << std::setw(11) << - cf_aer_wfs_p075(2)
            << std::setprecision(5) << std::setw(11) << - cf_aer_wfs_p100(2)
            << std::endl;

        Oout_cd_vs_alpha__beta << std::fixed << std::showpos
                << std::setprecision(2) << std::setw(6) << Valpha_deg[i]
                << std::setprecision(5) << std::setw(11) << - cf_aer_wfs_p000(0)
                << std::setprecision(5) << std::setw(11) << - cf_aer_wfs_p025(0)
                << std::setprecision(5) << std::setw(11) << - cf_aer_wfs_p050(0)
                << std::setprecision(5) << std::setw(11) << - cf_aer_wfs_p075(0)
                << std::setprecision(5) << std::setw(11) << - cf_aer_wfs_p100(0)
                << std::endl;

        Oout_cd_vs_cl__beta << std::fixed << std::showpos
                << std::setprecision(5) << std::setw(11) << - cf_aer_wfs_p000(2)
                << std::setprecision(5) << std::setw(11) << - cf_aer_wfs_p000(0)
                << std::setprecision(5) << std::setw(11) << - cf_aer_wfs_p025(2)
                << std::setprecision(5) << std::setw(11) << - cf_aer_wfs_p025(0)
                << std::setprecision(5) << std::setw(11) << - cf_aer_wfs_p050(2)
                << std::setprecision(5) << std::setw(11) << - cf_aer_wfs_p050(0)
                << std::setprecision(5) << std::setw(11) << - cf_aer_wfs_p075(2)
                << std::setprecision(5) << std::setw(11) << - cf_aer_wfs_p075(0)
                << std::setprecision(5) << std::setw(11) << - cf_aer_wfs_p100(2)
                << std::setprecision(5) << std::setw(11) << - cf_aer_wfs_p100(0)
                << std::endl;

        Oout_cmm_vs_alpha__beta << std::fixed << std::showpos
                << std::setprecision(2) << std::setw(6) << Valpha_deg[i]
                << std::setprecision(5) << std::setw(11) << cm_aer_bfs_p000(1)
                << std::setprecision(5) << std::setw(11) << cm_aer_bfs_p025(1)
                << std::setprecision(5) << std::setw(11) << cm_aer_bfs_p050(1)
                << std::setprecision(5) << std::setw(11) << cm_aer_bfs_p075(1)
                << std::setprecision(5) << std::setw(11) << cm_aer_bfs_p100(1)
                << std::endl;
    }

    Oout_cl_vs_alpha__beta.close();
    Oout_cd_vs_alpha__beta.close();
    Oout_cd_vs_cl__beta.close();
    Oout_cmm_vs_alpha__beta.close();
} // closes test2_cl_cd_cmm_alpha__beta

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void acft::test::Tapm_plots::test3_cy_cml_cmn_beta__deltaR() {
    acft::aero0 Oaero(math::logic::biparabolic);
    double d2r = math::constant::D2R();

    boost::filesystem::path path_outputs(math::share::phd_outputs_prefix);
    boost::filesystem::path path_acft_test("acft");
    boost::filesystem::path path_file_cy_vs_beta__deltaR("test3_cy_vs_beta__deltaR.txt");
    boost::filesystem::path path_file_cmn_vs_beta__deltaR("test3_cmn_vs_beta__deltaR.txt");
    boost::filesystem::path path_file_cml_vs_beta__deltaR("test3_cml_vs_beta__deltaR.txt");

    std::ofstream Oout_cy_vs_beta__deltaR, Oout_cmn_vs_beta__deltaR, Oout_cml_vs_beta__deltaR;
    Oout_cy_vs_beta__deltaR.open((path_outputs / path_acft_test / path_file_cy_vs_beta__deltaR).string());
    Oout_cmn_vs_beta__deltaR.open((path_outputs / path_acft_test / path_file_cmn_vs_beta__deltaR).string());
    Oout_cml_vs_beta__deltaR.open((path_outputs / path_acft_test / path_file_cml_vs_beta__deltaR).string());

    size_t nel = 201;
    std::vector<double>Vbeta_deg(nel);
    double beta_rad;

    ang::quat delta_control_n8(0.0, 0.0, 0.0, -8.0);
    ang::quat delta_control_n4(0.0, 0.0, 0.0, -4.0);
    ang::quat delta_control_p0(0.0, 0.0, 0.0, +0.0);
    ang::quat delta_control_p4(0.0, 0.0, 0.0, +4.0);
    ang::quat delta_control_p8(0.0, 0.0, 0.0, +8.0);

    double vtas_mps = 30.0;
    double alpha_rad = 4 * d2r;
    ang::euler euler_wfsbfs_rad(0.0, alpha_rad, 0.0);
    Eigen::Vector3d w_nedbfsbfs_rps(0.0, 0.0, 0.0);

    Eigen::Vector3d cf_aer_wfs_n8, cf_aer_wfs_n4, cf_aer_wfs_p0, cf_aer_wfs_p4, cf_aer_wfs_p8;
    Eigen::Vector3d cm_aer_bfs_n8, cm_aer_bfs_n4, cm_aer_bfs_p0, cm_aer_bfs_p4, cm_aer_bfs_p8;

    for (int i = 0; i != nel; ++i) {
        Vbeta_deg[i] = -10 + 0.1 * i;
        beta_rad = Vbeta_deg[i] * d2r;
        euler_wfsbfs_rad.set_yaw_rad(-beta_rad);

        cf_aer_wfs_n8 = euler_wfsbfs_rad * Oaero.obtain_cf_aer(vtas_mps, euler_wfsbfs_rad, delta_control_n8, w_nedbfsbfs_rps);
        cf_aer_wfs_n4 = euler_wfsbfs_rad * Oaero.obtain_cf_aer(vtas_mps, euler_wfsbfs_rad, delta_control_n4, w_nedbfsbfs_rps);
        cf_aer_wfs_p0 = euler_wfsbfs_rad * Oaero.obtain_cf_aer(vtas_mps, euler_wfsbfs_rad, delta_control_p0, w_nedbfsbfs_rps);
        cf_aer_wfs_p4 = euler_wfsbfs_rad * Oaero.obtain_cf_aer(vtas_mps, euler_wfsbfs_rad, delta_control_p4, w_nedbfsbfs_rps);
        cf_aer_wfs_p8 = euler_wfsbfs_rad * Oaero.obtain_cf_aer(vtas_mps, euler_wfsbfs_rad, delta_control_p8, w_nedbfsbfs_rps);

        cm_aer_bfs_n8 =  Oaero.obtain_cm_aer(vtas_mps, euler_wfsbfs_rad, delta_control_n8, w_nedbfsbfs_rps);
        cm_aer_bfs_n4 =  Oaero.obtain_cm_aer(vtas_mps, euler_wfsbfs_rad, delta_control_n4, w_nedbfsbfs_rps);
        cm_aer_bfs_p0 =  Oaero.obtain_cm_aer(vtas_mps, euler_wfsbfs_rad, delta_control_p0, w_nedbfsbfs_rps);
        cm_aer_bfs_p4 =  Oaero.obtain_cm_aer(vtas_mps, euler_wfsbfs_rad, delta_control_p4, w_nedbfsbfs_rps);
        cm_aer_bfs_p8 =  Oaero.obtain_cm_aer(vtas_mps, euler_wfsbfs_rad, delta_control_p8, w_nedbfsbfs_rps);

        Oout_cy_vs_beta__deltaR << std::fixed << std::showpos
                                 << std::setprecision(2) << std::setw(6) << Vbeta_deg[i]
                                 << std::setprecision(5) << std::setw(11) << - 100.0 * cf_aer_wfs_n8(1)
                                 << std::setprecision(5) << std::setw(11) << - 100.0 * cf_aer_wfs_n4(1)
                                 << std::setprecision(5) << std::setw(11) << - 100.0 * cf_aer_wfs_p0(1)
                                 << std::setprecision(5) << std::setw(11) << - 100.0 * cf_aer_wfs_p4(1)
                                 << std::setprecision(5) << std::setw(11) << - 100.0 * cf_aer_wfs_p8(1)
                                 << std::endl;
        Oout_cmn_vs_beta__deltaR << std::fixed << std::showpos
                                 << std::setprecision(2) << std::setw(6) << Vbeta_deg[i]
                                 << std::setprecision(5) << std::setw(11) << 100.0 * cm_aer_bfs_n8(2)
                                 << std::setprecision(5) << std::setw(11) << 100.0 * cm_aer_bfs_n4(2)
                                 << std::setprecision(5) << std::setw(11) << 100.0 * cm_aer_bfs_p0(2)
                                 << std::setprecision(5) << std::setw(11) << 100.0 * cm_aer_bfs_p4(2)
                                 << std::setprecision(5) << std::setw(11) << 100.0 * cm_aer_bfs_p8(2)
                                 << std::endl;
        Oout_cml_vs_beta__deltaR << std::fixed << std::showpos
                                 << std::setprecision(2) << std::setw(6) << Vbeta_deg[i]
                                 << std::setprecision(5) << std::setw(11) << 100.0 * cm_aer_bfs_n8(0)
                                 << std::setprecision(5) << std::setw(11) << 100.0 * cm_aer_bfs_n4(0)
                                 << std::setprecision(5) << std::setw(11) << 100.0 * cm_aer_bfs_p0(0)
                                 << std::setprecision(5) << std::setw(11) << 100.0 * cm_aer_bfs_p4(0)
                                 << std::setprecision(5) << std::setw(11) << 100.0 * cm_aer_bfs_p8(0)
                                 << std::endl;
    }

    Oout_cy_vs_beta__deltaR.close();
    Oout_cmn_vs_beta__deltaR.close();
    Oout_cml_vs_beta__deltaR.close();
} // closes test3_cy_cml_cmn_beta__deltaR

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void acft::test::Tapm_plots::test4_cy_cml_cmn_beta__deltaA() {
    acft::aero0 Oaero(math::logic::biparabolic);
    double d2r = math::constant::D2R();

    boost::filesystem::path path_outputs(math::share::phd_outputs_prefix);
    boost::filesystem::path path_acft_test("acft");
    boost::filesystem::path path_file_cy_vs_beta__deltaA("test4_cy_vs_beta__deltaA.txt");
    boost::filesystem::path path_file_cml_vs_beta__deltaA("test4_cml_vs_beta__deltaA.txt");
    boost::filesystem::path path_file_cmn_vs_beta__deltaA("test4_cmn_vs_beta__deltaA.txt");

    std::ofstream Oout_cy_vs_beta__deltaA, Oout_cml_vs_beta__deltaA, Oout_cmn_vs_beta__deltaA;
    Oout_cy_vs_beta__deltaA.open((path_outputs / path_acft_test / path_file_cy_vs_beta__deltaA).string());
    Oout_cml_vs_beta__deltaA.open((path_outputs / path_acft_test / path_file_cml_vs_beta__deltaA).string());
    Oout_cmn_vs_beta__deltaA.open((path_outputs / path_acft_test / path_file_cmn_vs_beta__deltaA).string());

    size_t nel = 201;
    std::vector<double>Vbeta_deg(nel);
    double beta_rad;

    ang::quat delta_control_n8(0.0, 0.0, -8.0, 0.0);
    ang::quat delta_control_n4(0.0, 0.0, -4.0, 0.0);
    ang::quat delta_control_p0(0.0, 0.0, +0.0, 0.0);
    ang::quat delta_control_p4(0.0, 0.0, +4.0, 0.0);
    ang::quat delta_control_p8(0.0, 0.0, +8.0, 0.0);

    double vtas_mps = 30.0;
    double alpha_rad = 4 * d2r;
    ang::euler euler_wfsbfs_rad(0.0, alpha_rad, 0.0);
    Eigen::Vector3d w_nedbfsbfs_rps(0.0, 0.0, 0.0);

    Eigen::Vector3d cf_aer_wfs_n8, cf_aer_wfs_n4, cf_aer_wfs_p0, cf_aer_wfs_p4, cf_aer_wfs_p8;
    Eigen::Vector3d cm_aer_bfs_n8, cm_aer_bfs_n4, cm_aer_bfs_p0, cm_aer_bfs_p4, cm_aer_bfs_p8;

    for (int i = 0; i != nel; ++i) {
        Vbeta_deg[i] = -10 + 0.1 * i;
        beta_rad = Vbeta_deg[i] * d2r;
        euler_wfsbfs_rad.set_yaw_rad(-beta_rad);

        cf_aer_wfs_n8 = euler_wfsbfs_rad * Oaero.obtain_cf_aer(vtas_mps, euler_wfsbfs_rad, delta_control_n8, w_nedbfsbfs_rps);
        cf_aer_wfs_n4 = euler_wfsbfs_rad * Oaero.obtain_cf_aer(vtas_mps, euler_wfsbfs_rad, delta_control_n4, w_nedbfsbfs_rps);
        cf_aer_wfs_p0 = euler_wfsbfs_rad * Oaero.obtain_cf_aer(vtas_mps, euler_wfsbfs_rad, delta_control_p0, w_nedbfsbfs_rps);
        cf_aer_wfs_p4 = euler_wfsbfs_rad * Oaero.obtain_cf_aer(vtas_mps, euler_wfsbfs_rad, delta_control_p4, w_nedbfsbfs_rps);
        cf_aer_wfs_p8 = euler_wfsbfs_rad * Oaero.obtain_cf_aer(vtas_mps, euler_wfsbfs_rad, delta_control_p8, w_nedbfsbfs_rps);

        cm_aer_bfs_n8 =  Oaero.obtain_cm_aer(vtas_mps, euler_wfsbfs_rad, delta_control_n8, w_nedbfsbfs_rps);
        cm_aer_bfs_n4 =  Oaero.obtain_cm_aer(vtas_mps, euler_wfsbfs_rad, delta_control_n4, w_nedbfsbfs_rps);
        cm_aer_bfs_p0 =  Oaero.obtain_cm_aer(vtas_mps, euler_wfsbfs_rad, delta_control_p0, w_nedbfsbfs_rps);
        cm_aer_bfs_p4 =  Oaero.obtain_cm_aer(vtas_mps, euler_wfsbfs_rad, delta_control_p4, w_nedbfsbfs_rps);
        cm_aer_bfs_p8 =  Oaero.obtain_cm_aer(vtas_mps, euler_wfsbfs_rad, delta_control_p8, w_nedbfsbfs_rps);

        Oout_cy_vs_beta__deltaA << std::fixed << std::showpos
                                << std::setprecision(2) << std::setw(6) << Vbeta_deg[i]
                                << std::setprecision(5) << std::setw(11) << - 100.0 * cf_aer_wfs_n8(1)
                                << std::setprecision(5) << std::setw(11) << - 100.0 * cf_aer_wfs_n4(1)
                                << std::setprecision(5) << std::setw(11) << - 100.0 * cf_aer_wfs_p0(1)
                                << std::setprecision(5) << std::setw(11) << - 100.0 * cf_aer_wfs_p4(1)
                                << std::setprecision(5) << std::setw(11) << - 100.0 * cf_aer_wfs_p8(1)
                                << std::endl;
        Oout_cml_vs_beta__deltaA << std::fixed << std::showpos
                                 << std::setprecision(2) << std::setw(6) << Vbeta_deg[i]
                                 << std::setprecision(5) << std::setw(11) << 100.0 * cm_aer_bfs_n8(0)
                                 << std::setprecision(5) << std::setw(11) << 100.0 * cm_aer_bfs_n4(0)
                                 << std::setprecision(5) << std::setw(11) << 100.0 * cm_aer_bfs_p0(0)
                                 << std::setprecision(5) << std::setw(11) << 100.0 * cm_aer_bfs_p4(0)
                                 << std::setprecision(5) << std::setw(11) << 100.0 * cm_aer_bfs_p8(0)
                                 << std::endl;

        Oout_cmn_vs_beta__deltaA << std::fixed << std::showpos
                                 << std::setprecision(2) << std::setw(6) << Vbeta_deg[i]
                                 << std::setprecision(5) << std::setw(11) << 100.0 * cm_aer_bfs_n8(2)
                                 << std::setprecision(5) << std::setw(11) << 100.0 * cm_aer_bfs_n4(2)
                                 << std::setprecision(5) << std::setw(11) << 100.0 * cm_aer_bfs_p0(2)
                                 << std::setprecision(5) << std::setw(11) << 100.0 * cm_aer_bfs_p4(2)
                                 << std::setprecision(5) << std::setw(11) << 100.0 * cm_aer_bfs_p8(2)
                                 << std::endl;

    }

    Oout_cy_vs_beta__deltaA.close();
    Oout_cml_vs_beta__deltaA.close();
    Oout_cmn_vs_beta__deltaA.close();
} // closes test4_cy_cml_cmn_beta__deltaA

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void acft::test::Tapm_plots::test5_cmn_alpha__deltaR() {
    acft::aero0 Oaero(math::logic::biparabolic);
    double d2r = math::constant::D2R();

    boost::filesystem::path path_outputs(math::share::phd_outputs_prefix);
    boost::filesystem::path path_acft_test("acft");
    boost::filesystem::path path_file_cmn_vs_alpha__deltaR("test5_cmn_vs_alpha__deltaR.txt");

    std::ofstream Oout_cmn_vs_alpha__deltaR;
    Oout_cmn_vs_alpha__deltaR.open((path_outputs / path_acft_test / path_file_cmn_vs_alpha__deltaR).string());

    size_t nel = 176;
    std::vector<double>Valpha_deg(nel);
    double alpha_rad;

    ang::quat delta_control_n8(0.0, 0.0, 0.0, -8.0);
    ang::quat delta_control_n4(0.0, 0.0, 0.0, -4.0);
    ang::quat delta_control_p0(0.0, 0.0, 0.0, +0.0);
    ang::quat delta_control_p4(0.0, 0.0, 0.0, +4.0);
    ang::quat delta_control_p8(0.0, 0.0, 0.0, +8.0);

    double vtas_mps = 30.0;
    double beta_rad = 0 * d2r;
    ang::euler euler_wfsbfs_rad(-beta_rad, 0.0, 0.0);
    Eigen::Vector3d w_nedbfsbfs_rps(0.0, 0.0, 0.0);

    Eigen::Vector3d cf_aer_wfs_n8, cf_aer_wfs_n4, cf_aer_wfs_p0, cf_aer_wfs_p4, cf_aer_wfs_p8;
    Eigen::Vector3d cm_aer_bfs_n8, cm_aer_bfs_n4, cm_aer_bfs_p0, cm_aer_bfs_p4, cm_aer_bfs_p8;

    for (int i = 0; i != nel; ++i) {
        Valpha_deg[i] = -5 + 0.1 * i;
        alpha_rad = Valpha_deg[i] * d2r;
        euler_wfsbfs_rad.set_pitch_rad(alpha_rad);

        cf_aer_wfs_n8 = euler_wfsbfs_rad * Oaero.obtain_cf_aer(vtas_mps, euler_wfsbfs_rad, delta_control_n8, w_nedbfsbfs_rps);
        cf_aer_wfs_n4 = euler_wfsbfs_rad * Oaero.obtain_cf_aer(vtas_mps, euler_wfsbfs_rad, delta_control_n4, w_nedbfsbfs_rps);
        cf_aer_wfs_p0 = euler_wfsbfs_rad * Oaero.obtain_cf_aer(vtas_mps, euler_wfsbfs_rad, delta_control_p0, w_nedbfsbfs_rps);
        cf_aer_wfs_p4 = euler_wfsbfs_rad * Oaero.obtain_cf_aer(vtas_mps, euler_wfsbfs_rad, delta_control_p4, w_nedbfsbfs_rps);
        cf_aer_wfs_p8 = euler_wfsbfs_rad * Oaero.obtain_cf_aer(vtas_mps, euler_wfsbfs_rad, delta_control_p8, w_nedbfsbfs_rps);

        cm_aer_bfs_n8 =  Oaero.obtain_cm_aer(vtas_mps, euler_wfsbfs_rad, delta_control_n8, w_nedbfsbfs_rps);
        cm_aer_bfs_n4 =  Oaero.obtain_cm_aer(vtas_mps, euler_wfsbfs_rad, delta_control_n4, w_nedbfsbfs_rps);
        cm_aer_bfs_p0 =  Oaero.obtain_cm_aer(vtas_mps, euler_wfsbfs_rad, delta_control_p0, w_nedbfsbfs_rps);
        cm_aer_bfs_p4 =  Oaero.obtain_cm_aer(vtas_mps, euler_wfsbfs_rad, delta_control_p4, w_nedbfsbfs_rps);
        cm_aer_bfs_p8 =  Oaero.obtain_cm_aer(vtas_mps, euler_wfsbfs_rad, delta_control_p8, w_nedbfsbfs_rps);

        Oout_cmn_vs_alpha__deltaR << std::fixed << std::showpos
                                 << std::setprecision(2) << std::setw(6) << Valpha_deg[i]
                                 << std::setprecision(5) << std::setw(11) << 100.0 * cm_aer_bfs_n8(2)
                                 << std::setprecision(5) << std::setw(11) << 100.0 * cm_aer_bfs_n4(2)
                                 << std::setprecision(5) << std::setw(11) << 100.0 * cm_aer_bfs_p0(2)
                                 << std::setprecision(5) << std::setw(11) << 100.0 * cm_aer_bfs_p4(2)
                                 << std::setprecision(5) << std::setw(11) << 100.0 * cm_aer_bfs_p8(2)
                                 << std::endl;
    }

    Oout_cmn_vs_alpha__deltaR.close();
} // closes test5_cmn_alpha__deltaR

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void acft::test::Tapm_plots::test6_cml_alpha__deltaA() {
    acft::aero0 Oaero(math::logic::biparabolic);
    double d2r = math::constant::D2R();

    boost::filesystem::path path_outputs(math::share::phd_outputs_prefix);
    boost::filesystem::path path_acft_test("acft");
    boost::filesystem::path path_file_cml_vs_alpha__deltaA("test6_cml_vs_alpha__deltaA.txt");

    std::ofstream Oout_cml_vs_alpha__deltaA;
    Oout_cml_vs_alpha__deltaA.open((path_outputs / path_acft_test / path_file_cml_vs_alpha__deltaA).string());

    size_t nel = 176;
    std::vector<double>Valpha_deg(nel);
    double alpha_rad;

    ang::quat delta_control_n8(0.0, 0.0, -8.0, 0.0);
    ang::quat delta_control_n4(0.0, 0.0, -4.0, 0.0);
    ang::quat delta_control_p0(0.0, 0.0, +0.0, 0.0);
    ang::quat delta_control_p4(0.0, 0.0, +4.0, 0.0);
    ang::quat delta_control_p8(0.0, 0.0, +8.0, 0.0);

    double vtas_mps = 30.0;
    double beta_rad = 0 * d2r;
    ang::euler euler_wfsbfs_rad(-beta_rad, 0.0, 0.0);
    Eigen::Vector3d w_nedbfsbfs_rps(0.0, 0.0, 0.0);

    Eigen::Vector3d cf_aer_wfs_n8, cf_aer_wfs_n4, cf_aer_wfs_p0, cf_aer_wfs_p4, cf_aer_wfs_p8;
    Eigen::Vector3d cm_aer_bfs_n8, cm_aer_bfs_n4, cm_aer_bfs_p0, cm_aer_bfs_p4, cm_aer_bfs_p8;

    for (int i = 0; i != nel; ++i) {
        Valpha_deg[i] = -5 + 0.1 * i;
        alpha_rad = Valpha_deg[i] * d2r;
        euler_wfsbfs_rad.set_pitch_rad(alpha_rad);

        cf_aer_wfs_n8 = euler_wfsbfs_rad * Oaero.obtain_cf_aer(vtas_mps, euler_wfsbfs_rad, delta_control_n8, w_nedbfsbfs_rps);
        cf_aer_wfs_n4 = euler_wfsbfs_rad * Oaero.obtain_cf_aer(vtas_mps, euler_wfsbfs_rad, delta_control_n4, w_nedbfsbfs_rps);
        cf_aer_wfs_p0 = euler_wfsbfs_rad * Oaero.obtain_cf_aer(vtas_mps, euler_wfsbfs_rad, delta_control_p0, w_nedbfsbfs_rps);
        cf_aer_wfs_p4 = euler_wfsbfs_rad * Oaero.obtain_cf_aer(vtas_mps, euler_wfsbfs_rad, delta_control_p4, w_nedbfsbfs_rps);
        cf_aer_wfs_p8 = euler_wfsbfs_rad * Oaero.obtain_cf_aer(vtas_mps, euler_wfsbfs_rad, delta_control_p8, w_nedbfsbfs_rps);

        cm_aer_bfs_n8 =  Oaero.obtain_cm_aer(vtas_mps, euler_wfsbfs_rad, delta_control_n8, w_nedbfsbfs_rps);
        cm_aer_bfs_n4 =  Oaero.obtain_cm_aer(vtas_mps, euler_wfsbfs_rad, delta_control_n4, w_nedbfsbfs_rps);
        cm_aer_bfs_p0 =  Oaero.obtain_cm_aer(vtas_mps, euler_wfsbfs_rad, delta_control_p0, w_nedbfsbfs_rps);
        cm_aer_bfs_p4 =  Oaero.obtain_cm_aer(vtas_mps, euler_wfsbfs_rad, delta_control_p4, w_nedbfsbfs_rps);
        cm_aer_bfs_p8 =  Oaero.obtain_cm_aer(vtas_mps, euler_wfsbfs_rad, delta_control_p8, w_nedbfsbfs_rps);

        Oout_cml_vs_alpha__deltaA << std::fixed << std::showpos
                                  << std::setprecision(2) << std::setw(6) << Valpha_deg[i]
                                  << std::setprecision(5) << std::setw(11) << 100.0 * cm_aer_bfs_n8(0)
                                  << std::setprecision(5) << std::setw(11) << 100.0 * cm_aer_bfs_n4(0)
                                  << std::setprecision(5) << std::setw(11) << 100.0 * cm_aer_bfs_p0(0)
                                  << std::setprecision(5) << std::setw(11) << 100.0 * cm_aer_bfs_p4(0)
                                  << std::setprecision(5) << std::setw(11) << 100.0 * cm_aer_bfs_p8(0)
                                  << std::endl;
    }

    Oout_cml_vs_alpha__deltaA.close();
} // closes test6_cml_alpha__deltaA

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void acft::test::Tapm_plots::test7_P_F_Hp__deltaT() {
    acft::prop Oprop;
    env::atm Oam_isa(0.0, 0.0);

    boost::filesystem::path path_outputs(math::share::phd_outputs_prefix);
    boost::filesystem::path path_acft_test("acft");
    boost::filesystem::path path_file_P_vs_Hp__deltaT("test7_P_vs_Hp__deltaT.txt");
    boost::filesystem::path path_file_F_vs_Hp__deltaT("test7_F_vs_Hp__deltaT.txt");

    std::ofstream Oout_P_vs_Hp__deltaT, Oout_F_vs_Hp__deltaT;
    Oout_P_vs_Hp__deltaT.open((path_outputs / path_acft_test / path_file_P_vs_Hp__deltaT).string());
    Oout_F_vs_Hp__deltaT.open((path_outputs / path_acft_test / path_file_F_vs_Hp__deltaT).string());

    size_t nel = 51;
    std::vector<double>VHp_m(nel);

    double P_W_100, P_W_090, P_W_080, P_W_070, P_W_060;
    double F_kgphr_100, F_kgphr_090, F_kgphr_080, F_kgphr_070, F_kgphr_060;

    for (int i = 0; i != nel; ++i) {
        VHp_m[i] = 0 + 100.0 * i;

        double p_pa = env::atm::Hp2p(VHp_m[i]);
        double T_degK = Oam_isa.Hp2T(VHp_m[i]);

        P_W_100 = Oprop.compute_engine_power(1.0, p_pa, T_degK);
        P_W_090 = Oprop.compute_engine_power(0.9, p_pa, T_degK);
        P_W_080 = Oprop.compute_engine_power(0.8, p_pa, T_degK);
        P_W_070 = Oprop.compute_engine_power(0.7, p_pa, T_degK);
        P_W_060 = Oprop.compute_engine_power(0.6, p_pa, T_degK);

        F_kgphr_100 = - 3600 * Oprop.compute_fuel_consumption(P_W_100);
        F_kgphr_090 = - 3600 * Oprop.compute_fuel_consumption(P_W_090);
        F_kgphr_080 = - 3600 * Oprop.compute_fuel_consumption(P_W_080);
        F_kgphr_070 = - 3600 * Oprop.compute_fuel_consumption(P_W_070);
        F_kgphr_060 = - 3600 * Oprop.compute_fuel_consumption(P_W_060);

        Oout_P_vs_Hp__deltaT << std::fixed << std::showpos
                                 << std::setprecision(0) << std::setw(11) << VHp_m[i]
                                 << std::setprecision(5) << std::setw(15) << 1e-3 * P_W_100
                                 << std::setprecision(5) << std::setw(15) << 1e-3 * P_W_090
                                 << std::setprecision(5) << std::setw(15) << 1e-3 * P_W_080
                                 << std::setprecision(5) << std::setw(15) << 1e-3 * P_W_070
                                 << std::setprecision(5) << std::setw(15) << 1e-3 * P_W_060
                                 << std::endl;

        Oout_F_vs_Hp__deltaT << std::fixed << std::showpos
                             << std::setprecision(0) << std::setw(11) << VHp_m[i]
                             << std::setprecision(7) << std::setw(15) << F_kgphr_100
                             << std::setprecision(7) << std::setw(15) << F_kgphr_090
                             << std::setprecision(7) << std::setw(15) << F_kgphr_080
                             << std::setprecision(7) << std::setw(15) << F_kgphr_070
                             << std::setprecision(7) << std::setw(15) << F_kgphr_060
                             << std::endl;
    }

    Oout_P_vs_Hp__deltaT.close();
    Oout_F_vs_Hp__deltaT.close();
} // closes test7_P_F_Hp__deltaT

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void acft::test::Tapm_plots::test8_ct_cp_eta__J() {
    acft::prop Oprop;
    env::atm Oam_isa(0.0, 0.0);

    boost::filesystem::path path_outputs(math::share::phd_outputs_prefix);
    boost::filesystem::path path_acft_test("acft");
    boost::filesystem::path path_file_Ct_Cp_eta_vs_J("test8_Ct_Cp_eta_vs_J.txt");

    std::ofstream Oout_Ct_Cp_eta_vs_J;
    Oout_Ct_Cp_eta_vs_J.open((path_outputs / path_acft_test / path_file_Ct_Cp_eta_vs_J).string());

    size_t nel = 96;
    std::vector<double>VJ(nel);
    double Ct, Cp, eta;

    for (int i = 0; i != nel; ++i) {
        VJ[i] = 0.2 + 0.01 * i;

        Ct = Oprop.compute_ct(VJ[i]);
        Cp = Oprop.compute_cp(VJ[i]);
        eta = acft::prop::compute_eta(Ct, Cp, VJ[i]);

        Oout_Ct_Cp_eta_vs_J << std::fixed << std::showpos
                             << std::setprecision(2) << std::setw(11) << VJ[i]
                             << std::setprecision(5) << std::setw(15) << 10.0 * Ct
                             << std::setprecision(5) << std::setw(15) << 10.0 * Cp
                             << std::setprecision(5) << std::setw(15) << eta
                             << std::endl;
    }
    Oout_Ct_Cp_eta_vs_J.close();
} // closes test8_ct_cp_eta__J

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void acft::test::Tapm_plots::test9_n_T_M_P__vtas() {
    acft::prop Oprop;
    env::atm Oam_isa(0.0, 0.0);

    boost::filesystem::path path_outputs(math::share::phd_outputs_prefix);
    boost::filesystem::path path_acft_test("acft");
    boost::filesystem::path path_file_n_vs_P__vtas("test9_n_vs_P__vtas.txt");
    boost::filesystem::path path_file_T_vs_P__vtas("test9_T_vs_P__vtas.txt");
    boost::filesystem::path path_file_M_vs_P__vtas("test9_M_vs_P__vtas.txt");

    std::ofstream Oout_n_vs_P__vtas, Oout_T_vs_P__vtas, Oout_M_vs_P__vtas;
    Oout_n_vs_P__vtas.open((path_outputs / path_acft_test / path_file_n_vs_P__vtas).string());
    Oout_T_vs_P__vtas.open((path_outputs / path_acft_test / path_file_T_vs_P__vtas).string());
    Oout_M_vs_P__vtas.open((path_outputs / path_acft_test / path_file_M_vs_P__vtas).string());

    size_t nel = 41;
    std::vector<double>VP_W(nel);

    double Hp_m = 1000.0;
    double p_pa = env::atm::Hp2p(Hp_m);
    double T_degK = Oam_isa.Hp2T(Hp_m);
    double rho_kgm3 = env::atm::pT2rho(T_degK, p_pa);

    double vtas_mps_30 = 30.0;
    double vtas_mps_35 = 35.0;
    double vtas_mps_40 = 40.0;
    double vtas_mps_45 = 45.0;
    double vtas_mps_50 = 50.0;

    for (int i = 0; i != nel; ++i) {
        VP_W[i] = 2000 + 50 * i;

        double n_revps_30 = Oprop.compute_engine_speed(VP_W[i], vtas_mps_30, rho_kgm3);
        double n_revps_35 = Oprop.compute_engine_speed(VP_W[i], vtas_mps_35, rho_kgm3);
        double n_revps_40 = Oprop.compute_engine_speed(VP_W[i], vtas_mps_40, rho_kgm3);
        double n_revps_45 = Oprop.compute_engine_speed(VP_W[i], vtas_mps_45, rho_kgm3);
        double n_revps_50 = Oprop.compute_engine_speed(VP_W[i], vtas_mps_50, rho_kgm3);

        Eigen::Vector3d f_pro_bfs_N_30 = Oprop.compute_propulsion_force(n_revps_30, vtas_mps_30, rho_kgm3);
        Eigen::Vector3d f_pro_bfs_N_35 = Oprop.compute_propulsion_force(n_revps_35, vtas_mps_35, rho_kgm3);
        Eigen::Vector3d f_pro_bfs_N_40 = Oprop.compute_propulsion_force(n_revps_40, vtas_mps_40, rho_kgm3);
        Eigen::Vector3d f_pro_bfs_N_45 = Oprop.compute_propulsion_force(n_revps_45, vtas_mps_45, rho_kgm3);
        Eigen::Vector3d f_pro_bfs_N_50 = Oprop.compute_propulsion_force(n_revps_50, vtas_mps_50, rho_kgm3);

        Eigen::Vector3d m_pro_bfs_Nm_30 = Oprop.compute_propulsion_moment(f_pro_bfs_N_30, n_revps_30, VP_W[i]);
        Eigen::Vector3d m_pro_bfs_Nm_35 = Oprop.compute_propulsion_moment(f_pro_bfs_N_35, n_revps_35, VP_W[i]);
        Eigen::Vector3d m_pro_bfs_Nm_40 = Oprop.compute_propulsion_moment(f_pro_bfs_N_40, n_revps_40, VP_W[i]);
        Eigen::Vector3d m_pro_bfs_Nm_45 = Oprop.compute_propulsion_moment(f_pro_bfs_N_45, n_revps_45, VP_W[i]);
        Eigen::Vector3d m_pro_bfs_Nm_50 = Oprop.compute_propulsion_moment(f_pro_bfs_N_50, n_revps_50, VP_W[i]);

        Oout_n_vs_P__vtas << std::fixed << std::showpos
                           << std::setprecision(3) << std::setw(15) << 1e-3 * VP_W[i]
                           << std::setprecision(3) << std::setw(15) << 60 * n_revps_30
                           << std::setprecision(3) << std::setw(15) << 60 * n_revps_35
                           << std::setprecision(3) << std::setw(15) << 60 * n_revps_40
                           << std::setprecision(3) << std::setw(15) << 60 * n_revps_45
                           << std::setprecision(3) << std::setw(15) << 60 * n_revps_50
                           << std::endl;

        Oout_T_vs_P__vtas << std::fixed << std::showpos
                          << std::setprecision(3) << std::setw(15) << 1e-3 * VP_W[i]
                          << std::setprecision(3) << std::setw(15) << f_pro_bfs_N_30(0)
                          << std::setprecision(3) << std::setw(15) << f_pro_bfs_N_35(0)
                          << std::setprecision(3) << std::setw(15) << f_pro_bfs_N_40(0)
                          << std::setprecision(3) << std::setw(15) << f_pro_bfs_N_45(0)
                          << std::setprecision(3) << std::setw(15) << f_pro_bfs_N_50(0)
                          << std::endl;

        Oout_M_vs_P__vtas << std::fixed << std::showpos
                          << std::setprecision(3) << std::setw(15) << 1e-3 * VP_W[i]
                          << std::setprecision(3) << std::setw(15) << m_pro_bfs_Nm_30(0)
                          << std::setprecision(3) << std::setw(15) << m_pro_bfs_Nm_35(0)
                          << std::setprecision(3) << std::setw(15) << m_pro_bfs_Nm_40(0)
                          << std::setprecision(3) << std::setw(15) << m_pro_bfs_Nm_45(0)
                          << std::setprecision(3) << std::setw(15) << m_pro_bfs_Nm_50(0)
                          << std::endl;

    }

    Oout_n_vs_P__vtas.close();
    Oout_T_vs_P__vtas.close();
    Oout_M_vs_P__vtas.close();
} // closes test9_n_T_M_P__vtas

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////






















