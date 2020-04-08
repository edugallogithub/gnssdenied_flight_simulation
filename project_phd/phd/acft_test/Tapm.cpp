#include "Tapm.h"

#include "acft/acft/prop.h"
#include "acft/acft/iner.h"
#include "acft/acft/aero3.h"
#include "acft/acft/aero2.h"
#include "acft/acft/aero1.h"
#include "acft/acft/aero0.h"
#include "ang/quat.h"

acft::test::Tapm::Tapm(jail::counter& Ocounter)
: ::jail::unit_test(Ocounter) {
}
/* constructor based on counter */

void acft::test::Tapm::run() {
	::jail::unit_test::run();

    test_aero0();
    test_aero1();
    test_aero2();
    test_aero3();
    test_prop();
    test_iner();

	finished();
}
/* execute tests and write results on console */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void acft::test::Tapm::test_aero0() {
    double rho_kgm3 = 1.225;
    double vtas_mps = 40.0;
    ang::euler euler_wfsbfs_rad(-3.8048177693476383e-02, 2.2863813201125717e-02, 0.0000000000000000e+00);
    Eigen::Vector3d w_nedbfsbfs_rps(1.0821041362364843e-02, -9.8320214529013881e-03, 1.2042771838760874e-02);
    Eigen::Array4d delta_control;
    delta_control << 7.1999999999999997e-01, 5.4000000000000004e+00, -3.7999999999999998e+00, 2.8999999999999999e+00;
    Eigen::Array3d Target_cf_aer_bfs(-1.2901514368404546e-01, -3.3336829526298059e-02, -6.9227994821190020e-01);
    Eigen::Array3d Target_f_aer_bfs_N(-1.1095163020472732e+02, -2.8669313354857451e+01, -5.9535327883879359e+02);
    Eigen::Array3d Target_cm_aer_bfs(-1.8239226614967959e-02, -2.1778229430921459e-01, 1.3230796787692281e-02);
    Eigen::Array3d Target_m_aer_bfs_Nm(-4.2037241586003020e+01, -6.7226023734251982e+01, 3.0493957484090952e+01);

    /////////////////////////////////////////////////////////
    //euler_wfsbfs_rad.get_pitch_rad() = -5 * math::constant::PI() / 180;
    //euler_wfsbfs_rad.get_yaw_rad()   = 10 * math::constant::PI() / 180;
    //delta_control(1) = -8;

    //euler_wfsbfs_rad.get_pitch_rad() = 12.5 * math::constant::PI() / 180;
    //euler_wfsbfs_rad.get_yaw_rad()   = -10 * math::constant::PI() / 180;
    //delta_control(1) = -8;

    //euler_wfsbfs_rad.get_pitch_rad() = -5 * math::constant::PI() / 180;
    //euler_wfsbfs_rad.get_yaw_rad()   = -10 * math::constant::PI() / 180;
    //delta_control(1) = -8;
    /////////////////////////////////////////////////////////

    acft::aero0 Oaero(math::logic::lagrange_first);
    Eigen::Vector3d cf_aer_bfs   = Oaero.obtain_cf_aer(vtas_mps, euler_wfsbfs_rad, delta_control, w_nedbfsbfs_rps);
    Eigen::Vector3d f_aer_bfs_N  = Oaero.cfaer2faer(cf_aer_bfs, rho_kgm3, vtas_mps);
    Eigen::Vector3d cm_aer_bfs   = Oaero.obtain_cm_aer(vtas_mps, euler_wfsbfs_rad, delta_control, w_nedbfsbfs_rps);
    Eigen::Vector3d m_aer_bfs_Nm = Oaero.cmaer2maer(cm_aer_bfs, rho_kgm3, vtas_mps);

    check("AFM0 cf_aer_bfs1           ", cf_aer_bfs(0), Target_cf_aer_bfs(0), 1e-12);
    check("AFM0 cf_aer_bfs2           ", cf_aer_bfs(1), Target_cf_aer_bfs(1), 1e-12);
    check("AFM0 cf_aer_bfs3           ", cf_aer_bfs(2), Target_cf_aer_bfs(2), 1e-12);
    check("AFM0 f_aer_bfs1            ", f_aer_bfs_N(0), Target_f_aer_bfs_N(0), 1e-12);
    check("AFM0 f_aer_bfs2            ", f_aer_bfs_N(1), Target_f_aer_bfs_N(1), 1e-12);
    check("AFM0 f_aer_bfs3            ", f_aer_bfs_N(2), Target_f_aer_bfs_N(2), 1e-12);
    check("AFM0 cm_aer_bfs1           ", cm_aer_bfs(0), Target_cm_aer_bfs(0), 1e-12);
    check("AFM0 cm_aer_bfs2           ", cm_aer_bfs(1), Target_cm_aer_bfs(1), 1e-12);
    check("AFM0 cm_aer_bfs3           ", cm_aer_bfs(2), Target_cm_aer_bfs(2), 1e-12);
    check("AFM0 m_aer_bfs1            ", m_aer_bfs_Nm(0), Target_m_aer_bfs_Nm(0), 1e-12);
    check("AFM0 m_aer_bfs2            ", m_aer_bfs_Nm(1), Target_m_aer_bfs_Nm(1), 1e-12);
    check("AFM0 m_aer_bfs3            ", m_aer_bfs_Nm(2), Target_m_aer_bfs_Nm(2), 1e-12);
} // closes test_aero0

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void acft::test::Tapm::test_aero1() {
    double rho_kgm3 = 1.225;
    double vtas_mps = 40.0;
    ang::euler euler_wfsbfs_rad(-3.8048177693476383e-02, 2.2863813201125717e-02, 0.0000000000000000e+00);
    Eigen::Vector3d w_nedbfsbfs_rps(1.0821041362364843e-02, -9.8320214529013881e-03, 1.2042771838760874e-02);
    Eigen::Array4d delta_control;
    delta_control << 7.1999999999999997e-01, 5.4000000000000004e+00, -3.7999999999999998e+00, 2.8999999999999999e+00;
    Eigen::Array3d Target_cf_aer_bfs(-1.2907478788780921e-01, -3.3078726046077062e-02, -6.9229142578557923e-01);
    Eigen::Array3d Target_f_aer_bfs_N(-1.1100292357580675e+02, -2.8447347149384978e+01, -5.9536314942819968e+02);
    Eigen::Array3d Target_cm_aer_bfs(-1.8206638815483768e-02, -2.1769397877474575e-01, 1.2772414700997790e-02);
    Eigen::Array3d Target_m_aer_bfs_Nm(-4.1962134168973122e+01, -6.7198762095581415e+01, 2.9437491718088609e+01);

    /////////////////////////////////////////////////////////
    //euler_wfsbfs_rad.get_pitch_rad() = -5 * math::constant::PI() / 180;
    //delta_control(1) = -8;
    //euler_wfsbfs_rad.get_pitch_rad() = 12.5 * math::constant::PI() / 180;
    //delta_control(1) = -8;

    /////////////////////////////////////////////////////////

    acft::aero1 Oaero (math::logic::lagrange_first);
    Eigen::Vector3d cf_aer_bfs   = Oaero.obtain_cf_aer(vtas_mps, euler_wfsbfs_rad, delta_control, w_nedbfsbfs_rps);
    Eigen::Vector3d f_aer_bfs_N  = Oaero.cfaer2faer(cf_aer_bfs, rho_kgm3, vtas_mps);
    Eigen::Vector3d cm_aer_bfs   = Oaero.obtain_cm_aer(vtas_mps, euler_wfsbfs_rad, delta_control, w_nedbfsbfs_rps);
    Eigen::Vector3d m_aer_bfs_Nm = Oaero.cmaer2maer(cm_aer_bfs, rho_kgm3, vtas_mps);

    check("AFM1 cf_aer_bfs1           ", cf_aer_bfs(0), Target_cf_aer_bfs(0), 1e-12);
    check("AFM1 cf_aer_bfs2           ", cf_aer_bfs(1), Target_cf_aer_bfs(1), 1e-12);
    check("AFM1 cf_aer_bfs3           ", cf_aer_bfs(2), Target_cf_aer_bfs(2), 1e-12);
    check("AFM1 f_aer_bfs1            ", f_aer_bfs_N(0), Target_f_aer_bfs_N(0), 1e-12);
    check("AFM1 f_aer_bfs2            ", f_aer_bfs_N(1), Target_f_aer_bfs_N(1), 1e-12);
    check("AFM1 f_aer_bfs3            ", f_aer_bfs_N(2), Target_f_aer_bfs_N(2), 1e-12);
    check("AFM1 cm_aer_bfs1           ", cm_aer_bfs(0), Target_cm_aer_bfs(0), 1e-12);
    check("AFM1 cm_aer_bfs2           ", cm_aer_bfs(1), Target_cm_aer_bfs(1), 1e-12);
    check("AFM1 cm_aer_bfs3           ", cm_aer_bfs(2), Target_cm_aer_bfs(2), 1e-12);
    check("AFM1 m_aer_bfs1            ", m_aer_bfs_Nm(0), Target_m_aer_bfs_Nm(0), 1e-12);
    check("AFM1 m_aer_bfs2            ", m_aer_bfs_Nm(1), Target_m_aer_bfs_Nm(1), 1e-12);
    check("AFM1 m_aer_bfs3            ", m_aer_bfs_Nm(2), Target_m_aer_bfs_Nm(2), 1e-12);
} // closes test_aero1

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void acft::test::Tapm::test_aero2() {
    double rho_kgm3 = 1.225;
    double vtas_mps = 40.0;
    ang::euler euler_wfsbfs_rad(-3.8048177693476383e-02, 2.2863813201125717e-02, 0.0000000000000000e+00);
    Eigen::Vector3d w_nedbfsbfs_rps(1.0821041362364843e-02, -9.8320214529013881e-03, 1.2042771838760874e-02);
    Eigen::Array4d delta_control;
    delta_control << 7.1999999999999997e-01, 5.4000000000000004e+00, -3.7999999999999998e+00, 2.8999999999999999e+00;
    Eigen::Array3d Target_cf_aer_bfs(-1.2780261909914345e-01, -3.3142473008833488e-02, -6.9256665101129133e-01);
    Eigen::Array3d Target_f_aer_bfs_N(-1.0990887215697711e+02, -2.8502168848888306e+01, -5.9559984014987970e+02);
    Eigen::Array3d Target_cm_aer_bfs(-1.8124047265447356e-02, -2.1943750483540680e-01, 1.2826135803933846e-02);
    Eigen::Array3d Target_m_aer_bfs_Nm(-4.1771779554979027e+01, -6.7736961606735747e+01, 2.9561306561232023e+01);

    acft::aero2 Oaero(math::logic::lagrange_first);
    Eigen::Vector3d cf_aer_bfs   = Oaero.obtain_cf_aer(vtas_mps, euler_wfsbfs_rad, delta_control, w_nedbfsbfs_rps);
    Eigen::Vector3d f_aer_bfs_N  = Oaero.cfaer2faer(cf_aer_bfs, rho_kgm3, vtas_mps);
    Eigen::Vector3d cm_aer_bfs   = Oaero.obtain_cm_aer(vtas_mps, euler_wfsbfs_rad, delta_control, w_nedbfsbfs_rps);
    Eigen::Vector3d m_aer_bfs_Nm = Oaero.cmaer2maer(cm_aer_bfs, rho_kgm3, vtas_mps);

    check("AFM2 cf_aer_bfs1           ", cf_aer_bfs(0), Target_cf_aer_bfs(0), 1e-12);
    check("AFM2 cf_aer_bfs2           ", cf_aer_bfs(1), Target_cf_aer_bfs(1), 1e-12);
    check("AFM2 cf_aer_bfs3           ", cf_aer_bfs(2), Target_cf_aer_bfs(2), 1e-12);
    check("AFM2 f_aer_bfs1            ", f_aer_bfs_N(0), Target_f_aer_bfs_N(0), 1e-12);
    check("AFM2 f_aer_bfs2            ", f_aer_bfs_N(1), Target_f_aer_bfs_N(1), 1e-12);
    check("AFM2 f_aer_bfs3            ", f_aer_bfs_N(2), Target_f_aer_bfs_N(2), 1e-12);
    check("AFM2 cm_aer_bfs1           ", cm_aer_bfs(0), Target_cm_aer_bfs(0), 1e-12);
    check("AFM2 cm_aer_bfs2           ", cm_aer_bfs(1), Target_cm_aer_bfs(1), 1e-12);
    check("AFM2 cm_aer_bfs3           ", cm_aer_bfs(2), Target_cm_aer_bfs(2), 1e-12);
    check("AFM2 m_aer_bfs1            ", m_aer_bfs_Nm(0), Target_m_aer_bfs_Nm(0), 1e-12);
    check("AFM2 m_aer_bfs2            ", m_aer_bfs_Nm(1), Target_m_aer_bfs_Nm(1), 1e-12);
    check("AFM2 m_aer_bfs3            ", m_aer_bfs_Nm(2), Target_m_aer_bfs_Nm(2), 1e-12);
} // closes test_aero2

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void acft::test::Tapm::test_aero3() {
	double rho_kgm3 = 1.225;
	double vtas_mps = 40.0;
	ang::euler euler_wfsbfs_rad(-3.8048177693476383e-02, 2.2863813201125717e-02, 0.0000000000000000e+00);
	Eigen::Vector3d w_nedbfsbfs_rps(1.0821041362364843e-02, -9.8320214529013881e-03, 1.2042771838760874e-02);
	ang::quat delta_control;
    delta_control << 7.1999999999999997e-01, 5.4000000000000004e+00, -3.7999999999999998e+00, 2.8999999999999999e+00;
	Eigen::Array3d Target_cf_aer_bfs(-1.3121864991740020e-01, -3.3170356280316511e-02, -6.9348591792556247e-01);
	Eigen::Array3d Target_f_aer_bfs_N(-1.1284662176754507e+02, -2.8526148161224377e+01, -5.9639039976807021e+02);
	Eigen::Array3d Target_cm_aer_bfs(-1.7924198521573377e-02, -2.1983727070338435e-01, 1.2925597975203313e-02);
	Eigen::Array3d Target_m_aer_bfs_Nm(-4.1311173954520321e+01, -6.7860363143183164e+01, 2.9790544094740806e+01);

	acft::aero3 Oaero;
	Eigen::Vector3d cf_aer_bfs   = Oaero.obtain_cf_aer(vtas_mps, euler_wfsbfs_rad, delta_control, w_nedbfsbfs_rps);
	Eigen::Vector3d f_aer_bfs_N  = Oaero.cfaer2faer(cf_aer_bfs, rho_kgm3, vtas_mps);
	Eigen::Vector3d cm_aer_bfs   = Oaero.obtain_cm_aer(vtas_mps, euler_wfsbfs_rad, delta_control, w_nedbfsbfs_rps);
	Eigen::Vector3d m_aer_bfs_Nm = Oaero.cmaer2maer(cm_aer_bfs, rho_kgm3, vtas_mps);

	check("AFM3 cf_aer_bfs1           ", cf_aer_bfs(0), Target_cf_aer_bfs(0), 1e-12);
	check("AFM3 cf_aer_bfs2           ", cf_aer_bfs(1), Target_cf_aer_bfs(1), 1e-12);
	check("AFM3 cf_aer_bfs3           ", cf_aer_bfs(2), Target_cf_aer_bfs(2), 1e-12);
	check("AFM3 f_aer_bfs1            ", f_aer_bfs_N(0), Target_f_aer_bfs_N(0), 1e-12);
	check("AFM3 f_aer_bfs2            ", f_aer_bfs_N(1), Target_f_aer_bfs_N(1), 1e-12);
	check("AFM3 f_aer_bfs3            ", f_aer_bfs_N(2), Target_f_aer_bfs_N(2), 1e-12);
	check("AFM3 cm_aer_bfs1           ", cm_aer_bfs(0), Target_cm_aer_bfs(0), 1e-12);
	check("AFM3 cm_aer_bfs2           ", cm_aer_bfs(1), Target_cm_aer_bfs(1), 1e-12);
	check("AFM3 cm_aer_bfs3           ", cm_aer_bfs(2), Target_cm_aer_bfs(2), 1e-12);
	check("AFM3 m_aer_bfs1            ", m_aer_bfs_Nm(0), Target_m_aer_bfs_Nm(0), 1e-12);
	check("AFM3 m_aer_bfs2            ", m_aer_bfs_Nm(1), Target_m_aer_bfs_Nm(1), 1e-12);
	check("AFM3 m_aer_bfs3            ", m_aer_bfs_Nm(2), Target_m_aer_bfs_Nm(2), 1e-12);
} // closes test_aero3

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void acft::test::Tapm::test_prop() {
    double vtas_mps = 40.0;
    double deltaT = 0.74;
    double p_pa = 98500.0;
    double T_degK = 265.0;
    double rho_kgm3 = 1.225;
    Eigen::Vector3d Target_f_pro_bfs_N(6.2747464665684483e+01, 0.0, 0.0);
    Eigen::Vector3d Target_m_pro_bfs_Nm(-4.8066928468893080e+00, 0.0, 0.0);
    double Target_dm_dt_kgps = -3.0931999999999997e-04;
    double Target_n_revps = 1.0241929027989494e+02;

    acft::prop Oprop;

    double P_W = Oprop.compute_engine_power(deltaT, p_pa, T_degK);
    double n_revps = Oprop.compute_engine_speed(P_W, vtas_mps, rho_kgm3);
    check("MUGP n_revps             ", n_revps, Target_n_revps, 1e-06);
    Eigen::Vector3d f_pro_bfs_N = Oprop.compute_propulsion_force(n_revps, vtas_mps, rho_kgm3);
    check("MUGP f_pro_bfs_N1        ", f_pro_bfs_N(0), Target_f_pro_bfs_N(0), 1e-06);
    check("MUGP f_pro_bfs_N2        ", f_pro_bfs_N(1), Target_f_pro_bfs_N(1), 1e-06);
    check("MUGP f_pro_bfs_N3        ", f_pro_bfs_N(2), Target_f_pro_bfs_N(2), 1e-06);
    Eigen::Vector3d m_pro_bfs_Nm = Oprop.compute_propulsion_moment(f_pro_bfs_N, n_revps, P_W);
    check("MUGP m_pro_bfs_Nm1       ", m_pro_bfs_Nm(0), Target_m_pro_bfs_Nm(0), 1e-07);
    check("MUGP m_pro_bfs_Nm2       ", m_pro_bfs_Nm(1), Target_m_pro_bfs_Nm(1), 1e-07);
    check("MUGP m_pro_bfs_Nm3       ", m_pro_bfs_Nm(2), Target_m_pro_bfs_Nm(2), 1e-07);
    double dm_dt_kgps = Oprop.compute_fuel_consumption(P_W);
    check("MUGP dm_dt               ", dm_dt_kgps, Target_dm_dt_kgps, 1e-12);
} // closes test_prop

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void acft::test::Tapm::test_iner() {
    acft::iner Oiner;
	check("MUGI mass                ", Oiner.get_full_m_kg(), 19.715, 1e-12);
	check("MUGI xcg_m1              ", Oiner.get_full_Trbb_m()(0), 0.207, 1e-12);
	check("MUGI xcg_m2              ", Oiner.get_full_Trbb_m()(1), 0, 1e-12);
	check("MUGI xcg_m3              ", Oiner.get_full_Trbb_m()(2), -0.005, 1e-12);
	check("MUGI I_kgm2_11           ", Oiner.get_full_I_kgm2()(0,0), 2.202, 1e-12);
	check("MUGI I_kgm2_12           ", Oiner.get_full_I_kgm2()(0,1), 0, 1e-12);
	check("MUGI I_kgm2_13           ", Oiner.get_full_I_kgm2()(0,2), -0.191, 1e-12);
	check("MUGI I_kgm2_21           ", Oiner.get_full_I_kgm2()(1,0), 0, 1e-12);
	check("MUGI I_kgm2_22           ", Oiner.get_full_I_kgm2()(1,1), 3.462, 1e-12);
	check("MUGI I_kgm2_23           ", Oiner.get_full_I_kgm2()(1,2), 0, 1e-12);
	check("MUGI I_kgm2_31           ", Oiner.get_full_I_kgm2()(2,0), -0.191, 1e-12);
	check("MUGI I_kgm2_32           ", Oiner.get_full_I_kgm2()(2,1), 0, 1e-12);
	check("MUGI I_kgm2_33           ", Oiner.get_full_I_kgm2()(2,2), 5.490, 1e-12);

    check("MUGI mass                ", Oiner.get_empty_m_kg(), 17.835, 1e-12);
    check("MUGI xcg_m1              ", Oiner.get_empty_Trbb_m()(0), 0.219, 1e-12);
    check("MUGI xcg_m2              ", Oiner.get_empty_Trbb_m()(1), 0, 1e-12);
    check("MUGI xcg_m3              ", Oiner.get_empty_Trbb_m()(2), -0.006, 1e-12);
    check("MUGI I_kgm2_11           ", Oiner.get_empty_I_kgm2()(0,0), 2.198, 1e-12);
    check("MUGI I_kgm2_12           ", Oiner.get_empty_I_kgm2()(0,1), 0, 1e-12);
    check("MUGI I_kgm2_13           ", Oiner.get_empty_I_kgm2()(0,2), -0.192, 1e-12);
    check("MUGI I_kgm2_21           ", Oiner.get_empty_I_kgm2()(1,0), 0, 1e-12);
    check("MUGI I_kgm2_22           ", Oiner.get_empty_I_kgm2()(1,1), 3.430, 1e-12);
    check("MUGI I_kgm2_23           ", Oiner.get_empty_I_kgm2()(1,2), 0, 1e-12);
    check("MUGI I_kgm2_31           ", Oiner.get_empty_I_kgm2()(2,0), -0.192, 1e-12);
    check("MUGI I_kgm2_32           ", Oiner.get_empty_I_kgm2()(2,1), 0, 1e-12);
    check("MUGI I_kgm2_33           ", Oiner.get_empty_I_kgm2()(2,2), 5.458, 1e-12);

    double ratio = Oiner.get_ratio((19.715 * 2 + 17.835) / 3);

    check("MUGI mass                ", Oiner.get_m_kg(ratio), (19.715 * 2 + 17.835) / 3, 1e-12);
    check("MUGI xcg_m1              ", Oiner.get_Trbb_m(ratio)(0), (0.207 * 2 + 0.219) / 3, 1e-12);
    check("MUGI xcg_m2              ", Oiner.get_Trbb_m(ratio)(1), 0, 1e-12);
    check("MUGI xcg_m3              ", Oiner.get_Trbb_m(ratio)(2), (-0.005 * 2 - 0.006) /3 , 1e-12);
    check("MUGI I_kgm2_11           ", Oiner.get_I_kgm2(ratio)(0,0), (2.202 * 2 + 2.198) / 3, 1e-12);
    check("MUGI I_kgm2_12           ", Oiner.get_I_kgm2(ratio)(0,1), 0, 1e-12);
    check("MUGI I_kgm2_13           ", Oiner.get_I_kgm2(ratio)(0,2), (-0.191 * 2 - 0.192) / 3, 1e-12);
    check("MUGI I_kgm2_21           ", Oiner.get_I_kgm2(ratio)(1,0), 0, 1e-12);
    check("MUGI I_kgm2_22           ", Oiner.get_I_kgm2(ratio)(1,1), (3.462 * 2 + 3.430) / 3, 1e-12);
    check("MUGI I_kgm2_23           ", Oiner.get_I_kgm2(ratio)(1,2), 0, 1e-12);
    check("MUGI I_kgm2_31           ", Oiner.get_I_kgm2(ratio)(2,0), (-0.191 * 2 - 0.192) / 3, 1e-12);
    check("MUGI I_kgm2_32           ", Oiner.get_I_kgm2(ratio)(2,1), 0, 1e-12);
    check("MUGI I_kgm2_33           ", Oiner.get_I_kgm2(ratio)(2,2), (5.490 * 2 + 5.458) / 3, 1e-12);

    Eigen::Matrix3d I = Oiner.get_I_kgm2(ratio);
    Eigen::Matrix3d Ii = Oiner.get_I_kgm2_inverse(I);
    Eigen::Matrix3d Iu = I * Ii;
    check("MUGI inverse             ", Iu(0,0), 1., 1e-12);
    check("MUGI inverse             ", Iu(0,1), 0., 1e-12);
    check("MUGI inverse             ", Iu(0,2), 0., 1e-12);
    check("MUGI inverse             ", Iu(1,0), 0., 1e-12);
    check("MUGI inverse             ", Iu(1,1), 1., 1e-12);
    check("MUGI inverse             ", Iu(1,2), 0., 1e-12);
    check("MUGI inverse             ", Iu(2,0), 0., 1e-12);
    check("MUGI inverse             ", Iu(2,1), 0., 1e-12);
    check("MUGI inverse             ", Iu(2,2), 1., 1e-12);


} // closes test_iner

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////


