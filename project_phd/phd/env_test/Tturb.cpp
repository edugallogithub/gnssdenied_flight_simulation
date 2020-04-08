#include "Tturb.h"
#include "ang/rotate/euler.h"
#include "ang/rotate/rodrigues.h"
#include "env/turb.h"
#include "math/templates/metrics_.h"

env::test::Tturb::Tturb(jail::counter& Ocounter)
: ::jail::unit_test(Ocounter) {
}
/* constructor based on counter */

void env::test::Tturb::run() {
	::jail::unit_test::run();

	test_turb();
    test_turb_obtain_differential(1.0);
    test_turb_obtain_differential(0.5);
    test_turb_obtain_differential(0.2);

	finished();
}
/* execute tests and write results on console */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void env::test::Tturb::test_turb() {
    double t_sec_init = -50.0;
	double t_sec = 202.06 + t_sec_init; // equivalent to 202.06 in Matlab where results were generated (trajectories start at -50 [sec])
	double height_m = 500.0;
	double chiwind20_rad = 35 * math::constant::D2R();
	ang::euler euler_nedbfs_rad(25 * math::constant::D2R(), -6 * math::constant::D2R(), 4 * math::constant::D2R());
	//ang::dcm C_nedbfs(euler_nedbfs_rad);
    ang::rodrigues q_nedbfs(euler_nedbfs_rad);
    double turb_factor = 1.0;

	env::turb_dryden Oturb_dryden(t_sec_init, 1, 1000, turb_factor);
	
	Eigen::Vector3d windhf_bfs_mps = Oturb_dryden.compute_wind_high_frequency_twomils_bfs(t_sec, height_m, chiwind20_rad, q_nedbfs);
	check("windhf_bfs_mps1 low      ", windhf_bfs_mps(0), 0.164177890565047, 1e-12);
	check("windhf_bfs_mps2 low      ", windhf_bfs_mps(1), -0.188677330092551, 1e-12);
	check("windhf_bfs_mps3 low      ", windhf_bfs_mps(2), 0.113284283283296, 1e-12);
    std::cout << std::endl;
} // closes test_tub

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void env::test::Tturb::test_turb_obtain_differential(const double& turb_factor) {
    double t_sec_init = -50.0;
    env::turb_dryden Oturb1(t_sec_init, 1, 1000, turb_factor); // 1000 [sec] correspond to  500001 points at 500 [hz], 100001 points at 100 [hz]
    env::turb_dryden Oturb2(t_sec_init, 1, 5000, turb_factor); // 5000 [sec] correspond to 2500001 points at 500 [hz], 500001 points at 100 [hz]
    double height_m = 3000.0;
    double chiwind20_rad = 0.; // not used at high altitudes
    ang::rodrigues q_nb(1., 0., 0., 0.); // not used at high altitudes
    double t_sec;
    Eigen::Vector3d windhf_bfs_mps;
    std::vector<double> VAdiffwind0(500000),  VAdiffwind1(500000),  VAdiffwind2(500000);
    std::vector<double> VBdiffwind0(100000),  VBdiffwind1(100000),  VBdiffwind2(100000);
    std::vector<double> VCdiffwind0(2500000), VCdiffwind1(2500000), VCdiffwind2(2500000);
    std::vector<double> VDdiffwind0(500000),  VDdiffwind1(500000),  VDdiffwind2(500000);

    Eigen::Vector3d windhf_bfs_mps_prev = windhf_bfs_mps = Oturb1.compute_wind_high_frequency_twomils_bfs(-10.0, height_m, chiwind20_rad, q_nb);
    for (int i=1; i != 500001; i++) {
        t_sec = t_sec_init + 0.002 * i;
        windhf_bfs_mps = Oturb1.compute_wind_high_frequency_twomils_bfs(t_sec, height_m, chiwind20_rad, q_nb);
        VAdiffwind0[i] = (windhf_bfs_mps(0) - windhf_bfs_mps_prev(0)) / 0.002;
        VAdiffwind1[i] = (windhf_bfs_mps(1) - windhf_bfs_mps_prev(1)) / 0.002;
        VAdiffwind2[i] = (windhf_bfs_mps(2) - windhf_bfs_mps_prev(2)) / 0.002;
        windhf_bfs_mps_prev = windhf_bfs_mps;
    }

    windhf_bfs_mps_prev = windhf_bfs_mps = Oturb1.compute_wind_high_frequency_twomils_bfs(-10.0, height_m, chiwind20_rad, q_nb);
    for (int i=1; i != 100001; i++) {
        t_sec = t_sec_init + 0.01 * i;
        windhf_bfs_mps = Oturb1.compute_wind_high_frequency_twomils_bfs(t_sec, height_m, chiwind20_rad, q_nb);
        VBdiffwind0[i] = (windhf_bfs_mps(0) - windhf_bfs_mps_prev(0)) / 0.01;
        VBdiffwind1[i] = (windhf_bfs_mps(1) - windhf_bfs_mps_prev(1)) / 0.01;
        VBdiffwind2[i] = (windhf_bfs_mps(2) - windhf_bfs_mps_prev(2)) / 0.01;
        windhf_bfs_mps_prev = windhf_bfs_mps;
    }

    windhf_bfs_mps_prev = windhf_bfs_mps = Oturb2.compute_wind_high_frequency_twomils_bfs(-10.0, height_m, chiwind20_rad, q_nb);
    for (int i=1; i != 2500001; i++) {
        t_sec = t_sec_init + 0.002 * i;
        windhf_bfs_mps = Oturb2.compute_wind_high_frequency_twomils_bfs(t_sec, height_m, chiwind20_rad, q_nb);
        VCdiffwind0[i] = (windhf_bfs_mps(0) - windhf_bfs_mps_prev(0)) / 0.002;
        VCdiffwind1[i] = (windhf_bfs_mps(1) - windhf_bfs_mps_prev(1)) / 0.002;
        VCdiffwind2[i] = (windhf_bfs_mps(2) - windhf_bfs_mps_prev(2)) / 0.002;
        windhf_bfs_mps_prev = windhf_bfs_mps;
    }

    windhf_bfs_mps_prev = windhf_bfs_mps = Oturb2.compute_wind_high_frequency_twomils_bfs(-10.0, height_m, chiwind20_rad, q_nb);
    for (int i=1; i != 500001; i++) {
        t_sec = t_sec_init + 0.01 * i;
        windhf_bfs_mps = Oturb2.compute_wind_high_frequency_twomils_bfs(t_sec, height_m, chiwind20_rad, q_nb);
        VDdiffwind0[i] = (windhf_bfs_mps(0) - windhf_bfs_mps_prev(0)) / 0.01;
        VDdiffwind1[i] = (windhf_bfs_mps(1) - windhf_bfs_mps_prev(1)) / 0.01;
        VDdiffwind2[i] = (windhf_bfs_mps(2) - windhf_bfs_mps_prev(2)) / 0.01;
        windhf_bfs_mps_prev = windhf_bfs_mps;
    }

    Eigen::Vector3d Amean_diffwind(math::mean(VAdiffwind0), math::mean(VAdiffwind1), math::mean(VAdiffwind2));
    Eigen::Vector3d Astd_diffwind(math::std(VAdiffwind0, Amean_diffwind(0)), math::std(VAdiffwind1, Amean_diffwind(1)), math::std(VAdiffwind2, Amean_diffwind(2)));
    Eigen::Vector3d Bmean_diffwind(math::mean(VBdiffwind0), math::mean(VBdiffwind1), math::mean(VBdiffwind2));
    Eigen::Vector3d Bstd_diffwind(math::std(VBdiffwind0, Bmean_diffwind(0)), math::std(VBdiffwind1, Bmean_diffwind(1)), math::std(VBdiffwind2, Bmean_diffwind(2)));
    Eigen::Vector3d Cmean_diffwind(math::mean(VCdiffwind0), math::mean(VCdiffwind1), math::mean(VCdiffwind2));
    Eigen::Vector3d Cstd_diffwind(math::std(VCdiffwind0, Cmean_diffwind(0)), math::std(VCdiffwind1, Cmean_diffwind(1)), math::std(VCdiffwind2, Cmean_diffwind(2)));
    Eigen::Vector3d Dmean_diffwind(math::mean(VDdiffwind0), math::mean(VDdiffwind1), math::mean(VDdiffwind2));
    Eigen::Vector3d Dstd_diffwind(math::std(VDdiffwind0, Dmean_diffwind(0)), math::std(VDdiffwind1, Dmean_diffwind(1)), math::std(VDdiffwind2, Dmean_diffwind(2)));

    std::cout << "Metrics for differential of wind turbulence at high altitudes: " << std::endl;
    std::cout << "These numbers provide covariance of lack of vbodydot in accelerometer equation." << std::endl;
    std::cout << "Considering all time stamps for 1000 [sec] every 0.002 [sec]: " << std::endl;
    std::cout << "Mean [mps2]: " << std::fixed << std::setw(13) << std::setprecision(7) << Amean_diffwind(0)
                                 << std::fixed << std::setw(13) << std::setprecision(7) << Amean_diffwind(1)
                                 << std::fixed << std::setw(13) << std::setprecision(7) << Amean_diffwind(2) << std::endl;
    std::cout << "std [mps2]:  " << std::fixed << std::setw(13) << std::setprecision(7) << Astd_diffwind(0)
                                 << std::fixed << std::setw(13) << std::setprecision(7) << Astd_diffwind(1)
                                 << std::fixed << std::setw(13) << std::setprecision(7) << Astd_diffwind(2) << std::endl;
    std::cout << "Considering time stamps for 1000 [sec] every 0.01 [sec]: " << std::endl;
    std::cout << "Mean [mps2]: " << std::fixed << std::setw(13) << std::setprecision(7) << Bmean_diffwind(0)
                                 << std::fixed << std::setw(13) << std::setprecision(7) << Bmean_diffwind(1)
                                 << std::fixed << std::setw(13) << std::setprecision(7) << Bmean_diffwind(2) << std::endl;
    std::cout << "std [mps2]:  " << std::fixed << std::setw(13) << std::setprecision(7) << Bstd_diffwind(0)
                                 << std::fixed << std::setw(13) << std::setprecision(7) << Bstd_diffwind(1)
                                 << std::fixed << std::setw(13) << std::setprecision(7) << Bstd_diffwind(2) << std::endl;
    std::cout << "Considering all time stamps for 5000 [sec] every 0.002 [sec]: " << std::endl;
    std::cout << "Mean [mps2]: " << std::fixed << std::setw(13) << std::setprecision(7) << Cmean_diffwind(0)
                                 << std::fixed << std::setw(13) << std::setprecision(7) << Cmean_diffwind(1)
                                 << std::fixed << std::setw(13) << std::setprecision(7) << Cmean_diffwind(2) << std::endl;
    std::cout << "std [mps2]:  " << std::fixed << std::setw(13) << std::setprecision(7) << Cstd_diffwind(0)
                                 << std::fixed << std::setw(13) << std::setprecision(7) << Cstd_diffwind(1)
                                 << std::fixed << std::setw(13) << std::setprecision(7) << Cstd_diffwind(2) << std::endl;
    std::cout << "Considering time stamps for 5000 [sec] every 0.01 [sec]: " << std::endl;
    std::cout << "Mean [mps2]: " << std::fixed << std::setw(13) << std::setprecision(7) << Dmean_diffwind(0)
                                 << std::fixed << std::setw(13) << std::setprecision(7) << Dmean_diffwind(1)
                                 << std::fixed << std::setw(13) << std::setprecision(7) << Dmean_diffwind(2) << std::endl;
    std::cout << "std [mps2]:  " << std::fixed << std::setw(13) << std::setprecision(7) << Dstd_diffwind(0)
                                 << std::fixed << std::setw(13) << std::setprecision(7) << Dstd_diffwind(1)
                                 << std::fixed << std::setw(13) << std::setprecision(7) << Dstd_diffwind(2) << std::endl;
    std::cout << "===== ===== CONCLUSIONS ===== =====" << std::endl;
    std::cout << "Although these numbers are obtained with seeds #01, they vary very little with other seeds." << std::endl;
    std::cout << "The mean is very small and not zero, but tends to zero as the number of points grows." << std::endl;
    std::cout << "The standard deviation is approximately constant at ~0.61 [mps2] for vdot_bfs_1 and" << std::endl;
    std::cout << "~0.75 [mps2] for vdot_bfs_2 and vdot_bfs_3. " << std::endl;
    std::cout << "The variance caused by not including the wind body differential with time in the " << std::endl;
    std::cout << "accelerometer equation is hence 0.372 [m2ps4] for the x axis and 0.563 [m2ps4] for the 2nd " << std::endl;
    std::cout << "and 3rd axes. Note that this is for the wind, the tas acceleration not included." << std::endl;
    std::cout << std::endl;
} // closes test_tub

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////






