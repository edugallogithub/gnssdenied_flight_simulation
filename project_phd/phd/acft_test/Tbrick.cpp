#include "Tbrick.h"
#include "env/speed.h"
#include "ang/rotate/rodrigues.h"

acft::test::Tbrick::Tbrick(jail::counter& Ocounter)
: ::jail::unit_test(Ocounter) {
}
/* constructor based on counter */

void acft::test::Tbrick::run() {
	::jail::unit_test::run();

    test_brick_motion();

	finished();
}
/* execute tests and write results on console */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void acft::test::brick_motion::aux(const double& t_sec,
                               env::geodetic_coord& dx_gdt_rad_m,
                               Eigen::Vector3d& dv_bfs_mps_dt,
                               ang::quat& dq_nedbfs_dt,
                               Eigen::Vector3d& dh_bfsirsbfs_Nms_dt,
                               const env::geodetic_coord& x_gdt_rad_m,
                               const Eigen::Vector3d& v_bfs_mps,
                               const ang::rodrigues& q_nedbfs,
                               const Eigen::Vector3d& h_bfsirsbfs_Nms,
                               const env::geo& Ogeo,
                               const Eigen::Matrix3d& I_kgm2) {

    double N_m = Ogeo.radius_vert(x_gdt_rad_m.get_phi_rad());
    double M_m = Ogeo.radius_mer(x_gdt_rad_m.get_phi_rad(), N_m);
    Eigen::Vector3d v_ned_mps = q_nedbfs * v_bfs_mps;

    Eigen::Vector3d a_cor_ned_mps2 = Ogeo.compute_coriolis_n(v_ned_mps, x_gdt_rad_m.get_phi_rad());
    Eigen::Vector3d a_cor_bfs_mps2 = q_nedbfs / a_cor_ned_mps2;

    Eigen::Vector3d w_irsbfsbfs_rps = env::speed::h2omega(I_kgm2, h_bfsirsbfs_Nms);
    Eigen::Vector3d w_ecefnedned_rps = Ogeo.compute_wenn_rps(v_ned_mps, N_m, M_m, x_gdt_rad_m.get_phi_rad(), x_gdt_rad_m.get_h_m());
    Eigen::Vector3d w_ecefnedbfs_rps = q_nedbfs / w_ecefnedned_rps;
    Eigen::Vector3d w_irsecefned_rps = Ogeo.compute_wien_rps(x_gdt_rad_m.get_phi_rad());
    Eigen::Vector3d w_irsecefbfs_rps = q_nedbfs / w_irsecefned_rps;
    Eigen::Vector3d w_ecefbfsbfs_rps = w_irsbfsbfs_rps - w_irsecefbfs_rps;
    Eigen::Vector3d w_nedbfsbfs_rps = w_ecefbfsbfs_rps - w_ecefnedbfs_rps;

    //env::geocentric_coord x_gct_rad_m = Ogeo.geodetic2geocentric(x_gdt_rad_m, N_m);
    Eigen::Vector3d gc_ned_mps2 = Ogeo.compute_gravity_n_truth(x_gdt_rad_m);
    Eigen::Vector3d gc_bfs_mps2 = q_nedbfs / gc_ned_mps2;

    dx_gdt_rad_m.get_lambda_rad() = v_ned_mps(1) / ((N_m + x_gdt_rad_m.get_h_m()) * cos(x_gdt_rad_m.get_phi_rad()));
    dx_gdt_rad_m.get_phi_rad() = v_ned_mps(0) / (M_m + x_gdt_rad_m.get_h_m());
    dx_gdt_rad_m.get_h_m() = -v_ned_mps(2);
    dv_bfs_mps_dt = gc_bfs_mps2 - w_ecefbfsbfs_rps.cross(v_bfs_mps) - a_cor_bfs_mps2;
    dq_nedbfs_dt = q_nedbfs.omegabody2dot(w_nedbfsbfs_rps);
    dh_bfsirsbfs_Nms_dt = - w_irsbfsbfs_rps.cross(h_bfsirsbfs_Nms);
}
/* computes the state vector differentials with time based on the state vector */

std::vector<double> acft::test::brick_motion::solve(double t_sec_end) {

    // Initial conditions - attitude
    double psi0_deg = +14;		double psi0_rad = psi0_deg * math::constant::D2R();
    double theta0_deg = -48;    double theta0_rad = theta0_deg * math::constant::D2R();
    double xi0_deg = -127;      double xi0_rad = xi0_deg * math::constant::D2R();
    ang::euler euler0_bfsned(psi0_rad, theta0_rad, xi0_rad);
    ang::rodrigues q0_nedbfs(euler0_bfsned);

    // Initial conditions - speed
    Eigen::Vector3d v0_ned_mps(15.2, 3.8, 2.3);
    Eigen::Vector3d v0_bfs_mps = q0_nedbfs / v0_ned_mps;

    // Initial conditions - position
    double lambda0_deg = 30.0;	double lambda0_rad = lambda0_deg * math::constant::D2R();
    double phi0_deg = 25.0;		double phi0_rad = phi0_deg * math::constant::D2R();
    double h0_m = 1500.0;
    env::geodetic_coord x0_gdt_rad_m(lambda0_rad, phi0_rad, h0_m);

    // Initial condition - angular momentum
    Eigen::Matrix3d I_kgm2;
    I_kgm2 << 1.0, 0.0, 0.0, 0.0, 0.7, 0.0, 0.0, 0.0, 0.3;
    Eigen::Vector3d w0_bfsirsbfs_dps(270.0, 120.0, 38.0);
    Eigen::Vector3d w0_bfsirsbfs_rps = w0_bfsirsbfs_dps * math::constant::D2R();
    Eigen::Vector3d h0_bfsirsbfs_Nms = env::speed::omega2h(I_kgm2, w0_bfsirsbfs_rps);

    //Time stamps for integration
    double Deltat_sec = 0.001; // 1000 [hz]
    unsigned nel = (unsigned)floor(t_sec_end / Deltat_sec) + 1; // state vector size

    // Allocate memory for state vector
    std::vector<double> t_sec(nel);
    std::vector<env::geodetic_coord> x_gdt_rad_m(nel);
    std::vector<Eigen::Vector3d> v_bfs_mps(nel);
    std::vector<ang::rodrigues> q_nedbfs(nel);
    std::vector<Eigen::Vector3d> h_bfsirsbfs_Nms(nel);

    // Add initial coniditions to state vector
    t_sec[0] = 0.;
    x_gdt_rad_m[0] = x0_gdt_rad_m;
    v_bfs_mps[0] = v0_bfs_mps;
    q_nedbfs[0] = q0_nedbfs;
    h_bfsirsbfs_Nms[0] = h0_bfsirsbfs_Nms;

    env::geo_mix Ogeo(env::logic::mag_default); // ellipsoidal Earth model

    // temporary objects
    env::geodetic_coord Tdx_gdt_rad_m_dt, Rdx_gdt_rad_m_dt;
    Eigen::Vector3d Tdv_bfs_mps_dt, Rdv_bfs_mps_dt;
    ang::quat Tdq_bfsned_dt, Rdq_bfsned_dt;
    Eigen::Vector3d Tdh_bfsirsbfs_Nms_dt, Rdh_bfsirsbfs_Nms_dt;

    env::geodetic_coord Tx_gdt_rad_m;
    Eigen::Vector3d Tv_bfs_mps;
    ang::rodrigues Tq_bfsned;
    Eigen::Vector3d Th_bfsirsbfs_Nms;

    for (int i = 1; i != nel; ++i) {
        t_sec[i] = t_sec[i - 1] + Deltat_sec;

        // fist step of 2nd order fixed step integration
        aux(t_sec[i - 1], Tdx_gdt_rad_m_dt, Tdv_bfs_mps_dt, Tdq_bfsned_dt, Tdh_bfsirsbfs_Nms_dt, x_gdt_rad_m[i - 1], v_bfs_mps[i - 1], q_nedbfs[i - 1], h_bfsirsbfs_Nms[i - 1], Ogeo, I_kgm2);

        // compute intermediate state vector values
        Tx_gdt_rad_m = x_gdt_rad_m[i - 1] + Tdx_gdt_rad_m_dt * Deltat_sec;
        Tv_bfs_mps = v_bfs_mps[i - 1] + Tdv_bfs_mps_dt * Deltat_sec;
        Tq_bfsned = q_nedbfs[i - 1] + Deltat_sec * Tdq_bfsned_dt;
        Th_bfsirsbfs_Nms = h_bfsirsbfs_Nms[i - 1] + Deltat_sec * Tdh_bfsirsbfs_Nms_dt;
        Tq_bfsned.normalize(); // normalize quaternion

        // second step of 2nd order fixed step integration
        aux(t_sec[i], Rdx_gdt_rad_m_dt, Rdv_bfs_mps_dt, Rdq_bfsned_dt, Rdh_bfsirsbfs_Nms_dt, Tx_gdt_rad_m, Tv_bfs_mps, Tq_bfsned, Th_bfsirsbfs_Nms, Ogeo, I_kgm2);

        // compute next state vector values
        x_gdt_rad_m[i] = x_gdt_rad_m[i-1] + (Tdx_gdt_rad_m_dt + Rdx_gdt_rad_m_dt) * 0.5 * Deltat_sec;
        v_bfs_mps[i] = v_bfs_mps[i - 1] + (Tdv_bfs_mps_dt + Rdv_bfs_mps_dt) * 0.5 * Deltat_sec;
        q_nedbfs[i] = q_nedbfs[i - 1] + (Tdq_bfsned_dt + Rdq_bfsned_dt) * 0.5 * Deltat_sec;
        h_bfsirsbfs_Nms[i] = h_bfsirsbfs_Nms[i - 1] + (Tdh_bfsirsbfs_Nms_dt + Rdh_bfsirsbfs_Nms_dt) * 0.5 * Deltat_sec;
        q_nedbfs[i].normalize(); // normalize quaternion
    }

    // return final values
    std::vector<double> res(17);
    res[0] = t_sec.back();
    res[1] = x_gdt_rad_m.back().get_lambda_rad();
    res[2] = x_gdt_rad_m.back().get_phi_rad();
    res[3] = x_gdt_rad_m.back().get_h_m();
    res[4] = v_bfs_mps.back()(0);
    res[5] = v_bfs_mps.back()(1);
    res[6] = v_bfs_mps.back()(2);
    res[7] = q_nedbfs.back()(0);
    res[8] = q_nedbfs.back()(1);
    res[9] = q_nedbfs.back()(2);
    res[10] = q_nedbfs.back()(3);
    res[11] = h_bfsirsbfs_Nms.back()(0);
    res[12] = h_bfsirsbfs_Nms.back()(1);
    res[13] = h_bfsirsbfs_Nms.back()(2);
    ang::euler euler_nedbfs_rad_final(q_nedbfs.back());
    res[14] = euler_nedbfs_rad_final.get_yaw_rad();
    res[15] = euler_nedbfs_rad_final.get_pitch_rad();
    res[16] = euler_nedbfs_rad_final.get_bank_rad();
    return res;
}
/* integrates the motion of a brick for the input time */

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

void acft::test::Tbrick::test_brick_motion() {
	std::vector<double> res = acft::test::brick_motion::solve(100);
	check("brick_motion - t_sec             ", res[0], 100.0, 1e-09);
	check("brick_motion - lambda_rad        ", res[1], 0.523709751375284, 1e-12);
	check("brick_motion - phi_rad           ", res[2], 0.436572383905928, 1e-12);
	check("brick_motion - h_m               ", res[3], -4.777896864511917e+04, 1e-9);
	check("brick_motion - v_bfs_mps1        ", res[4], 3.849548105878110e+02, 1e-09);
	check("brick_motion - v_bfs_mps2        ", res[5], -4.207002128602089e+02, 1e-09);
	check("brick_motion - v_bfs_mps3        ", res[6], 8.043701624909780e+02, 1e-09);
	check("brick_motion - q_bfsned0         ", res[7], 0.951995662452564, 1e-12);
	check("brick_motion - q_bfsned1         ", res[8], -0.218492023975641, 1e-12);
	check("brick_motion - q_bfsned2         ", res[9], -0.208263020043691, 1e-12);
	check("brick_motion - q_bfsned3         ", res[10], 0.050911772831178, 1e-12);
	check("brick_motion - h_bfsirsbfs_Nms1  ", res[11], 4.829150422293826, 1e-12);
	check("brick_motion - h_bfsirsbfs_Nms2  ", res[12], -0.885801804954300, 1e-11);
	check("brick_motion - h_bfsirsbfs_Nms3  ", res[13], 0.538757426630898, 1e-12);
	check("brick_motion - psi_rad           ", res[14], 0.204088568132570, 1e-12);
	check("brick_motion - theta_rad         ", res[15], -0.383623831677668, 1e-12);
	check("brick_motion - xi_rad            ", res[16], -0.490971701683007, 1e-12);

} // closes test_brick_motion

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

