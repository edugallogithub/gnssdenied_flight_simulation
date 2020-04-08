#include "speed.h"
#include <cmath>

// CLASS SPEED
// ===========
// ===========

Eigen::Vector3d env::speed::vtas2vtasbfs(const double& vtas_mps, const ang::euler& euler_wb) {
	double cosalpha = cos(+euler_wb.get_pitch_rad());
	double sinalpha = sin(+euler_wb.get_pitch_rad());
	double sinbeta  = sin(-euler_wb.get_yaw_rad());
	double cosbeta  = cos(-euler_wb.get_yaw_rad());
    return {vtas_mps * cosalpha * cosbeta, vtas_mps * sinbeta, vtas_mps * sinalpha * cosbeta};
}
/* compute the three true airspeed BFS (Body Fixed System) components [mps] based
on the true airspeed modulus, the angle of attack, and the angle of sideslip. */

Eigen::Vector3d env::speed::vtas2vtasned(const double& vtas_mps, const ang::euler& euler_nw) {
	double coschiTAS   = cos(euler_nw.get_yaw_rad());
	double sinchiTAS   = sin(euler_nw.get_yaw_rad());
	double cosgammaTAS = cos(euler_nw.get_pitch_rad());
	double singammaTAS = sin(euler_nw.get_pitch_rad());
	return {vtas_mps * cosgammaTAS * coschiTAS, vtas_mps * cosgammaTAS * sinchiTAS, -vtas_mps * singammaTAS};
}
/* compute the three true airspeed NED (North - East - Down) components [mps]
based on the true airspeed modulus and the NED to WFS Euler angles. */

double env::speed::vtasbfs2vtas(const Eigen::Vector3d& vtas_b_mps) {
	return vtas_b_mps.norm();
}
/* compute the airspeed modulus [mps] based on the three true airspeed BFS
(Body Fixed System) components. */

ang::euler env::speed::vtasbfs2euler_wb(const Eigen::Vector3d& vtas_b_mps) {
	return ang::euler(-asin(vtas_b_mps(1) / vtas_b_mps.norm()), atan(vtas_b_mps(2) / vtas_b_mps(0)), 0.);
}
/* compute the WFS to BFS Euler angles (minus angles of sideslip, angle of attack,
zero) based on the three true airspeed BFS (Body Fixed System) components. */

ang::euler env::speed::vtasbfs2euler_wb(const Eigen::Vector3d& vtas_b_mps, const double& vtas_mps) {
    return ang::euler(-asin(vtas_b_mps(1) / vtas_mps), atan(vtas_b_mps(2) / vtas_b_mps(0)), 0.);
}
/* compute the WFS to BFS Euler angles (minus angles of sideslip, angle of attack,
zero) based on the three true airspeed BFS (Body Fixed System) components and its norm. */

Eigen::Vector3d env::speed::vtasbfsdot(const double& vtas_mps, const ang::euler& euler_wb, const double& dvtas_dt_mps2, const Eigen::Vector3d& deuler_wb_dt_rps) {
	double cosalpha = cos(+euler_wb.get_pitch_rad());
	double sinalpha = sin(+euler_wb.get_pitch_rad());
	double sinbeta  = sin(-euler_wb.get_yaw_rad());
	double cosbeta  = cos(-euler_wb.get_yaw_rad());

	double dalpha_dt_rps = + deuler_wb_dt_rps(1);
	double dbeta_dt_rps  = - deuler_wb_dt_rps(0);

	return {+ dvtas_dt_mps2 * cosalpha * cosbeta - dalpha_dt_rps * vtas_mps * sinalpha * cosbeta - dbeta_dt_rps * vtas_mps * cosalpha * sinbeta,
		    + dvtas_dt_mps2 * sinbeta + dbeta_dt_rps * vtas_mps * cosbeta,
		    + dvtas_dt_mps2 * sinalpha * cosbeta + dalpha_dt_rps * vtas_mps * cosalpha * cosbeta - dbeta_dt_rps * vtas_mps * sinalpha * sinbeta};
}
/* Compute the differential with time of the true airspeed vector in
BFS(Body Fixed System) [mps2] based on the true airspeed modulus, the angle of
attack, the angle of sideslip, and their differentials with time. */

Eigen::Vector3d env::speed::omega2h(const Eigen::Matrix3d& I_kgm2, const Eigen::Vector3d& w_rps) {
	// NOTE: This function assumes that the body is symmetric along its "xz"
	// plane, this is, the Jxy and Jyz members of I are zero. Invalid if this is not the case.
	return I_kgm2 * w_rps;
}
/* computes the kinetic moment [Nms] as the product of the inertial tensor by the angular speed. */

Eigen::Vector3d env::speed::h2omega(const Eigen::Matrix3d&I_kgm2, const Eigen::Vector3d& h_Nms) {
	double temp = I_kgm2(0,0) * I_kgm2(2,2) - pow(I_kgm2(0,2), 2);
	return Eigen::Vector3d((-I_kgm2(0,2) * h_Nms(2) + I_kgm2(2,2) * h_Nms(0)) / temp,
		h_Nms(1) / I_kgm2(1,1),
		(-I_kgm2(0,2) * h_Nms(0) + I_kgm2(0,0) * h_Nms(2)) / temp);
}
/* computes the angular speed [rps] based on the inertial tensor and kinetic moment */





