 #include "euler.h"
#include "rodrigues.h"
#include "dcm.h"
#include "../auxiliary.h"
#include "../tools.h"

// CLASS EULER
// ===========
// ===========

/* ===== ===== Constructors ===== ===== */
/* ==================================== */

ang::euler::euler(const ang::rodrigues& q) {
    *_ypr_rad       = atan2(+2 * (+q(1) * q(2) + q(0) * q(3)), 1 - 2 * (pow(q(2),2) + pow(q(3),2)));
    *(_ypr_rad + 1) = asin(-2 * (-q(0) * q(2) + q(1) * q(3)));
    *(_ypr_rad + 2) = atan2(+2 * (+q(2) * q(3) + q(0) * q(1)), 1 - 2 * (pow(q(1),2) + pow(q(2),2)));
}
/* constructor based on Rodrigues parameters */

 ang::euler::euler(const ang::dcm& R) {
    *_ypr_rad       = atan2(R(1,0), R(0,0));
    *(_ypr_rad + 1) = asin(-R(2,0));
    *(_ypr_rad + 2) = atan2(R(2,1), R(2,2));
}
/* constructor based on direction cosine matrix */

 ang::euler::euler(const ang::rotv& rv) {
	// there is no direct method;
	rodrigues q(rv);
    *_ypr_rad       = atan2(+2 * (+q(1) * q(2) + q(0) * q(3)), 1 - 2 * (pow(q(2),2) + pow(q(3),2)));
    *(_ypr_rad + 1) = asin(-2 * (-q(0) * q(2) + q(1) * q(3)));
    *(_ypr_rad + 2) = atan2(+2 * (+q(2) * q(3) + q(0) * q(1)), 1 - 2 * (pow(q(1),2) + pow(q(2),2)));
}
/* constructor based on rotation vector */

/* ===== ===== Move Constructors ===== ===== */
/* ========================================= */

ang::euler::euler(ang::rodrigues&& q) {
     *_ypr_rad       = atan2(+2 * (+q(1) * q(2) + q(0) * q(3)), 1 - 2 * (pow(q(2),2) + pow(q(3),2)));
     *(_ypr_rad + 1) = asin(-2 * (-q(0) * q(2) + q(1) * q(3)));
     *(_ypr_rad + 2) = atan2(+2 * (+q(2) * q(3) + q(0) * q(1)), 1 - 2 * (pow(q(1),2) + pow(q(2),2)));
}
/* move constructor based on Rodrigues parameters */

ang::euler::euler(ang::dcm&& R) {
     *_ypr_rad       = atan2(R(1,0), R(0,0));
     *(_ypr_rad + 1) = asin(-R(2,0));
     *(_ypr_rad + 2) = atan2(R(2,1), R(2,2));
}
/* move constructor based on direction cosine matrix */

 ang::euler::euler(ang::rotv&& rv) {
     // there is no direct method;
     rodrigues q(rv);
     *_ypr_rad       = atan2(+2 * (+q(1) * q(2) + q(0) * q(3)), 1 - 2 * (pow(q(2),2) + pow(q(3),2)));
     *(_ypr_rad + 1) = asin(-2 * (-q(0) * q(2) + q(1) * q(3)));
     *(_ypr_rad + 2) = atan2(+2 * (+q(2) * q(3) + q(0) * q(1)), 1 - 2 * (pow(q(1),2) + pow(q(2),2)));
}
/* move constructor based on rotation vector */

/* ===== ===== Assignments ===== ===== */
/* =================================== */

ang::euler& ang::euler::operator=(const ang::rodrigues& q) {
    *_ypr_rad       = atan2(+2 * (+q(1) * q(2) + q(0) * q(3)), 1 - 2 * (pow(q(2),2) + pow(q(3),2)));
    *(_ypr_rad + 1) = asin(-2 * (-q(0) * q(2) + q(1) * q(3)));
    *(_ypr_rad + 2) = atan2(+2 * (+q(2) * q(3) + q(0) * q(1)), 1 - 2 * (pow(q(1),2) + pow(q(2),2)));
    return *this;
}
/* assignment operator based on Rodrigues parameters */

ang::euler& ang::euler::operator=(const ang::dcm& R) {
    *_ypr_rad       = atan2(R(1,0), R(0,0));
    *(_ypr_rad + 1) = asin(-R(2,0));
    *(_ypr_rad + 2) = atan2(R(2,1), R(2,2));
    return *this;
}
/* assignment operator based on direction cosine matrix */

ang::euler& ang::euler::operator=(const ang::rotv& rv) {
    // there is no direct method;
    rodrigues q(rv);
    *_ypr_rad       = atan2(+2 * (+q(1) * q(2) + q(0) * q(3)), 1 - 2 * (pow(q(2),2) + pow(q(3),2)));
    *(_ypr_rad + 1) = asin(-2 * (-q(0) * q(2) + q(1) * q(3)));
    *(_ypr_rad + 2) = atan2(+2 * (+q(2) * q(3) + q(0) * q(1)), 1 - 2 * (pow(q(1),2) + pow(q(2),2)));
    return *this;
}
/* assignment operator based on rotation vector */

/* ===== ===== Move Assignments ===== ===== */
/* ======================================== */

 ang::euler& ang::euler::operator=(ang::rodrigues&& q) {
     *_ypr_rad       = atan2(+2 * (+q(1) * q(2) + q(0) * q(3)), 1 - 2 * (pow(q(2),2) + pow(q(3),2)));
     *(_ypr_rad + 1) = asin(-2 * (-q(0) * q(2) + q(1) * q(3)));
     *(_ypr_rad + 2) = atan2(+2 * (+q(2) * q(3) + q(0) * q(1)), 1 - 2 * (pow(q(1),2) + pow(q(2),2)));
     return *this;
 }
/* move assignment operator based on Rodrigues parameters */

 ang::euler& ang::euler::operator=(ang::dcm&& R) {
     *_ypr_rad       = atan2(R(1,0), R(0,0));
     *(_ypr_rad + 1) = asin(-R(2,0));
     *(_ypr_rad + 2) = atan2(R(2,1), R(2,2));
     return *this;
 }
/* move assignment operator based on direction cosine matrix */

 ang::euler& ang::euler::operator=(ang::rotv&& rv) {
     // there is no direct method;
     rodrigues q(rv);
     *_ypr_rad       = atan2(+2 * (+q(1) * q(2) + q(0) * q(3)), 1 - 2 * (pow(q(2),2) + pow(q(3),2)));
     *(_ypr_rad + 1) = asin(-2 * (-q(0) * q(2) + q(1) * q(3)));
     *(_ypr_rad + 2) = atan2(+2 * (+q(2) * q(3) + q(0) * q(1)), 1 - 2 * (pow(q(1),2) + pow(q(2),2)));
     return *this;
 }
/* move assignment operator based on rotation vector */

/* ===== ===== Transformations ===== ===== */
/* ======================================= */

Eigen::Vector3d ang::euler::operator*(const Eigen::Vector3d& vecin) const {
    double sy = sin(*_ypr_rad);
    double cy = cos(*_ypr_rad);
    double sp = sin(*(_ypr_rad + 1));
    double cp = cos(*(_ypr_rad + 1));
    double sr = sin(*(_ypr_rad + 2));
    double cr = cos(*(_ypr_rad + 2));

    double vec11 = vecin(0);
    double vec12 = cr * vecin(1) - sr * vecin(2);
    double vec13 = cr * vecin(2) + sr * vecin(1);

    double vec21 = cp * vec11 + sp * vec13;
    double vec22 = vec12;
    double vec23 = cp * vec13 - sp * vec11;

    return Eigen::Vector3d(cy * vec21 - sy * vec22, cy * vec22 + sy * vec21, vec23);
}
/* overloaded operator * (forward rotation) */

Eigen::Vector3d ang::euler::operator/(const Eigen::Vector3d& vecin) const {
    double sy = sin(*_ypr_rad);
    double cy = cos(*_ypr_rad);
    double sp = sin(*(_ypr_rad + 1));
    double cp = cos(*(_ypr_rad + 1));
    double sr = sin(*(_ypr_rad + 2));
    double cr = cos(*(_ypr_rad + 2));

    double vec11 = cy * vecin(0) + sy * vecin(1);
    double vec12 = cy * vecin(1) - sy * vecin(0);
    double vec13 = vecin(2);

    double vec21 = cp * vec11 - sp * vec13;
    double vec22 = vec12;
    double vec23 = cp * vec13 + sp * vec11;

    return Eigen::Vector3d(vec21, cr * vec22 + sr * vec23, cr * vec23 - sr * vec22);
}
/* overloaded operator / (backward rotation) */

/* ===== ===== Angular Velocity ===== ===== */
/* ======================================== */

Eigen::Vector3d ang::euler::dot2omegabody(const Eigen::Vector3d& eulerdot_rps) const {
    double sp = sin(*(_ypr_rad + 1));
    double cp = cos(*(_ypr_rad + 1));
    double sr = sin(*(_ypr_rad + 2));
    double cr = cos(*(_ypr_rad + 2));
	return Eigen::Vector3d(-sp * eulerdot_rps(0) + eulerdot_rps(2),
			+sr * cp * eulerdot_rps(0) + cr * eulerdot_rps(1),
			+cr * cp * eulerdot_rps(0) - sr * eulerdot_rps(1));
}
/* obtains the body angular velocity from the Euler angles and their time differentials. */

Eigen::Vector3d ang::euler::omegabody2dot(const Eigen::Vector3d& omega_body_rps) const {
	double tanp = tan(*(_ypr_rad + 1));
	double secp = 1 / cos(*(_ypr_rad + 1));
    double sinr = sin(*(_ypr_rad + 2));
    double cosr = cos(*(_ypr_rad + 2));
	return Eigen::Vector3d(sinr * secp * omega_body_rps(1) + cosr * secp * omega_body_rps(2),
		cosr * omega_body_rps(1) - sinr * omega_body_rps(2),
		omega_body_rps(0) + sinr * tanp * omega_body_rps(1) + cosr * tanp * omega_body_rps(2));
}
/* obtains the Euler angles differentials with time based on the Euler angles and the body angular velocity. */

Eigen::Vector3d ang::euler::dot2omegaspace(const Eigen::Vector3d& eulerdot_rps) const {
    double sy = sin(*_ypr_rad);
    double cy = cos(*_ypr_rad);
    double sp = sin(*(_ypr_rad + 1));
    double cp = cos(*(_ypr_rad + 1));
    return Eigen::Vector3d(- sy * eulerdot_rps(1) + cy * cp * eulerdot_rps(2),
               + cy * eulerdot_rps(1) + sy * cp * eulerdot_rps(2),
               eulerdot_rps(0) - sp * eulerdot_rps(2));
}
/* obtains the space angular velocity from the Euler angles and their time differentials. */

Eigen::Vector3d ang::euler::omegaspace2dot(const Eigen::Vector3d& omega_space_rps) const {
    double siny = sin(*_ypr_rad);
    double cosy = cos(*_ypr_rad);
    double tanp = tan(*(_ypr_rad + 1));
    double secp = 1 / cos(*(_ypr_rad + 1));
    return Eigen::Vector3d(cosy * tanp * omega_space_rps(0) + siny * tanp * omega_space_rps(1) + omega_space_rps(2),
               - siny * omega_space_rps(0)      + cosy * omega_space_rps(1),
               cosy * secp * omega_space_rps(0) + siny * secp * omega_space_rps(1));
}
/* obtains the Euler angles differentials with time based on the Euler angles and the space angular velocity. */

/* ===== ===== Other ===== ===== */
/* ============================= */

void ang::euler::obtain_euler_nedgrd(euler& euler_ng, const euler& euler_nb, const Eigen::Vector3d& v_n_mps) {
	// absolute bearing angle
	*euler_ng._ypr_rad = atan2(v_n_mps(1), v_n_mps(0));
	// absolute path angle
	*(euler_ng._ypr_rad + 1) = atan2(-v_n_mps(2), sqrt(pow(v_n_mps(0),2) + pow(v_n_mps(1),2)));

	// yaw[-180, 180] and pitch[-90, 90]
	double sgamma = sin(*(euler_ng._ypr_rad + 1));
	double cgamma = cos(*(euler_ng._ypr_rad + 1));

	double stheta = sin(*(euler_nb._ypr_rad + 1));
	double ctheta = cos(*(euler_nb._ypr_rad + 1));

	double sxi = sin(*(euler_nb._ypr_rad + 2));
	double cxi = cos(*(euler_nb._ypr_rad + 2));
	double txi = sxi / cxi;

	double Deltachi = *euler_ng._ypr_rad - *euler_nb._ypr_rad;
	ang::tools::correct_yaw_rad(Deltachi);
	double sDeltachi = sin(Deltachi);
	double cDeltachi = cos(Deltachi);

	*(euler_ng._ypr_rad + 2) = atan((+sgamma * sDeltachi + txi * stheta * sgamma * cDeltachi + txi * ctheta * cgamma) / (+cDeltachi - txi * stheta * sDeltachi));
}
/* obtains the Euler angles that define the rotation between NED and GRD
(chi, gamma, mu) from the Euler angles between NED and BFS (psi,
# theta, xi) and the absolute speed in NED. */

/*
void ang::euler::obtain_euler_nedbfs(euler& euler_nedbfs, const euler& euler_nedwfs, const euler& euler_wfsbfs) {
	double coschiTAS = cos(*euler_nedwfs._ypr_rad);
	double sinchiTAS = sin(*euler_nedwfs._ypr_rad);

	double cosgammaTAS = cos(*(euler_nedwfs._ypr_rad + 1));
	double singammaTAS = sin(*(euler_nedwfs._ypr_rad + 1));
	double tangammaTAS = singammaTAS / cosgammaTAS;

	double cosmuTAS = cos(*(euler_nedwfs._ypr_rad + 2));
    double sinmuTAS = sin(*(euler_nedwfs._ypr_rad + 2));

	double cosalpha = cos(*(euler_wfsbfs._ypr_rad + 1));
	double sinalpha = sin(*(euler_wfsbfs._ypr_rad + 1));
	double cosbeta = cos(- *euler_wfsbfs._ypr_rad);
	double sinbeta = sin(- *euler_wfsbfs._ypr_rad);

	// yaw[-180, 180]
	*euler_nedbfs._ypr_rad = atan2(+cosalpha * cosbeta * cosgammaTAS * sinchiTAS
		- cosalpha * sinbeta * cosmuTAS * coschiTAS
		- cosalpha * sinbeta * sinmuTAS * singammaTAS * sinchiTAS
		+ sinalpha * sinmuTAS * coschiTAS
		- sinalpha * cosmuTAS * singammaTAS * sinchiTAS,
		+cosalpha * cosbeta * cosgammaTAS * coschiTAS
		+ cosalpha * sinbeta * cosmuTAS * sinchiTAS
		- cosalpha * sinbeta * sinmuTAS * singammaTAS * coschiTAS
		- sinalpha * sinmuTAS * sinchiTAS
		- sinalpha * cosmuTAS * singammaTAS * coschiTAS);
	// pitch[-90, 90]
	*(euler_nedbfs._ypr_rad + 1) = asin(+cosalpha * cosbeta * singammaTAS
		+ cosalpha * sinbeta * sinmuTAS * cosgammaTAS
		+ sinalpha * cosmuTAS * cosgammaTAS);
	// bank[-90, 90]
	*(euler_nedbfs._ypr_rad + 2) = atan((-sinbeta * tangammaTAS + cosbeta * sinmuTAS) /
		(-sinalpha * cosbeta * tangammaTAS - sinalpha * sinbeta * sinmuTAS + cosalpha * cosmuTAS));
}
 */
/* obtains the Euler angles that define the rotation between NED and BFS
(psi, ttheta, xi) from the Euler angles between NED and WFS (chiTAS,
gammaTAS, muTAS) and the Euler angles between WFS and BFS(alpha, beta). */

/*
void ang::euler::obtain_euler_wfsbfs(euler& euler_wfsbfs, const euler& euler_nedbfs, const euler& euler_nedwfs) {
	double cospsi = cos(*euler_nedbfs._ypr_rad);
	double sinpsi = sin(*euler_nedbfs._ypr_rad);
	double costheta = cos(*(euler_nedbfs._ypr_rad + 1));
	double sintheta = sin(*(euler_nedbfs._ypr_rad + 1));
	double cosxi = cos(*(euler_nedbfs._ypr_rad + 2));
	double sinxi = sin(*(euler_nedbfs._ypr_rad + 2));

	double coschiTAS = cos(*euler_nedwfs._ypr_rad);
	double sinchiTAS = sin(*euler_nedwfs._ypr_rad);
	double cosgammaTAS = cos(*(euler_nedwfs._ypr_rad + 1));
	double singammaTAS = sin(*(euler_nedwfs._ypr_rad + 1));
	double cosmuTAS = cos(*(euler_nedwfs._ypr_rad + 2));
	double sinmuTAS = sin(*(euler_nedwfs._ypr_rad + 2));

	// angle of sideslip[-90, 90],
	*euler_wfsbfs._ypr_rad = -asin(-cosxi * sinpsi * coschiTAS * cosgammaTAS
		+ sinxi * sintheta * cospsi * coschiTAS * cosgammaTAS
		+ cosxi * cospsi * sinchiTAS * cosgammaTAS
		+ sinxi * sintheta * sinpsi * sinchiTAS * cosgammaTAS
		- sinxi * costheta * singammaTAS);
	// angle of attack[-90, 90]
	*(euler_wfsbfs._ypr_rad + 1) = asin(-costheta * cospsi * coschiTAS * singammaTAS * cosmuTAS
		- costheta * cospsi * sinchiTAS * sinmuTAS
		- costheta * sinpsi * sinchiTAS * singammaTAS * cosmuTAS
		+ costheta * sinpsi * coschiTAS * sinmuTAS
		+ sintheta * cosgammaTAS * cosmuTAS);
	// third angle is always zero
	*(euler_wfsbfs._ypr_rad + 2) = 0.0;
}
*/
/* obtains the Euler angles that define the rotation between WFS and BFS
alpha, beta) from the Euler angles between NED and WFS (chiTAS,
gammaTAS, muTAS) and the Euler angles between NED and BFS(psi, ttheta, xi). */

/*
void ang::euler::obtain_euler_nedwfs(euler& euler_nedwfs, const euler& euler_nedbfs, const euler& euler_wfsbfs) {
    double cospsi = cos(*euler_nedbfs._ypr_rad);
    double sinpsi = sin(*euler_nedbfs._ypr_rad);
    double costheta = cos(*(euler_nedbfs._ypr_rad + 1));
    double sintheta = sin(*(euler_nedbfs._ypr_rad + 1));
	double tantheta = sintheta / costheta;
    double cosxi = cos(*(euler_nedbfs._ypr_rad + 2));
    double sinxi = sin(*(euler_nedbfs._ypr_rad + 2));

	double cosalpha = cos(*(euler_wfsbfs._ypr_rad + 1));
	double sinalpha = sin(*(euler_wfsbfs._ypr_rad + 1));
	double cosbeta = cos(- *euler_wfsbfs._ypr_rad);
	double sinbeta = sin(- *euler_wfsbfs._ypr_rad);

	// yaw[-180, 180]
	*euler_nedwfs._ypr_rad = atan2(+cosalpha * cosbeta * costheta * sinpsi
		+ sinbeta * cosxi * cospsi
		+ sinbeta * sinxi * sintheta * sinpsi
		- sinalpha * cosbeta * sinxi * cospsi
		+ sinalpha * cosbeta * cosxi * sintheta * sinpsi,
		+cosalpha * cosbeta * costheta * cospsi
		- sinbeta * cosxi * sinpsi
		+ sinbeta * sinxi * sintheta * cospsi
		+ sinalpha * cosbeta * sinxi * sinpsi
		+ sinalpha * cosbeta * cosxi * sintheta * cospsi);
	// pitch[-90, 90]
	*(euler_nedwfs._ypr_rad + 1) = asin(+cosalpha * cosbeta * sintheta - sinbeta * sinxi * costheta	- sinalpha * cosbeta * cosxi * costheta);
	// bank[-90, 90]
	*(euler_nedwfs._ypr_rad + 2) = atan((+sinbeta * tantheta * cosalpha + cosbeta * sinxi - sinalpha * sinbeta * cosxi) /
		(+sinalpha * tantheta + cosalpha * cosxi));
}
*/
/* obtains the Euler angles that define the rotation between NED and WFS
(chiTAS, gammaTAS, muTAS) from the Euler angles between NED and BFS
(psi, ttheta, xi) and the Euler angles between WFS and BFS (alpha, beta). */

double ang::euler::control_bearing_diff(const double& a_deg, const double& b_deg) {
    double diff = a_deg - b_deg;
    double ab = std::fabs(diff);
    if (ab > 180.0) {
        diff = (360.0 - ab) * (-1) * diff / ab;
    }
    if (std::fabs(diff) > 90.0) {
        diff = std::nan("");
    }
    return diff;
}
/* returns difference between two bearings in degrees, nan if higher than plus minus 90. */

void ang::euler::obtain_cross_long_track_errors(double& error_cross_m, double& error_long_m, const double& error_north_m, const double& error_east_m, const double& chi_rad) {
    double error_hor_m = std::sqrt(std::pow(error_north_m,2) + std::pow(error_east_m,2));
    double alpha_rad   = std::atan2(error_east_m, error_north_m);
    double beta_rad    = alpha_rad - chi_rad;
    error_cross_m      = error_hor_m * sin(beta_rad);
    error_long_m       = error_hor_m * cos(beta_rad);
 }
/* fills up the cross track error (positive to the right, negative to the left) and the long
 * track errors (positive to the front, negative to the back) based on the North error (positive
 * to the North, negative to the South), the East error (positive to the East, negative to the
 * West), and the trajectory bearing angle */

/* ===== ===== Angles for Forward Looking Vectors (choose yaw and pitch) ===== ===== */
/* ================================================================================= */

 double ang::euler::obtain_yaw_forward(const Eigen::Vector3d& v_ned) {
     return atan2(v_ned(1), v_ned(0));
 }
/* given a forward looking vector (generally speed) expressed in NED, it returns the yaw angle [rad] of that vector with respect to the NED frame */

 double ang::euler::obtain_pitch_forward(const Eigen::Vector3d& v_ned) {
     return atan2(- v_ned(2), sqrt(pow(v_ned(0), 2) + pow(v_ned(1), 2)));
 }
/* given a forward looking vector (generally speed) expressed in NED, it returns the pitch angle [rad] of that vector with respect to the NED frame */

/* ===== ===== Angles for Downward Looking Vectors (choose pitch and roll) ===== ===== */
/* =================================================================================== */

double ang::euler::obtain_pitch_downward(const Eigen::Vector3d& g_ned) {
     return atan2(- g_ned(2), g_ned(0));
}
/*< given a downward looking vector (generally gravitation) expressed in NED, it returns the pitch angle [rad] of that vector with respect to the NED frame */

double ang::euler::obtain_bank_downward(const Eigen::Vector3d& g_ned) {
     return atan2(- g_ned(1), sqrt(pow(g_ned(0), 2) + pow(g_ned(2), 2)));
}
/* given a downward looking vector (generally gravitation) expressed in NED, it returns the bank angle [rad] of that vector with respect to the NED frame */





