#include "tools.h"
#include "auxiliary.h"
#include "math/logic/constant.h"
#include <iostream>

// CLASS TOOLS
// ===========
// ===========

/* ===== ===== ===== ===== Vector3d = Vector3d cross Vector3d ===== ===== ===== ===== */
/* ===== ===== ===== ===== ================================== ===== ===== ===== ===== */
/* ===== a.cross(b) == a.skew3() * b == b.right_skew3() * a == - b.skew3() * a  ===== */
/* ===== ====================================================================== ===== */
Eigen::Matrix3d ang::tools::skew3(const Eigen::Vector3d& v) {
	Eigen::Matrix3d v_skew;
    v_skew << 0., -v(2), v(1), v(2), 0., -v(0), -v(1), v(0), 0.;
	return v_skew;
}
/* returns the skew symmetric matrix of a vector, so a.cross(b) == a.skew3() * b */

Eigen::Vector3d ang::tools::skew3_inverse(const Eigen::Matrix3d& m) {
	return {m(2,1), m(0,2), m(1,0)};
}
/* returns the vector from a skew symmetric matrix */

Eigen::Matrix3d ang::tools::right_skew3(const Eigen::Vector3d& v) {
    Eigen::Matrix3d v_skew_right;
    v_skew_right << 0., v(2), -v(1), -v(2), 0., v(0), v(1), -v(0), 0.;
    return v_skew_right;
}
/* returns the RIGHT skew symmetric matrix of a vector, so a.cross(b) == b.right_skew3() * a == - b.skew3() * a */

Eigen::Vector3d ang::tools::right_skew3_inverse(const Eigen::Matrix3d& m) {
    return {m(1,2), m(2,0), m(0,1)};
}
/* returns the vector from a RIGHT skew symmetric matrix */

/* ===== ===== ===== Quat4d = Quat4d * Quat4d ===== ===== ===== */
/* ===== ===== ===== ======================== ===== ===== ===== */
/* ===== ===== a.quat_cross(b) == a.skew4() * b     ===== ===== */
/* ===== ===== a.quat_cross(b) == b.right_skew4() * a === ===== */
/* ===== ===== ====================================== === ===== */
Eigen::Matrix4d ang::tools::skew4(const Eigen::Vector4d& q) {
    Eigen::Matrix4d q_skew;
    q_skew << q(0), -q(1), -q(2), - q(3), q(1), q(0), -q(3), q(2), q(2), q(3), q(0), -q(1), q(3), -q(2), q(1), q(0);
    return q_skew;
}
/* returns the quaternion left product matrix (skew symmetric) of a quaternion, so a.quat_cross(b) == a.skew4() * b */

Eigen::Vector4d ang::tools::skew4_inverse(const Eigen::Matrix4d& m) {
    return m.leftCols<1>();
}
/* returns the quaternion from a quaternion left product matrix (skew symmetric) */

Eigen::Matrix4d ang::tools::right_skew4(const Eigen::Vector4d& q) {
    Eigen::Matrix4d q_skew_right;
    q_skew_right << q(0), -q(1), -q(2), - q(3), q(1), q(0), q(3), -q(2), q(2), -q(3), q(0), q(1), q(3), q(2), -q(1), q(0);
    return q_skew_right;
 }
/* returns the quaternion RIGHT product matrix (skew symmetric) of a quaternion, so a.quat_cross(b) == b.right_skew4() * a */

Eigen::Vector4d ang::tools::right_skew4_inverse(const Eigen::Matrix4d& m) {
    return m.leftCols<1>();    
}
/* returns the quaternion from a quaternion RIGHT product matrix (skew symmetric) */

/* ===== ===== ===== Quat4d = Quat4d * [0 Vector3d] ===== ===== ===== */
/* ===== ===== ===== ============================== ===== ===== ===== */
/* ===== ===== a.quat_cross([0 b]) == a.skew43() * b      ===== ===== */
/* ===== ===== a.quat_cross([0 b]) == b.right_skew43() * a ==== ===== */
/* ================================================================== */
Eigen::Matrix<double,4,3> ang::tools::skew43(const Eigen::Vector4d& q) {
    Eigen::Matrix<double,4,3> q_skew;
    q_skew << -q(1), -q(2), - q(3), q(0), -q(3), q(2), q(3), q(0), -q(1), -q(2), q(1), q(0);
    return q_skew;
}
/* returns the quaternion left product matrix (skew symmetric) of a quaternion, so for q quaternion b size 3 vector, a.quat_cross([0 b]) == a.skew43() * b */

Eigen::Matrix4d ang::tools::right_skew43(const Eigen::Vector3d& v) {
    Eigen::Matrix4d v_skew_right;
    v_skew_right << 0., -v(0), -v(1), -v(2), v(0), 0., v(2), -v(1), v(1), -v(2), 0., v(0), v(2), v(1), -v(0), 0.;
    return v_skew_right;
}
/* returns the quaternion RIGHT product matrix (skew symmetric) of a size 3 vector, so for q quaternion and b size 3 vector, a.quat_cross([0 b]) == b.right_skew43() * a */

/* ===== ===== ===== Other Methods ===== ===== ===== */
/* ===== ===== ===== ============= ===== ===== ===== */
void ang::tools::correct_yaw_rad(double& yaw_rad) {
	if (yaw_rad > math::constant::PI()) {
		yaw_rad = yaw_rad - 2.0 * math::constant::PI();
	}
	else if (yaw_rad <= (-math::constant::PI())) {
		yaw_rad = yaw_rad + 2.0 * math::constant::PI();
	}
}
/* corrects input yaw (bearing or heading) angle so it falls in the range (-pi, +pi] */

void ang::tools::correct_longitude_rad(double& lambda_rad) {
    if (lambda_rad >= 2.0 * math::constant::PI()) {
        lambda_rad = lambda_rad - 2.0 * math::constant::PI();
    }
    else if (lambda_rad < 0.) {
        lambda_rad = lambda_rad + 2.0 * math::constant::PI();
    }
}
/* corrects input longitude so it falls in the range [0, 2*pi) */

void ang::tools::correct_yaw_deg(double& yaw_deg) {
    if (yaw_deg > 180.0) {
        yaw_deg = yaw_deg - 360.0;
    }
    else if (yaw_deg <= (-180.0)) {
        yaw_deg = yaw_deg + 360.0;
    }
}
/* corrects input yaw (bearing or heading) angle so it falls in the range (-180, +180] */

void ang::tools::correct_longitude_deg(double& lambda_deg) {
    if (lambda_deg >= 360.0) {
        lambda_deg = lambda_deg - 360.0;
    }
    else if (lambda_deg < 0.) {
        lambda_deg = lambda_deg + 360.0;
    }
}
/* corrects input longitude so it falls in the range [0, 360) */

double ang::tools::angle_diff_rad(const double& a_rad, const double& b_rad) {
	double diff = a_rad - b_rad;
	double ab = fabs(diff);
	if (ab > math::constant::PI()) {
		diff = (ab - 2 * math::constant::PI()) * diff / ab;
	}
	return diff;
}
/* obtains the difference between two angles (in radians) maintaining the result in the plus minus 180 range.
 * NOTE: Corrects by 360[deg] but it is not iterative as it should be. */

double ang::tools::angle_diff_deg(const double& a_deg, const double& b_deg) {
    double diff = a_deg - b_deg;
    double ab = fabs(diff);
    if (ab > 180.0) {
        diff = (ab - 360.0) * diff / ab;
    }
    return diff;
}
/* obtains the difference between two angles (in degrees) maintaining the result in the plus minus 180 range.
 * NOTE: Corrects by 360[deg] but it is not iterative as it should be. */

Eigen::Matrix3d ang::tools::itself_by_transpose(const Eigen::Vector3d& vec) {
    return vec * vec.transpose();
}
/* returns matrix resulting from multiplying the vector transpose by itself */



