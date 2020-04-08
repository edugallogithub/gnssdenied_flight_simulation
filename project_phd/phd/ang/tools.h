#ifndef ANG_TOOLS
#define ANG_TOOLS

#include "ang.h"

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ang {

// CLASS TOOLS
// ===========
// ===========

class ANG_API tools {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**< ===== ===== ===== ===== Vector3d = Vector3d cross Vector3d ===== ===== ===== ===== */
    /**< ===== ===== ===== ===== ================================== ===== ===== ===== ===== */
    /**< ===== a.cross(b) == a.skew3() * b == b.right_skew3() * a == - b.skew3() * a  ===== */
    /**< ===== =====================================================================  ===== */
    /**< returns the skew symmetric matrix of a vector, so a.cross(b) == a.skew3() * b */
    static Eigen::Matrix3d skew3(const Eigen::Vector3d& v);
	/**< returns the vector from a skew symmetric matrix */
	static Eigen::Vector3d skew3_inverse(const Eigen::Matrix3d& m);
    /**< returns the RIGHT skew symmetric matrix of a vector, so a.cross(b) == b.right_skew3() * a == - b.skew3() * a */
    static Eigen::Matrix3d right_skew3(const Eigen::Vector3d& v);
    /**< returns the vector from a RIGHT skew symmetric matrix */
    static Eigen::Vector3d right_skew3_inverse(const Eigen::Matrix3d& m);

    /**< ===== ===== ===== Quat4d = Quat4d * Quat4d ===== ===== ===== */
    /**< ===== ===== ===== ======================== ===== ===== ===== */
    /**< ===== ===== a.quat_cross(b) == a.skew4() * b     ===== ===== */
    /**< ===== ===== a.quat_cross(b) == b.right_skew4() * a === ===== */
    /**< ===== ===== ====================================== === ===== */
	/**< returns the quaternion left product matrix (skew symmetric) of a quaternion, so a.quat_cross(b) == a.skew4() * b */
	static Eigen::Matrix4d skew4(const Eigen::Vector4d& q);
    /**< returns the quaternion from a quaternion left product matrix (skew symmetric) */
    static Eigen::Vector4d skew4_inverse(const Eigen::Matrix4d& m);
    /**< returns the quaternion RIGHT product matrix (skew symmetric) of a quaternion, so a.quat_cross(b) == b.right_skew4() * a */
    static Eigen::Matrix4d right_skew4(const Eigen::Vector4d& q);
    /**< returns the quaternion from a quaternion RIGHT product matrix (skew symmetric) */
    static Eigen::Vector4d right_skew4_inverse(const Eigen::Matrix4d& m);

    /**< ===== ===== ===== Quat4d = Quat4d * [0 Vector3d] ===== ===== ===== */
    /**< ===== ===== ===== ============================== ===== ===== ===== */
    /**< ===== ===== a.quat_cross([0 b]) == a.skew43() * b      ===== ===== */
    /**< ===== ===== a.quat_cross([0 b]) == b.right_skew43() * a ==== ===== */
    /**< ================================================================== */
    /**< returns the quaternion left product matrix (skew symmetric) of a quaternion, so for a quaternion and b size 3 vector,
     * a.quat_cross([0 b]) == a.skew43() * b */
    static Eigen::Matrix<double,4,3> skew43(const Eigen::Vector4d& q);
    /**< returns the quaternion RIGHT product matrix (skew symmetric) of a size 3 vector, so for a quaternion and b size 3 vector,
     * a.quat_cross([0 b]) == b.right_skew43() * a */
    static Eigen::Matrix4d right_skew43(const Eigen::Vector3d& v);

    /**< ===== ===== ===== Other Methods ===== ===== ===== */
    /**< ===== ===== ===== ============= ===== ===== ===== */
	/**< corrects input yaw (bearing or heading) angle so it falls in the range (-pi, +pi] */
	static void correct_yaw_rad(double& yaw_rad);
    /**< corrects input longitude so it falls in the range [0, 2*pi) */
    static void correct_longitude_rad(double& lambda_rad);
    /**< corrects input yaw (bearing or heading) angle so it falls in the range (-180, +180] */
    static void correct_yaw_deg(double& yaw_deg);
    /**< corrects input longitude so it falls in the range [0, 360) */
    static void correct_longitude_deg(double& lambda_deg);

	/**< obtains the difference between two angles (in radians) maintaining the result in the plus minus 180 range.
	 * NOTE: Corrects by 360[deg] but it is not iterative as it should be. */
	static double angle_diff_rad(const double& a_rad, const double& b_rad);
    /**< obtains the difference between two angles (in degrees) maintaining the result in the plus minus 180 range.
     * NOTE: Corrects by 360[deg] but it is not iterative as it should be. */
    static double angle_diff_deg(const double& a_deg, const double& b_deg);

    /**< returns matrix resulting from multiplying the vector transpose by itself */
    static Eigen::Matrix3d itself_by_transpose(const Eigen::Vector3d&);
}; // closes class tools

} // closes namespace ang

#endif
