
#ifndef ANG_GROUPS_SO3
#define ANG_GROUPS_SO3

#include "../ang.h"
#include "../tools.h"
#include "../quat.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ang {

// CLASS SO3
// =========
// =========

/**< Special Orthogonal Group SO(3) */

class ANG_API SO3 {
protected:
    /**< normalized quaternion */
    ang::quat _q;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/**< ===== ===== Constructors ===== ===== */
	/**< default constructor (creates no rotation) */
	SO3();
	/**< constructor based on Euler angles */
	SO3(const double& yaw_rad, const double& pitch_rad, const double& bank_rad);
	/**< constructor based on direction cosine matrix representing the rotation */
	explicit SO3(const Eigen::Matrix3d& dcm);
	/**< constructor based on quaternion representing the rotation */
	explicit SO3(const ang::quat& q);
	/**< constructor based on rotation vector representing the rotation */
	explicit SO3(const Eigen::Vector3d& rotv);
    /**< copy constructor */
    SO3(const SO3&) = default;
    /**< destructor */
    ~SO3() = default;

    /**< ===== ===== Move Constructors ===== ===== */
    /**< move constructor */
    SO3(SO3&&) = default;
    /**< move constructor based on direction cosine matrix representing the rotation */
    explicit SO3(Eigen::Matrix3d&& dcm);
    /**< move constructor based on quaternion representing the rotation */
    explicit SO3(ang::quat&& q);
    /**< move constructor based on rotation vector representing the rotation */
    explicit SO3(Eigen::Vector3d&& rotv);

    /**< ===== ===== Assignments ===== ===== */
    /**< copy assignment */
    SO3& operator=(const SO3&) = default;
    /**< assignment operator based on direction cosine matrix representing the rotation */
    SO3& operator=(const Eigen::Matrix3d& dcm);
    /**< assignment operator based on quaternion representing the rotation */
    SO3& operator=(const ang::quat& q);
    /**< assignment operator based on rotation vector representing the rotation */
    SO3& operator=(const Eigen::Vector3d& rotv);

    /**< ===== ===== Move Assignments ===== ===== */
    /**< move assignment */
    SO3& operator=(SO3&&) = default;
    /**< move assignment operator based on direction cosine matrix representing the rotation */
    SO3& operator=(Eigen::Matrix3d&& dcm);
    /**< move assignment operator based on quaternion representing the rotation */
    SO3& operator=(ang::quat&& q);
    /**< move assignment operator based on rotation vector representing the rotation */
    SO3& operator=(Eigen::Vector3d&& rotv);

	/**< ===== ===== Transformations ===== ===== */
    /**< overloaded operator * (combination of rotations) */
    SO3 operator*(const SO3&) const;
    /**< overloaded operator / (backward combination of rotations) */
    SO3 operator/(const SO3& op2) const;
    /**< overloaded operator * (forward rotation) */
	Eigen::Vector3d operator*(const Eigen::Vector3d&) const;
    /**< overloaded operator / (backward rotation) */
    Eigen::Vector3d operator/(const Eigen::Vector3d&) const;
	/**< returns inverse or opposite rotation */
	SO3 inverse() const;
    /**< returns the same rotation but with negative quaternion (all indexes reversed) */
    SO3 negative() const;
    /**< executes object rotation a fraction (t < 1) or a multiple (t > 1) of times.
     * Returns exponential map of the exponential function applied to the object logarithmic map. */
    SO3 pow(const double& t) const;
    /**< spherical linear interpolation, returns s0 for t=0 and s1 for t=1 */
    static SO3 slerp(const SO3& s0, const SO3& s1, const double& t);
    /**< plus operator (input rotation located in vector space tangent to object manifold, which is not verified) */
    SO3 plus(const Eigen::Vector3d& rotv) const;
    /**< minus operator (output rotation located in vector space tangent to input manifold, not in object manifold) */
    Eigen::Vector3d minus(const SO3&) const;

    /**< ===== ===== Exponential and Logarithmic Maps ===== ===== */
    /**< converts a 3x1 rotation vector representation of a rotation into an SO3 object */
    static SO3 exp_map(const Eigen::Vector3d& rotv) {return SO3(rotv);}
    /**< converts an SO3 object into a 3x1 rotation vector representation of a rotation */
    static Eigen::Vector3d log_map(const SO3& so3) {return so3.get_rotv();}

    /**< ===== ===== Setters ===== ===== */
    /**< modify object based on Euler angles */
    void set(const double& yaw_rad, const double& pitch_rad, const double& bank_rad);
    /**< modify object based on direction cosine matrix representing the rotation */
    void set(const Eigen::Matrix3d& dcm);
    /**< modify object based on quaternion representing the rotation */
    void set(const ang::quat& q);
    /**< modify object based on rotation vector representing the rotation */
    void set(const Eigen::Vector3d& rotv);

    /**< ===== ===== Getters ===== ===== */
	/**< return vector with yaw, pitch, and bank angles */
	Eigen::Vector3d get_euler() const;
	/**< returns direction cosine matrix representing rotation */
	Eigen::Matrix3d get_dcm() const;
	/**< return unit quaternion attribute */
	const ang::quat& get_quat() const {return _q;}
	/**< returns the 3D rotation vector associated to the object rotation */
	Eigen::Vector3d get_rotv() const;

	/**< ===== ===== Other Methods ===== ===== */
	/**< returns the SO3 lie bracket of the two input rotation vectors (equivalent to cross product) */
	static Eigen::Vector3d lie_bracket(const Eigen::Vector3d& rotv1, const Eigen::Vector3d& rotv2) {return rotv1.cross(rotv2);}
}; // closes class SO3

/**< adds the input SO3 object associated rotation vector to the stream */
ANG_API inline std::ostream& operator <<(std::ostream & out_str, const SO3& so3) {
	out_str << so3.get_rotv();
	return out_str;
}

} // end namespace ang

#endif
