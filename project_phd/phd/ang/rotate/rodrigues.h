#ifndef ANG_REPRESENTATION_RODRIGUES
#define ANG_REPRESENTATION_RODRIGUES

#include "../ang.h"
#include "../quat.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

/*
This file contains the "rodrigues" class that models the Rodrigues parameters (quaternion) representation of a rotation. */

namespace ang {

class euler;
class dcm;
class rotv;

// CLASS RODRIGUES
// ===============
// ===============

class ANG_API rodrigues : public quat {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< ===== ===== Constructors ===== ===== */
	/**< default constructor (not initialized) */
	rodrigues() = default;
	/**< normal constructor */
	rodrigues(double p0, double p1, double p2, double p3) : ang::quat(p0, p1, p2, p3) {this->normalize();}
    /**< constructor based on quaternion */
    rodrigues(const quat& q) : ang::quat(q) {this->normalize();}
    /**< constructor based on Euler angles */
	explicit rodrigues(const euler&);
	/**< constructor based on direction cosine matrix */
	explicit rodrigues(const dcm&);
	/**< constructor based on rotation vector */
	explicit rodrigues(const rotv&);
    /**< copy constructor */
    rodrigues(const rodrigues&) = default;
    /**< destructor */
	~rodrigues() override = default;

    /**< ===== ===== Move Constructors ===== ===== */
    /**< move constructor */
    rodrigues(rodrigues&&) = default;
    /**< move constructor based on quaternion */
    rodrigues(quat&& q) : ang::quat(q) {this->normalize();}
    /**< move constructor based on Euler angles */
    explicit rodrigues(euler&&);
    /**< move constructor based on direction cosine matrix */
    explicit rodrigues(dcm&&);
    /**< move constructor based on rotation vector */
    explicit rodrigues(rotv&&);

    /**< ===== ===== Assignments ===== ===== */
    /**< copy assignment */
    rodrigues& operator=(const rodrigues&) = default;
    /**< assignment operator = based on quaternion */
    rodrigues& operator=(const quat&);
    /**< assignment operator based on Euler angles */
    rodrigues& operator=(const euler&);
    /**< assignment operator based on direction cosine matrix */
    rodrigues& operator=(const dcm&);
    /**< assignment operator based on rotation vector */
    rodrigues& operator=(const rotv&);

    /**< ===== ===== Move Assignments ===== ===== */
    /**< move assignment */
    rodrigues& operator=(rodrigues&&) = default;
    /**< move assignment operator = based on quaternion */
    rodrigues& operator=(quat&&);
    /**< move assignment operator based on Euler angles */
    rodrigues& operator=(euler&&);
    /**< move assignment operator based on direction cosine matrix */
    rodrigues& operator=(dcm&&);
    /**< move assignment operator based on rotation vector */
    rodrigues& operator=(rotv&&);

    /**< ===== ===== Transformations ===== ===== */
    /**< overloaded operator * (combination of rotations) */
	rodrigues operator*(const rodrigues& op2) const;
    /**< overloaded operator / (backward combination of rotations) */
    rodrigues operator/(const rodrigues& op2) const;
	/**< overloaded operator * (forward rotation) */
	Eigen::Vector3d operator*(const Eigen::Vector3d&) const;
    /**< overloaded operator / (backward rotation) */
    Eigen::Vector3d operator/(const Eigen::Vector3d&) const;
	/**< returns inverse or opposite rotation */
	rodrigues inverse() const;
    /**< returns the same rotation but with negative quaternion (all indexes reversed) */
    rodrigues negative() const;
    /**< executes object rotation a fraction (t < 1) or a multiple (t > 1) of times.
     * Returns exponential map of the power function applied to the object logarithmic map. */
    rodrigues pow(const double& t) const;
    /**< spherical linear interpolation, returns q0 for t=0 and q1 for t=1 */
    static rodrigues slerp(const rodrigues& q0, const rodrigues& q1, const double& t);
    /**< plus operator (input rotation located in vector space tangent to object manifold, which is not verified) */
    rodrigues plus(const rotv&) const;
    /**< minus operator (output rotation located in vector space tangent to input manifold, not in object manifold) */
    rotv minus(const rodrigues&) const;

    /**< ===== ===== Logarithmic Map ===== ===== */
    /**< logarithmic map that returns the rotation vector */
    ang::rotv log_map() const;
     /**< converts a 4x1 unit quaternion representation of a rotation into the rotation vector */
    static ang::rotv log_map(const ang::quat& q);

    /**< ===== ===== Angular Velocity ===== ===== */
	/**< obtains the body angular velocity from the Rodrigues parameters and their time differentials. */
    Eigen::Vector3d dot2omegabody(const quat& rodriguesdot) const;
	/**< obtains the Rodrigues parameters differentials with time based on the Rodrigues parameters and the body angular velocity. */
	quat omegabody2dot(const Eigen::Vector3d& omega_body_rps) const;

    /**< obtains the space angular velocity from the Rodrigues parameters and their time differentials. */
    Eigen::Vector3d dot2omegaspace(const quat& rodriguesdot) const;
    /**< obtains the Rodrigues parameters differentials with time based on the Rodrigues parameters and the space angular velocity. */
    quat omegaspace2dot(const Eigen::Vector3d& omega_space_rps) const;

    /**< ===== ===== Jacobians ===== ===== */
    /**< returns the jacobian [3x4] of a forward rotation with respect to the quaternion, equal to d(q * v)/dq.
     * Note that the forward rotation IS NOT linear on the quaternion, so q * v != d(q * v)/dq * q */
    Eigen::Matrix<double,3,4> jacobian_quat_forward_rotation(const Eigen::Vector3d& v) const;
    /**< returns the jacobian [3x4] of a backward rotation with respect to the quaternion, equal to d(q / v)/dq.
     * Note that the backward rotation IS NOT linear on the quaternion, so q / v != d(q /v)/dq * q */
    Eigen::Matrix<double,3,4> jacobian_quat_backward_rotation(const Eigen::Vector3d& v) const;
    /**< returns the jacobian [3x3] of a forward rotation with respect to the vector, equal to d(q * v)/dv.
     * Note that the forward rotation IS linear on the vector, so q * v == d(q * v)/dv * v.
     * It coincides with the rotation matrix of the quaternion. */
    Eigen::Matrix3d jacobian_vector_forward_rotation() const;
    /**< returns the jacobian [3x3] of a backward rotation with respect to the vector, equal to d(q / v)/dv.
     * Note that the backward rotation IS linear on the vector, so q / v == d(q / v)/dv * v.
     * It coincides with the transpose of the rotation matrix of the quaternion. */
    Eigen::Matrix3d jacobian_vector_backward_rotation() const;
}; // closes class rodrigues

} // closes namespace ang

#endif






















