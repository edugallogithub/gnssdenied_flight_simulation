#ifndef ANG_REPRESENTATION_ROTV
#define ANG_REPRESENTATION_ROTV

#include "../ang.h"
#include "../auxiliary.h" // required for the proper behavior of the << operator
#include <Eigen/Core>
#include <Eigen/Geometry>

/*
This file contains the "rotv" class that models the rotation vector representation of a rotation. */

namespace ang {

class euler;
class rodrigues;
class dcm;

// CLASS ROTV
// ==========
// ==========

class ANG_API rotv : public Eigen::Vector3d {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< ===== ===== Constructors ===== ===== */
	/**< empty constructor (not initialized) */
	rotv() = default;
    /**< normal constructor based on rotation vector components */
    rotv(double p1, double p2, double p3);
    /**< constructor based on size 3 vector */
    rotv(const Eigen::Vector3d& Ovec);
    /**< constructor based on size 3 vector multiplied by input factor */
    rotv(const Eigen::Vector3d& Ovec, const double& factor);
	/**< constructor based on Euler angles */
	explicit rotv(const euler&);
	/**< constructor based on Rodrigues parameters */
	explicit rotv(const rodrigues&);
	/**< constructor based on direction cosine matrix */
	explicit rotv(const dcm&);
    /**< VERY IMPORTANT constructor that obtains the rotation from v1 to v2, both normalized.
    The direction is orthogonal to the plane formed by v1 and v2. The magnitude is the angle
    between both input vectors in that plane. The modulus of the input vectors does not matter.
    The result is such that v2.normalized() = this * v1.normalized(). */
    rotv(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2);
    /**< copy constructor */
	rotv(const rotv&) = default;
	/**< destructor */
	~rotv() = default;

    /**< ===== ===== Move Constructors ===== ===== */
    /**< move constructor */
    rotv(rotv&&) = default;
    /**< move constructor based on size 3 vector */
    explicit rotv(Eigen::Vector3d&& Ovec);
    /**< move constructor based on size 3 vector multiplied by input factor */
    rotv(const Eigen::Vector3d&& Ovec, const double& factor);
    /**< move constructor based on Euler angles */
    explicit rotv(euler&&);
    /**< move constructor based on Rodrigues parameters */
    explicit rotv(rodrigues&&);
    /**< move constructor based on direction cosine matrix */
    explicit rotv(dcm&&);

    /**< ===== ===== Assignments ===== ===== */
	/**< copy assignment */
	rotv& operator=(const rotv&) = default;
    /**< assignment operator based on size 3 vector */
    rotv& operator=(const Eigen::Vector3d&);
    /**< assignment operator based on Euler angles */
    rotv& operator=(const euler&);
    /**< assignment operator based on Rodrigues parameters */
    rotv& operator=(const rodrigues&);
    /**< assignment operator based on direction cosine matrix */
    rotv& operator=(const dcm&);

    /**< ===== ===== Move Assignments ===== ===== */
    /**< move assignment */
    rotv& operator=(rotv&&) = default;
    /**< move assignemnt overator based on size 3 vector */
    rotv& operator=(Eigen::Vector3d&&);
    /**< move assignment operator based on Euler angles */
    rotv& operator=(euler&&);
    /**< move assignment operator based on Rodrigues parameters */
    rotv& operator=(rodrigues&&);
    /**< move assignment operator based on direction cosine matrix */
    rotv& operator=(dcm&&);

    /**< ===== ===== Transformations ===== ===== */
	/**< overloaded operator * (combination of rotations) */
	rotv operator*(const rotv& op2) const;
    /**< overloaded operator / (backward combination of rotations) */
    rotv operator/(const rotv& op2) const;
	/**< overloaded operator * (forward rotation) */
	Eigen::Vector3d operator*(const Eigen::Vector3d&) const;
    /**< overloaded operator / (backward rotation) */
    Eigen::Vector3d operator/(const Eigen::Vector3d&) const;
    /**< returns inverse or opposite rotation */
	rotv inverse() const;
    /**< overloaded operator * (multiplication of rotation angle with no change in direction) */
    rotv operator*(const double& factor) const;
    /**< overloaded operator / (division of rotation angle with no change in direction) */
    rotv operator/(const double& factor) const;
    /**< executes object rotation a fraction (t < 1) or a multiple (t > 1) of times.
     * Returns input fraction (interpolation or extrapolation) of the object. */
    rotv pow(const double& t) const;
    /**< spherical linear interpolation, returns rotv0 for t=0 and rotv1 for t=1 */
    static rotv slerp(const rotv& rotv0, const rotv& rotv1, const double& t);

    /**< ===== ===== Exponential Map ===== ===== */
    /**< exponential map that returns rodrigues parameters */
    ang::rodrigues exp_map_rodrigues() const;
    /**< exponential map that returns direction cosine matrix */
    ang::dcm exp_map_dcm() const;
    /**< converts a 3x1 rotation vector representation of a rotation into the rodrigues parameters */
    static ang::rodrigues exp_map_rodrigues(const Eigen::Vector3d& rotv);
    /**< converts a 3x1 rotation vector representation of a rotation into the direction cosine matrix */
    static ang::dcm exp_map_dcm(const Eigen::Vector3d& rotv);

    /**< ===== ===== Angular Velocity ===== ===== */
	/**< obtains the body angular velocity from the rotation vector and	its time differential. */
	Eigen::Vector3d dot2omegabody(const Eigen::Vector3d& rvdot) const;
    /**< obtains the rotation vector differential with time based on the rotation vector and the body angular velocity. */
    Eigen::Vector3d omegabody2dot(const Eigen::Vector3d& omega_body_rps) const;

    /**< obtains the space angular velocity from the rotation vector and its time differential. */
    Eigen::Vector3d dot2omegaspace(const Eigen::Vector3d& rvdot) const;
    /**< obtains the rotation vector differential with time based on the rotation vector and the space angular velocity. */
    Eigen::Vector3d omegaspace2dot(const Eigen::Vector3d& omega_space_rps) const;
}; // closes class rotv
    
} // closes namespace ang

#endif

