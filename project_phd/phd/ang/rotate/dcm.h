#ifndef ANG_REPRESENTATION_DCM
#define ANG_REPRESENTATION_DCM

#include "../ang.h"
#include "../auxiliary.h" // required for the proper behavior of the << operator
#include <Eigen/Core>
#include <Eigen/Geometry>

/*
This file contains the "dcm" class that models the direction cosine matrix representation of a rotation.
*/

namespace ang {

class rodrigues;
class euler;
class rotv;

// CLASS DCM
// =========
// =========

class ANG_API dcm : public Eigen::Matrix3d {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< ===== ===== Constructors ===== ===== */
    /**< empty constructor (not initialized) */
    dcm() = default;
    /**< constructor based on 3x3 matrix */
    dcm(const Eigen::Matrix3d& Omat) : Eigen::Matrix3d(Omat) {this->normalize();}
    /**< constructor based on Euler angles */
    explicit dcm(const euler&);
    /**< constructor based on Rodrigues parameters */
    explicit dcm(const rodrigues&);
    /**< constructor based on rotation vector */
    explicit dcm(const rotv&);
    /**< copy constructor */
    dcm(const dcm&) = default;
    /**< destructor */
    ~dcm() = default;

    /**< ===== ===== Move Constructors ===== ===== */
    /**< move constructor */
    dcm(dcm&&) = default;
    /**< move constructor based on 3x3 matrix */
    dcm(Eigen::Matrix3d&& Omat) : Eigen::Matrix3d(Omat) {this->normalize();}
    /**< move constructor based on Euler angles */
    explicit dcm(euler&&);
    /**< move constructor based on Rodrigues parameters */
    explicit dcm(rodrigues&&);
    /**< move constructor based on rotation vector */
    explicit dcm(rotv&&);

    /**< ===== ===== Assignments ===== ===== */
    /**< copy assignment */
    dcm& operator=(const dcm&) = default;
    /**< assignment operator based 3x3 matrix */
    dcm& operator=(const Eigen::Matrix3d&);
    /**< assignment operator based on Euler angles */
    dcm& operator=(const euler&);
    /**< assignment operator based on Rodrigues parameters */
    dcm& operator=(const rodrigues&);
    /**< assignment operator based on rotation vector */
    dcm& operator=(const rotv&);

    /**< ===== ===== Move Assignments ===== ===== */
    /**< move assignment */
    dcm& operator=(dcm&&) = default;
    /**< move assignment operator based 3x3 matrix */
    dcm& operator=(Eigen::Matrix3d&&);
    /**< move assignment operator based on Euler angles */
    dcm& operator=(euler&&);
    /**< move assignment operator based on Rodrigues parameters */
    dcm& operator=(rodrigues&&);
    /**< move assignment operator based on rotation vector */
    dcm& operator=(rotv&&);

    /**< ===== ===== Transformations ===== ===== */
    /**< overloaded operator * (combination of rotations) */
    dcm operator*(const dcm& op2) const;
    /**< overloaded operator / (backward combination of rotations) */
    dcm operator/(const dcm& op2) const;
    /**< overloaded operator * (forward rotation) */
    Eigen::Vector3d operator*(const Eigen::Vector3d&) const;
    /**< overloaded operator / (backward rotation) */
    Eigen::Vector3d operator/(const Eigen::Vector3d&) const;
    /**< returns inverse or opposite rotation */
    dcm inverse() const;
    /**< executes object rotation a fraction (t < 1) or a multiple (t > 1) of times.
     * Returns exponential map of the power function applied to the object logarithmic map. */
    dcm pow(const double& t) const;
    /**< spherical linear interpolation, returns R0 for t=0 and R1 for t=1 */
    static dcm slerp(const dcm& R0, const dcm& R1, const double& t);
    /**< plus operator (input rotation located in vector space tangent to object manifold, which is not verified) */
    dcm plus(const rotv&) const;
    /**< minus operator (output rotation located in vector space tangent to input manifold, not in object manifold) */
    rotv minus(const dcm&) const;

    /**< ===== ===== Logarithmic Map ===== ===== */
    /**< logarithmic map that returns the rotation vector */
    ang::rotv log_map() const;
    /**< converts a 3x3 direction cosine matrix representation of a rotation into the rotation vector */
    static ang::rotv log_map(const Eigen::Matrix3d& R);

    /**< ===== ===== Angular Velocity ===== ===== */
    /**< obtains the body angular velocity from the direction cosine matrix and its time differentials. */
    Eigen::Vector3d dot2omegabody(const Eigen::Matrix3d& dcmdot) const;
    /**< obtains the direction cosine matrix differentials with time based on the direction cosine matrix and the body angular velocity. */
    Eigen::Matrix3d omegabody2dot(const Eigen::Vector3d& omega_body_rps) const;

    /**< obtains the space angular velocity from the direction cosine matrix and its time differentials. */
    Eigen::Vector3d dot2omegaspace(const Eigen::Matrix3d& dcmdot) const;
    /**< obtains the direction cosine matrix differentials with time based on the direction cosine matrix and the space angular velocity. */
    Eigen::Matrix3d omegaspace2dot(const Eigen::Vector3d& omega_space_rps) const;

    /**< ===== ===== Other ===== ===== */
    /**< normalize the rotation matrix ensuring it is orthonormal */
    void normalize();
}; // closes class dcm

} // closes namespace ang

#endif














