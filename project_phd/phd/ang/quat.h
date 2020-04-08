#ifndef ANG_ALGEBRA_QUAT
#define ANG_ALGEBRA_QUAT

#include "ang.h"
#include "auxiliary.h" // required for the proper behavior of the << operator
#include <Eigen/Core>
#include <ostream>

/*
This file contains the "quat" class (models a size 4 vector - quaternion). It
contains overloaded operators and the most typical algebraic operations. */

namespace ang {

// CLASS 4D VECTOR - quat
// ======================
// ======================

class quat : public Eigen::Vector4d {
protected:
    friend class rodrigues; // rodrigues has access to the quat private attributes
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< default constructor (does not initialize) */
    quat() = default;
    /**< constructor based on one number and a vector */
    quat(const double& q0, const Eigen::Vector3d& qv) : Eigen::Vector4d(q0, qv(0), qv(1), qv(2)) {}
    /**< constructor based on four numbers */
    quat(const double& q0, const double& q1, const double& q2, const double& q3) : Eigen::Vector4d(q0, q1, q2, q3) {}  
    /**< copy constructor that allows to construct quat from Eigen expressions (NO explicit) */
    template<typename OtherDerived>
    quat(const Eigen::MatrixBase<OtherDerived>& other) : Eigen::Vector4d(other) {}
    /**< move constructor (NO explicit) */
    template<typename OtherDerived>
    quat(Eigen::MatrixBase<OtherDerived>&& other) : Eigen::Vector4d(other) {}
    /**< destructor */
    virtual ~quat() = default;
    /**< assignment that allows assigning Eigen expressions to quat (ignore warnings) */
    template<typename OtherDerived>
    quat& operator=(const Eigen::MatrixBase <OtherDerived>& other) {
        this->Eigen::Vector4d::operator=(other);
        return *this;
    }
    /**< move assignment (ignore warnings) */
    template<typename OtherDerived>
    quat& operator=(Eigen::MatrixBase <OtherDerived>&& other) {
        this->Eigen::Vector4d::operator=(other);
        return *this;
    }

    /**< ===== ===== 3D <--> 4D ===== ===== */
    /**< create a 4D vector from a 3D vector */
    inline static Eigen::Vector4d convert_3dto4d(const Eigen::Vector3d& v3) {return Eigen::Vector4d(0., v3(0), v3(1), v3(2));}
    /**< create a 3D vector from a 4D vector */
    inline static Eigen::Vector3d convert_4dto3d(const Eigen::Vector4d& v4) {return Eigen::Vector3d(v4(1), v4(2), v4(3));}

    /**< ===== ===== Overloaded Operators ===== ===== */
    /**< overloaded operator * (quaternion product) */
    inline quat operator*(const quat& o) const {
        quat res;
        res << (*this)(0) * o(0) - (*this)(1) * o(1) - (*this)(2) * o(2) - (*this)(3) * o(3),
               (*this)(1) * o(0) + (*this)(0) * o(1) - (*this)(3) * o(2) + (*this)(2) * o(3),
               (*this)(2) * o(0) + (*this)(3) * o(1) + (*this)(0) * o(2) - (*this)(1) * o(3),
               (*this)(3) * o(0) - (*this)(2) * o(1) + (*this)(1) * o(2) + (*this)(0) * o(3);
        return res;
    }
    /**< overloaded operator * (scalar product) */
    quat operator*(const double& o) const {
        quat res;
        res << (*this)(0) * o,  (*this)(1) * o, (*this)(2) * o, (*this)(3) * o;
        return res;
    }
    /**< overloaded operator / (scalar division) */
    quat operator/(const double& o) const {
        quat res;
        res << (*this)(0) / o,  (*this)(1) / o, (*this)(2) / o, (*this)(3) / o;
        return res;
    }

    /**< overloaded operator - (negative) */
    quat operator-() const {
        return ang::quat(- static_cast<const Eigen::Vector4d&>(*this));
    }

    /**< ===== ===== Other Methods ===== ===== */
    /**< returns adjoint quaternion (same as conjugate) */
    inline quat adjoint() const {
        quat res;
        res << (*this)(0), - (*this)(1), - (*this)(2), - (*this)(3);
        return res;
    }
    /**< returns conjugate quaternion (same as adjoint) */
    inline quat conjugate() const {
        quat res;
        res << (*this)(0), - (*this)(1), - (*this)(2), - (*this)(3);
        return res;
    }
}; // closes class quat

} // closes namespace ang

#endif
