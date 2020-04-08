#ifndef ANG_AUXILIARY
#define ANG_AUXILIARY

#include "ang.h"

#include <vector>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Geometry>

// EIGEN EXTENSIONS
// ================
// ================

namespace Eigen {
    typedef	Matrix< double, 4, 4 > Matrix4d;
    typedef Matrix< double, 6, 1 > Vector6d;
    typedef Matrix< double, 6, 6 > Matrix6d;
    typedef Matrix< double, 9, 1 > Vector9d;
    
    /**< adds the input 3D Eigen vector object associated rotation vector to the stream */
    ANG_API inline std::ostream& operator <<(std::ostream & out_str, const Eigen::Vector3d& v) {
        out_str << v.transpose();
        return out_str;
    }
    /**< adds the input 4D Eigen vector object to the stream */
    ANG_API inline std::ostream& operator <<(std::ostream & out_str, const Eigen::Vector4d& v) {
        out_str << v.transpose();
        return out_str;
    }
    /**< adds the input 6D Eigen vector object to the stream */
    ANG_API inline std::ostream& operator <<(std::ostream & out_str, const Eigen::Vector6d& v) {
        out_str << v.transpose();
        return out_str;
    }

    /**< adds the input 3D Eigen matrix object to the stream */
    ANG_API inline std::ostream& operator <<(std::ostream & out_str, const Eigen::Matrix3d& m) {
        out_str << m.row(0) << " | " << m.row(1) << " | " << m.row(2);
        return out_str;
    }
    /**< adds the input 4D Eigen matrix object to the stream */
    ANG_API inline std::ostream& operator <<(std::ostream & out_str, const Eigen::Matrix4d& m) {
        out_str << m.row(0) << " | " << m.row(1) << " | " << m.row(2) << " | " << m.row(3);
        return out_str;
    }
} // close namespace Eigen

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#endif
