#ifndef ACFT_AERO
#define ACFT_AERO

#include "../acft.h"
#include "aero.h"
#include "ang/rotate/euler.h"
#include <Eigen/Core>

/*
 * This file contains the Aerodynamic Force Model base class.
 */

namespace acft {

// CLASS AERO
// ==========
// ==========

class ACFT_API aero {
protected:
    /**< wing surface */
    double _S_m2;
    /**< wing chord */
    double _c_m;
    /**< wing span */
    double _b_m;
    /**< convenience coefficients */
    Eigen::Array3d _bc;
    /**< center of gravity position */
    Eigen::Array3d _xcg_m;

public:
    /**< empty constructor */
    aero() = default;
    /**< copy constructor */
    aero(const aero&) = delete;
    /**< move constructor */
    aero(aero&&) = delete;
    /**< destructor */
    virtual ~aero() = default;
    /**< copy assignment */
    aero& operator=(const aero&) = delete;
    /**< move assignment */
    aero& operator=(aero&&) = delete;

    /**< computes the aerodynamic forces coefficient based on the true airspeed, the Euler angles
    from WFS to BFS (angles of attack and sideslip), the control parameters, and the angular
    velocity of BFS over NED expressed in BFS. */
    virtual Eigen::Vector3d obtain_cf_aer(const double& vtas_mps, const ang::euler& euler_wfsbfs_rad, const Eigen::Array4d& delta_control, const Eigen::Vector3d& w_nedbfsbfs_rps) const = 0;
    /**< converts aerodynamic force coefficcient into aerodynamic force */
    virtual Eigen::Vector3d cfaer2faer(const Eigen::Vector3d& cf_aer_bfs, const double& rho_kgm3, const double& vtas_mps) const
    {return cf_aer_bfs * .5 * rho_kgm3 * _S_m2 * pow(vtas_mps, 2);}
    /**< computes the aerodynamics moments coefficient based on the true airspeed, the Euler angles
    from WFS to BFS (angles of attack and sideslip), the control parameters, and the angular
    velocity of BFS over NED expressed in BFS. */
    virtual Eigen::Vector3d obtain_cm_aer(const double& vtas_mps, const ang::euler& euler_wfsbfs_rad, const Eigen::Array4d& delta_control, const Eigen::Vector3d& w_nedbfsbfs_rps) const = 0;
    /**< converts aerodynamic moment coefficient into aerodynamic moment */
    virtual Eigen::Vector3d cmaer2maer(const Eigen::Vector3d& cm_aer_bfs, const double& rho_kgm3, const double& vtas_mps) const
    {return (cm_aer_bfs.array() * _bc).matrix() * rho_kgm3 * _S_m2 * pow(vtas_mps,2);}
}; // closes class aero3

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

} // closes namespace acft

#endif

