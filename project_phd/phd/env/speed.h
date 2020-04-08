#ifndef ENV_SPEED
#define ENV_SPEED

#include "env.h"
#include "ang/rotate/euler.h"

/*
This file contains the static class "speed", with methods related to the computation
of the true airspeed vector in different reference frames, as well as the conversion
between angular velocity and kinetic moment. */

namespace env {

// CLASS SPEED
// ===========
// ===========

class ENV_API speed {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	/**< compute the three true airspeed BFS (Body Fixed System) components [mps] based
	on the true airspeed modulus, the angle of attack, and the angle of sideslip. */
	static Eigen::Vector3d vtas2vtasbfs(const double& vtas_mps, const ang::euler& euler_wb);
	/**< compute the three true airspeed NED (North - East - Down) components [mps]
	based on the true airspeed modulus and the NED to WFS Euler angles. */
	static Eigen::Vector3d vtas2vtasned(const double& vtas_mps, const ang::euler& euler_nw);
	/**< compute the airspeed modulus [mps] based on the three true airspeed BFS
	(Body Fixed System) components. */
	static double vtasbfs2vtas(const Eigen::Vector3d& vtas_b_mps);
	/**< compute the WFS to BFS Euler angles (minus angles of sideslip, angle of attack,
	zero) based on the three true airspeed BFS (Body Fixed System) components. */
	static ang::euler vtasbfs2euler_wb(const Eigen::Vector3d& vtas_b_mps);
    /**< compute the WFS to BFS Euler angles (minus angles of sideslip, angle of attack,
    zero) based on the three true airspeed BFS (Body Fixed System) components and its norm. */
    static ang::euler vtasbfs2euler_wb(const Eigen::Vector3d& vtas_b_mps, const double& vtas_mps);
	/**< Compute the differential with time of the true airspeed vector in
	BFS(Body Fixed System) [mps2] based on the true airspeed modulus, the angle of
	attack, the angle of sideslip, and their differentials with time. */
	static Eigen::Vector3d vtasbfsdot(const double& vtas_mps, const ang::euler& euler_wb, const double& dvtas_dt_mps2, const Eigen::Vector3d& deuler_wb_dt_rps);
	/**< computes the kinetic moment [Nms] as the product of the inertial tensor by the angular speed. */
	static Eigen::Vector3d omega2h(const Eigen::Matrix3d& I_kgm2, const Eigen::Vector3d& w_rps);
	/**< computes the angular speed [rps] based on the inertial tensor and kinetic moment */
	static Eigen::Vector3d h2omega(const Eigen::Matrix3d&I_kgm2, const Eigen::Vector3d& h_Nms);
}; // closes class speed

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

}; // closes namespace env

#endif


