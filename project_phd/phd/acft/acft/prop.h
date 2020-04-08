#ifndef ACFT_PROP
#define ACFT_PROP

#include "../acft.h"
#include "math/math/func.h"
#include "math/vec/vec1.h"
#include "math/pred/pred1v/f_table1V.h"
#include "ang/rotate/euler.h"
#include <string>

namespace acft {

// CLASS PROP
// ==========
// ==========

class ACFT_API prop {
private:
	/**< name required internally */
	static const std::string _filename;
    /**< pointer of vector of aspect ratioss */
    math::vec1* _Pvec_J;
    /**< pointer of vector of propeller thrust coefficients */
    math::vec1* _Pvec_Ct;
    /**< pointer of vector of propeller power coefficients */
    math::vec1* _Pvec_Cp;
    /**< pointer to table obtaining Ct from J */
    math::f_table1V* _Pfun_J2Ct;
    /**< pointer to table obtaining Cp from J */
    math::f_table1V* _Pfun_J2Cp;

	/**< propeller diameter */
	double _Dp_m;
	/**< engine max power at sea level standard conditions */
	double _Pmax_W;
	/**< min engine speed(1600[rpm] ~27[revps]) */
	double _n_revps_min;
	/**< max engine speed(7800[rpm] ~130[revps]) */
	double _n_revps_max;
	/**< approximate location of the center of the propeller
	with respect to the center of gravity */
    //Eigen::Vector3d _x_prop_bfs_m;
	/**< fuel consumption coefficient provided as a ratio between
	the power[W] and the fuel consumption[kg / sec]. */
	double _cf_m2ps2;

	/**< returns difference between two different ways of computing the power coefficient.
	It should be zero as both should coincide. */
	double alt_power_coeff_difference(const double& n_revps, const double& vtas_mps, const double& P_W, const double& rho_kgm3) const;
	/**< returns derivate with n_revps of the difference between two different ways of
	computing the power coefficients, which should coincide. */
	double alt_power_coeff_difference_derivate(const double& n_revps, const double& vtas_mps, const double& P_W, const double& rho_kgm3) const;
public:
    /**< default constructor (based on internal text file) */
    prop();
    /**< copy constructor */
    prop(const prop&) = delete;
    /**< move constructor */
    prop(prop&&) = delete;
    /**< destructor */
    ~prop();
    /**< copy assignment */
    prop& operator=(const prop&) = delete;
    /**< move assignment */
    prop& operator=(prop&&) = delete;

	/**< returns the power produced by the power plant [W] based on the throttle parameter, the pressure,
	and the temperature */
	double compute_engine_power(const double& deltaT, const double& p_pa, const double& T_degK) const;
	/**< returns the power plants angular speeds [revolutions per second] based on the true airspeed, the throttle parameter,
	and the atmospheric pressure, temperature, and density. "flag_single_loop" is a boolean that if true imposes a single loop
	iteration without checking the accuracy of the output. Should only be checked true if you know what you are doing. */
	double compute_engine_speed(const double& P_W, const double& vtas_mps, const double& rho_kgm3, bool flag_single_loop = false) const;
	/**< return the propulsive force expressed in BFS [N] based on the power plant angular speed, the true
	airspeed, and the air density. */
    Eigen::Vector3d compute_propulsion_force(const double& n_revps, const double& vtas_mps, const double& rho_kgm3) const;
	/**< return the propulsive mement [Nm] expressed in BFS based on the propulsive force, the power
	plant angular speed, and the engine power output */
    Eigen::Vector3d compute_propulsion_moment(const Eigen::Vector3d& f_pro_bfs_N, const double& n_revps, const double& P_W) const;
	/**< returns the fuel consumption [kgps] based on the power plant power output */
	double compute_fuel_consumption(const double& P_W) const;
    /**< returns thrust coefficient based on aspect ratio */
    double compute_ct(const double& J) const;
    /**< returns power coefficient based on aspect ratio */
    double compute_cp(const double& J) const;
    /**< returns propeller efficiency based on thrust coefficient, power coefficient, and aspect ratio */
    static double compute_eta(const double& Ct, const double& Cp, const double& J);
    /**< returns maximum power to read */
    const double& get_Pmax_W() const {return _Pmax_W;}
    /**< returns propeller diameter */
    const double& get_Dp_m() const {return _Dp_m;}
private:
	/**< encapsulates the method to be employed by other functions. */
	class n_revps_u : public math::func {
	private:
		const prop* const _prop_this;
	public:
		n_revps_u(const prop* const);
		double exec(const double& n_revps, const std::vector<double>& par);
	};

}; // closes class prop

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

}; // closes namespace acft

#endif


