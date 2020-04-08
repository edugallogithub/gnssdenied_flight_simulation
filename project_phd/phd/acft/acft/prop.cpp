#include "prop.h"

#include "math/logic/share.h"
#include "env/atm.h"
#include "ang/quat.h"
#include "ang/rotate/dcm.h"
#include <boost/filesystem.hpp>

// CLASS PROP
// ==========
// ==========

const std::string acft::prop::_filename = "apm/mugin_propulsion_cpp.txt";
/* name required internally */

acft::prop::prop(): _Pvec_J(nullptr), _Pvec_Ct(nullptr), _Pvec_Cp(nullptr), _Pfun_J2Ct(nullptr), _Pfun_J2Cp(nullptr) {
    boost::filesystem::path p0(math::share::phd_configuration_prefix);
    boost::filesystem::path p1(_filename);
    std::string file_st = (p0 / p1).string();
    std::ifstream mystream(file_st.c_str()); // create stream

    // vector of aspect ratios
    _Pvec_J = new math::vec1(mystream);
    // vector of propeller thrust coefficients
    _Pvec_Ct = new math::vec1(mystream);
    // vector of propeller power coefficients
    _Pvec_Cp = new math::vec1(mystream);

    // table obtaining Ct from J
    _Pfun_J2Ct = new math::f_table1V(*_Pvec_J, *_Pvec_Ct, math::logic::lagrange_first_precompute);

    // table obtaining Cp from J
    _Pfun_J2Cp = new math::f_table1V(*_Pvec_J, *_Pvec_Cp, math::logic::lagrange_first_precompute);

	mystream >> _Dp_m >> _Pmax_W >> _n_revps_min >> _n_revps_max;
	//mystream >> _x_prop_bfs_m(0) >> _x_prop_bfs_m(1) >> _x_prop_bfs_m(2);
	mystream >> _cf_m2ps2;

	mystream.close(); // close stream
}
/* empty constructor (based on internal text file) */

acft::prop::~prop() {
	delete _Pvec_J;
	delete _Pvec_Ct;
	delete _Pvec_Cp;
	delete _Pfun_J2Ct;
	delete _Pfun_J2Cp;
}
/* destructor */

double acft::prop::compute_engine_power(const double& deltaT, const double& p_pa, const double& T_degK) const {
	double delta = env::atm::p2delta(p_pa); // pressure ratio
	double theta = env::atm::T2theta(T_degK); // temperature ratio
	return std::min(_Pmax_W * deltaT, _Pmax_W * delta / sqrt(theta)); // power
}
/* returns the power produced by the power plant [W] based on the throttle parameter, the pressure,
and the temperature */

double acft::prop::compute_engine_speed(const double& P_W, const double& vtas_mps, const double& rho_kgm3, bool flag_single_loop) const {
    // function uses previous revolutions to accelerate loop convergence. THIS IS EXTREMELY DANGEROUS,
    // and I need to ensure that in every trajectory the first call is with flag_single_loop = false.
    // Two reasons for that:
    // 1. If first call is with true flag, the error of that call may be quite big as the previous
    //    hardcoded revolutions are not near the solution.
    // 2. If several trajectories are computed one after another, the results would be different
    //    than if called one by one, as the first call of the 1st trajectory has the hardcoded
    //    revolutions, while the 2nd, 3rd, ... have previous values that may be far from solution.
    // The code below solves the issue. If first call is with flag true, it will throw exception
    // in first trajectory.
    static double n_revps_prev = std::nan("");
    if (flag_single_loop == false) {
        n_revps_prev = -100.0;
    }
    else {
        if (std::isnan(n_revps_prev) == true) {
            throw std::runtime_error("Wrong initialization of prop class compute_engine_speed method.");
        }
    }

    static int counter = 0; // to avoid accumulating errors, once in a while it does not use differential
    counter++;
	// if initialized (or any other reason) outside of valid range, set within
	if ((n_revps_prev < _n_revps_min) || (n_revps_prev > _n_revps_max)) {
		n_revps_prev = 70.0; // within min-max (27-130), more or less in the middle
	}
	// use Taylor to obtain the angular velocity.
	// very few iterations should be necessary
	double DeltaCp = 1e6;
	double n_revps = 0.;
	while (fabs(DeltaCp) > 1e-5) {
        if (counter > 10000) { // called twice (2nd order integration) at 500 hz - every 10 sec use this block
            counter = 0;
            double n_revps0 = 70.0; // hardcoded initial value in between 27 - 130
            n_revps_u Func(this);
            std::vector<double> par(3);
            par[0] = vtas_mps;
            par[1] = P_W;
            par[2] = rho_kgm3;
            n_revps = Func.find_zero_secant(par, 0., n_revps0 - 5, n_revps0 + 5, 1e-11);
            n_revps_prev = n_revps;
            return n_revps;
        }
		DeltaCp = this->alt_power_coeff_difference(n_revps_prev, vtas_mps, P_W, rho_kgm3);
		double DeltadCp_dn = this->alt_power_coeff_difference_derivate(n_revps_prev, vtas_mps, P_W, rho_kgm3);
		n_revps = n_revps_prev - DeltaCp / DeltadCp_dn;
		if ((n_revps < _n_revps_min) || (n_revps > _n_revps_max)) {
			// sometimes just adding a derivative as above sends it out of bounds and never recovers
			double n_revps0 = 70.0; // hardcoded initial value in between 27 - 130
			n_revps_u Func(this);
			std::vector<double> par(3);
			par[0] = vtas_mps;
			par[1] = P_W;
			par[2] = rho_kgm3;
			n_revps = Func.find_zero_secant(par, 0., n_revps0 - 5, n_revps0 + 5, 1e-11);
			n_revps_prev = n_revps;
			return n_revps;
		}
		n_revps_prev = n_revps;
		if (flag_single_loop == true) {
			break;
		}
	}
    //double J = vtas_mps / n_revps / _Dp_m; ////////////////////////////////////////////////////
    //double Cp1 = _Pfun_J2Cp->value(J); ///////////////////////////////////
    //double Cp2 = P_W / (rho_kgm3 * pow(n_revps, 3) * pow(_Dp_m, 5)); ////////////////////////////////////
    //std::cout << vtas_mps << "   " << n_revps << "   "  << P_W << "   " << J << "  " << Cp1 << "  " << Cp2 << std::endl; ////////////////////////////////
	return n_revps;
}
/* returns the power plants angular speeds [revolutions per second] based on the true airspeed, the throttle parameter,
and the atmospheric pressure, temperature, and density. "flag_single_loop" is a boolen that if true imposes a single loop
iteration without checking the accuracy of the output. Should only be checked true if you know what you are doing. */

Eigen::Vector3d acft::prop::compute_propulsion_force(const double& n_revps, const double& vtas_mps, const double& rho_kgm3) const {
	//double omega_rps = 2 * math::constant::PI() * n_revps; // angular velocity
	double J = vtas_mps / n_revps / _Dp_m; // propeller advance ratio
	double Ct = _Pfun_J2Ct->value(J); // thrust coefficient
	double T_N = Ct * rho_kgm3 * pow(n_revps, 2) * pow(_Dp_m, 4); // thrust modulus
    return {T_N, 0.0, 0.0}; // propulsive force in BFS
}
/* return the propulsive force [N] expressed in BFS based on the power plant angular speed, the true
airspeed, and the air density. */

Eigen::Vector3d acft::prop::compute_propulsion_moment(const Eigen::Vector3d& f_pro_bfs_N, const double& n_revps, const double& P_W) const {
	//double omega_rps = 2 * math::constant::PI() * n_revps; // angular velocity
    //Eigen::Vector3d Q1_pro_Nm = f_pro_bfs_N.cross(_x_prop_bfs_m); // force moment
    //Eigen::Vector3d Q2_pro_Nm(-P_W / omega_rps, 0.0, 0.0); // torque
    return {-P_W / (2 * math::constant::PI() * n_revps), 0.0, 0.0};
    //return Q1_pro_Nm + Q2_pro_Nm; // moments in BFS
}
/* return the propulsive moment [Nm] expressed in BFS based on the propulsive force, the power
plant angular speed, and the engine power output */

double acft::prop::compute_fuel_consumption(const double& P_W) const {
	return -_cf_m2ps2 * P_W; // fuel consumption proportional to power
}
/* returns the fuel consumption [kgps] based on the power plant power output */

double acft::prop::alt_power_coeff_difference(const double& n_revps, const double& vtas_mps, const double& P_W, const double& rho_kgm3) const {
	double J = vtas_mps / n_revps / _Dp_m;
    double Cp1 = _Pfun_J2Cp->value(J);
    double Cp2 = P_W / (rho_kgm3 * pow(n_revps, 3) * pow(_Dp_m, 5));
	return Cp1 - Cp2;
}
/* returns difference between two different ways of computing the power coefficient.
It should be zero as both should coincide. */

double acft::prop::alt_power_coeff_difference_derivate(const double& n_revps, const double& vtas_mps, const double& P_W, const double& rho_kgm3) const {
	double J = vtas_mps / n_revps / _Dp_m;
    int posJ = _Pfun_J2Cp->compute_pos1(J);
    double dJ_dn = -J / n_revps;
    double dCp1_dn = _Pfun_J2Cp->compute_diff(posJ, J, dJ_dn);
    double dCp2_dn = -3.0 * P_W / (rho_kgm3 * pow(n_revps, 4) * pow(_Dp_m, 5));
    return dCp1_dn - dCp2_dn;

	//unsigned idJ = _J->compute_id(J);
	//double ratioJ = _J->compute_ratio(J, idJ);
	//double dJ_dn = -J / n_revps;
	//double dCp1_dJ = ((*_Cp)[idJ + 1] - (*_Cp)[idJ]) / _J->get_step();
	//double dCp2_dn = -3.0 * P_W / (rho_kgm3 * pow(n_revps, 4) * pow(_Dp_m, 5));
	//return dCp1_dJ * dJ_dn - dCp2_dn;
}
/* returns derivate with n_revps of the difference between two different ways of
computing the power coefficients, which should coincide. */

acft::prop::n_revps_u::n_revps_u(const prop* const prop_this) : _prop_this(prop_this) {}
/* encapsulates the n_revps function to be employed by other functions. */

double acft::prop::n_revps_u::exec(const double& n_revps, const std::vector<double>& par) {
	return _prop_this->alt_power_coeff_difference(n_revps, par[0], par[1], par[2]);
}
/* encapsulates the method to be employed by other functions. */

double acft::prop::compute_ct(const double& J) const {
    return _Pfun_J2Ct->value(J);
}
/* returns thrust coefficient based on aspect ratio */

double acft::prop::compute_cp(const double& J) const {
    return _Pfun_J2Cp->value(J);
}
/* returns power coefficient based on aspect ratio */

double acft::prop::compute_eta(const double& Ct, const double& Cp, const double& J) {
    return Ct * J / Cp;
}
/* returns propeller efficiency based on thrust coefficient, power coefficient, and aspect ratio */

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////




