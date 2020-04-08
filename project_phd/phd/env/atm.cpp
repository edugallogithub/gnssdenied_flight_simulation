#include "atm.h"
#include "geo.h"

// CLASS ATM
// =========
// =========

const double	env::atm::_R = 287.05287;
/* air specific constant [J/(kg*degK)] = [m2/(degK*s2)] */
const double	env::atm::_kappa = 1.4;
/* air adiabatic index [-] */
const double	env::atm::_betaT = -6.5e-3;
/* troposphere thermal gradient [degK/m] */

const double	env::atm::_Hp0_m(0.);
/* standard mean sea level pressure altitude */
const double	env::atm::_p0_pa(101325.);
/* standard mean sea level pressure */
const double	env::atm::_T0_degK(288.15);
/* standard mean sea level temperature */

env::atm::atm(const double& DeltaT_degK, const double& Deltap_pa)
: _DeltaT_degK(DeltaT_degK), _Deltap_pa(Deltap_pa) {
}
/* constructor based on temperature and pressure differentials at mean sea level. */

env::atm::atm(const atm& Oatm)
: _DeltaT_degK(Oatm._DeltaT_degK), _Deltap_pa(Oatm._Deltap_pa) {
}
/* copy constructor */

env::atm::atm(atm&& Oatm)
: _DeltaT_degK(Oatm._DeltaT_degK), _Deltap_pa(Oatm._Deltap_pa) {
}
/* move constructor */

env::atm& env::atm::operator=(const atm& op2) {
	_DeltaT_degK = op2._DeltaT_degK;
	_Deltap_pa = op2._Deltap_pa;
	return *this;
}
/* copy assignment */

env::atm& env::atm::operator=(atm&& op2) {
    _DeltaT_degK = op2._DeltaT_degK;
    _Deltap_pa = op2._Deltap_pa;
    return *this;
}
/* move assignment */

void env::atm::set(const double& DeltaT_degK, const double& Deltap_pa) {
    _DeltaT_degK = DeltaT_degK;
    _Deltap_pa = Deltap_pa;
}
/* sets the temperature and pressure offsets at mean sea level */

/* ===== ===== ===== Static Functions ===== ===== ===== */
/* ==================================================== */

double env::atm::Hp2p(const double& Hp_m) {
	return _p0_pa * pow((1. + _betaT * Hp_m / _T0_degK), (-env::geo::_g0_mps2 / _R / _betaT));
}
/* returns atmospheric pressure [pa] from pressure altitude. */

double env::atm::p2Hp(const double& p_pa) {
	return (_T0_degK / _betaT) * (pow((p_pa / _p0_pa), (-_betaT * _R / env::geo::_g0_mps2)) - 1);
}
/* returns pressure altitude [m] from atmospheric pressure */

double env::atm::obtain_dp_dHp(const double& p_pa, const double& Hp_m) {
    return - env::geo::_g0_mps2 * p_pa / _R / (_T0_degK + _betaT * Hp_m);
}
/* returns the differential of atmospheric pressure [pa] with respect to
 * pressure altitude [m] based on both of them (for speed reasons to avoid
 * recomputing one of them) */

double env::atm::Hp2Tisa(const double& Hp_m) {
	return _T0_degK + _betaT * Hp_m;
}
/* returns the standard temperature [degK] based on the pressure altitude */

double env::atm::Hp2T(const double& Hp_m, const double& DeltaT_degK) {
    return _T0_degK + DeltaT_degK + _betaT * Hp_m;
}
/* returns the temperature [degK] based on the pressure altitude and temperature offset */

double env::atm::Hp2H(const double& Hp_m, const double& DeltaT_degK, const double& Deltap_pa) {
    // mean sea level pressure
    double p_MSL_pa = _p0_pa + Deltap_pa;
    // mean sea level pressure altitude
    double Hp_MSL_m = (_T0_degK / _betaT) * (pow((p_MSL_pa / _p0_pa), (- _betaT * _R / env::geo::_g0_mps2)) - 1);
    // mean sea level temperature
    double T_MSL_degK = _T0_degK + DeltaT_degK + _betaT * Hp_MSL_m;
    // geopotential altitude
    return Hp_m - Hp_MSL_m + (DeltaT_degK / _betaT) * log((_T0_degK + _betaT * Hp_m) / (T_MSL_degK - DeltaT_degK));
}
/* returns the geopotential altitude [m] from the pressure altitude, the temperature
 * offset, and the pressure offset. */

double env::atm::H2Hp(const double& H_m, const double& DeltaT_degK, const double& Deltap_pa, bool flag_single_loop) {
    // function uses previous altitude to accelerate loop convergence. THIS IS EXTREMELY DANGEROUS,
    // and I need to ensure that in every trajectory the first call is with flag_single_loop = false.
    // Two reasons for that:
    // 1. If first call is with true flag, the error of that call may be quite big as the previous
    //    hardcoded altitude is not near the solution.
    // 2. If several trajectories are computed one after another, the results would be different
    //    than if called one by one, as the first call of the 1st trajectory has the hardcoded
    //    altitude, while the 2nd, 3rd, ... have previous values that may be far from solution.
    // The code below solves the issue. If first call is with flag true, it will throw exception
    // in first trajectory.
    static double Hp_m_prev = std::nan("");
    if (flag_single_loop == false) {
        Hp_m_prev = 700;
    }
    else {
        if (std::isnan(Hp_m_prev) == true) {
            throw std::runtime_error("Wrong initialization of atm class H2Hp method.");
        }
    }
    
    // use Taylor to obtain the pressure altitude.
    // very few iterations should be necessary
    double Deltaf = 1e6;
    double Hp_m = 0.;
    while (fabs(Deltaf) > 1e-4) {
        Deltaf = env::atm::alt_difference(Hp_m_prev, H_m, DeltaT_degK, Deltap_pa);
        double Deltadf_dHp = env::atm::alt_difference_derivate(Hp_m_prev, H_m, DeltaT_degK);
        Hp_m = Hp_m_prev - Deltaf / Deltadf_dHp;
        if ((Hp_m < 0.0) || (Hp_m > 10000.0)) {
            // sometimes just adding a derivative as above sends it out of bounds and never recovers
            Hp2H_bis_u Func;
            std::vector<double> par(2);
            par[0] = DeltaT_degK;
            par[1] = Deltap_pa;
            Hp_m = Func.find_zero_secant(par, H_m, H_m-5, H_m+5, 1e-11);
            Hp_m_prev = Hp_m;
            return Hp_m;
        }
        Hp_m_prev = Hp_m;
        if (flag_single_loop == true) {
            break;
        }
    }
    return Hp_m;
}
/* returns the pressure altitude [m] based on the geopotential altitude, the temperature
 * offset, and the pressure offset. "flag_single_loop" is a boolean that if true imposes a
 * single loop iteration without checking the accuracy of the output. Should only be checked
 * true if you know what you are doing. */

double env::atm::T2a(const double& T_degK) {
    return sqrt(T_degK * _kappa * _R);
}
/* returns speed of sound [mps] based on temperature */

double env::atm::pT2rho(const double& T_degK, const double& p_pa) {
    return p_pa / (T_degK * _R);
}
/* returns density [kgm3] based on temperature and pressure */

double env::atm::p2delta(const double& p_pa) {
    return p_pa / _p0_pa;
}
/* returns atmospheric pressure ratio delta [-] from atmospheric pressure */

double env::atm::T2theta(const double& T_degK) {
    return T_degK / _T0_degK;
}
/* returns atmospheric temperature ratio theta [-] from atmospheric temperature */

double env::atm::rho2sigma(const double& rho_kgm3) {
    return rho_kgm3 / env::atm::pT2rho(_T0_degK, _p0_pa);
}
/* returns atmospheric density sigma [-] from atmospheric density */

double env::atm::a2alpha(const double& a_mps) {
    return a_mps / atm::T2a(_T0_degK);
}
/* returns speed of sound ratio alpha [-] from speed of sound */

double env::atm::compute_tas(const double& p_pa, const double& rho_kgm3, const double& pt_pa) {
    return sqrt(2.0 * _kappa * p_pa / (_kappa - 1) / rho_kgm3 * (pow(((pt_pa - p_pa) / p_pa + 1), ((_kappa - 1) / _kappa)) - 1));
}
/* return true airspeed [mps] based on static pressure, density, and dynatmic pressure. */

double env::atm::compute_pt(const double& p_pa, const double& M) {
    return p_pa * pow((1 + (_kappa - 1) / 2 * pow(M, 2)), (_kappa / (_kappa - 1)));
}
/* return dynamic pressure [pa] based on static pressure and Mach number */

double env::atm::vtas2M(const double& vtas_mps, const double& a_mps) {
    return vtas_mps / a_mps;
}
/* return Mach number [-] based on true airspeed and speed of sound */

double env::atm::obtain_DeltaT_degK(const double& Hp_m, const double& T_degK) {
    double Tisa_degK = env::atm::Hp2Tisa(Hp_m);
    return T_degK - Tisa_degK;
}
/* return the temperature offset at mean sea level based on the temperature and the pressure altitude */

double env::atm::obtain_Deltap_pa(const double& DeltaT_degK, const double& Hp_m, const double& H_m, bool flag_single_loop) {
    // function uses previous mean sea level pressure altitude to accelerate loop convergence. THIS IS EXTREMELY DANGEROUS,
    // and I need to ensure that in every trajectory the first call is with flag_single_loop = false.
    // Two reasons for that:
    // 1. If first call is with true flag, the error of that call may be quite big as the previous
    //    hardcoded altitude is not near the solution.
    // 2. If several trajectories are computed one after another, the results would be different
    //    than if called one by one, as the first call of the 1st trajectory has the hardcoded
    //    altitude, while the 2nd, 3rd, ... have previous values that may be far from solution.
    // The code below solves the issue. If first call is with flag true, it will throw exception
    // in first trajectory.
    static double Hp_m_msl_prev = std::nan("");
    if (flag_single_loop == false) {
        Hp_m_msl_prev = 0.0;
    }
    else {
        if (std::isnan(Hp_m_msl_prev) == true) {
            throw std::runtime_error("Wrong initialization of atm class obtain_Deltap_pa method.");
        }
    }
    
    static int counter = 0; // to avoid accumulating errors, once in a while it does not use differential
    counter++;
    // use Taylor to obtain the converge quicker (very few iterations should be necessary)
    double DeltaH = 1e7;
    double Hp_m_msl = 0.;
    while (fabs(DeltaH) > 1e-7) {
        if (counter > 100) { // called once at 100 hz - every 1 sec use this block
            counter = 0;
            env::atm::Hp_MSL_aux_u Func;
            std::vector<double> par(3);
            par[0] = H_m;
            par[1] = Hp_m;
            par[2] = DeltaT_degK;
            Hp_m_msl = Func.find_zero_secant(par, 0., -200, +200, 1e-12);
            Hp_m_msl_prev = Hp_m_msl;
            return env::atm::Hp2p(Hp_m_msl) - _p0_pa;
        }
        DeltaH = env::atm::obtain_Hp_MSL_aux(Hp_m_msl_prev, H_m, Hp_m, DeltaT_degK);
        double DeltaH_dHpmsl = env::atm::obtain_Hp_MSL_aux_diff(Hp_m_msl_prev, DeltaT_degK);
        Hp_m_msl = Hp_m_msl_prev - DeltaH / DeltaH_dHpmsl;
        if (std::fabs(Hp_m_msl) > 500) {
            // sometimes just adding a derivative as above sends it out of bounds and never recovers
            env::atm::Hp_MSL_aux_u Func;
            std::vector<double> par(3);
            par[0] = H_m;
            par[1] = Hp_m;
            par[2] = DeltaT_degK;
            Hp_m_msl = Func.find_zero_secant(par, 0., -200, +200, 1e-12);
            Hp_m_msl_prev = Hp_m_msl;
            return env::atm::Hp2p(Hp_m_msl) - _p0_pa;
        }
        Hp_m_msl_prev = Hp_m_msl;
        if (flag_single_loop == true) {
            break;
        }
    }
    return env::atm::Hp2p(Hp_m_msl) - _p0_pa;
}
/* return the pressure offset at mean sea level based on the temperature offset at mean sea level,
* pressure altitude, and geopotential altitude. "flag_single_loop" is a boolean that if true imposes a single loop
iteration without checking the accuracy of the output. Should only be checked true if you know what you are doing. */

/* ===== ===== ===== Non Static Functions ===== ===== ===== */
/* ======================================================== */

double env::atm::Hp2T(const double& Hp_m) const {
	return _T0_degK + _DeltaT_degK + _betaT * Hp_m;
}
/* returns the temperature [degK] based on the pressure altitude */

double env::atm::Hp2H(const double& Hp_m) const {
	// mean sea level pressure
	double p_MSL_pa = _p0_pa + _Deltap_pa;
	// mean sea level pressure altitude
	double Hp_MSL_m = (_T0_degK / _betaT) * (pow((p_MSL_pa / _p0_pa), (- _betaT * _R / env::geo::_g0_mps2)) - 1);
	// mean sea level temperature
	double T_MSL_degK = _T0_degK + _DeltaT_degK + _betaT * Hp_MSL_m;
	// geopotential altitude
	return Hp_m - Hp_MSL_m + (_DeltaT_degK / _betaT) * log((_T0_degK + _betaT * Hp_m) / (T_MSL_degK - _DeltaT_degK));
}
/* returns the geopotential altitude [m] from the pressure altitude */

double env::atm::H2Hp(const double& H_m, bool flag_single_loop) const {
    // function uses previous altitude to accelerate loop convergence. THIS IS EXTREMELY DANGEROUS,
    // and I need to ensure that in every trajectory the first call is with flag_single_loop = false.
    // Two reasons for that:
    // 1. If first call is with true flag, the error of that call may be quite big as the previous
    //    hardcoded altitude is not near the solution.
    // 2. If several trajectories are computed one after another, the results would be different
    //    than if called one by one, as the first call of the 1st trajectory has the hardcoded
    //    altitude, while the 2nd, 3rd, ... have previous values that may be far from solution.
    // The code below solves the issue. If first call is with flag true, it will throw exception
    // in first trajectory.
    static double Hp_m_prev = std::nan("");
    if (flag_single_loop == false) {
        Hp_m_prev = 700;
    }
    else {
        if (std::isnan(Hp_m_prev) == true) {
            throw std::runtime_error("Wrong initialization of atm class H2Hp method.");
        }
    }

	// use Taylor to obtain the pressure altitude.
	// very few iterations should be necessary
	double Deltaf = 1e6;
	double Hp_m = 0.;
	while (fabs(Deltaf) > 1e-4) {
		Deltaf = this->alt_difference(Hp_m_prev, H_m);
		double Deltadf_dHp = this->alt_difference_derivate(Hp_m_prev, H_m);
		Hp_m = Hp_m_prev - Deltaf / Deltadf_dHp;
		if ((Hp_m < 0.0) || (Hp_m > 10000.0)) {
			// sometimes just adding a derivative as above sends it out of bounds and never recovers
				Hp2H_u Func(this);
			std::vector<double> par(0);
			Hp_m = Func.find_zero_secant(par, H_m, H_m-5, H_m+5, 1e-11);
			Hp_m_prev = Hp_m;
			return Hp_m;
		}
		Hp_m_prev = Hp_m;
		if (flag_single_loop == true) {
			break;
		}
	}
	return Hp_m;
}
/* returns the pressure altitude [m] based on the geopotential
altitude. "flag_single_loop" is a boolean that if true imposes a single loop
iteration without checking the accuracy of the output. Should only
be checked true if you know what you are doing. */

/* ===== ===== ===== Private Functions ===== ===== ===== */
/* ===================================================== */

double env::atm::alt_difference(const double& Hp_m, const double& H_m) const {
	return H_m - this->Hp2H(Hp_m);
}
/* returns difference between the input geopotential altitude and
that computed from the input pressure altitude. It should
be zero as both should coincide. */

double env::atm::alt_difference(const double& Hp_m, const double& H_m, const double& DeltaT_degK, const double& Deltap_pa) {
    return H_m - env::atm::Hp2H(Hp_m, DeltaT_degK, Deltap_pa);
}
/* returns difference between the input geopotential altitude and that computed from the
 * input pressure altitude based on the temperature and pressure offsets. It should be zero
 * as both should coincide. */

double env::atm::alt_difference_derivate(const double&Hp_m, const double& H_m) const {
	return -(_T0_degK + _DeltaT_degK + _betaT * Hp_m) / (_T0_degK + _betaT * Hp_m);
}
/* returns derivate with pressure altitude of the difference
between the input geopotential altitude and that computed from the
input pressure altitude, which should coincide. */

double env::atm::alt_difference_derivate(const double&Hp_m, const double& H_m, const double& DeltaT_degK) {
    return -(_T0_degK + DeltaT_degK + _betaT * Hp_m) / (_T0_degK + _betaT * Hp_m);
}
/* returns derivate with pressure altitude of the difference between the input geopotential altitude and
 * that computed from the input pressure altitude and the temperature offset, which should coincide. */

env::atm::Hp2H_u::Hp2H_u(const atm* const atm_this) : _atm_this(atm_this) {}
/* encapsulates the Hp2H function to be employed by other functions. */

double env::atm::Hp2H_u::exec(const double& Hp, const std::vector<double>& par) {
	return _atm_this->Hp2H(Hp);
}

double env::atm::Hp2H_bis_u::exec(const double& Hp, const std::vector<double>& par) {
    // par[0] == DeltaT_degK
    // par[1] == Deltap_pa
    return env::atm::Hp2H(Hp, par[0], par[1]);
}

double env::atm::obtain_Hp_MSL_aux(const double& Hp_MSL_m, const double& H_m, const double& Hp_m, const double& DeltaT_degK) {
    return H_m - Hp_m + Hp_MSL_m - DeltaT_degK / env::atm::_betaT * log((env::atm::_T0_degK + env::atm::_betaT * Hp_m) / (env::atm::_T0_degK + env::atm::_betaT * Hp_MSL_m));
}
/* based on the function H = H(Hp, DeltaT, Deltap), this function should always return 0 if the inputs are
 * compatible among themselves. Intended to be used as part of iteration to obtain pressure altitude at
 * mean sea level. Other inputs are geopotential altitude, pressure altitude, and temperature offset. */

double env::atm::obtain_Hp_MSL_aux_diff(const double& Hp_MSL_m, const double& DeltaT_degK) {
    return 1.0 + DeltaT_degK / (env::atm::_T0_degK + env::atm::_betaT * Hp_MSL_m);

}
/* differential with respect to Hp_MSL_m of the obtain_Hp_MSL_aux function */

double env::atm::Hp_MSL_aux_u::exec(const double& Hp_MSL_m, const std::vector<double>& par) {
    // par[0] == H_m
    // par[1] == Hp_m
    // par[2] == DeltaT_degK;
    return env::atm::obtain_Hp_MSL_aux(Hp_MSL_m, par[0], par[1], par[2]);
}







































