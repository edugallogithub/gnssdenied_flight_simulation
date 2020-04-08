#ifndef ENV_ATM
#define ENV_ATM

#include "env.h"
#include "math/math/func.h"

/*
This file contains the atmospheric model class "am". The constructor is based on the temperature
and pressure offsets at mean sea level. It contains methods showing the relationships between
pressure altitude Hp, geopotential pressure H, temperature T, pressure p, density
rho, and speed of sound a, as well as methods to obtain the dynamic pressure pt, true airspeed
vtas, and Mach number M.
 */

namespace env {

// CLASS ATM
// =========
// =========

class ENV_API atm {
public:
	/**< air specific constant [J/(kg*degK)] = [m2/(degK*s2)] */
	static const double _R;
	/**< air adiabatic index [-] */
	static const double _kappa;
	/**< troposphere thermal gradient [degK/m] */
	static const double _betaT;

	/**< standard mean sea level pressure altitude */
	static const double _Hp0_m;
	/**< standard mean sea level pressure */
	static const double _p0_pa;
	/**< standard mean sea level temperature */
	static const double _T0_degK;

	/**< returns difference between the input geopotential altitude and that computed from the
	 * input pressure altitude. It should be zero as both should coincide. */
	double alt_difference(const double& Hp_m, const double& H_m) const;
    /**< returns difference between the input geopotential altitude and that computed from the
     * input pressure altitude based on the temperature and pressure offsets. It should be zero
     * as both should coincide. */
    static double alt_difference(const double& Hp_m, const double& H_m, const double& DeltaT_degK, const double& Deltap_pa);
	/**< returns derivate with pressure altitude of the difference between the input geopotential altitude and
	 * that computed from the input pressure altitude, which should coincide. */
	double alt_difference_derivate(const double&Hp_m, const double& H_m) const;
    /**< returns derivate with pressure altitude of the difference between the input geopotential altitude and
     * that computed from the input pressure altitude and the temperature offset, which should coincide. */
    static double alt_difference_derivate(const double&Hp_m, const double& H_m, const double& DeltaT_degK);
    /**< based on the function H = H(Hp, DeltaT, Deltap), this function should always return 0 if the inputs are
    * compatible among themselves. Intended to be used as part of iteration to obtain pressure altitude at
    * mean sea level. Other inputs are geopotential altitude, pressure altitude, and temperature offset. */
    static double obtain_Hp_MSL_aux(const double& Hp_MSL_m, const double& H_m, const double& Hp_m, const double& DeltaT_degK);
    /**< differential with respect to Hp_MSL_m of the obtain_Hp_MSL_aux function */
    static double obtain_Hp_MSL_aux_diff(const double& Hp_MSL_m, const double& DeltaT_degK);
private:
	/**< temperature offset at mean sea level */
	double _DeltaT_degK;
	/**< pressure offset at mean sea level */
	double _Deltap_pa;
public:
	/**< default constructor (attributes not initialized) */
    atm() = default;
	/**< constructor based on temperature and pressure differentials at mean sea level. */
	atm(const double& DeltaT_degK, const double& Deltap_pa);
    /**< copy constructor */
	atm(const atm&);
    /**< move constructor */
    atm(atm&&);
    /**< destructor */
    ~atm() = default;
    /**< copy assignment */
    atm& operator=(const atm&);
    /**< move assignment */
    atm& operator=(atm&&);

	/**< sets the temperature and pressure offsets at mean sea level */
	void set(const double& DeltaT_degK, const double& Deltap_pa);
	/**< returns the value of the temperature offset at mean sea level */
	inline const double& get_DeltaT_degK() const {return _DeltaT_degK;}
	/**< returns the value of the pressure offset at mean sea level. */
	inline const double& get_Deltap_pa() const {return _Deltap_pa;}

    /**< ===== ===== ===== Static Functions ===== ===== ===== */
    /**< ==================================================== */
	/**< returns atmospheric pressure [pa] from pressure altitude. */
	static double Hp2p(const double& Hp_m);
	/**< returns pressure altitude [m] from atmospheric pressure */
	static double p2Hp(const double& p_pa);
    /**< returns the differential of atmospheric pressure [pa] with respect to pressure altitude
     * based on both of them (for speed reasons to avoid recomputing one of them) */
    static double obtain_dp_dHp(const double& p_pa, const double& Hp_m);
	/**< returns the standard temperature [degK] based on the pressure altitude */
	static double Hp2Tisa(const double& Hp_m);
    /**< returns the temperature [degK] based on the pressure altitude and temperature offset */
    static double Hp2T(const double& Hp_m, const double& DeltaT_degK);
    /**< returns the geopotential altitude [m] from the pressure altitude, the temperature
     * offset, and the pressure offset. */
    static double Hp2H(const double& Hp_m, const double& DeltaT_degK, const double& Deltap_pa);
    /**< returns the pressure altitude [m] based on the geopotential altitude, the temperature
     * offset, and the pressure offset. "flag_single_loop" is a boolean that if true imposes a
     * single loop iteration without checking the accuracy of the output. Should only be checked
     * true if you know what you are doing. */
    static double H2Hp(const double& H_m, const double& DeltaT_degK, const double& Deltap_pa, bool flag_single_loop = false);
    /**< returns speed of sound [mps] based on temperature */
    static double T2a(const double& T_degK);
    /**< returns density [kgm3] based on temperature and pressure */
    static double pT2rho(const double& T_degK, const double& p_pa);
    /**< returns atmospheric pressure ratio delta [-] from atmospheric pressure */
    static double p2delta(const double& p_pa);
    /**< returns atmospheric temperature ratio theta [-] from atmospheric temperature */
    static double T2theta(const double& T_degK);
    /**< returns atmospheric density sigma [-] from atmospheric density */
    static double rho2sigma(const double& rho_kgm3);
    /**< returns speed of sound ratio alpha [-] from speed of sound */
    static double a2alpha(const double& a_mps);
    /**< return true airspeed [mps] based on static pressure, density, and dynamic pressure. */
    static double compute_tas(const double& p_pa, const double& rho_kgm3, const double& pt_pa);
    /**< return dynamic pressure [pa] based on static pressure and Mach number */
    static double compute_pt(const double& p_pa, const double& M);
    /**< return Mach number [-] based on true airspeed and speed of sound */
    static double vtas2M(const double& vtas_mps, const double& a_mps);
    /**< return the temperature offset at mean sea level based on the temperature and the pressure altitude */
    static double obtain_DeltaT_degK(const double& Hp_m, const double& T_degK);
    /**< return the pressure offset at mean sea level based on the temperature offset at mean sea level,
    * pressure altitude, and geopotential altitude. "flag_single_loop" is a boolean that if true imposes a single loop
    iteration without checking the accuracy of the output. Should only be checked true if you know what you are doing. */
    static double obtain_Deltap_pa(const double& DeltaT_degK, const double& Hp_m, const double& H_m, bool flag_single_loop);

    /**< ===== ===== ===== Non Static Functions ===== ===== ===== */
    /**< ======================================================== */
    /**< returns the temperature [degK] based on the pressure altitude */
	double Hp2T(const double& Hp_m) const;
	/**< returns the geopotential altitude [m] from the pressure altitude */
	double Hp2H(const double& Hp_m) const;
	/**< returns the pressure altitude [m] based on the geopotential
	altitude. "flag_single_loop" is a boolean that if true imposes a single loop
	iteration without checking the accuracy of the output. Should only
	be checked true if you know what you are doing. */
	double H2Hp(const double& H_m, bool flag_single_loop = false) const;
private:
	/**< encapsulates the Hp2H function to be employed by other functions. */
	class Hp2H_u : public math::func {
	private:
		const atm* const _atm_this;
	public:
		explicit Hp2H_u(const atm* const);
		double exec(const double& Hp, const std::vector<double>& par) override;
	};

    /**< encapsulates the Hp2H function to be employed by other functions. */
    class Hp2H_bis_u : public math::func {
    public:
        double exec(const double& Hp, const std::vector<double>& par) override;
    };

    /**< encapsulates the obtain_Hp_MSL_aux function to be employed by other functions. */
    class Hp_MSL_aux_u : public math::func {
    public:
        double exec(const double& Hp_MSL_m, const std::vector<double>& par) override;
    };

}; // closes class atm

}; // closes namespace env

#endif


