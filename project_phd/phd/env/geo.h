#ifndef ENV_GEO
#define ENV_GEO

#include "env.h"
#include "coord.h"
#include "mag.h"
#include "ang/rotate/euler.h"
#include "ang/rotate/dcm.h"
#include "math/pred/pred2v/f_table2V.h"

namespace env {

/* geo       --> ellipsoidal Earth coordinates, radii, H-h, gravity & centrifugal
 * geo_mix   --> ellipsoidal Earth coordinates, radii, gravity & centrifugal, spherical Earth H-h (THIS IS THE ONE TO USE)
 * geo_mix11 --> geo_mix but zero Earth angular velocity (w_ie), zero motion angular
 *               velocity (w_en), zero Coriolis acceleration, zero transport acceleration
 */

/* ===== ===== ===== GRAVITATION VERSUS GRAVITY ===== ===== =====
 * Gravity is the sum of gravitation plus centrifugal acceleration. For years I have been
 * computing gravitation based on the first two terms of the EGM96 model and then adding the
 * exact centrifugal acceleration, but the problem with this approach is that the resultant
 * is not orthogonal to the WGS84 ellipsoid, as it should be. There is a North-South component,
 * very small, but real. Then I found the WGS84 ellipsoidal model, which directly provides
 * gravity, and it only has one component in NED. The results are extremely similar between
 * both approaches, but the second is the one to use. If gravitation is desired, subtract
 * centrifugal acceleration from gravity.
 */

/* ===== ===== ===== GRAVITY ===== ===== =====
 * It is important to distinguish between the true gravity field and that provided by a model, as the
 * former influences the aircraft motion and is measured by the accelerometers while the later
 * represents our best knowledge of it based on position.
 * This file computes gravity based on the simplified WGS84 ellipsoidal formula. More accurate versions
 * based on many terms of the EGM96 Legendre polynomials could be implemented. The reason not to use a more
 * detailed model (with more EGM96 coefficients) is programming complexity and computing resources. If that
 * were the case, the results would be closer to reality, but that is not the point. The important fact
 * is to recognize that we have a model, not reality. We can assume with no loss of generality that
 * the differences between the model and reality are those with more accurate models, instead of our
 * simplified implementations. Complying with this is just a matter of adding more
 * coefficients to the model.
 * I base my modeling of the differences between the real and modelled gravity on the Chatfield book,
 * with some assistance from the paper "Gravity Modeling for Variable Fidelity Environments, Massen2006".
 * With respect to the ellipsoidal model, the real gravity is computed as follows, varying the model in three
 * different ways:
 * - 1 --> modifies the gravitation modulus according to a normal distribution of zero mean and
 *         standard deviation 1e-4 [mps2].
 * - 2 --> rotates the gravitation vector around a rotation vector contained in the horizontal plane
 *         (this is, orthogonal to the gravity) of angle taken from a normal distribution of zero mean
 *         and standard deviation 0.0028 [deg], ...
 * - 3 --> and direction taken from a uniform distribution U(-179 deg, 180 deg).
 * This variations are maintained constant during each trajectory, this is, it does not change with
 * time nor position.
 */

// CLASS GEO
// =========
// =========

class ENV_API geo {
protected:
	/**< standard acceleration of free fall */
	static const double _g0_mps2;
	/**< Earth gravitational constant (atmosphere included)[m3 / s2] */
	static const double _GM_m3ps2;
	/**< Earth rotation velocity */
	static const double _w_irsecefecefiii_rps;
    /**< ellipsoid semimajor */
    static const double _a_m;
    /**< ellipsoid flattening [-] */
    static const double _f;
    /**< EGM96 2nd degree zonal harmonical */
    static const double _C20;
    /**< theoretical WGS84 gravity value at Equator */
    static const double _gcMSLE_mps2;
    /**< theoretical WGS84 gravity value at the poles */
    static const double _gcMSLP_mps2;
    /**< ellipsoid semiminor */
    double _b_m;
    /**< ellipsoid 1st eccentricity */
    double _e;
    /**< ellipsoid 2nd eccentricity */
    double _g;
    /**< squares of previous attributes */
    double _a2;
    double _b2;
    double _e2;
    double _g2;
    /**< factor equal to (omegaEarth^2 * a^2 * b) / GM */
    double _m;
    /**< factor equal to [(b * gMSLP) / (a * gMSLE) - 1] */
    double _k;

    /**< names required internally */
    static const std::string _sgeom;
    static const std::string _sgeop;
    static const std::string _slat;
    static const std::string _ssgeom;
    static const std::string _ssgeop;
    static const std::string _sslat;
    /**< table for H->h conversion (geopotential to geodetic altitudes) */
    math::f_table2V* _H2h;
    /**< table for h->H conversion (geodetic to geopotential altitudes) */
    math::f_table2V* _htoH;

    /**< pointer to magnetic model */
	env::mag* _Pmag;

    /**< standard deviation of gravity modulus deviation with respect to model */
    double _sigma_gc_mps2;
    /**< standard deviation of gravity angular deviation with respect to model */
    double _sigma_gc_deg;

    /**< gravity modulus deviation with respect to model (realization of normal distribution) */
    double _Delta_gc_mps2;
    /**< modulus of rotation vector that applies to gravity (realization of normal distribution) */
    double _Delta_gc_rad;
    /**< bearing of rotation vector that applies to gravity (realization of uniform distribution) */
    double _Delta_gc_bearing_rad;
    /**< rotation matrix encompassing effect of _Delta_gc_rad and _Delta_gc_bearing_rad. */
    ang::dcm _R_gc;

    /**< seed generator */
    std::ranlux24_base _gen;
    /**< standard normal distribution */
    std::normal_distribution<double> _dist_normal;
    /**< uniform distribution */
    std::uniform_int_distribution<> _dist_uniform;

    /**< updates all private attributes based on semimajor and flattening */
    void set_ready();

    /**< has access to private members */
	friend class atm;
public:
    /**< default constructor */
    geo() = delete;
    /**< constructor based on flight area. Model and real gravitation and magnetic fields coincide. */
    explicit geo(env::logic::MAG_ID);
    /**< constructor based on flight area, realism enumerator, and realism seed. Seed only applies to realistic
     * (not model) gravitation and magnetism. */
    geo(env::logic::MAG_ID, env::logic::REALISM_ID, const int& seed);
    /**< copy constructor */
    geo(const geo&) = delete;
    /**< move constructor */
    geo(geo&&) = delete;
    /**< destructor */
    virtual ~geo();
    /**< copy assignment */
    geo& operator=(const geo&) = delete;
    /**< move assignment */
    geo& operator=(geo&&) = delete;

    /**< ===== ===== ===== Coordinate Change ===== ===== ===== */
	/**< transforms geocentric coordinates into cartesian ones. */
	virtual env::cartesian_coord geocentric2cartesian(const env::geocentric_coord& sisu) const;
	/**< transforms cartesian coordinates into geocentric ones. */
	virtual env::geocentric_coord cartesian2geocentric(const env::cartesian_coord& cisu) const;
	/**< transforms geodetic coordinates into cartesian ones. Requires radius of curvature of meridian. */
	virtual env::cartesian_coord geodetic2cartesian(const env::geodetic_coord& gisu, const double& N_m) const;
	/**< transforms geodetic coordinates into geocentric ones. Requires radius of curvature of meridian. */
	virtual env::geocentric_coord geodetic2geocentric(const env::geodetic_coord& gisu, const double& N_m) const;
	/**< transforms geocentric plus cartesian coordinates into geodetic ones. */
	virtual env::geodetic_coord geocentriccartesian2geodetic(const env::geocentric_coord& sisu, const env::cartesian_coord& cisu) const;

    /**< ===== ===== ===== Radii of Curvature ===== ===== ===== */
	/**< returns the radius of curvature of prime vertical N [m] based on latitude */
	virtual double radius_vert(const double& phi_rad) const;
	/**< returns the radius of curvature of meridian M [m] based on latitude and radius of prime vertical  */
	virtual double radius_mer(const double& phi_rad, const double& N_m) const;

    /**< ===== ===== ===== Geodetic <--> Geopotential Altitude Conversion ===== ===== ===== */
	/**< returns geodetic altitude [m] based on geopotential altitude and latitude */
	virtual double H2h(const double& H_m, const double& phi_rad) const;
	/**< returns geopotential altitude [m] based on geodetic altitude and latitude */
	virtual double htoH(const double& h_m, const double& phi_rad) const;

    /**< ===== ===== ===== Gravitation, Centrifugal, and Gravity ===== ===== ===== */
	/**< computes the gravitation acceleration viewed in NED based on geodetic position and radius of curvature of prime vertical.
	 * Based on spherical or ellipsoidal EGM96 model, which is not orthogonal to ellipsoid. */
	virtual Eigen::Vector3d compute_gravitation_n(const env::geodetic_coord& x_gdt_rad_m, const double& N_m) const;
    /**< computes the centrifugal acceleration viewed in NED based on geodetic position and radius of curvature of prime vertical. */
	virtual Eigen::Vector3d compute_centrifugal_n(const env::geodetic_coord& x_gdt_rad_m, const double& N_m) const;
    /**< computes the MODEL gravity acceleration viewed in NED based on geodetic position.
     * Based on ellipsoidal WGS84 model, which is orthogonal to ellipsoid. */
    virtual Eigen::Vector3d compute_gravity_n_model(const env::geodetic_coord& x_gdt_rad_m) const;
    /**< computes the REAL gravity acceleration viewed in NED based on geodetic position.
    * It first calls the compute_gravity_n_model and then modifies it to add stochastic realism. */
    virtual Eigen::Vector3d compute_gravity_n_truth(const env::geodetic_coord& x_gdt_rad_m) const;
    /**< modifies the input MODEL gravity acceleration viewed in NED adding stochastic realism. */
    virtual Eigen::Vector3d compute_gravity_n_truth(const Eigen::Vector3d& gc_n_mps2_model) const;
    /**< describe gravity model and realism in stream */
    virtual void create_text(std::ostream& Ostream, const env::geodetic_coord& x_gdt_rad_m) const;

    /**< ===== ===== ===== Conversion between Ground Speed and Geodetic Coordinates Differentials ===== ===== ===== */
    /**< computes the time differential of the geodetic coordinates based on the absolute speed in NED and the radii of curvature */
    static Eigen::Vector3d vned_to_xgdtdot(const Eigen::Vector3d& v_n_mps, const env::geodetic_coord& x_gdt_rad_m, const double& N_m, const double& M_m);
    /**< computes the absolute speed in NED based o the time differential of the geodetic coordinates and the radii of curvature */
    static Eigen::Vector3d xgdtdot_to_vned(const Eigen::Vector3d& xdot_gdt_rad_m, const env::geodetic_coord& x_gdt_rad_m, const double& N_m, const double& M_m);

    /**< ===== ===== ===== Earth Angular Velocities and Inertial Accelerations ===== ===== ===== */
	/**< computes the angular speed [rps] of the ECEF(Earth centered Earth fixed)
	reference frame with respect to the IRS(Inertial Reference System) expressed in NED
	based on the latitude. */
	virtual Eigen::Vector3d compute_wien_rps(const double& phi_rad) const;
	/**< computes the angular speed [rps] of the NED(North - East - Down) frame with
	respect to the ECEF(Earth centered Earth fixed) frame expressed in NED based on
	the ground speed, the radii of curvature, the latitude, and the geometric altitude. */
	virtual Eigen::Vector3d compute_wenn_rps(const Eigen::Vector3d& v_n_mps, const double& N_m, const double& M_m, const double& phi_rad, const double& h_m) const;
	/**< computes the Coriolis acceleration [mps2] in the NED reference frame based	on the ground speed and the latitude. */
	virtual Eigen::Vector3d compute_coriolis_n(const Eigen::Vector3d& v_n_mps, const double& phi_rad) const;
    /**< computes the Coriolis acceleration [mps2] in the NED reference frame based on the ground speed and the Earth angular velocity */
    virtual Eigen::Vector3d compute_coriolis_n(const Eigen::Vector3d& v_n_mps, const Eigen::Vector3d& w_ien_rps) const;
	/**< computes the transport acceleration [mps2] in the NED reference frame based
	on the radius of curvature of prime vertical, the geodetic altitude, and the latitude. */
	virtual Eigen::Vector3d compute_transport_n(const double& N_m, const double& h_m, const double& phi_rad) const;

    /**< ===== ===== Geodesics or Great Circles ===== ===== */
    /**< returns true if input coordinates can be considered the same point, in which case the geodesic_distance method
     * should not be called as the bearings are meaningless and can lead to instabilities. */
    static bool is_same_point(const double& lambda1_rad, const double& phi1_rad, const double& lambda2_rad, const double& phi2_rad);
    /**< computes the initial bearing chi1, the distance, and the final bearing chi2 along a geodesic based on
     * the longitude and latitude of the initial and final points, based on T. Vicentry inverse algorithm.
     * Tolerance in angular difference at default sphere between both points, default taken from
     * Vicentry algorithm. Ensure two points are not the same by previously calling is_same_point. */
    virtual void geodesic_distance(double& chi1_rad_res, double& dist2__m_res, double& chi2_rad_res,
                                   const double& lambda1_rad, const double& phi1_rad, const double& lambda2_rad, const double& phi2_rad,
                                   double ltol = math::constant::GEODESIC_TOL()) const;
    /**< computes the longitude, latitude, and bearing chi a given point placed on a geodesic defined by
     * its initial longitude, latitude, and bearing, identified by its distance to the initial point, based on
     * T. Vicentry direct algorithm.  Tolerance in angular difference at default sphere between both points,
     * default taken from Vicentry algorithm. */
    virtual void geodesic_point(double& lambda_rad_result, double& phi_rad_result, double& chi_rad_result,
                                const double& lambda1_rad, const double& phi1_rad, const double& chi1_rad, const double& dist_m,
                                double Stol = math::constant::GEODESIC_TOL()) const;

    /**< ===== ===== ===== Magnetic Field ===== ===== ===== */
	/**< access to the magnetic model */
	virtual const env::mag& get_mag() const {return *_Pmag;}
}; // closes class geo

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

// CLASS GEO_MIX
// =============
// =============

class ENV_API geo_mix : public geo {
protected:
    /**< Earth radius */
    static const double _Re_m;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< default constructor */
    geo_mix() = delete;
    /**< constructor based on flight area. Model and real gravitation and magnetic fields coincide. */
    explicit geo_mix(env::logic::MAG_ID);
    /**< constructor based on flight area, realism enumerator, and realism seed. Seed only applies to realistic
     * (not model) gravitation and magnetism. */
    geo_mix(env::logic::MAG_ID, env::logic::REALISM_ID, const int& seed);
    /**< copy constructor */
    geo_mix(const geo_mix&) = delete;
    /**< move constructor */
    geo_mix(geo_mix&&) = delete;
    /**< destructor */
    virtual ~geo_mix() = default;
    /**< copy assignment */
    geo_mix& operator=(const geo_mix&) = delete;
    /**< move assignment */
    geo_mix& operator=(geo_mix&&) = delete;

    /**< returns geodetic altitude [m] based on geopotential altitude and latitude */
    virtual double H2h(const double& H_m, const double& phi_rad) const;
    /**< returns geopotential altitude [m] based on geodetic altitude and latitude */
    virtual double htoH(const double& h_m, const double& phi_rad) const;
}; // closes class geo_mix

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

// CLASS GEO_MIX11
// ===============
// ===============

class ENV_API geo_mix11 : public geo_mix {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< default constructor */
    geo_mix11() = delete;
    /**< constructor based on flight area. Model and real gravitation and magnetic fields coincide. */
    explicit geo_mix11(env::logic::MAG_ID);
    /**< constructor based on flight area, realism enumerator, and realism seed. Seed only applies to realistic
     * (not model) gravitation and magnetism. */
    geo_mix11(env::logic::MAG_ID, env::logic::REALISM_ID, const int& seed);
    /**< copy constructor */
    geo_mix11(const geo_mix11&) = delete;
    /**< move constructor */
    geo_mix11(geo_mix11&&) = delete;
    /**< destructor */
    virtual ~geo_mix11() = default;
    /**< copy assignment */
    geo_mix11& operator=(const geo_mix11&) = delete;
    /**< move assignment */
    geo_mix11& operator=(geo_mix11&&) = delete;

    /**< computes the angular speed [rps] of the ECEF(Earth centered Earth fixed)
    reference frame with respect to the IRS(Inertial Reference System) expressed in NED
    based on the latitude. */
    virtual Eigen::Vector3d compute_wien_rps(const double& phi_rad) const override;
    /**< computes the angular speed [rps] of the NED(North - East - Down) frame with
    respect to the ECEF(Earth centered Earth fixed) frame expressed in NED based on
    the ground speed, the radii of curvature, the latitude, and the geometric altitude. */
    virtual Eigen::Vector3d compute_wenn_rps(const Eigen::Vector3d& v_n_mps, const double& N_m, const double& M_m, const double& phi_rad, const double& h_m) const override;
    /**< computes the Coriolis acceleration [mps2] in the NED reference frame based	on the ground speed and the latitude. */
    virtual Eigen::Vector3d compute_coriolis_n(const Eigen::Vector3d& v_n_mps, const double& phi_rad) const override;
    /**< computes the transport acceleration [mps2] in the NED reference frame based
    on the radius of curvature of prime vertical, the geodetic altitude, and the latitude. */
    virtual Eigen::Vector3d compute_coriolis_n(const Eigen::Vector3d& v_n_mps, const Eigen::Vector3d& w_ien_rps) const override;
    /**< computes the transport acceleration [mps2] in the NED reference frame based
    on the radius of curvature of prime vertical, the geodetic altitude, and the latitude. */
    virtual Eigen::Vector3d compute_transport_n(const double& N_m, const double& h_m, const double& phi_rad) const override;
}; // closes class geo_mix11

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////


}; // closes namespace env

#endif


