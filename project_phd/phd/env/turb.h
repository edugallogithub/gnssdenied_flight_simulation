#ifndef ENV_TURB
#define ENV_TURB

#include "env.h"
#include "ang/rotate/dcm.h"
#include "math/vec/vec2.h"

/*
This file contains the turbulent wind system class "tws". The constructor is based on the seeds
employed to obtain the turbulence (integer among 1 and 25) and the maximum time for which data
is available, which can be either 1000 or 5000 [sec]. The class works as long as it is interrogated
exclusively at times that are multiples of 0.002 [sec].
*/

namespace env {

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS TURB
// ==========
// ==========

class ENV_API turb {
protected:
    /**< user defined factor with which to multiply all preloaded turbulence levels */
    double _factor;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< default constructor */
    turb() = delete;
    /**< constructor based on factor with which to multiply all preloaded turbulence levels */
    explicit turb(const double& factor);
    /**< copy constructor */
    turb(const turb&) = delete;
    /**< move constructor */
    turb(turb&&) = delete;
    /**< destructor */
    virtual ~turb() = default;
    /**< copy assignment */
    turb& operator=(const turb&) = delete;
    /**< move assignment */
    turb& operator=(turb&&) = delete;

	/**< computes the high frequency wind in BFS [mps] based on the input time, the height above the terrain, the wind direction at an altitude of 20 [ft],
	 * and the rotation matrix from NED to BFS. CAUTION: It is only	valid if the input time is a multiple of 0.002 [sec]. */
    virtual Eigen::Vector3d compute_wind_high_frequency_twomils_bfs(const double& t_sec, const double& height_m, const double& chiwind20_rad, const ang::rodrigues& q_nedbfs) const = 0;
    /**< return pointer to turbulence object based on input ID. Other four parameters [initial time, random seeds (integer among 1 and 25), final time (only two choices - 1000 or 5000),
     * and factor with which to multiply all preloaded turbulence levels] only applicable in case of turbulence */
    static env::turb* create_turb(const double& t_sec_init, const unsigned short& seed_order, const int& t_sec_end, const double& turb_factor);
}; // closes class turb

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS TURB_ZERO
// ===============
// ===============

class ENV_API turb_zero : public turb {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< default constructor */
    turb_zero();
    /**< copy constructor */
    turb_zero(const turb_zero&) = delete;
    /**< move constructor */
    turb_zero(turb_zero&&) = delete;
    /**< destructor */
    ~turb_zero() override = default;
    /**< copy assignment */
    turb_zero& operator=(const turb_zero&) = delete;
    /**< move assignment */
    turb_zero& operator=(turb_zero&&) = delete;

    /**< computes the high frequency wind in BFS [mps] based on the input time, the height above the terrain, the wind direction at an altitude of 20 [ft],
     * and the rotation matrix from NED to BFS. CAUTION: It is only valid if the input time is a multiple of 0.002 [sec]. */
    Eigen::Vector3d compute_wind_high_frequency_twomils_bfs(const double& t_sec, const double& height_m, const double& chiwind20_rad, const ang::rodrigues& q_nedbfs) const override;
}; // closes class turb_zero

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS TURB_DRYDEN
// =================
// =================

class ENV_API turb_dryden : public turb {
private:
    /**< names required internally */
    static const std::string _folder;
    static const std::string _filename_begin;
    static const std::string _filename_end;

    /**< initial time */
    double _t_sec_init;
    /**< number of time samples */
    unsigned _n;
    /**< height above terrain for boundary between low and medium altitude turbulences. */
    double _height_low_m;
    /**< height above terrain for boundary between medium and high altitude turbulences. */
    double _height_high_m;
    /**< low altitude turbulent wind every 0.002 [sec] */
    math::vec2* _wind_low_tws_fps_twomils;
    /**< high altitude turbulent wind every 0.002 [sec] */
    math::vec2* _wind_high_tws_fps_twomils;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**< default constructor */
    turb_dryden() = delete;
    /**< constructor based on initial time, random seeds (integer among 1 and 25), final time (only two choices - 1000 or 5000),
     * and factor with which to multiply all preloaded turbulence levels */
    turb_dryden(const double& t_init_sec, const unsigned short& seed_order, const int& t_sec_end, const double& factor);
    /**< copy constructor */
    turb_dryden(const turb_dryden&) = delete;
    /**< move constructor */
    turb_dryden(turb_dryden&&) = delete;
    /**< destructor */
    ~turb_dryden() override;
    /**< copy assignment */
    turb_dryden& operator=(const turb_dryden&) = delete;
    /**< move assignment */
    turb_dryden& operator=(turb_dryden&&) = delete;

    /**< computes the high frequency wind in BFS [mps] based on the input time, the height above the terrain, the wind direction at an altitude of 20 [ft],
     * and the rotation matrix from NED to BFS. CAUTION: It is only valid if the input time is a multiple of 0.002 [sec]. */
    Eigen::Vector3d compute_wind_high_frequency_twomils_bfs(const double& t_sec, const double& height_m, const double& chiwind20_rad, const ang::rodrigues& q_nedbfs) const override;
}; // closes class turb_dryden

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

}; // closes namespace env

#endif


