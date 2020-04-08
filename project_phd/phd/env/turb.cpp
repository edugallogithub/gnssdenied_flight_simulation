#include "turb.h"
#include "math/logic/share.h"
#include "math/logic/seeder.h"
#include "ang/rotate/euler.h"
#include "ang/rotate/rodrigues.h"
#include "ang/auxiliary.h"

#include <boost/filesystem.hpp>
#include <iostream>

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS TURB
// ==========
// ==========

env::turb::turb(const double& factor)
: _factor(factor) {}
/* constructor based on factor with which to multiply all preloaded turbulence levels */

env::turb* env::turb::create_turb(const double& t_sec_init, const unsigned short& seed_order, const int& t_sec_end, const double& turb_factor) {
    if (turb_factor < math::constant::EPS()) {
        return new env::turb_zero();
    }
    else {
        return new env::turb_dryden(t_sec_init, seed_order, t_sec_end, turb_factor);
    }
}
/* return pointer to turbulence object based on input ID. Other four parameters [initial time, random seeds (integer among 1 and 25), final time (only two choices - 1000 or 5000),
 * and factor with which to multiply all preloaded turbulence levels] only applicable in case of turbulence */

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS TURB_ZERO
// ===============
// ===============

env::turb_zero::turb_zero()
        : turb(0.) {}
/* default constructor */

Eigen::Vector3d env::turb_zero::compute_wind_high_frequency_twomils_bfs(const double& t_sec, const double& height_m, const double& chiwind20_rad, const ang::rodrigues& q_nedbfs) const {
    return Eigen::Vector3d(0., 0., 0.);
};
/* computes the high frequency wind in BFS [mps] based on the input time, the height above the terrain, the wind direction at an altitude of 20 [ft],
 * and the rotation matrix from NED to BFS. CAUTION: It is only valid if the input time is a multiple of 0.002 [sec]. */

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS TURB_DRYDEN
// =================
// =================

const std::string env::turb_dryden::_folder = "turbulence";
const std::string env::turb_dryden::_filename_begin = "wind_tws_";
const std::string env::turb_dryden::_filename_end = "_11_300_25.txt";
/* names required internally */

env::turb_dryden::turb_dryden(const double& t_sec_init, const unsigned short& seed_order, const int& t_sec_end, const double& factor)
: turb(factor), _t_sec_init(t_sec_init) {

	// convert seed number and final time into strings
	std::string st_t_sec_end = std::to_string(t_sec_end);
    std::string st_seeds = math::seeder::seed2string(seed_order);

	// create and open stream
    boost::filesystem::path p0(math::share::phd_inputs_prefix);
    boost::filesystem::path p1(_folder);
    boost::filesystem::path p2 = p0 / p1;
    boost::filesystem::path p3(_filename_begin + st_t_sec_end + "_seeds_" + st_seeds + _filename_end);
    std::string file_st = (p2 / p3).string();

    std::ifstream mystream(file_st.c_str()); // create stream
    // number of time samples
    mystream >> _n;
    // height above terrain for boundary between low and medium altitude turbulences.
    mystream >> _height_low_m;
    // height above terrain for boundary between medium and high altitude turbulences.
    mystream >> _height_high_m;
    // low altitude turbulent wind
    _wind_low_tws_fps_twomils = new math::vec2(3, _n, mystream);
    // high altitude turbulent wind
    _wind_high_tws_fps_twomils = new math::vec2(3, _n, mystream);

    mystream.close(); // close stream
}
/* constructor based on initial time, random seeds (integer among 1 and 25), final time (only two choices - 1000 or 5000),
 * factor with which to multiply all preloaded turbulence levels */

env::turb_dryden::~turb_dryden() {
    delete _wind_low_tws_fps_twomils;
    delete _wind_high_tws_fps_twomils;
}
/* destructor */

Eigen::Vector3d env::turb_dryden::compute_wind_high_frequency_twomils_bfs(const double& t_sec, const double& height_m, const double& chiwind20_rad, const ang::rodrigues& q_nedbfs) const {
	// obtain position (only valid if input t_sec is multiple of 0.002 [sec])
	auto pos = unsigned(rint((t_sec - _t_sec_init) / 0.002));
	if (height_m <= _height_low_m) { //  low altitude
        Eigen::Vector3d wind_low_tws_fps(_wind_low_tws_fps_twomils->get(0, pos), _wind_low_tws_fps_twomils->get(1, pos), _wind_low_tws_fps_twomils->get(2, pos));
		ang::euler euler_twsned(-chiwind20_rad, 0.0, 0.0);
        Eigen::Vector3d wind_low_ned_fps = euler_twsned / wind_low_tws_fps;
		return q_nedbfs / wind_low_ned_fps * 0.3048 * _factor;
	}
	else if(height_m >= _height_high_m) { // high altitude
        Eigen::Vector3d wind_high_tws_fps(_wind_high_tws_fps_twomils->get(0, pos), _wind_high_tws_fps_twomils->get(1, pos), _wind_high_tws_fps_twomils->get(2, pos));
		return wind_high_tws_fps * 0.3048 * _factor;
	}
	else { // medium altitudes
        Eigen::Vector3d wind_low_tws_fps(_wind_low_tws_fps_twomils->get(0, pos), _wind_low_tws_fps_twomils->get(1, pos), _wind_low_tws_fps_twomils->get(2, pos));
		ang::euler euler_twsned(-chiwind20_rad, 0.0, 0.0);
        Eigen::Vector3d wind_low_ned_fps = euler_twsned / wind_low_tws_fps;
        Eigen::Vector3d wind_low_bfs_mps = q_nedbfs / wind_low_ned_fps * 0.3048;

        Eigen::Vector3d wind_high_tws_fps(_wind_high_tws_fps_twomils->get(0, pos), _wind_high_tws_fps_twomils->get(1, pos), _wind_high_tws_fps_twomils->get(2, pos));
        Eigen::Vector3d wind_high_bfs_mps = wind_high_tws_fps * 0.3048;

		return (wind_low_bfs_mps + (wind_high_bfs_mps - wind_low_bfs_mps) * (height_m - _height_low_m) / (_height_high_m - _height_low_m)) * _factor;
	}
};
/* computes the high frequency wind in BFS [mps] based on the input time, the height above the terrain, the wind direction at an altitude of 20 [ft],
 * and the rotation matrix from NED to BFS. CAUTION: It is only valid if the input time is a multiple of 0.002 [sec]. */

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////