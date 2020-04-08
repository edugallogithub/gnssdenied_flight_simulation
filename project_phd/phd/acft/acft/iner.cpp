#include "iner.h"
#include "math/logic/share.h"
#include "env/atm.h"
#include "ang/quat.h"
#include "ang/rotate/dcm.h"
#include <boost/filesystem.hpp>

// CLASS INER
// ==========
// ==========

const std::string acft::iner::_filename = "apm/mugin_inertia_cpp.txt";
/* name required internally */

acft::iner::iner() {
    boost::filesystem::path p0(math::share::phd_configuration_prefix);
    boost::filesystem::path p1(_filename);
    std::string file_st = (p0 / p1).string();
	std::ifstream mystream(file_st.c_str()); // create stream
		
	mystream >> _full_m_kg;
	mystream >> _full_Trbb_m(0);
	mystream >> _full_Trbb_m(1);
	mystream >> _full_Trbb_m(2);

	mystream >> _full_I_kgm2(0,0);
	mystream >> _full_I_kgm2(0,1);
	mystream >> _full_I_kgm2(0,2);
	mystream >> _full_I_kgm2(1,0);
	mystream >> _full_I_kgm2(1,1);
	mystream >> _full_I_kgm2(1,2);
	mystream >> _full_I_kgm2(2,0);
	mystream >> _full_I_kgm2(2,1);
	mystream >> _full_I_kgm2(2,2);

    mystream >> _empty_m_kg;
    mystream >> _empty_Trbb_m(0);
    mystream >> _empty_Trbb_m(1);
    mystream >> _empty_Trbb_m(2);

    mystream >> _empty_I_kgm2(0,0);
    mystream >> _empty_I_kgm2(0,1);
    mystream >> _empty_I_kgm2(0,2);
    mystream >> _empty_I_kgm2(1,0);
    mystream >> _empty_I_kgm2(1,1);
    mystream >> _empty_I_kgm2(1,2);
    mystream >> _empty_I_kgm2(2,0);
    mystream >> _empty_I_kgm2(2,1);
    mystream >> _empty_I_kgm2(2,2);

	mystream.close(); // close stream

    // IMU (platform) reference point is located 30 [cm] forward and 10 [cm] below wing trailing edge
    //_Trpb_m << 0.3, 0.0, 0.1;

    // Euler angles from body to platform
    //_euler_bp.set_yaw_rad(0.5 * math::constant::D2R());
    //_euler_bp.set_pitch_rad(2.0 * math::constant::D2R());
    //_euler_bp.set_bank_rad(0.1 * math::constant::D2R());
    //_R_bp = _euler_bp;
    //_q_bp = _euler_bp;
}
/* empty constructor (based on internal text file) */

/* ===== ===== ===== Mass and Inertia ===== ===== ===== */
/* ==================================================== */

double acft::iner::get_ratio(const double& m_kg) const {
    return (_full_m_kg - m_kg) / (_full_m_kg - _empty_m_kg);
}
/* obtain mass ratio (0 means full, 1 means empty) */

double acft::iner::get_m_kg(const double& ratio) const {
    return _full_m_kg + ratio * (_empty_m_kg - _full_m_kg);
}
/* get aircraft mass based on mass ratio */

Eigen::Matrix3d acft::iner::get_I_kgm2(const double& ratio) const {
    return _full_I_kgm2 + ratio * (_empty_I_kgm2 - _full_I_kgm2);
}
/* get inertia matrix based on mass ratio */

Eigen::Matrix3d acft::iner::get_I_kgm2_inverse(const Eigen::Matrix3d& I) {
    Eigen::Matrix3d res;
    res << I(1,1)*I(2,2), 0., -I(0,2)*I(1,1), 0., I(0,0)*I(2,2)-pow(I(0,2),2), 0., -I(0,2)*I(1,1), 0., I(0,0)*I(1,1);
    return res / (I(0,0) * I(1,1) * I(2,2) - pow(I(0,2),2) * I(1,1));
}
/* get inverse of inertia matrix */

Eigen::Array3d acft::iner::get_Trbb_m(const double& ratio) const {
    return _full_Trbb_m + ratio * (_empty_Trbb_m - _full_Trbb_m);
}
/* get translation from aircraft reference (wing trailing edge) "r" to body (center of gravity) viewed in body based on mass ratio */

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

























