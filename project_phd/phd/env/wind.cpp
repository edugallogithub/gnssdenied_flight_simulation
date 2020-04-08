#include "wind.h"
#include "ang/tools.h"

// CLASS WIND
// ==========
// ==========

env::wind::wind(const int& seed)
: _gen(seed), _dist(0.,1.) {
}
/* constructor based on seed */

env::wind* env::wind::create_wind(env::logic::WIND_ID wind_id, const int& seed, bool flag_console) {
    env::wind* Pres = nullptr;
    switch(wind_id) {
        case env::logic::wind_id00:
            Pres = new env::wind_constant_fixed(0., 0., 0., seed, flag_console);
            break;
        case env::logic::wind_id01:
            Pres = new env::wind_constant_random(5., 5., 0., seed, flag_console);
            break;
        case env::logic::wind_id02:
            Pres = new env::wind_ramp2(400.0, 1200.0, 600.0, 50.0, 5.0, 2.0, 5.0, 2.0, seed, flag_console);
            break;
        case env::logic::wind_id03:
            Pres = new env::wind_ramp(400.0, 1200.0, 600.0, 50.0, 7.0, 3.0, 15.0, seed, flag_console);
            break;
        case env::logic::wind_id04:
            Pres = new env::wind_ramp(400.0, 1200.0, 600.0, 50.0, 7.0, 0.0, 0.0, seed, flag_console);
            break;
        case env::logic::wind_size:
            throw std::runtime_error("Wind model not available");
        default:
            throw std::runtime_error("Wind model not available");
    }
    return Pres;
}
/* create wind model, flag true to show summary on console. */

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS WIND_CONSTANT_FIXED
// =========================
// =========================

env::wind_constant_fixed::wind_constant_fixed(const double& wind_nedi_mps, const double& wind_nedii_mps, const double& wind_nediii_mps, const int& seed, bool flag_console)
: wind(seed), _wind_ned_mps(wind_nedi_mps, wind_nedii_mps, wind_nediii_mps) {
    if (flag_console == true) {
        this->create_text(std::cout);
    }
}
/* constructor based on the North, East, and Down constant wind components. Seed not employed. Flag false to not showing summary on console. */

Eigen::Vector3d env::wind_constant_fixed::compute_wind_ned(const double& t_sec, const env::geodetic_coord& x_gdt_rad_m) const {
    return _wind_ned_mps;
}
/* computes the NED wind based on time and geodetic coordinates */

void env::wind_constant_fixed::create_text(std::ostream& Ostream) const {
    Ostream << std::endl << "CONSTANT FIXED WIND:" << std::endl << std::endl;
    Ostream << "North wind [mps]:  " << std::fixed << std::setw(6)  << std::setprecision(2) << std::showpos << _wind_ned_mps(0) << std::endl;
    Ostream << "East  wind [mps]:  " << std::fixed << std::setw(6)  << std::setprecision(2) << std::showpos << _wind_ned_mps(1) << std::endl;
    Ostream << "Down  wind [mps]:  " << std::fixed << std::setw(6)  << std::setprecision(2) << std::showpos << _wind_ned_mps(2) << std::endl;
}
/* describe wind model in stream */

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS WIND_CONSTANT_RANDOM
// ==========================
// ==========================

env::wind_constant_random::wind_constant_random(const double& std_wind_nedi_mps, const double& std_wind_nedii_mps, const double& std_wind_nediii_mps, const int& seed, bool flag_console)
: wind(seed) {
    _wind_ned_mps(0) = std_wind_nedi_mps * _dist(_gen);
    _wind_ned_mps(1) = std_wind_nedii_mps * _dist(_gen);
    _wind_ned_mps(2) = std_wind_nediii_mps * _dist(_gen);

    if (flag_console == true) {
        this->create_text(std::cout);
    }
}
/* constructor based on the standard deviations of the North, East, and Down constant wind components. Flag false to not showing summary on console. */

Eigen::Vector3d env::wind_constant_random::compute_wind_ned(const double& t_sec, const env::geodetic_coord& x_gdt_rad_m) const {
    return _wind_ned_mps;
}
/* computes the NED wind based on time and geodetic coordinates */

void env::wind_constant_random::create_text(std::ostream& Ostream) const {
    Ostream << std::endl << "CONSTANT RANDOM WIND:" << std::endl << std::endl;
    Ostream << "North wind [mps]:  " << std::fixed << std::setw(6)  << std::setprecision(2) << std::showpos << _wind_ned_mps(0) << std::endl;
    Ostream << "East  wind [mps]:  " << std::fixed << std::setw(6)  << std::setprecision(2) << std::showpos << _wind_ned_mps(1) << std::endl;
    Ostream << "Down  wind [mps]:  " << std::fixed << std::setw(6)  << std::setprecision(2) << std::showpos << _wind_ned_mps(2) << std::endl;
}
/* describe wind model in stream */

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS WIND_TIME
// ===============
// ===============

env::wind_time::wind_time(math::pred1v* Pwind_nedi_mps, math::pred1v* Pwind_nedii_mps, const int& seed, bool flag_console)
: wind(seed), _Pwind_nedi_mps(Pwind_nedi_mps), _Pwind_nedii_mps(Pwind_nedii_mps) {
    if (flag_console == true) {
        this->create_text(std::cout);
    }
}
/* constructor based on time functions providing the North and East wind components (no vertical wind).
 * Seed not employed. Flag false to not showing summary on console. */

env::wind_time::~wind_time() {
    delete _Pwind_nedi_mps;
    delete _Pwind_nedii_mps;
}
/* destructor */

Eigen::Vector3d env::wind_time::compute_wind_ned(const double& t_sec, const env::geodetic_coord& x_gdt_rad_m) const {
    return Eigen::Vector3d{_Pwind_nedi_mps->value(t_sec), _Pwind_nedii_mps->value(t_sec), 0.};
}
/* computes the NED wind based on time and geodetic coordinates */

void env::wind_time::create_text(std::ostream& Ostream) const {
    Ostream << std::endl << "TIME WIND:" << std::endl << std::endl;
}
/* describe wind model in stream */

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS WIND_RAMP2
// ================
// ================

env::wind_ramp2::wind_ramp2(const double& t_sec_ini, const double& Jumpt_sec, const double& std_t_sec, const double& min_t_sec,
                            const double& std_wind_north_mps_ini, const double& std_Jump_wind_north_mps,
                            const double& std_wind_east_mps_ini, const double& std_Jump_wind_east_mps,
                            const int& seed, bool flag_console)
: wind(seed) {
    do {
        _t_sec_ini_north = t_sec_ini + std_t_sec * _dist(_gen);
    } while (_t_sec_ini_north < min_t_sec);
    do {
        _t_sec_ini_east = t_sec_ini + std_t_sec * _dist(_gen);
    } while (_t_sec_ini_east < min_t_sec);
    do {
        _t_sec_end_north  = _t_sec_ini_north + Jumpt_sec + std_t_sec * _dist(_gen);
    } while (_t_sec_end_north < min_t_sec);
    do {
        _t_sec_end_east  = _t_sec_ini_east + Jumpt_sec + std_t_sec * _dist(_gen);
    } while (_t_sec_end_east < min_t_sec);

    if (_t_sec_end_north < _t_sec_ini_north) {
        double temp = _t_sec_end_north;
        _t_sec_end_north  = _t_sec_ini_north + std_t_sec;
        _t_sec_ini_north = temp;
    }
    if (_t_sec_end_east < _t_sec_ini_east) {
        double temp = _t_sec_end_east;
        _t_sec_end_east  = _t_sec_ini_east + std_t_sec;
        _t_sec_ini_east = temp;
    }

    _wind_north_mps_ini = std_wind_north_mps_ini * _dist(_gen);
    _wind_east_mps_ini  = std_wind_east_mps_ini  * _dist(_gen);
    _wind_north_mps_end  = _wind_north_mps_ini + std_Jump_wind_north_mps * _dist(_gen);
    _wind_east_mps_end   = _wind_east_mps_ini  + std_Jump_wind_east_mps  * _dist(_gen);

    _factor_north = (_wind_north_mps_end - _wind_north_mps_ini) / (_t_sec_end_north - _t_sec_ini_north);
    _factor_east  = (_wind_east_mps_end  - _wind_east_mps_ini)  / (_t_sec_end_east  - _t_sec_ini_east);

    if (flag_console == true) {
        this->create_text(std::cout);
    }
}
/* constructor based on
 * 1. mean time for beginning of ramps for north and east wind speed variations.
 * 2. mean time for duration of ramps for north and east wind speed variations.
 * 3. std time for beginning and end of ramps for north and east winds variations.
 * 4. minimum time before which nothing happens.
 * 5. std (zero mean) of initial (before ramp) north wind speed.
 * 6. std (zero mean) of north wind speed variation in ramp.
 * 7. std (zero mean) of initial (before ramp) east wind speed.
 * 8. std (zero mean) of east wind speed variation in ramp.
 * 9. seed for repeatibility of all stochastical parameters.
 * 10. Flag false to not showing summary on console.
 */

Eigen::Vector3d env::wind_ramp2::compute_wind_ned(const double& t_sec, const env::geodetic_coord& x_gdt_rad_m) const {
    double wind_north_mps, wind_east_mps;

    if (t_sec <= _t_sec_ini_north) {
        wind_north_mps = _wind_north_mps_ini;
    }
    else if (t_sec <= _t_sec_end_north) {
        wind_north_mps = _wind_north_mps_ini + _factor_north * (t_sec - _t_sec_ini_north);
    }
    else {
        wind_north_mps = _wind_north_mps_end;
    }

    if (t_sec <= _t_sec_ini_east) {
        wind_east_mps = _wind_east_mps_ini;
    }
    else if (t_sec <= _t_sec_end_east) {
        wind_east_mps = _wind_east_mps_ini + _factor_east * (t_sec - _t_sec_ini_east);
    }
    else {
        wind_east_mps = _wind_east_mps_end;
    }

    return Eigen::Vector3d{wind_north_mps, wind_east_mps, 0.};
}
/* computes the NED wind based on time and geodetic coordinates */

void env::wind_ramp2::create_text(std::ostream& Ostream) const {
    Ostream << std::endl << "RAMP WIND:" << std::endl << std::endl;
    Ostream << "North ramp ini - end [sec]: "
            << std::fixed << std::setw(9) << std::setprecision(2) << std::showpos << _t_sec_ini_north
            << std::fixed << std::setw(9) << std::setprecision(2) << std::showpos << _t_sec_end_north << std::endl;
    Ostream << "North wind ini - end [mps]: "
            << std::fixed << std::setw(9) << std::setprecision(2) << std::showpos << _wind_north_mps_ini
            << std::fixed << std::setw(9) << std::setprecision(2) << std::showpos << _wind_north_mps_end << std::endl;
    Ostream << "East ramp ini - end [sec]:  "
            << std::fixed << std::setw(9) << std::setprecision(2) << std::showpos << _t_sec_ini_east
            << std::fixed << std::setw(9) << std::setprecision(2) << std::showpos << _t_sec_end_east << std::endl;
    Ostream << "East wind ini - end [mps]:  "
            << std::fixed << std::setw(9) << std::setprecision(2) << std::showpos << _wind_east_mps_ini
            << std::fixed << std::setw(9) << std::setprecision(2) << std::showpos << _wind_east_mps_end << std::endl;
}
/* describe wind model in stream */

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS WIND_RAMP
// ===============
// ===============

env::wind_ramp::wind_ramp(const double& t_sec_ini, const double& Jumpt_sec, const double& std_t_sec, const double& min_t_sec,
                          const double& std_wind_mps_ini, const double& std_Jump_wind_mps, const double& std_Jump_chi_wind_deg,
                          const int& seed, bool flag_console)
: wind(seed), _dist_uniform(-179, 180) {
    do {
        _t_sec_ini = t_sec_ini + std_t_sec * _dist(_gen);
    } while (_t_sec_ini < min_t_sec);
    do {
        _t_sec_end  = _t_sec_ini + Jumpt_sec + std_t_sec * _dist(_gen);
    } while (_t_sec_end < (_t_sec_ini + std_t_sec / 2.));

    _wind_mps_ini = std_wind_mps_ini * _dist(_gen);
    _chi_wind_deg_ini = _dist_uniform(_gen);
    if (_wind_mps_ini < 0.0) {
        _wind_mps_ini = - _wind_mps_ini;
    }

    _wind_mps_end  = _wind_mps_ini + std_Jump_wind_mps * _dist(_gen);
    _chi_wind_deg_end = _chi_wind_deg_ini + std_Jump_chi_wind_deg * _dist(_gen);
    if (_wind_mps_end < 0.0) {
        _wind_mps_end = - _wind_mps_end;
        _chi_wind_deg_end = _chi_wind_deg_end + 180.0;
    }
    ang::tools::correct_yaw_deg(_chi_wind_deg_end);

    double chi_wind_rad_ini = _chi_wind_deg_ini * math::constant::D2R();
    double chi_wind_rad_end = _chi_wind_deg_end * math::constant::D2R();

    _wind_north_mps_ini  = _wind_mps_ini * cos(chi_wind_rad_ini);
    _wind_east_mps_ini   = _wind_mps_ini * sin(chi_wind_rad_ini);

    _wind_north_mps_end  = _wind_mps_end * cos(chi_wind_rad_end);
    _wind_east_mps_end   = _wind_mps_end * sin(chi_wind_rad_end);

    _factor_north = (_wind_north_mps_end - _wind_north_mps_ini) / (_t_sec_end - _t_sec_ini);
    _factor_east  = (_wind_east_mps_end  - _wind_east_mps_ini)  / (_t_sec_end  - _t_sec_ini);

    if (flag_console == true) {
        this->create_text(std::cout);
    }
}
/* constructor based on
 * 1. mean time for beginning of ramp for wind variation.
 * 2. mean time for duration of ramp for wind variation.
 * 3. std time for beginning and end of ramp for wind variation.
 * 4. minimum time before which nothing happens.
 * 5. std (zero mean) of initial (before ramp) wind speed.
 * 6. std (zero mean) of wind speed variation in ramp.
 * 7. std (zero mean) of wind angle variation in ramp.
 * 8. seed for repeatability of all stochastical parameters.
 * 9. Flag false to not showing summary on console.
 */

Eigen::Vector3d env::wind_ramp::compute_wind_ned(const double& t_sec, const env::geodetic_coord& x_gdt_rad_m) const {
    double wind_north_mps, wind_east_mps;

    if (t_sec <= _t_sec_ini) {
        wind_north_mps = _wind_north_mps_ini;
    }
    else if (t_sec <= _t_sec_end) {
        wind_north_mps = _wind_north_mps_ini + _factor_north * (t_sec - _t_sec_ini);
    }
    else {
        wind_north_mps = _wind_north_mps_end;
    }

    if (t_sec <= _t_sec_ini) {
        wind_east_mps = _wind_east_mps_ini;
    }
    else if (t_sec <= _t_sec_end) {
        wind_east_mps = _wind_east_mps_ini + _factor_east * (t_sec - _t_sec_ini);
    }
    else {
        wind_east_mps = _wind_east_mps_end;
    }

    return Eigen::Vector3d{wind_north_mps, wind_east_mps, 0.};
}
/* computes the NED wind based on time and geodetic coordinates */

void env::wind_ramp::create_text(std::ostream& Ostream) const {
    Ostream << std::endl << "RAMP WIND:" << std::endl << std::endl;
    Ostream << "Wind ramp ini - end [sec]:    "
            << std::fixed << std::setw(9) << std::setprecision(2) << std::showpos << _t_sec_ini
            << std::fixed << std::setw(9) << std::setprecision(2) << std::showpos << _t_sec_end << std::endl;
    Ostream << "Wind speed ini - end [mps]:   "
            << std::fixed << std::setw(9) << std::setprecision(2) << std::showpos << _wind_mps_ini
            << std::fixed << std::setw(9) << std::setprecision(2) << std::showpos << _wind_mps_end << std::endl;
    Ostream << "Wind bearing ini - end [deg]: "
            << std::fixed << std::setw(9) << std::setprecision(2) << std::showpos << _chi_wind_deg_ini
            << std::fixed << std::setw(9) << std::setprecision(2) << std::showpos << _chi_wind_deg_end << std::endl;
    Ostream << "North wind ini - end [mps]:   "
            << std::fixed << std::setw(9) << std::setprecision(2) << std::showpos << _wind_north_mps_ini
            << std::fixed << std::setw(9) << std::setprecision(2) << std::showpos << _wind_north_mps_end << std::endl;
    Ostream << "East wind ini - end [mps]:    "
            << std::fixed << std::setw(9) << std::setprecision(2) << std::showpos << _wind_east_mps_ini
            << std::fixed << std::setw(9) << std::setprecision(2) << std::showpos << _wind_east_mps_end << std::endl;
}
/* describe wind model in stream */

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////




