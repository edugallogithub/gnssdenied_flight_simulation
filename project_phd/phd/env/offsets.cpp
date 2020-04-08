#include "offsets.h"

// CLASS OFFSETS
// =============
// =============

env::offsets::offsets(const int& seed)
: _gen(seed), _dist(0.,1.) {
}
/* constructor based on seed */

env::offsets* env::offsets::create_offsets(env::logic::OFFSETS_ID offsets_id, const int& seed, bool flag_console) {
    env::offsets* Pres = nullptr;
    switch(offsets_id) {
        case env::logic::offsets_id00:
            Pres = new env::offsets_constant_fixed(0., 0., seed, flag_console);
            break;
        case env::logic::offsets_id01:
            Pres = new env::offsets_constant_random(10., 100., seed, flag_console);
            break;
        case env::logic::offsets_id02:
            Pres = new env::offsets_ramp(400.0, 1200.0, 600.0, 50.0, 10.0, 3.0, 1500.0, 300.0, seed, flag_console);
            break;
        case env::logic::offsets_id03:
            Pres = new env::offsets_ramp(400.0, 1200.0, 600.0, 50.0, 10.0, 0., 1500.0, 0., seed, flag_console);
            break;
        case env::logic::offsets_size:
            throw std::runtime_error("Offsets model not available");
        default:
            throw std::runtime_error("Offsets model not available");
    }
    return Pres;
}
/* create offsets model, flag true to show summary on console */

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS OFFSETS_CONSTANT_FIXED
// ============================
// ============================

env::offsets_constant_fixed::offsets_constant_fixed(const double& DeltaT_degK, const double& Deltap_pa, const int& seed, bool flag_console)
: offsets(seed), _DeltaT_degK(DeltaT_degK), _Deltap_pa(Deltap_pa) {
    if (flag_console == true) {
        this->create_text(std::cout);
    }
}
/* constructor based on temperature and pressure differentials at mean sea level. Seed not employed. Flag false to not showing summary on console. */

double env::offsets_constant_fixed::compute_DeltaT_degK(const double& t_sec, const double& lambda_rad, const double& phi_rad) const {
    return _DeltaT_degK;
}
/* computes the temperature differential at mean sea level */

double env::offsets_constant_fixed::compute_Deltap_pa(const double& t_sec, const double& lambda_rad, const double& phi_rad) const {
    return _Deltap_pa;
}
/* computes the pressure differential at mean sea level */

void env::offsets_constant_fixed::create_text(std::ostream& Ostream) const {
    Ostream << std::endl << "CONSTANT FIXED OFFSETS:" << std::endl << std::endl;
    Ostream << "DeltaT [degK]: " << std::fixed << std::setw(8)  << std::setprecision(2) << std::showpos << _DeltaT_degK << std::endl;
    Ostream << "Deltap [pa]:   " << std::fixed << std::setw(8)  << std::setprecision(2) << std::showpos << _Deltap_pa << std::endl;
}
/* describe offsets model in stream */

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS OFFSETS_CONSTANT_RANDOM
// =============================
// =============================

env::offsets_constant_random::offsets_constant_random(const double& std_DeltaT_degK, const double& std_Deltap_pa, const int& seed, bool flag_console)
: offsets(seed) {
    _DeltaT_degK = std_DeltaT_degK * _dist(_gen);
    _Deltap_pa   = std_Deltap_pa   * _dist(_gen);
    if (flag_console == true) {
        this->create_text(std::cout);
    }
}
/* constructor based on standard deviations of temperature and pressure differentials at mean sea level. Flag false to not showing summary on console.*/

double env::offsets_constant_random::compute_DeltaT_degK(const double& t_sec, const double& lambda_rad, const double& phi_rad) const {
    return _DeltaT_degK;
}
/* computes the temperature differential at mean sea level */

double env::offsets_constant_random::compute_Deltap_pa(const double& t_sec, const double& lambda_rad, const double& phi_rad) const {
    return _Deltap_pa;
}
/* computes the pressure differential at mean sea level */

void env::offsets_constant_random::create_text(std::ostream& Ostream) const {
    Ostream << std::endl << "CONSTANT RANDOM OFFSETS:" << std::endl << std::endl;
    Ostream << "DeltaT [degK]: " << std::fixed << std::setw(8)  << std::setprecision(2) << std::showpos << _DeltaT_degK << std::endl;
    Ostream << "Deltap [pa]:   " << std::fixed << std::setw(8)  << std::setprecision(2) << std::showpos << _Deltap_pa << std::endl;
}
/* describe offsets model in stream */

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS OFFSETS_TIME
// ==================
// ==================

env::offsets_time::offsets_time(math::pred1v* PDeltaT_degK, math::pred1v* PDeltap_pa, const int& seed, bool flag_console)
: offsets(seed), _PDeltaT_degK(PDeltaT_degK), _PDeltap_pa(PDeltap_pa) {
    if (flag_console == true) {
        this->create_text(std::cout);
    }
}
/* constructor based on time functions providing the temperature and pressure differentials
 * at mean sea level. Seed not employed. Flag false to not showing summary on console. */

env::offsets_time::~offsets_time() {
    delete _PDeltaT_degK;
    delete _PDeltap_pa;
}
/* destructor */

double env::offsets_time::compute_DeltaT_degK(const double& t_sec, const double& lambda_rad, const double& phi_rad) const {
    return _PDeltaT_degK->value(t_sec);
}
/* computes the temperature differential at mean sea level */

double env::offsets_time::compute_Deltap_pa(const double& t_sec, const double& lambda_rad, const double& phi_rad) const {
    return _PDeltap_pa->value(t_sec);
}
/* computes the pressure differential at mean sea level */

void env::offsets_time::create_text(std::ostream& Ostream) const {
    Ostream << std::endl << "TIME OFFSETS:" << std::endl << std::endl;
}
/* describe offsets model in stream */

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS OFFSETS_RAMP
// ==================
// ==================

env::offsets_ramp::offsets_ramp(const double& t_sec_ini, const double& Jumpt_sec, const double& std_t_sec, const double& min_t_sec,
                                const double& std_DeltaT_degK_ini, const double& std_Jump_DeltaT_degK,
                                const double& std_Deltap_pa_ini, const double& std_Jump_Deltap_pa,
                                const int& seed, bool flag_console)
: offsets(seed) {
    do {
        _t_sec_ini_T = t_sec_ini + std_t_sec * _dist(_gen);
    } while (_t_sec_ini_T < min_t_sec);
    do {
        _t_sec_ini_p = t_sec_ini + std_t_sec * _dist(_gen);
    } while (_t_sec_ini_p < min_t_sec);
    do {
        _t_sec_end_T  = _t_sec_ini_T + Jumpt_sec + std_t_sec * _dist(_gen);
    } while (_t_sec_end_T < (_t_sec_ini_T + std_t_sec));
    do {
        _t_sec_end_p  = _t_sec_ini_p + Jumpt_sec + std_t_sec * _dist(_gen);
    } while (_t_sec_end_p < (_t_sec_ini_p + std_t_sec));

    _DeltaT_degK_ini = std_DeltaT_degK_ini * _dist(_gen);
    _Deltap_pa_ini   = std_Deltap_pa_ini   * _dist(_gen);
    _DeltaT_degK_end  = _DeltaT_degK_ini + std_Jump_DeltaT_degK * _dist(_gen);
    _Deltap_pa_end    = _Deltap_pa_ini   + std_Jump_Deltap_pa   * _dist(_gen);

    _factorT = (_DeltaT_degK_end - _DeltaT_degK_ini) / (_t_sec_end_T - _t_sec_ini_T);
    _factorp = (_Deltap_pa_end   - _Deltap_pa_ini)   / (_t_sec_end_p - _t_sec_ini_p);

    if (flag_console == true) {
        this->create_text(std::cout);
    }
}
/* constructor based on
 * 1. mean time for beginning of ramps for temperature and pressure offsets variations.
 * 2. mean time for duration of ramps for temperature and pressure offsets variations.
 * 3. std time for beginning and end of ramps for temperature and pressure offsets variations.
 * 4. minimum time before which nothing happens.
 * 5. std (zero mean) of initial (before ramp) temperature offset.
 * 6. std (zero mean) of temperature offset variation in ramp.
 * 7. std (zero mean) of initial (before ramp) pressure offset.
 * 8. std (zero mean) of pressure offset variation in ramp.
 * 9. seed for repeatability of all stochastical parameters.
 * 10. Flag false to not showing summary on console.
 */

double env::offsets_ramp::compute_DeltaT_degK(const double& t_sec, const double& lambda_rad, const double& phi_rad) const {
    if (t_sec <= _t_sec_ini_T) {
        return _DeltaT_degK_ini;
    }
    else if (t_sec <= _t_sec_end_T) {
        return _DeltaT_degK_ini + _factorT * (t_sec - _t_sec_ini_T);
    }
    else {
        return _DeltaT_degK_end;
    }
}
/* computes the temperature differential at mean sea level */

double env::offsets_ramp::compute_Deltap_pa(const double& t_sec, const double& lambda_rad, const double& phi_rad) const {
    if (t_sec <= _t_sec_ini_p) {
        return _Deltap_pa_ini;
    }
    else if (t_sec <= _t_sec_end_p) {
        return _Deltap_pa_ini + _factorp * (t_sec - _t_sec_ini_p);
    }
    else {
        return _Deltap_pa_end;
    }
}
/* computes the pressure differential at mean sea level */

void env::offsets_ramp::create_text(std::ostream& Ostream) const {
    Ostream << std::endl << "RAMP OFFSETS:" << std::endl << std::endl;
    Ostream << "DeltaT ramp ini - end [sec]: "
            << std::fixed << std::setw(9) << std::setprecision(2) << std::showpos << _t_sec_ini_T
            << std::fixed << std::setw(9) << std::setprecision(2) << std::showpos << _t_sec_end_T << std::endl;
    Ostream << "DeltaT ini - end [degK]:     "
            << std::fixed << std::setw(9) << std::setprecision(2) << std::showpos << _DeltaT_degK_ini
            << std::fixed << std::setw(9) << std::setprecision(2) << std::showpos << _DeltaT_degK_end << std::endl;
    Ostream << "Deltap ramp ini - end [sec]: "
            << std::fixed << std::setw(9) << std::setprecision(2) << std::showpos << _t_sec_ini_p
            << std::fixed << std::setw(9) << std::setprecision(2) << std::showpos << _t_sec_end_p << std::endl;
    Ostream << "Deltap ini - end [pa]:       "
            << std::fixed << std::setw(9) << std::setprecision(2) << std::showpos << _Deltap_pa_ini
            << std::fixed << std::setw(9) << std::setprecision(2) << std::showpos << _Deltap_pa_end << std::endl;
}
/* describe offsets model in stream */

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////





