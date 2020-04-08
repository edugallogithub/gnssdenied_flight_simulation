#ifndef ENV_WIND
#define ENV_WIND

#include "env.h"
#include "math/classifiers.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <random>

namespace env {

    class geodetic_coord;

    namespace logic {
        /**< Enumeration that contains the different wind models */
        enum WIND_ID {
            wind_id00 = 0,
            wind_id01 = 1,
            wind_id02 = 2,
            wind_id03 = 3,
            wind_id04 = 4,
            wind_size = 5
        };
    } // closes namespace logic

// CLASS WIND
// ==========
// ==========

class ENV_API wind {
protected:
    /**< seed generator */
    std::ranlux24_base _gen;
    /**< standard normal distribution */
    std::normal_distribution<double> _dist;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< default constructor */
    wind() = delete;
    /**< constructor based on seed */
    explicit wind(const int& seed);
    /**< copy constructor */
    wind(const wind&) = delete;
    /**< move constructor */
    wind(wind&&) = delete;
    /**< destructor */
    virtual ~wind() = default;
    /**< copy assignment */
    wind& operator=(const wind&) = delete;
    /**< move assignment */
    wind& operator=(wind&&) = delete;

    /**< computes the NED wind based on time and geodetic coordinates */
    virtual Eigen::Vector3d compute_wind_ned(const double& t_sec, const env::geodetic_coord& x_gdt_rad_m) const = 0;
    /**< describe wind model in stream */
    virtual void create_text(std::ostream& Ostream) const = 0;

    /**< create wind model, flag true to show summary on console */
    static env::wind* create_wind(env::logic::WIND_ID, const int& seed, bool flag_console = true);
}; // closes class wind

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS WIND_CONSTANT_FIXED
// =========================
// =========================

class ENV_API wind_constant_fixed : public wind {
private:
    /**< constant NED wind */
    Eigen::Vector3d _wind_ned_mps;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< default constructor */
    wind_constant_fixed() = delete;
    /**< constructor based on the North, East, and Down constant wind components.
     * Seed not employed. Flag false to not showing summary on console. */
    wind_constant_fixed(const double& wind_nedi_mps, const double& wind_nedii_mps, const double& wind_nediii_mps, const int& seed = 0, bool flag_console = true);
    /**< copy constructor */
    wind_constant_fixed(const wind_constant_fixed&) = delete;
    /**< move constructor */
    wind_constant_fixed(wind_constant_fixed&&) = delete;
    /**< destructor */
    ~wind_constant_fixed() override = default;
    /**< copy assignment */
    wind_constant_fixed& operator=(const wind_constant_fixed&) = delete;
    /**< move assignment */
    wind_constant_fixed& operator=(wind_constant_fixed&&) = delete;

    /**< computes the NED wind based on time and geodetic coordinates */
    Eigen::Vector3d compute_wind_ned(const double& t_sec, const env::geodetic_coord& x_gdt_rad_m) const override;
    /**< describe wind model in stream */
    void create_text(std::ostream& Ostream) const override;
}; // closes class wind_constant_fixed

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS WIND_CONSTANT_RANDOM
// ==========================
// ==========================

class ENV_API wind_constant_random : public wind {
private:
    /**< constant NED wind */
    Eigen::Vector3d _wind_ned_mps;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< default constructor */
    wind_constant_random() = delete;
    /**< constructor based on standard deviations of the North, East, and Down constant wind components.
     * Flag false to not showing summary on console. */
    wind_constant_random(const double& std_wind_nedi_mps, const double& std_wind_nedii_mps, const double& std_wind_nediii_mps, const int& seed = 0, bool flag_console = true);
    /**< copy constructor */
    wind_constant_random(const wind_constant_random&) = delete;
    /**< move constructor */
    wind_constant_random(wind_constant_random&&) = delete;
    /**< destructor */
    ~wind_constant_random() override = default;
    /**< copy assignment */
    wind_constant_random& operator=(const wind_constant_random&) = delete;
    /**< move assignment */
    wind_constant_random& operator=(wind_constant_random&&) = delete;

    /**< computes the NED wind based on time and geodetic coordinates */
    Eigen::Vector3d compute_wind_ned(const double& t_sec, const env::geodetic_coord& x_gdt_rad_m) const override;
    /**< describe wind model in stream */
    void create_text(std::ostream& Ostream) const override;
}; // closes class wind_constant_random

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS WIND_TIME
// ===============
// ===============

class ENV_API wind_time : public wind {
private:
    /**< function of time providing the North wind */
    math::pred1v* _Pwind_nedi_mps;
    /**< function of time providing the East wind */
    math::pred1v* _Pwind_nedii_mps;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< default constructor */
    wind_time() = delete;
    /**< constructor based on time functions providing the North and East wind components (no vertical wind).
     * Seed not employed. Flag false to not showing summary on console. */
    wind_time(math::pred1v* Pwind_nedi_mps, math::pred1v* Pwind_nedii_mps, const int& seed = 0, bool flag_console = true);
    /**< copy constructor */
    wind_time(const wind_time&) = delete;
    /**< move constructor */
    wind_time(wind_time&&) = delete;
    /**< destructor */
    ~wind_time() override;
    /**< copy assignment */
    wind_time& operator=(const wind_time&) = delete;
    /**< move assignment */
    wind_time& operator=(wind_time&&) = delete;

    /**< computes the NED wind based on time and geodetic coordinates */
    Eigen::Vector3d compute_wind_ned(const double& t_sec, const env::geodetic_coord& x_gdt_rad_m) const override;
    /**< describe wind model in stream */
    void create_text(std::ostream& Ostream) const override;
}; // closes class wind_time

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS WIND_RAMP2
// ================
// ================

/* Wind model that depends exclusively on time in which both the North and East winds
 * begin the simulation at constant levels, then move linearly to a different value,
 * at which they remain until the end.
 * The four values that define each ramp (iniial and final times, and iniial and final
 * offsets) are stochastically produced.
 */

class ENV_API wind_ramp2 : public wind {
private:
    /**< time at which the north wind starts to change */
    double _t_sec_ini_north;
    /**< time at which the north wind concludes changing */
    double _t_sec_end_north;
    /**< time at which the east wind starts to change */
    double _t_sec_ini_east;
    /**< time at which the east wind concludes changing */
    double _t_sec_end_east;

    /**< initial north wind */
    double _wind_north_mps_ini;
    /**< final north wind */
    double _wind_north_mps_end;
    /**< initial east wind */
    double _wind_east_mps_ini;
    /**< final east wind */
    double _wind_east_mps_end;

    /**< north wind factor (internal to accelerate computations) */
    double _factor_north;
    /**< east wind factor (internal to accelerate computations) */
    double _factor_east;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< default constructor */
    wind_ramp2() = delete;
    /**< constructor based on
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
    wind_ramp2(const double& t_sec_ini, const double& Jumpt_sec, const double& std_t_sec, const double& min_t_sec,
               const double& std_wind_north_mps_ini, const double& std_Jump_wind_north_mps,
               const double& std_wind_east_mps_ini, const double& std_Jump_wind_east_mps,
               const int& seed, bool flag_console = true);
    /**< copy constructor */
    wind_ramp2(const wind_ramp2&) = delete;
    /**< move constructor */
    wind_ramp2(wind_ramp2&&) = delete;
    /**< destructor */
    ~wind_ramp2() override = default;
    /**< copy assignment */
    wind_ramp2& operator=(const wind_ramp2&) = delete;
    /**< move assignment */
    wind_ramp2& operator=(wind_ramp2&&) = delete;

    /**< computes the NED wind based on time and geodetic coordinates */
    Eigen::Vector3d compute_wind_ned(const double& t_sec, const env::geodetic_coord& x_gdt_rad_m) const override;
    /**< describe wind model in stream */
    void create_text(std::ostream& Ostream) const override;

    /**< get time at which the north wind speed starts to change */
    const double& get_t_sec_ini_north() const {return _t_sec_ini_north;}
    /**< get time at which the north wind speed concludes changing */
    const double& get_t_sec_end_north() const {return _t_sec_end_north;}
    /**< get time at which the east wind speed starts to change */
    const double& get_t_sec_ini_east() const {return _t_sec_ini_east;}
    /**< get time at which the east wind speed concludes changing */
    const double& get_t_sec_end_east() const {return _t_sec_end_east;}

    /**< get initial north wind speed */
    const double& get_wind_north_mps_ini() const {return _wind_north_mps_ini;}
    /**< get final north wind speed */
    const double& get_wind_north_mps_end() const {return _wind_north_mps_end;}
    /**< get initial east wind speed */
    const double& get_wind_east_mps_ini() const {return _wind_east_mps_ini;}
    /**< get final east wind speed */
    const double& get_wind_east_mps_end() const {return _wind_east_mps_end;}
}; // closes class wind_ramp2

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS WIND_RAMP
// ===============
// ===============

/* Wind model that depends exclusively on time in which both the North and East winds
 * begin the simulation at constant levels, then move linearly to a different value,
 * at which they remain until the end.
 * The four values that define each ramp (initial and final times, and initial and final
 * offsets) are stochastically produced.
 */

class ENV_API wind_ramp : public wind {
private:
    /**< uniform integer distribution */
    std::uniform_int_distribution<int> _dist_uniform;

    /**< time at which the wind starts to change */
    double _t_sec_ini;
    /**< time at which the wind concludes changing */
    double _t_sec_end;

    /**< initial wind speed*/
    double _wind_mps_ini;
    /**< final wind speed */
    double _wind_mps_end;
    /**< initial wind bearing */
    double _chi_wind_deg_ini;
    /**< final wind bearing */
    double _chi_wind_deg_end;

    /**< initial north wind */
    double _wind_north_mps_ini;
    /**< final north wind */
    double _wind_north_mps_end;
    /**< initial east wind */
    double _wind_east_mps_ini;
    /**< final east wind */
    double _wind_east_mps_end;

    /**< north wind factor (internal to accelerate computations) */
    double _factor_north;
    /**< east wind factor (internal to accelerate computations) */
    double _factor_east;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< default constructor */
    wind_ramp() = delete;
    /**< constructor based on
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
    wind_ramp(const double& t_sec_ini, const double& Jumpt_sec, const double& std_t_sec, const double& min_t_sec,
              const double& std_wind_mps_ini, const double& std_Jump_wind_mps, const double& std_Jump_chi_wind_deg,
              const int& seed, bool flag_console = true);
    /**< copy constructor */
    wind_ramp(const wind_ramp&) = delete;
    /**< move constructor */
    wind_ramp(wind_ramp&&) = delete;
    /**< destructor */
    ~wind_ramp() override = default;
    /**< copy assignment */
    wind_ramp& operator=(const wind_ramp&) = delete;
    /**< move assignment */
    wind_ramp& operator=(wind_ramp&&) = delete;

    /**< computes the NED wind based on time and geodetic coordinates */
    Eigen::Vector3d compute_wind_ned(const double& t_sec, const env::geodetic_coord& x_gdt_rad_m) const override;
    /**< describe wind model in stream */
    void create_text(std::ostream& Ostream) const override;

    /**< get time at which the wind starts to change */
    const double& get_t_sec_ini() const {return _t_sec_ini;}
    /**< get time at which the wind concludes changing */
    const double& get_t_sec_end() const {return _t_sec_end;}

    /**< get initial wind speed */
    const double& get_wind_mps_ini() const {return _wind_mps_ini;}
    /**< get final wind speed */
    const double& get_wind_mps_end() const {return _wind_mps_end;}
    /**< get initial wind bearing */
    const double& get_chi_wind_deg_ini() const {return _chi_wind_deg_ini;}
    /**< get final wind bearing */
    const double& get_chi_wind_deg_end() const {return _chi_wind_deg_end;}

    /**< get initial north wind speed */
    const double& get_wind_north_mps_ini() const {return _wind_north_mps_ini;}
    /**< get final north wind speed */
    const double& get_wind_north_mps_end() const {return _wind_north_mps_end;}
    /**< get initial east wind speed */
    const double& get_wind_east_mps_ini() const {return _wind_east_mps_ini;}
    /**< get final east wind speed */
    const double& get_wind_east_mps_end() const {return _wind_east_mps_end;}
}; // closes class wind_ramp

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

} // closes namespace env

#endif
