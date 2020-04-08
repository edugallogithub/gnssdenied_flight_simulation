#ifndef ENV_OFFSETS
#define ENV_OFFSETS

#include "env.h"
#include "math/classifiers.h"
#include <random>

/*
 * ATMOSPHERIC PRESSURE RECORDS (sea level) (1 mbar == 100 pa):
 * 1086 mbar -> highest ever recorded
 * 1030 mbar -> strong high pressure system
 * 1013 mbar -> average
 * 1000 mbar -> typical low pressure
 *  980 mbar -> CAT1 hurricane
 *  950 mbar -> CAT3 hurricane
 *  870 mbar -> lowest ever recorded 
 */


namespace env {

    class geodetic_coord;

    namespace logic {
        /**< Enumeration that contains the different offsets models */
        enum OFFSETS_ID {
            offsets_id00 = 0,
            offsets_id01 = 1,
            offsets_id02 = 2,
            offsets_id03 = 3,
            offsets_size = 4
        };
    } // closes namespace logic

// CLASS OFFSETS
// =============
// =============

class ENV_API offsets {
protected:
    /**< seed generator */
    std::ranlux24_base _gen;
    /**< standard normal distribution */
    std::normal_distribution<double> _dist;
public:
    /**< default constructor */
    offsets() = delete;
    /**< constructor based on seed */
    explicit offsets(const int& seed);
    /**< copy constructor */
    offsets(const offsets&) = delete;
    /**< move constructor */
    offsets(offsets&&) = delete;
    /**< destructor */
    virtual ~offsets() = default;
    /**< copy assignment */
    offsets& operator=(const offsets&) = delete;
    /**< move assignment */
    offsets& operator=(offsets&&) = delete;

    /**< computes the temperature differential at mean sea level */
    virtual double compute_DeltaT_degK(const double& t_sec, const double& lambda_rad, const double& phi_rad) const = 0;
    /**< computes the pressure differential at mean sea level */
    virtual double compute_Deltap_pa(const double& t_sec, const double& lambda_rad, const double& phi_rad) const = 0;
    /**< describe offsets model in stream */
    virtual void create_text(std::ostream& Ostream) const = 0;

    /**< create offsets model, flag true to show summary on console */
    static env::offsets* create_offsets(env::logic::OFFSETS_ID, const int& seed, bool flag_console = true);
}; // closes class offsets

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS OFFSETS_CONSTANT_FIXED
// ============================
// ============================

class ENV_API offsets_constant_fixed : public offsets {
private:
    /**< constant temperature differential at mean sea level */
    double _DeltaT_degK;
    /**< constant pressure differential at mean sea level */
    double _Deltap_pa;
public:
    /**< default constructor */
    offsets_constant_fixed() = delete;
    /**< constructor based on temperature and pressure differentials at mean sea level.
     * Seed not employed. Flag false to not showing summary on console.  */
    offsets_constant_fixed(const double& DeltaT_degK, const double& Deltap_pa, const int& seed = 0, bool flag_console = true);
    /**< copy constructor */
    offsets_constant_fixed(const offsets_constant_fixed&) = delete;
    /**< move constructor */
    offsets_constant_fixed(offsets_constant_fixed&&) = delete;
    /**< destructor */
    ~offsets_constant_fixed() override = default;
    /**< copy assignment */
    offsets_constant_fixed& operator=(const offsets_constant_fixed&) = delete;
    /**< move assignment */
    offsets_constant_fixed& operator=(offsets_constant_fixed&&) = delete;

    /**< computes the temperature differential at mean sea level */
    double compute_DeltaT_degK(const double& t_sec, const double& lambda_rad, const double& phi_rad) const override;
    /**< computes the pressure differential at mean sea level */
    double compute_Deltap_pa(const double& t_sec, const double& lambda_rad, const double& phi_rad) const override;
    /**< describe offsets model in stream */
    void create_text(std::ostream& Ostream) const override;
}; // closes class offsets_constant_fixed

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS OFFSETS_CONSTANT_RANDOM
// =============================
// =============================

class ENV_API offsets_constant_random : public offsets {
private:
    /**< constant temperature differential at mean sea level */
    double _DeltaT_degK;
    /**< constant pressure differential at mean sea level */
    double _Deltap_pa;
public:
    /**< default constructor */
    offsets_constant_random() = delete;
    /**< constructor based on standard deviations of temperature and pressure differentials at mean sea level.
     * Flag false to not showing summary on console.*/
    offsets_constant_random(const double& std_DeltaT_degK, const double& std_Deltap_pa, const int& seed = 0, bool flag_console = true);
    /**< copy constructor */
    offsets_constant_random(const offsets_constant_random&) = delete;
    /**< move constructor */
    offsets_constant_random(offsets_constant_random&&) = delete;
    /**< destructor */
    ~offsets_constant_random() override = default;
    /**< copy assignment */
    offsets_constant_random& operator=(const offsets_constant_random&) = delete;
    /**< move assignment */
    offsets_constant_random& operator=(offsets_constant_random&&) = delete;

    /**< computes the temperature differential at mean sea level */
    double compute_DeltaT_degK(const double& t_sec, const double& lambda_rad, const double& phi_rad) const override;
    /**< computes the pressure differential at mean sea level */
    double compute_Deltap_pa(const double& t_sec, const double& lambda_rad, const double& phi_rad) const override;
    /**< describe offsets model in stream */
    void create_text(std::ostream& Ostream) const override;
}; // closes class offsets_constant_random

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS OFFSETS_TIME
// ==================
// ==================

class ENV_API offsets_time : public offsets {
private:
    /**< function of time providing the temperature differential at mean sea level */
    math::pred1v* _PDeltaT_degK;
    /**< function of time providing the pressure differential at mean sea level */
    math::pred1v* _PDeltap_pa;
public:
    /**< default constructor */
    offsets_time() = delete;
    /**< constructor based on time functions providing the temperature and pressure differentials
     * at mean sea level. Seed not employed. Flag false to not showing summary on console. */
    offsets_time(math::pred1v* PDeltaT_degK, math::pred1v* PDeltap_pa, const int& seed = 0, bool flag_console = true);
    /**< copy constructor */
    offsets_time(const offsets_time&) = delete;
    /**< move constructor */
    offsets_time(offsets_time&&) = delete;
    /**< destructor */
    ~offsets_time() override;
    /**< copy assignment */
    offsets_time& operator=(const offsets_time&) = delete;
    /**< move assignment */
    offsets_time& operator=(offsets_time&&) = delete;

    /**< computes the temperature differential at mean sea level */
    double compute_DeltaT_degK(const double& t_sec, const double& lambda_rad, const double& phi_rad) const override;
    /**< computes the pressure differential at mean sea level */
    double compute_Deltap_pa(const double& t_sec, const double& lambda_rad, const double& phi_rad) const override;
    /**< describe offsets model in stream */
    void create_text(std::ostream& Ostream) const override;
}; // closes class offsets_time

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS OFFSETS_RAMP
// ==================
// ==================

/* Weather model that depends exclusively on time in which both the temperature and pressure
 * offsets at mean sea level begin the simulation at constant levels, then move linearly to
 * a different value, at which they remain until the end.
 * The four values that define each ramp (initial and final times, and initial and final
 * offsets) are stochastically produced.
 */

class ENV_API offsets_ramp : public offsets {
private:
    /**< time at which the temperature offset at mean sea level starts to change */
    double _t_sec_ini_T;
    /**< time at which the temperature offset at mean sea level concludes changing */
    double _t_sec_end_T;
    /**< time at which the pressure offset at mean sea level starts to change */
    double _t_sec_ini_p;
    /**< time at which the pressure offset at mean sea level concludes changing */
    double _t_sec_end_p;

    /**< initial temperature offset at mean sea level */
    double _DeltaT_degK_ini;
    /**< final temperature offset at mean sea level */
    double _DeltaT_degK_end;
    /**< initial pressure offset at mean sea level */
    double _Deltap_pa_ini;
    /**< final pressure offset at mean sea level */
    double _Deltap_pa_end;

    /**< temperature factor (internal to accelerate computations) */
    double _factorT;
    /**< pressure factor (internal to accelerate computations) */
    double _factorp;
public:
    /**< default constructor */
    offsets_ramp() = delete;
    /**< constructor based on
     * 1. mean time for beginning of ramps for temperature and pressure offsets variations.
     * 2. mean time for duration of ramps for temperature and pressure offsets variations.
     * 3. std time for beginning and end of ramps for temperature and pressure offsets variations.
     * 4. minimum time before which nothing happens.
     * 5. std (zero mean) of initial (before ramp) temperature offset.
     * 6. std (zero mean) of temperature offset variation in ramp.
     * 7. std (zero mean) of initial (before ramp) pressure offset.
     * 8. std (zero mean) of pressure offset variation in ramp.
     * 9. seed for repeatibility of all stochastical parameters.
     * 10. Flag false to not showing summary on console.
     */
    offsets_ramp(const double& t_sec_ini, const double& Jumpt_sec, const double& std_t_sec, const double& min_t_sec,
                 const double& std_DeltaT_degK_ini, const double& std_Jump_DeltaT_degK,
                 const double& std_Deltap_pa, const double& std_Jump_Deltap_pa,
                 const int& seed, bool flag_console = true);
    /**< copy constructor */
    offsets_ramp(const offsets_ramp&) = delete;
    /**< move constructor */
    offsets_ramp(offsets_ramp&&) = delete;
    /**< destructor */
    ~offsets_ramp() override = default;
    /**< copy assignment */
    offsets_ramp& operator=(const offsets_ramp&) = delete;
    /**< move assignment */
    offsets_ramp& operator=(offsets_ramp&&) = delete;

    /**< computes the temperature differential at mean sea level */
    double compute_DeltaT_degK(const double& t_sec, const double& lambda_rad, const double& phi_rad) const override;
    /**< computes the pressure differential at mean sea level */
    double compute_Deltap_pa(const double& t_sec, const double& lambda_rad, const double& phi_rad) const override;
    /**< describe offsets model in stream */
    void create_text(std::ostream& Ostream) const override;

    /**< get time at which the temperature offset at mean sea level starts to change */
    const double& get_t_sec_ini_T() const {return _t_sec_ini_T;}
    /**< get time at which the temperature offset at mean sea level concludes changing */
    const double& get_t_sec_end_T() const {return _t_sec_end_T;}
    /**< get time at which the pressure offset at mean sea level starts to change */
    const double& get_t_sec_ini_p() const {return _t_sec_ini_p;}
    /**< get time at which the pressure offset at mean sea level concludes changing */
    const double& get_t_sec_end_p() const {return _t_sec_end_p;}

    /**< get initial temperature offset at mean sea level */
    const double& get_DeltaT_degK_ini() const {return _DeltaT_degK_ini;}
    /**< get final temperature offset at mean sea level */
    const double& get_DeltaT_degK_end() const {return _DeltaT_degK_end;}
    /**< get initial pressure offset at mean sea level */
    const double& get_Deltap_pa_ini() const {return _Deltap_pa_ini;}
    /**< get final pressure offset at mean sea level */
    const double& get_Deltap_pa_end() const {return _Deltap_pa_end;}
}; // closes class offsets_ramp

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

} // closes namespace env

#endif
