#ifndef NAV_MOTION
#define NAV_MOTION

#include "../nav.h"
#include "../init/init_error.h"
#include "../nav/filter_nav.h"
#include "acft/st/sti.h"
#include "acft/guid/guid.h"
#include "acft/st/trj_sens_in.h"
#include "acft/st/trj_sens_out.h"
#include "acft/st/trj_nav_in.h"
#include "acft/st/trj_nav_out.h"
#include "acft/st/trj_gps_out.h"
#include "acft/st/trj_out.h"
#include "acft/st/trj_cntr.h"
#include "acft/st/st_extra.h"
#include "acft/control/cntr.h"
#include "acft/acft/aero.h"
#include "acft/acft/prop.h"
#include "acft/acft/iner.h"
#include "acft/sens/suite.h"
#include "acft/sens/camera.h"
#include "env/earth.h"

namespace nav {

// CLASS MOTION
// ============
// ============

class NAV_API motion {
private:
    /**< pointer to initial conditions */
    st::sti* _Psti;
    /**< pointer to guidance */
    control::guid* _Pguid;
    /**< pointer to control */
    control::cntr* _Pcntr;
    /**< pointer to navigation filter */
    nav::filter_nav* _Pnav;
    /**< pointer to Earth */
    env::earth* _Pearth;
    /**< pointer to sensor suite */
    sens::suite* _Psuite;
    /**< pointer to initial errors */
    nav::init_error* _Perr0;
    /**< pointer to aircraft inertia */
    acft::iner* _Piner;
    /**< pointer to aircraft aerodynamics */
    acft::aero* _Paero;
    /**< pointer to aircraft propulsion */
    acft::prop* _Pprop;
    /**< pointer to camera */
    sens::camera* _Pcam;

    /**< time separation between consecutive samples for integration */
    double _Deltat_sec_truth;
    /**< time separation between consecutive samples for sensors */
    double _Deltat_sec_sens;
    /**< time separation between consecutive samples for controls */
    double _Deltat_sec_cntr;
    /**< time separation between consecutive samples for output */
    double _Deltat_sec_out;
    /**< time separation between consecutive samples for gps outputs */
    double _Deltat_sec_gps;

    /**< pointer to truth aircraft trajectory */
    st::trj_truth* _Ptr_truth;
    /**< pointer to input sensor aircraft trajectory */
    st::trj_sens_in* _Ptr_sens_in;
    /**< pointer to output sensor aircraft trajectory */
    st::trj_sens_out* _Ptr_sens_out;
    /**< pointer to output gps aircraft trajectory */
    st::trj_gps_out* _Ptr_gps_out;
    /**< pointer to input navigation aircraft trajectory */
    st::trj_nav_in* _Ptr_nav_in;
    /**< pointer to output navigation aircraft trajectory */
    st::trj_nav_out* _Ptr_nav_out;
    /**< pointer to control aircraft trajectory */
    st::trj_cntr* _Ptr_cntr;
    /**< pointer to output aircraft trajectory */
    st::trj_out* _Ptr_out;

    /**< number of truth state vector */
    unsigned int _nel_truth;
    /**< number of operations */
    unsigned short _nel_op;
    /**< current operation */
    unsigned short _op;
    /**< true if GPS receiver has provided a new measurement, false otherwise */
    bool _flag_gps;
    /**< flag indicating GPS signal has already been lost */
    bool _flag_gps_lost;
    /**< enumeration that contains the different accelerometer models */
    sens::logic::SENS_COMPLETE_ID _sens_complete_id;

    /**< create trajectory objects of appropriate size based on final time and time at which gps signal is lost. */
    void create_trj_objects(const double& t_sec_end, const double& t_sec_gpsloss);
    /**< compute_sensors_input */
    void compute_sensors_input(st::st_sens_in& Ost_sens_in, st::st_nav_in& Ost_nav_in, const st::st_truth& Ost_truth) const;
    /**< compute sensors output */
    void compute_sensors_output(st::st_sens_out& Ost_sens_out, const st::st_sens_in& Ost_sens_in, const st::st_sens_out& Ost_sens_out_prev, const st::st_sens_in& Ost_sens_in_prev) const;
    /**< compute gps output */
    void compute_gps_output(st::st_gps_out& Ost_gps_out, const st::st_sens_in& Ost_sens_in) const;
    /**< initialization of navigation filter */
    void initialization_nav_filter(st::st_nav_out& Ost_nav_out, st::st_nav_in& Ost_nav_in, const st::st_sens_out& Ost_sens_out, const st::st_gps_out& Ost_gps_out);
    /**< 1st step of 2nd order improved Euler (Heun's) method, filling up intermediate state vector values */
    void solve_2nd_1st_step(st::st_truth& Tst_truth, const st::st_truth& Ost_truth_prev, const st::st_diff& Tdst_dt) const;
    /**< 2nd step of 2nd order improved Euler (Heun's) method, filling up next state vector values */
    void solve_2nd_2nd_step(st::st_truth& Ost_truth_next, const st::st_truth& Ost_truth_prev, const st::st_diff& Tdst_dt, const st::st_diff& Rdst_dt) const;
    /**< execute control step */
    void compute_control(st::st_cntr& Ost_cntr, const st::st_nav_out& Ost_nav_out, const st::st_cntr& Ost_cntr_prev) const;
    /**< compute differentials of equation of motion state vector */
    void compute_differentials(st::st_diff& Odst_dt, st::st_out& Ost_out, st::st_extra& Ost_extra, const st::st_truth& Ost_truth) const;
    /**< complete parts of the output state vector from within the main loop */
    void complete_output(st::st_out& Ost_out, const st::st_truth& Ost_truth, const st::st_sens_in& Ost_sens_in, const st::st_nav_in& Ost_nav_in, const st::st_nav_out& Ost_nav_out) const;
    /**< reset control primary loops set point ramp low pass filters */
    void reset_control_primary_loops(const unsigned int& c) const;
    /**< reset control secondary loops at beginning of new operation */
    void reset_control_secondary_loops(const unsigned int& c) const;
    /**< complete the last member of the output state vector once the main loop has finished */
    void complete_output_after(st::st_out& Ost_out, const st::st_truth& Ost_truth, const st::st_sens_in& Ost_sens_in, const st::st_nav_in& Ost_nav_in, const st::st_nav_out& Ost_nav_out);
    /**< resize trajectory vectors as integration concludes because of lack of new operations */
    void resize_st(const unsigned int& t, const unsigned int& c, const unsigned int& s, const unsigned int& o, const unsigned int& g);
    /**< set location of beginning of new operation in all trajectory vectors */
    void set_op_start(const unsigned int& t, const unsigned int& c, const unsigned int& s, const unsigned int& o, const unsigned int& g);
    /**< resize number of operations in all trajectory vectors if some of them have not been executed */
    void resize_op();
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< default constructor */
    motion() = delete;
    /**< constructor based on the different components of the trajectory */
    motion(st::sti* Psti,
           control::guid* Pguid,
           control::cntr* Pcntr,
           nav::filter_nav* Pnav,
           env::earth* Pearth,
           sens::suite* Psuite,
           nav::init_error* Perr0,
           acft::iner* Piner,
           acft::aero* Paero,
           acft::prop* Pprop,
           sens::camera* Pcam,
           sens::logic::SENS_COMPLETE_ID sens_complete_id,
           const double& Deltat_sec_truth = 0.002, const double& Deltat_sec_sens = 0.01, const double& Deltat_sec_gps = 1.0, const double& Deltat_sec_cntr = 0.02, const double& Deltat_sec_out = 0.1);
    /**< copy constructor */
    motion(const motion&) = delete;
    /**< move constructor */
    motion(motion&&) = delete;
    /**< destructor */
    ~motion();
    /**< copy assignment */
    motion& operator=(const motion&) = delete;
    /**< move assignment */
    motion& operator=(motion&&) = delete;

    /**< integrates the trajectory employing a 2nd order improved Euler (Heun's) method */
    void solve_2nd(const double& t_sec_end, const double& t_sec_gpsloss);
    /**< integrates the trajectory employing a 4th order Runge-Kutta method */
    void solve_4th_perfect(const double& t_sec_end);

    /**< get truth aircraft trajectory */
    st::trj_truth& get_trj_truth() {return *_Ptr_truth;}
    const st::trj_truth& get_trj_truth() const {return *_Ptr_truth;}
    /**< get sensor input aircraft trajectory */
    st::trj_sens_in& get_trj_sens_in() {return *_Ptr_sens_in;}
    const st::trj_sens_in& get_trj_sens_in() const {return *_Ptr_sens_in;}
    /**< get sensor output aircraft trajectory */
    st::trj_sens_out& get_trj_sens_out() {return *_Ptr_sens_out;}
    const st::trj_sens_out& get_trj_sens_out() const {return *_Ptr_sens_out;}
    /**< get input navigation aircraft trajectory */
    st::trj_nav_in& get_trj_nav_in() {return *_Ptr_nav_in;}
    const st::trj_nav_in& get_trj_nav_in() const {return *_Ptr_nav_in;}
    /**< get output navigation aircraft trajectory */
    st::trj_nav_out& get_trj_nav_out() {return *_Ptr_nav_out;}
    const st::trj_nav_out& get_trj_nav_out() const {return *_Ptr_nav_out;}
    /**< get gps output aircraft trajectory */
    st::trj_gps_out& get_trj_gps_out() {return *_Ptr_gps_out;}
    const st::trj_gps_out& get_trj_gps_out() const {return *_Ptr_gps_out;}
    /**< get control aircraft trajectory */
    st::trj_cntr& get_trj_cntr() {return *_Ptr_cntr;}
    const st::trj_cntr& get_trj_cntr() const {return *_Ptr_cntr;}
    /**< get output aircraft trajectory */
    st::trj_out& get_trj_out() {return *_Ptr_out;}
    const st::trj_out& get_trj_out() const {return *_Ptr_out;}
    /**< get number of operations */
    unsigned short& get_nel_op() {return _nel_op;}
    const unsigned short& get_nel_op() const {return _nel_op;}

    /**< get reference to initial conditions */
    const st::sti& get_sti() const {return *_Psti;}
    /**< get reference to Earth */
    const env::earth& get_earth() const {return *_Pearth;}
    /**< get reference to geophysics */
    const env::geo& get_geo() const {return _Pearth->get_geo();}
    /**< get reference to offsets */
    const env::offsets& get_offsets() const {return _Pearth->get_offsets();}
    /**< get reference to wind */
    const env::wind& get_wind() const {return _Pearth->get_wind();}
    /**< get reference to navigation filter */
    const nav::filter_nav& get_filter_nav() const {return *_Pnav;}
    /**< get reference to guidance */
    const control::guid& get_guid() const {return *_Pguid;}
    /**< get reference to initial errors */
    const nav::init_error& get_err0() const {return *_Perr0;}
    /**< get reference to sensor suite */
    const sens::suite& get_suite() const {return *_Psuite;}
    /**< get reference to camera */
    const sens::camera& get_camera() const {return *_Pcam;}
}; // closes class motion

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

} // closes namespace nav

#endif
