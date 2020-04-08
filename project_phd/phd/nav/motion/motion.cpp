#include <env/coord.h>
#include "motion.h"
#include "env/speed.h"

// CLASS MOTION
// ============
// ============

nav::motion::motion(st::sti* Psti, control::guid* Pguid, control::cntr* Pcntr, nav::filter_nav* Pnav, env::earth* Pearth,
                       sens::suite* Psuite, nav::init_error* Perr0, acft::iner* Piner, acft::aero* Paero, acft::prop* Pprop, sens::camera* Pcam, sens::logic::SENS_COMPLETE_ID sens_complete_id,
                       const double& Deltat_sec_truth, const double& Deltat_sec_sens, const double& Deltat_sec_gps, const double& Deltat_sec_cntr, const double& Deltat_sec_out)
: _Psti(Psti), _Pguid(Pguid), _Pcntr(Pcntr), _Pnav(Pnav), _Pearth(Pearth), _Psuite(Psuite),
  _Perr0(Perr0), _Piner(Piner), _Paero(Paero), _Pprop(Pprop), _Pcam(Pcam),
  _flag_gps(false), _flag_gps_lost(false), _Ptr_truth(nullptr), _Ptr_sens_in(nullptr), _Ptr_sens_out(nullptr), _Ptr_gps_out(nullptr),
  _Ptr_nav_in(nullptr), _Ptr_nav_out(nullptr), _Ptr_cntr(nullptr), _Ptr_out(nullptr),
  _sens_complete_id(sens_complete_id),
  _Deltat_sec_truth(Deltat_sec_truth), _Deltat_sec_sens(Deltat_sec_sens), _Deltat_sec_gps(Deltat_sec_gps), _Deltat_sec_cntr(Deltat_sec_cntr), _Deltat_sec_out(Deltat_sec_out),
  _nel_truth(0), _nel_op(_Pguid->get_nel_op()), _op(0) {

    double mod_sens_truth = fmod(Deltat_sec_sens, Deltat_sec_truth);
    double mod_cntr_sens  = fmod(Deltat_sec_cntr, Deltat_sec_sens);
    double mod_gps_sens   = fmod(Deltat_sec_gps,  Deltat_sec_sens);
    double mod_out_cntr   = fmod(Deltat_sec_out,  Deltat_sec_cntr);

    double abs_sens_truth = fabs(mod_sens_truth - Deltat_sec_truth);
    double abs_cntr_sens  = fabs(mod_cntr_sens  - Deltat_sec_sens);
    double abs_gps_sens   = fabs(mod_gps_sens   - Deltat_sec_sens);
    double abs_out_cntr   = fabs(mod_out_cntr   - Deltat_sec_cntr);

    if (mod_sens_truth > math::constant::TOL()) {if (abs_sens_truth > math::constant::TOL()) {throw std::runtime_error("Frequencies must be multiples of each other.");}}
    if (mod_cntr_sens  > math::constant::TOL()) {if (abs_cntr_sens  > math::constant::TOL()) {throw std::runtime_error("Frequencies must be multiples of each other.");}}
    if (mod_gps_sens   > math::constant::TOL()) {if (abs_gps_sens   > math::constant::TOL()) {throw std::runtime_error("Frequencies must be multiples of each other.");}}
    if (mod_out_cntr   > math::constant::TOL()) {if (abs_out_cntr   > math::constant::TOL()) {throw std::runtime_error("Frequencies must be multiples of each other.");}}
}
/* constructor based on the different components of the trajectory */

nav::motion::~motion() {
    delete _Psti;
    delete _Pguid;
    delete _Pcntr;
    delete _Pnav;
    delete _Pearth;
    delete _Psuite;
    delete _Perr0;
    delete _Piner;
    delete _Paero;
    delete _Pprop;
    delete _Pcam;

    delete _Ptr_truth;
    delete _Ptr_sens_in;
    delete _Ptr_sens_out;
    delete _Ptr_gps_out;
    delete _Ptr_nav_in;
    delete _Ptr_nav_out;
    delete _Ptr_cntr;
    delete _Ptr_out;
}
/* destructor */

void nav::motion::solve_2nd(const double& t_sec_end_input, const double& t_sec_gpsloss_input) {
    // create trajectory objects of the appropriate size
    this->create_trj_objects(t_sec_end_input, t_sec_gpsloss_input);

    // use initial conditions to fill up truth[0]
    _Psti->complete_and_fill_up_st_truth(_Ptr_truth->get()[0], *_Pearth);

    // compute first sensors inputs, filling up sens_in[0] and nav_in[0]
    this->compute_sensors_input(_Ptr_sens_in->get()[0], _Ptr_nav_in->get()[0], _Ptr_truth->get()[0]);

    // compute first sensors outputs, filling up sens_out[0] and gps_out[0]
    this->compute_sensors_output(_Ptr_sens_out->get()[0], _Ptr_sens_in->get()[0], _Ptr_sens_out->get()[0], _Ptr_sens_in->get()[0]);
    this->compute_gps_output(_Ptr_gps_out->get()[0], _Ptr_sens_in->get()[0]);

    // initialization of navigation filters, filling up nav_out[0] and one variable with nav_in[0] without executing filters
    this->initialization_nav_filter(_Ptr_nav_out->get()[0], _Ptr_nav_in->get()[0], _Ptr_sens_out->get()[0], _Ptr_gps_out->get()[0]);

    // show geophysics models on console
    _Pearth->get_geo().get_mag().create_text(std::cout, _Ptr_nav_in->get()[0].get_t_sec(), _Ptr_nav_in->get()[0].get_x_gdt_rad_m());
    _Pearth->get_geo().create_text(std::cout, _Ptr_nav_in->get()[0].get_x_gdt_rad_m());
    std::cout << std::fixed << std::setprecision(1);

    // load initial trigger target so it works later
    _Pguid->get()[_op]->init_trg(_Ptr_nav_out->get()[0]);

    // fills up cntr[0] based on initial control parameter positions present in truth[0]
    _Pcntr->initialization(_Ptr_cntr->get()[0], _Ptr_truth->get()[0], _Ptr_nav_in->get()[0], *_Pguid);

    // temporary objects
    st::st_diff Tdst_dt, Rdst_dt;
    st::st_truth Tst_truth;
    st::st_sens_in Rst_sens_in;
    st::st_out Rst_out;
    st::st_extra Ost_extra;

    // Loop with two possible exits:
    // 1 --> Final time reached
    // 2 --> No more operations available
    // In both cases, it is necessary to fill up the last member of the output vector, as well as to
    // execute the control one last time to fill up the last member of the control vector
    // loop starts with truth[t=0], sens_in[s=0], sens_out[s=0], nav_in[s=0], nav_out[s=0], gps_out[g=0], cntr[c=0] filled up
    // loop starts with out[o=0] empty

    std::cout << std::endl << "EXECUTION:" << std::endl << std::endl;

    unsigned int t = 1, s = 1, g = 1, c = 1, o = 1;
    bool flag_exit_final_time = false, flag_eval_trigger = false;
    for ( ; ; ) {
        /////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////////////////////////
        // BLOCK #1 --> OPERATION CHANGE
        // Frequency: only executes immediately after block #4 of previous iteration
        // Inputs:    nav_out[s-1]
        // Outputs:   none
        // Indexes:   none
        /////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////////////////////////

        if (flag_exit_final_time == true) { // final time reached --> exit loop
            // if final time has been reached, all trajectory vectors have the proper size and are completely filled up
            // however, those vectors may contain too many operations if not all of them have been executed, so they
            // may need to be resized accordingly
            if (_op < (_nel_op - 1)) {
                this->resize_op();
            }
            break;
        }
        if (flag_eval_trigger == true) {
            flag_eval_trigger = false;
            // evaluate trigger and advance operation if necessary
            if (_Pguid->get()[_op]->eval_trg(_Ptr_nav_out->get()[s - 1]) == true) {
                std::cout << "Conclude op # " << _op << " at time " << _Ptr_nav_out->get()[s - 1].get_t_sec() << " [sec]." << std::endl;
                ++_op;
                if (_op >= _Pguid->get_nel_op()) { // no more operations --> exit loop
                    // if there are no more operations, trajectory vectors and filters most likely do not have the proper
                    // size containing empty members at the end. They need to be resized accordingly. Also, _op needs to
                    // be reduced so control can be executed one last time outside the main loop
                    this->resize_st(t - 1, c, s - 1, o, g - 1);
                    _Pnav->resize(s, g);
                    --_op;
                    // ===== // ===== // ===== // ===== // ===== // ===== // ===== // ===== // ===== // ===== // ===== //
                    std::cout << "No more ops at time " << _Ptr_nav_out->get()[s - 1].get_t_sec() << " [sec]." << std::endl;
                    // ===== // ===== // ===== // ===== // ===== // ===== // ===== // ===== // ===== // ===== // ===== //
                    break;
                }
                // set location of operation start in all state vectors
                this->set_op_start(t - 1, c, s - 1, o, g - 1);

                // initialize new operation trigger
                _Pguid->get()[_op]->init_trg(_Ptr_nav_out->get()[s - 1]);

                // reset control primary loops set point ramp low pass filters
                this->reset_control_primary_loops(c);

                // Reset secondary loops (accumulated to zero and error to zero so derivative control starts there)
                // If primary loop, it does not matter as secondary values not employed
                // Also initialize auxiliary low pass filters for new operations
                this->reset_control_secondary_loops(c);
            }
        }

        /////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////////////////////////
        // BLOCK #2 --> CONTROL
        // Frequency: ratio of control versus integration
        // Inputs:    nav_out[s-1], nav_in[s-1], cntr[c-1]
        // Outputs:   cntr[c], truth[t-1] (controls only)
        // Indexes:   ++c
        /////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////////////////////////

        if (t > (_Ptr_cntr->get_quot() * c)) {
            // execute control step, filling up Ost_cntr[c]
            this->compute_control(_Ptr_cntr->get()[c], _Ptr_nav_out->get()[s - 1], _Ptr_cntr->get()[c - 1]);
            // copy control parameters from Ost_cntr[c] into Ost_truth[t-1], instantaneously replacing the values employed to compute the sensors inputs TODO
            // the right thing to do is to let the control surfaces slowly move towards their targets
            _Ptr_truth->get()[t-1].get_delta_control() = _Ptr_cntr->get()[c].get_delta_control();

            ++c;
        }

        /////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////////////////////////
        // BLOCK #3 --> INTEGRATION --> improved Euler (Heun's) method of order 2
        // Frequency: always
        // Inputs:    truth[t-1], sens_in[s-1], nav_out[s-1]
        // Outputs:   out[o-1], truth[t] (except controls)
        // Indexes:   ++t, and maybe ++o
        /////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////////////////////////

        // 1st integration step, filling up Tdst_dt and part of Ost_out[o-1]
        this->compute_differentials(Tdst_dt, _Ptr_out->get()[o-1], Ost_extra, _Ptr_truth->get()[t-1]);

        // compute intermediate state vector values, filling up Tst_truth
        this->solve_2nd_1st_step(Tst_truth, _Ptr_truth->get()[t-1], Tdst_dt);

        if (t > _Ptr_out->get_quot() * (o-1)) {
            // complete output vector, filling up last part of Ost_out[o-1]
            this->complete_output(_Ptr_out->get()[o-1], _Ptr_truth->get()[t-1], _Ptr_sens_in->get()[s-1], _Ptr_nav_in->get()[s-1], _Ptr_nav_out->get()[s-1]);

            if (std::fmod(o-1,100) == 0) {
                std::cout << _Ptr_out->get()[o-1].get_t_sec() << "   " << _op << std::endl;
            }

            ++o;
        }

        // 2nd integration step, filling up Rdst_dt and part fo Rst_out
        this->compute_differentials(Rdst_dt, Rst_out, Ost_extra, Tst_truth);

        // compute next state vector values, filling up Ost_truth[t]
        this->solve_2nd_2nd_step(_Ptr_truth->get()[t], _Ptr_truth->get()[t-1], Tdst_dt, Rdst_dt);

        if ((t+1) == _nel_truth) { // final time reached --> exit loop
            if (_flag_gps_lost == false) { // if gps signal has not been lost yet
                _Ptr_gps_out->resize_st(g+1);
            }
            // ===== // ===== // ===== // ===== // ===== // ===== // ===== // ===== // ===== // ===== // ===== //
            flag_exit_final_time = true;
            std::cout << "Reached final time " << _Ptr_truth->get()[t].get_t_sec() << " [sec]." << std::endl;
            // ===== // ===== // ===== // ===== // ===== // ===== // ===== // ===== // ===== // ===== // ===== //
        }
        ++t;

        /////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////////////////////////
        // BLOCK #4 --> SENSORS AND NAVIGATION
        // Frequency: ratio of sensors/navigation versus integration
        // Inputs:    truth[t-1], nav_out[s-1]
        // Outputs:   sens_in[s], sens_out[s], nav_in[s], nav_out[s], gps_out[g]
        // Indexes:   ++s, and maybe ++g
        /////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////////////////////////
        if (t > _Ptr_sens_in->get_quot() * s) {
            flag_eval_trigger = true;
            // compute sensors inputs, filling up Ost_sens_in[s] and Ost_nav_in[s]
            this->compute_sensors_input(_Ptr_sens_in->get()[s], _Ptr_nav_in->get()[s], _Ptr_truth->get()[t-1]);

            // compute sensors outputs, filling up Ost_sens_out[s]
            this->compute_sensors_output(_Ptr_sens_out->get()[s], _Ptr_sens_in->get()[s], _Ptr_sens_out->get()[s-1], _Ptr_sens_in->get()[s-1]);

            if (_flag_gps_lost == false) { // if gps signal has not been lost yet
                if (t > _Ptr_gps_out->get_quot() * g) {
                    // compute gps outputs, filling up Ost_gps_out[g-1]
                    this->compute_gps_output(_Ptr_gps_out->get()[g], _Ptr_sens_in->get()[s]);
                    _flag_gps = true;
                    ++g;
                    if (g == _Ptr_gps_out->get_nel()) {
                        std::cout << "GPS signal lost at time " << _Ptr_truth->get()[t-1].get_t_sec() << " [sec]." << std::endl;
                        if (g == _Ptr_gps_out->get_op_start(_op)) { // operation has just jumped
                            _Ptr_gps_out->activate_flag_gps_lost(g, _op);
                        }
                        else {
                            _Ptr_gps_out->activate_flag_gps_lost(g, _op + 1);
                        }
                        _flag_gps_lost = true;
                        // evaluate navigation filter, filling up Ost_nav_out[s].
                        _Pnav->execute_step(_Ptr_nav_out->get()[s], _Ptr_nav_out->get()[s-1], _Ptr_sens_out->get()[s], _Ptr_gps_out->get()[g-1], _Ptr_nav_in->get()[s], s, g-1, _flag_gps);
                        _flag_gps = false;
                        // switch navigation filter from gps active mode to gps lost mode
                        _Pnav->switch_to_gps_lost_mode(_Ptr_nav_out->get()[s], s);
                        ++s;
                        continue;
                    }
                }
            }

            // evaluate navigation filter, filling up Ost_nav_out[s].
            _Pnav->execute_step(_Ptr_nav_out->get()[s], _Ptr_nav_out->get()[s-1], _Ptr_sens_out->get()[s], _Ptr_gps_out->get()[g-1], _Ptr_nav_in->get()[s], s, g-1, _flag_gps);
            _flag_gps = false;
            ++s;
        }
        /////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////////////////////////
    } // closes main integration loop

    // complete last member of the output vector, which is empty
    this->complete_output_after(_Ptr_out->get()[o - 1], _Ptr_truth->get()[t - 1], _Ptr_sens_in->get()[s - 1], _Ptr_nav_in->get()[s - 1], _Ptr_nav_out->get()[s - 1]);

    // complete last member of control vector, which is empty
    this->compute_control(_Ptr_cntr->get()[c], _Ptr_nav_out->get()[s - 1], _Ptr_cntr->get()[c - 1]);

    // copy control parameters from control vector to truth vector
    _Ptr_truth->get()[t-1].get_delta_control() = _Ptr_cntr->get()[c].get_delta_control();
}
/* integrates the trajectory employing a 2nd order improved Euler (Heun's) method */

void nav::motion::solve_4th_perfect(const double& t_sec_end) {
    /*

    // truth states
    unsigned int nel_truth = (unsigned int)floor((t_sec_end - _Psti->get_t_sec()) / _Deltat_sec_truth) + 1;
    _Ptr_truth = new st::trj_truth(_Deltat_sec_truth, nel_truth);

    // initial conditions
    _Psti->get(_Ptr_truth->get().at(0), _Ptr_cntr->get().at(0), *_Pgeo, *_Pwind, *_Pturb, *_Ptrim, *_Pguid);

    // sensor states
    unsigned int nel_sens_in = (unsigned int)floor((t_sec_end - _Psti->get_t_sec()) / _Deltat_sec_sens) + 1;
    _Ptr_sens_in = new st::trj_sens_in(_Deltat_sec_sens, nel_sens_in, _Deltat_sec_truth);

    // control states
    unsigned int nel_cntr = (unsigned int)floor((t_sec_end - _Psti->get_t_sec()) / _Deltat_sec_cntr) + 1;
    _Ptr_cntr = new st::trj_cntr(_Deltat_sec_cntr, nel_cntr, _Deltat_sec_truth);

    // FALTA ACTUALIZAR EL CONTROL

    // output states
    unsigned int nel_out = (unsigned int)floor((t_sec_end - _Psti->get_t_sec()) / _Deltat_sec_out) + 1;
    _Ptr_out = new st::trj_out(_Deltat_sec_out, nel_out, _Deltat_sec_truth);

    // temporary objects
    st::st_diff Tdst_dt, Rdst_dt, Qdst_dt, Mdst_dt;
    st::st_truth Tst_truth, Rst_truth, Qst_truth;
    st::st_sens Rst_sens, Qst_sens, Mst_sens;
    st::st_cntr XXXXXst_cntr;
    st::st_out Rst_out, Qst_out, Mst_out;
    st::st_extra Ost_extra;
    bool flag_cntr;

    // ===== =====  4th order Runge-Kutta method ===== =====
    for (unsigned int t = 1, s = 1, c = 1, o = 1; t != nel_truth; ++t) {
        // 1st step
        this->eval(Tdst_dt, _Ptr_sens_in->get()[s-1], _Ptr_out->get()[o-1], Ost_extra, _Ptr_truth->get()[t-1], _Ptr_cntr->get()[c-1]);

        // compute 1st intermediate state vector values
        Tst_truth.get_t_sec()           = _Ptr_truth->get()[t-1].get_t_sec()           + 0.5 * _Deltat_sec_truth;
        Tst_truth.get_x_gdt_rad_m()     = _Ptr_truth->get()[t-1].get_x_gdt_rad_m()     + Tdst_dt.get_dx_gdt_rad_m_dt()     * 0.5 * _Deltat_sec_truth;
        Tst_truth.get_v_bfs_mps()       = _Ptr_truth->get()[t-1].get_v_bfs_mps()       + Tdst_dt.get_dv_bfs_mps_dt()       * 0.5 * _Deltat_sec_truth;
        Tst_truth.get_q_nedbfs()        = _Ptr_truth->get()[t-1].get_q_nedbfs()        + Tdst_dt.get_dq_nedbfs_dt()        * 0.5 * _Deltat_sec_truth;
        Tst_truth.get_w_irsbfsbfs_rps() = _Ptr_truth->get()[t-1].get_w_irsbfsbfs_rps() + Tdst_dt.get_dw_irsbfsbfs_rps_dt() * 0.5 * _Deltat_sec_truth;
        Tst_truth.get_m_kg()            = _Ptr_truth->get()[t-1].get_m_kg()            + Tdst_dt.get_dm_kg_dt()            * 0.5 * _Deltat_sec_truth;
        Tst_truth.get_q_nedbfs().normalize();

        // only keep sensed state when its own sample rate requires it
        if (t > _Ptr_sens_in->get_quot() * (s-1)) {++s;}

        // only keep output state when its own sample rate requires it
        if (t > _Ptr_out->get_quot() * (o-1)) {
            _Ptr_out->get()[o-1].get_euler_nedbfs_rad() = _Ptr_truth->get()[t-1].get_q_nedbfs();
            ++o;
        }

        // 2nd step
        this->eval(Rdst_dt, Rst_sens, Rst_out, Ost_extra, Tst_truth, XXXXXst_cntr);
        // compute 2nd intermediate state vector values
        Rst_truth.get_t_sec()           = _Ptr_truth->get()[t-1].get_t_sec()           + 0.5 * _Deltat_sec_truth;
        Rst_truth.get_x_gdt_rad_m()     = _Ptr_truth->get()[t-1].get_x_gdt_rad_m()     + Rdst_dt.get_dx_gdt_rad_m_dt()     * 0.5 * _Deltat_sec_truth;
        Rst_truth.get_v_bfs_mps()       = _Ptr_truth->get()[t-1].get_v_bfs_mps()       + Rdst_dt.get_dv_bfs_mps_dt()       * 0.5 * _Deltat_sec_truth;
        Rst_truth.get_q_nedbfs()        = _Ptr_truth->get()[t-1].get_q_nedbfs()        + Rdst_dt.get_dq_nedbfs_dt()        * 0.5 * _Deltat_sec_truth;
        Rst_truth.get_w_irsbfsbfs_rps() = _Ptr_truth->get()[t-1].get_w_irsbfsbfs_rps() + Rdst_dt.get_dw_irsbfsbfs_rps_dt() * 0.5 * _Deltat_sec_truth;
        Rst_truth.get_m_kg()            = _Ptr_truth->get()[t-1].get_m_kg()            + Rdst_dt.get_dm_kg_dt()            * 0.5 * _Deltat_sec_truth;
        Rst_truth.get_q_nedbfs().normalize();

        // 3rd step
        this->eval(Qdst_dt, Qst_sens, Qst_out, Ost_extra, Rst_truth, XXXXXst_cntr);
        // compute 3rd intermediate state vector values
        Qst_truth.get_t_sec()           = _Ptr_truth->get()[t-1].get_t_sec()           + _Deltat_sec_truth;
        Qst_truth.get_x_gdt_rad_m()     = _Ptr_truth->get()[t-1].get_x_gdt_rad_m()     + Qdst_dt.get_dx_gdt_rad_m_dt()     * _Deltat_sec_truth;
        Qst_truth.get_v_bfs_mps()       = _Ptr_truth->get()[t-1].get_v_bfs_mps()       + Qdst_dt.get_dv_bfs_mps_dt()       * _Deltat_sec_truth;
        Qst_truth.get_q_nedbfs()        = _Ptr_truth->get()[t-1].get_q_nedbfs()        + Qdst_dt.get_dq_nedbfs_dt()        * _Deltat_sec_truth;
        Qst_truth.get_w_irsbfsbfs_rps() = _Ptr_truth->get()[t-1].get_w_irsbfsbfs_rps() + Qdst_dt.get_dw_irsbfsbfs_rps_dt() * _Deltat_sec_truth;
        Qst_truth.get_m_kg()            = _Ptr_truth->get()[t-1].get_m_kg()            + Qdst_dt.get_dm_kg_dt()            * _Deltat_sec_truth;
        Qst_truth.get_q_nedbfs().normalize();

        // 4th step
        this->eval(Mdst_dt, Mst_sens, Mst_out, Ost_extra, Qst_truth, XXXXXst_cntr);
        // compute next state vector values
        _Ptr_truth->get()[t].get_t_sec()           = _Ptr_truth->get()[t-1].get_t_sec()           + _Deltat_sec_truth;
        _Ptr_truth->get()[t].get_x_gdt_rad_m()     = _Ptr_truth->get()[t-1].get_x_gdt_rad_m()     + (Tdst_dt.get_dx_gdt_rad_m_dt()     / 6.0 + Rdst_dt.get_dx_gdt_rad_m_dt()     / 3.0 + Qdst_dt.get_dx_gdt_rad_m_dt()     / 3.0 + Mdst_dt.get_dx_gdt_rad_m_dt()     / 6.0) * _Deltat_sec_truth;
        _Ptr_truth->get()[t].get_v_bfs_mps()       = _Ptr_truth->get()[t-1].get_v_bfs_mps()       + (Tdst_dt.get_dv_bfs_mps_dt()       / 6.0 + Rdst_dt.get_dv_bfs_mps_dt()       / 3.0 + Qdst_dt.get_dv_bfs_mps_dt()       / 3.0 + Mdst_dt.get_dv_bfs_mps_dt()       / 6.0) * _Deltat_sec_truth;
        _Ptr_truth->get()[t].get_q_nedbfs()        = _Ptr_truth->get()[t-1].get_q_nedbfs()        + (Tdst_dt.get_dq_nedbfs_dt()        / 6.0 + Rdst_dt.get_dq_nedbfs_dt()        / 3.0 + Qdst_dt.get_dq_nedbfs_dt()        / 3.0 + Mdst_dt.get_dq_nedbfs_dt()        / 6.0) * _Deltat_sec_truth;
        _Ptr_truth->get()[t].get_w_irsbfsbfs_rps() = _Ptr_truth->get()[t-1].get_w_irsbfsbfs_rps() + (Tdst_dt.get_dw_irsbfsbfs_rps_dt() / 6.0 + Rdst_dt.get_dw_irsbfsbfs_rps_dt() / 3.0 + Qdst_dt.get_dw_irsbfsbfs_rps_dt() / 3.0 + Mdst_dt.get_dw_irsbfsbfs_rps_dt() / 6.0) * _Deltat_sec_truth;
        _Ptr_truth->get()[t].get_m_kg()            = _Ptr_truth->get()[t-1].get_m_kg()            + (Tdst_dt.get_dm_kg_dt()            / 6.0 + Rdst_dt.get_dm_kg_dt()            / 3.0 + Qdst_dt.get_dm_kg_dt()            / 3.0 + Mdst_dt.get_dm_kg_dt()            / 6.0) * _Deltat_sec_truth;
        _Ptr_truth->get()[t].get_q_nedbfs().normalize();
    }

    */
}
/* integrates the trajectory employing a 4th order Runge-Kutta method */

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////





