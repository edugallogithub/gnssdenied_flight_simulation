#ifndef ACFT_GUID
#define ACFT_GUID

#include "../acft.h"
#include "../st/sti.h"
#include "../st/trj_nav_out.h"
#include "../control/logic.h"
#include "math/logic/seeder.h"
#include <vector>

namespace control {
    class trg;
    class motion;

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS GUIDANCE_OPERATION
// ========================
// ========================

class ACFT_API guid_op {
private:
    /**< throttle guidance mode */
    control::logic::THR_ID _guid_thr_id;
    /**< elevator guidance mode */
    control::logic::ELV_ID _guid_elv_id;
    /**< ailerons guidance mode */
    control::logic::AIL_ID _guid_ail_id;
    /**< rudder guidance mode */
    control::logic::RUD_ID _guid_rud_id;
    /**< trigger guidance mode */
    control::logic::TRG_ID _guid_trg_id;

    /**< guidance mode values (throttle, elevator, ailerons, rudder) */
    Eigen::Array4d _guid_val;
    /**< trigger guidance mode value */
    double _guid_trg_val;

    /**< pointer to functor that evaluates trigger */
    control::trg* _Pguid_trg_functor;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< default constructor */
    guid_op() = delete;
    /**< constructor based on guidance identificators */
    guid_op(control::logic::THR_ID guid_thr_id, const double& guid_thr_val,
            control::logic::ELV_ID guid_elv_id, const double& guid_elv_val,
            control::logic::AIL_ID guid_ail_id, const double& guid_ail_val,
            control::logic::RUD_ID guid_rud_id, const double& guid_rud_val,
            control::logic::TRG_ID guid_trg_id, const double& guid_trg_val);
    /**< copy constructor */
    guid_op(const guid_op&) = delete;
    /**< move constructor */
    guid_op(guid_op&&) = delete;
    /**< destructor */
    ~guid_op();
    /**< copy assignment */
    guid_op& operator=(const guid_op&) = delete;
    /**< move assignment */
    guid_op& operator=(guid_op&&) = delete;

    /**< initialize trigger so it later jumps when it switches sign */
    void init_trg(const st::st_nav_out& Ost_nav_out);
    /**< evaluate trigger, returning true when it jumps */
    bool eval_trg(const st::st_nav_out& Ost_nav_out) const;

    /**< get throttle guidance id */
    const control::logic::THR_ID& get_guid_thr_id() const {return _guid_thr_id;}
    /**< get elevator guidance id */
    const control::logic::ELV_ID& get_guid_elv_id() const {return _guid_elv_id;}
    /**< get ailerons guidance id */
    const control::logic::AIL_ID& get_guid_ail_id() const {return _guid_ail_id;}
    /**< get rudder guidance id */
    const control::logic::RUD_ID& get_guid_rud_id() const {return _guid_rud_id;}
    /**< get trigger id */
    const control::logic::TRG_ID& get_guid_trg_id() const {return _guid_trg_id;}

    /**< get guidance target value based on guidance mode (throttle, elevator, ailerons, rudder) */
    const double & get_guid_val(control::logic::CNTR_ID cntr_id) const {return _guid_val(cntr_id);}
    /**< get trigger value */
    const double get_guid_trg_val() const {return _guid_trg_val;}
}; // closes class guid_op

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS GUIDANCE
// ==============
// ==============

class ACFT_API guid {
private:
    /**< initial conditions ID */
    st::logic::STI_ID _sti_id;
    /**< number of operations */
    unsigned short _nel_op;
    /**< vector with operations */
    std::vector<control::guid_op*> _Vguid_op;
    /**< final time overriding intent */
    double _t_sec_end;
    /**< guidance case */
    unsigned short _case_guid;

    /**< seed generator */
    std::ranlux24_base _gen;
    /**< standard normal distribution */
    std::normal_distribution<double> _dist_normal;
    /**< uniform integer distribution */
    std::uniform_int_distribution<int> _dist_uniform_bearing;
    /**< uniform integer distribution */
    std::uniform_int_distribution<int> _dist_uniform_time;
public:
    /**< default constructor */
    guid() = delete;
    /**< constructor based on number of operations (user responsibility to fill it up afterwards) and final time.
     * Seed not employed*/
    guid(const unsigned short& nel_op, const double& t_sec_end, const int& seed, const unsigned short& case_guid);
    /**< copy constructor */
    guid(const guid&) = delete;
    /**< move constructor */
    guid(guid&&) = delete;
    /**< destructor */
    ~guid();
    /**< copy assignment */
    guid& operator=(const guid&) = delete;
    /**< move assignment */
    guid& operator=(guid&&) = delete;

    /**< adds operation at position indicated by index */
    void add_op(const unsigned short& index, control::guid_op* Pop);

    /**< get vector of operations */
    std::vector<control::guid_op*>& operator()() {return _Vguid_op;}
    const std::vector<control::guid_op*>& operator()() const {return _Vguid_op;}
    /**< get vector of operations */
    std::vector<control::guid_op*>& get() {return _Vguid_op;}
    const std::vector<control::guid_op*>& get() const {return _Vguid_op;}
    /**< get final time overriding intent */
    double& get_t_sec_end() {return _t_sec_end;}
    const double& get_t_sec_end() const {return _t_sec_end;}
    /**< get number of operations */
    const unsigned short& get_nel_op() const {return _nel_op;}
    /**< get appropriate initial conditions ID */
    const st::logic::STI_ID& get_sti_id() const {return _sti_id;}
    st::logic::STI_ID& get_sti_id() {return _sti_id;}
    /**< get guidance case */
    const unsigned short& get_case_guid() const {return _case_guid;}

    /**< returns pointer to guidance objectives based on seeder on case identifier. */
    static guid* create_guid(math::seeder& Oseeder, const unsigned short& case_guid, const double& t_sec_gpsloss, bool flag_console = true);
    /**< describe guidance in stream */
    void create_text(std::ostream& Ostream) const;
}; // closes class guid

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

} // closes namespace control

#endif
