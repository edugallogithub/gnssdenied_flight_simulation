#ifndef ACFT_TRG
#define ACFT_TRG

#include "../acft.h"
#include "../st/trj_nav_out.h"
#include <vector>

namespace control {
    class guid_op;

// CLASS TRIGGER
// =============
// =============

class ACFT_API trg {
protected:
    /**< weak pointer to operation */
    const control::guid_op* const _Pop;
    /**< sign of trigger (jumps when it changes) */
    bool _trg_sign_output;
public:
    /**< constructor based on reference to operation */
    explicit trg(const control::guid_op& Oop);
    /**< trigger initialization */
    virtual void initialize(const st::st_nav_out& Ost_nav_out) = 0;
    /**< trigger evaluation */
    virtual bool eval(const st::st_nav_out& Ost_nav_out) = 0;
}; // closes class trg

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS TRG_T_SEC
// ===============
// ===============

class ACFT_API trg_t_sec : public trg {
private:
    /**< internal trigger computation */
    double compute(const st::st_nav_out& Ost_nav_out) const;
public:
    /**< constructor based on reference to operation */
    explicit trg_t_sec(const control::guid_op& Oop) : trg(Oop) {}
    /**< trigger initialization */
    void initialize(const st::st_nav_out& Ost_nav_out) override;
    /**< trigger evaluation */
    bool eval(const st::st_nav_out& Ost_nav_out) override;
 }; // closes class trg_t_sec

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS TRG_DELTAT_SEC
// ====================
// ====================

class ACFT_API trg_Deltat_sec : public trg {
private:
    /**< initialization time */
    double _t_sec_init;
    /**< internal trigger computation */
    double compute(const st::st_nav_out& Ost_nav_out) const;
public:
    /**< constructor based on reference to operation */
    explicit trg_Deltat_sec(const control::guid_op& Oop) : trg(Oop), _t_sec_init(0.) {}
    /**< trigger initialization */
    void initialize(const st::st_nav_out& Ost_nav_out) override;
    /**< trigger evaluation */
    bool eval(const st::st_nav_out& Ost_nav_out) override;
}; // closes class trg_Deltat_sec

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS TRG_h_M
// =============
// =============

class ACFT_API trg_h_m : public trg {
private:
    /**< internal trigger computation */
    double compute(const st::st_nav_out& Ost_nav_out) const;
public:
    /**< constructor based on reference to operation */
    explicit trg_h_m(const control::guid_op& Oop) : trg(Oop) {}
    /**< trigger initialization */
    void initialize(const st::st_nav_out& Ost_nav_out) override;
    /**< trigger evaluation */
    bool eval(const st::st_nav_out& Ost_nav_out) override;
}; // closes class trg_h_m

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS TRG_HP_M
// ==============
// ==============

class ACFT_API trg_Hp_m : public trg {
private:
    /**< internal trigger computation */
    double compute(const st::st_nav_out& Ost_nav_out) const;
public:
    /**< constructor based on reference to operation */
    explicit trg_Hp_m(const control::guid_op& Oop) : trg(Oop) {}
    /**< trigger initialization */
    void initialize(const st::st_nav_out& Ost_nav_out) override;
    /**< trigger evaluation */
    bool eval(const st::st_nav_out& Ost_nav_out) override;
}; // closes class trg_Hp_m

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS TRG_GAMMA_DEG
// ===================
// ===================

class ACFT_API trg_gamma_deg : public trg {
private:
    /**< internal trigger computation */
    double compute(const st::st_nav_out& Ost_nav_out) const;
public:
    /**< constructor based on reference to operation */
    explicit trg_gamma_deg(const control::guid_op& Oop) : trg(Oop) {}
    /**< trigger initialization */
    void initialize(const st::st_nav_out& Ost_nav_out) override;
    /**< trigger evaluation */
    bool eval(const st::st_nav_out& Ost_nav_out) override;
}; // closes class trg_gamma_deg

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS TRG_CHI_DEG
// =================
// =================

class ACFT_API trg_chi_deg : public trg {
private:
    /**< flag that indicates whether trigger may jump or not (is more than 90 deg away) */
    bool _flag_active;
    /**< internal trigger computation */
    double compute(const st::st_nav_out& Ost_nav_out) const;
public:
    /**< constructor based on reference to operation */
    explicit trg_chi_deg(const control::guid_op& Oop) : trg(Oop), _flag_active(false) {}
    /**< trigger initialization */
    void initialize(const st::st_nav_out& Ost_nav_out) override;
    /**< trigger evaluation */
    bool eval(const st::st_nav_out& Ost_nav_out) override;
}; // closes class trg_chi_deg

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS TRG_PSI_DEG
// =================
// =================

class ACFT_API trg_psi_deg : public trg {
private:
    /**< flag that indicates whether trigger may jump or not (is more than 90 deg away) */
    bool _flag_active;
    /**< internal trigger computation */
    double compute(const st::st_nav_out& Ost_nav_out) const;
public:
    /**< constructor based on reference to operation */
    explicit trg_psi_deg(const control::guid_op& Oop) : trg(Oop) {}
    /**< trigger initialization */
    void initialize(const st::st_nav_out& Ost_nav_out) override;
    /**< trigger evaluation */
    bool eval(const st::st_nav_out& Ost_nav_out) override;
}; // closes class trg_psi_deg

//////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////

} // closes namespace control

#endif
