#ifndef ACFT_TRJ
#define ACFT_TRJ

#include "../acft.h"
#include <vector>

namespace st {

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS TRAJECTORY BASE
// =====================
// =====================

class ACFT_API trj {
protected:
    /**< time separation between consecutive samples */
    double _Deltat_sec;
    /**< trajectory size at the frequency specified by _Deltat_sec */
    unsigned int _nel;
    /**< number of operations */
    unsigned short _nel_op;
    /**< vector containing first state of each operation */
    std::vector<unsigned int> _Vop_start;
    /**< factor between _Deltat_sec and the time separation of the truth trajectory */
    int _quot;
public:
    /**< default constructor */
    trj() = delete;
    /**< constructor based on time separation between consecutive samples and number of operations */
    trj(const double& Deltat_sec, const unsigned int& nel, const unsigned short& nel_op, const int& quot)
        : _Deltat_sec(Deltat_sec), _nel(nel), _nel_op(nel_op), _Vop_start(nel_op, 0), _quot(quot) {}
    /**< copy constructor */
    trj(const trj&) = delete;
    /**< move constructor */
    trj(trj&&) = delete;
    /**< destructor */
    virtual ~trj() = default;
    /**< copy assignment */
    trj& operator=(const trj&) = delete;
    /**< move assignment */
    trj& operator=(trj&&) = delete;

    /**< get time separation between consecutive samples */
    virtual const double& get_Deltat_sec() const {return _Deltat_sec;}
    /**< get trajectory size at the frequency specified by _Deltat_sec */
    virtual const unsigned int& get_nel() const {return _nel;}
    /**< get number of operations */
    virtual const unsigned short& get_nel_op() const {return _nel_op;}
    /**< get vector containing first state of each operation */
    virtual const std::vector<unsigned int>& get_Vop_start() const {return _Vop_start;}
    /**< get factor between _Deltat_sec and the time separation of the truth trajectory */
    virtual const int& get_quot() const {return _quot;}

    /**< resize trajectory to new size, leaving only the first members. Number of operations does not change. */
    virtual void resize_st(const unsigned int& nel) = 0;
    /**< update number of operations. State vectors do not change */
    virtual void resize_op(const unsigned short& nel_op) {
        _nel_op = nel_op;
        _Vop_start.resize(nel_op);
    }
    /**< sets the state at which a given operation starts */
    virtual void set_op_start(const unsigned short& op, const unsigned int& t) {
        _Vop_start[op] = t;
    }
    /**< get position of first state of input operation */
    virtual const unsigned int& get_op_start(const unsigned short& op) const {
        return _Vop_start[op];
    }
    /**< get position of last state of input operation */
    virtual unsigned int get_op_end(const unsigned short& op) const {
        if (op < (_nel_op - 1)) {
            return _Vop_start[op+1] - 1;
        }
        else {
            return _nel - 1;
        }
    }
}; // closes class trj

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

} // closes namespace st

#endif















