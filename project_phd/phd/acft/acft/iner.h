#ifndef ACFT_INER
#define ACFT_INER

#include "../acft.h"
#include "math/math/func.h"
#include "math/vec/vec1.h"
#include "math/pred/pred1v/f_table1V.h"
#include "ang/rotate/euler.h"
#include "ang/rotate/dcm.h"
#include "ang/rotate/rodrigues.h"
#include <string>

namespace acft {

// CLASS INER
// ==========
// ==========

class ACFT_API iner {
private:
	/**< name required internally */
	static const std::string _filename;

	/**< full tank aircraft mass */
	double _full_m_kg;
	/**< full tank translation from aircraft reference (wing trailing edge) "r" to body (center of gravity) viewed in body */
    Eigen::Array3d _full_Trbb_m;
	/**< full tank tensor of inertia */
    Eigen::Matrix3d _full_I_kgm2;

    /**< empty tank aircraft mass */
    double _empty_m_kg;
    /**< empty tank translation from aircraft reference (wing trailing edge) "r" to body (center of gravity) viewed in body */
    Eigen::Array3d _empty_Trbb_m;
    /**< empty tank tensor of inertia */
    Eigen::Matrix3d _empty_I_kgm2;
public:
    /**< default constructor (based on internal text file) */
    iner();
    /**< copy constructor */
    iner(const iner&) = delete;
    /**< move constructor */
    iner(iner&&) = delete;
    /**< destructor */
    ~iner() = default;
    /**< copy assignment */
    iner& operator=(const iner&) = delete;
    /**< move assignment */
    iner& operator=(iner&&) = delete;

    /**< ===== ===== ===== Full Tank ===== ===== ===== */
    /**< get full tank aircraft mass */
    const double& get_full_m_kg() const {return _full_m_kg;}
    /**< get full tank translation from aircraft reference (wing trailing edge) "r" to body (center of gravity) viewed in body */
    const Eigen::Array3d& get_full_Trbb_m() const {return _full_Trbb_m;}
    /**< get full tank inertia matrix */
    const Eigen::Matrix3d& get_full_I_kgm2() const {return _full_I_kgm2;}

    /**< ===== ===== ===== Empty Tank ===== ===== ===== */
    /**< get empty tank aircraft mass */
    const double& get_empty_m_kg() const {return _empty_m_kg;}
    /**< get empty tank translation from aircraft reference (wing trailing edge) "r" to body (center of gravity) viewed in body */
    const Eigen::Array3d& get_empty_Trbb_m() const {return _empty_Trbb_m;}
    /**< get empty tank inertia matrix */
    const Eigen::Matrix3d& get_empty_I_kgm2() const {return _empty_I_kgm2;}

    /**< ===== ===== ===== Mass and Inertia ===== ===== ===== */
    /**< obtain mass ratio (0 means full, 1 means empty) */
    double get_ratio(const double& m_kg) const;
    /**< get aircraft mass based on mass ratio */
    double get_m_kg(const double& ratio) const;
    /**< get inertia matrix based on mass ratio */
    Eigen::Matrix3d get_I_kgm2(const double& ratio) const;
    /**< get inverse of inertia matrix */
    static Eigen::Matrix3d get_I_kgm2_inverse(const Eigen::Matrix3d& I);

    /**< get translation from aircraft reference (wing trailing edge) "r" to body (center of gravity) viewed in body based on mass ratio */
	Eigen::Array3d get_Trbb_m(const double& ratio) const;

}; // closes class iner

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

}; // closes namespace acft

#endif


