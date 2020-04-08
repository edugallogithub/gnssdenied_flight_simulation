#ifndef ACFT_SUITE
#define ACFT_SUITE

#include "../acft.h"
#include "sens_single_inertial.h"
#include "sens_single_noise.h"
#include "sens_single_no_walk.h"
#include "sens_triple_acc.h"
#include "sens_triple_gyr.h"
#include "sens_triple_mag.h"
#include "sens_triple_gps.h"
#include "platform.h"
#include "math/logic/seeder.h"
#include "ang/rotate/euler.h"

namespace sens {

// CLASS SUITE
// ===========
// ===========

class ACFT_API suite {
private:
    /**< pointer to accelerometer */
    sens::sens_triple_acc* _Pacc;
    /**< pointer to gyroscope */
    sens::sens_triple_gyr* _Pgyr;
    /**< pointer to magnetometer */
    sens::sens_triple_mag* _Pmag;
    /**< pointer to outside static pressure sensor */
    sens::sens_single_no_walk* _Posp;
    /**< pointer to outside air temperature sensor */
    sens::sens_single_no_walk* _Poat;
    /**< pointer to true airspeed sensor */
    sens::sens_single_no_walk* _Ptas;
    /**< pointer to angle of attack sensor */
    sens::sens_single_no_walk* _Paoa;
    /**< pointer to angle of sideslip sensor */
    sens::sens_single_no_walk* _Paos;
    /**< pointer to GPS sensor */
    sens::sens_triple_gps* _Pgps;
    /**< pointer to platform */
    sens::platform* _Pplat;

    /**< accelerometer id */
    sens::logic::ACC_ID _acc_id;
    /** gyroscope id */
    sens::logic::GYR_ID _gyr_id;

    /**< includes relationship between body and platform frames */
    void add_bp(const int& acft_bp_seed);
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**< default constructor */
    suite() = delete;
    /**< constructor based on seed order and identifiers. Flag true to show platform summary in console. */
    suite(math::seeder& Oseeder,
          sens::logic::ACC_ID acc_id, sens::logic::GYR_ID gyr_id, sens::logic::MAG_ID mag_id,
          sens::logic::OSP_ID osp_id, sens::logic::OAT_ID oat_id, sens::logic::TAS_ID tas_id,
          sens::logic::AOA_ID aoa_id, sens::logic::AOS_ID aos_id, sens::logic::GPS_ID gps_id,
          sens::logic::BAND_ID band_id,
          const double& Deltat_sec_sens = 0.01, bool flag_console = true);
    /**< copy constructor */
    suite(const suite&) = delete;
    /**< move constructor */
    suite(suite&&) = delete;
    /**< destructor */
    ~suite();
    /**< copy assignment */
    suite& operator=(const suite&) = delete;
    /**< move assignment */
    suite& operator=(suite&&) = delete;

    /**< get accelerometer */
    const sens::sens_triple_acc& get_sens_acc() const {return *_Pacc;}
    sens::sens_triple_acc& get_sens_acc() {return *_Pacc;}
    /**< get gyroscope */
    const sens::sens_triple_gyr& get_sens_gyr() const {return *_Pgyr;}
    sens::sens_triple_gyr& get_sens_gyr() {return *_Pgyr;}
    /**< get magnetometer */
    const sens::sens_triple_mag& get_sens_mag() const {return *_Pmag;}
    sens::sens_triple_mag& get_sens_mag() {return *_Pmag;}
    /**< get outside static pressure sensor */
    const sens::sens_single_no_walk& get_sens_osp() const {return *_Posp;}
    sens::sens_single_no_walk& get_sens_osp() {return *_Posp;}
    /**< get outside air temperature sensor */
    const sens::sens_single_no_walk& get_sens_oat() const {return *_Poat;}
    sens::sens_single_no_walk& get_sens_oat() {return *_Poat;}
    /**< get true airspeed sensor */
    const sens::sens_single_no_walk& get_sens_tas() const {return *_Ptas;}
    sens::sens_single_no_walk& get_sens_tas() {return *_Ptas;}
    /**< get angle of attack sensor */
    const sens::sens_single_no_walk& get_sens_aoa() const {return *_Paoa;}
    sens::sens_single_no_walk& get_sens_aoa() {return *_Paoa;}
    /**< get angle of sideslip sensor */
    const sens::sens_single_no_walk& get_sens_aos() const {return *_Paos;}
    sens::sens_single_no_walk& get_sens_aos() {return *_Paos;}
    /**< get GPS sensor */
    const sens::sens_triple_gps& get_sens_gps() const {return *_Pgps;}
    sens::sens_triple_gps& get_sens_gps() {return *_Pgps;}
    /**< get GPS sensor */
    const sens::platform& get_platform() const {return *_Pplat;}
    sens::platform& get_platform() {return *_Pplat;}

}; // closes class suite

}; // closes namespace sens

#endif


