#include "suite.h"

#include "math/logic/share.h"
#include "ang/rotate/rodrigues.h"
#include "acft/acft/iner.h"
#include <fstream>
#include <boost/filesystem.hpp>

// CLASS SUITE
// ===========
// ===========

sens::suite::suite(math::seeder& Oseeder,
                   sens::logic::ACC_ID acc_id, sens::logic::GYR_ID gyr_id, sens::logic::MAG_ID mag_id,
                   sens::logic::OSP_ID osp_id, sens::logic::OAT_ID oat_id, sens::logic::TAS_ID tas_id,
                   sens::logic::AOA_ID aoa_id, sens::logic::AOS_ID aos_id, sens::logic::GPS_ID gps_id,
                   sens::logic::BAND_ID band_id,
                   const double& Deltat_sec_sens, bool flag_console)
: _acc_id(acc_id), _gyr_id(gyr_id),
  _Pacc(nullptr), _Pgyr(nullptr), _Pmag(nullptr), _Posp(nullptr), _Poat(nullptr), _Ptas(nullptr), _Paoa(nullptr), _Paos(nullptr), _Pgps(nullptr), _Pplat(nullptr) {

    // fill up individual sensor pointers
    _Pplat = sens::platform::create_platform(Oseeder.provide_seed(math::seeder::seeder_acft_plat), flag_console);
    _Pacc  = sens::sens_triple_acc::create_accelerometer(acc_id,  band_id, Oseeder.provide_seed(math::seeder::seeder_acft_acc), Oseeder.provide_seed(math::seeder::seeder_acc), *_Pplat, Deltat_sec_sens);
    _Pgyr  = sens::sens_triple_gyr::create_gyroscope(gyr_id,      band_id, Oseeder.provide_seed(math::seeder::seeder_acft_gyr), Oseeder.provide_seed(math::seeder::seeder_gyr), *_Pplat, Deltat_sec_sens);
    _Pmag  = sens::sens_triple_mag::create_magnetometer(mag_id,   Oseeder.provide_seed(math::seeder::seeder_acft_mag), Oseeder.provide_seed(math::seeder::seeder_mag), Oseeder.get_seed_order(), Deltat_sec_sens);
    _Posp  = sens::sens_single_no_walk::create_osp_sensor(osp_id, Oseeder.provide_seed(math::seeder::seeder_osp), Deltat_sec_sens);
    _Ptas  = sens::sens_single_no_walk::create_tas_sensor(tas_id, Oseeder.provide_seed(math::seeder::seeder_tas), Deltat_sec_sens);
    _Paoa  = sens::sens_single_no_walk::create_aoa_sensor(aoa_id, Oseeder.provide_seed(math::seeder::seeder_aoa), Deltat_sec_sens);
    _Paos  = sens::sens_single_no_walk::create_aos_sensor(aos_id, Oseeder.provide_seed(math::seeder::seeder_aos), Deltat_sec_sens);
    _Poat  = sens::sens_single_no_walk::create_oat_sensor(oat_id, Oseeder.provide_seed(math::seeder::seeder_oat), Deltat_sec_sens);
    _Pgps  = sens::sens_triple_gps::create_gps_sensor(gps_id,     Oseeder.provide_seed(math::seeder::seeder_gps));
}
/* constructor based on seed order, specific accelerometer, gyroscope, magnetometer, pressure, temperature, airspeed, angle of attack, angle of sideslip. Flag true to show platform summary in console. */

sens::suite::~suite() {
    delete _Pacc;
    delete _Pgyr;
    delete _Pmag;
    delete _Posp;
    delete _Poat;
    delete _Ptas;
    delete _Paoa;
    delete _Paos;
    delete _Pgps;
    delete _Pplat;
}
/* destructor */

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////







