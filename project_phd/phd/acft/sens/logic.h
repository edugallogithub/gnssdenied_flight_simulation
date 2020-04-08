#ifndef ACFT_SENSOR_LOGIC
#define ACFT_SENSOR_LOGIC

#include "../acft.h"

namespace sens {

    namespace logic {
        /**< Enumeration that specifies the different terms considered in the sensors */
        enum SENS_COMPLETE_ID {
            sens_complete_noise             = 0,
            sens_complete_bias              = 1,
            sens_complete_bias_scale        = 2,
            sens_complete_bias_scale_par    = 3,
            sens_complete_size              = 4
        };
        /**< Enumeration that contains the random walk oscillation bands for inertial sensors */
        enum BAND_ID {
            band_id_base    = 0,
            band_id_bigger  = 1,
            band_id_biggest = 2,
            band_id_size    = 3
        };
        /**< Enumeration that contains the different accelerometer models */
        enum ACC_ID {
            acc_id_zero       = 0,
            acc_id_base       = 1,
            acc_id_better     = 2,
            acc_id_worse      = 3,
            acc_id_worst      = 4,
            acc_id53 = 5,
            acc_id_noise_only = 6,
            acc_size          = 7
        };
        /**< Enumeration that contains the different gyroscope models */
        enum GYR_ID {
            gyr_id_zero       = 0,
            gyr_id_base       = 1,
            gyr_id_better     = 2,
            gyr_id_worse      = 3,
            gyr_id_worst      = 4,
            gyr_id53 = 5,
            gyr_id54 = 6,
            gyr_id_noise_only = 7,
            gyr_size          = 8
        };
        /**< Enumeration that contains the different magnetometer models */
        enum MAG_ID {
            mag_id_zero   = 0,
            mag_id_base   = 1,
            mag_id_better = 2,
            mag_id_worse  = 3,
            mag_size      = 4
        };
        /**< Enumeration that contains the different outside static pressure sensor models */
        enum OSP_ID {
            osp_id_zero  = 0,
            osp_id_base  = 1,
            osp_id_worse = 2,
            osp_id_worst = 3,
            osp_size     = 4
        };
        /**< Enumeration that contains the different outside air temperature sensor models */
        enum OAT_ID {
            oat_id_zero  = 0,
            oat_id_base  = 1,
            oat_id_worse = 2,
            oat_id_worst = 3,
            oat_size     = 4
        };
        /**< Enumeration that contains the different true airspeed sensor models */
        enum TAS_ID {
            tas_id_zero   = 0,
            tas_id_base   = 1,
            tas_id_worse  = 2,
            tas_id_better = 3,
            tas_size      = 4
        };
        /**< Enumeration that contains the different angle of attack sensor models */
        enum AOA_ID {
            aoa_id_zero   = 0,
            aoa_id_base   = 1,
            aoa_id_worse  = 2,
            aoa_id_better = 3,
            aoa_size      = 4
        };
        /**< Enumeration that contains the different angle of sideslip sensor models */
        enum AOS_ID {
            aos_id_zero   = 0,
            aos_id_base   = 1,
            aos_id_worse  = 2,
            aos_id_better = 3,
            aos_size      = 4
        };
        /**< Enumeration that contains the different GPS receiver models */
        enum GPS_ID {
            gps_id_zero = 0,
            gps_id_base = 1,
            gps_size    = 2
        };

    } // closes namespace logic

}; // closes namespace sens

#endif
