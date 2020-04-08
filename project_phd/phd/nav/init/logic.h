#ifndef NAV_MOTION_LOGIC
#define NAV_MOTION_LOGIC

#include "../nav.h"

namespace nav {
    namespace logic {
        /**< Enumeration that contains the different initial Euler angle error generators */
        enum INITEUL_ID {
            initeul_id_zero   = 0,
            initeul_id_base   = 1,
            initeul_id_better = 2,
            initeul_id_worse  = 3,
            initeul_id_worst  = 4,
            initeul_size      = 5
        };
        /**< Enumeration that contains the different initial accelerometer error generators */
        enum INITACC_ID {
            initacc_id_zero  = 0,
            initacc_id_base  = 1,
            initacc_id_worse = 2,
            initacc_id_worst = 3,
            initacc_size     = 4
        };
        /**< Enumeration that contains the different initial gyroscope error generators */
        enum INITGYR_ID {
            initgyr_id_zero  = 0,
            initgyr_id_base  = 1,
            initgyr_id_worse = 2,
            initgyr_id_worst = 3,
            initgyr_size     = 4
        };
        /**< Enumeration that contains the different initial magnetometer error generators */
        enum INITMAG_ID {
            initmag_id_zero  = 0,
            initmag_id_base  = 1,
            initmag_id_worse = 2,
            initmag_id_worst = 3,
            initmag_size     = 4
        };
        /**< Enumeration that contains the different initial NED magnetic field error generators */
        enum INITMGN_ID {
            initmgn_id_zero  = 0,
            initmgn_id_base  = 1,
            initmgn_id_worse = 2,
            initmgn_id_worst = 3,
            initmgn_size     = 4
        };
    } // closes namespace logic
}; // closes namespace nav

#endif
