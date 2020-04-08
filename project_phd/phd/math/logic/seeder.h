#ifndef MATH_SEEDER
#define MATH_SEEDER

#include "../math.h"
#include <random>

namespace math {

// SEEDER
// ======
// ======

class MATH_API seeder {
public:
    /**< enumeration to be able to ensure that seeds are always provided in the same order and that way
    * the pseudo numbers produced with them are full repeatable. */
    enum SEEDER_ID {
        seeder_acc        = 0,  // suite --> accelerometers bias offset, bias drift, and white noise
        seeder_gyr        = 1,  // suite --> gyroscopes bias offset, bias drift, and white noise
        seeder_mag        = 2,  // suite --> magnetometers bias offset and white noise
        seeder_osp        = 3,  // suite --> static pressure sensor white noise
        seeder_tas        = 4,  // suite --> airspeed sensor white noise
        seeder_aoa        = 5,  // suite --> angle of attack bias offset and white noise
        seeder_aos        = 6,  // suite --> angle of sideslip bias offset and white noise
        seeder_oat        = 7,  // suite --> temperature sensor white noise
        seeder_gps        = 8,  // suite --> gps ionospheric, position white noise, and vel white noise
        seeder_initeul    = 9,  // init_error --> initial Euler angles error
        seeder_acft_plat  = 10, // suite --> relation between body and platform frames
        seeder_initmgn    = 11, // init error --> initial magnetic field error
        seeder_geo        = 12, // earth --> gravity and magnetic field realism
        seeder_acft_acc   = 13, // suite --> accelerometers scale factor & cross coupling
        seeder_acft_gyr   = 14, // suite --> gyroscopes scale factor & cross coupling
        seeder_acft_mag   = 15, // suite --> magnetometers scale factor & cross coupling
        seeder_align      = 16, // NOT USED
        seeder_offsets    = 17, // earth --> temperature and pressure offsets
        seeder_wind       = 18, // earth --> wind field
        seeder_guid       = 19, // guid --> guidance objectives
        seeder_initacc    = 20, // init_error --> initial accelerometer error
        seeder_initgyr    = 21, // init_error --> initial gyroscope error
        seeder_initmag    = 22, // init_error --> initial magnetometer error
        seeder_id_size    = 23
    };
private:
    /**< starting seed order (from 1 to 99) */
    unsigned short _seed_order;
    /**< specific seeds for the different processes */
    std::vector<int> _seeds;
        
    /**< seed generator */
    std::ranlux24_base _Ogen;
    /**< uniform distribution to generate seeds */
    std::uniform_int_distribution<> _Odist;
public:
    /**< default constructor */
    seeder() = delete;
    /**< constructor based on starting seed order (from 1 to 50) */
    explicit seeder(const unsigned short& seed_order);
    /**< destructor */
    ~seeder() = default;

    /**< generate pseudorandom seed, verifying that requested seed is in the right order */
    int provide_seed(math::seeder::SEEDER_ID seeder_id);
    /**< generates input number of seeds (always the same ones) and stores them in text file. Intended to be run only once.*/
    static void generate_seeds(const unsigned short& n);
    /**< get order with which object was created */
    const unsigned short& get_seed_order() const {return _seed_order;}
    /**< returns a two digit string containing the seed order (07 for 7, 84 for 84, 00 for 100) */
    static std::string seed2string(unsigned short seed_order);

}; // closes class seeder

} // closes namespace math

#endif


