#include "earth.h"
#include "math/logic/share.h"

// CLASS EARTH
// ===========
// ===========

env::earth::earth(math::seeder& Oseeder,
                  env::logic::WIND_ID wind_id, env::logic::OFFSETS_ID offsets_id, env::logic::MAG_ID mag_id, env::logic::REALISM_ID realism_id,
                  const double& turb_factor, const double& t_sec_turb_init, const int& t_sec_turb_end)
: _Poffsets(nullptr), _Pwind(nullptr), _Pturb(nullptr), _Pgeo(nullptr) {

    _Poffsets = env::offsets::create_offsets(offsets_id, Oseeder.provide_seed(math::seeder::seeder_offsets));
    _Pwind    = env::wind::create_wind(wind_id, Oseeder.provide_seed(math::seeder::seeder_wind));
    _Pturb    = env::turb::create_turb(t_sec_turb_init, Oseeder.get_seed_order(), t_sec_turb_end, turb_factor);
    _Pgeo     = new env::geo_mix(mag_id, realism_id, Oseeder.provide_seed(math::seeder::seeder_geo));
}
/* constructor based on seeder object, identifiers for wind filed, atmospheric offsets, magnetic location, gravity and
 * magnetism realism, together with turbulence level factor, initial time (for turbulence), and final time (only two
 * values allowed as preloaded, 1000 or 5000) */

env::earth::~earth() {
    delete _Poffsets;
    delete _Pwind;
    delete _Pturb;
    delete _Pgeo;
}
/* destructor */

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////







