#ifndef ENV_EARTH
#define ENV_EARTH

#include "env.h"
#include "mag.h"
#include "offsets.h"
#include "wind.h"
#include "turb.h"
#include "geo.h"
#include "math/logic/seeder.h"

namespace env {

// CLASS EARTH
// ===========
// ===========

class ENV_API earth {
private:
    /**< pointer to temperature and pressure offsets */
    env::offsets* _Poffsets;
    /**< pointer to wind */
    env::wind* _Pwind;
    /**< pointer to turbulence */
    env::turb* _Pturb;
    /**< pointer to geodetics */
    env::geo* _Pgeo;
public:
    /**< default constructor */
    earth() = delete;
    /**< constructor based on seeder object, identifiers for wind filed, atmospheric offsets, magnetic location, gravity and
     * magnetism realism, together with turbulence level factor, initial time (for turbulence), and final time (only two
     * values allowed as preloaded, 1000 or 5000) */
    earth(math::seeder& Oseeder,
          env::logic::WIND_ID wind_id, env::logic::OFFSETS_ID offsets_id, env::logic::MAG_ID mag_id, env::logic::REALISM_ID realism_id, const double& turb_factor,
          const double& t_sec_turb_init, const int& t_sec_turb_end);
    /**< copy constructor */
    earth(const earth&) = delete;
    /**< move constructor */
    earth(earth&&) = delete;
    /**< destructor */
    ~earth();
    /**< copy assignment */
    earth& operator=(const earth&) = delete;
    /**< move assignment */
    earth& operator=(earth&&) = delete;

    /**< get temperature and pressure offsets */
    const env::offsets& get_offsets() const {return *_Poffsets;}
    env::offsets& get_offsets() {return *_Poffsets;}
    /**< get wind */
    const env::wind& get_wind() const {return *_Pwind;}
    env::wind& get_wind() {return *_Pwind;}
    /**< get turbulence */
    const env::turb& get_turb() const {return *_Pturb;}
    env::turb& get_turb() {return *_Pturb;}
    /**< get geodetics */
    const env::geo& get_geo() const {return *_Pgeo;}
    env::geo& get_geo() {return *_Pgeo;}
}; // closes class earth

}; // closes namespace env

#endif























