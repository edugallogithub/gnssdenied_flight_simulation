#include "seeder.h"
#include "share.h"

#include <fstream>
#include <iostream>
#include <boost/filesystem.hpp>

// SEEDER
// ======
// ======

math::seeder::seeder(const unsigned short& seed_order)
: _seed_order(seed_order), _seeds(math::seeder::seeder_id_size) {

    if ((seed_order > 100) || (seed_order < 1)) {throw std::runtime_error("Seed order too high, needs to be between 1 and 100.");}

    // read suite seed based on text file and input order
    boost::filesystem::path path_inputs(math::share::phd_inputs_prefix);
    boost::filesystem::path path_seeds("seeds");
    boost::filesystem::path path_file_seeds("seeds.txt");
    std::ifstream Oseeds;
    Oseeds.open((path_inputs / path_seeds / path_file_seeds).string());
    unsigned short i = 0;
    unsigned int seed;
    do {
        Oseeds >> seed;
        i++;
    } while (i <= (seed_order-1));
    Oseeds.close();
    //std::cout << seed << std::endl;

    _Ogen = std::ranlux24_base(seed);

    // generate seeds (always in same order for repeatability)
    _seeds[math::seeder::seeder_acc]      = _Odist(_Ogen);
    _seeds[math::seeder::seeder_gyr]      = _Odist(_Ogen);
    _seeds[math::seeder::seeder_mag]      = _Odist(_Ogen);
    _seeds[math::seeder::seeder_osp]      = _Odist(_Ogen);
    _seeds[math::seeder::seeder_tas]      = _Odist(_Ogen);
    _seeds[math::seeder::seeder_aoa]      = _Odist(_Ogen);
    _seeds[math::seeder::seeder_aos]      = _Odist(_Ogen);
    _seeds[math::seeder::seeder_oat]      = _Odist(_Ogen);
    _seeds[math::seeder::seeder_gps]      = _Odist(_Ogen);
    _seeds[math::seeder::seeder_initeul]  = _Odist(_Ogen);
    _seeds[math::seeder::seeder_acft_plat]= _Odist(_Ogen);
    _seeds[math::seeder::seeder_initmgn]  = _Odist(_Ogen);
    _seeds[math::seeder::seeder_geo]      = _Odist(_Ogen);
    _seeds[math::seeder::seeder_acft_acc] = _Odist(_Ogen);
    _seeds[math::seeder::seeder_acft_gyr] = _Odist(_Ogen);
    _seeds[math::seeder::seeder_acft_mag] = _Odist(_Ogen);
    _seeds[math::seeder::seeder_align]    = _Odist(_Ogen);
    _seeds[math::seeder::seeder_offsets]  = _Odist(_Ogen);
    _seeds[math::seeder::seeder_wind]     = _Odist(_Ogen);
    _seeds[math::seeder::seeder_guid]     = _Odist(_Ogen);
    _seeds[math::seeder::seeder_initacc]  = _Odist(_Ogen);
    _seeds[math::seeder::seeder_initgyr]  = _Odist(_Ogen);
    _seeds[math::seeder::seeder_initmag]  = _Odist(_Ogen);
}
/* constructor based on starting seed order (from 1 to 990) */

int math::seeder::provide_seed(math::seeder::SEEDER_ID seeder_id) {
    return _seeds[seeder_id];
}
/* generate pseudorandom seed, verifying that requested seed is in the right order */

void math::seeder::generate_seeds(const unsigned short& n) {
    boost::filesystem::path path_inputs(math::share::phd_inputs_prefix);
    boost::filesystem::path path_seeds("seeds");
    boost::filesystem::path path_file_seeds("seeds.txt");
    std::ofstream Oout_seeds;
    Oout_seeds.open((path_inputs / path_seeds / path_file_seeds).string());

    unsigned seed = 1;
    std::ranlux24_base Ogen(seed);
    std::uniform_int_distribution<> Odistr;
    for (unsigned short i=0; i < n; ++i) {
        Oout_seeds << Odistr(Ogen) << std::endl;
    }
    Oout_seeds.close();
}
/* generates input number of seeds (always the same ones) and stores them in text file. Intended to be run only once. */

std::string math::seeder::seed2string(unsigned short seed_order) {
    if ((seed_order > 100) || (seed_order < 1)) {throw std::runtime_error("Seed order too high, needs to be between 1 and 100.");}

    std::string st_seeds = std::to_string(seed_order);
    switch (st_seeds.size()) {
        case 1:
            st_seeds.insert(0, "0");
            return st_seeds;
        case 2:
            return st_seeds;
        case 3:
            return st_seeds.substr(1,2);
        default:
            throw std::runtime_error("Incorrect seeds choice.");
    }
}
/* returns a two digit string containing the seed order (07 for 7, 84 for 84, 00 for 100) */

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////



























