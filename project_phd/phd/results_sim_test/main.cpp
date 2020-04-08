
#include "Tplots_seeds.h"
#include "Tplots_control.h"
#include "Tplots_air.h"
#include "Tplots_attitude.h"
#include "Tplots_pos.h"
#include "Tplots_number.h"
#include <iostream>

int main(int argc, char **argv) {

    unsigned short seed_init = 1;
    unsigned short seed_end  = 100;

    {nav::test::Tplots_seeds            o(seed_init, seed_end); o.run(); }
    {nav::test::Tplots_control          o(seed_init, seed_end); o.run(); }
    {nav::test::Tplots_air             	o(seed_init, seed_end); o.run(); }
    {nav::test::Tplots_attitude         o(seed_init, seed_end); o.run(); }
    {nav::test::Tplots_pos              o(seed_init, seed_end); o.run(); }
    {nav::test::Tplots_number           o(seed_init, seed_end); o.run(); }

	return 0;
}



