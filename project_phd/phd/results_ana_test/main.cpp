
#include "Tplots_gnss.h"
#include "Tplots_pos_filter.h"
#include "Tplots_att_filter.h"
#include "Tplots_att_filter_alter.h"
#include "Tplots_sensors_bias.h"
#include "Tplots_sensors_gyr.h"
#include "Tplots_sensors_acc.h"
#include "Tplots_sensors_mag.h"
#include "Tplots_sensors_air.h"
#include "Tplots_sensors_vtasb.h"
#include "Tplots_init.h"
#include <iostream>

int main(int argc, char **argv) {

    unsigned short seed_init = 1;
    unsigned short seed_end  = 100;

    {nav::test::Tplots_gnss             	o(seed_init, seed_end); o.run(); }
    {nav::test::Tplots_pos_filter           o(seed_init, seed_end); o.run(); }
    {nav::test::Tplots_att_filter           o(seed_init, seed_end); o.run(); }
    {nav::test::Tplots_att_filter_alter     o(seed_init, seed_end); o.run(); }
    {nav::test::Tplots_sensors_bias         o(seed_init, seed_end); o.run(); }
    {nav::test::Tplots_sensors_gyr          o(seed_init, seed_end); o.run(); }
    {nav::test::Tplots_sensors_acc          o(seed_init, seed_end); o.run(); }
    {nav::test::Tplots_sensors_mag          o(seed_init, seed_end); o.run(); }
    {nav::test::Tplots_sensors_vtasb        o(seed_init, seed_end); o.run(); }
    {nav::test::Tplots_sensors_air          o(seed_init, seed_end); o.run(); }
    {nav::test::Tplots_init                 o(seed_init, seed_end); o.run(); }

	return 0;
}



