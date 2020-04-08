
#include "Tsimple.h"
#include "Ttrj_sizes.h"
#include "Talign_coarse.h"
#include <iostream>

int main(int argc, char **argv) {

    jail::counter Ocounter;

    {nav::test::Talign_coarse             	o(Ocounter); o.run(); }
    {nav::test::Tsimple              	o(Ocounter); o.run(); }
    {nav::test::Ttrj_sizes             	o(Ocounter); o.run(); }

	Ocounter.write_results();

	return 0;
}



