
#include "Tenvironment.h"
#include "Tmagnetic.h"
#include "Tgravity.h"
#include "Tturb.h"
#include "Tenv_plots_phd.h"
#include <iostream>

int main(int argc, char **argv) {

    jail::counter Ocounter;

    {env::test::Tenvironment        o(Ocounter); o.run(); }
    {env::test::Tmagnetic           o(Ocounter); o.run(); }
    {env::test::Tgravity            o(Ocounter); o.run(); }
    {env::test::Tturb               o(Ocounter); o.run(); }
    {env::test::Tenv_plots_phd      o(Ocounter); o.run(); }

    Ocounter.write_results();

	return 0;
}



