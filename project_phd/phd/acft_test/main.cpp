
#include "Tapm.h"
#include "Tapm_plots.h"
#include "Tbrick.h"
#include "Tsensor.h"
#include "Tsti.h"

int main(int argc, char **argv) {

    jail::counter Ocounter;

    {acft::test::Tapm		    o(Ocounter); o.run(); }
    {acft::test::Tapm_plots    	o(Ocounter); o.run(); }
    //{acft::test::Tsensor	   	o(Ocounter); o.run(); }
    {acft::test::Tbrick	   	    o(Ocounter); o.run(); }
    {acft::test::Tsti	   	    o(Ocounter); o.run(); }

	Ocounter.write_results();

	return 0;
}



