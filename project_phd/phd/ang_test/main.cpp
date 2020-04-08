
#include "Tso3.h"
#include "Tspecific.h"
#include "Tintegration.h"
#include "Tintegration.h"
#include "Tcheck.h"

#include <iostream>

int main(int argc, char **argv) {

    jail::counter Ocounter;

    {ang::test::Tso3        	o(Ocounter); o.run();}
    {ang::test::Tcheck           	o(Ocounter); o.run();}
    {ang::test::Tspecific      	o(Ocounter); o.run();}
    {ang::test::Tintegration   	o(Ocounter); o.run();}

	Ocounter.write_results();

	return 0;
}



