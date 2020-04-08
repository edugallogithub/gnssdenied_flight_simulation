#include "pred.h"


// CLASS PRED
// ==========
// ==========

const std::string math::pred::describe_pred(const math::logic::PRED_NAME index) {
	switch(index) {
		case math::logic::f_null:				return("f_null");			break;
		case math::logic::f_constant:			return("f_constant");		break;
		case math::logic::f_lineal:			return("f_lineal");			break;
		case math::logic::f_parabolic:		return("f_parabolic");		break;
		case math::logic::f_cubic:		    return("f_cubic");		    break;
		case math::logic::f_steps:			return("f_steps");			break;
		case math::logic::f_table1V:			return("f_table1V");		break;
		case math::logic::f_cat:				return("f_cat");			break;
		case math::logic::f_cat_partial:		return("f_cat_partial");	break;
		case math::logic::f_table1V_num:		return("f_table1V_num");	break;
		case math::logic::f_table1V_spl:		return("f_table1V_spl");	break;
		case math::logic::f_lineal_double:	return("f_lineal_double");	break;
		case math::logic::f_table2V:			return("f_table2V");		break;
		case math::logic::f_lineal_triple:	return("f_lineal_triple");	break;
		case math::logic::f_table3V:			return("f_table3V");		break;
		case math::logic::f_tabular3V:		return("f_tabular3V");		break;
		case math::logic::f_table4V:			return("f_table4V");		break;
		default:
			throw std::runtime_error("Incorrect predicate name.");;
			return("");
			break;
	}
}
/* returns a string containing the name of the predicate in the
PRED_NAME enumeration */

const math::logic::PRED_NAME math::pred::reverse_describe_pred(const std::string& st) {
	if		(st == "f_null"          ) {return math::logic::f_null;}
	else if	(st == "f_constant"      ) {return math::logic::f_constant;}
	else if	(st == "f_lineal"        ) {return math::logic::f_lineal;}
	else if	(st == "f_parabolic"     ) {return math::logic::f_parabolic;}
	else if	(st == "f_cubic"         ) {return math::logic::f_cubic;}
	else if	(st == "f_steps"         ) {return math::logic::f_steps;}
	else if	(st == "f_table1V"       ) {return math::logic::f_table1V;}
	else if	(st == "f_table1Veq"     ) {return math::logic::f_table1V;}
	else if	(st == "f_cat"           ) {return math::logic::f_cat;}
	else if	(st == "f_cat_partial"   ) {return math::logic::f_cat_partial;}
	else if	(st == "f_table1V_num"   ) {return math::logic::f_table1V_num;}
	else if	(st == "f_table1V_spl"   ) {return math::logic::f_table1V_spl;}
	else if	(st == "f_lineal_double" ) {return math::logic::f_lineal_double;}
	else if	(st == "f_table2V"       ) {return math::logic::f_table2V;}
	else if	(st == "f_table2Veq"     ) {return math::logic::f_table2V;}
	else if	(st == "f_lineal_triple" ) {return math::logic::f_lineal_triple;}
	else if	(st == "f_table3V"       ) {return math::logic::f_table3V;}
	else if	(st == "f_table3Veq"     ) {return math::logic::f_table3V;}
	else if	(st == "f_tabular3V"     ) {return math::logic::f_tabular3V;}
	else if	(st == "f_table4V"       ) {return math::logic::f_table4V;}
	else if	(st == "f_table4Veq"     ) {return math::logic::f_table4V;}
	else {
		throw std::runtime_error("Incorrect predicated name.");
		return math::logic::pred_size;
	}
}
/* returns the name of the predicate in the PRED_NAME enumeration based on a
string describing it */






