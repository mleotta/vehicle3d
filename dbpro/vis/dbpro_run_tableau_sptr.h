// This is dbpro/vis/dbpro_run_tableau_sptr.h
#ifndef dbpro_run_tableau_sptr_h_
#define dbpro_run_tableau_sptr_h_
//:
// \file
// \author  Matthew Leotta (mleotta@lems.brown.edu)
// \brief   Smart-pointer to a dbpro_run_tableau tableau.
//
//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
//

#include <vgui/vgui_tableau_sptr.h>

class dbpro_run_tableau;
typedef vgui_tableau_sptr_t<dbpro_run_tableau> dbpro_run_tableau_sptr;

#endif // dbpro_run_tableau_sptr_h_
