// This is gui/gnd_cal_tableau_sptr.h
#ifndef gnd_cal_tableau_sptr_h_
#define gnd_cal_tableau_sptr_h_
//:
// \file
// \brief  Smart-pointer to a gnd_cal_tableau tableau.
//
//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
//

#include <vgui/vgui_tableau_sptr.h>

class dbgui_gnd_cal_tableau;

//: Smart-pointer to a dbgui_gnd_cal_tableau tableau.
typedef vgui_tableau_sptr_t<dbgui_gnd_cal_tableau> dbgui_gnd_cal_tableau_sptr;

#endif // gnd_cal_tableau_sptr_h_
