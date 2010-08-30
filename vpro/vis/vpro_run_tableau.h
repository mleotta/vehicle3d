// This is vpro/vis/vpro_run_tableau.h
#ifndef vpro_run_tableau_h_
#define vpro_run_tableau_h_
//:
// \file
// \brief  Tableau that runs a vpro process graph in its idle handler
// \author Matthew Leotta (mleotta@brown.lems.edu)
// \date   July 14, 2006
//
//          Copyright Matthew Leotta 2006 - 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file ../../LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)
//
// \verbatim
//  Modifications
// \endverbatim


#include <vpro/vis/vpro_run_tableau_sptr.h>
#include <vpro/vpro_executive.h>
#include <vgui/vgui_tableau.h>
#include <vul/vul_timer.h>


//: Tableau that runs a vpro process graph in its idle handler
class vpro_run_tableau: public vgui_tableau
{
  public:
    //: Constructor
    vpro_run_tableau(const vpro_executive& g);

    //: Enable idle events (start the processing)
    void enable_idle();

    //: Enable idle events for one iteration
    void enable_idle_once();

    //: Run one step of the processing
    bool run_step();

    //: Toggle the text display
    void toggle_display();

    //: Builds a popup menu
    void get_popup(const vgui_popup_params& params,
                   vgui_menu &menu);

    //: Handle events
    bool handle(const vgui_event& e);

    //: Handle keypress
    bool key_press (int x, int y, vgui_key, vgui_modifier);

    //: Handle idle events
    bool idle();

  private:
    vpro_executive graph_;
    vul_timer time_;
    double fps_;
    unsigned int count_;
    bool display_;
    bool running_;
};


//: Creates a smart-pointer to a vpro_run_tableau tableau.
struct vpro_run_tableau_new : public vpro_run_tableau_sptr
{
  typedef vpro_run_tableau_sptr base;

  //: Constructor
  vpro_run_tableau_new(const vpro_executive& g)
    : base(new vpro_run_tableau(g)) {}
};

#endif // vpro_run_tableau_h_
